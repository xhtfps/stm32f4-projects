#include "phase_fft.h"

#include "./MALLOC/malloc.h"

#include <math.h>
#include <stddef.h>

#define PHASE_FFT_PI 3.14159265358979323846f

typedef struct
{
    float real;
    float imag;
} PhaseFftPhasor_t;

/* CPU-only DFT coefficients live in CCM; neither buffer is a DMA target. */
static float *s_phase_dft_real;
static float *s_phase_dft_imag;
static uint32_t s_phase_dft_sample_rate_hz;
static float s_phase_dft_signal_frequency_hz;
static uint16_t s_phase_dft_point_count;
static float s_phase_dft_window_sum;

static float PhaseFft_NormalizeDegrees(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }
    while (degrees < -180.0f)
    {
        degrees += 360.0f;
    }
    return degrees;
}

static float PhaseFft_PhaseDegrees(PhaseFftPhasor_t value)
{
    /*
     * exp(-jωt) DFT对 sin(ωt+φ) 得到 φ-90°。这里加回90°，
     * 使公开结果直接表示正弦模型中的φ。否则同频相减虽不受影响，
     * 但要求3的 φout-2φin 会固定多出90°。
     */
    return PhaseFft_NormalizeDegrees(
        atan2f(value.imag, value.real) * 180.0f /
        PHASE_FFT_PI + 90.0f);
}

/* Independent time-domain phase from interpolated rising zero crossings. */
static uint8_t PhaseFft_CrossingPhase(
    const int16_t *samples,
    const PhaseFftConfig_t *config,
    float *phase_deg)
{
    float mean = 0.0f;
    float peak = 0.0f;
    float reference_phase = 0.0f;
    float phase_sum = 0.0f;
    float first_crossing_index = 0.0f;
    float samples_per_cycle;
    float previous;
    float slope_threshold;
    uint16_t index;
    uint16_t crossing_count = 0U;

    for (index = 0U; index < config->point_count; ++index)
    {
        mean += (float)samples[index];
    }
    mean /= (float)config->point_count;
    for (index = 0U; index < config->point_count; ++index)
    {
        float centered = (float)samples[index] - mean;
        float magnitude = fabsf(centered);
        if (magnitude > peak)
        {
            peak = magnitude;
        }
    }
    if (peak < 16.0f)
    {
        return 0U;
    }

    slope_threshold = peak * 0.05f;
    samples_per_cycle = (float)config->sample_rate_hz /
                        config->signal_frequency_hz;
    previous = (float)samples[0] - mean;
    for (index = 1U; index < config->point_count; ++index)
    {
        float current = (float)samples[index] - mean;
        float slope = current - previous;

        if ((previous <= 0.0f) && (current > 0.0f) &&
            (slope >= slope_threshold))
        {
            float fraction = -previous / slope;
            float crossing_index = (float)(index - 1U) + fraction;
            uint32_t cycle_number = 0U;
            float equivalent_index;
            float crossing_phase;

            if (crossing_count == 0U)
            {
                first_crossing_index = crossing_index;
            }
            else
            {
                cycle_number = (uint32_t)(
                    (crossing_index - first_crossing_index) /
                    samples_per_cycle + 0.5f);
            }
            equivalent_index = crossing_index -
                               (float)cycle_number * samples_per_cycle;
            crossing_phase = PhaseFft_NormalizeDegrees(
                -360.0f * config->signal_frequency_hz * equivalent_index /
                (float)config->sample_rate_hz);
            if (crossing_count == 0U)
            {
                reference_phase = crossing_phase;
                phase_sum = crossing_phase;
            }
            else
            {
                phase_sum += reference_phase + PhaseFft_NormalizeDegrees(
                    crossing_phase - reference_phase);
            }
            ++crossing_count;
        }
        previous = current;
    }

    if (crossing_count < 2U)
    {
        return 0U;
    }
    *phase_deg = PhaseFft_NormalizeDegrees(
        phase_sum / (float)crossing_count);
    return 1U;
}

static uint16_t PhaseFft_SelectBin(const PhaseFftConfig_t *config)
{
    float bin_float;
    uint32_t bin;
    uint32_t half_length;

    half_length = config->point_count / 2U;
    if (config->signal_frequency_hz == 0U)
    {
        return 0U;
    }

    bin_float = config->signal_frequency_hz *
                (float)config->point_count /
                (float)config->sample_rate_hz;
    bin = (uint32_t)(bin_float + 0.5f);
    if (bin == 0U)
    {
        bin = 1U;
    }
    if (bin >= half_length)
    {
        bin = half_length - 1U;
    }
    return (uint16_t)bin;
}

static uint8_t PhaseFft_PrepareDft(const PhaseFftConfig_t *config)
{
    uint16_t index;
    float carrier_cos = 1.0f;
    float carrier_sin = 0.0f;
    float carrier_step_cos;
    float carrier_step_sin;
    float window_cos = 1.0f;
    float window_sin = 0.0f;
    float window_step_cos;
    float window_step_sin;
    float carrier_angle;
    float window_angle;

    if (s_phase_dft_real == NULL)
    {
        s_phase_dft_real = (float *)mymalloc(
            SRAMCCM, PHASE_FFT_MAX_POINTS * sizeof(float));
    }
    if (s_phase_dft_imag == NULL)
    {
        s_phase_dft_imag = (float *)mymalloc(
            SRAMCCM, PHASE_FFT_MAX_POINTS * sizeof(float));
    }
    if ((s_phase_dft_real == NULL) || (s_phase_dft_imag == NULL))
    {
        return 0U;
    }

    if ((s_phase_dft_sample_rate_hz == config->sample_rate_hz) &&
        (fabsf(s_phase_dft_signal_frequency_hz -
               config->signal_frequency_hz) < 0.01f) &&
        (s_phase_dft_point_count == config->point_count) &&
        (s_phase_dft_window_sum > 0.0f))
    {
        return 1U;
    }

    carrier_angle =
        2.0f * PHASE_FFT_PI * (float)config->signal_frequency_hz /
        (float)config->sample_rate_hz;
    window_angle =
        2.0f * PHASE_FFT_PI / (float)(config->point_count - 1U);
    carrier_step_cos = cosf(carrier_angle);
    carrier_step_sin = sinf(carrier_angle);
    window_step_cos = cosf(window_angle);
    window_step_sin = sinf(window_angle);
    s_phase_dft_window_sum = 0.0f;

    for (index = 0U; index < config->point_count; ++index)
    {
        float next_cos;
        float window = 0.5f * (1.0f - window_cos);

        s_phase_dft_real[index] = window * carrier_cos;
        s_phase_dft_imag[index] = -window * carrier_sin;
        s_phase_dft_window_sum += window;

        next_cos = carrier_cos * carrier_step_cos -
                   carrier_sin * carrier_step_sin;
        carrier_sin = carrier_sin * carrier_step_cos +
                      carrier_cos * carrier_step_sin;
        carrier_cos = next_cos;

        next_cos = window_cos * window_step_cos -
                   window_sin * window_step_sin;
        window_sin = window_sin * window_step_cos +
                     window_cos * window_step_sin;
        window_cos = next_cos;
    }

    s_phase_dft_sample_rate_hz = config->sample_rate_hz;
    s_phase_dft_signal_frequency_hz = config->signal_frequency_hz;
    s_phase_dft_point_count = config->point_count;
    return (s_phase_dft_window_sum > 0.0f) ? 1U : 0U;
}

static PhaseFftPhasor_t PhaseFft_RunChannel(
    const int16_t *samples, const PhaseFftConfig_t *config,
    float *amplitude)
{
    uint16_t index;
    float mean = 0.0f;
    float scale;
    PhaseFftPhasor_t result = {0.0f, 0.0f};

    for (index = 0U; index < config->point_count; ++index)
    {
        mean += (float)samples[index];
    }
    mean /= (float)config->point_count;

    for (index = 0U; index < config->point_count; ++index)
    {
        float centered = (float)samples[index] - mean;

        result.real += centered * s_phase_dft_real[index];
        result.imag += centered * s_phase_dft_imag[index];
    }

    scale = 2.0f / s_phase_dft_window_sum;
    *amplitude = sqrtf(result.real * result.real +
                       result.imag * result.imag) * scale;
    return result;
}

void PhaseFftConfig_Default(PhaseFftConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->sample_rate_hz = 256000U;
    config->signal_frequency_hz = 10000.0f;
    config->point_count = 1024U;
}

uint8_t PhaseFft_Analyze(const int16_t *channel1,
                         const int16_t *channel2,
                         const PhaseFftConfig_t *config,
                         PhaseFftResult_t *result)
{
    return PhaseFft_AnalyzeDual(channel1, channel2, config, config,
                                result);
}

uint8_t PhaseFft_AnalyzeDual(const int16_t *channel1,
                             const int16_t *channel2,
                             const PhaseFftConfig_t *channel1_config,
                             const PhaseFftConfig_t *channel2_config,
                             PhaseFftResult_t *result)
{
    PhaseFftPhasor_t channel1_phasor;
    PhaseFftPhasor_t channel2_phasor;
    uint16_t channel1_bin;
    uint16_t channel2_bin;

    if ((channel1 == NULL) || (channel2 == NULL) ||
        (channel1_config == NULL) || (channel2_config == NULL) ||
        (result == NULL) ||
        (channel1_config->sample_rate_hz == 0U) ||
        (channel1_config->sample_rate_hz !=
         channel2_config->sample_rate_hz) ||
        (channel1_config->point_count !=
         channel2_config->point_count) ||
        (channel1_config->point_count < 8U) ||
        (channel1_config->point_count > PHASE_FFT_MAX_POINTS))
    {
        return 0U;
    }

    result->selected_bin = 0U;
    result->channel2_selected_bin = 0U;
    result->channel1_amplitude = 0.0f;
    result->channel2_amplitude = 0.0f;
    result->channel1_phase_deg = 0.0f;
    result->channel2_phase_deg = 0.0f;
    result->phase_difference_deg = 0.0f;
    result->channel1_crossing_phase_deg = 0.0f;
    result->channel2_crossing_phase_deg = 0.0f;
    result->crossing_valid = 0U;
    result->valid = 0U;

    if (((channel1_config->signal_frequency_hz * 2.0f) >=
         (float)channel1_config->sample_rate_hz) ||
        ((channel2_config->signal_frequency_hz * 2.0f) >=
         (float)channel2_config->sample_rate_hz))
    {
        return 0U;
    }

    channel1_bin = PhaseFft_SelectBin(channel1_config);
    channel2_bin = PhaseFft_SelectBin(channel2_config);
    if ((channel1_bin == 0U) || (channel2_bin == 0U))
    {
        return 0U;
    }
    if (PhaseFft_PrepareDft(channel1_config) == 0U)
    {
        return 0U;
    }

    channel1_phasor = PhaseFft_RunChannel(
        channel1, channel1_config, &result->channel1_amplitude);
    if (PhaseFft_PrepareDft(channel2_config) == 0U)
    {
        return 0U;
    }
    channel2_phasor = PhaseFft_RunChannel(
        channel2, channel2_config, &result->channel2_amplitude);

    result->selected_bin = channel1_bin;
    result->channel2_selected_bin = channel2_bin;
    result->channel1_phase_deg = PhaseFft_PhaseDegrees(channel1_phasor);
    result->channel2_phase_deg = PhaseFft_PhaseDegrees(channel2_phasor);
    result->phase_difference_deg = PhaseFft_NormalizeDegrees(
        result->channel2_phase_deg - result->channel1_phase_deg);
    if ((PhaseFft_CrossingPhase(
             channel1, channel1_config,
             &result->channel1_crossing_phase_deg) != 0U) &&
        (PhaseFft_CrossingPhase(
             channel2, channel2_config,
             &result->channel2_crossing_phase_deg) != 0U))
    {
        result->crossing_valid = 1U;
    }
    result->valid = 1U;
    return 1U;
}
