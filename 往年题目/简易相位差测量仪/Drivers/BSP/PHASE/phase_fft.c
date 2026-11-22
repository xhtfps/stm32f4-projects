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
static uint32_t s_phase_dft_signal_frequency_hz;
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
    return atan2f(value.imag, value.real) * 180.0f / PHASE_FFT_PI;
}

static uint16_t PhaseFft_SelectBin(const PhaseFftConfig_t *config)
{
    uint32_t bin;
    uint32_t half_length;

    half_length = config->point_count / 2U;
    if (config->signal_frequency_hz == 0U)
    {
        return 0U;
    }

    bin = (((uint64_t)config->signal_frequency_hz *
            (uint64_t)config->point_count) +
           ((uint64_t)config->sample_rate_hz / 2U)) /
          (uint64_t)config->sample_rate_hz;
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
        (s_phase_dft_signal_frequency_hz ==
         config->signal_frequency_hz) &&
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
    config->signal_frequency_hz = 10000U;
    config->point_count = 256U;
}

uint8_t PhaseFft_Analyze(const int16_t *channel1,
                         const int16_t *channel2,
                         const PhaseFftConfig_t *config,
                         PhaseFftResult_t *result)
{
    PhaseFftPhasor_t channel1_phasor;
    PhaseFftPhasor_t channel2_phasor;
    uint16_t bin;

    if ((channel1 == NULL) || (channel2 == NULL) ||
        (config == NULL) || (result == NULL) ||
        (config->sample_rate_hz == 0U) ||
        (config->point_count < 8U) ||
        (config->point_count > PHASE_FFT_MAX_POINTS) ||
        (config->point_count == 1U))
    {
        return 0U;
    }

    result->selected_bin = 0U;
    result->channel1_amplitude = 0.0f;
    result->channel2_amplitude = 0.0f;
    result->channel1_phase_deg = 0.0f;
    result->channel2_phase_deg = 0.0f;
    result->phase_difference_deg = 0.0f;
    result->valid = 0U;

    if (((uint64_t)config->signal_frequency_hz * 2ULL) >=
        (uint64_t)config->sample_rate_hz)
    {
        return 0U;
    }

    bin = PhaseFft_SelectBin(config);
    if (bin == 0U)
    {
        return 0U;
    }
    if (PhaseFft_PrepareDft(config) == 0U)
    {
        return 0U;
    }

    channel1_phasor = PhaseFft_RunChannel(
        channel1, config, &result->channel1_amplitude);
    channel2_phasor = PhaseFft_RunChannel(
        channel2, config, &result->channel2_amplitude);

    result->selected_bin = bin;
    result->channel1_phase_deg = PhaseFft_PhaseDegrees(channel1_phasor);
    result->channel2_phase_deg = PhaseFft_PhaseDegrees(channel2_phasor);
    result->phase_difference_deg = PhaseFft_NormalizeDegrees(
        result->channel2_phase_deg - result->channel1_phase_deg);
    result->valid = 1U;
    return 1U;
}
