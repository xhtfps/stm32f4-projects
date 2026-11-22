#include "phase_fft.h"

#include "../FFT/FFT.h"

#include <math.h>
#include <stddef.h>

#define PHASE_FFT_PI 3.14159265358979323846f

static complex s_phase_fft_buffer[PHASE_FFT_MAX_POINTS];

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

static float PhaseFft_PhaseDegrees(complex value)
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

static complex PhaseFft_RunChannel(const int16_t *samples,
                                   uint16_t point_count,
                                   uint16_t bin,
                                   float *amplitude)
{
    uint16_t index;
    float mean = 0.0f;
    float scale;
    complex result = {0.0f, 0.0f};

    for (index = 0U; index < point_count; ++index)
    {
        mean += (float)samples[index];
    }
    mean /= (float)point_count;

    for (index = 0U; index < point_count; ++index)
    {
        float window = 0.5f *
                       (1.0f - cosf((2.0f * PHASE_FFT_PI * (float)index) /
                                    (float)(point_count - 1U)));
        s_phase_fft_buffer[index].real = ((float)samples[index] - mean) * window;
        s_phase_fft_buffer[index].imag = 0.0f;
    }

    fft(point_count, s_phase_fft_buffer);
    result = s_phase_fft_buffer[bin];
    scale = 2.0f / (float)point_count;
    *amplitude = complex_abs_float(result) * scale;
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
    complex channel1_phasor;
    complex channel2_phasor;
    uint16_t bin;

    if ((channel1 == NULL) || (channel2 == NULL) ||
        (config == NULL) || (result == NULL) ||
        (config->sample_rate_hz == 0U) ||
        (config->point_count < 8U) ||
        (config->point_count > PHASE_FFT_MAX_POINTS) ||
        (FFT_IsValidLength(config->point_count) == 0U) ||
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

    channel1_phasor = PhaseFft_RunChannel(channel1, config->point_count,
                                          bin, &result->channel1_amplitude);
    channel2_phasor = PhaseFft_RunChannel(channel2, config->point_count,
                                          bin, &result->channel2_amplitude);

    result->selected_bin = bin;
    result->channel1_phase_deg = PhaseFft_PhaseDegrees(channel1_phasor);
    result->channel2_phase_deg = PhaseFft_PhaseDegrees(channel2_phasor);
    result->phase_difference_deg = PhaseFft_NormalizeDegrees(
        result->channel2_phase_deg - result->channel1_phase_deg);
    result->valid = 1U;
    return 1U;
}
