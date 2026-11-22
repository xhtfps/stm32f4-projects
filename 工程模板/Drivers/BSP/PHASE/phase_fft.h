#ifndef PHASE_FFT_H
#define PHASE_FFT_H

#include <stdint.h>

#define PHASE_FFT_MAX_POINTS 256U

typedef struct
{
    uint32_t sample_rate_hz;
    uint32_t signal_frequency_hz;
    uint16_t point_count;
} PhaseFftConfig_t;

typedef struct
{
    uint16_t selected_bin;
    float channel1_amplitude;
    float channel2_amplitude;
    float channel1_phase_deg;
    float channel2_phase_deg;
    float phase_difference_deg;
    uint8_t valid;
} PhaseFftResult_t;

void PhaseFftConfig_Default(PhaseFftConfig_t *config);

/*
 * Analyze two signed, bias-removed ADC blocks. The function is intentionally
 * non-reentrant: it owns one work buffer, so it belongs in the instrument task.
 */
uint8_t PhaseFft_Analyze(const int16_t *channel1,
                         const int16_t *channel2,
                         const PhaseFftConfig_t *config,
                         PhaseFftResult_t *result);

#endif /* PHASE_FFT_H */
