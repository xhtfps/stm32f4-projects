#ifndef PHASE_VOFA_H
#define PHASE_VOFA_H

#include <stdint.h>

#define PHASE_VOFA_POINT_COUNT       64U
#define PHASE_VOFA_NORMALIZED_PEAK 1000

/*
 * FireWater payload: exactly two numeric columns per sample,
 * ADC1 (PA1/X) and ADC2 (PA2/Y). Each coherent frame is DC-corrected and
 * scaled with one shared gain so an XY plot preserves the amplitude ratio.
 * Measurement metadata stays out of the waveform stream.
 */

typedef struct
{
    uint8_t menu;
    uint8_t measure_state;
    uint8_t capture_route;
    uint8_t adc_mode;
    uint8_t frequency_valid;
    uint8_t phase_valid;
    float frequency_hz;
    float capture_frequency_hz;
    float channel1_amplitude;
    float channel2_amplitude;
    float phase_deg;
    float gain_db;
    uint32_t local_oscillator_hz;
    uint32_t adc_sample_rate_hz;
    uint32_t adc_overrun_count;
    uint8_t calibration_valid;
    uint8_t learning_active;
    uint8_t learning_point_index;
    uint8_t learning_point_total;
} PhaseVofaSnapshot_t;

/* Creates the low-priority FireWater waveform sender task. */
uint8_t PhaseVofa_CreateTask(void);
void PhaseVofa_SetOutputEnabled(uint8_t enabled);

/* Copies and normalizes one coherent ADC block; this call never blocks. */
void PhaseVofa_SubmitBlock(const int16_t *channel1,
                           const int16_t *channel2,
                           uint16_t length);

/* Updates the latest timer-capture result used by the FireWater stream. */
void PhaseVofa_SubmitTimerCapture(float frequency_hz,
                                  uint32_t period_ticks,
                                  uint8_t valid);
void PhaseVofa_SubmitSnapshot(const PhaseVofaSnapshot_t *snapshot);

uint32_t PhaseVofa_GetFrameCount(void);
uint32_t PhaseVofa_GetOverwriteCount(void);

#endif /* PHASE_VOFA_H */
