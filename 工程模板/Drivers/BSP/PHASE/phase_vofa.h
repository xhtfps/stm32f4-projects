#ifndef PHASE_VOFA_H
#define PHASE_VOFA_H

#include <stdint.h>

#define PHASE_VOFA_POINT_COUNT 32U

/* Creates the low-priority FireWater waveform sender task. */
uint8_t PhaseVofa_CreateTask(void);

/* Copies and downsamples one ADC block; this call never blocks. */
void PhaseVofa_SubmitBlock(const int16_t *channel1,
                           const int16_t *channel2,
                           uint16_t length);

/* Updates the latest timer-capture result used by the FireWater stream. */
void PhaseVofa_SubmitTimerCapture(float frequency_hz,
                                  uint32_t period_ticks,
                                  uint8_t valid);

uint32_t PhaseVofa_GetFrameCount(void);
uint32_t PhaseVofa_GetOverwriteCount(void);

#endif /* PHASE_VOFA_H */
