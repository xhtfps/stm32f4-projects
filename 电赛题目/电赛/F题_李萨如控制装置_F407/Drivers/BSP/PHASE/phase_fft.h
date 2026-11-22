#ifndef PHASE_FFT_H
#define PHASE_FFT_H

#include <stdint.h>

#define PHASE_FFT_MAX_POINTS 1024U

typedef struct
{
    uint32_t sample_rate_hz;
    float signal_frequency_hz;
    uint16_t point_count;
} PhaseFftConfig_t;

typedef struct
{
    uint16_t selected_bin;          /* 通道1对应的近似FFT bin */
    uint16_t channel2_selected_bin; /* 通道2对应的近似FFT bin */
    float channel1_amplitude;
    float channel2_amplitude;
    float channel1_phase_deg;
    float channel2_phase_deg;
    float phase_difference_deg;
    float channel1_crossing_phase_deg;
    float channel2_crossing_phase_deg;
    uint8_t crossing_valid;
    uint8_t valid;
} PhaseFftResult_t;

void PhaseFftConfig_Default(PhaseFftConfig_t *config);

/*
 * Analyze two signed, bias-removed ADC blocks with a Hann-windowed,
 * single-frequency DFT at the configured physical frequency.  Coefficients are
 * cached, so steady-state processing performs no per-sample trigonometry.
 * The function is non-reentrant and belongs in the instrument task.
 */
uint8_t PhaseFft_Analyze(const int16_t *channel1,
                         const int16_t *channel2,
                         const PhaseFftConfig_t *config,
                         PhaseFftResult_t *result);

/*
 * 双频单点DFT。两路必须来自同一次同步ADC采样，采样率和点数相同，
 * 但允许通道1在fin、通道2在2fin处分别提取相位和幅值。
 */
uint8_t PhaseFft_AnalyzeDual(const int16_t *channel1,
                             const int16_t *channel2,
                             const PhaseFftConfig_t *channel1_config,
                             const PhaseFftConfig_t *channel2_config,
                             PhaseFftResult_t *result);

#endif /* PHASE_FFT_H */
