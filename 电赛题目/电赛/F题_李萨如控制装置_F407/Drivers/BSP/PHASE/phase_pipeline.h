#ifndef PHASE_PIPELINE_H
#define PHASE_PIPELINE_H

#include "phase_fft.h"
#include "phase_measure.h"

typedef struct
{
    PhaseFrequencyConfig_t frequency;
    PhaseFftConfig_t fft;
} PhasePipelineConfig_t;

typedef struct
{
    PhaseFrequencyResult_t frequency;
    PhaseFftResult_t phase;
    uint8_t frequency_valid;
    uint8_t phase_valid;
} PhasePipelineResult_t;

typedef struct
{
    PhasePipelineConfig_t config;
    PhasePipelineResult_t result;
    uint8_t initialized;
} PhasePipeline_t;

void PhasePipelineConfig_Default(PhasePipelineConfig_t *config);
PhaseMeasureStatus_t PhasePipeline_Init(PhasePipeline_t *pipeline,
                                        const PhasePipelineConfig_t *config);
PhaseMeasureStatus_t PhasePipeline_ProcessPeriod(PhasePipeline_t *pipeline,
                                                 uint32_t period_ticks,
                                                 uint32_t divider);
PhaseMeasureStatus_t PhasePipeline_ProcessCapturedPeriod(
    PhasePipeline_t *pipeline,
    uint32_t period_ticks,
    uint32_t capture_cycles,
    uint32_t divider);
void PhasePipeline_SetSignalFrequency(PhasePipeline_t *pipeline,
                                      uint32_t signal_frequency_hz);
void PhasePipeline_SetSampleRate(PhasePipeline_t *pipeline,
                                 uint32_t sample_rate_hz);
uint8_t PhasePipeline_ProcessAdcBlock(PhasePipeline_t *pipeline,
                                      const int16_t *channel1,
                                      const int16_t *channel2);
const PhasePipelineResult_t *PhasePipeline_GetResult(
    const PhasePipeline_t *pipeline);

/* Default object/wrappers for the single-instrument firmware. */
extern PhasePipeline_t g_phase_pipeline;
void phase_pipeline_init(void);

#endif /* PHASE_PIPELINE_H */
