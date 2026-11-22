#include "phase_pipeline.h"

#include <stddef.h>
#include <string.h>

PhasePipeline_t g_phase_pipeline;

void PhasePipelineConfig_Default(PhasePipelineConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    PhaseFrequencyConfig_Default(&config->frequency);
    PhaseFftConfig_Default(&config->fft);
}

PhaseMeasureStatus_t PhasePipeline_Init(PhasePipeline_t *pipeline,
                                        const PhasePipelineConfig_t *config)
{
    if ((pipeline == NULL) || (config == NULL))
    {
        return PHASE_MEASURE_INVALID_ARGUMENT;
    }

    (void)memset(pipeline, 0, sizeof(*pipeline));
    pipeline->config = *config;
    pipeline->initialized = 1U;
    return PHASE_MEASURE_OK;
}

PhaseMeasureStatus_t PhasePipeline_ProcessPeriod(PhasePipeline_t *pipeline,
                                                 uint32_t period_ticks,
                                                 uint32_t divider)
{
    if ((pipeline == NULL) || (pipeline->initialized == 0U))
    {
        return PHASE_MEASURE_NOT_READY;
    }

    return PhasePipeline_ProcessCapturedPeriod(
        pipeline, period_ticks,
        pipeline->config.frequency.capture_cycles, divider);
}

PhaseMeasureStatus_t PhasePipeline_ProcessCapturedPeriod(
    PhasePipeline_t *pipeline,
    uint32_t period_ticks,
    uint32_t capture_cycles,
    uint32_t divider)
{
    PhaseMeasureStatus_t status;
    PhaseFrequencyConfig_t frequency_config;

    if ((pipeline == NULL) || (pipeline->initialized == 0U) ||
        (capture_cycles == 0U))
    {
        return PHASE_MEASURE_NOT_READY;
    }

    frequency_config = pipeline->config.frequency;
    frequency_config.capture_cycles = capture_cycles;
    status = PhaseFrequency_FromPeriod(&frequency_config,
                                       period_ticks, divider,
                                       &pipeline->result.frequency);
    pipeline->result.frequency_valid =
        (status == PHASE_MEASURE_OK) ? 1U : 0U;
    return status;
}

void PhasePipeline_SetSignalFrequency(PhasePipeline_t *pipeline,
                                      uint32_t signal_frequency_hz)
{
    if ((pipeline != NULL) && (pipeline->initialized != 0U))
    {
        pipeline->config.fft.signal_frequency_hz = signal_frequency_hz;
    }
}

void PhasePipeline_SetSampleRate(PhasePipeline_t *pipeline,
                                 uint32_t sample_rate_hz)
{
    if ((pipeline != NULL) && (pipeline->initialized != 0U) &&
        (sample_rate_hz != 0U))
    {
        pipeline->config.fft.sample_rate_hz = sample_rate_hz;
    }
}

uint8_t PhasePipeline_ProcessAdcBlock(PhasePipeline_t *pipeline,
                                      const int16_t *channel1,
                                      const int16_t *channel2)
{
    if ((pipeline == NULL) || (pipeline->initialized == 0U))
    {
        return 0U;
    }

    pipeline->result.phase_valid = PhaseFft_Analyze(
        channel1, channel2, &pipeline->config.fft,
        &pipeline->result.phase);
    return pipeline->result.phase_valid;
}

const PhasePipelineResult_t *PhasePipeline_GetResult(
    const PhasePipeline_t *pipeline)
{
    if ((pipeline == NULL) || (pipeline->initialized == 0U))
    {
        return NULL;
    }
    return &pipeline->result;
}

void phase_pipeline_init(void)
{
    PhasePipelineConfig_t config;

    PhasePipelineConfig_Default(&config);
    (void)PhasePipeline_Init(&g_phase_pipeline, &config);
}
