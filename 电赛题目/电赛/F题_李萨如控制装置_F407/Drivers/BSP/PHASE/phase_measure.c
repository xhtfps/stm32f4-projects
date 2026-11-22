#include "phase_measure.h"

#include <stddef.h>

void PhaseFrequencyConfig_Default(PhaseFrequencyConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->timer_tick_hz = PHASE_MEASURE_DEFAULT_TIMER_HZ;
    config->capture_cycles = PHASE_MEASURE_CAPTURE_CYCLES;
    config->input_min_hz = PHASE_MEASURE_MIN_HZ;
    config->input_max_hz = PHASE_MEASURE_MAX_HZ;
    config->divider_low = PHASE_MEASURE_DIVIDER_LOW;
    config->divider_high = PHASE_MEASURE_DIVIDER_HIGH;
    config->divider_threshold_hz = PHASE_MEASURE_DIVIDER_THRESHOLD;
    config->divider_hysteresis_hz = PHASE_MEASURE_DIVIDER_HYSTERESIS;
}

uint32_t PhaseFrequency_SelectDivider(const PhaseFrequencyConfig_t *config,
                                      uint32_t estimated_input_hz,
                                      uint32_t current_divider)
{
    uint32_t lower_threshold;
    uint32_t upper_threshold;

    if (config == NULL)
    {
        return PHASE_MEASURE_DIVIDER_LOW;
    }

    lower_threshold =
        config->divider_threshold_hz -
        ((config->divider_threshold_hz >
          config->divider_hysteresis_hz) ?
         config->divider_hysteresis_hz : 0U);
    upper_threshold =
        config->divider_threshold_hz +
        config->divider_hysteresis_hz;

    if (current_divider == config->divider_high)
    {
        /* Exactly 4.9 MHz remains inside the hold band. */
        return (estimated_input_hz < lower_threshold) ?
               config->divider_low : config->divider_high;
    }
    if (current_divider == config->divider_low)
    {
        /* Exactly 5.1 MHz remains inside the hold band. */
        return (estimated_input_hz > upper_threshold) ?
               config->divider_high : config->divider_low;
    }

    return (estimated_input_hz > config->divider_threshold_hz) ?
           config->divider_high : config->divider_low;
}

uint32_t PhaseFrequency_TimestampDelta(uint32_t previous,
                                       uint32_t current)
{
    if (current >= previous)
    {
        return current - previous;
    }

    return (uint32_t)(0xFFFFFFFFUL - previous + 1UL + current);
}

PhaseMeasureStatus_t PhaseFrequency_FromPeriod(
    const PhaseFrequencyConfig_t *config,
    uint32_t period_ticks,
    uint32_t divider,
    PhaseFrequencyResult_t *result)
{
    uint64_t effective_tick_hz;
    double divided_frequency;
    double input_frequency;
    double minimum_accepted_hz;
    double maximum_accepted_hz;

    if ((config == NULL) || (result == NULL) ||
        (config->timer_tick_hz == 0U) ||
        (config->capture_cycles == 0U) || (period_ticks == 0U) ||
        (divider == 0U))
    {
        return PHASE_MEASURE_INVALID_ARGUMENT;
    }

    result->period_ticks = period_ticks;
    result->divided_frequency_hz = 0U;
    result->input_frequency_hz = 0U;
    result->divided_frequency_hz_f = 0.0f;
    result->input_frequency_hz_f = 0.0f;
    result->divider = divider;
    result->valid = 0U;

    effective_tick_hz = (uint64_t)config->timer_tick_hz *
                        (uint64_t)config->capture_cycles;
    divided_frequency = (double)effective_tick_hz / (double)period_ticks;
    input_frequency = divided_frequency * (double)divider;

    /*
     * A nominal 10 Hz source may calculate as 9.99x Hz because the generator
     * and MCU clock are not mathematically exact. Do not reject a valid edge
     * measurement solely because it lies just outside the nominal endpoint.
     */
    minimum_accepted_hz =
        (double)config->input_min_hz *
        (double)(100U - PHASE_MEASURE_RANGE_GUARD_PERCENT) / 100.0;
    maximum_accepted_hz =
        (double)config->input_max_hz *
        (double)(100U + PHASE_MEASURE_RANGE_GUARD_PERCENT) / 100.0;

    if (input_frequency > 4294967295.0)
    {
        return PHASE_MEASURE_OUT_OF_RANGE;
    }

    result->divided_frequency_hz =
        (uint32_t)(divided_frequency + 0.5);
    result->input_frequency_hz = (uint32_t)(input_frequency + 0.5);
    result->divided_frequency_hz_f = (float)divided_frequency;
    result->input_frequency_hz_f = (float)input_frequency;

    if ((input_frequency < minimum_accepted_hz) ||
        (input_frequency > maximum_accepted_hz))
    {
        return PHASE_MEASURE_OUT_OF_RANGE;
    }

    result->valid = 1U;
    return PHASE_MEASURE_OK;
}
