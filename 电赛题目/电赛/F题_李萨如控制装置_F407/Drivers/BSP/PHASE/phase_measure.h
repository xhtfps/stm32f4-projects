#ifndef PHASE_MEASURE_H
#define PHASE_MEASURE_H

#include <stdint.h>

/*
 * TIM2 uses an adaptive internal edge prescaler.  The 1.25 MHz value is the
 * required capture-event-rate capability, not the OUT1 relay crossover.
 */
#define PHASE_MEASURE_MIN_HZ              10U
#define PHASE_MEASURE_MAX_HZ              10000000U
#define PHASE_MEASURE_CAPTURE_MIN_HZ       10U
#define PHASE_MEASURE_CAPTURE_MAX_HZ  1250000U
#define PHASE_MEASURE_DEFAULT_TIMER_HZ    84000000U
/*
 * Keep the advertised range unchanged, but allow clock/source tolerance at
 * its exact endpoints. The measured value itself is never clamped.
 */
#define PHASE_MEASURE_RANGE_GUARD_PERCENT 1U

/* OUT1 routing: RAW below 4.9 MHz, external /32 above 5.1 MHz. */
#define PHASE_MEASURE_DIVIDER_LOW         1U
#define PHASE_MEASURE_DIVIDER_HIGH       32U
#define PHASE_MEASURE_DIVIDER_THRESHOLD 5000000U
#define PHASE_MEASURE_DIVIDER_HYSTERESIS 100000U
#define PHASE_MEASURE_CAPTURE_CYCLES       1U

typedef enum
{
    PHASE_MEASURE_OK = 0,
    PHASE_MEASURE_ERROR,
    PHASE_MEASURE_INVALID_ARGUMENT,
    PHASE_MEASURE_OUT_OF_RANGE,
    PHASE_MEASURE_NOT_READY
} PhaseMeasureStatus_t;

typedef struct
{
    uint32_t timer_tick_hz;
    uint32_t capture_cycles;
    uint32_t input_min_hz;
    uint32_t input_max_hz;
    uint32_t divider_low;
    uint32_t divider_high;
    uint32_t divider_threshold_hz;
    uint32_t divider_hysteresis_hz;
} PhaseFrequencyConfig_t;

typedef struct
{
    uint32_t period_ticks;
    uint32_t divided_frequency_hz;
    uint32_t input_frequency_hz;
    float divided_frequency_hz_f;
    float input_frequency_hz_f;
    uint32_t divider;
    uint8_t valid;
} PhaseFrequencyResult_t;

void PhaseFrequencyConfig_Default(PhaseFrequencyConfig_t *config);

/* Select the external divider before a new capture window is started. */
uint32_t PhaseFrequency_SelectDivider(const PhaseFrequencyConfig_t *config,
                                      uint32_t estimated_input_hz,
                                      uint32_t current_divider);

/* Convert two timer timestamps into a period, including a 32-bit wrap. */
uint32_t PhaseFrequency_TimestampDelta(uint32_t previous,
                                       uint32_t current);

/* Convert one captured period into the original input frequency. */
PhaseMeasureStatus_t PhaseFrequency_FromPeriod(
    const PhaseFrequencyConfig_t *config,
    uint32_t period_ticks,
    uint32_t divider,
    PhaseFrequencyResult_t *result);

#endif /* PHASE_MEASURE_H */
