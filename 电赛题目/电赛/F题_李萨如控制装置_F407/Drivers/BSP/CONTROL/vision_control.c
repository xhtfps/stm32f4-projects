#include "vision_control.h"

#include <string.h>

#define VISION_CONTROL_ANCHOR_COUNT           28U
#define VISION_CONTROL_PRIME_COUNT            164U
#define VISION_CONTROL_PROBE_TOTAL            192U
#define VISION_CONTROL_PROBE_TIMEOUT_MS       50U
#define VISION_CONTROL_FEEDBACK_TIMEOUT_MS    100U
#define VISION_CONTROL_MAX_FEEDBACK_TIMEOUTS  20U
#define VISION_CONTROL_MIN_INPUT_MILLIHZ       1000000UL
#define VISION_CONTROL_MAX_INPUT_MILLIHZ       100000000UL
#define VISION_CONTROL_MAX_OUTPUT_MILLIHZ      200000000UL
#define VISION_CONTROL_SEARCH_CONFIDENCE       800U
#define VISION_CONTROL_LOCK_CONFIDENCE         850U
#define VISION_CONTROL_INITIAL_AMPLITUDE_MV    471.0f
#define VISION_CONTROL_MIN_AMPLITUDE_MV        20.0f
#define VISION_CONTROL_MAX_AMPLITUDE_MV        500.0f

/* K = fy / 100 Hz. These composite anchors catch common small ratios early. */
static const uint16_t s_anchor_k[VISION_CONTROL_ANCHOR_COUNT] =
{
    420U, 720U, 150U, 378U, 660U, 392U, 416U,
    345U, 663U, 435U, 465U, 555U, 1089U, 125U,
    65U, 81U, 123U, 129U, 141U, 529U, 667U,
    713U, 841U, 851U, 899U, 943U, 961U, 989U
};

static float VisionControl_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t VisionControl_AbsI32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t VisionControl_IsPrime(uint16_t value)
{
    uint16_t divisor;

    if (value < 2U)
        return 0U;
    if ((value & 1U) == 0U)
        return (value == 2U) ? 1U : 0U;
    for (divisor = 3U;
         (uint32_t)divisor * (uint32_t)divisor <= value;
         divisor = (uint16_t)(divisor + 2U))
    {
        if ((value % divisor) == 0U)
            return 0U;
    }
    return 1U;
}

static uint16_t VisionControl_PrimeAt(uint16_t index)
{
    uint16_t value;
    uint16_t found = 0U;

    for (value = 11U; value <= 997U; value = (uint16_t)(value + 2U))
    {
        if (VisionControl_IsPrime(value) != 0U)
        {
            if (found == index)
                return value;
            ++found;
        }
    }
    return 0U;
}

static uint32_t VisionControl_ProbeFrequency(uint16_t cursor)
{
    if (cursor < VISION_CONTROL_ANCHOR_COUNT)
        return (uint32_t)s_anchor_k[cursor] * 100000UL;
    return (uint32_t)VisionControl_PrimeAt(
        (uint16_t)(cursor - VISION_CONTROL_ANCHOR_COUNT)) * 200000UL;
}

static uint32_t VisionControl_NormalizePhase(int32_t phase_millideg)
{
    while (phase_millideg < 0)
        phase_millideg += 360000L;
    while (phase_millideg >= 360000L)
        phase_millideg -= 360000L;
    return (uint32_t)phase_millideg;
}

static void VisionControl_Request(VisionControl_t *control,
                                  uint32_t now_ms,
                                  uint16_t probe_id,
                                  uint32_t timeout_ms,
                                  uint8_t apply_dds,
                                  VisionControlAction_t *action)
{
    ++control->sequence;
    if (control->sequence == 0U)
        ++control->sequence;
    control->pending_probe_id = probe_id;
    control->waiting_for_frame = 1U;
    control->deadline_ms = now_ms + timeout_ms;

    action->apply_dds = apply_dds;
    action->send_probe = 1U;
    action->frequency_millihz = control->output_frequency_millihz;
    action->amplitude_mv = control->output_amplitude_mv;
    action->phase_millideg = control->output_phase_millideg;
    action->sequence = control->sequence;
    action->probe_id = probe_id;
    action->target_shape = control->target_shape;
}

static void VisionControl_RequestSearchProbe(VisionControl_t *control,
                                             uint32_t now_ms,
                                             VisionControlAction_t *action)
{
    uint16_t cursor = control->probe_cursor++;

    control->current_probe_id = (uint16_t)(cursor + 1U);
    control->output_frequency_millihz =
        VisionControl_ProbeFrequency(cursor);
    control->output_amplitude_mv = VISION_CONTROL_INITIAL_AMPLITUDE_MV;
    control->output_phase_millideg = 0U;
    VisionControl_Request(control, now_ms, control->current_probe_id,
                          VISION_CONTROL_PROBE_TIMEOUT_MS, 1U, action);
}

static void VisionControl_RequestFeedback(VisionControl_t *control,
                                          uint32_t now_ms,
                                          uint8_t apply_dds,
                                          VisionControlAction_t *action)
{
    VisionControl_Request(
        control, now_ms,
        (uint16_t)(0x8000U | (uint16_t)control->stage),
        VISION_CONTROL_FEEDBACK_TIMEOUT_MS, apply_dds, action);
}

static uint8_t VisionControl_RatioMatches(
    const VisionControl_t *control,
    const VisionLinkMeasurement_t *measurement)
{
    return ((measurement->valid != 0U) &&
            (measurement->confidence_permille >=
             VISION_CONTROL_LOCK_CONFIDENCE) &&
            (measurement->ratio_y == control->expected_ratio_y) &&
            (measurement->ratio_x == control->expected_ratio_x)) ? 1U : 0U;
}

static uint8_t VisionControl_HandleSearchMeasurement(
    VisionControl_t *control,
    const VisionLinkMeasurement_t *measurement)
{
    uint64_t input_millihz;
    uint32_t rounded;
    uint32_t difference;
    uint32_t tolerance;

    if ((measurement->valid == 0U) ||
        (measurement->confidence_permille <
         VISION_CONTROL_SEARCH_CONFIDENCE) ||
        (measurement->ratio_y == 0U) ||
        (measurement->ratio_x == 0U) ||
        (measurement->ratio_y > 20U) ||
        (measurement->ratio_x > 20U))
        return 0U;

    /* Protocol ratio is reduced fy:fx, hence fx = fy * x / y. */
    input_millihz =
        (uint64_t)control->output_frequency_millihz *
        measurement->ratio_x;
    input_millihz = (input_millihz + measurement->ratio_y / 2U) /
        measurement->ratio_y;
    if ((input_millihz < VISION_CONTROL_MIN_INPUT_MILLIHZ) ||
        (input_millihz > VISION_CONTROL_MAX_INPUT_MILLIHZ))
        return 0U;

    rounded = (uint32_t)(((input_millihz + 50000ULL) / 100000ULL) *
                         100000ULL);
    if (control->candidate_hits == 0U)
    {
        control->candidate_input_millihz = rounded;
        control->candidate_hits = 1U;
        control->stage = VISION_CONTROL_LOCAL_CONFIRM;
        return 1U;
    }

    difference = (rounded > control->candidate_input_millihz) ?
        (rounded - control->candidate_input_millihz) :
        (control->candidate_input_millihz - rounded);
    tolerance = control->candidate_input_millihz / 100U;
    if (tolerance < 200000UL)
        tolerance = 200000UL;
    if (difference > tolerance)
        return 0U;

    control->candidate_input_millihz =
        (control->candidate_input_millihz + rounded) / 2U;
    control->candidate_hits = 2U;
    control->input_frequency_millihz = control->candidate_input_millihz;
    control->output_frequency_millihz =
        (control->target_shape == 3U) ?
        control->input_frequency_millihz * 2UL :
        control->input_frequency_millihz;
    control->expected_ratio_y =
        (control->target_shape == 3U) ? 2U : 1U;
    control->expected_ratio_x = 1U;
    if (control->output_frequency_millihz >
        VISION_CONTROL_MAX_OUTPUT_MILLIHZ)
    {
        control->stage = VISION_CONTROL_ERROR;
        control->active = 0U;
        return 1U;
    }
    control->stage = VISION_CONTROL_FREQUENCY_LOCK;
    control->stable_count = 0U;
    control->timeout_count = 0U;
    return 1U;
}

void VisionControl_Init(VisionControl_t *control)
{
    if (control != NULL)
        (void)memset(control, 0, sizeof(*control));
}

uint8_t VisionControl_Start(VisionControl_t *control,
                            uint8_t target_shape,
                            uint32_t now_ms)
{
    if ((control == NULL) || (target_shape < 1U) || (target_shape > 3U))
        return 0U;
    VisionControl_Init(control);
    control->active = 1U;
    control->target_shape = target_shape;
    control->stage = VISION_CONTROL_COARSE_SWEEP;
    control->start_ms = now_ms;
    return 1U;
}

void VisionControl_Stop(VisionControl_t *control)
{
    if (control == NULL)
        return;
    control->active = 0U;
    control->waiting_for_frame = 0U;
    control->stage = VISION_CONTROL_IDLE;
}

void VisionControl_Abort(VisionControl_t *control)
{
    if (control == NULL)
        return;
    control->active = 0U;
    control->waiting_for_frame = 0U;
    control->stage = VISION_CONTROL_ERROR;
}

uint8_t VisionControl_Process(VisionControl_t *control,
                              uint32_t now_ms,
                              const VisionLinkMeasurement_t *measurement,
                              VisionControlAction_t *action)
{
    uint8_t changed = 0U;
    uint8_t accepted = 0U;
    uint8_t apply_dds = 0U;
    float span_error;
    float amplitude_step;
    int32_t phase_step;
    int32_t frequency_step_millihz;

    if ((control == NULL) || (action == NULL))
        return 0U;
    (void)memset(action, 0, sizeof(*action));
    if (control->active == 0U)
        return 0U;

    if ((measurement != NULL) &&
        (measurement->sequence == control->sequence) &&
        (measurement->probe_id == control->pending_probe_id))
    {
        control->latest = *measurement;
        control->waiting_for_frame = 0U;
        control->timeout_count = 0U;
        accepted = 1U;
        changed = 1U;
    }

    if (control->waiting_for_frame != 0U)
    {
        if ((int32_t)(now_ms - control->deadline_ms) < 0)
            return changed;
        control->waiting_for_frame = 0U;
        ++control->timeout_count;
        changed = 1U;
        if ((control->stage != VISION_CONTROL_COARSE_SWEEP) &&
            (control->stage != VISION_CONTROL_LOCAL_CONFIRM) &&
            (control->timeout_count >=
             VISION_CONTROL_MAX_FEEDBACK_TIMEOUTS))
        {
            VisionControl_Abort(control);
            return 1U;
        }
    }

    if ((control->stage == VISION_CONTROL_COARSE_SWEEP) ||
        (control->stage == VISION_CONTROL_LOCAL_CONFIRM))
    {
        if (accepted != 0U)
            changed |= VisionControl_HandleSearchMeasurement(
                control, measurement);
        if (control->stage == VISION_CONTROL_ERROR)
            return 1U;
        if (control->stage == VISION_CONTROL_FREQUENCY_LOCK)
        {
            VisionControl_RequestFeedback(control, now_ms, 1U, action);
            return 1U;
        }
        if (control->probe_cursor >= VISION_CONTROL_PROBE_TOTAL)
        {
            VisionControl_Abort(control);
            return 1U;
        }
        VisionControl_RequestSearchProbe(control, now_ms, action);
        return 1U;
    }

    if (accepted != 0U)
    {
        if (control->stage == VISION_CONTROL_FREQUENCY_LOCK)
        {
            control->stable_count = VisionControl_RatioMatches(
                control, measurement) ?
                (uint8_t)(control->stable_count + 1U) : 0U;
            if (control->stable_count >= 3U)
            {
                control->stage = VISION_CONTROL_AMPLITUDE_LOCK;
                control->stable_count = 0U;
            }
        }
        else if (control->stage == VISION_CONTROL_AMPLITUDE_LOCK)
        {
            span_error = 8.0f -
                (float)measurement->span_y_millidiv / 1000.0f;
            if ((VisionControl_RatioMatches(control, measurement) != 0U) &&
                (VisionControl_AbsFloat(span_error) <= 0.20f))
                ++control->stable_count;
            else
                control->stable_count = 0U;

            amplitude_step = span_error * 20.0f;
            if (amplitude_step > 30.0f)
                amplitude_step = 30.0f;
            else if (amplitude_step < -30.0f)
                amplitude_step = -30.0f;
            control->output_amplitude_mv += amplitude_step;
            if (control->output_amplitude_mv >
                VISION_CONTROL_MAX_AMPLITUDE_MV)
                control->output_amplitude_mv =
                    VISION_CONTROL_MAX_AMPLITUDE_MV;
            else if (control->output_amplitude_mv <
                     VISION_CONTROL_MIN_AMPLITUDE_MV)
                control->output_amplitude_mv =
                    VISION_CONTROL_MIN_AMPLITUDE_MV;
            apply_dds = 1U;
            if (control->stable_count >= 3U)
            {
                control->stage = VISION_CONTROL_PHASE_LOCK;
                control->stable_count = 0U;
            }
        }
        else if ((control->stage == VISION_CONTROL_PHASE_LOCK) ||
                 (control->stage == VISION_CONTROL_HOLD))
        {
            frequency_step_millihz =
                -measurement->phase_rate_millideg_s / 360L;
            if (frequency_step_millihz > 2000L)
                frequency_step_millihz = 2000L;
            else if (frequency_step_millihz < -2000L)
                frequency_step_millihz = -2000L;
            if ((frequency_step_millihz < 0) &&
                ((uint32_t)(-frequency_step_millihz) >=
                 control->output_frequency_millihz))
                control->output_frequency_millihz = 1U;
            else
                control->output_frequency_millihz = (uint32_t)(
                    (int32_t)control->output_frequency_millihz +
                    frequency_step_millihz);

            phase_step = -measurement->phase_error_millideg;
            if (phase_step > 15000L)
                phase_step = 15000L;
            else if (phase_step < -15000L)
                phase_step = -15000L;
            control->output_phase_millideg = VisionControl_NormalizePhase(
                (int32_t)control->output_phase_millideg + phase_step);
            apply_dds = 1U;

            if ((VisionControl_RatioMatches(control, measurement) != 0U) &&
                (VisionControl_AbsI32(
                    measurement->phase_error_millideg) <= 5000L) &&
                (VisionControl_AbsI32(
                    measurement->phase_rate_millideg_s) <= 3600L))
                ++control->stable_count;
            else
                control->stable_count = 0U;

            if ((control->stage == VISION_CONTROL_PHASE_LOCK) &&
                (control->stable_count >= 3U))
            {
                control->stage = VISION_CONTROL_VERIFY;
                control->stable_count = 0U;
            }
            else if ((control->stage == VISION_CONTROL_HOLD) &&
                     (control->stable_count == 0U))
            {
                control->stage = VISION_CONTROL_PHASE_LOCK;
            }
        }
        else if (control->stage == VISION_CONTROL_VERIFY)
        {
            span_error = 8.0f -
                (float)measurement->span_y_millidiv / 1000.0f;
            if ((VisionControl_RatioMatches(control, measurement) != 0U) &&
                (VisionControl_AbsFloat(span_error) <= 0.25f) &&
                (VisionControl_AbsI32(
                    measurement->phase_error_millideg) <= 8000L) &&
                (VisionControl_AbsI32(
                    measurement->phase_rate_millideg_s) <= 5000L))
                ++control->stable_count;
            else
                control->stable_count = 0U;
            if (control->stable_count >= 5U)
            {
                control->stage = VISION_CONTROL_HOLD;
                control->stable_count = 1U;
            }
        }
    }

    VisionControl_RequestFeedback(control, now_ms, apply_dds, action);
    return 1U;
}

uint16_t VisionControl_GetProbeTotal(void)
{
    return VISION_CONTROL_PROBE_TOTAL;
}
