/**
 * @file    wired_control.c
 * @brief   F题要求1~4频率、相位和幅值PI闭环实现。
 */

#include "wired_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* 题目输入频率范围以及二倍频输出上限。 */
#define WIRED_CONTROL_MIN_FREQUENCY_HZ        900.0f
#define WIRED_CONTROL_MAX_FREQUENCY_HZ     200000.0f
/* 连续削顶时先把DDS幅度命令降低10%，保护ADC和后级。 */
#define WIRED_CONTROL_CLIP_REDUCTION          0.90f
#define WIRED_CONTROL_REFEED_STABILITY_MV       50U
#define WIRED_CONTROL_REFEED_TRIGGER_MV         200U
#define WIRED_CONTROL_PI_F                    3.14159265358979323846f
#define WIRED_CONTROL_RAD_TO_DEG             57.29577951308232f
#define WIRED_CONTROL_PID_GAIN_MAX          1000.0f
#define WIRED_CONTROL_FREQUENCY_PROFILE_BOUNDARY_HZ 4500.0f
#define WIRED_CONTROL_PHASE_PROFILE_CONFIRM_COUNT 3U
#define WIRED_CONTROL_LOW_GAIN_SCALE_MIN       0.20f

/** 单个PI的运行时状态。 */
typedef struct
{
    float integral;
    float previous_error;
    float filtered_derivative;
    float output;
    uint8_t derivative_ready;
} WiredPiRuntime_t;

/** 模块唯一实例，避免在F407上进行动态内存分配。 */
typedef struct
{
    WiredControlConfig_t config;
    WiredControlResult_t result;
    WiredControlCommand_t pending_command;
    WiredPiRuntime_t frequency_pi;
    WiredPiRuntime_t phase_pi;
    WiredPiRuntime_t amplitude_pi;
    uint32_t start_ms;
    uint32_t last_update_ms;
    uint32_t verify_start_ms;
    uint32_t phase_cal_state_start_ms;
    uint32_t phase_cal_last_block_count;
    uint32_t frequency_control_last_sample_count;
    uint32_t frequency_control_last_update_ms;
    uint32_t phase_control_last_sample_count;
    uint32_t phase_control_last_update_ms;
    uint32_t amplitude_settle_start_ms;
    uint32_t amplitude_last_block_count;
    uint32_t amplitude_last_update_ms;
    uint32_t amplitude_refeed_candidate_vpp_mv;
    float amplitude_feedforward_mv;
    float filtered_input_vpp_mv;
    float input_baseline_vpp_mv;
    float filtered_output_vpp_mv;
    float amplitude_sample_dt_s;
    float phase_cal_sin_sum;
    float phase_cal_cos_sum;
    float phase_calibrated_frequency_hz;
    uint16_t phase_cal_sample_count;
    uint8_t amplitude_feedforward_ready;
    uint8_t amplitude_target_fixed;
    uint8_t input_baseline_ready;
    uint8_t amplitude_filter_ready;
    uint8_t amplitude_new_sample;
    uint8_t amplitude_refeed_count;
    uint8_t output_clip_bad_count;
    uint8_t output_clip_good_count;
    uint8_t phase_cal_retry_count;
    uint8_t phase_recal_bad_count;
    uint8_t phase_frequency_change_count;
    uint8_t low_frequency_profile_active;
    uint8_t low_frequency_profile_switch_count;
    uint8_t frequency_good_count;
    uint8_t amplitude_good_count;
    uint8_t phase_good_count;
    uint8_t command_pending;
    uint8_t clip_active;
    uint8_t running;
} WiredControlContext_t;

static WiredControlContext_t s_control;

/** 把浮点数限制在给定区间。 */
static float WiredControl_ClampFloat(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

/* Confirm a band change over several control ticks to avoid boundary chatter. */
static uint8_t WiredControl_UpdateFrequencyProfile(float target_frequency_hz)
{
    uint8_t requested_low =
        (target_frequency_hz <
         WIRED_CONTROL_FREQUENCY_PROFILE_BOUNDARY_HZ) ? 1U : 0U;

    if (requested_low == s_control.low_frequency_profile_active)
    {
        s_control.low_frequency_profile_switch_count = 0U;
        return 0U;
    }

    if (s_control.low_frequency_profile_switch_count < 255U)
    {
        ++s_control.low_frequency_profile_switch_count;
    }
    if (s_control.low_frequency_profile_switch_count <
        WIRED_CONTROL_PHASE_PROFILE_CONFIRM_COUNT)
    {
        return 0U;
    }

    s_control.low_frequency_profile_active = requested_low;
    s_control.low_frequency_profile_switch_count = 0U;
    return 1U;
}

/** 返回浮点绝对值，避免不同库配置下fabs/fabsf差异。 */
static float WiredControl_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 返回有符号32位整数的绝对值，输入范围远离INT32_MIN。 */
static int32_t WiredControl_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static float WiredControl_NormalizeDegrees(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }
    while (degrees <= -180.0f)
    {
        degrees += 360.0f;
    }
    return degrees;
}

static float WiredControl_PositiveDegrees(float degrees)
{
    degrees = WiredControl_NormalizeDegrees(degrees);
    return (degrees < 0.0f) ? degrees + 360.0f : degrees;
}

static float WiredControl_MaxFloat(float first, float second)
{
    return (first > second) ? first : second;
}

static uint32_t WiredControl_AbsDiffU32(
    uint32_t first,
    uint32_t second)
{
    return (first >= second) ?
           (first - second) : (second - first);
}

/** 清除一个PI运行时，不改变参数。 */
static void WiredControl_ResetPi(WiredPiRuntime_t *runtime)
{
    if (runtime != NULL)
    {
        runtime->integral = 0.0f;
        runtime->previous_error = 0.0f;
        runtime->filtered_derivative = 0.0f;
        runtime->output = 0.0f;
        runtime->derivative_ready = 0U;
    }
}

/**
 * @brief 执行带积分分离和抗饱和的反向PI。
 *
 * 本工程统一定义误差为“实际值-目标值”。因此执行量修正使用
 * u=-(Kp*e+Ki*∫e)，正误差会降低DDS命令，负误差会提高命令。
 */
static float WiredControl_RunPi(
    WiredPiRuntime_t *runtime,
    const WiredPiConfig_t *config,
    float error,
    float dt_s,
    float gain_scale,
    uint8_t circular_error)
{
    float candidate_integral;
    float candidate_output;
    float derivative_delta;
    float raw_derivative;
    float derivative_alpha;
    float limited_output;
    uint8_t drives_further_into_saturation;

    if ((runtime == NULL) || (config == NULL))
    {
        return 0.0f;
    }

    if ((runtime->derivative_ready != 0U) && (dt_s > 0.0f))
    {
        derivative_delta = error - runtime->previous_error;
        if (circular_error != 0U)
        {
            derivative_delta =
                WiredControl_NormalizeDegrees(derivative_delta);
        }
        raw_derivative = derivative_delta / dt_s;
        derivative_alpha = WiredControl_ClampFloat(
            config->derivative_filter_alpha, 0.0f, 1.0f);
        runtime->filtered_derivative += derivative_alpha *
            (raw_derivative - runtime->filtered_derivative);
    }
    else
    {
        runtime->filtered_derivative = 0.0f;
        runtime->derivative_ready = 1U;
    }
    runtime->previous_error = error;

    candidate_integral = runtime->integral;
    if (WiredControl_AbsFloat(error) <=
        config->integral_separation)
    {
        candidate_integral += error * dt_s;
    }

    candidate_output =
        -(config->kp * gain_scale * error +
          config->ki * gain_scale * candidate_integral +
          config->kd * gain_scale * runtime->filtered_derivative);
    limited_output = WiredControl_ClampFloat(
        candidate_output,
        config->output_min,
        config->output_max);

    /*
     * 若积分会把已经饱和的输出继续推向饱和端，则拒绝本次积分；
     * 若误差正在帮助退出饱和，则允许积分回退。
     */
    drives_further_into_saturation = 0U;
    if ((candidate_output < config->output_min) &&
        (error > 0.0f))
    {
        drives_further_into_saturation = 1U;
    }
    else if ((candidate_output > config->output_max) &&
             (error < 0.0f))
    {
        drives_further_into_saturation = 1U;
    }

    if (drives_further_into_saturation == 0U)
    {
        runtime->integral = candidate_integral;
    }
    else
    {
        candidate_output =
            -(config->kp * gain_scale * error +
              config->ki * gain_scale * runtime->integral +
              config->kd * gain_scale * runtime->filtered_derivative);
        limited_output = WiredControl_ClampFloat(
            candidate_output,
            config->output_min,
            config->output_max);
    }

    runtime->output = limited_output;
    return limited_output;
}

/** 根据模式得到目标频率倍数。 */
static float WiredControl_FrequencyRatio(WiredControlMode_t mode)
{
    return (mode == WIRED_CONTROL_MODE_REQUIREMENT3) ?
           2.0f : 1.0f;
}

/** 根据模式得到未校准的物理相位目标。 */
static float WiredControl_BasePhaseDegrees(
    WiredControlMode_t mode)
{
    /*
     * 要求1和要求3的基准相位为0°；要求2以90°作为正交相位。
     * DPLL通过微调频率消除剩余相差。
     */
    return (mode == WIRED_CONTROL_MODE_REQUIREMENT2) ?
           90.0f : 0.0f;
}

static void WiredControl_UpdatePhaseTargets(void)
{
    float target_deg;

    target_deg = WiredControl_PositiveDegrees(
        WiredControl_BasePhaseDegrees(s_control.result.mode) +
        s_control.result.phase_calibration_offset_deg);
    s_control.result.capture_phase_target_deg = target_deg;
}

static void WiredControl_ResetPhaseSamples(uint32_t now_ms)
{
    s_control.phase_cal_sin_sum = 0.0f;
    s_control.phase_cal_cos_sum = 0.0f;
    s_control.phase_cal_sample_count = 0U;
    s_control.phase_cal_state_start_ms = now_ms;
}

static void WiredControl_BeginPhaseCalibration(uint32_t now_ms)
{
    s_control.result.phase_calibrated = 0U;
    s_control.result.phase_calibration_state =
        WIRED_PHASE_CAL_CAPTURE_LOCK;
    s_control.result.locked = 0U;
    s_control.phase_cal_retry_count = 0U;
    s_control.phase_recal_bad_count = 0U;
    s_control.phase_good_count = 0U;
    WiredControl_ResetPhaseSamples(now_ms);
}

static uint8_t WiredControl_AmplitudeFeedbackValid(
    const WiredControlInput_t *input)
{
    return ((input != NULL) &&
            (input->amplitude_valid != 0U) &&
            (input->input_bias_valid != 0U) &&
            (input->output_bias_valid != 0U) &&
            (input->clipped == 0U)) ? 1U : 0U;
}

static uint8_t WiredControl_InputAmplitudeReferenceValid(
    const WiredControlInput_t *input)
{
    return ((input != NULL) &&
            (input->input_amplitude_valid != 0U)) ? 1U : 0U;
}

static uint8_t WiredControl_CollectAnalogPhase(
    const WiredControlInput_t *input)
{
    float radians;

    if ((input == NULL) ||
        (input->analog_phase_valid == 0U) ||
        (input->clipped != 0U) ||
        (s_control.result.amplitude_refeed_active != 0U) ||
        (input->adc_block_count == s_control.phase_cal_last_block_count))
    {
        return 0U;
    }

    s_control.phase_cal_last_block_count = input->adc_block_count;
    radians = input->analog_phase_error_deg /
              WIRED_CONTROL_RAD_TO_DEG;
    s_control.phase_cal_sin_sum += sinf(radians);
    s_control.phase_cal_cos_sum += cosf(radians);
    if (s_control.phase_cal_sample_count < 65535U)
    {
        ++s_control.phase_cal_sample_count;
    }
    return 1U;
}

static uint8_t WiredControl_GetAnalogPhaseMean(
    float *mean_error_deg,
    float *resultant)
{
    float magnitude;

    if ((mean_error_deg == NULL) || (resultant == NULL) ||
        (s_control.phase_cal_sample_count == 0U))
    {
        return 0U;
    }
    *mean_error_deg = atan2f(
        s_control.phase_cal_sin_sum,
        s_control.phase_cal_cos_sum) * WIRED_CONTROL_RAD_TO_DEG;
    magnitude = sqrtf(
        s_control.phase_cal_sin_sum * s_control.phase_cal_sin_sum +
        s_control.phase_cal_cos_sum * s_control.phase_cal_cos_sum);
    *resultant = magnitude /
                 (float)s_control.phase_cal_sample_count;
    return 1U;
}

static uint8_t WiredControl_ApplyPhaseCalibration(
    float mean_error_deg,
    uint32_t now_ms)
{
    float correction_deg = -mean_error_deg;

    if (WiredControl_AbsFloat(correction_deg) >
        s_control.config.phase_cal_max_correction_deg)
    {
        s_control.result.phase_calibration_state =
            WIRED_PHASE_CAL_FAILED;
        s_control.result.phase_calibrated = 0U;
        return 0U;
    }

    s_control.result.phase_calibration_offset_deg =
        WiredControl_NormalizeDegrees(
            s_control.result.phase_calibration_offset_deg +
            correction_deg);
    WiredControl_UpdatePhaseTargets();
    s_control.phase_good_count = 0U;
    s_control.result.phase_calibration_state =
        WIRED_PHASE_CAL_RELOCK;
    WiredControl_ResetPhaseSamples(now_ms);
    return 1U;
}

/** 把状态切换到下一阶段并重置对应的连续达标计数。 */
static void WiredControl_SetStage(WiredControlStage_t stage)
{
    if (s_control.result.stage == stage)
    {
        return;
    }

    s_control.result.stage = stage;
    s_control.frequency_good_count = 0U;
    s_control.amplitude_good_count = 0U;
    s_control.phase_good_count = 0U;
    s_control.verify_start_ms = 0U;

    if (stage == WIRED_CONTROL_STAGE_FREQUENCY)
    {
        WiredControl_ResetPi(&s_control.phase_pi);
        s_control.result.phase_pi_correction_hz = 0.0f;
        s_control.result.phase_calibrated = 0U;
        s_control.result.phase_calibration_state =
            WIRED_PHASE_CAL_IDLE;
        s_control.result.locked = 0U;
    }
    else if (stage == WIRED_CONTROL_STAGE_AMPLITUDE)
    {
        s_control.result.locked = 0U;
    }
    else if (stage == WIRED_CONTROL_STAGE_PHASE)
    {
        WiredControl_BeginPhaseCalibration(
            s_control.last_update_ms);
    }
}

/** 生成并缓存一条完整DDS命令。 */
static void WiredControl_QueueCommand(
    float target_frequency_hz,
    float frequency_correction_hz,
    float amplitude_mv)
{
    uint32_t frequency_millihz;
    uint32_t phase_millideg;
    uint32_t target_frequency_millihz;
    int32_t correction_millihz;
    int64_t corrected_frequency_millihz;
    uint8_t changed;

    target_frequency_hz = WiredControl_ClampFloat(
        target_frequency_hz,
        WIRED_CONTROL_MIN_FREQUENCY_HZ,
        WIRED_CONTROL_MAX_FREQUENCY_HZ);
    amplitude_mv = WiredControl_ClampFloat(
        amplitude_mv,
        s_control.config.dds_amplitude_min_mv,
        s_control.config.dds_amplitude_max_mv);
    /*
     * 不能先把微小PI修正量加到100kHz量级的float再转毫赫兹：
     * 单精度float在100kHz附近的间隔约为7.8mHz，会吞掉相位环产生
     * 的亚毫赫兹修正。这里先分别量化，再用整数毫赫兹相加。
     */
    target_frequency_millihz =
        (uint32_t)(target_frequency_hz * 1000.0f + 0.5f);
    correction_millihz =
        (frequency_correction_hz >= 0.0f) ?
        (int32_t)(frequency_correction_hz * 1000.0f + 0.5f) :
        (int32_t)(frequency_correction_hz * 1000.0f - 0.5f);
    corrected_frequency_millihz =
        (int64_t)target_frequency_millihz +
        (int64_t)correction_millihz;
    if (corrected_frequency_millihz <
        (int64_t)(WIRED_CONTROL_MIN_FREQUENCY_HZ * 1000.0f))
    {
        corrected_frequency_millihz =
            (int64_t)(WIRED_CONTROL_MIN_FREQUENCY_HZ * 1000.0f);
    }
    else if (corrected_frequency_millihz >
             (int64_t)(WIRED_CONTROL_MAX_FREQUENCY_HZ * 1000.0f))
    {
        corrected_frequency_millihz =
            (int64_t)(WIRED_CONTROL_MAX_FREQUENCY_HZ * 1000.0f);
    }
    frequency_millihz =
        (uint32_t)corrected_frequency_millihz;
    phase_millideg = (uint32_t)(
        WiredControl_PositiveDegrees(
            s_control.result.capture_phase_target_deg) *
        1000.0f + 0.5f);
    if (phase_millideg >= 360000UL)
    {
        phase_millideg -= 360000UL;
    }

    changed = 0U;
    if ((s_control.result.command_valid == 0U) ||
        (s_control.result.command_frequency_millihz !=
         frequency_millihz) ||
        (WiredControl_AbsFloat(
            s_control.result.command_amplitude_mv -
            amplitude_mv) >=
         s_control.config.amplitude_command_deadband_mv) ||
        (s_control.result.command_phase_millideg !=
         phase_millideg))
    {
        changed = 1U;
    }

    if (changed != 0U)
    {
        s_control.result.command_frequency_millihz =
            frequency_millihz;
        s_control.result.command_amplitude_mv = amplitude_mv;
        s_control.result.command_phase_millideg =
            phase_millideg;
        s_control.result.command_valid = 1U;
        s_control.pending_command.frequency_millihz =
            frequency_millihz;
        s_control.pending_command.amplitude_mv =
            amplitude_mv;
        s_control.pending_command.phase_millideg =
            phase_millideg;
        s_control.pending_command.output_enable = 1U;
        s_control.command_pending = 1U;
    }
}

/** 判断当前三类误差是否都已满足验证阈值。 */
static uint8_t WiredControl_AllErrorsLocked(
    const WiredControlInput_t *input)
{
    if ((input->feedback_frequency_valid == 0U) ||
        (s_control.result.amplitude_feedback_valid == 0U) ||
        (s_control.result.amplitude_refeed_active != 0U) ||
        (s_control.amplitude_refeed_count != 0U) ||
        (WiredControl_AbsFloat(
            s_control.result.frequency_error_hz) >
         s_control.config.frequency_lock_hz) ||
        (WiredControl_AbsInt32(
            s_control.result.amplitude_error_mv) >
         s_control.config.amplitude_lock_mv))
    {
        return 0U;
    }

    if ((s_control.result.mode !=
         WIRED_CONTROL_MODE_REQUIREMENT4) &&
        ((input->phase_valid == 0U) ||
         (input->analog_phase_valid == 0U) ||
         (s_control.result.phase_calibrated == 0U) ||
         (WiredControl_AbsFloat(
            s_control.result.phase_error_deg) >
          s_control.config.phase_lock_deg) ||
         (WiredControl_AbsFloat(
            s_control.result.analog_phase_error_deg) >
          s_control.config.phase_cal_verify_deg)))
    {
        return 0U;
    }
    return 1U;
}

void WiredControlConfig_Default(WiredControlConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));

    /*
     * 频率环只做粗锁；相位环才负责长期消除微小频偏。
     * 单位分别为 Hz/Hz、Hz/(Hz*s)。
     */
    config->frequency_pi.kp = 1.0f;
    config->frequency_pi.ki = 2.0f;
    config->frequency_pi.kd = 0.0f;
    config->frequency_pi.derivative_filter_alpha = 0.0f;
    config->frequency_pi.output_min = -10.0f;
    config->frequency_pi.output_max = 10.0f;
    config->frequency_pi.integral_separation = 5.0f;

    /* Low-band values are the 4.5 kHz endpoint before gain scheduling. */
    config->frequency_pi_low = config->frequency_pi;
    config->frequency_pi_low.kp = 0.25f;
    config->frequency_pi_low.ki = 0.03f;
    config->frequency_pi_low.kd = 0.0f;
    config->frequency_pi_low.derivative_filter_alpha = 0.0f;
    config->frequency_pi_low.output_min = -3.0f;
    config->frequency_pi_low.output_max = 3.0f;
    config->frequency_pi_low.integral_separation = 2.0f;

    /*
     * 相位PI的输出不是“相位角”，而是DDS频率修正量。
     * 单位分别为 Hz/deg、Hz/(deg*s)。
     */
    config->phase_pi.kp = 0.2f;
    config->phase_pi.ki = 0.14f;
    config->phase_pi.kd = 0.001f;
    config->phase_pi.derivative_filter_alpha = 0.05f;
    config->phase_pi.output_min = -5.0f;
    config->phase_pi.output_max = 5.0f;
    config->phase_pi.integral_separation = 30.0f;

    /* Low-band values are the 4.5 kHz endpoint before gain scheduling. */
    config->phase_pi_low = config->phase_pi;
    config->phase_pi_low.kp = 0.08f;
    config->phase_pi_low.ki = 0.10f;
    config->phase_pi_low.kd = 0.001f;
    config->phase_pi_low.derivative_filter_alpha = 0.05f;
    config->phase_pi_low.output_min = -1.5f;
    config->phase_pi_low.output_max = 1.5f;
    config->phase_pi_low.integral_separation = 15.0f;

    /*
     * 幅值误差使用示波器端mV，输出为AD9959端mVpp修正量。
     */
    config->amplitude_pi.kp = 0.000400f;
    config->amplitude_pi.ki = 0.050000f;
    config->amplitude_pi.kd = 0.000f;
    config->amplitude_pi.derivative_filter_alpha = 0.00f;
    config->amplitude_pi.output_min = -250.0f;
    config->amplitude_pi.output_max = 250.0f;
    config->amplitude_pi.integral_separation = 500.0f;

    config->update_period_ms = 20U;
    config->verify_time_ms = 500U;
    config->phase_cal_settle_ms = 200U;
    config->phase_cal_min_time_ms = 200U;
    config->amplitude_settle_ms = 200U;
    config->frequency_lock_hz = 0.50f;
    config->frequency_release_hz = 2.00f;
    config->amplitude_lock_mv = 50;
    config->amplitude_release_mv = 120;
    config->phase_lock_deg = 1.0f;
    config->phase_release_deg = 3.0f;
    config->phase_cal_verify_deg = 2.0f;
    config->phase_cal_max_correction_deg = 120.0f;
    config->phase_cal_min_resultant = 0.995f;
    config->amplitude_filter_alpha = 0.125f;
    config->amplitude_command_deadband_mv = 1.0f;
    config->amplitude_control_deadband_mv = 30;
    config->frequency_lock_count = 5U;
    config->amplitude_lock_count = 5U;
    config->phase_lock_count = 10U;
    config->phase_cal_sample_count = 32U;
    config->phase_cal_verify_count = 16U;
    config->phase_cal_retry_limit = 1U;
    config->phase_recal_release_count = 5U;
    config->amplitude_refeed_count = 5U;
    config->output_clip_assert_count = 3U;
    config->output_clip_release_count = 8U;
    config->dds_amplitude_min_mv = 5.0f;
    config->dds_amplitude_max_mv = 500.0f;
    config->output_chain_gain = 8.6f;
}

void wired_control_init(const WiredControlConfig_t *config)
{
    WiredControlConfig_t default_config;

    (void)memset(&s_control, 0, sizeof(s_control));
    if (config == NULL)
    {
        WiredControlConfig_Default(&default_config);
        s_control.config = default_config;
    }
    else
    {
        s_control.config = *config;
    }
    s_control.result.stage = WIRED_CONTROL_STAGE_STOPPED;
}

uint8_t wired_control_get_pid_config(
    WiredControlLoop_t loop,
    WiredPiConfig_t *config)
{
    if (config == NULL)
    {
        return 0U;
    }

    switch (loop)
    {
        case WIRED_CONTROL_LOOP_FREQUENCY:
            *config = s_control.config.frequency_pi;
            break;
        case WIRED_CONTROL_LOOP_FREQUENCY_LOW:
            *config = s_control.config.frequency_pi_low;
            break;
        case WIRED_CONTROL_LOOP_PHASE:
            *config = s_control.config.phase_pi;
            break;
        case WIRED_CONTROL_LOOP_PHASE_LOW:
            *config = s_control.config.phase_pi_low;
            break;
        case WIRED_CONTROL_LOOP_AMPLITUDE:
            *config = s_control.config.amplitude_pi;
            break;
        default:
            return 0U;
    }
    return 1U;
}

uint8_t wired_control_set_pid_config(
    WiredControlLoop_t loop,
    float kp,
    float ki,
    float kd,
    float derivative_filter_alpha)
{
    WiredPiConfig_t *config;
    WiredPiRuntime_t *runtime;
    float *result_output;

    if (!(kp >= 0.0f) || !(kp <= WIRED_CONTROL_PID_GAIN_MAX) ||
        !(ki >= 0.0f) || !(ki <= WIRED_CONTROL_PID_GAIN_MAX) ||
        !(kd >= 0.0f) || !(kd <= WIRED_CONTROL_PID_GAIN_MAX) ||
        !(derivative_filter_alpha >= 0.0f) ||
        !(derivative_filter_alpha <= 1.0f))
    {
        return 0U;
    }

    switch (loop)
    {
        case WIRED_CONTROL_LOOP_FREQUENCY:
            config = &s_control.config.frequency_pi;
            runtime = &s_control.frequency_pi;
            result_output =
                &s_control.result.frequency_pi_correction_hz;
            break;
        case WIRED_CONTROL_LOOP_FREQUENCY_LOW:
            config = &s_control.config.frequency_pi_low;
            runtime = &s_control.frequency_pi;
            result_output =
                &s_control.result.frequency_pi_correction_hz;
            break;
        case WIRED_CONTROL_LOOP_PHASE:
            config = &s_control.config.phase_pi;
            runtime = &s_control.phase_pi;
            result_output = &s_control.result.phase_pi_correction_hz;
            break;
        case WIRED_CONTROL_LOOP_PHASE_LOW:
            config = &s_control.config.phase_pi_low;
            runtime = &s_control.phase_pi;
            result_output = &s_control.result.phase_pi_correction_hz;
            break;
        case WIRED_CONTROL_LOOP_AMPLITUDE:
            config = &s_control.config.amplitude_pi;
            runtime = &s_control.amplitude_pi;
            result_output =
                &s_control.result.amplitude_pi_correction_mv;
            break;
        default:
            return 0U;
    }

    config->kp = kp;
    config->ki = ki;
    config->kd = kd;
    config->derivative_filter_alpha = derivative_filter_alpha;
    WiredControl_ResetPi(runtime);
    *result_output = 0.0f;
    return 1U;
}

uint8_t wired_control_start(
    WiredControlMode_t mode,
    uint8_t target_div,
    uint32_t now_ms)
{
    uint32_t target_vpp_mv;

    if ((mode < WIRED_CONTROL_MODE_REQUIREMENT1) ||
        (mode > WIRED_CONTROL_MODE_REQUIREMENT4))
    {
        return 0U;
    }
    if ((mode == WIRED_CONTROL_MODE_REQUIREMENT4) &&
        (target_div != 2U) && (target_div != 4U) &&
        (target_div != 6U) && (target_div != 8U))
    {
        return 0U;
    }

    WiredControl_ResetPi(&s_control.frequency_pi);
    WiredControl_ResetPi(&s_control.phase_pi);
    WiredControl_ResetPi(&s_control.amplitude_pi);
    (void)memset(
        &s_control.result, 0, sizeof(s_control.result));
    (void)memset(
        &s_control.pending_command, 0,
        sizeof(s_control.pending_command));

    target_vpp_mv = (uint32_t)target_div * 500UL;
    s_control.result.mode = mode;
    s_control.result.stage = WIRED_CONTROL_STAGE_WAIT_INPUT;
    s_control.result.target_output_vpp_mv = target_vpp_mv;
    s_control.start_ms = now_ms;
    s_control.last_update_ms = now_ms;
    s_control.verify_start_ms = 0U;
    s_control.frequency_good_count = 0U;
    s_control.amplitude_good_count = 0U;
    s_control.phase_good_count = 0U;
    s_control.phase_cal_sample_count = 0U;
    s_control.phase_cal_retry_count = 0U;
    s_control.phase_recal_bad_count = 0U;
    s_control.phase_frequency_change_count = 0U;
    s_control.phase_cal_last_block_count = 0U;
    s_control.frequency_control_last_sample_count = 0U;
    s_control.frequency_control_last_update_ms = now_ms;
    s_control.phase_control_last_sample_count = 0U;
    s_control.phase_control_last_update_ms = now_ms;
    s_control.low_frequency_profile_active = 0U;
    s_control.low_frequency_profile_switch_count = 0U;
    s_control.phase_cal_sin_sum = 0.0f;
    s_control.phase_cal_cos_sum = 0.0f;
    s_control.phase_calibrated_frequency_hz = 0.0f;
    s_control.filtered_input_vpp_mv = 0.0f;
    s_control.input_baseline_vpp_mv = 0.0f;
    s_control.filtered_output_vpp_mv = 0.0f;
    s_control.amplitude_sample_dt_s =
        (float)s_control.config.update_period_ms * 0.001f;
    s_control.amplitude_last_block_count = 0U;
    s_control.amplitude_last_update_ms = now_ms;
    s_control.amplitude_refeed_candidate_vpp_mv = 0U;
    s_control.amplitude_refeed_count = 0U;
    s_control.amplitude_filter_ready = 0U;
    s_control.amplitude_new_sample = 0U;
    s_control.output_clip_bad_count = 0U;
    s_control.output_clip_good_count = 0U;
    s_control.amplitude_settle_start_ms = 0U;
    s_control.command_pending = 0U;
    s_control.clip_active = 0U;
    s_control.running = 1U;
    s_control.result.phase_calibration_state =
        WIRED_PHASE_CAL_IDLE;
    s_control.result.phase_calibrated = 0U;
    s_control.result.phase_calibration_offset_deg = 0.0f;
    s_control.result.amplitude_feedback_valid = 0U;
    s_control.result.amplitude_refeed_active = 0U;
    s_control.amplitude_target_fixed =
        (mode == WIRED_CONTROL_MODE_REQUIREMENT4) ? 1U : 0U;
    s_control.input_baseline_ready = 0U;
    WiredControl_UpdatePhaseTargets();

    /*
     * 要求4的目标固定，可直接根据8.5倍输出链路算前馈。
     * 要求1/2/3先用8div的471mV安全初值，等检测到输入幅值后再更新。
     */
    if (mode == WIRED_CONTROL_MODE_REQUIREMENT4)
    {
        s_control.amplitude_feedforward_mv =
            (float)target_vpp_mv /
            s_control.config.output_chain_gain;
        s_control.amplitude_feedforward_ready = 1U;
    }
    else
    {
        s_control.amplitude_feedforward_mv = 471.0f;
        s_control.amplitude_feedforward_ready = 0U;
    }
    return 1U;
}

void wired_control_stop(void)
{
    WiredControlConfig_t saved_config;

    saved_config = s_control.config;
    (void)memset(&s_control, 0, sizeof(s_control));
    s_control.config = saved_config;
    s_control.result.stage = WIRED_CONTROL_STAGE_STOPPED;
}

uint8_t wired_control_set_amplitude_target_div(
    uint8_t target_div,
    uint32_t now_ms)
{
    uint32_t target_vpp_mv;

    if ((s_control.running == 0U) ||
        ((target_div != 2U) && (target_div != 4U) &&
         (target_div != 6U) && (target_div != 8U)))
    {
        return 0U;
    }

    target_vpp_mv = (uint32_t)target_div * 500UL;
    s_control.amplitude_target_fixed = 1U;
    s_control.input_baseline_ready = 0U;
    s_control.result.target_output_vpp_mv = target_vpp_mv;
    s_control.amplitude_feedforward_mv = WiredControl_ClampFloat(
        (float)target_vpp_mv / s_control.config.output_chain_gain,
        s_control.config.dds_amplitude_min_mv,
        s_control.config.dds_amplitude_max_mv);
    s_control.amplitude_feedforward_ready = 1U;
    s_control.amplitude_refeed_count = 0U;
    s_control.amplitude_refeed_candidate_vpp_mv = 0U;
    s_control.result.amplitude_refeed_active = 1U;
    s_control.amplitude_settle_start_ms = now_ms;
    s_control.amplitude_good_count = 0U;
    s_control.verify_start_ms = 0U;
    s_control.result.locked = 0U;
    WiredControl_ResetPi(&s_control.amplitude_pi);
    s_control.result.amplitude_pi_correction_mv = 0.0f;

    if ((s_control.result.stage >= WIRED_CONTROL_STAGE_AMPLITUDE) &&
        (s_control.result.stage <= WIRED_CONTROL_STAGE_HOLD))
    {
        WiredControl_SetStage(WIRED_CONTROL_STAGE_AMPLITUDE);
    }
    return 1U;
}

uint8_t wired_control_is_amplitude_target_fixed(void)
{
    return (s_control.amplitude_target_fixed != 0U) ? 1U : 0U;
}

static uint32_t WiredControl_RoundPositive(float value)
{
    return (value <= 0.0f) ? 0U : (uint32_t)(value + 0.5f);
}

static void WiredControl_UpdateAmplitudeFeedforward(
    const WiredControlInput_t *input)
{
    uint32_t desired_vpp_mv;
    uint32_t feedforward_vpp_mv;
    uint32_t candidate_difference_mv;
    uint32_t elapsed_ms;
    float new_feedforward_mv;
    uint8_t reference_valid;

    s_control.amplitude_new_sample = 0U;
    if (input->adc_block_count !=
        s_control.amplitude_last_block_count)
    {
        s_control.amplitude_last_block_count =
            input->adc_block_count;
        s_control.amplitude_new_sample = 1U;
        elapsed_ms = (uint32_t)(
            input->now_ms - s_control.amplitude_last_update_ms);
        s_control.amplitude_last_update_ms = input->now_ms;
        if (elapsed_ms == 0U)
        {
            elapsed_ms = s_control.config.update_period_ms;
        }
        s_control.amplitude_sample_dt_s =
            (float)elapsed_ms * 0.001f;
        if (s_control.amplitude_sample_dt_s > 0.100f)
        {
            s_control.amplitude_sample_dt_s = 0.100f;
        }

        if (input->output_clipped != 0U)
        {
            s_control.output_clip_good_count = 0U;
            if (s_control.output_clip_bad_count < 255U)
            {
                ++s_control.output_clip_bad_count;
            }
            if ((s_control.clip_active == 0U) &&
                (s_control.output_clip_bad_count >=
                 s_control.config.output_clip_assert_count))
            {
                s_control.clip_active = 1U;
                WiredControl_ResetPi(&s_control.amplitude_pi);
                s_control.result.amplitude_pi_correction_mv = 0.0f;
                s_control.amplitude_feedforward_mv *=
                    WIRED_CONTROL_CLIP_REDUCTION;
                s_control.amplitude_feedforward_mv =
                    WiredControl_ClampFloat(
                        s_control.amplitude_feedforward_mv,
                        s_control.config.dds_amplitude_min_mv,
                        s_control.config.dds_amplitude_max_mv);
                s_control.result.amplitude_refeed_active = 1U;
                s_control.amplitude_settle_start_ms =
                    input->now_ms;
                s_control.result.locked = 0U;
            }
        }
        else
        {
            s_control.output_clip_bad_count = 0U;
            if (s_control.clip_active != 0U)
            {
                if (s_control.output_clip_good_count < 255U)
                {
                    ++s_control.output_clip_good_count;
                }
                if (s_control.output_clip_good_count >=
                    s_control.config.output_clip_release_count)
                {
                    s_control.output_clip_good_count = 0U;
                    s_control.clip_active = 0U;
                }
            }
        }
    }

    s_control.result.amplitude_feedback_valid =
        ((WiredControl_AmplitudeFeedbackValid(input) != 0U) &&
         (s_control.clip_active == 0U)) ? 1U : 0U;
    reference_valid = (s_control.amplitude_target_fixed != 0U) ? 1U :
        WiredControl_InputAmplitudeReferenceValid(input);

    if ((s_control.amplitude_target_fixed == 0U) &&
        (reference_valid != 0U) &&
        (s_control.amplitude_new_sample != 0U))
    {
        if (s_control.filtered_input_vpp_mv <= 0.0f)
        {
            s_control.filtered_input_vpp_mv =
                (float)input->input_vpp_mv;
        }
        else
        {
            s_control.filtered_input_vpp_mv +=
                s_control.config.amplitude_filter_alpha *
                ((float)input->input_vpp_mv -
                 s_control.filtered_input_vpp_mv);
        }
        s_control.result.target_output_vpp_mv =
            WiredControl_RoundPositive(
                s_control.filtered_input_vpp_mv);
    }

    if ((s_control.result.amplitude_feedback_valid != 0U) &&
        (s_control.amplitude_new_sample != 0U))
    {
        if (s_control.amplitude_filter_ready == 0U)
        {
            s_control.filtered_output_vpp_mv =
                (float)input->output_vpp_mv;
            s_control.amplitude_filter_ready = 1U;
        }
        else
        {
            s_control.filtered_output_vpp_mv +=
                s_control.config.amplitude_filter_alpha *
                ((float)input->output_vpp_mv -
                 s_control.filtered_output_vpp_mv);
        }
    }

    if (s_control.result.amplitude_refeed_active != 0U)
    {
        if ((uint32_t)(input->now_ms -
            s_control.amplitude_settle_start_ms) >=
            s_control.config.amplitude_settle_ms)
        {
            s_control.result.amplitude_refeed_active = 0U;
        }
    }

    if (s_control.result.amplitude_feedback_valid == 0U)
    {
        s_control.result.locked = 0U;
    }

    /* 前馈确认只消费新ADC块，不能重复计算同一组测量。 */
    if ((s_control.amplitude_new_sample == 0U) ||
        (reference_valid == 0U) ||
        (input->output_clipped != 0U) ||
        (s_control.clip_active != 0U))
    {
        if ((reference_valid == 0U) ||
            (input->output_clipped != 0U) ||
            (s_control.clip_active != 0U))
        {
            s_control.amplitude_refeed_count = 0U;
            s_control.amplitude_refeed_candidate_vpp_mv = 0U;
        }
        return;
    }

    desired_vpp_mv = s_control.result.target_output_vpp_mv;
    if (desired_vpp_mv == 0U)
    {
        return;
    }

    /* 记录用户最后一次手动档位时的输入幅值基准。 */
    if ((s_control.amplitude_new_sample != 0U) &&
        (WiredControl_InputAmplitudeReferenceValid(input) != 0U))
    {
        if (s_control.input_baseline_ready == 0U)
        {
            s_control.input_baseline_vpp_mv =
                (float)input->input_vpp_mv;
            s_control.input_baseline_ready = 1U;
        }
        else if ((s_control.amplitude_target_fixed != 0U) &&
                 (WiredControl_AbsDiffU32(
                     (uint32_t)WiredControl_RoundPositive(
                         s_control.input_baseline_vpp_mv),
                     input->input_vpp_mv) >=
                  WIRED_CONTROL_REFEED_TRIGGER_MV))
        {
            /* 输入大幅变化时，菜单2先自动恢复等幅跟踪。 */
            s_control.amplitude_target_fixed = 0U;
            s_control.amplitude_feedforward_ready = 0U;
            s_control.amplitude_refeed_count = 0U;
            s_control.amplitude_refeed_candidate_vpp_mv = 0U;
            s_control.input_baseline_vpp_mv =
                (float)input->input_vpp_mv;
            WiredControl_ResetPi(&s_control.amplitude_pi);
            s_control.result.amplitude_pi_correction_mv = 0.0f;
            s_control.result.locked = 0U;
            s_control.verify_start_ms = 0U;
        }
    }

    /* 固定分度模式只接受菜单命令，不能被输入幅值变化覆盖。 */
    if (s_control.amplitude_target_fixed != 0U)
    {
        s_control.amplitude_refeed_count = 0U;
        s_control.amplitude_refeed_candidate_vpp_mv = 0U;
        return;
    }

    /*
     * 菜单1的小幅波动继续交给PI处理；输入幅值稳定变化超过200mV时，
     * 重新计算前馈，使4Vpp到3Vpp这类变化不受PI输出范围限制。
     */
    if (s_control.amplitude_feedforward_ready != 0U)
    {
        feedforward_vpp_mv = WiredControl_RoundPositive(
            s_control.amplitude_feedforward_mv *
            s_control.config.output_chain_gain);
        if (WiredControl_AbsDiffU32(
                desired_vpp_mv, feedforward_vpp_mv) <
            WIRED_CONTROL_REFEED_TRIGGER_MV)
        {
            s_control.amplitude_refeed_count = 0U;
            s_control.amplitude_refeed_candidate_vpp_mv = 0U;
            return;
        }
    }

    if (s_control.amplitude_refeed_count == 0U)
    {
        s_control.amplitude_refeed_candidate_vpp_mv =
            desired_vpp_mv;
        s_control.amplitude_refeed_count = 1U;
        return;
    }

    candidate_difference_mv = WiredControl_AbsDiffU32(
        desired_vpp_mv,
        s_control.amplitude_refeed_candidate_vpp_mv);
    if (candidate_difference_mv >
        WIRED_CONTROL_REFEED_STABILITY_MV)
    {
        s_control.amplitude_refeed_candidate_vpp_mv =
            desired_vpp_mv;
        s_control.amplitude_refeed_count = 1U;
        return;
    }

    s_control.amplitude_refeed_candidate_vpp_mv =
        WiredControl_RoundPositive(
            0.75f *
            (float)s_control.amplitude_refeed_candidate_vpp_mv +
            0.25f * (float)desired_vpp_mv);
    if (s_control.amplitude_refeed_count < 255U)
    {
        ++s_control.amplitude_refeed_count;
    }

    if (s_control.amplitude_refeed_count <
        s_control.config.amplitude_refeed_count)
    {
        return;
    }

    desired_vpp_mv =
        s_control.amplitude_refeed_candidate_vpp_mv;
    new_feedforward_mv =
        (float)desired_vpp_mv /
        s_control.config.output_chain_gain;
    new_feedforward_mv = WiredControl_ClampFloat(
        new_feedforward_mv,
        s_control.config.dds_amplitude_min_mv,
        s_control.config.dds_amplitude_max_mv);
    s_control.amplitude_feedforward_mv = new_feedforward_mv;
    s_control.amplitude_feedforward_ready = 1U;
    s_control.amplitude_refeed_count = 0U;
    s_control.amplitude_refeed_candidate_vpp_mv = 0U;
    s_control.result.amplitude_refeed_active = 1U;
    s_control.amplitude_settle_start_ms = input->now_ms;

    /*
     * 反馈有效时保留原PI修正，只替换前馈基准；这样真实目标变化会按
     * 前馈量响应，同时不会因为清零PI额外产生一次反向阶跃。
     */
    if (s_control.result.amplitude_feedback_valid == 0U)
    {
        WiredControl_ResetPi(&s_control.amplitude_pi);
        s_control.result.amplitude_pi_correction_mv = 0.0f;
    }
    s_control.result.locked = 0U;
    s_control.verify_start_ms = 0U;
}

static void WiredControl_ProcessPhaseCalibration(
    const WiredControlInput_t *input)
{
    float mean_error_deg;
    float resultant;
    uint8_t capture_locked;

    /*
     * ADC phase calibration is meaningful only after the output amplitude
     * has settled. Otherwise comparator time walk or a low-SNR DFT can
     * establish a repeatable but wrong phase reference.
     */
    if ((s_control.result.amplitude_feedback_valid == 0U) ||
        (s_control.result.amplitude_refeed_active != 0U) ||
        (s_control.amplitude_refeed_count != 0U) ||
        (WiredControl_AbsInt32(
            s_control.result.amplitude_error_mv) >
         s_control.config.amplitude_lock_mv))
    {
        WiredControl_SetStage(WIRED_CONTROL_STAGE_AMPLITUDE);
        return;
    }

    capture_locked =
        ((input->phase_valid != 0U) &&
         (WiredControl_AbsFloat(
             s_control.result.phase_error_deg) <=
          s_control.config.phase_lock_deg)) ? 1U : 0U;

    switch ((WiredPhaseCalibrationState_t)
            s_control.result.phase_calibration_state)
    {
        case WIRED_PHASE_CAL_CAPTURE_LOCK:
            if (capture_locked != 0U)
            {
                if (s_control.phase_good_count < 255U)
                {
                    ++s_control.phase_good_count;
                }
            }
            else
            {
                s_control.phase_good_count = 0U;
            }
            if (s_control.phase_good_count >=
                s_control.config.phase_lock_count)
            {
                s_control.phase_good_count = 0U;
                s_control.result.phase_calibration_state =
                    WIRED_PHASE_CAL_SETTLE;
                s_control.phase_cal_state_start_ms =
                    input->now_ms;
            }
            break;

        case WIRED_PHASE_CAL_SETTLE:
            if ((input->phase_valid == 0U) ||
                (WiredControl_AbsFloat(
                    s_control.result.phase_error_deg) >
                 s_control.config.phase_release_deg))
            {
                s_control.result.phase_calibration_state =
                    WIRED_PHASE_CAL_CAPTURE_LOCK;
                s_control.phase_good_count = 0U;
            }
            else if ((uint32_t)(input->now_ms -
                     s_control.phase_cal_state_start_ms) >=
                     s_control.config.phase_cal_settle_ms)
            {
                s_control.result.phase_calibration_state =
                    WIRED_PHASE_CAL_COLLECT;
                s_control.phase_cal_last_block_count =
                    input->adc_block_count;
                WiredControl_ResetPhaseSamples(input->now_ms);
            }
            break;

        case WIRED_PHASE_CAL_COLLECT:
            (void)WiredControl_CollectAnalogPhase(input);
            if ((s_control.phase_cal_sample_count >=
                 s_control.config.phase_cal_sample_count) &&
                ((uint32_t)(input->now_ms -
                  s_control.phase_cal_state_start_ms) >=
                 s_control.config.phase_cal_min_time_ms) &&
                (WiredControl_GetAnalogPhaseMean(
                    &mean_error_deg, &resultant) != 0U))
            {
                if (resultant <
                    s_control.config.phase_cal_min_resultant)
                {
                    s_control.phase_cal_last_block_count =
                        input->adc_block_count;
                    WiredControl_ResetPhaseSamples(
                        input->now_ms);
                }
                else
                {
                    (void)WiredControl_ApplyPhaseCalibration(
                        mean_error_deg, input->now_ms);
                }
            }
            break;

        case WIRED_PHASE_CAL_RELOCK:
            if (capture_locked != 0U)
            {
                if (s_control.phase_good_count < 255U)
                {
                    ++s_control.phase_good_count;
                }
            }
            else
            {
                s_control.phase_good_count = 0U;
            }
            if (s_control.phase_good_count >=
                s_control.config.phase_lock_count)
            {
                s_control.phase_good_count = 0U;
                s_control.result.phase_calibration_state =
                    WIRED_PHASE_CAL_VERIFY;
                s_control.phase_cal_last_block_count =
                    input->adc_block_count;
                WiredControl_ResetPhaseSamples(input->now_ms);
            }
            break;

        case WIRED_PHASE_CAL_VERIFY:
            (void)WiredControl_CollectAnalogPhase(input);
            if ((s_control.phase_cal_sample_count >=
                 s_control.config.phase_cal_verify_count) &&
                ((uint32_t)(input->now_ms -
                  s_control.phase_cal_state_start_ms) >=
                 s_control.config.phase_cal_min_time_ms) &&
                (WiredControl_GetAnalogPhaseMean(
                    &mean_error_deg, &resultant) != 0U))
            {
                s_control.result.analog_phase_error_deg =
                    mean_error_deg;
                if (resultant <
                    s_control.config.phase_cal_min_resultant)
                {
                    s_control.phase_cal_last_block_count =
                        input->adc_block_count;
                    WiredControl_ResetPhaseSamples(
                        input->now_ms);
                }
                else if (WiredControl_AbsFloat(mean_error_deg) <=
                         s_control.config.phase_cal_verify_deg)
                {
                    s_control.result.phase_calibrated = 1U;
                    s_control.result.phase_calibration_state =
                        WIRED_PHASE_CAL_DONE;
                    s_control.phase_calibrated_frequency_hz =
                        input->input_frequency_hz;
                    WiredControl_SetStage(
                        WIRED_CONTROL_STAGE_VERIFY);
                }
                else if (s_control.phase_cal_retry_count <
                         s_control.config.phase_cal_retry_limit)
                {
                    ++s_control.phase_cal_retry_count;
                    (void)WiredControl_ApplyPhaseCalibration(
                        mean_error_deg, input->now_ms);
                }
                else
                {
                    s_control.result.phase_calibration_state =
                        WIRED_PHASE_CAL_FAILED;
                    s_control.result.phase_calibrated = 0U;
                }
            }
            break;

        case WIRED_PHASE_CAL_IDLE:
            WiredControl_BeginPhaseCalibration(input->now_ms);
            break;

        default:
            break;
    }
}

uint8_t wired_control_step(const WiredControlInput_t *input)
{
    float dt_s;
    float frequency_dt_s;
    float phase_dt_s;
    float target_frequency_hz;
    float low_frequency_gain_scale;
    float frequency_correction_hz;
    float command_amplitude_mv;
    WiredPiConfig_t scheduled_frequency_config;
    WiredPiConfig_t scheduled_phase_config;
    const WiredPiConfig_t *frequency_config;
    const WiredPiConfig_t *phase_config;
    uint32_t sample_elapsed_ms;
    uint8_t profile_changed;
    uint8_t needs_phase;

    if ((input == NULL) || (s_control.running == 0U))
    {
        return 0U;
    }
    if ((uint32_t)(input->now_ms - s_control.last_update_ms) <
        s_control.config.update_period_ms)
    {
        return 0U;
    }

    dt_s = (float)(uint32_t)(
        input->now_ms - s_control.last_update_ms) * 0.001f;
    if (dt_s > 0.100f)
    {
        /* 长时间没有测量时，防止积分器一次积累过多。 */
        dt_s = 0.100f;
    }
    s_control.last_update_ms = input->now_ms;
    s_control.result.control_elapsed_ms =
        (uint32_t)(input->now_ms - s_control.start_ms);
    s_control.result.fault = 0U;
    needs_phase =
        (s_control.result.mode ==
         WIRED_CONTROL_MODE_REQUIREMENT4) ? 0U : 1U;

    if (input->input_frequency_valid == 0U)
    {
        WiredControl_SetStage(WIRED_CONTROL_STAGE_WAIT_INPUT);
        return 1U;
    }

    target_frequency_hz =
        WiredControl_FrequencyRatio(s_control.result.mode) *
        input->input_frequency_hz;
    if ((target_frequency_hz <
         WIRED_CONTROL_MIN_FREQUENCY_HZ) ||
        (target_frequency_hz >
         WIRED_CONTROL_MAX_FREQUENCY_HZ))
    {
        s_control.result.stage = WIRED_CONTROL_STAGE_FAULT;
        s_control.result.fault = 1U;
        s_control.result.locked = 0U;
        return 1U;
    }

    profile_changed =
        WiredControl_UpdateFrequencyProfile(target_frequency_hz);
    frequency_config = (s_control.low_frequency_profile_active != 0U) ?
        &s_control.config.frequency_pi_low : &s_control.config.frequency_pi;
    phase_config = (s_control.low_frequency_profile_active != 0U) ?
        &s_control.config.phase_pi_low : &s_control.config.phase_pi;
    if (profile_changed != 0U)
    {
        /* Do not carry the integral accumulated for the other frequency band. */
        WiredControl_ResetPi(&s_control.frequency_pi);
        WiredControl_ResetPi(&s_control.phase_pi);
        s_control.result.frequency_pi_correction_hz = 0.0f;
        s_control.result.phase_pi_correction_hz = 0.0f;
    }
    if (s_control.low_frequency_profile_active != 0U)
    {
        low_frequency_gain_scale = WiredControl_ClampFloat(
            target_frequency_hz /
            WIRED_CONTROL_FREQUENCY_PROFILE_BOUNDARY_HZ,
            WIRED_CONTROL_LOW_GAIN_SCALE_MIN, 1.0f);
        scheduled_frequency_config = *frequency_config;
        scheduled_frequency_config.kp *= low_frequency_gain_scale;
        scheduled_frequency_config.ki *=
            low_frequency_gain_scale * low_frequency_gain_scale;
        scheduled_frequency_config.kd *= low_frequency_gain_scale;
        frequency_config = &scheduled_frequency_config;

        scheduled_phase_config = *phase_config;
        scheduled_phase_config.kp *= low_frequency_gain_scale;
        scheduled_phase_config.ki *=
            low_frequency_gain_scale * low_frequency_gain_scale;
        scheduled_phase_config.kd *= low_frequency_gain_scale;
        phase_config = &scheduled_phase_config;
    }

    if (s_control.result.stage ==
        WIRED_CONTROL_STAGE_WAIT_INPUT)
    {
        WiredControl_SetStage(WIRED_CONTROL_STAGE_FREQUENCY);
    }

    WiredControl_UpdateAmplitudeFeedforward(input);

    if (input->feedback_frequency_valid != 0U)
    {
        s_control.result.frequency_error_hz =
            input->feedback_frequency_hz -
            target_frequency_hz;
    }
    else
    {
        s_control.result.frequency_error_hz = 0.0f;
    }
    s_control.result.amplitude_error_mv =
        (int32_t)WiredControl_RoundPositive(
            (s_control.amplitude_filter_ready != 0U) ?
            s_control.filtered_output_vpp_mv :
            (float)input->output_vpp_mv) -
        (int32_t)s_control.result.target_output_vpp_mv;
    s_control.result.phase_error_deg = input->phase_error_deg;
    s_control.result.analog_phase_error_deg =
        input->analog_phase_error_deg;

    if ((needs_phase != 0U) &&
        (s_control.result.phase_calibrated != 0U))
    {
        float frequency_recal_hz = WiredControl_MaxFloat(
            2.0f, input->input_frequency_hz * 0.0001f);

        if (WiredControl_AbsFloat(
                input->input_frequency_hz -
                s_control.phase_calibrated_frequency_hz) >
            frequency_recal_hz)
        {
            if (s_control.phase_frequency_change_count < 255U)
            {
                ++s_control.phase_frequency_change_count;
            }
        }
        else
        {
            s_control.phase_frequency_change_count = 0U;
        }
        if (s_control.phase_frequency_change_count >= 5U)
        {
            s_control.phase_frequency_change_count = 0U;
            WiredControl_SetStage(
                WIRED_CONTROL_STAGE_FREQUENCY);
        }
    }

    /*
     * 频率粗环只在FREQUENCY阶段积分。进入相位环后保留其最后输出，
     * 防止频率PI和相位PI同时争夺同一个DDS频率控制量。
     */
    if ((s_control.result.stage ==
         WIRED_CONTROL_STAGE_FREQUENCY) &&
        (input->feedback_frequency_valid != 0U) &&
        (input->feedback_sample_count !=
         s_control.frequency_control_last_sample_count))
    {
        s_control.frequency_control_last_sample_count =
            input->feedback_sample_count;
        sample_elapsed_ms = (uint32_t)(
            input->now_ms - s_control.frequency_control_last_update_ms);
        s_control.frequency_control_last_update_ms = input->now_ms;
        if (sample_elapsed_ms == 0U)
        {
            sample_elapsed_ms = s_control.config.update_period_ms;
        }
        frequency_dt_s = (float)sample_elapsed_ms * 0.001f;
        if (frequency_dt_s > 0.100f)
        {
            frequency_dt_s = 0.100f;
        }
        s_control.result.frequency_pi_correction_hz =
            WiredControl_RunPi(
                &s_control.frequency_pi,
                frequency_config,
                s_control.result.frequency_error_hz,
                frequency_dt_s,
                1.0f,
                0U);

        if (WiredControl_AbsFloat(
                s_control.result.frequency_error_hz) <=
            s_control.config.frequency_lock_hz)
        {
            if (s_control.frequency_good_count < 255U)
            {
                ++s_control.frequency_good_count;
            }
        }
        else
        {
            s_control.frequency_good_count = 0U;
        }

        if (s_control.frequency_good_count >=
            s_control.config.frequency_lock_count)
        {
            /*
             * Phase modes also pass through AMPLITUDE. That stage already
             * requires several consecutive good ADC blocks before routing
             * requirement 1/2/3 onward to PHASE.
             */
            WiredControl_SetStage(
                WIRED_CONTROL_STAGE_AMPLITUDE);
        }
    }

    /*
     * 幅值PI在得到可靠双ADC幅值后即可工作。即使当前处于频率阶段，
     * 也允许幅值同步靠近目标，以缩短总收敛时间。
     */
    if ((s_control.result.amplitude_feedback_valid != 0U) &&
        (s_control.result.amplitude_refeed_active == 0U) &&
        (s_control.amplitude_refeed_count == 0U) &&
        (s_control.amplitude_new_sample != 0U) &&
        (WiredControl_AbsInt32(
            s_control.result.amplitude_error_mv) >
         s_control.config.amplitude_control_deadband_mv))
    {
        s_control.result.amplitude_pi_correction_mv =
            WiredControl_RunPi(
                &s_control.amplitude_pi,
                &s_control.config.amplitude_pi,
                (float)s_control.result.amplitude_error_mv,
                s_control.amplitude_sample_dt_s,
                1.0f,
                0U);
    }

    if (s_control.result.stage ==
        WIRED_CONTROL_STAGE_AMPLITUDE)
    {
        if (s_control.amplitude_new_sample != 0U)
        {
            if ((s_control.result.amplitude_feedback_valid != 0U) &&
                (s_control.result.amplitude_refeed_active == 0U) &&
                (s_control.amplitude_refeed_count == 0U) &&
                (WiredControl_AbsInt32(
                    s_control.result.amplitude_error_mv) <=
                 s_control.config.amplitude_lock_mv))
            {
                if (s_control.amplitude_good_count < 255U)
                {
                    ++s_control.amplitude_good_count;
                }
            }
            else
            {
                s_control.amplitude_good_count = 0U;
            }
        }

        if (s_control.amplitude_good_count >=
            s_control.config.amplitude_lock_count)
        {
            WiredControl_SetStage(
                (needs_phase != 0U) ?
                WIRED_CONTROL_STAGE_PHASE :
                WIRED_CONTROL_STAGE_VERIFY);
        }
    }

    /*
     * 相位环是DPLL：相位误差经过PI后形成Hz级频率微调。
     * 要求1/2/3一进入幅值稳定阶段就保持运行；幅值门限只约束
     * ADC最终目标校准，不能关闭TIM2内环，否则会表现为闭环失效。
     */
    if ((needs_phase != 0U) &&
        ((s_control.result.stage ==
          WIRED_CONTROL_STAGE_AMPLITUDE) ||
         (s_control.result.stage ==
          WIRED_CONTROL_STAGE_PHASE) ||
        (s_control.result.stage ==
          WIRED_CONTROL_STAGE_VERIFY) ||
         (s_control.result.stage ==
          WIRED_CONTROL_STAGE_HOLD)) &&
        (input->phase_valid != 0U) &&
        (input->phase_sample_count !=
         s_control.phase_control_last_sample_count))
    {
        s_control.phase_control_last_sample_count =
            input->phase_sample_count;
        sample_elapsed_ms = (uint32_t)(
            input->now_ms - s_control.phase_control_last_update_ms);
        s_control.phase_control_last_update_ms = input->now_ms;
        if (sample_elapsed_ms == 0U)
        {
            sample_elapsed_ms = s_control.config.update_period_ms;
        }
        phase_dt_s = (float)sample_elapsed_ms * 0.001f;
        if (phase_dt_s > 0.100f)
        {
            phase_dt_s = 0.100f;
        }
        s_control.result.phase_pi_correction_hz =
            WiredControl_RunPi(
                &s_control.phase_pi,
                phase_config,
                s_control.result.phase_error_deg,
                phase_dt_s,
                1.0f,
                1U);
    }

    if ((needs_phase != 0U) &&
        (s_control.result.stage ==
         WIRED_CONTROL_STAGE_PHASE))
    {
        WiredControl_ProcessPhaseCalibration(input);
    }

    if ((needs_phase != 0U) &&
        (s_control.result.stage ==
         WIRED_CONTROL_STAGE_HOLD) &&
        (s_control.result.phase_calibrated != 0U) &&
        (s_control.result.amplitude_feedback_valid != 0U) &&
        (input->analog_phase_valid != 0U) &&
        (input->adc_block_count !=
         s_control.phase_cal_last_block_count))
    {
        s_control.phase_cal_last_block_count =
            input->adc_block_count;
        if (WiredControl_AbsFloat(
                input->analog_phase_error_deg) >
            s_control.config.phase_release_deg)
        {
            if (s_control.phase_recal_bad_count < 255U)
            {
                ++s_control.phase_recal_bad_count;
            }
        }
        else
        {
            s_control.phase_recal_bad_count = 0U;
        }
        if (s_control.phase_recal_bad_count >=
            s_control.config.phase_recal_release_count)
        {
            WiredControl_SetStage(
                WIRED_CONTROL_STAGE_PHASE);
        }
    }

    /*
     * 后级状态一旦发现明显失频，立即退回频率粗锁并清空相位积分。
     * 幅值失锁则退回幅值环；HOLD状态也执行同样保护。
     */
    if ((s_control.result.stage ==
         WIRED_CONTROL_STAGE_HOLD) &&
        (s_control.result.amplitude_feedback_valid == 0U))
    {
        WiredControl_SetStage(WIRED_CONTROL_STAGE_VERIFY);
    }
    else if ((s_control.result.stage >
         WIRED_CONTROL_STAGE_FREQUENCY) &&
        (input->feedback_frequency_valid != 0U) &&
        (WiredControl_AbsFloat(
            s_control.result.frequency_error_hz) >
         s_control.config.frequency_release_hz))
    {
        WiredControl_SetStage(WIRED_CONTROL_STAGE_FREQUENCY);
    }
    else if ((s_control.result.stage >
              WIRED_CONTROL_STAGE_AMPLITUDE) &&
             (s_control.result.amplitude_feedback_valid != 0U) &&
             (s_control.result.amplitude_refeed_active == 0U) &&
             (WiredControl_AbsInt32(
                s_control.result.amplitude_error_mv) >
              s_control.config.amplitude_release_mv))
    {
        /* Keep the DPLL alive while the independent amplitude loop recovers. */
        WiredControl_SetStage(
            (needs_phase != 0U) ?
            WIRED_CONTROL_STAGE_VERIFY :
            WIRED_CONTROL_STAGE_AMPLITUDE);
    }
    else if ((needs_phase != 0U) &&
             (s_control.result.stage >
              WIRED_CONTROL_STAGE_PHASE) &&
             (input->phase_valid != 0U) &&
             (WiredControl_AbsFloat(
                s_control.result.phase_error_deg) >
              s_control.config.phase_release_deg))
    {
        WiredControl_SetStage(WIRED_CONTROL_STAGE_PHASE);
    }

    if (s_control.result.stage ==
        WIRED_CONTROL_STAGE_VERIFY)
    {
        if (WiredControl_AllErrorsLocked(input) != 0U)
        {
            if (s_control.verify_start_ms == 0U)
            {
                s_control.verify_start_ms = input->now_ms;
            }
            else if ((uint32_t)(
                input->now_ms - s_control.verify_start_ms) >=
                s_control.config.verify_time_ms)
            {
                s_control.result.stage =
                    WIRED_CONTROL_STAGE_HOLD;
                s_control.result.locked = 1U;
            }
        }
        else
        {
            s_control.verify_start_ms = 0U;
        }
    }

    frequency_correction_hz =
        s_control.result.frequency_pi_correction_hz +
        s_control.result.phase_pi_correction_hz;
    command_amplitude_mv =
        s_control.amplitude_feedforward_mv +
        s_control.result.amplitude_pi_correction_mv;
    WiredControl_QueueCommand(
        target_frequency_hz,
        frequency_correction_hz,
        command_amplitude_mv);
    return 1U;
}

const WiredControlResult_t *wired_control_get_result(void)
{
    return &s_control.result;
}

float wired_control_get_capture_phase_target_deg(void)
{
    return s_control.result.capture_phase_target_deg;
}

uint8_t wired_control_take_command(WiredControlCommand_t *command)
{
    if ((command == NULL) || (s_control.command_pending == 0U))
    {
        return 0U;
    }

    *command = s_control.pending_command;
    s_control.command_pending = 0U;
    return 1U;
}
