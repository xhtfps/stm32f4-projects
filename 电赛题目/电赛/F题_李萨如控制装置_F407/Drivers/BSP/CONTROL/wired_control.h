/**
 * @file    wired_control.h
 * @brief   F题要求1~4的有线闭环控制器。
 *
 * 本模块只负责控制算法和状态机，不直接访问ADC、定时器、AD9959或LCD。
 * 上层应用把PB10/PB11测频以及PA1/PA2测幅、测相结果送入本模块，再把
 * 本模块生成的频率、幅度和相位命令写入AD9959。
 *
 * 控制器分工：
 *   - 频率PI：消除 fout - N*fin 的静态频差；
 *   - 幅值PI：消除输出幅值误差，并连续稳定后才允许相位校准；
 *   - 相位PI：作为数字锁相环，把相位误差转换为微小频率修正；
 *   - 状态机：依次完成频率、幅值、相位、严格验证和连续保持。
 *
 * 要求5是纯视觉自动模式，不使用本模块。
 */

#ifndef WIRED_CONTROL_H
#define WIRED_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 控制器运行模式。 */
typedef enum
{
    WIRED_CONTROL_MODE_NONE = 0U,
    WIRED_CONTROL_MODE_REQUIREMENT1,
    WIRED_CONTROL_MODE_REQUIREMENT2,
    WIRED_CONTROL_MODE_REQUIREMENT3,
    WIRED_CONTROL_MODE_REQUIREMENT4
} WiredControlMode_t;

/** 有线控制的顺序状态。 */
typedef enum
{
    WIRED_CONTROL_STAGE_STOPPED = 0U,
    WIRED_CONTROL_STAGE_WAIT_INPUT,
    WIRED_CONTROL_STAGE_FREQUENCY,
    WIRED_CONTROL_STAGE_AMPLITUDE,
    WIRED_CONTROL_STAGE_PHASE,
    WIRED_CONTROL_STAGE_VERIFY,
    WIRED_CONTROL_STAGE_HOLD,
    WIRED_CONTROL_STAGE_FAULT
} WiredControlStage_t;

/** TIM2 DPLL锁定后，由ADC模拟相位执行一次性目标校准。 */
typedef enum
{
    WIRED_PHASE_CAL_IDLE = 0U,
    WIRED_PHASE_CAL_CAPTURE_LOCK,
    WIRED_PHASE_CAL_SETTLE,
    WIRED_PHASE_CAL_COLLECT,
    WIRED_PHASE_CAL_RELOCK,
    WIRED_PHASE_CAL_VERIFY,
    WIRED_PHASE_CAL_DONE,
    WIRED_PHASE_CAL_FAILED
} WiredPhaseCalibrationState_t;

/** 单个PI控制器的可调参数。 */
typedef struct
{
    float kp;                    /**< 比例系数。 */
    float ki;                    /**< 积分系数，单位按控制量/误差/秒。 */
    float kd;                    /**< 微分系数，单位按控制量*秒/误差。 */
    float derivative_filter_alpha; /**< 微分一阶滤波系数，范围0~1。 */
    float output_min;            /**< PI修正量下限。 */
    float output_max;            /**< PI修正量上限。 */
    float integral_separation;   /**< 超过该误差时暂停积分。 */
} WiredPiConfig_t;

/** Runtime-tunable control loops exposed to the serial tuning service. */
typedef enum
{
    WIRED_CONTROL_LOOP_FREQUENCY = 0U,
    WIRED_CONTROL_LOOP_FREQUENCY_LOW,
    WIRED_CONTROL_LOOP_PHASE,
    WIRED_CONTROL_LOOP_PHASE_LOW,
    WIRED_CONTROL_LOOP_AMPLITUDE
} WiredControlLoop_t;

/** 有线闭环的全部可调参数。 */
typedef struct
{
    WiredPiConfig_t frequency_pi;
    WiredPiConfig_t frequency_pi_low;
    WiredPiConfig_t phase_pi;
    WiredPiConfig_t phase_pi_low;
    WiredPiConfig_t amplitude_pi;

    uint32_t update_period_ms;       /**< PI最短更新周期。 */
    uint32_t verify_time_ms;         /**< 连续达标后进入HOLD的时间。 */
    uint32_t phase_cal_settle_ms;    /**< TIM2初锁后模拟链路稳定时间。 */
    uint32_t phase_cal_min_time_ms;  /**< ADC圆周平均的最短覆盖时间。 */
    uint32_t amplitude_settle_ms;    /**< 重算幅值前馈后的积分冻结时间。 */

    float frequency_lock_hz;         /**< 进入下一状态的频差阈值。 */
    float frequency_release_hz;      /**< HOLD/后级退回频率环的阈值。 */
    int32_t amplitude_lock_mv;       /**< 幅值达标阈值。 */
    int32_t amplitude_release_mv;    /**< 幅值失锁阈值。 */
    float phase_lock_deg;            /**< 相位达标阈值。 */
    float phase_release_deg;         /**< 相位失锁阈值。 */
    float phase_cal_verify_deg;      /**< ADC模拟相位校准验收阈值。 */
    float phase_cal_max_correction_deg; /**< 单次允许的最大校准量。 */
    float phase_cal_min_resultant;   /**< ADC圆周平均最小集中度。 */
    float amplitude_filter_alpha;    /**< 输入/输出Vpp一阶滤波系数。 */
    float amplitude_command_deadband_mv; /**< DDS幅值命令最小变化。 */
    int32_t amplitude_control_deadband_mv; /**< 幅值PI误差死区。 */

    uint8_t frequency_lock_count;    /**< 连续频率达标次数。 */
    uint8_t amplitude_lock_count;    /**< 连续幅值达标次数。 */
    uint8_t phase_lock_count;        /**< 连续相位达标次数。 */
    uint8_t phase_cal_sample_count;  /**< 首次ADC校准样本数。 */
    uint8_t phase_cal_verify_count;  /**< 校准后ADC验证样本数。 */
    uint8_t phase_cal_retry_limit;   /**< ADC残差允许的再次校准次数。 */
    uint8_t phase_recal_release_count; /**< HOLD模拟相位失锁计数。 */
    uint8_t amplitude_refeed_count;  /**< 幅值变化连续确认次数。 */
    uint8_t output_clip_assert_count; /**< 输出削顶连续确认次数。 */
    uint8_t output_clip_release_count; /**< 输出削顶连续恢复次数。 */

    float dds_amplitude_min_mv;      /**< AD9959幅度命令下限。 */
    float dds_amplitude_max_mv;      /**< AD9959幅度命令上限。 */
    float output_chain_gain;         /**< DDS至示波器Y轴的标称增益。 */
} WiredControlConfig_t;

/** 每次执行控制器时输入的最新测量值。 */
typedef struct
{
    uint32_t now_ms;
    float input_frequency_hz;
    float feedback_frequency_hz;
    uint32_t input_vpp_mv;
    uint32_t output_vpp_mv;
    float phase_error_deg;
    float analog_phase_error_deg;
    uint32_t adc_block_count;
    uint32_t feedback_sample_count;
    uint32_t phase_sample_count;
    uint8_t input_frequency_valid;
    uint8_t feedback_frequency_valid;
    uint8_t input_amplitude_valid;
    uint8_t amplitude_valid;
    uint8_t phase_valid;
    uint8_t analog_phase_valid;
    uint8_t input_clipped;
    uint8_t output_clipped;
    uint8_t clipped;
    uint8_t input_bias_valid;
    uint8_t output_bias_valid;
} WiredControlInput_t;

/** 需要写入AD9959的一组完整命令。 */
typedef struct
{
    uint32_t frequency_millihz;
    float amplitude_mv;
    uint32_t phase_millideg;
    uint8_t output_enable;
} WiredControlCommand_t;

/** 提供给应用层和调试界面的控制状态。 */
typedef struct
{
    WiredControlMode_t mode;
    WiredControlStage_t stage;
    uint32_t control_elapsed_ms;
    uint32_t target_output_vpp_mv;
    float frequency_error_hz;
    int32_t amplitude_error_mv;
    float phase_error_deg;
    float analog_phase_error_deg;
    float capture_phase_target_deg;
    float phase_calibration_offset_deg;
    float frequency_pi_correction_hz;
    float phase_pi_correction_hz;
    float amplitude_pi_correction_mv;
    uint32_t command_frequency_millihz;
    float command_amplitude_mv;
    uint32_t command_phase_millideg;
    uint8_t command_valid;
    uint8_t phase_calibration_state;
    uint8_t phase_calibrated;
    uint8_t amplitude_feedback_valid;
    uint8_t amplitude_refeed_active;
    uint8_t locked;
    uint8_t fault;
} WiredControlResult_t;

/** 获取一套保守的默认参数，台架调参应从此配置开始。 */
void WiredControlConfig_Default(WiredControlConfig_t *config);

/** 初始化控制服务。 */
void wired_control_init(const WiredControlConfig_t *config);

/** Read the active PID coefficients for one loop. */
uint8_t wired_control_get_pid_config(
    WiredControlLoop_t loop,
    WiredPiConfig_t *config);

/** Atomically replace Kp/Ki/Kd/filter alpha and reset that loop state. */
uint8_t wired_control_set_pid_config(
    WiredControlLoop_t loop,
    float kp,
    float ki,
    float kd,
    float derivative_filter_alpha);

/**
 * @brief 启动某个有线要求的闭环。
 * @param mode       要求1、2、3或4。
 * @param target_div 要求4的2/4/6/8div；其他模式传8。
 * @param now_ms     当前系统毫秒时刻。
 * @return 1表示成功，0表示参数非法。
 */
uint8_t wired_control_start(
    WiredControlMode_t mode,
    uint8_t target_div,
    uint32_t now_ms);

/**
 * @brief 运行中切换固定的Y轴峰峰值目标，不重启频率和相位控制。
 * @param target_div 目标峰峰值档位，只允许2/4/6/8div（0.5V/div）。
 * @param now_ms     当前系统毫秒时刻。
 * @return 1表示目标已更新，0表示闭环未运行或档位非法。
 */
uint8_t wired_control_set_amplitude_target_div(
    uint8_t target_div,
    uint32_t now_ms);

/** 返回当前是否仍处于菜单2的手动幅值目标。 */
uint8_t wired_control_is_amplitude_target_fixed(void);

/** 停止闭环并清空积分器。 */
void wired_control_stop(void);

/**
 * @brief 送入最新测量结果并执行一次状态机。
 * @return 1表示内部状态或控制命令有更新，0表示尚未到更新周期。
 */
uint8_t wired_control_step(const WiredControlInput_t *input);

/** 获取当前只读结果。 */
const WiredControlResult_t *wired_control_get_result(void);

/** 返回当前经ADC校准后的TIM2捕获相位目标。 */
float wired_control_get_capture_phase_target_deg(void);

/**
 * @brief 取走一条待执行DDS命令。
 *
 * 命令只会被取走一次，从而避免应用任务在控制周期之间重复写串行口。
 */
uint8_t wired_control_take_command(WiredControlCommand_t *command);

#ifdef __cplusplus
}
#endif

#endif /* WIRED_CONTROL_H */
