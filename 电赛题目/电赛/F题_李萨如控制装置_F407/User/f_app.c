/**
 * @file    f_app.c
 * @brief   F题分项应用状态机。
 *
 * 分项边界：
 *   要求1~4：信号源与装置连接，PB10测频，PA1/PA2双ADC测幅值和相位；
 *   要求5：装置输入必须与信号源断开，只等待PC视觉串口命令；
 *   调试菜单：回车保留PB10/PA1/PA2测试，数字键可独立测试AD9959 CH0。
 */

#include "f_app.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"

#include "./BSP/AD9959/Drive_AD9959.h"
#include "./BSP/DAC/DAC.h"
#include "./BSP/CONTROL/pid_serial.h"
#include "./BSP/CONTROL/wired_control.h"
#include "./BSP/FREQUENCY/frequency_meter.h"
#include "./BSP/MEASUREMENT/wired_measurement.h"
#include "./BSP/PHASE/phase_vofa.h"
#include "./BSP/LED/led.h"
#include "./BSP/CONTROL/vision_control.h"
#include "./BSP/VISION/vision_link.h"
#include "./BSP/VISION/vision_slave.h"

/* UI 数据发布的最小间隔（毫秒） */
#define F_APP_UI_PUBLISH_MS            100U
/* DDS 输出通道编号 */
#define F_APP_DDS_OUTPUT_CHANNEL       0U
/* 要求1/2/3的开环起振幅度，沿用9224864已验证值。 */
#define F_APP_WIRED_FULL_SCALE_MV      471.0f
/* 菜单6 DDS测试的安全默认幅度，放大8.5倍后约为1 Vpp。 */
#define F_APP_DDS_TEST_AMPLITUDE_MV    118.0f
/* 相邻FTW抖动每10ms选择一次输出频率字。 */
#define F_APP_DDS_DITHER_PERIOD_MS     10U
/* 混合比例使用固定16级，单级约等于0.364mHz（25MHz系统时钟）。 */
#define F_APP_DDS_DITHER_DENOMINATOR   16U
#define F_APP_DDS_DITHER_DEFAULT_NUM   8U
#define F_APP_PHASE_SAMPLE_TIMEOUT_MS 100U
#define F_APP_LOW_BAND_MAX_FREQUENCY_HZ 4500U
#define F_APP_LOW_BAND_CAPTURE_EDGES     4U
/* Repeat START until K230 answers with its first SET command. */
#define F_APP_VISION_START_RETRY_MS       300U

/* K230 $DONE completion output for the external sound/light circuit. */
#define F_APP_COMPLETE_GPIO_PORT           GPIOG
#define F_APP_COMPLETE_GPIO_PIN            GPIO_PIN_10

/* Temporary PA4 sawtooth test; set the enable macro to 0 after bench test. */
#define F_APP_DAC_SAWTOOTH_TEST_ENABLE  0U
#define F_APP_VISION_PC_MASTER_ENABLE   1U
#define F_APP_DAC_SAWTOOTH_TEST_FREQ_HZ 1000U
#define F_APP_DAC_SAWTOOTH_TEST_VPP     2.0

/* ---------- 静态变量 ---------- */

/** 测量结果发布回调函数，由UI注册 */
static FAppMeasurementPublisher_t s_publisher;
/** 当前测量数据结构，定期推送至UI */
static UI_Measurement_t s_measurement;
/** 上次发布数据的时刻，用于控制发布频率 */
static TickType_t s_last_publish_tick;
/** 模块是否已完成初始化 */
static uint8_t s_initialized;
/** 频率计是否正在运行 */
static uint8_t s_frequency_running;
/** 有线测量（双ADC）是否正在运行 */
static uint8_t s_wired_running;
/** 当前有线测量模式（要求1/2/3/4或调试） */
static WiredMeasurementMode_t s_wired_mode;
/** 菜单6独立DDS测试正在运行。 */
static uint8_t s_dds_test_active;
/** 相邻FTW抖动开关。 */
static uint8_t s_dds_dither_enabled;
/** 高端FTW的使用比例分子，分母固定为16。 */
static uint8_t s_dds_dither_numerator;
/** Bresenham/Σ-Δ误差累加器。 */
static uint8_t s_dds_dither_accumulator;
/** 相邻FTW抖动的低端基准频率字。 */
static uint32_t s_dds_base_ftw;
/** 最近一次实际写入AD9959的频率字，用于跳过重复串行写。 */
static uint32_t s_dds_last_written_ftw;
/** 最近一次抖动调度时刻。 */
static TickType_t s_dds_last_update_tick;
/** 有线PI闭环使用相邻FTW混合的高端比例。 */
static uint8_t s_wired_dither_numerator;
/** 有线PI闭环的Σ-Δ累加器。 */
static uint8_t s_wired_dither_accumulator;
/** 有线PI闭环命令对应的低端FTW。 */
static uint32_t s_wired_base_ftw;
/** 有线PI最近一次实际写入的FTW。 */
static uint32_t s_wired_last_written_ftw;
/** 有线PI最近一次FTW调度时刻。 */
static TickType_t s_wired_last_update_tick;
/** 有线PI相邻FTW混合是否有效。 */
static uint8_t s_wired_dither_active;
/** 要求1~4是否已经完成一次可靠的开环前馈起振。 */
static uint8_t s_wired_feedforward_applied;
/** 菜单2固定Y轴幅值目标；菜单1保持跟随输入等幅。 */
static uint8_t s_fixed_amplitude_target;
static float s_capture_phase_error_deg;
static uint32_t s_capture_phase_sample_count;
static TickType_t s_capture_phase_last_tick;
static uint8_t s_dac_sawtooth_test_active;
/** Requirement 5 visual-only controller. */
static VisionControl_t s_vision_control;
static uint8_t s_vision_running;
static uint8_t s_vision_slave_running;
static uint32_t s_vision_slave_start_ms;
static uint32_t s_vision_slave_last_start_tx_ms;
static uint16_t s_vision_slave_last_sequence;
static uint16_t s_vision_slave_last_probe_id;
static uint8_t s_complete_output_high;

/* ---------- 内部辅助函数 ---------- */

/**
 * @brief 将当前的测量数据发布给UI。
 *
 * 若 publisher 已注册则调用，并更新发布时刻。
 */
static void FApp_Publish(void)
{
    if (s_publisher != NULL)
    {
        s_publisher(&s_measurement);
    }
    s_last_publish_tick = xTaskGetTickCount();
}

/**
 * @brief 设置状态信息并立即发布。
 *
 * @param ok   状态码，0表示错误，1表示正常。
 * @param text 状态描述字符串，为NULL时填充空字符串。
 */
static void FApp_SetStatus(uint8_t ok, const char *text)
{
    s_measurement.event = UI_EVENT_STATUS;
    s_measurement.status_code = ok;
    if (text == NULL)
    {
        text = "";
    }
    (void)strncpy(s_measurement.status_text, text,
                  sizeof(s_measurement.status_text) - 1U);
    s_measurement.status_text[
        sizeof(s_measurement.status_text) - 1U] = '\0';
    FApp_Publish();
}

static void FApp_SetVisionIndicators(uint8_t red,
                                     uint8_t green,
                                     uint8_t blue,
                                     uint8_t yellow)
{
    /* Board LEDs are active low. */
//    LED1((red != 0U) ? 0U : 1U);
//    LED2((green != 0U) ? 0U : 1U);
//    LED3((blue != 0U) ? 0U : 1U);
//    LED4((yellow != 0U) ? 0U : 1U);
}

static void FApp_CompleteOutputInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* Preload PG10 low before switching it to output mode. */
    HAL_GPIO_WritePin(F_APP_COMPLETE_GPIO_PORT,
                      F_APP_COMPLETE_GPIO_PIN, GPIO_PIN_RESET);
    s_complete_output_high = 0U;
    gpio.Pin = F_APP_COMPLETE_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(F_APP_COMPLETE_GPIO_PORT, &gpio);
}

static void FApp_SetCompleteOutput(uint8_t completed)
{
    s_complete_output_high = (completed != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(
        F_APP_COMPLETE_GPIO_PORT,
        F_APP_COMPLETE_GPIO_PIN,
        (s_complete_output_high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void FApp_ToggleCompleteOutput(void)
{
    FApp_SetCompleteOutput((s_complete_output_high == 0U) ? 1U : 0U);
    FApp_SetStatus(
        1U,
        (s_complete_output_high != 0U) ?
            "PG10 TEST HIGH" : "PG10 TEST LOW");
}

static uint8_t FApp_MapVisionState(VisionControlStage_t stage)
{
    switch (stage)
    {
        case VISION_CONTROL_COARSE_SWEEP:
            return UI_CONTROL_STATE_COARSE_SWEEP_1X;
        case VISION_CONTROL_LOCAL_CONFIRM:
            return UI_CONTROL_STATE_LOCAL_SWEEP_1X;
        case VISION_CONTROL_FREQUENCY_LOCK:
            return UI_CONTROL_STATE_FREQUENCY_LOCK_1X;
        case VISION_CONTROL_AMPLITUDE_LOCK:
            return UI_CONTROL_STATE_AMPLITUDE_CONTROL;
        case VISION_CONTROL_PHASE_LOCK:
            return UI_CONTROL_STATE_PHASE_CONTROL;
        case VISION_CONTROL_VERIFY:
            return UI_CONTROL_STATE_STRICT_VERIFY;
        case VISION_CONTROL_HOLD:
            return UI_CONTROL_STATE_HOLD;
        case VISION_CONTROL_ERROR:
            return UI_CONTROL_STATE_ERROR;
        default:
            return UI_CONTROL_STATE_IDLE;
    }
}

static const char *FApp_VisionStatus(VisionControlStage_t stage)
{
    switch (stage)
    {
        case VISION_CONTROL_COARSE_SWEEP:
            return "VISION PRIME SWEEP";
        case VISION_CONTROL_LOCAL_CONFIRM:
            return "VISION RATIO CONFIRM";
        case VISION_CONTROL_FREQUENCY_LOCK:
            return "VISION FREQ LOCK";
        case VISION_CONTROL_AMPLITUDE_LOCK:
            return "VISION AMPLITUDE LOCK";
        case VISION_CONTROL_PHASE_LOCK:
            return "VISION PHASE DPLL";
        case VISION_CONTROL_VERIFY:
            return "VISION STRICT VERIFY";
        case VISION_CONTROL_HOLD:
            return "VISION LOCKED";
        case VISION_CONTROL_ERROR:
            return "VISION CONTROL ERROR";
        default:
            return "VISION IDLE";
    }
}

static uint8_t FApp_RunVisionControl(TickType_t now)
{
    VisionLinkMeasurement_t frame;
    VisionControlAction_t action;
    uint32_t now_ms = (uint32_t)now * (uint32_t)portTICK_PERIOD_MS;
    uint8_t frame_ready;
    uint8_t changed;
    uint8_t tx_busy = 0U;

    if (s_vision_running == 0U)
        return 0U;

    frame_ready = VisionLink_Poll(now_ms, &frame);
    changed = VisionControl_Process(
        &s_vision_control, now_ms,
        (frame_ready != 0U) ? &frame : NULL, &action);

    if (action.apply_dds != 0U)
    {
        if (AD9959_SetChannelOutputPrecise(
                F_APP_DDS_OUTPUT_CHANNEL,
                action.frequency_millihz,
                action.amplitude_mv,
                action.phase_millideg,
                1U) != HAL_OK)
        {
            VisionControl_Abort(&s_vision_control);
            s_vision_running = 0U;
            (void)AD9959_StopOutput();
            s_measurement.control_state = UI_CONTROL_STATE_ERROR;
            (void)strcpy(s_measurement.status_text,
                         "VISION DDS WRITE ERROR");
            return 1U;
        }
    }
    if ((action.send_probe != 0U) &&
        (VisionLink_SendProbe(
             action.sequence, action.probe_id,
             action.frequency_millihz, action.phase_millideg,
             action.target_shape) == 0U))
    {
        /* Keep the state alive; its timeout will retry the request. */
        (void)strcpy(s_measurement.status_text, "K230 TX BUSY");
        tx_busy = 1U;
        changed = 1U;
    }

    s_measurement.vision_online = VisionLink_IsOnline(now_ms);
    if (s_vision_control.latest.sequence != 0U)
    {
        s_measurement.vision_valid = s_vision_control.latest.valid;
        s_measurement.vision_confidence =
            (float)s_vision_control.latest.confidence_permille / 1000.0f;
        s_measurement.vision_span_x_div =
            (float)s_vision_control.latest.span_x_millidiv / 1000.0f;
        s_measurement.vision_span_y_div =
            (float)s_vision_control.latest.span_y_millidiv / 1000.0f;
        s_measurement.vision_center_x_div =
            (float)s_vision_control.latest.center_x_millidiv / 1000.0f;
        s_measurement.vision_center_y_div =
            (float)s_vision_control.latest.center_y_millidiv / 1000.0f;
        s_measurement.vision_phase_error_deg =
            (float)s_vision_control.latest.phase_error_millideg / 1000.0f;
        s_measurement.vision_phase_rate_deg_s =
            (float)s_vision_control.latest.phase_rate_millideg_s / 1000.0f;
        s_measurement.vision_probe_id =
            s_vision_control.latest.probe_id;
        s_measurement.vision_ratio_y =
            s_vision_control.latest.ratio_y;
        s_measurement.vision_ratio_x =
            s_vision_control.latest.ratio_x;
    }
    s_measurement.sweep_index = s_vision_control.probe_cursor;
    s_measurement.sweep_total = VisionControl_GetProbeTotal();
    s_measurement.control_elapsed_ms = now_ms - s_vision_control.start_ms;
    s_measurement.control_state =
        FApp_MapVisionState(s_vision_control.stage);
    s_measurement.frequency_valid =
        (s_vision_control.input_frequency_millihz != 0U) ? 1U : 0U;
    s_measurement.frequency_hz =
        (s_vision_control.input_frequency_millihz + 500UL) / 1000UL;
    s_measurement.frequency_hz_precise =
        (float)s_vision_control.input_frequency_millihz / 1000.0f;
    s_measurement.phase_valid = s_measurement.vision_valid;
    s_measurement.phase_error_deg =
        s_measurement.vision_phase_error_deg;
    s_measurement.dds_frequency_millihz =
        s_vision_control.output_frequency_millihz;
    s_measurement.dds_frequency_hz =
        (s_vision_control.output_frequency_millihz + 500UL) / 1000UL;
    s_measurement.dds_amplitude_mv =
        s_vision_control.output_amplitude_mv;
    s_measurement.dds_phase_deg = (uint16_t)(
        (s_vision_control.output_phase_millideg + 500UL) / 1000UL);

    if (s_vision_control.stage == VISION_CONTROL_ERROR)
    {
        s_vision_running = 0U;
        (void)AD9959_StopOutput();
        s_measurement.status_code = 0U;
    }
    else
    {
        s_measurement.status_code = 1U;
    }
    if (tx_busy == 0U)
    {
        (void)strcpy(s_measurement.status_text,
                     FApp_VisionStatus(s_vision_control.stage));
    }
    if (changed != 0U)
        s_measurement.event = UI_EVENT_VISION_MEASUREMENT;
    return changed;
}

/* Fast PC-master path: F407 only executes one AD9959 channel. */
static uint8_t FApp_RunVisionSlave(TickType_t now)
{
    VisionSlaveCommand_t command;
    uint32_t now_ms = (uint32_t)now * (uint32_t)portTICK_PERIOD_MS;
    uint8_t changed = 0U;

    if (s_vision_slave_running == 0U)
        return 0U;
    if (VisionSlave_Poll(now_ms, &command) != 0U)
    {
        changed = 1U;
        if (command.kind == VISION_SLAVE_COMMAND_STOP)
        {
            s_vision_slave_running = 0U;
            FApp_SetCompleteOutput(0U);
            (void)AD9959_StopOutput();
            s_measurement.control_state = UI_CONTROL_STATE_IDLE;
            s_measurement.status_code = 1U;
            (void)strcpy(s_measurement.status_text,
                         "PC VISION STOP");
            FApp_SetVisionIndicators(0U, 0U, 0U, 0U);
        }
        else if (command.kind == VISION_SLAVE_COMMAND_LOCKED)
        {
            if (command.sequence != s_vision_slave_last_sequence)
                changed = 0U;
            else
            {
                s_measurement.control_state = UI_CONTROL_STATE_HOLD;
                s_measurement.status_code = 1U;
                s_measurement.frequency_valid = 1U;
                s_measurement.frequency_hz =
                    (command.input_frequency_millihz + 500UL) / 1000UL;
                s_measurement.frequency_hz_precise =
                    (float)command.input_frequency_millihz / 1000.0f;
                s_measurement.vision_valid = 1U;
                (void)strcpy(s_measurement.status_text,
                             "PC VISION HOLD 5S");
                FApp_SetVisionIndicators(0U, 0U, 1U, 0U);
            }
        }
        else if (command.kind == VISION_SLAVE_COMMAND_DONE)
        {
            if (command.sequence != s_vision_slave_last_sequence)
                changed = 0U;
            else
            {
                s_measurement.control_state = UI_CONTROL_STATE_HOLD;
                s_measurement.status_code = 1U;
                s_measurement.frequency_valid = 1U;
                s_measurement.frequency_hz =
                    (command.input_frequency_millihz + 500UL) / 1000UL;
                s_measurement.frequency_hz_precise =
                    (float)command.input_frequency_millihz / 1000.0f;
                s_measurement.vision_valid = 1U;
                (void)strcpy(s_measurement.status_text,
                             "AUTO COMPLETE");
                FApp_SetCompleteOutput(1U);
                FApp_SetVisionIndicators(0U, 1U, 0U, 0U);
            }
        }
        else if (command.kind == VISION_SLAVE_COMMAND_FAILED)
        {
            s_vision_slave_running = 0U;
            FApp_SetCompleteOutput(0U);
            /*
             * Keep the last bounded probe output instead of collapsing the
             * Y axis to zero.  The K230 search only commands 20..500 mV and
             * validated DDS frequencies, so holding the last probe is safe
             * and leaves a visible waveform for ROI/threshold diagnosis.
             * A new automatic run or an explicit stop still disables DDS via
             * FApp_StopActive().
             */
            s_measurement.control_state = UI_CONTROL_STATE_ERROR;
            s_measurement.status_code = 0U;
            s_measurement.vision_valid = 0U;
            s_measurement.vision_probe_id = command.error_code;
            (void)strcpy(s_measurement.status_text,
                         "PC VISION FAILED HOLD");
            FApp_SetVisionIndicators(1U, 0U, 0U, 0U);
        }
        else if (command.kind == VISION_SLAVE_COMMAND_SET)
        {
            FApp_SetCompleteOutput(0U);
            s_measurement.vision_valid = 0U;
            if ((command.sequence == s_vision_slave_last_sequence) &&
                (command.probe_id == s_vision_slave_last_probe_id))
            {
                (void)VisionSlave_SendApplied(
                    command.sequence, command.probe_id);
            }
            else if (AD9959_SetChannelOutputPrecise(
                         F_APP_DDS_OUTPUT_CHANNEL,
                         command.frequency_millihz,
                         command.amplitude_mv,
                         command.phase_millideg,
                         1U) != HAL_OK)
            {
                (void)VisionSlave_SendError(
                    command.sequence, command.probe_id, 1U);
                s_vision_slave_running = 0U;
                (void)AD9959_StopOutput();
                s_measurement.control_state = UI_CONTROL_STATE_ERROR;
                s_measurement.status_code = 0U;
                (void)strcpy(s_measurement.status_text,
                             "PC DDS WRITE ERROR");
                FApp_SetVisionIndicators(1U, 0U, 0U, 0U);
            }
            else
            {
                s_vision_slave_last_sequence = command.sequence;
                s_vision_slave_last_probe_id = command.probe_id;
                (void)VisionSlave_SendApplied(
                    command.sequence, command.probe_id);
                s_measurement.control_state =
                    UI_CONTROL_STATE_COARSE_SWEEP_1X;
                s_measurement.status_code = 1U;
                (void)strcpy(s_measurement.status_text,
                             "PC PROBE APPLIED");
                s_measurement.dds_frequency_millihz =
                    command.frequency_millihz;
                s_measurement.dds_frequency_hz =
                    (command.frequency_millihz + 500UL) / 1000UL;
                s_measurement.dds_amplitude_mv = command.amplitude_mv;
                s_measurement.dds_phase_deg = (uint16_t)(
                    (command.phase_millideg + 500UL) / 1000UL);
                s_measurement.vision_probe_id = command.probe_id;
                s_measurement.target_shape = command.target_shape;
                s_measurement.sweep_index = command.probe_id;
                s_measurement.sweep_total =
                    VisionControl_GetProbeTotal();
                FApp_SetVisionIndicators(0U, 0U, 0U, 1U);
            }
        }
    }

    s_measurement.vision_online = VisionSlave_IsOnline(now_ms);
    if ((s_vision_slave_running != 0U) &&
        (s_vision_slave_last_sequence == 0U) &&
        ((uint32_t)(now_ms - s_vision_slave_last_start_tx_ms) >=
         F_APP_VISION_START_RETRY_MS))
    {
        /*
         * START is intentionally idempotent before the first SET.  Repeating
         * it removes the power-up race where K230 enables UART after the
         * original one-shot frame has already left USART6.
         */
        (void)VisionSlave_SendStart(s_measurement.target_shape);
        s_vision_slave_last_start_tx_ms = now_ms;
    }
    s_measurement.control_elapsed_ms =
        now_ms - s_vision_slave_start_ms;
    s_measurement.event = (changed != 0U) ?
        UI_EVENT_CONTROL_PROGRESS : UI_EVENT_MEASUREMENT;
    return changed;
}

/**
 * @brief 把菜单6精调状态复制到统一UI测量结构。
 */
#if (F_APP_DAC_SAWTOOTH_TEST_ENABLE != 0U)
static void FApp_StartDacSawtoothTest(void)
{
    if (BSP_DAC_StartWaveform(
            F_APP_DAC_SAWTOOTH_TEST_VPP,
            F_APP_DAC_SAWTOOTH_TEST_FREQ_HZ,
            50.0,
            SAWTOOTHWAVE) != HAL_OK)
    {
        s_dac_sawtooth_test_active = 0U;
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "DAC SAW INIT ERROR");
        return;
    }

    s_dac_sawtooth_test_active = 1U;
    s_measurement.dds_frequency_hz = F_APP_DAC_SAWTOOTH_TEST_FREQ_HZ;
    s_measurement.dds_frequency_millihz =
        F_APP_DAC_SAWTOOTH_TEST_FREQ_HZ * 1000UL;
    s_measurement.dds_amplitude_mv =
        (float)(F_APP_DAC_SAWTOOTH_TEST_VPP * 1000.0);
    s_measurement.dds_phase_deg = 0U;
    FApp_SetStatus(1U, "DAC SAW PA4 RUNNING");
}
#endif

static void FApp_UpdateDdsFineUi(void)
{
    s_measurement.dds_base_ftw = s_dds_base_ftw;
    s_measurement.dds_dither_numerator = s_dds_dither_numerator;
    s_measurement.dds_dither_enabled = s_dds_dither_enabled;
    s_measurement.dds_test_active = s_dds_test_active;
}

/**
 * @brief 清除菜单6精细锁频调试状态。
 */
static void FApp_ResetDdsFineState(void)
{
    s_dds_test_active = 0U;
    s_dds_dither_enabled = 0U;
    s_dds_dither_numerator = F_APP_DDS_DITHER_DEFAULT_NUM;
    s_dds_dither_accumulator = 0U;
    s_dds_base_ftw = 0U;
    s_dds_last_written_ftw = 0U;
    s_dds_last_update_tick = xTaskGetTickCount();
    FApp_UpdateDdsFineUi();
}

/**
 * @brief 只更新菜单6测试通道的FTW，不改相位和幅度。
 */
static HAL_StatusTypeDef FApp_WriteDdsTestFtw(uint32_t ftw)
{
    HAL_StatusTypeDef status;

    if (ftw == s_dds_last_written_ftw)
    {
        return HAL_OK;
    }

    status = AD9959_SetChannelFrequencyRaw(
        F_APP_DDS_OUTPUT_CHANNEL, ftw);
    if (status == HAL_OK)
    {
        s_dds_last_written_ftw = ftw;
    }
    return status;
}

/**
 * @brief 按固定16级比例在base FTW与base+1之间进行Σ-Δ调度。
 *
 * 该函数由任务循环调用，不在中断中执行GPIO模拟串行写入。
 */
static void FApp_ProcessDdsDither(TickType_t now)
{
    uint32_t output_ftw;
    uint32_t maximum_ftw;

    if ((s_dds_test_active == 0U) ||
        (s_dds_dither_enabled == 0U) ||
        ((TickType_t)(now - s_dds_last_update_tick) <
         pdMS_TO_TICKS(F_APP_DDS_DITHER_PERIOD_MS)))
    {
        return;
    }

    s_dds_last_update_tick = now;
    output_ftw = s_dds_base_ftw;
    s_dds_dither_accumulator =
        (uint8_t)(s_dds_dither_accumulator +
                  s_dds_dither_numerator);
    if (s_dds_dither_accumulator >=
        F_APP_DDS_DITHER_DENOMINATOR)
    {
        s_dds_dither_accumulator =
            (uint8_t)(s_dds_dither_accumulator -
                      F_APP_DDS_DITHER_DENOMINATOR);
        maximum_ftw = AD9959_FrequencyMilliHzToFtw(
            AD9959_PROJECT_MAX_FREQUENCY_HZ * 1000UL);
        if (output_ftw < maximum_ftw)
        {
            ++output_ftw;
        }
    }

    if (FApp_WriteDdsTestFtw(output_ftw) != HAL_OK)
    {
        s_dds_dither_enabled = 0U;
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_UpdateDdsFineUi();
        FApp_SetStatus(0U, "DDS DITHER WRITE ERROR");
    }
}

/** 清除要求1~4自动相邻FTW混合状态。 */
static void FApp_ResetWiredDither(void)
{
    s_wired_dither_numerator = 0U;
    s_wired_dither_accumulator = 0U;
    s_wired_base_ftw = 0U;
    s_wired_last_written_ftw = 0U;
    s_wired_last_update_tick = xTaskGetTickCount();
    s_wired_dither_active = 0U;
    s_wired_feedforward_applied = 0U;
}

/**
 * @brief 把0.001Hz命令换算为固定的最近FTW。
 *
 * 闭环阶段禁止10ms相邻FTW周期切换，避免给李萨如图引入确定性的
 * 频率/相位调制。保留high_numerator输出仅为兼容现有状态结构。
 */
static uint8_t FApp_CalculateWiredDither(
    uint32_t frequency_millihz,
    uint32_t *base_ftw,
    uint8_t *high_numerator)
{
    const AD9959_State_t *dds_state;

    if ((base_ftw == NULL) || (high_numerator == NULL))
    {
        return 0U;
    }

    dds_state = AD9959_GetState();
    if ((dds_state == NULL) ||
        (dds_state->initialized == 0U) ||
        (dds_state->system_clock_hz == 0U))
    {
        return 0U;
    }

    *base_ftw = AD9959_FrequencyMilliHzToFtw(frequency_millihz);
    *high_numerator = 0U;
    return 1U;
}

/** 有线闭环只更新FTW，不改变当前幅度和相位。 */
static HAL_StatusTypeDef FApp_WriteWiredFtw(uint32_t ftw)
{
    HAL_StatusTypeDef status;

    if (ftw == s_wired_last_written_ftw)
    {
        return HAL_OK;
    }

    status = AD9959_SetChannelFrequencyRaw(
        F_APP_DDS_OUTPUT_CHANNEL, ftw);
    if (status == HAL_OK)
    {
        s_wired_last_written_ftw = ftw;
    }
    return status;
}

/**
 * @brief 调度要求1~4的相邻FTW序列。
 *
 * 更新FTW不会清零AD9959相位累加器，因此不会像重复重启DDS那样产生
 * 明显相位跳变。10ms调度产生的瞬时相位纹波远小于测量阈值。
 */
static void FApp_ProcessWiredDither(TickType_t now)
{
    uint32_t output_ftw;

    if ((s_wired_dither_active == 0U) ||
        ((TickType_t)(now - s_wired_last_update_tick) <
         pdMS_TO_TICKS(F_APP_DDS_DITHER_PERIOD_MS)))
    {
        return;
    }

    s_wired_last_update_tick = now;
    output_ftw = s_wired_base_ftw;
    s_wired_dither_accumulator =
        (uint8_t)(s_wired_dither_accumulator +
                  s_wired_dither_numerator);
    if (s_wired_dither_accumulator >=
        F_APP_DDS_DITHER_DENOMINATOR)
    {
        s_wired_dither_accumulator =
            (uint8_t)(s_wired_dither_accumulator -
                      F_APP_DDS_DITHER_DENOMINATOR);
        ++output_ftw;
    }

    if (FApp_WriteWiredFtw(output_ftw) != HAL_OK)
    {
        s_wired_dither_active = 0U;
        wired_control_stop();
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "DDS PI FTW ERROR");
    }
}

/**
 * @brief 把PI控制器的完整命令写入AD9959。
 *
 * 首次写入或幅相变化时写FTW/CPOW/ASF；后续频率细分只修改FTW，
 * 因而相位累加器保持连续。
 */
static HAL_StatusTypeDef FApp_ApplyWiredControlCommand(
    const WiredControlCommand_t *command)
{
    uint32_t base_ftw;
    uint8_t high_numerator;
    uint16_t pow;
    uint16_t asf;
    uint8_t dither_changed;
    uint32_t output_ftw;

    if ((command == NULL) ||
        (FApp_CalculateWiredDither(
            command->frequency_millihz,
            &base_ftw,
            &high_numerator) == 0U))
    {
        return HAL_ERROR;
    }

    dither_changed =
        ((s_wired_base_ftw != base_ftw) ||
         (s_wired_dither_numerator != high_numerator)) ?
        1U : 0U;
    s_wired_base_ftw = base_ftw;
    s_wired_dither_numerator = high_numerator;
    s_wired_dither_active = 0U;
    if (dither_changed != 0U)
    {
        s_wired_dither_accumulator = 0U;
    }

    /*
     * 完整幅相命令也必须服从同一Σ-Δ序列，不能每20ms强制写base；
     * 否则PI频繁更新幅度时会系统性降低base+1的实际占空比。
     */
    output_ftw = s_wired_base_ftw;
    s_wired_dither_accumulator =
        (uint8_t)(s_wired_dither_accumulator +
                  s_wired_dither_numerator);
    if (s_wired_dither_accumulator >=
        F_APP_DDS_DITHER_DENOMINATOR)
    {
        s_wired_dither_accumulator =
            (uint8_t)(s_wired_dither_accumulator -
                      F_APP_DDS_DITHER_DENOMINATOR);
        ++output_ftw;
    }

    pow = AD9959_PhaseMilliDegreeToPow(
        command->phase_millideg);
    asf = AD9959_AmplitudeMilliVoltToAsf(
        command->amplitude_mv);
    if (AD9959_SetChannelRaw(
            F_APP_DDS_OUTPUT_CHANNEL,
            output_ftw,
            pow,
            asf,
            command->output_enable) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_wired_last_update_tick = xTaskGetTickCount();
    s_wired_last_written_ftw = output_ftw;

    s_measurement.dds_frequency_millihz =
        command->frequency_millihz;
    s_measurement.dds_frequency_hz =
        (command->frequency_millihz + 500UL) / 1000UL;
    s_measurement.dds_amplitude_mv =
        command->amplitude_mv;
    s_measurement.dds_phase_deg =
        (uint16_t)((command->phase_millideg + 500UL) /
                   1000UL);
    return HAL_OK;
}

/**
 * @brief 停止当前所有活动的测量与输出。
 *
 * @param stop_dds 若为非零，同时关闭DDS输出。
 *                 调试模式或仅停止测量时可传入0。
 */
static void FApp_StopActive(uint8_t stop_dds)
{
    /*
     * 必须先关闭任务级抖动调度，防止停止DDS后又被下一次任务循环
     * 写回非零频率字。
     */
    FApp_ResetDdsFineState();
    FApp_ResetWiredDither();
    VisionControl_Stop(&s_vision_control);
    s_vision_running = 0U;
    s_vision_slave_running = 0U;
    s_vision_slave_start_ms = 0U;
    s_vision_slave_last_start_tx_ms = 0U;
    s_vision_slave_last_sequence = 0U;
    s_vision_slave_last_probe_id = 0U;
    FApp_SetCompleteOutput(0U);
    FApp_SetVisionIndicators(0U, 0U, 0U, 0U);
    if (s_dac_sawtooth_test_active != 0U)
    {
        (void)BSP_DAC_StopWaveform();
        s_dac_sawtooth_test_active = 0U;
    }
    PhaseVofa_SetOutputEnabled(0U);
    wired_control_stop();
    if (s_wired_running != 0U)
    {
        (void)wired_measurement_stop();
        s_wired_running = 0U;
    }
    if (s_frequency_running != 0U)
    {
        (void)frequency_meter_stop();
        s_frequency_running = 0U;
    }
    if (stop_dds != 0U)
    {
        (void)AD9959_StopOutput();
        s_measurement.dds_frequency_hz = 0U;
        s_measurement.dds_frequency_millihz = 0U;
        s_measurement.dds_amplitude_mv = 0.0f;
        s_measurement.dds_phase_deg = 0U;
    }
    s_wired_mode = WIRED_MEASUREMENT_MODE_NONE;
    s_fixed_amplitude_target = 0U;
    s_capture_phase_error_deg = 0.0f;
    s_capture_phase_sample_count = 0U;
    s_capture_phase_last_tick = 0U;
    s_measurement.frequency_pi_correction_hz = 0.0f;
    s_measurement.phase_pi_correction_hz = 0.0f;
    s_measurement.amplitude_pi_correction_mv = 0.0f;
    s_measurement.wired_control_locked = 0U;
}

/**
 * @brief 判断当前有线测量模式是否需要DDS输出。
 *
 * @param mode 有线测量模式。
 * @return 1 需要DDS输出，0 不需要。
 */
static uint8_t FApp_ModeUsesDds(WiredMeasurementMode_t mode)
{
    return ((mode == WIRED_MEASUREMENT_MODE_REQUIREMENT1) ||
            (mode == WIRED_MEASUREMENT_MODE_REQUIREMENT2) ||
            (mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3) ||
            (mode == WIRED_MEASUREMENT_MODE_REQUIREMENT4)) ? 1U : 0U;
}

static HAL_StatusTypeDef FApp_UpdateCaptureEdgeCount(
    uint32_t input_frequency_hz)
{
    uint32_t desired_edges =
        (input_frequency_hz < F_APP_LOW_BAND_MAX_FREQUENCY_HZ) ?
        F_APP_LOW_BAND_CAPTURE_EDGES : PHASE_CAPTURE_FAST_EDGE_COUNT;

    if (PhaseCapture_GetEdgeCount(&g_phase_capture) == desired_edges)
    {
        return HAL_OK;
    }
    return PhaseCapture_SetEdgeCount(
        &g_phase_capture, desired_edges);
}

/**
 * @brief 根据要求4的目标分度给出DDS端开环幅度。
 *
 * 数值沿用9224864中已经验证能正常出波的前馈表。硬件链路为
 * AD9959 -> OPA1612固定8.5倍，示波器保持0.5V/div。
 */
static float FApp_WiredFeedforwardAmplitude(void)
{
    if (s_fixed_amplitude_target == 0U)
    {
        return F_APP_WIRED_FULL_SCALE_MV;
    }

    switch (s_measurement.amplitude_target_div)
    {
        case 2U:
            return 118.0f;
        case 4U:
            return 235.0f;
        case 6U:
            return 353.0f;
        case 8U:
            return F_APP_WIRED_FULL_SCALE_MV;
        default:
            return F_APP_WIRED_FULL_SCALE_MV;
    }
}

/**
 * @brief 在闭环接管前，先按9224864的方式直接启动AD9959 CH0。
 *
 * PB10频率有效后只执行一次。要求2输出同频90度，要求3输出二倍频
 * 0度，要求4输出同频0度和目标分度对应的前馈幅度。后续PI/DPLL仍
 * 通过FApp_ApplyWiredControlCommand更新FTW、相位和幅度。
 */
static HAL_StatusTypeDef FApp_ApplyWiredFeedforward(
    uint32_t input_frequency_hz)
{
    uint32_t output_frequency_hz;
    uint32_t phase_deg;
    float amplitude_mv;

    if ((input_frequency_hz == 0U) ||
        (FApp_ModeUsesDds(s_wired_mode) == 0U))
    {
        return HAL_ERROR;
    }

    output_frequency_hz = input_frequency_hz;
    phase_deg = 0U;
    amplitude_mv = FApp_WiredFeedforwardAmplitude();

    if (s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT2)
    {
        phase_deg = 90U;
    }
    else if (s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3)
    {
        if (input_frequency_hz >
            (AD9959_PROJECT_MAX_FREQUENCY_HZ / 2UL))
        {
            return HAL_ERROR;
        }
        output_frequency_hz = input_frequency_hz * 2UL;
    }

    if (AD9959_SetChannelOutput(
            F_APP_DDS_OUTPUT_CHANNEL,
            output_frequency_hz,
            amplitude_mv,
            phase_deg,
            1U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_wired_feedforward_applied = 1U;
    s_measurement.dds_frequency_hz = output_frequency_hz;
    s_measurement.dds_frequency_millihz =
        output_frequency_hz * 1000UL;
    s_measurement.dds_amplitude_mv = amplitude_mv;
    s_measurement.dds_phase_deg = (uint16_t)phase_deg;
    return HAL_OK;
}

/** 把测量模式转换为PI控制模式。 */
static WiredControlMode_t FApp_ToControlMode(
    WiredMeasurementMode_t mode)
{
    switch (mode)
    {
        case WIRED_MEASUREMENT_MODE_REQUIREMENT1:
            return WIRED_CONTROL_MODE_REQUIREMENT1;
        case WIRED_MEASUREMENT_MODE_REQUIREMENT2:
            return WIRED_CONTROL_MODE_REQUIREMENT2;
        case WIRED_MEASUREMENT_MODE_REQUIREMENT3:
            return WIRED_CONTROL_MODE_REQUIREMENT3;
        case WIRED_MEASUREMENT_MODE_REQUIREMENT4:
            return WIRED_CONTROL_MODE_REQUIREMENT4;
        default:
            return WIRED_CONTROL_MODE_NONE;
    }
}

/** 把有线控制阶段映射到现有UI状态枚举。 */
static uint8_t FApp_MapControlState(
    WiredControlStage_t stage)
{
    switch (stage)
    {
        case WIRED_CONTROL_STAGE_WAIT_INPUT:
            return UI_CONTROL_STATE_WIRED_WAIT_FREQUENCY;
        case WIRED_CONTROL_STAGE_FREQUENCY:
            return UI_CONTROL_STATE_FREQUENCY_LOCK_1X;
        case WIRED_CONTROL_STAGE_AMPLITUDE:
            return UI_CONTROL_STATE_AMPLITUDE_CONTROL;
        case WIRED_CONTROL_STAGE_PHASE:
            return UI_CONTROL_STATE_PHASE_CONTROL;
        case WIRED_CONTROL_STAGE_VERIFY:
            return UI_CONTROL_STATE_STRICT_VERIFY;
        case WIRED_CONTROL_STAGE_HOLD:
            return UI_CONTROL_STATE_HOLD;
        case WIRED_CONTROL_STAGE_FAULT:
            return UI_CONTROL_STATE_ERROR;
        default:
            return UI_CONTROL_STATE_IDLE;
    }
}

/**
 * @brief 把闭环结果复制到UI，并给出当前控制阶段提示。
 */
static void FApp_CopyControlResult(
    const WiredControlResult_t *control,
    const FrequencyMeterResult_t *frequency,
    const WiredMeasurementResult_t *wired,
    uint8_t phase_feedback_valid)
{
    const char *status;

    if (control == NULL)
    {
        return;
    }

    s_measurement.control_state =
        FApp_MapControlState(control->stage);
    s_measurement.control_elapsed_ms =
        control->control_elapsed_ms;
    s_measurement.frequency_pi_correction_hz =
        control->frequency_pi_correction_hz;
    s_measurement.phase_pi_correction_hz =
        control->phase_pi_correction_hz;
    s_measurement.amplitude_pi_correction_mv =
        control->amplitude_pi_correction_mv;
    s_measurement.wired_control_locked = control->locked;
    s_measurement.dds_frequency_millihz =
        control->command_frequency_millihz;

    s_measurement.status_code =
        (control->fault == 0U) ? 1U : 0U;
    status = "PI STOPPED";

    /*
     * 硬件异常优先于普通阶段提示，便于台架上快速定位接线和量程。
     */
    if (control->fault != 0U)
    {
        status = "PI RANGE FAULT";
    }
    else if ((frequency == NULL) ||
             (frequency->valid == 0U))
    {
        status = "PI WAIT PB10";
    }
    else if ((wired != NULL) &&
             (wired->input_clipped != 0U))
    {
        s_measurement.status_code = 0U;
        status = "ADC INPUT CLIPPED";
    }
    else if ((wired != NULL) &&
             (wired->output_clipped != 0U))
    {
        s_measurement.status_code = 0U;
        status = "ADC OUTPUT CLIPPED";
    }
    else if ((wired != NULL) &&
             (wired->bias_valid == 0U))
    {
        s_measurement.status_code = 0U;
        status = "AMP MEAS INVALID";
    }
    else if (control->amplitude_feedback_valid == 0U)
    {
        s_measurement.status_code = 0U;
        status = "AMP MEAS INVALID";
    }
    else if (control->amplitude_refeed_active != 0U)
    {
        status = "PI AMP REFEED";
    }
    else
    {
        switch (control->stage)
        {
            case WIRED_CONTROL_STAGE_WAIT_INPUT:
                status = "PI WAIT PB10";
                break;
            case WIRED_CONTROL_STAGE_FREQUENCY:
                status =
                    ((frequency != NULL) &&
                     (frequency->feedback_valid != 0U)) ?
                    "PI FREQUENCY" : "PI WAIT PB11";
                break;
            case WIRED_CONTROL_STAGE_AMPLITUDE:
                status =
                    ((wired != NULL) &&
                     (wired->amplitude_valid != 0U)) ?
                    "PI AMPLITUDE" : "PI WAIT AMPLITUDE";
                break;
            case WIRED_CONTROL_STAGE_PHASE:
                if (phase_feedback_valid == 0U)
                {
                    status = "PI WAIT CAPTURE";
                }
                else
                {
                    switch ((WiredPhaseCalibrationState_t)
                            control->phase_calibration_state)
                    {
                        case WIRED_PHASE_CAL_CAPTURE_LOCK:
                            status = "PH CAPTURE LOCK";
                            break;
                        case WIRED_PHASE_CAL_SETTLE:
                            status = "PH ANALOG SETTLE";
                            break;
                        case WIRED_PHASE_CAL_COLLECT:
                            status = "PH ADC CALIBRATE";
                            break;
                        case WIRED_PHASE_CAL_RELOCK:
                            status = "PH CAPTURE RELOCK";
                            break;
                        case WIRED_PHASE_CAL_VERIFY:
                            status = "PH ADC VERIFY";
                            break;
                        case WIRED_PHASE_CAL_FAILED:
                            s_measurement.status_code = 0U;
                            status = "PH CAL FAILED";
                            break;
                        default:
                            status = "PI PHASE DPLL";
                            break;
                    }
                }
                break;
            case WIRED_CONTROL_STAGE_VERIFY:
                status = "PI VERIFY";
                break;
            case WIRED_CONTROL_STAGE_HOLD:
                status = "PI HOLD LOCKED";
                break;
            case WIRED_CONTROL_STAGE_FAULT:
                status = "PI FAULT";
                break;
            default:
                status = "PI STOPPED";
                break;
        }
    }

    (void)strncpy(
        s_measurement.status_text,
        status,
        sizeof(s_measurement.status_text) - 1U);
    s_measurement.status_text[
        sizeof(s_measurement.status_text) - 1U] = '\0';
}

/**
 * @brief 组合最新频率、幅值和相位结果，运行一次要求1~4闭环。
 */
static uint8_t FApp_RunWiredControl(
    const FrequencyMeterResult_t *frequency,
    const WiredMeasurementResult_t *wired,
    TickType_t now)
{
    WiredControlInput_t input;
    WiredControlCommand_t command;
    const WiredControlResult_t *control;
    float capture_phase_error_deg = s_capture_phase_error_deg;
    float measured_phase_error_deg;
    float target_phase_deg = 0.0f;
    uint8_t capture_phase_valid =
        ((s_capture_phase_sample_count != 0U) &&
         ((TickType_t)(now - s_capture_phase_last_tick) <=
          pdMS_TO_TICKS(F_APP_PHASE_SAMPLE_TIMEOUT_MS))) ? 1U : 0U;
    uint8_t frequency_ratio = 1U;
    uint8_t changed;

    if ((FApp_ModeUsesDds(s_wired_mode) == 0U) ||
        (frequency == NULL) || (wired == NULL))
    {
        return 0U;
    }

    (void)memset(&input, 0, sizeof(input));
    if ((s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT1) ||
        (s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT2) ||
        (s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3))
    {
        target_phase_deg =
            wired_control_get_capture_phase_target_deg();
    }
    if (s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3)
    {
        frequency_ratio = 2U;
    }
    if ((s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT1) ||
        (s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT2) ||
        (s_wired_mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3))
    {
        if ((frequency->valid != 0U) &&
            (frequency->feedback_valid != 0U))
        {
            if (PhaseCapture_GetPhaseError(
                    &g_phase_capture,
                    frequency->period_ticks,
                    frequency->capture_cycles,
                    frequency_ratio,
                    target_phase_deg,
                    &measured_phase_error_deg) != 0U)
            {
                s_capture_phase_error_deg = measured_phase_error_deg;
                capture_phase_error_deg = measured_phase_error_deg;
                ++s_capture_phase_sample_count;
                s_capture_phase_last_tick = now;
                capture_phase_valid = 1U;
            }
        }
    }

    input.now_ms = (uint32_t)now;
    input.input_frequency_hz =
        frequency->filtered_frequency_hz;
    input.feedback_frequency_hz =
        frequency->feedback_filtered_frequency_hz;
    input.input_vpp_mv = wired->input_vpp_mv;
    input.output_vpp_mv = wired->output_vpp_mv;
    /* TIM2 drives the fast DPLL; ADC phase only calibrates its target. */
    input.phase_error_deg = capture_phase_error_deg;
    input.analog_phase_error_deg = wired->phase_error_deg;
    input.adc_block_count = wired->block_count;
    input.feedback_sample_count = frequency->feedback_sample_count;
    input.phase_sample_count = s_capture_phase_sample_count;
    input.input_frequency_valid = frequency->valid;
    input.feedback_frequency_valid =
        frequency->feedback_valid;
    input.input_amplitude_valid =
        wired->input_amplitude_valid;
    input.amplitude_valid = wired->amplitude_valid;
    input.phase_valid = capture_phase_valid;
    input.analog_phase_valid = wired->phase_valid;
    input.input_clipped = wired->input_clipped;
    input.output_clipped = wired->output_clipped;
    input.clipped = wired->clipped;
    input.input_bias_valid = wired->input_bias_valid;
    input.output_bias_valid = wired->output_bias_valid;

    changed = wired_control_step(&input);
    if (wired_control_take_command(&command) != 0U)
    {
        if (FApp_ApplyWiredControlCommand(&command) != HAL_OK)
        {
            s_wired_dither_active = 0U;
            wired_control_stop();
            s_measurement.control_state =
                UI_CONTROL_STATE_ERROR;
            s_measurement.status_code = 0U;
            (void)strcpy(
                s_measurement.status_text,
                "DDS PI SERIAL ERROR");
            return 1U;
        }
        changed = 1U;
    }

    control = wired_control_get_result();
    FApp_CopyControlResult(control, frequency, wired,
                           capture_phase_valid);
    return changed;
}

/**
 * @brief 将频率计结果同步到 UI 测量结构。
 *
 * @param result 频率计测量结果指针。
 */
static void FApp_CopyFrequencyResult(
    const FrequencyMeterResult_t *result)
{
    float expected_ratio;

    if (result == NULL)
    {
        return;
    }

    /*
     * 浮点滤波值只留在FrequencyMeter内部。应用、DDS和LCD统一使用
     * 以100Hz为基础单位的整数频点。
     */
    s_measurement.frequency_hz = result->frequency_hz;
    s_measurement.frequency_hz_precise =
        result->filtered_frequency_hz;
    s_measurement.frequency_100hz = result->frequency_100hz;
    s_measurement.frequency_jitter_ppm = result->jitter_ppm;
    s_measurement.period_ticks = result->period_ticks;
    s_measurement.capture_cycles = result->capture_cycles;
    s_measurement.frequency_sample_count = result->sample_count;
    s_measurement.signal_age_ms = result->signal_age_ms;
    s_measurement.capture_dma_error_count =
        result->dma_error_count;
    s_measurement.frequency_valid = result->valid;

    /*
     * PB11 是输出反馈的精确频率通道，不经过100 Hz量化。
     * 要求3的目标是2*fin，其余有线模式按1*fin计算频差。
     */
    s_measurement.feedback_frequency_hz_precise =
        result->feedback_filtered_frequency_hz;
    s_measurement.feedback_period_ticks =
        result->feedback_period_ticks;
    s_measurement.feedback_capture_cycles =
        result->feedback_capture_cycles;
    s_measurement.feedback_signal_age_ms =
        result->feedback_signal_age_ms;
    s_measurement.feedback_dma_error_count =
        result->feedback_dma_error_count;
    s_measurement.feedback_frequency_valid =
        result->feedback_valid;
    expected_ratio =
        (s_wired_mode ==
         WIRED_MEASUREMENT_MODE_REQUIREMENT3) ? 2.0f : 1.0f;
    if ((result->valid != 0U) &&
        (result->feedback_valid != 0U))
    {
        s_measurement.feedback_frequency_error_hz =
            result->feedback_filtered_frequency_hz -
            expected_ratio * result->filtered_frequency_hz;
    }
    else
    {
        s_measurement.feedback_frequency_error_hz = 0.0f;
    }
}

/**
 * @brief 将双ADC有线测量结果同步到 UI 测量结构。
 *
 * 同时根据当前模式和硬件状态生成简单的状态提示信息。
 *
 * @param result 有线测量结果指针。
 */
static void FApp_CopyWiredResult(
    const WiredMeasurementResult_t *result)
{
    PhaseVofaSnapshot_t vofa_snapshot;

    if (result == NULL)
    {
        return;
    }

    (void)memset(&vofa_snapshot, 0, sizeof(vofa_snapshot));
    vofa_snapshot.menu = s_measurement.active_menu;
    vofa_snapshot.measure_state = s_measurement.control_state;
    vofa_snapshot.frequency_valid = s_measurement.frequency_valid;
    vofa_snapshot.phase_valid = result->phase_valid;
    vofa_snapshot.frequency_hz =
        (result->output_analysis_frequency_hz != 0U) ?
        (float)result->output_analysis_frequency_hz :
        s_measurement.frequency_hz_precise;
    vofa_snapshot.channel1_amplitude = (float)result->input_vpp_mv;
    vofa_snapshot.channel2_amplitude = (float)result->output_vpp_mv;
    vofa_snapshot.phase_deg = result->phase_relation_deg;
    vofa_snapshot.adc_sample_rate_hz = result->adc_sample_rate_hz;
    vofa_snapshot.adc_overrun_count = result->adc_overrun_count;
    PhaseVofa_SubmitSnapshot(&vofa_snapshot);

    s_measurement.adc_sample_rate_hz =
        result->adc_sample_rate_hz;
    s_measurement.adc_overrun_count =
        result->adc_overrun_count;
    s_measurement.adc_rejected_block_count =
        result->adc_rejected_block_count;
    s_measurement.input_vpp_mv = result->input_vpp_mv;
    s_measurement.output_vpp_mv = result->output_vpp_mv;
    s_measurement.input_bias_mv = result->input_bias_mv;
    s_measurement.output_bias_mv = result->output_bias_mv;
    s_measurement.output_min_count = result->output_min_count;
    s_measurement.output_max_count = result->output_max_count;
    s_measurement.input_phase_deg = result->input_phase_deg;
    s_measurement.output_phase_deg = result->output_phase_deg;
    s_measurement.phase_relation_deg =
        result->phase_relation_deg;
    s_measurement.phase_error_deg = result->phase_error_deg;
    s_measurement.crossing_phase_relation_deg =
        result->crossing_phase_relation_deg;
    s_measurement.phase_estimator_difference_deg =
        result->phase_estimator_difference_deg;
    s_measurement.phase_agreement_count =
        result->phase_agreement_count;
    s_measurement.wired_block_count = result->block_count;
    s_measurement.amplitude_valid = result->amplitude_valid;
    s_measurement.wired_phase_valid = result->phase_valid;
    s_measurement.adc_clipped = result->clipped;
    s_measurement.phase_valid = result->phase_valid;
    s_measurement.channel1_amplitude =
        (float)result->input_vpp_mv;
    s_measurement.channel2_amplitude =
        (float)result->output_vpp_mv;

    /* 要求4需要计算输出幅度与设定目标的误差 */
    if (s_fixed_amplitude_target != 0U)
    {
        s_measurement.amplitude_error_mv =
            (int32_t)result->output_vpp_mv -
            (int32_t)((uint32_t)s_measurement.amplitude_target_div *
                      500U);
    }
    else
    {
        s_measurement.amplitude_error_mv =
            result->amplitude_error_mv;
    }

    /* 依据硬件状态给出提示 */
    if (result->input_clipped != 0U)
    {
        s_measurement.status_code = 0U;
        (void)strcpy(
            s_measurement.status_text, "ADC INPUT CLIPPED");
    }
    else if (result->output_clipped != 0U)
    {
        s_measurement.status_code = 0U;
        (void)strcpy(
            s_measurement.status_text, "ADC OUTPUT CLIPPED");
    }
    else if (result->bias_valid == 0U)
    {
        s_measurement.status_code = 0U;
        (void)strcpy(s_measurement.status_text, "CHECK 1.635V BIAS");
    }
    else if (result->amplitude_valid == 0U)
    {
        s_measurement.status_code = 0U;
        (void)strcpy(s_measurement.status_text, "WAIT PA1/PA2 SIGNAL");
    }
    else
    {
        s_measurement.status_code = 1U;
        (void)strcpy(s_measurement.status_text, "WIRED MEAS VALID");
    }
}

/**
 * @brief 启动有线测量模式（要求1/2/3/4或调试）。
 *
 * 会先停止当前活动，然后依次启动频率计和有线测量。
 *
 * @param mode       有线测量子模式。
 * @param app_mode   对应的UI应用模式。
 * @param target_div 目标幅度分度数（仅要求4有意义）。
 */
static void FApp_StartWiredMode(
    WiredMeasurementMode_t mode,
    UI_AppMode_t app_mode,
    uint8_t target_shape,
    uint8_t target_div,
    uint8_t fixed_amplitude_target)
{
    FApp_StopActive(1U);
    s_fixed_amplitude_target = fixed_amplitude_target;

    s_capture_phase_error_deg = 0.0f;
    s_capture_phase_sample_count = 0U;
    s_capture_phase_last_tick = 0U;

    if ((frequency_meter_start() != HAL_OK) ||
        (wired_measurement_start(mode) != HAL_OK))
    {
        FApp_StopActive(1U);
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "WIRED START ERROR");
        return;
    }

    s_frequency_running = 1U;
    s_wired_running = 1U;
    s_wired_mode = mode;
    s_measurement.active_menu = (uint8_t)app_mode;
    s_measurement.target_shape = target_shape;
    s_measurement.control_state =
        UI_CONTROL_STATE_WIRED_WAIT_FREQUENCY;
    s_measurement.frequency_valid = 0U;
    s_measurement.phase_valid = 0U;
    s_measurement.amplitude_valid = 0U;
    s_measurement.wired_phase_valid = 0U;
    s_measurement.amplitude_target_div = target_div;
    s_measurement.wired_control_locked = 0U;
    s_measurement.event = UI_EVENT_MEASUREMENT;
    PhaseVofa_SetOutputEnabled(1U);

    if ((FApp_ModeUsesDds(mode) != 0U) &&
        (wired_control_start(
            FApp_ToControlMode(mode),
            target_div,
            (uint32_t)xTaskGetTickCount()) == 0U))
    {
        FApp_StopActive(1U);
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "PI START PARAM ERROR");
        return;
    }

    if ((fixed_amplitude_target != 0U) &&
        (wired_control_set_amplitude_target_div(
            target_div,
            (uint32_t)xTaskGetTickCount()) == 0U))
    {
        FApp_StopActive(1U);
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "AMPLITUDE TARGET ERROR");
        return;
    }

    (void)strcpy(s_measurement.status_text, "WAIT PB10 FREQUENCY");
    FApp_Publish();
}

/**
 * @brief 启动自动模式（要求5，依赖PC视觉闭环）。
 *
 * 不会使能PB10、PA1/PA2等任何与信号源电气连接的硬件；AD9959
 * CH0只输出探测/锁定正弦波，频率、幅度和相位由PC串口命令更新。
 *
 * @param target_shape 目标波形形状（正弦/方波等）。
 */
static void FApp_StartAutomaticMode(uint8_t target_shape)
{
#if (F_APP_VISION_PC_MASTER_ENABLE != 0U)
    if ((target_shape < UI_TARGET_SHAPE_DIAGONAL) ||
        (target_shape > UI_TARGET_SHAPE_INFINITY))
    {
        FApp_SetStatus(0U, "PC TARGET ERROR");
        return;
    }
    FApp_StopActive(1U);
    VisionSlave_Init();
    s_vision_slave_running = 1U;
    s_vision_slave_start_ms = (uint32_t)xTaskGetTickCount() *
        (uint32_t)portTICK_PERIOD_MS;
    s_vision_slave_last_start_tx_ms = s_vision_slave_start_ms;
    s_vision_slave_last_sequence = 0U;
    s_vision_slave_last_probe_id = 0U;
    FApp_SetVisionIndicators(0U, 0U, 0U, 1U);
    s_measurement.active_menu = UI_APP_MODE_AUTOMATIC_CONTROL;
    s_measurement.target_shape = target_shape;
    s_measurement.control_state = UI_CONTROL_STATE_WAIT_VISION;
    s_measurement.target_span_x_div = 8.0f;
    s_measurement.target_span_y_div = 8.0f;
    s_measurement.frequency_valid = 0U;
    s_measurement.vision_online = 0U;
    s_measurement.vision_valid = 0U;
    s_measurement.sweep_index = 0U;
    s_measurement.sweep_total = VisionControl_GetProbeTotal();
    s_measurement.event = UI_EVENT_CONTROL_PROGRESS;
    (void)VisionSlave_SendStart(target_shape);
    FApp_SetStatus(1U, "PC MASTER WAIT SET");
#else
    uint32_t now_ms;

    FApp_StopActive(1U);

    /*
     * 要求5禁止装置与信号源存在任何电气连接，因此这里绝不启动
     * PB10、PA1或PA2。K230有效帧接入后才能进入视觉粗扫。
     */
    now_ms = (uint32_t)xTaskGetTickCount() *
        (uint32_t)portTICK_PERIOD_MS;
    VisionLink_Reset();
    if (VisionControl_Start(&s_vision_control, target_shape,
                            now_ms) == 0U)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "VISION TARGET ERROR");
        return;
    }

    s_vision_running = 1U;
    s_measurement.active_menu = UI_APP_MODE_AUTOMATIC_CONTROL;
    s_measurement.target_shape = target_shape;
    s_measurement.control_state = UI_CONTROL_STATE_COARSE_SWEEP_1X;
    s_measurement.target_span_x_div = 8.0f;
    s_measurement.target_span_y_div = 8.0f;
    s_measurement.frequency_valid = 0U;
    s_measurement.vision_online = 0U;
    s_measurement.vision_valid = 0U;
    s_measurement.vision_ratio_y = 0U;
    s_measurement.vision_ratio_x = 0U;
    s_measurement.sweep_index = 0U;
    s_measurement.sweep_total = VisionControl_GetProbeTotal();
    s_measurement.control_elapsed_ms = 0U;
    s_measurement.event = UI_EVENT_CONTROL_PROGRESS;
    FApp_SetStatus(1U, "VISION PRIME SWEEP");
#endif
}

/**
 * @brief 启动菜单6中的AD9959 CH0独立输出测试。
 *
 * 该测试不会启动PB10频率计或PA1/PA2双ADC，避免与原有调试流程混杂。
 * UI通过数字1~4传入1 kHz、10 kHz、100 kHz或200 kHz；幅度固定为
 * 118 mVpp，相位固定为0度。再次按回车仍可恢复原来的有线调试。
 *
 * @param command UI传入的DDS测试参数。
 */
static void FApp_StartAd9959Test(const UI_Command_t *command)
{
    const AD9959_State_t *dds_state;
    float amplitude_mv;

    if ((command == NULL) ||
        (command->frequency_hz == 0U) ||
        (command->frequency_hz >
         AD9959_PROJECT_MAX_FREQUENCY_HZ))
    {
        FApp_SetStatus(0U, "DDS TEST PARAM ERROR");
        return;
    }

    amplitude_mv = command->amplitude_mv;
    if (amplitude_mv <= 0.0f)
    {
        amplitude_mv = F_APP_DDS_TEST_AMPLITUDE_MV;
    }

    FApp_StopActive(1U);
    if (AD9959_SetChannelOutput(
            F_APP_DDS_OUTPUT_CHANNEL,
            command->frequency_hz,
            amplitude_mv,
            command->phase_deg,
            1U) != HAL_OK)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "DDS CH0 TEST ERROR");
        return;
    }

    s_measurement.active_menu = UI_APP_MODE_DEBUG;
    s_measurement.control_state = UI_CONTROL_STATE_HOLD;
    s_measurement.frequency_valid = 0U;
    s_measurement.phase_valid = 0U;
    s_measurement.dds_channel = F_APP_DDS_OUTPUT_CHANNEL;
    s_measurement.dds_frequency_hz = command->frequency_hz;
    s_measurement.dds_frequency_millihz =
        command->frequency_hz * 1000UL;
    s_measurement.dds_amplitude_mv = amplitude_mv;
    s_measurement.dds_phase_deg =
        (uint16_t)(command->phase_deg % 360U);

    /*
     * 以驱动实际写入的FTW作为精调基准，避免重新计算时引入另一轮
     * 舍入。默认只准备50%混合比例，抖动开关保持关闭。
     */
    dds_state = AD9959_GetState();
    if (dds_state == NULL)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "DDS STATE ERROR");
        return;
    }
    s_dds_test_active = 1U;
    s_dds_dither_enabled = 0U;
    s_dds_dither_numerator = F_APP_DDS_DITHER_DEFAULT_NUM;
    s_dds_dither_accumulator = 0U;
    s_dds_base_ftw =
        dds_state->channel[F_APP_DDS_OUTPUT_CHANNEL].ftw;
    s_dds_last_written_ftw = s_dds_base_ftw;
    s_dds_last_update_tick = xTaskGetTickCount();
    FApp_UpdateDdsFineUi();
    FApp_SetStatus(1U, "DDS CH0 TEST ON");
}

/**
 * @brief 菜单6将低端基准FTW加减1。
 */
static void FApp_AdjustAd9959Ftw(int8_t step)
{
    uint32_t maximum_ftw;

    if (s_dds_test_active == 0U)
    {
        FApp_SetStatus(0U, "START DDS TEST FIRST");
        return;
    }
    if ((step != -1) && (step != 1))
    {
        FApp_SetStatus(0U, "FTW STEP ERROR");
        return;
    }

    maximum_ftw = AD9959_FrequencyMilliHzToFtw(
        AD9959_PROJECT_MAX_FREQUENCY_HZ * 1000UL);
    if (((step < 0) && (s_dds_base_ftw == 0U)) ||
        ((step > 0) && (s_dds_base_ftw >= maximum_ftw)))
    {
        FApp_SetStatus(0U, "FTW LIMIT");
        return;
    }

    if (step < 0)
    {
        --s_dds_base_ftw;
    }
    else
    {
        ++s_dds_base_ftw;
    }
    s_dds_dither_accumulator = 0U;
    s_dds_last_update_tick = xTaskGetTickCount();

    /*
     * 先落到新的低端FTW；若抖动开启，后续任务周期会继续按比例
     * 选择base或base+1。
     */
    if (FApp_WriteDdsTestFtw(s_dds_base_ftw) != HAL_OK)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "DDS FTW WRITE ERROR");
        return;
    }

    FApp_UpdateDdsFineUi();
    FApp_SetStatus(1U, (step < 0) ? "FTW -1" : "FTW +1");
}

/**
 * @brief 菜单6将相邻FTW混合比例加减1/16。
 */
static void FApp_AdjustAd9959Dither(int8_t step)
{
    if (s_dds_test_active == 0U)
    {
        FApp_SetStatus(0U, "START DDS TEST FIRST");
        return;
    }
    if ((step != -1) && (step != 1))
    {
        FApp_SetStatus(0U, "MIX STEP ERROR");
        return;
    }

    if ((step < 0) && (s_dds_dither_numerator > 0U))
    {
        --s_dds_dither_numerator;
    }
    else if ((step > 0) &&
             (s_dds_dither_numerator <
              F_APP_DDS_DITHER_DENOMINATOR))
    {
        ++s_dds_dither_numerator;
    }
    else
    {
        FApp_SetStatus(0U, "MIX LIMIT");
        return;
    }

    s_dds_dither_accumulator = 0U;
    s_dds_last_update_tick = xTaskGetTickCount();
    FApp_UpdateDdsFineUi();
    FApp_SetStatus(1U, (step < 0) ? "MIX -1/16" : "MIX +1/16");
}

/**
 * @brief 菜单6开关相邻FTW抖动。
 */
static void FApp_ToggleAd9959Dither(void)
{
    if (s_dds_test_active == 0U)
    {
        FApp_SetStatus(0U, "START DDS TEST FIRST");
        return;
    }

    s_dds_dither_enabled =
        (s_dds_dither_enabled == 0U) ? 1U : 0U;
    s_dds_dither_accumulator = 0U;
    s_dds_last_update_tick = xTaskGetTickCount();

    /*
     * 关闭时立即恢复低端FTW，避免停留在上一个base+1周期。
     * 开启时也从base开始，随后由任务周期执行Σ-Δ序列。
     */
    if (FApp_WriteDdsTestFtw(s_dds_base_ftw) != HAL_OK)
    {
        s_dds_dither_enabled = 0U;
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_UpdateDdsFineUi();
        FApp_SetStatus(0U, "DDS FTW WRITE ERROR");
        return;
    }

    FApp_UpdateDdsFineUi();
    FApp_SetStatus(
        1U, (s_dds_dither_enabled != 0U) ?
        "DITHER ON" : "DITHER OFF");
}

/**
 * @brief 关闭菜单6的DDS测试输出，但保持在调试页面。
 */
static void FApp_StopAd9959Test(void)
{
    FApp_StopActive(1U);
    s_measurement.active_menu = UI_APP_MODE_DEBUG;
    s_measurement.control_state = UI_CONTROL_STATE_IDLE;
    s_measurement.frequency_valid = 0U;
    s_measurement.phase_valid = 0U;
    FApp_SetStatus(1U, "DDS TEST OFF");
}

/* ---------- 公开接口 ---------- */

/**
 * @brief 注册测量数据发布回调。
 *
 * @param publisher 回调函数指针，传入NULL可取消注册。
 */
void f_app_set_measurement_publisher(
    FAppMeasurementPublisher_t publisher)
{
    s_publisher = publisher;
}

/**
 * @brief 初始化F题应用模块。
 *
 * 执行顺序：初始化数据结构 → 配置并初始化频率计 → 初始化双ADC。
 * 若任一外设初始化失败，则进入错误状态。
 *
 * @return 1 成功，0 失败。
 */
uint8_t f_app_init(void)
{
    FrequencyMeterConfig_t meter_config;
    WiredControlConfig_t control_config;

    (void)memset(&s_measurement, 0, sizeof(s_measurement));
    FApp_CompleteOutputInit();
    FrequencyMeterConfig_Default(&meter_config);
    s_measurement.active_menu = UI_APP_MODE_IDLE;
    s_measurement.control_state = UI_CONTROL_STATE_IDLE;
    s_measurement.target_shape = UI_TARGET_SHAPE_NONE;
    s_measurement.capture_cycles = PHASE_CAPTURE_FAST_EDGE_COUNT;
    s_measurement.divider = 1U;
    s_measurement.dds_ready = 0U;
    s_measurement.dds_channel = F_APP_DDS_OUTPUT_CHANNEL;
    s_measurement.target_span_x_div = 8.0f;
    s_measurement.target_span_y_div = 8.0f;
    s_measurement.amplitude_target_div = 8U;
    (void)strcpy(s_measurement.status_text, "READY");

    if (AD9959_Init() != HAL_OK)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "DDS SERIAL INIT ERROR");
        return 0U;
    }
    s_measurement.dds_ready = 1U;
    if (frequency_meter_init(&meter_config) != HAL_OK)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "TIM2 INIT ERROR");
        return 0U;
    }
    if (wired_measurement_init() != HAL_OK)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "DUAL ADC INIT ERROR");
        return 0U;
    }
    if (PhaseVofa_CreateTask() == 0U)
    {
        s_measurement.control_state = UI_CONTROL_STATE_ERROR;
        FApp_SetStatus(0U, "VOFA TASK ERROR");
        return 0U;
    }

    /*
     * PI参数集中在WiredControlConfig_Default中，台架调参只需修改
     * 该配置，不需要改动测量链路或AD9959驱动。
     */
    WiredControlConfig_Default(&control_config);
    wired_control_init(&control_config);
    PidSerial_Init();
    VisionLink_Init();
    VisionControl_Init(&s_vision_control);
    VisionSlave_Init();
    s_vision_running = 0U;
    s_vision_slave_running = 0U;
    FApp_ResetWiredDither();

    s_initialized = 1U;
    FApp_SetStatus(1U, "READY");
#if (F_APP_DAC_SAWTOOTH_TEST_ENABLE != 0U)
    FApp_StartDacSawtoothTest();
#endif
    return 1U;
}

/**
 * @brief 定时处理测量流程，需在任务循环中调用。
 *
 * 主要工作：
 *   - 运行频率计处理并更新测量数据；
 *   - 在有线测量模式下，根据频率变化重新配置ADC采样率和DDS；
 *   - 运行有线测量处理；
 *   - 按固定周期发布测量结果。
 */
void f_app_step(void)
{
    const FrequencyMeterResult_t *frequency_result = NULL;
    const WiredMeasurementResult_t *wired_result = NULL;
#if (F_APP_VISION_PC_MASTER_ENABLE != 0U)
    VisionSlaveCommand_t discarded_slave_command;
#else
    VisionLinkMeasurement_t discarded_vision_frame;
#endif
    TickType_t now;
    uint8_t frequency_changed = 0U;
    uint8_t wired_changed = 0U;
    uint8_t control_changed = 0U;
    uint8_t vision_changed = 0U;

    if (s_initialized == 0U)
    {
        vTaskDelay(pdMS_TO_TICKS(10U));
        return;
    }

    now = xTaskGetTickCount();
    if (s_vision_slave_running != 0U)
        vision_changed = FApp_RunVisionSlave(now);
    else if (s_vision_running != 0U)
        vision_changed = FApp_RunVisionControl(now);
#if (F_APP_VISION_PC_MASTER_ENABLE != 0U)
    else
    {
        if (VisionSlave_Poll(
                (uint32_t)now * (uint32_t)portTICK_PERIOD_MS,
                &discarded_slave_command) != 0U &&
            discarded_slave_command.kind == VISION_SLAVE_COMMAND_RUN)
        {
            FApp_StartAutomaticMode(discarded_slave_command.target_shape);
        }
        else
        {
            PidSerial_Process();
        }
    }
#else
    else if (s_measurement.active_menu == UI_APP_MODE_AUTOMATIC_CONTROL)
        (void)VisionLink_Poll(
            (uint32_t)now * (uint32_t)portTICK_PERIOD_MS,
            &discarded_vision_frame);
    else
        PidSerial_Process();
#endif
    FApp_ProcessDdsDither(now);
    FApp_ProcessWiredDither(now);

    if (s_frequency_running != 0U)
    {
        frequency_changed = frequency_meter_process();
        frequency_result = frequency_meter_get_result();
        FApp_CopyFrequencyResult(frequency_result);

        if ((frequency_result != NULL) &&
            (frequency_result->valid != 0U))
        {
            /* 通知有线测量模块新的频率，用于设置ADC采样率 */
            if (FApp_UpdateCaptureEdgeCount(
                    frequency_result->frequency_hz) != HAL_OK)
            {
                s_measurement.control_state =
                    UI_CONTROL_STATE_ERROR;
                (void)strcpy(s_measurement.status_text,
                             "CAPTURE RATE ERROR");
            }
            else if (wired_measurement_set_frequency(
                         frequency_result->frequency_hz) != HAL_OK)
            {
                s_measurement.control_state =
                    UI_CONTROL_STATE_ERROR;
                (void)strcpy(s_measurement.status_text,
                             "ADC RATE ERROR");
            }
            else
            {
                /*
                 * 要求1~4先按9224864的可靠路径直接启动一次CH0，
                 * 避免等待PB11/ADC闭环反馈期间Y轴一直没有波形。
                 * 后续频率、幅值和相位仍由FApp_RunWiredControl接管。
                 */
                if ((FApp_ModeUsesDds(s_wired_mode) != 0U) &&
                    (s_wired_feedforward_applied == 0U))
                {
                    if (FApp_ApplyWiredFeedforward(
                            frequency_result->frequency_hz) != HAL_OK)
                    {
                        s_measurement.control_state =
                            UI_CONTROL_STATE_ERROR;
                        (void)strcpy(
                            s_measurement.status_text,
                            "DDS FEEDFORWARD ERROR");
                    }
                }
                else if (FApp_ModeUsesDds(s_wired_mode) == 0U)
                {
                    s_measurement.control_state =
                        UI_CONTROL_STATE_WIRED_MEASUREMENT;
                }
            }
        }
        else
        {
            (void)strcpy(s_measurement.status_text,
                         "WAIT PB10 FREQUENCY");
        }
    }

    if (s_wired_running != 0U)
    {
        wired_changed = wired_measurement_process(
            ((frequency_result != NULL) &&
             (frequency_result->valid != 0U)) ?
            frequency_result->filtered_frequency_hz : 0.0f);
        wired_result = wired_measurement_get_result();
        if (wired_changed != 0U)
        {
            FApp_CopyWiredResult(wired_result);
        }
    }

    /*
     * 控制器每20ms自行限速。它同时使用PB10/PB11精确频率与双ADC
     * 幅相结果，并且只在要求1~4工作。
     */
    if (FApp_ModeUsesDds(s_wired_mode) != 0U)
    {
        if (frequency_result == NULL)
        {
            frequency_result = frequency_meter_get_result();
        }
        if (wired_result == NULL)
        {
            wired_result = wired_measurement_get_result();
        }
    control_changed = FApp_RunWiredControl(
            frequency_result, wired_result, now);
        if ((s_fixed_amplitude_target != 0U) &&
            (wired_control_is_amplitude_target_fixed() == 0U))
        {
            s_fixed_amplitude_target = 0U;
            FApp_SetStatus(1U, "AUTO AMPLITUDE FOLLOW");
        }
    }

    /* 频率或有线数据更新时，或达到发布间隔时，将结果推送给UI */
    if (((frequency_changed != 0U) ||
         (wired_changed != 0U) ||
         (control_changed != 0U) ||
         (vision_changed != 0U)) ||
        (((s_frequency_running != 0U) ||
          (s_wired_running != 0U) ||
          (s_vision_running != 0U) ||
          (s_vision_slave_running != 0U)) &&
         ((TickType_t)(now - s_last_publish_tick) >=
          pdMS_TO_TICKS(F_APP_UI_PUBLISH_MS))))
    {
        s_measurement.event = UI_EVENT_MEASUREMENT;
        FApp_Publish();
    }

    /* 给按键/UI任务留出确定的调度机会。 */
    vTaskDelay(pdMS_TO_TICKS(1U));
}

/**
 * @brief 处理来自UI的指令。
 *
 * 支持启动各项要求、调试及停止输出等命令。
 *
 * @param command 指向UI命令的指针，不可为NULL。
 */
static WiredMeasurementMode_t FApp_ShapeToMeasurementMode(
    uint8_t target_shape)
{
    if (target_shape == UI_TARGET_SHAPE_DIAGONAL)
        return WIRED_MEASUREMENT_MODE_REQUIREMENT1;
    if (target_shape == UI_TARGET_SHAPE_ELLIPSE)
        return WIRED_MEASUREMENT_MODE_REQUIREMENT2;
    if (target_shape == UI_TARGET_SHAPE_INFINITY)
        return WIRED_MEASUREMENT_MODE_REQUIREMENT3;
    return WIRED_MEASUREMENT_MODE_NONE;
}

static void FApp_StartShapeControl(
    const UI_Command_t *command,
    uint8_t fixed_amplitude_target)
{
    WiredMeasurementMode_t mode =
        FApp_ShapeToMeasurementMode(command->target_shape);

    if (mode == WIRED_MEASUREMENT_MODE_NONE)
    {
        FApp_SetStatus(0U, "SHAPE PARAM ERROR");
        return;
    }

    FApp_StartWiredMode(
        mode,
        (fixed_amplitude_target != 0U) ?
            UI_APP_MODE_AMPLITUDE_CONTROL :
            UI_APP_MODE_WAVEFORM_CONTROL,
        command->target_shape,
        (fixed_amplitude_target != 0U) ?
            command->target_span_y_div : 8U,
        fixed_amplitude_target);
}

static void FApp_UpdateAmplitudeTarget(uint8_t target_div)
{
    if ((s_wired_running == 0U) ||
        (s_fixed_amplitude_target == 0U) ||
        (wired_control_set_amplitude_target_div(
            target_div,
            (uint32_t)xTaskGetTickCount()) == 0U))
    {
        FApp_SetStatus(0U, "AMPLITUDE UPDATE ERROR");
        return;
    }

    s_measurement.amplitude_target_div = target_div;
    s_measurement.target_span_y_div = (float)target_div;
    s_measurement.amplitude_pi_correction_mv = 0.0f;
    s_measurement.wired_control_locked = 0U;
    s_measurement.control_state = UI_CONTROL_STATE_AMPLITUDE_CONTROL;
    FApp_SetStatus(1U, "AMPLITUDE TARGET UPDATED");
}

void f_app_handle_command(const UI_Command_t *command)
{
    if ((command == NULL) || (s_initialized == 0U))
    {
        return;
    }

    switch (command->type)
    {
        case UI_COMMAND_START_WAVEFORM_CONTROL:
            FApp_StartShapeControl(command, 0U);
            break;

        case UI_COMMAND_START_AMPLITUDE_CONTROL:
            FApp_StartShapeControl(command, 1U);
            break;

        case UI_COMMAND_UPDATE_AMPLITUDE_TARGET:
            FApp_UpdateAmplitudeTarget(command->target_span_y_div);
            break;

        case UI_COMMAND_START_AUTOMATIC_CONTROL:
            FApp_StartAutomaticMode(command->target_shape);
            break;

        case UI_COMMAND_START_DEBUG:
            FApp_StartWiredMode(
                WIRED_MEASUREMENT_MODE_DEBUG,
                UI_APP_MODE_DEBUG,
                UI_TARGET_SHAPE_NONE, 8U, 0U);
            break;

        case UI_COMMAND_START_AD9959_TEST:
            FApp_StartAd9959Test(command);
            break;

        case UI_COMMAND_STOP_AD9959_TEST:
            FApp_StopAd9959Test();
            break;

        case UI_COMMAND_AD9959_ADJUST_FTW:
            FApp_AdjustAd9959Ftw(command->dds_fine_step);
            break;

        case UI_COMMAND_AD9959_ADJUST_DITHER:
            FApp_AdjustAd9959Dither(command->dds_fine_step);
            break;

        case UI_COMMAND_AD9959_TOGGLE_DITHER:
            FApp_ToggleAd9959Dither();
            break;

        case UI_COMMAND_TOGGLE_COMPLETE_OUTPUT:
            FApp_ToggleCompleteOutput();
            break;

        case UI_COMMAND_STOP_OUTPUT:
            FApp_StopActive(1U);
            s_measurement.frequency_valid = 0U;
            s_measurement.phase_valid = 0U;
            s_measurement.active_menu = UI_APP_MODE_IDLE;
            s_measurement.control_state = UI_CONTROL_STATE_IDLE;
            s_measurement.target_shape = UI_TARGET_SHAPE_NONE;
            FApp_SetStatus(1U, "STOPPED");
            break;

        default:
            break;
    }
}
