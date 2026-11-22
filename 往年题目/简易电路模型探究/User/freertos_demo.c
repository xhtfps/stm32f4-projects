/**
 * @file    freertos_demo.c
 * @brief   FreeRTOS 演示程序：多任务架构下的电路模型探究装置
 *
 * 本模块将整个系统拆分为三个主要任务：
 *   - KeyTask：轮询 PS/2 键盘，将按键值送入队列
 *   - UiTask：驱动 TFT LCD 显示，处理按键并转换为仪器命令
 *   - InstrumentTask：串行化所有模拟外设（AD9959、ADS1256、ADC/DAC），
 *                      执行测量、扫频建模以及 IIR 实时推理
 *
 * 任务间通过 FreeRTOS 队列进行通信，保证数据安全和外设互斥
 */

#include "freertos_demo.h"

#include <math.h>
#include <string.h>

#include "./BSP/ADC/ADC.h"                     /* ADC 驱动（波形采集） */
#include "./BSP/AD9959/Drive_Communication.h"  /* AD9959 DDS 驱动 */
#include "./BSP/DEBUG/debug_uart6.h"
#include "./BSP/ADS1256/ADS1256.h"    /* USART6 debug: PC6/PC7 */             /* ADS1256 24位 ADC 驱动 */
#include "./BSP/CIRCUIT/circuit_measure.h"     /* 电路测量接口 */
#include "./BSP/CIRCUIT/circuit_model.h"       /* 电路建模和 IIR 滤波器 */
#include "./BSP/DAC/DAC.h"                     /* DAC 驱动（波形输出） */
#include "./BSP/PS2/ps2.h"                     /* PS/2 键盘驱动 */
#include "user.h"                              /* UI 模块接口与结构体定义 */

#include "FreeRTOS.h"
#include "queue.h"                             /* 消息队列 */
#include "task.h"                              /* 任务管理 */

/* ================================================================
 * 任务优先级与栈大小
 * ================================================================ */

/** 启动任务优先级（最低，仅用于创建其他任务） */
#define START_TASK_PRIO        1U
/** 启动任务栈大小（字） */
#define START_TASK_STACK_SIZE  128U

/** 按键采集任务优先级（最高，保证按键响应实时性） */
#define KEY_TASK_PRIO          4U
/** 按键任务栈大小 */
#define KEY_TASK_STACK_SIZE    128U

/** UI 刷新任务优先级（中高，负责屏幕绘制） */
#define UI_TASK_PRIO           3U
/** UI 任务栈大小（512 字，因含大量字符串操作） */
#define UI_TASK_STACK_SIZE     512U

/** 仪器控制任务优先级（菜单4实时块处理需高于UI/按键任务） */
#define INSTRUMENT_TASK_PRIO   5U
/** 仪器任务栈大小 */
#define INSTRUMENT_TASK_STACK_SIZE 512U

/* ================================================================
 * FreeRTOS 队列句柄
 * ================================================================ */

/** 按键值队列：KeyTask -> UiTask */
static QueueHandle_t s_key_queue;
/** 命令队列：UiTask -> InstrumentTask */
static QueueHandle_t s_command_queue;
/** 测量数据队列：InstrumentTask -> UiTask */
static QueueHandle_t s_measurement_queue;
/** 菜单5进度邮箱：只保留最新值，不阻塞扫频任务 */
static QueueHandle_t s_ads_progress_queue;
/** Woken directly by ADC DMA during real-time menu-4 inference. */
static TaskHandle_t s_instrument_task_handle;
static volatile uint8_t s_stop_requested;

/* ================================================================
 * 测量与控制常量
 * ================================================================ */

/** 扫频起始频率 (Hz) */
#define CIRCUIT_SWEEP_START_HZ      1000U
/** 扫频终止频率 (Hz) */
#define CIRCUIT_SWEEP_STOP_HZ       50000U
/** 扫频总点数（对数分布） */
#define CIRCUIT_SWEEP_POINTS        256U
/*
 * 单通道更新需要 F1 协处理器烧录对应的新协议。当前工程默认使用
 * 四通道同步帧，兼容现场已经烧录的旧的 F1 固件；确保 F1 已更新且
 * 实测物理输出通道后，再将此宏改为 1U */
#define CIRCUIT_DDS_SWEEP_USE_SINGLE_CHANNEL 1U
#define CIRCUIT_DDS_SWEEP_CHANNEL   0U
#if CIRCUIT_DDS_SWEEP_USE_SINGLE_CHANNEL
#define CIRCUIT_DDS_SWEEP_MODE_TEXT "single"
#else
#define CIRCUIT_DDS_SWEEP_MODE_TEXT "all4"
#endif
/** 未知模型扫频激励：DDS 250 mV，经 OPA1612×8 后约 2 V 峰值 */
#define CIRCUIT_SOURCE_AMPLITUDE_MV 250.0f
/** 输出后等待稳定的延时 (ms) */
#define CIRCUIT_SETTLE_MS           120U
/** 低频切换后至少等待的信号周期数，为 AD637 检波器留出建立时间 */
#define CIRCUIT_SETTLE_CYCLES       5U
/** 单个扫频点的测量重试次数 */
#define CIRCUIT_MEASUREMENT_RETRY_COUNT 5U
#define CIRCUIT_DDS_RETRY_COUNT       3U
/** 测量失败后的重试间隔 (ms) */
#define CIRCUIT_MEASUREMENT_RETRY_DELAY_MS 50U
/** 测量消息队列长度；满时阻塞生产者，禁止覆盖中间进度 */
#define MEASUREMENT_QUEUE_LENGTH     16U
/** 菜单5对每个ADS1256通道重复采样次数 */
#define ADS1256_TEST_REPEAT_COUNT    4U
/** 校准测试每个通道的重复采样次数 */
#define CALIBRATION_REPEAT_COUNT     32U
/** 通道一致性测试使用的DDS幅值点 */
#define CALIBRATION_GAIN_LEVEL_COUNT 3U
/** 菜单2中 AD9959 后级 OPA1612 的实测放大倍数 */
#define MENU2_SOURCE_OPA_GAIN       7.432f
/**
 * 菜单2示波器校准倍率：目标 2.00 Vpp、1 kHz 时实测 2.40 V 和 2.38 V。
 * 该倍率只修正菜单2的激励前馈，不改变菜单3/4/5的测量链路。
 */
#define MENU2_SCOPE_CAL_TARGET_VPP  2.0f
#define MENU2_SCOPE_CAL_MEASURED_VPP ((2.40f + 2.38f) / 2.0f)
#define MENU2_SCOPE_GAIN_CORRECTION  (MENU2_SCOPE_CAL_MEASURED_VPP / \
                                      MENU2_SCOPE_CAL_TARGET_VPP)
/** RMS 值换算为正弦峰峰值的系数 */
#define CIRCUIT_RMS_TO_VPP          2.82842712475f

/*
 * 菜单2已知模型二维步进扫频。
 * 幅值是已知模型输出端的目标正弦峰峰值；使用整数0.1 V档位避免
 * 浮点累加使最后一个2.0 V点变成1.999...。
 */

/*
 * 示波器+信号发生器标准数据的线性拟合：
 * model_input_vpp = slope * target_output_vpp + intercept。
 * 三行依次对应 1 kHz、2 kHz、3 kHz。仅供菜单2前馈反算使用。
 */
/** 推理输出采样率 (Hz) */
#define CIRCUIT_INFERENCE_FS_HZ     500000.0f
/** ADC 参考电压 (V) */
#define CIRCUIT_INFERENCE_ADC_VREF  3.3f
/** ADC 直流偏置 (V)，假设无输入时码值对应中点 */
#define CIRCUIT_INFERENCE_ADC_BIAS  1.65f
/** DAC 直流偏置 (V) */
#define CIRCUIT_INFERENCE_DAC_BIAS  1.65f
/** 每次处理的数据块长度（与 ADC 驱动块长度一致） */
#define CIRCUIT_INFERENCE_BLOCK_LEN BSP_ADC_WAVEFORM_BLOCK_LENGTH
#define DAC_TEST_SAMPLE_RATE_HZ     256000U
#define DAC_TEST_LOW_V              0.50f
#define DAC_TEST_MID_V              1.65f
#define DAC_TEST_HIGH_V             2.65f

/* ================================================================
 * ADS1256 测量通道配置
 * ================================================================ */
static CircuitMeasureConfig_t s_measure_config =
{
    /* 输入通道：AIN0（连接到 AD637 直流输出） */
    (ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM),
    /* 输出通道：AIN1 */
    (ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM),
    32U,                              /* 平均采样次数 */
    ADS1256_REFERENCE_VOLTAGE_DEFAULT,/* 参考电压 */
    1U,                               /* PGA 增益 */
    1.0f,                             /* 输入电压换算系数 */
    1.0f,                             /* 输出电压换算系数 */
    1.0f,                             /* 输入通道硬件增益（跟随器，无增益） */
    1.0f,                             /* 输出通道硬件增益 */
    0.001191959f,                     /* 输入通道零点偏置 (V)，检测二 CAL_ZERO_AIN0 */
    0.002212874f                      /* 输出通道零点偏置 (V)，检测二 CAL_ZERO_AIN1 */
};

/* ================================================================
 * 全局应用状态（持久化，不在任务栈中）
 * ================================================================ */

/** 扫频与建模结果存储 */
static CircuitModel_t s_circuit_model;

typedef struct
{
    uint16_t attempted_points;
    uint16_t valid_points;
    uint16_t missing_points;
    uint16_t dds_fail_points;
    uint16_t dds_retry_total;
    uint16_t ads_fail_points;
    uint16_t invalid_points;
    uint16_t append_fail_points;
    uint16_t retry_total;
    uint16_t retry_exhausted;
    uint32_t elapsed_ms;
    uint8_t aborted;
} InstrumentSweepStats_t;
static UI_Measurement_t s_ads_test_snapshot;


/* IIR 滤波器实例（用于实时推理） */
static CircuitBiquad_t s_inference_filter;
/** ADC 采集缓冲区 */
static uint16_t s_inference_adc_block[CIRCUIT_INFERENCE_BLOCK_LEN];
/** DAC 输出缓冲（整个波形周期） */
static uint16_t s_inference_dac_buffer[BSP_ADC_WAVEFORM_BUFFER_LENGTH];
static uint16_t s_dac_test_buffer[BSP_ADC_WAVEFORM_BUFFER_LENGTH];

/* ================================================================
 * 内部任务函数声明
 * ================================================================ */
static void StartTask(void *argument);
static void KeyTask(void *argument);
static void UiTask(void *argument);
static void InstrumentTask(void *argument);

void Instrument_RequestStop(void)
{
    s_stop_requested = 1U;
    if (s_instrument_task_handle != NULL)
    {
        xTaskNotifyGive(s_instrument_task_handle);
    }
}

/*
 * ADC driver ISR hook.  A direct task notification avoids the old 1 ms polling
 * sleep, which spans four 128-sample blocks at 500 kS/s.
 */
void BSP_ADC_WaveformBlockReadyCallback(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_instrument_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_instrument_task_handle,
                               &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static void Instrument_ClearStopRequest(void)
{
    s_stop_requested = 0U;
}

/* ================================================================
 * freertos_demo：入口函数，创建队列与启动任务
 * ================================================================ */
void freertos_demo(void)
{
    /* 创建三个队列 */
    s_key_queue = xQueueCreate(8U, sizeof(PS2_KeyValue_t));
    s_command_queue = xQueueCreate(4U, sizeof(UI_Command_t));
    s_measurement_queue = xQueueCreate(MEASUREMENT_QUEUE_LENGTH,
                                        sizeof(UI_Measurement_t));
    /* xQueueOverwrite 仅用于长度为1的队列，作为菜单5最新值邮箱 */
    s_ads_progress_queue = xQueueCreate(1U, sizeof(UI_Measurement_t));

    /* 队列创建失败则直接返回 */
    if ((s_key_queue == NULL) || (s_command_queue == NULL) ||
        (s_measurement_queue == NULL) || (s_ads_progress_queue == NULL))
    {
        return;
    }

    /* 创建启动任务，由该任务负责创建其余任务 */
    (void)xTaskCreate(StartTask, "start", START_TASK_STACK_SIZE, NULL,
                      START_TASK_PRIO, NULL);
    /* 启动调度器，此后由 FreeRTOS 接管 */
    vTaskStartScheduler();
}

/* ================================================================
 * StartTask：初始化其他任务后自删除
 * ================================================================ */
static void StartTask(void *argument)
{
    (void)argument;
    taskENTER_CRITICAL();   /* 临界区保护，避免任务创建被打断 */

    (void)xTaskCreate(KeyTask, "key", KEY_TASK_STACK_SIZE, NULL,
                      KEY_TASK_PRIO, NULL);
    (void)xTaskCreate(UiTask, "ui", UI_TASK_STACK_SIZE, NULL,
                      UI_TASK_PRIO, NULL);
    (void)xTaskCreate(InstrumentTask, "instrument", INSTRUMENT_TASK_STACK_SIZE,
                      NULL, INSTRUMENT_TASK_PRIO,
                      &s_instrument_task_handle);

    taskEXIT_CRITICAL();
    vTaskDelete(NULL);      /* 启动任务自身不再需要 */
}

/* ================================================================
 * KeyTask：轮询 PS/2 键盘并将按键值发送到队列
 * 该任务不访问 LCD 或模拟外设，保持独立
 * ================================================================ */
static void KeyTask(void *argument)
{
    PS2_KeyValue_t key;

    (void)argument;
    for (;;)
    {
        PS2_KeyScan();                          /* 扫描键盘 */
        key = PS2_GetCurrentKeyValue();         /* 获取当前按键值 */
        if (key != PS2_KEY_VALUE_NULL)
        {
            /* 将按键值放入队列，若队列满则丢弃（不阻塞） */
            (void)xQueueSend(s_key_queue, &key, 0U);
        }
        vTaskDelay(pdMS_TO_TICKS(10U));         /* 10ms 轮询周期 */
    }
}

/* ================================================================
 * UiTask：负责 LCD 界面更新，并处理按键生成仪器命令
 * ================================================================ */
static void UiTask(void *argument)
{
    PS2_KeyValue_t key;
    UI_Command_t command;
    UI_Measurement_t measurement;

    (void)argument;
    UI_Init();   /* 初始化 UI，绘制主界面 */
    for (;;)
    {
        /*
         * 先处理扫频进度，再处理普通测量/模型结果。
         * 扫频结束时，最后一个进度可能已经在进度队列中；模型结果
         * 必须最后更新，避免模型结果画面被 256/256 进度再次覆盖。
         */
        while (xQueueReceive(s_ads_progress_queue, &measurement, 0U) == pdPASS)
        {
            UI_UpdateMeasurement(&measurement);
        }
        while (xQueueReceive(s_measurement_queue, &measurement, 0U) == pdPASS)
        {
            UI_UpdateMeasurement(&measurement);
        }

        /* 等待按键队列（最多 20ms），处理按键 */
        if (xQueueReceive(s_key_queue, &key, pdMS_TO_TICKS(20U)) == pdPASS)
        {
            if (key == PS2_KEY_VALUE_BACKSPACE)
            {
                Instrument_RequestStop();
            }
            if (UI_HandleKey(key, &command))
            {
                if (command.type == UI_COMMAND_STOP_OUTPUT)
                {
                    /* The instrument may be blocked by a full UI queue. */
                    Instrument_RequestStop();
                }
                (void)xQueueSend(s_command_queue, &command, 0U);
            }
        }
    }
}

/* ================================================================
 * 仪器任务辅助函数
 * ================================================================ */

/**
 * @brief 将测量数据通过队列发送给 UI 任务
 * @param measurement 指向测量数据的指针
 */
static uint8_t Instrument_Publish(const UI_Measurement_t *measurement)
{
    if (measurement != NULL)
    {
        /*
         * 扫频进度是有序数据，不能使用 xQueueOverwrite。
         * 队列满时阻塞当前仪器任务，等待 UiTask 消费，保证不丢点。
         */
        for (;;)
        {
            if (s_stop_requested != 0U)
            {
                return 0U;
            }
            if (xQueueSend(s_measurement_queue, measurement,
                           pdMS_TO_TICKS(20U)) == pdPASS)
            {
                return 1U;
            }
        }
    }
    return 0U;
}

/**
 * @brief 发布菜单5的最新扫频进度
 * @note  菜单5的完整频点已通过 USART6 输出，LCD只需要最新状态；
 *        使用长度为1的覆盖队列，不能阻塞 ADS/DDS 扫频任务。
 */
static uint8_t Instrument_PublishAdsProgress(
    const UI_Measurement_t *measurement)
{
    if ((measurement == NULL) || (s_stop_requested != 0U) ||
        (s_ads_progress_queue == NULL))
    {
        return 0U;
    }

    return (xQueueOverwrite(s_ads_progress_queue, measurement) == pdPASS) ?
           1U : 0U;
}

/**
 * @brief 计算扫频点的对数分布频率
 * @param index 当前扫频点索引 (0 ~ CIRCUIT_SWEEP_POINTS-1)
 * @return 对应的频率值 (Hz)
 */
static uint32_t Instrument_SweepFrequency(uint16_t index)
{
    float start_log = log10f((float)CIRCUIT_SWEEP_START_HZ);
    float stop_log = log10f((float)CIRCUIT_SWEEP_STOP_HZ);
    float ratio = (CIRCUIT_SWEEP_POINTS > 1U) ?
                  ((float)index / (float)(CIRCUIT_SWEEP_POINTS - 1U)) : 0.0f;

    /* 返回 10^(start_log + ratio * 对数范围) */
    return (uint32_t)powf(10.0f, start_log + ratio * (stop_log - start_log));
}

/**
 * @brief 计算题给已知二阶模型的增益幅度
 * @param frequency_hz 频率 (Hz)
 * @return 电压增益 (V/V)
 *
 * 题目给出的已知模型传递函数为：
 * H(s) = 5 / (10^-8*s^2 + 3*10^-4*s + 1)。
 * 这里计算其幅频特性 |H(jw)|，不再用近似 RLC 参数替代题目公式。
 */
static float Instrument_KnownModelGain(uint32_t frequency_hz)
{
    const float pi = 3.14159265358979323846f;
    const float a2 = 1.0e-8f;
    const float a1 = 3.0e-4f;
    const float numerator = 5.0f;
    float omega = 2.0f * pi * (float)frequency_hz;
    float real_part = 1.0f - a2 * omega * omega;
    float imaginary_part = a1 * omega;
    float denominator = sqrtf(real_part * real_part +
                              imaginary_part * imaginary_part);

    return (denominator > 0.000001f) ? (numerator / denominator) : 0.0f;
}

/**
 * @brief 计算某个标准频率锚点相对题给传递函数的经验修正系数
 * @note  标准值来自示波器与外置信号发生器，目标和结果均为 Vpp。
 */
static float Instrument_KnownSourceAmplitudeMv(uint32_t frequency_hz,
                                                float target_vpp)
{
    float model_gain;
    float source_amplitude_mv;

    if (target_vpp <= 0.0f)
    {
        target_vpp = UI_DEFAULT_KNOWN_TARGET_PEAK_V;
    }

    model_gain = Instrument_KnownModelGain(frequency_hz);
    if ((model_gain <= 0.000001f) || (MENU2_SOURCE_OPA_GAIN <= 0.0f))
    {
        return AD9959_MAX_OUTPUT_MV;
    }

    source_amplitude_mv = target_vpp * 1000.0f /
                          (model_gain * MENU2_SOURCE_OPA_GAIN *
                           MENU2_SCOPE_GAIN_CORRECTION);
    if (source_amplitude_mv < 0.0f)
    {
        source_amplitude_mv = 0.0f;
    }
    if (source_amplitude_mv > AD9959_MAX_OUTPUT_MV)
    {
        source_amplitude_mv = AD9959_MAX_OUTPUT_MV;
    }
    return source_amplitude_mv;
}

/**
 * @brief 电压值转换为 DAC 码值 (12 位)
 * @param voltage 电压 (V)，范围 0 ~ VREF
 * @return DAC 码值 (0 ~ 4095)
 */
static uint16_t Instrument_VoltageToDacCode(float voltage)
{
    if (voltage <= 0.0f)
    {
        return 0U;
    }
    if (voltage >= CIRCUIT_INFERENCE_ADC_VREF)
    {
        return 4095U;
    }
    return (uint16_t)(voltage * 4095.0f /
                      CIRCUIT_INFERENCE_ADC_VREF + 0.5f);
}

/**
 * @brief ADC 码值转换为电压值
 * @param code ADC 码值 (12 位)
 * @return 电压值 (V)
 */
static float Instrument_AdcCodeToVoltage(uint16_t code)
{
    return (float)code * CIRCUIT_INFERENCE_ADC_VREF / 4095.0f;
}

/**
 * @brief 检查命令队列中是否有停止输出指令（非阻塞）
 * @return 1 表示需要停止，0 表示继续
 */
static uint8_t Instrument_InferenceStopRequested(void)
{
    UI_Command_t command;

    if (s_stop_requested != 0U)
    {
        return 1U;
    }

    /* Catch a stop command that was queued before the fast flag was set. */
    if (xQueueReceive(s_command_queue, &command, 0U) == pdPASS)
    {
        if (command.type == UI_COMMAND_STOP_OUTPUT)
        {
            s_stop_requested = 1U;
            return 1U;
        }
        (void)xQueueSend(s_command_queue, &command, 0U);
    }

    return 0U;
}

/**
 * @brief 尝试初始化 ADS1256（若尚未成功）
 * @param ads_ready 输入输出参数，当前就绪状态，初始化成功后置 1
 */
static void Instrument_TryInitAds(uint8_t *ads_ready)
{
    if ((ads_ready != NULL) && (*ads_ready == 0U))
    {
        *ads_ready = (ADS1256_Init() == HAL_OK) ? 1U : 0U;
    }
}

/**
 * @brief  菜单5重复读取一个ADS1256通道并统计稳定性
 * @param  mux          MUX配置
 * @param  mean_raw     多次读取平均原始码
 * @param  min_raw      多次读取最小原始码
 * @param  max_raw      多次读取最大原始码
 * @param  samples_ok   成功读取次数
 * @return 1=所有重复读取成功，0=至少有一次读取失败
 * @note   即使部分读取失败，也保留已有统计值供菜单5定位问题。
 */
static uint8_t Instrument_ReadAdsChannelStats(uint8_t mux,
                                              uint8_t repeat_count,
                                              int32_t *mean_raw,
                                              int32_t *min_raw,
                                              int32_t *max_raw,
                                              uint8_t *samples_ok)
{
    uint8_t repeat;
    uint8_t success_count;
    uint8_t all_ok;
    int32_t sample;
    int64_t sum;

    if ((mean_raw == NULL) || (min_raw == NULL) ||
        (max_raw == NULL) || (samples_ok == NULL))
    {
        return 0U;
    }

    *mean_raw = 0;
    *min_raw = 0;
    *max_raw = 0;
    *samples_ok = 0U;
    success_count = 0U;
    all_ok = 1U;
    sum = 0;

    if (repeat_count == 0U)
    {
        return 0U;
    }

    for (repeat = 0U; repeat < repeat_count; ++repeat)
    {
        if (s_stop_requested != 0U)
        {
            all_ok = 0U;
            break;
        }
        if (ADS1256_ReadAverage(mux, s_measure_config.average_samples,
                                &sample) != HAL_OK)
        {
            all_ok = 0U;
            continue;
        }

        if (success_count == 0U)
        {
            *min_raw = sample;
            *max_raw = sample;
        }
        else
        {
            if (sample < *min_raw)
            {
                *min_raw = sample;
            }
            if (sample > *max_raw)
            {
                *max_raw = sample;
            }
        }

        sum += (int64_t)sample;
        ++success_count;
    }

    if (success_count > 0U)
    {
        *mean_raw = (int32_t)(sum / (int64_t)success_count);
    }
    *samples_ok = success_count;
    return all_ok;
}

/**
 * @brief 计算扫频点的模拟链路稳定等待时间
 * @note 120 ms 对常见 1 uF AD637 CAV 约覆盖其 1% 建立时间；低频点
 *       再按周期数补足，兼顾 AD637 建立时间和题目 2 分钟限制。
 */
static uint32_t Instrument_SweepSettleMs(uint32_t frequency_hz)
{
    uint32_t cycle_settle_ms;

    if (frequency_hz == 0U)
    {
        return CIRCUIT_SETTLE_MS;
    }

    cycle_settle_ms = (1000U * CIRCUIT_SETTLE_CYCLES + frequency_hz - 1U) /
                      frequency_hz;
    return (cycle_settle_ms > CIRCUIT_SETTLE_MS) ?
           cycle_settle_ms : CIRCUIT_SETTLE_MS;
}

/**
 * @brief  读取一个扫频点的双通道测量值，失败后有限重试
 * @param  result    测量结果
 * @param  ads_ready ADS1256 就绪状态
 * @return HAL_OK 或最后一次失败状态
 */
static HAL_StatusTypeDef Instrument_ReadCircuitWithRetry(
    CircuitMeasurement_t *result,
    uint8_t *ads_ready,
    uint8_t *attempts_used)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t attempt;

    if ((result == NULL) || (ads_ready == NULL))
    {
        return HAL_ERROR;
    }
    if (attempts_used != NULL)
    {
        *attempts_used = 0U;
    }

    for (attempt = 0U; attempt < CIRCUIT_MEASUREMENT_RETRY_COUNT; ++attempt)
    {
        if (attempts_used != NULL)
        {
            *attempts_used = (uint8_t)(attempt + 1U);
        }
        Instrument_TryInitAds(ads_ready);
        if (*ads_ready != 0U)
        {
            status = CircuitMeasure_Read(&s_measure_config, result);
            if ((status == HAL_OK) && (result->valid != 0U))
            {
                return HAL_OK;
            }

            /* Treat a HAL_OK result with invalid data as a failed attempt. */
            status = HAL_ERROR;
            *ads_ready = 0U;
        }

        if ((attempt + 1U) < CIRCUIT_MEASUREMENT_RETRY_COUNT)
        {
            vTaskDelay(pdMS_TO_TICKS(CIRCUIT_MEASUREMENT_RETRY_DELAY_MS));
        }
    }

    return status;
}

static HAL_StatusTypeDef Instrument_SetDdsWithRetry(
    uint32_t frequency_hz,
    float amplitude_mv,
    uint32_t phase_deg,
    uint8_t output,
    uint8_t *attempts_used)
{
    HAL_StatusTypeDef status = HAL_ERROR;
    uint8_t attempt;

    if (attempts_used != NULL)
    {
        *attempts_used = 0U;
    }

    for (attempt = 0U; attempt < CIRCUIT_DDS_RETRY_COUNT; ++attempt)
    {
        if (attempts_used != NULL)
        {
            *attempts_used = (uint8_t)(attempt + 1U);
        }
#if CIRCUIT_DDS_SWEEP_USE_SINGLE_CHANNEL
        status = AD9959_SetChannelOutput(CIRCUIT_DDS_SWEEP_CHANNEL,
                                          frequency_hz, amplitude_mv,
                                          phase_deg, output);
#else
        /* 默认走旧的 F1 已验证的四通道同步协议，避免 DDS 实际静音 */
        status = AD9959_SetOutput(frequency_hz, amplitude_mv,
                                   phase_deg, output);
#endif
        if (status == HAL_OK)
        {
            return status;
        }
        if ((attempt + 1U) < CIRCUIT_DDS_RETRY_COUNT)
        {
            vTaskDelay(pdMS_TO_TICKS(AD9959_FRAME_GAP_MS));
        }
    }

    return status;
}

/**
 * @brief Configure and run one known-model operating point.
 *
 * The source amplitude is calculated from the specified H(s); no sweep or
 * post-run manual adjustment is performed.
 */
static void Instrument_RunKnownModel(uint32_t frequency_hz,
                                      float target_vpp,
                                      uint32_t phase_deg,
                                      uint8_t *ads_ready)
{
    UI_Measurement_t measurement;
    CircuitMeasurement_t circuit_measurement;
    HAL_StatusTypeDef dds_status;
    HAL_StatusTypeDef measurement_status = HAL_ERROR;
    uint8_t dds_attempts = 0U;
    uint8_t measurement_attempts = 0U;
    uint32_t settle_ms;
    float model_gain;
    float model_gain_db;
    float source_amplitude_mv;
    float output_vpp = 0.0f;
    float expected_output_vpp = 0.0f;

    if (ads_ready == NULL)
    {
        return;
    }

    if (frequency_hz < UI_MIN_FREQUENCY_HZ)
    {
        frequency_hz = UI_MIN_FREQUENCY_HZ;
    } 
    if (frequency_hz > UI_KNOWN_MAX_FREQUENCY_HZ)
    {
        frequency_hz = UI_KNOWN_MAX_FREQUENCY_HZ;
    }
    if (target_vpp < UI_KNOWN_TARGET_MIN_VPP)
    {
        target_vpp = UI_KNOWN_TARGET_MIN_VPP;
    }
    if (target_vpp > UI_KNOWN_TARGET_MAX_VPP)
    {
        target_vpp = UI_KNOWN_TARGET_MAX_VPP;
    }

    Instrument_ClearStopRequest();
    Instrument_TryInitAds(ads_ready);
    model_gain = Instrument_KnownModelGain(frequency_hz);
    model_gain_db = (model_gain > 0.000001f) ?
                    20.0f * log10f(model_gain) : -120.0f;
    source_amplitude_mv = Instrument_KnownSourceAmplitudeMv(
        frequency_hz, target_vpp);
    settle_ms = Instrument_SweepSettleMs(frequency_hz);

    (void)memset(&measurement, 0, sizeof(measurement));
    (void)memset(&circuit_measurement, 0, sizeof(circuit_measurement));
    measurement.event = UI_EVENT_MEASUREMENT;
    measurement.frequency_hz = frequency_hz;
    measurement.target_output_peak_v = target_vpp;
    measurement.source_amplitude_mv = source_amplitude_mv;
    measurement.gain_db = -120.0f;

    (void)DebugUart6_PrintfBlocking(
        "MENU2_BEGIN,freq_hz=%lu,target_vpp=%.3f,phase_deg=%lu\r\n",
        (unsigned long)frequency_hz, (double)target_vpp,
        (unsigned long)phase_deg);
    (void)DebugUart6_PrintfBlocking(
        "MENU2_MODEL_GAIN=%.9f,source_opa_gain=%.6f,"
        "scope_gain_correction=%.6f,source_cmd_mv=%.6f,settle_ms=%lu\r\n",
        (double)model_gain, (double)source_amplitude_mv,
        (double)MENU2_SCOPE_GAIN_CORRECTION,
        (unsigned long)settle_ms);

    dds_status = Instrument_SetDdsWithRetry(
        frequency_hz, source_amplitude_mv, phase_deg, 1U, &dds_attempts);
    (void)DebugUart6_PrintfBlocking(
        "MENU2_DDS,status=%u,ok=%u,attempts=%u\r\n",
        (unsigned int)dds_status, (unsigned int)(dds_status == HAL_OK),
        (unsigned int)dds_attempts);

    if (dds_status == HAL_OK)
    {
        vTaskDelay(pdMS_TO_TICKS(settle_ms));
        measurement_status = Instrument_ReadCircuitWithRetry(
            &circuit_measurement, ads_ready, &measurement_attempts);
    }

    if ((dds_status == HAL_OK) &&
        (measurement_status == HAL_OK) &&
        (circuit_measurement.valid != 0U))
    {
        measurement.input_rms = circuit_measurement.input_rms;
        measurement.output_rms = circuit_measurement.output_rms;
        measurement.gain_db = circuit_measurement.gain_db;
        measurement.expected_output_rms = measurement.input_rms * model_gain;
        output_vpp = measurement.output_rms * CIRCUIT_RMS_TO_VPP;
        expected_output_vpp = measurement.expected_output_rms *
                              CIRCUIT_RMS_TO_VPP;
        measurement.output_peak_v = output_vpp;
        measurement.expected_output_peak_v = expected_output_vpp;
        measurement.valid = 1U;
    }

    (void)DebugUart6_PrintfBlocking(
        "MENU2_RESULT,freq_hz=%lu,target_vpp=%.6f,source_cmd_mv=%.6f,"
        "output_vpp=%.6f,model_vpp=%.6f,target_error_pct=%.6f,"
        "model_gain_db=%.6f,measured_gain_db=%.6f,valid=%u,"
        "dds_status=%u,measure_status=%u,measure_attempts=%u\r\n",
        (unsigned long)frequency_hz, (double)target_vpp,
        (double)source_amplitude_mv, (double)output_vpp,
        (double)expected_output_vpp,
        (double)((target_vpp > 0.000001f) ?
                 (output_vpp - target_vpp) * 100.0f / target_vpp : 0.0f),
        (double)model_gain_db, (double)measurement.gain_db,
        (unsigned int)measurement.valid, (unsigned int)dds_status,
        (unsigned int)measurement_status, (unsigned int)measurement_attempts);
    (void)Instrument_Publish(&measurement);

    if (Instrument_InferenceStopRequested() != 0U)
    {
        (void)AD9959_StopOutput();
    }
}

static void Instrument_RunInference(void)
{
    uint16_t offset;
    uint16_t index;
    float input_voltage;
    float input_ac;
    float output_ac;
    float output_voltage;
    uint16_t block_min_raw;
    uint16_t block_max_raw;
    uint32_t block_count = 0U;
    uint32_t busy_count = 0U;
    uint32_t last_report_tick = HAL_GetTick();
    HAL_StatusTypeDef dac_status;
    HAL_StatusTypeDef adc_status;

    (void)DebugUart6_Printf(
        "MENU4_INFER_BEGIN,model_type=%u,fs_hz=%lu,adc_channel=2,adc_pin=PA2,"
        "dac_pin=PA4,adc_bias=%.3f,dac_bias=%.3f\r\n",
        (unsigned int)s_circuit_model.type,
        (unsigned long)CIRCUIT_INFERENCE_FS_HZ,
        (double)CIRCUIT_INFERENCE_ADC_BIAS,
        (double)CIRCUIT_INFERENCE_DAC_BIAS);

    /* 初始化 DAC 缓冲区为直流偏置（输出静音） */
    (void)memset(s_inference_dac_buffer, 0x00,
                 sizeof(s_inference_dac_buffer));
    for (index = 0U; index < BSP_ADC_WAVEFORM_BUFFER_LENGTH; ++index)
    {
        s_inference_dac_buffer[index] =
            Instrument_VoltageToDacCode(CIRCUIT_INFERENCE_DAC_BIAS);
    }

    /* 启动 ADC 采集和 DAC 输出 */
    dac_status = BSP_DAC_StartSynchronizedStream(
        s_inference_dac_buffer, BSP_ADC_WAVEFORM_BUFFER_LENGTH);
    adc_status = HAL_ERROR;
    if (dac_status == HAL_OK)
    {
        adc_status = BSP_ADC_WaveformStart(
            (uint32_t)CIRCUIT_INFERENCE_FS_HZ);
    }
    (void)DebugUart6_Printf(
        "MENU4_INFER_START,dac_status=%u,adc_status=%u,"
        "trigger=TIM2_TRGO_SHARED,dma=DMA2_STREAM2\r\n",
        (unsigned int)dac_status,
        (unsigned int)adc_status);

    if ((dac_status != HAL_OK) || (adc_status != HAL_OK))
    {
        /* 启动失败则立即停止并返回 */
        (void)BSP_ADC_WaveformStop();
        (void)BSP_DAC_StopWaveform();
        return;
    }

    /* 主循环：不断读取 ADC 数据块，进行 IIR 运算，填入 DAC 缓冲 */
    for (;;)
    {
        if (Instrument_InferenceStopRequested() != 0U)
        {
            break;
        }

        if (BSP_ADC_WaveformTakeBlock(s_inference_adc_block,
                                      CIRCUIT_INFERENCE_BLOCK_LEN,
                                      &offset) == HAL_OK)
        {
            ++block_count;
            block_min_raw = 4095U;
            block_max_raw = 0U;
            for (index = 0U; index < CIRCUIT_INFERENCE_BLOCK_LEN; ++index)
            {
                if (s_inference_adc_block[index] < block_min_raw)
                {
                    block_min_raw = s_inference_adc_block[index];
                }
                if (s_inference_adc_block[index] > block_max_raw)
                {
                    block_max_raw = s_inference_adc_block[index];
                }
                input_voltage = Instrument_AdcCodeToVoltage(
                    s_inference_adc_block[index]);
                input_ac = input_voltage - CIRCUIT_INFERENCE_ADC_BIAS;
                output_ac = CircuitModel_BiquadProcess(
                    &s_inference_filter, input_ac);
                output_voltage = CIRCUIT_INFERENCE_DAC_BIAS + output_ac;
                s_inference_dac_buffer[offset + index] =
                    Instrument_VoltageToDacCode(output_voltage);
            }
            if ((block_count == 1U) ||
                ((HAL_GetTick() - last_report_tick) >= 1000U))
            {
                (void)DebugUart6_Printf(
                    "MENU4_ADC_BLOCK,count=%lu,raw_min=%u,raw_max=%u,"
                    "raw_span=%u,busy=%lu,overrun=%lu\r\n",
                    (unsigned long)block_count,
                    (unsigned int)block_min_raw,
                    (unsigned int)block_max_raw,
                    (unsigned int)(block_max_raw - block_min_raw),
                    (unsigned long)busy_count,
                    (unsigned long)BSP_ADC_WaveformGetOverrunCount());
                last_report_tick = HAL_GetTick();
            }
            taskYIELD();
        }
        else
        {
            /* Block until the next DMA half/full interrupt instead of losing
             * four blocks in the old 1 ms polling delay at 500 kS/s. */
            ++busy_count;
            (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10U));
        }
    }

    /* 停止流式传输 */
    (void)BSP_ADC_WaveformStop();
    (void)BSP_DAC_StopWaveform();
    (void)DebugUart6_Printf(
        "MENU4_INFER_END,blocks=%lu,busy=%lu,overrun=%lu,"
        "stop_requested=%u\r\n",
        (unsigned long)block_count,
        (unsigned long)busy_count,
        (unsigned long)BSP_ADC_WaveformGetOverrunCount(),
        (unsigned int)s_stop_requested);
}

/**
 * @brief 将建模结果打包并通过队列发布给 UI
 * @param model 指向电路模型数据的指针
 */
/**
 * @brief 打印 PA2 ADC 镜像测试的运行时寄存器，定位“引脚有波形但采样值接近0”。
 */
static void Instrument_PrintAdcMirrorRegisters(const char *stage)
{
    (void)DebugUart6_PrintfBlocking(
        "MIRROR_REG,stage=%s,adc_sr=%08lX,adc_cr1=%08lX,adc_cr2=%08lX,"
        "adc_smpr2=%08lX,adc_sqr1=%08lX,adc_sqr2=%08lX,adc_sqr3=%08lX,"
        "adc_dr=%08lX,adc_ccr=%08lX,dac_cr=%08lX\r\n",
        stage,
        (unsigned long)ADC2->SR,
        (unsigned long)ADC2->CR1,
        (unsigned long)ADC2->CR2,
        (unsigned long)ADC2->SMPR2,
        (unsigned long)ADC2->SQR1,
        (unsigned long)ADC2->SQR2,
        (unsigned long)ADC2->SQR3,
        (unsigned long)ADC2->DR,
        (unsigned long)ADC123_COMMON->CCR,
        (unsigned long)DAC->CR);
    (void)DebugUart6_PrintfBlocking(
        "MIRROR_REG,stage=%s,gpioa_moder=%08lX,gpioa_pupdr=%08lX,"
        "gpioa_idr=%08lX,dma_cr=%08lX,dma_ndtr=%lu,dma_par=%08lX,"
        "dma_m0ar=%08lX,dma_fcr=%08lX,tim2_cr1=%08lX,tim2_cr2=%08lX,"
        "tim2_psc=%lu,tim2_arr=%lu,tim2_cnt=%lu\r\n",
        stage,
        (unsigned long)GPIOA->MODER,
        (unsigned long)GPIOA->PUPDR,
        (unsigned long)GPIOA->IDR,
        (unsigned long)DMA2_Stream2->CR,
        (unsigned long)DMA2_Stream2->NDTR,
        (unsigned long)DMA2_Stream2->PAR,
        (unsigned long)DMA2_Stream2->M0AR,
        (unsigned long)DMA2_Stream2->FCR,
        (unsigned long)TIM2->CR1,
        (unsigned long)TIM2->CR2,
        (unsigned long)TIM2->PSC,
        (unsigned long)TIM2->ARR,
        (unsigned long)TIM2->CNT);
}

/* Menu5 item6: continuously output DAC and print the PA4-to-PA2 ADC test. */
static void Instrument_RunDacAdcMirrorTest(void)
{
    uint16_t index;
    uint16_t half_length;
    uint16_t code;
    uint16_t raw;
    uint16_t expected;
    uint16_t expected_index;
    uint16_t raw_min;
    uint16_t raw_max;
    uint16_t first_raw;
    uint16_t last_raw;
    uint16_t block_offset;
    uint32_t block_count;
    uint32_t busy_count;
    uint32_t total_error;
    uint32_t block_error;
    uint32_t block_mean;
    uint32_t start_tick;
    uint32_t elapsed_ms;
    uint32_t last_vofa_tick;
    float ratio;
    float voltage;
    HAL_StatusTypeDef dac_status;
    HAL_StatusTypeDef adc_status;

    (void)DebugUart6_PrintfBlocking(
        "MIRROR_BEGIN,dac_pin=PA4,adc_pin=PA2,wire=PA4_to_PA2,"
        "sample_rate=%lu,buffer=%u,block=%u\r\n",
        (unsigned long)((uint32_t)CIRCUIT_INFERENCE_FS_HZ),
        (unsigned int)BSP_ADC_WAVEFORM_BUFFER_LENGTH,
        (unsigned int)BSP_ADC_WAVEFORM_BLOCK_LENGTH);

    /* 与菜单4一致：先输出 0.65~2.65 V 的三角波，再启动 ADC+DMA。 */
    half_length = (uint16_t)(BSP_ADC_WAVEFORM_BUFFER_LENGTH / 2U);
    for (index = 0U; index < BSP_ADC_WAVEFORM_BUFFER_LENGTH; ++index)
    {
        if (index < half_length)
        {
            ratio = (half_length > 1U) ?
                    ((float)index / (float)(half_length - 1U)) : 0.0f;
        }
        else
        {
            ratio = (half_length > 1U) ?
                    (1.0f - ((float)(index - half_length) /
                             (float)(half_length - 1U))) : 0.0f;
        }
        voltage = 0.65f + 2.0f * ratio;
        code = Instrument_VoltageToDacCode(voltage);
        s_dac_test_buffer[index] = code;
    }

    /* One decimated frame per four DAC samples keeps the waveform visible
     * without filling the serial link before the ADC test starts. */
    for (index = 0U; index < BSP_ADC_WAVEFORM_BUFFER_LENGTH; index += 4U)
    {
        (void)DebugUart6_VofaPrintfBlocking(
            "MENU5_DAC_WAVE", "%u,%.6f",
            (unsigned int)index,
            (double)((float)s_dac_test_buffer[index] *
                     CIRCUIT_INFERENCE_ADC_VREF / 4095.0f));
    }

    (void)DebugUart6_PrintfBlocking("MIRROR_STAGE,stage=BUFFER_READY\r\n");
    (void)BSP_ADC_WaveformStop();
    (void)DebugUart6_PrintfBlocking("MIRROR_STAGE,stage=ADC_STOP_DONE\r\n");
    (void)BSP_DAC_StopWaveform();
    (void)DebugUart6_PrintfBlocking("MIRROR_STAGE,stage=DAC_STOP_DONE\r\n");
    dac_status = BSP_DAC_StartSynchronizedStream(
        s_dac_test_buffer, BSP_ADC_WAVEFORM_BUFFER_LENGTH);
    (void)DebugUart6_PrintfBlocking(
        "MIRROR_STAGE,stage=DAC_START_DONE,status=%u\r\n",
        (unsigned int)dac_status);
    adc_status = HAL_ERROR;
    if (dac_status == HAL_OK)
    {
        adc_status = BSP_ADC_WaveformStart(
            (uint32_t)CIRCUIT_INFERENCE_FS_HZ);
    }
    (void)DebugUart6_PrintfBlocking(
        "MIRROR_STAGE,stage=ADC_START_DONE,status=%u\r\n",
        (unsigned int)adc_status);
    (void)DebugUart6_PrintfBlocking(
        "MIRROR_START,dac_status=%u,adc_status=%u,tim2=TIM2_TRGO,"
        "dma=DMA2_STREAM2\r\n",
        (unsigned int)dac_status,
        (unsigned int)adc_status);

    if (adc_status == HAL_OK)
    {
        Instrument_PrintAdcMirrorRegisters("START");
    }

    if ((dac_status != HAL_OK) || (adc_status != HAL_OK))
    {
        (void)BSP_ADC_WaveformStop();
        (void)BSP_DAC_StopWaveform();
        (void)DebugUart6_PrintfBlocking("MIRROR_ERROR,stage=START\r\n");
        return;
    }

    block_count = 0U;
    busy_count = 0U;
    total_error = 0U;
    start_tick = HAL_GetTick();
    last_vofa_tick = 0U;
    while (Instrument_InferenceStopRequested() == 0U)
    {
        if (BSP_ADC_WaveformTakeBlock(s_inference_adc_block,
                                      BSP_ADC_WAVEFORM_BLOCK_LENGTH,
                                      &block_offset) != HAL_OK)
        {
            ++busy_count;
            vTaskDelay(pdMS_TO_TICKS(1U));
            continue;
        }

        ++block_count;
        if (block_count == 1U)
        {
            (void)DebugUart6_PrintfBlocking(
                "MIRROR_SAMPLE,buffer0=%u,buffer127=%u,buffer_addr=%08lX,"
                "block_offset=%u\r\n",
                (unsigned int)s_inference_adc_block[0U],
                (unsigned int)s_inference_adc_block[BSP_ADC_WAVEFORM_BLOCK_LENGTH - 1U],
                (unsigned long)(uintptr_t)s_inference_adc_block,
                (unsigned int)block_offset);
            Instrument_PrintAdcMirrorRegisters("FIRST_BLOCK");
        }
        raw_min = 4095U;
        raw_max = 0U;
        first_raw = s_inference_adc_block[0U];
        last_raw = s_inference_adc_block[BSP_ADC_WAVEFORM_BLOCK_LENGTH - 1U];
        block_mean = 0U;
        block_error = 0U;
        for (index = 0U; index < BSP_ADC_WAVEFORM_BLOCK_LENGTH; ++index)
        {
            raw = s_inference_adc_block[index];
            expected_index = (uint16_t)(block_offset + index);
            if (expected_index < half_length)
            {
                ratio = (half_length > 1U) ?
                        ((float)expected_index / (float)(half_length - 1U)) :
                        0.0f;
            }
            else
            {
                ratio = (half_length > 1U) ?
                        (1.0f - ((float)(expected_index - half_length) /
                                 (float)(half_length - 1U))) : 0.0f;
            }
            expected = Instrument_VoltageToDacCode(0.65f + 2.0f * ratio);
            if (raw < raw_min)
            {
                raw_min = raw;
            }
            if (raw > raw_max)
            {
                raw_max = raw;
            }
            block_mean += raw;
            block_error += (raw > expected) ?
                           (uint32_t)(raw - expected) :
                           (uint32_t)(expected - raw);

        }
        block_mean /= BSP_ADC_WAVEFORM_BLOCK_LENGTH;
        total_error += block_error;
        if ((block_count == 1U) ||
            ((HAL_GetTick() - last_vofa_tick) >= 20U))
        {
            (void)DebugUart6_VofaPrintfBlocking(
                "MENU5_MIRROR", "%lu,%u,%u,%lu,%lu,%u,%u,%lu",
                (unsigned long)block_count,
                (unsigned int)raw_min,
                (unsigned int)raw_max,
                (unsigned long)block_mean,
                (unsigned long)(block_error /
                                BSP_ADC_WAVEFORM_BLOCK_LENGTH),
                (unsigned int)first_raw,
                (unsigned int)last_raw,
                (unsigned long)busy_count);
            last_vofa_tick = HAL_GetTick();
        }
        (void)DebugUart6_PrintfBlocking(
            "MIRROR_BLOCK,index=%lu,offset=%u,raw_min=%u,raw_max=%u,"
            "raw_span=%u,raw_mean=%lu,first=%u,last=%u,avg_abs_err=%lu,"
            "busy=%lu\r\n",
            (unsigned long)block_count,
            (unsigned int)block_offset,
            (unsigned int)raw_min,
            (unsigned int)raw_max,
            (unsigned int)(raw_max - raw_min),
            (unsigned long)block_mean,
            (unsigned int)first_raw,
            (unsigned int)last_raw,
            (unsigned long)(block_error / BSP_ADC_WAVEFORM_BLOCK_LENGTH),
            (unsigned long)busy_count);
        taskYIELD();
    }

    elapsed_ms = HAL_GetTick() - start_tick;
    (void)BSP_ADC_WaveformStop();
    (void)BSP_DAC_StopWaveform();
    (void)BSP_DAC_SetValue(Instrument_VoltageToDacCode(DAC_TEST_MID_V));
    (void)DebugUart6_PrintfBlocking(
        "MIRROR_SUMMARY,blocks=%lu,busy=%lu,elapsed_ms=%lu,"
        "total_avg_abs_err=%lu,stop_requested=%u,valid=%u\r\n",
        (unsigned long)block_count,
        (unsigned long)busy_count,
        (unsigned long)elapsed_ms,
        (unsigned long)((block_count != 0U) ?
                        (total_error / (block_count *
                                        BSP_ADC_WAVEFORM_BLOCK_LENGTH)) : 0U),
        (unsigned int)s_stop_requested,
        (unsigned int)((block_count != 0U) ? 1U : 0U));
    (void)DebugUart6_VofaPrintfBlocking(
        "MENU5_MIRROR_SUMMARY", "%lu,%lu,%lu,%lu,%u,%u",
        (unsigned long)block_count,
        (unsigned long)busy_count,
        (unsigned long)elapsed_ms,
        (unsigned long)((block_count != 0U) ?
                        (total_error / (block_count *
                                        BSP_ADC_WAVEFORM_BLOCK_LENGTH)) : 0U),
        (unsigned int)s_stop_requested,
        (unsigned int)((block_count != 0U) ? 1U : 0U));
}

static void Instrument_PublishModel(const CircuitModel_t *model)
{
    UI_Measurement_t result;

    (void)memset(&result, 0, sizeof(result));
    result.event = UI_EVENT_MODEL_RESULT;
    result.model_type = model->type;
    result.f1_hz = model->f1_hz;
    result.f2_hz = model->f2_hz;
    result.center_hz = model->center_hz;
    result.q = model->q;
    result.gain_db = (model->fit_valid != 0U) ?
                     model->model_gain_db : model->peak_gain_db;
    result.valid = (model->type != CIRCUIT_MODEL_UNKNOWN) ? 1U : 0U;
    (void)DebugUart6_Printf(
        "MENU3_MODEL_RESULT,type=%u,name=%s,f1_hz=%.3f,f2_hz=%.3f,"
        "natural_hz=%.3f,q=%.6f,gain_db=%.6f,fit_rms=%.9f,"
        "fit_valid=%u\r\n",
        (unsigned int)model->type,
        CircuitModel_TypeName(model->type),
        (double)model->f1_hz,
        (double)model->f2_hz,
        (double)model->natural_hz,
        (double)model->q,
        (double)result.gain_db,
        (double)model->fit_rms_error,
        (unsigned int)model->fit_valid);
    Instrument_Publish(&result);
}

/**
 * @brief 执行 ADS1256 硬件自检
 *
 * 读取寄存器状态和通道原始值，并通过队列发送给 UI 显示。
 */
static void Instrument_RunAdsTest(void)
{
    UI_Measurement_t result;
    int32_t ain0_mean;
    int32_t ain1_mean;
    int32_t ain0_min;
    int32_t ain0_max;
    int32_t ain1_min;
    int32_t ain1_max;
    uint8_t init_status;
    uint8_t registers_ok;
    uint8_t ain0_all_ok;
    uint8_t ain1_all_ok;
    uint8_t ain0_samples_ok;
    uint8_t ain1_samples_ok;

    (void)memset(&result, 0, sizeof(result));
    registers_ok = 1U;
    ain0_all_ok = 0U;
    ain1_all_ok = 0U;
    result.event = UI_EVENT_ADS_TEST;
    result.ads_test_repeat_count = ADS1256_TEST_REPEAT_COUNT;
    /* 预设寄存器为无效值 */
    result.ads_status_reg = 0xFFU;
    result.ads_mux_reg = 0xFFU;
    result.ads_adcon_reg = 0xFFU;
    result.ads_drate_reg = 0xFFU;

    /* 停止 AD9959 输出，避免干扰 */
    (void)AD9959_StopOutput();
    init_status = (ADS1256_Init() == HAL_OK) ? 1U : 0U;
    result.ads_init_ok = init_status;
    result.ads_drdy_ready = ADS1256_IsDataReady();
    result.ads_pdwn_high = ADS1256_IsPowerDownHigh();

    /* 初始化成功则读取寄存器并采集通道 */
    if (init_status != 0U)
    {
        if (ADS1256_ReadRegister(ADS1256_STATUS,
                                  &result.ads_status_reg) != HAL_OK)
        {
            registers_ok = 0U;
        }
        if (ADS1256_ReadRegister(ADS1256_MUX,
                                  &result.ads_mux_reg) != HAL_OK)
        {
            registers_ok = 0U;
        }
        if (ADS1256_ReadRegister(ADS1256_ADCON,
                                  &result.ads_adcon_reg) != HAL_OK)
        {
            registers_ok = 0U;
        }
        if (ADS1256_ReadRegister(ADS1256_DRATE,
                                  &result.ads_drate_reg) != HAL_OK)
        {
            registers_ok = 0U;
        }

        ain0_all_ok = Instrument_ReadAdsChannelStats(
        ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM,
            ADS1256_TEST_REPEAT_COUNT,
            &ain0_mean, &ain0_min, &ain0_max,
            &ain0_samples_ok);
        result.ads_ain0_samples_ok = ain0_samples_ok;
        if (ain0_samples_ok > 0U)
        {
            result.ads_ain0_raw = ain0_mean;
            result.ads_ain0_min_raw = ain0_min;
            result.ads_ain0_max_raw = ain0_max;
            result.ads_ain0_voltage = ADS1256_CodeToVoltage(
                ain0_mean, s_measure_config.reference_voltage,
                s_measure_config.ads_gain);
        }
        ain1_all_ok = Instrument_ReadAdsChannelStats(
        ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM,
            ADS1256_TEST_REPEAT_COUNT,
            &ain1_mean, &ain1_min, &ain1_max,
            &ain1_samples_ok);
        result.ads_ain1_samples_ok = ain1_samples_ok;
        if (ain1_samples_ok > 0U)
        {
            result.ads_ain1_raw = ain1_mean;
            result.ads_ain1_min_raw = ain1_min;
            result.ads_ain1_max_raw = ain1_max;
            result.ads_ain1_voltage = ADS1256_CodeToVoltage(
                ain1_mean, s_measure_config.reference_voltage,
                s_measure_config.ads_gain);
        }
    }

    result.ads_reg_read_ok = (init_status != 0U && registers_ok != 0U) ?
                             1U : 0U;
    /* 菜单5显示重复采样结束后的DRDY状态，而不是初始化瞬间状态 */
    result.ads_drdy_ready = ADS1256_IsDataReady();

    /* valid 表示本次初始化、寄存器读取和两个 ADC 采样均成功 */
    result.valid = (init_status != 0U && registers_ok != 0U &&
                    ain0_all_ok != 0U && ain1_all_ok != 0U) ? 1U : 0U;
    (void)DebugUart6_Printf("ADS_TEST,init=%u,reg=%u,drdy_ready=%u,pdwn=%u,status=0x%02X,mux=0x%02X,adcon=0x%02X,drate=0x%02X,ain0=%ld,ain1=%ld,ain0_ok=%u,ain1_ok=%u,valid=%u\r\n",
                            (unsigned int)result.ads_init_ok,
                            (unsigned int)result.ads_reg_read_ok,
                            (unsigned int)result.ads_drdy_ready,
                            (unsigned int)result.ads_pdwn_high,
                            (unsigned int)result.ads_status_reg,
                            (unsigned int)result.ads_mux_reg,
                            (unsigned int)result.ads_adcon_reg,
                            (unsigned int)result.ads_drate_reg,
                            (long)result.ads_ain0_raw,
                            (long)result.ads_ain1_raw,
                            (unsigned int)result.ads_ain0_samples_ok,
                            (unsigned int)result.ads_ain1_samples_ok,
                            (unsigned int)result.valid);
    s_ads_test_snapshot = result;


    /* 菜单5只通过USART6输出校准数据，不再把ADS信息送到LCD */
}
/** 打印一组校准通道统计结果。 */
static void Instrument_PrintCalibrationChannel(
    const char *tag,
    uint8_t mux,
    uint8_t all_ok,
    int32_t mean_raw,
    int32_t min_raw,
    int32_t max_raw,
    uint8_t samples_ok)
{
    float mean_voltage;
    float min_voltage;
    float max_voltage;

    mean_voltage = ADS1256_CodeToVoltage(
        mean_raw, s_measure_config.reference_voltage,
        s_measure_config.ads_gain);
    min_voltage = ADS1256_CodeToVoltage(
        min_raw, s_measure_config.reference_voltage,
        s_measure_config.ads_gain);
    max_voltage = ADS1256_CodeToVoltage(
        max_raw, s_measure_config.reference_voltage,
        s_measure_config.ads_gain);

    (void)DebugUart6_VofaPrintfBlocking(
        (tag != NULL) ? tag : "CAL_CHANNEL",
        "%u,%u,%u,%ld,%ld,%ld,%ld,%.9f,%.9f,%.9f,%.9f",
        (unsigned int)mux,
        (unsigned int)all_ok,
        (unsigned int)samples_ok,
        (long)mean_raw,
        (long)min_raw,
        (long)max_raw,
        (long)(max_raw - min_raw),
        (double)mean_voltage,
        (double)min_voltage,
        (double)max_voltage,
        (double)(max_voltage - min_voltage));
}

/** 菜单5项目1：ADS参考配置、原始码和稳定性测试。 */
static void Instrument_RunCalibrationAdsReference(void)
{
    uint8_t init_ok;
    uint8_t reg_ok;
    uint8_t ain0_ok;
    uint8_t ain1_ok;
    uint8_t ain0_samples;
    uint8_t ain1_samples;
    uint8_t value;
    int32_t ain0_mean;
    int32_t ain0_min;
    int32_t ain0_max;
    int32_t ain1_mean;
    int32_t ain1_min;
    int32_t ain1_max;

    (void)AD9959_StopOutput();
    init_ok = (ADS1256_Init() == HAL_OK) ? 1U : 0U;
    reg_ok = 1U;
    ain0_ok = 0U;
    ain1_ok = 0U;
    ain0_samples = 0U;
    ain1_samples = 0U;
    ain0_mean = 0;
    ain0_min = 0;
    ain0_max = 0;
    ain1_mean = 0;
    ain1_min = 0;
    ain1_max = 0;

    (void)DebugUart6_Printf(
        "CAL_ADS_BEGIN,ref_v_config=%.9f,pga=%u,drate=0x%02X,avg_samples=%u,repeat=%u\r\n",
        (double)s_measure_config.reference_voltage,
        (unsigned int)s_measure_config.ads_gain,
        (unsigned int)ADS1256_DRATE_1000SPS,
        (unsigned int)s_measure_config.average_samples,
        (unsigned int)CALIBRATION_REPEAT_COUNT);

    if (init_ok != 0U)
    {
        if (ADS1256_ReadRegister(ADS1256_STATUS, &value) != HAL_OK)
        {
            reg_ok = 0U;
        }
        if (ADS1256_ReadRegister(ADS1256_MUX, &value) != HAL_OK)
        {
            reg_ok = 0U;
        }
        if (ADS1256_ReadRegister(ADS1256_ADCON, &value) != HAL_OK)
        {
            reg_ok = 0U;
        }
        if (ADS1256_ReadRegister(ADS1256_DRATE, &value) != HAL_OK)
        {
            reg_ok = 0U;
        }

        ain0_ok = Instrument_ReadAdsChannelStats(
            ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM,
            CALIBRATION_REPEAT_COUNT,
            &ain0_mean, &ain0_min, &ain0_max, &ain0_samples);
        ain1_ok = Instrument_ReadAdsChannelStats(
            ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM,
            CALIBRATION_REPEAT_COUNT,
            &ain1_mean, &ain1_min, &ain1_max, &ain1_samples);
    }

    (void)DebugUart6_Printf(
        "CAL_ADS_STATUS,init=%u,reg_read=%u,drdy_ready=%u,pdwn_high=%u\r\n",
        (unsigned int)init_ok,
        (unsigned int)reg_ok,
        (unsigned int)ADS1256_IsDataReady(),
        (unsigned int)ADS1256_IsPowerDownHigh());
    (void)DebugUart6_VofaPrintfBlocking(
        "CAL_ADS_STATUS", "%u,%u,%u,%u,%u",
        (unsigned int)init_ok,
        (unsigned int)reg_ok,
        (unsigned int)ADS1256_IsDataReady(),
        (unsigned int)ADS1256_IsPowerDownHigh(),
        (unsigned int)(init_ok != 0U && reg_ok != 0U &&
                       ain0_ok != 0U && ain1_ok != 0U));
    Instrument_PrintCalibrationChannel(
        "CAL_ADS_AIN0", ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM,
        ain0_ok, ain0_mean, ain0_min, ain0_max, ain0_samples);
    Instrument_PrintCalibrationChannel(
        "CAL_ADS_AIN1", ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM,
        ain1_ok, ain1_mean, ain1_min, ain1_max, ain1_samples);
    (void)DebugUart6_Printf(
        "CAL_ADS_SUMMARY,valid=%u\r\n",
        (unsigned int)(init_ok != 0U && reg_ok != 0U &&
                       ain0_ok != 0U && ain1_ok != 0U));
}

/** 菜单5项目2：DDS关闭时完整输入输出检波链路零点测试。 */
static void Instrument_RunCalibrationZeroOffset(void)
{
    uint8_t ads_ready;
    uint8_t ain0_ok;
    uint8_t ain1_ok;
    uint8_t applied;
    uint8_t ain0_samples;
    uint8_t ain1_samples;
    int32_t ain0_mean;
    int32_t ain0_min;
    int32_t ain0_max;
    int32_t ain1_mean;
    int32_t ain1_min;
    int32_t ain1_max;
    float input_offset_voltage;
    float output_offset_voltage;

    (void)AD9959_StopOutput();
    ads_ready = (ADS1256_Init() == HAL_OK) ? 1U : 0U;
    ain0_ok = 0U;
    ain1_ok = 0U;
    ain0_samples = 0U;
    ain1_samples = 0U;
    ain0_mean = 0;
    ain0_min = 0;
    ain0_max = 0;
    ain1_mean = 0;
    ain1_min = 0;
    ain1_max = 0;
    applied = 0U;
    input_offset_voltage = 0.0f;
    output_offset_voltage = 0.0f;

    (void)DebugUart6_Printf(
        "CAL_ZERO_BEGIN,dds=off,repeat=%u,avg_samples=%u,note=connect_both_measurement_inputs_to_zero\r\n",
        (unsigned int)CALIBRATION_REPEAT_COUNT,
        (unsigned int)s_measure_config.average_samples);
    if (ads_ready != 0U)
    {
        ain0_ok = Instrument_ReadAdsChannelStats(
            ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM,
            CALIBRATION_REPEAT_COUNT,
            &ain0_mean, &ain0_min, &ain0_max, &ain0_samples);
        ain1_ok = Instrument_ReadAdsChannelStats(
            ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM,
            CALIBRATION_REPEAT_COUNT,
            &ain1_mean, &ain1_min, &ain1_max, &ain1_samples);
    }
    Instrument_PrintCalibrationChannel(
        "CAL_ZERO_AIN0", ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM,
        ain0_ok, ain0_mean, ain0_min, ain0_max, ain0_samples);
    Instrument_PrintCalibrationChannel(
        "CAL_ZERO_AIN1", ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM,
        ain1_ok, ain1_mean, ain1_min, ain1_max, ain1_samples);

    /*
     * 菜单5校准必须真正作用于菜单3。此前这里只打印结果，菜单3仍
     * 使用编译时旧常量，导致 AD637 零漂变化时低电平被错误钳为0。
     */
    if ((ads_ready != 0U) && (ain0_samples > 0U) && (ain1_samples > 0U))
    {
        input_offset_voltage = ADS1256_CodeToVoltage(
            ain0_mean, s_measure_config.reference_voltage,
            s_measure_config.ads_gain);
        output_offset_voltage = ADS1256_CodeToVoltage(
            ain1_mean, s_measure_config.reference_voltage,
            s_measure_config.ads_gain);
        if ((input_offset_voltage >= 0.0f) &&
            (output_offset_voltage >= 0.0f))
        {
            s_measure_config.input_offset_voltage = input_offset_voltage;
            s_measure_config.output_offset_voltage = output_offset_voltage;
            applied = 1U;
        }
    }
    (void)DebugUart6_Printf(
        "CAL_ZERO_SUMMARY,ads_ready=%u,input_offset_v=%.9f,output_offset_v=%.9f,applied=%u\r\n",
        (unsigned int)ads_ready,
        (double)input_offset_voltage,
        (double)output_offset_voltage,
        (unsigned int)applied);
    (void)DebugUart6_VofaPrintfBlocking(
        "CAL_ZERO", "%u,%.9f,%.9f,%u",
        (unsigned int)ads_ready,
        (double)input_offset_voltage,
        (double)output_offset_voltage,
        (unsigned int)applied);
}

/** 菜单5项目3：直通连接下输入/输出检波通道一致性测试。 */
static void Instrument_RunCalibrationChannelGain(void)
{
    static const float amplitude_levels_mv[CALIBRATION_GAIN_LEVEL_COUNT] =
    {
        100.0f, 250.0f, 400.0f
    };
    CircuitMeasurement_t measurement;
    uint8_t ads_ready;
    uint8_t dds_attempts;
    uint8_t retry_attempts;
    uint8_t index;
    float output_correction;
    float raw_ratio;
    float raw_ratio_db;
    float corrected_gain_db;
    float correction;
    float correction_sum;
    float calibrated_output_gain;
    uint8_t correction_count;
    uint8_t applied;

    ads_ready = 0U;
    correction_sum = 0.0f;
    calibrated_output_gain = s_measure_config.output_ad637_gain;
    correction_count = 0U;
    applied = 0U;
    (void)DebugUart6_Printf(
        "CAL_GAIN_BEGIN,freq=%u,levels=100/250/400mV,note=bypass_dut_and_connect_input_to_output\r\n",
        1000U);

    for (index = 0U; index < CALIBRATION_GAIN_LEVEL_COUNT; ++index)
    {
        if (Instrument_InferenceStopRequested() != 0U)
        {
            (void)DebugUart6_Printf(
                "CAL_GAIN_ABORT,index=%u\r\n", (unsigned int)(index + 1U));
            break;
        }
        dds_attempts = 0U;
        retry_attempts = 0U;
        (void)memset(&measurement, 0, sizeof(measurement));
        (void)Instrument_SetDdsWithRetry(
            1000U, amplitude_levels_mv[index], 0U, 1U, &dds_attempts);
        vTaskDelay(pdMS_TO_TICKS(CIRCUIT_SETTLE_MS));
        if (Instrument_ReadCircuitWithRetry(
                &measurement, &ads_ready, &retry_attempts) != HAL_OK)
        {
            (void)DebugUart6_Printf(
                "CAL_GAIN_RESULT,index=%u,amp_cmd_mv=%.3f,dds_try=%u,retry=%u,valid=0,reason=MEASURE_FAIL\r\n",
                (unsigned int)(index + 1U),
                (double)amplitude_levels_mv[index],
                (unsigned int)dds_attempts,
                (unsigned int)retry_attempts);
            (void)DebugUart6_VofaPrintfBlocking(
                "CAL_GAIN", "%u,%.3f,0,0,0,0,0,0",
                (unsigned int)(index + 1U),
                (double)amplitude_levels_mv[index]);
            continue;
        }

        raw_ratio = (measurement.input_detector_voltage > 0.000001f) ?
                    measurement.output_detector_voltage /
                    measurement.input_detector_voltage : 0.0f;
        raw_ratio_db = (raw_ratio > 0.000001f) ?
                       20.0f * log10f(raw_ratio) : -120.0f;
        output_correction = (measurement.output_detector_voltage >
                             0.000001f) ?
                            measurement.input_detector_voltage /
                            measurement.output_detector_voltage : 0.0f;
        corrected_gain_db = (measurement.input_rms > 0.000001f &&
                             output_correction > 0.000001f) ?
                             20.0f * log10f(
                                 measurement.output_rms * output_correction /
                                 measurement.input_rms) : -120.0f;

        /* 直通测试中，两路真实 RMS 应相等；用中高幅值点估计输出链路
         * 的比例误差，避免 100mV 点接近 AD637 零点时污染校准结果。 */
        if ((index > 0U) && (measurement.input_rms > 0.01f) &&
            (measurement.output_rms > 0.01f))
        {
            correction = measurement.input_rms / measurement.output_rms;
            if ((correction >= 0.5f) && (correction <= 2.0f))
            {
                correction_sum += correction;
                ++correction_count;
            }
        }

        (void)DebugUart6_Printf(
            "CAL_GAIN_ADC,index=%u,amp_cmd_mv=%.3f,dds_try=%u,retry=%u,input_raw=%ld,output_raw=%ld,input_adc_v=%.9f,output_adc_v=%.9f\r\n",
            (unsigned int)(index + 1U),
            (double)amplitude_levels_mv[index],
            (unsigned int)dds_attempts,
            (unsigned int)retry_attempts,
            (long)measurement.input_raw_code,
            (long)measurement.output_raw_code,
            (double)measurement.input_detector_voltage,
            (double)measurement.output_detector_voltage);
        (void)DebugUart6_Printf(
            "CAL_GAIN_RESULT,index=%u,input_rms=%.9f,output_rms=%.9f,raw_ratio=%.9f,raw_ratio_db=%.6f,output_correction=%.9f,corrected_gain_db=%.6f,valid=%u\r\n",
            (unsigned int)(index + 1U),
            (double)measurement.input_rms,
            (double)measurement.output_rms,
            (double)raw_ratio,
            (double)raw_ratio_db,
            (double)output_correction,
             (double)corrected_gain_db,
             (unsigned int)measurement.valid);
        (void)DebugUart6_VofaPrintfBlocking(
            "CAL_GAIN", "%u,%.3f,%.9f,%.9f,%.9f,%.9f,%.9f,%u",
            (unsigned int)(index + 1U),
            (double)amplitude_levels_mv[index],
            (double)measurement.input_rms,
            (double)measurement.output_rms,
            (double)raw_ratio,
            (double)output_correction,
            (double)corrected_gain_db,
            (unsigned int)measurement.valid);
    }
    (void)AD9959_StopOutput();
    if (correction_count > 0U)
    {
        calibrated_output_gain = s_measure_config.output_ad637_gain *
                                 (correction_sum /
                                  (float)correction_count);
        if ((calibrated_output_gain >= 0.5f) &&
            (calibrated_output_gain <= 2.0f))
        {
            s_measure_config.output_ad637_gain = calibrated_output_gain;
            applied = 1U;
        }
    }
    (void)DebugUart6_Printf(
        "CAL_GAIN_APPLY,output_ad637_gain=%.9f,samples=%u,applied=%u\r\n",
        (double)s_measure_config.output_ad637_gain,
        (unsigned int)correction_count,
        (unsigned int)applied);
    (void)DebugUart6_VofaPrintfBlocking(
        "CAL_GAIN_APPLY", "%.9f,%u,%u",
        (double)s_measure_config.output_ad637_gain,
        (unsigned int)correction_count,
        (unsigned int)applied);
    (void)DebugUart6_Printf("CAL_GAIN_END\r\n");
}

static void Instrument_RunAdsDiagnosticSweep(void)
{
    UI_Measurement_t measurement;
    CircuitMeasurement_t circuit_measurement;
    InstrumentSweepStats_t stats;
    uint8_t ads_ready = 0U;
    uint8_t retry_attempts;
    uint8_t dds_attempts;
    uint8_t aborted = 0U;
    uint16_t index;
    uint32_t start_tick;
    uint32_t frequency_hz;
    uint32_t dds_status;
    uint32_t ads_status;
    uint32_t settle_ms;
    HAL_StatusTypeDef dds_hal_status;
    uint8_t ads_ready_before;
    uint8_t append_ok;
    const char *reason;

    (void)memset(&stats, 0, sizeof(stats));

    start_tick = HAL_GetTick();

    (void)DebugUart6_Printf(
        "ADS_SWEEP_BEGIN,start=%lu,stop=%lu,points=%u,amplitude_mv=%.2f,dds_mode=%s,dds_ch=%u,avg=%u,ref_v=%.6f,input_mux=0x%02X,output_mux=0x%02X,drate=0x%02X,input_offset_v=%.9f,output_offset_v=%.9f,input_gain=%.6f,output_gain=%.6f,settle_ms=%u\r\n",
        (unsigned long)CIRCUIT_SWEEP_START_HZ,
        (unsigned long)CIRCUIT_SWEEP_STOP_HZ,
        (unsigned int)CIRCUIT_SWEEP_POINTS,
        (double)CIRCUIT_SOURCE_AMPLITUDE_MV,
        CIRCUIT_DDS_SWEEP_MODE_TEXT,
        (unsigned int)CIRCUIT_DDS_SWEEP_CHANNEL,
        (unsigned int)s_measure_config.average_samples,
        (double)s_measure_config.reference_voltage,
        (unsigned int)s_measure_config.input_mux,
        (unsigned int)s_measure_config.output_mux,
        (unsigned int)ADS1256_DRATE_1000SPS,
        (double)s_measure_config.input_offset_voltage,
        (double)s_measure_config.output_offset_voltage,
        (double)s_measure_config.input_ad637_gain,
        (double)s_measure_config.output_ad637_gain,
        (unsigned int)CIRCUIT_SETTLE_MS);

    for (index = 0U; index < CIRCUIT_SWEEP_POINTS; ++index)
    {
        frequency_hz = Instrument_SweepFrequency(index);
        if (Instrument_InferenceStopRequested() != 0U)
        {
            aborted = 1U;
            break;
        }

        ++stats.attempted_points;
        retry_attempts = 0U;
        dds_attempts = 0U;
        reason = "OK";
        ads_status = 0U;
        settle_ms = Instrument_SweepSettleMs(frequency_hz);
        ads_ready_before = ads_ready;
        append_ok = 0U;
        measurement = s_ads_test_snapshot;
        (void)memset(&circuit_measurement, 0, sizeof(circuit_measurement));
        measurement.event = UI_EVENT_ADS_TEST;
        measurement.frequency_hz = frequency_hz;
        measurement.sweep_index = (uint16_t)(index + 1U);
        measurement.sweep_total = CIRCUIT_SWEEP_POINTS;
        measurement.gain_db = -120.0f;
        measurement.input_rms = 0.0f;
        measurement.output_rms = 0.0f;
        measurement.valid = 0U;

        (void)DebugUart6_Printf(
            "ADS_SWEEP_STAGE,index=%u,freq=%lu,tick=%lu,stack_free=%u,stage=DDS_BEGIN\r\n",
            (unsigned int)(index + 1U),
            (unsigned long)frequency_hz,
            (unsigned long)HAL_GetTick(),
            (unsigned int)uxTaskGetStackHighWaterMark(NULL));
        dds_hal_status = Instrument_SetDdsWithRetry(
            frequency_hz, CIRCUIT_SOURCE_AMPLITUDE_MV, 0U, 1U,
            &dds_attempts);
        if (dds_attempts > 1U)
        {
            stats.dds_retry_total = (uint16_t)(stats.dds_retry_total +
                                               dds_attempts - 1U);
        }
        dds_status = (dds_hal_status == HAL_OK) ? 1U : 0U;
        (void)DebugUart6_Printf(
            "ADS_SWEEP_STAGE,index=%u,freq=%lu,tick=%lu,stack_free=%u,stage=DDS_DONE,status=%u,try=%u\r\n",
            (unsigned int)(index + 1U),
            (unsigned long)frequency_hz,
            (unsigned long)HAL_GetTick(),
            (unsigned int)uxTaskGetStackHighWaterMark(NULL),
            (unsigned int)dds_hal_status,
            (unsigned int)dds_attempts);
        if (dds_status == 0U)
        {
            ++stats.dds_fail_points;
            reason = "DDS_TX_FAIL";
        }
        else
        {
            (void)DebugUart6_Printf(
                "ADS_SWEEP_STAGE,index=%u,freq=%lu,tick=%lu,stack_free=%u,stage=SETTLE_BEGIN,settle_ms=%lu\r\n",
                (unsigned int)(index + 1U),
                (unsigned long)frequency_hz,
                (unsigned long)HAL_GetTick(),
                (unsigned int)uxTaskGetStackHighWaterMark(NULL),
                (unsigned long)settle_ms);
            vTaskDelay(pdMS_TO_TICKS(settle_ms));
            (void)DebugUart6_Printf(
                "ADS_SWEEP_STAGE,index=%u,freq=%lu,tick=%lu,stage=SETTLE_DONE\r\n",
                (unsigned int)(index + 1U),
                (unsigned long)frequency_hz,
                (unsigned long)HAL_GetTick());
            (void)DebugUart6_Printf(
                "ADS_SWEEP_STAGE,index=%u,freq=%lu,tick=%lu,stage=ADS_BEGIN,ready=%u\r\n",
                (unsigned int)(index + 1U),
                (unsigned long)frequency_hz,
                (unsigned long)HAL_GetTick(),
                (unsigned int)ads_ready);
            ads_status = (Instrument_ReadCircuitWithRetry(
                              &circuit_measurement, &ads_ready,
                              &retry_attempts) == HAL_OK) ? 1U : 0U;
            (void)DebugUart6_Printf(
                "ADS_SWEEP_STAGE,index=%u,freq=%lu,tick=%lu,stage=ADS_DONE,status=%lu,ready=%u,retry=%u\r\n",
                (unsigned int)(index + 1U),
                (unsigned long)frequency_hz,
                (unsigned long)HAL_GetTick(),
                (unsigned long)ads_status,
                (unsigned int)ads_ready,
                (unsigned int)retry_attempts);
            if (retry_attempts > 1U)
            {
                stats.retry_total = (uint16_t)(stats.retry_total +
                                                retry_attempts - 1U);
            }

            if (ads_status == 0U)
            {
                ++stats.ads_fail_points;
                if (retry_attempts >= CIRCUIT_MEASUREMENT_RETRY_COUNT)
                {
                    ++stats.retry_exhausted;
                    reason = "ADS_RETRY_EXHAUSTED";
                }
                else
                {
                    reason = "ADS_READ_FAIL";
                }
            }
            else if (circuit_measurement.valid == 0U)
            {
                ++stats.invalid_points;
                reason = "MEAS_INVALID";
            }
            else if (stats.valid_points >= CIRCUIT_MODEL_MAX_POINTS)
            {
                ++stats.append_fail_points;
                reason = "MODEL_APPEND_FAIL";
            }
            else if ((circuit_measurement.input_rms <= 0.000001f) ||
                     (circuit_measurement.output_rms < 0.0f))
            {
                ++stats.append_fail_points;
                reason = "MODEL_APPEND_FAIL";
            }
            else
            {
                ++stats.valid_points;
                append_ok = 1U;
                measurement.input_rms = circuit_measurement.input_rms;
                measurement.output_rms = circuit_measurement.output_rms;
                measurement.gain_db = circuit_measurement.gain_db;
                measurement.valid = 1U;
            }
        }

        stats.missing_points = (uint16_t)(CIRCUIT_SWEEP_POINTS -
                                          stats.valid_points);
        (void)DebugUart6_Printf(
            "ADS_SWEEP_CTRL,index=%u,freq=%lu,settle_ms=%lu,dds=%lu,dds_try=%u,dds_hal=%u,dds_ok=%u,ads_ready_before=%u,ads_ready_after=%u\r\n",
            (unsigned int)(index + 1U),
            (unsigned long)frequency_hz,
            (unsigned long)settle_ms,
            (unsigned long)dds_status,
            (unsigned int)dds_attempts,
            (unsigned int)dds_hal_status,
            (unsigned int)(dds_hal_status == HAL_OK),
            (unsigned int)ads_ready_before,
            (unsigned int)ads_ready);
        (void)DebugUart6_Printf(
            "ADS_SWEEP_ADC,index=%u,ads=%lu,retry=%u,input_raw=%ld,output_raw=%ld,input_adc_v=%.6f,output_adc_v=%.6f\r\n",
            (unsigned int)(index + 1U),
            (unsigned long)ads_status,
            (unsigned int)retry_attempts,
            (long)circuit_measurement.input_raw_code,
            (long)circuit_measurement.output_raw_code,
            (double)circuit_measurement.input_detector_voltage,
            (double)circuit_measurement.output_detector_voltage);
        (void)DebugUart6_Printf(
            "ADS_SWEEP_RESULT,index=%u,freq=%lu,input=%.6f,output=%.6f,gain=%.6f,gain_db=%.3f,meas_valid=%u,append=%u,model_count=%u,reason=%s\r\n",
            (unsigned int)(index + 1U),
            (unsigned long)frequency_hz,
            (double)circuit_measurement.input_rms,
            (double)circuit_measurement.output_rms,
            (double)circuit_measurement.gain,
            (double)circuit_measurement.gain_db,
            (unsigned int)circuit_measurement.valid,
            (unsigned int)append_ok,
            (unsigned int)stats.valid_points,
            reason);
        /* 保留旧格式，便于已有串口脚本继续解析。 */
        (void)DebugUart6_Printf(
            "ADS_SWEEP_POINT,index=%u,freq=%lu,dds=%lu,dds_try=%u,dds_hal=%u,ads=%lu,retry=%u,input=%.6f,output=%.6f,gain_db=%.3f,valid=%u,reason=%s\r\n",
            (unsigned int)(index + 1U),
            (unsigned long)frequency_hz,
            (unsigned long)dds_status,
            (unsigned int)dds_attempts,
            (unsigned int)dds_hal_status,
            (unsigned long)ads_status,
            (unsigned int)retry_attempts,
            (double)circuit_measurement.input_rms,
            (double)circuit_measurement.output_rms,
            (double)circuit_measurement.gain_db,
            (unsigned int)measurement.valid,
            reason);
        (void)DebugUart6_VofaPrintfBlocking(
            "MENU5_SWEEP", "%.3f,%.9f,%.9f,%.9f,%u,%u",
            (double)frequency_hz,
            (double)circuit_measurement.input_rms,
            (double)circuit_measurement.output_rms,
            (double)circuit_measurement.gain_db,
            (unsigned int)circuit_measurement.valid,
            (unsigned int)append_ok);
        Instrument_PublishAdsProgress(&measurement);
    }

    stats.aborted = aborted;
    stats.missing_points = (uint16_t)(CIRCUIT_SWEEP_POINTS -
                                      stats.valid_points);
    stats.elapsed_ms = HAL_GetTick() - start_tick;
    (void)AD9959_StopOutput();

    (void)DebugUart6_Printf(
        "ADS_SWEEP_SUMMARY,expected=%u,attempted=%u,valid=%u,missing=%u,dds_fail=%u,dds_retry_total=%u,ads_fail=%u,invalid=%u,append_fail=%u,retry_total=%u,retry_exhausted=%u,aborted=%u,elapsed_ms=%lu,debug_overflow=%lu,pending=%u\r\n",
        (unsigned int)CIRCUIT_SWEEP_POINTS,
        (unsigned int)stats.attempted_points,
        (unsigned int)stats.valid_points,
        (unsigned int)stats.missing_points,
        (unsigned int)stats.dds_fail_points,
        (unsigned int)stats.dds_retry_total,
        (unsigned int)stats.ads_fail_points,
        (unsigned int)stats.invalid_points,
        (unsigned int)stats.append_fail_points,
        (unsigned int)stats.retry_total,
        (unsigned int)stats.retry_exhausted,
        (unsigned int)stats.aborted,
        (unsigned long)stats.elapsed_ms,
        (unsigned long)DebugUart6_GetOverflowCount(),
        (unsigned int)DebugUart6_GetPendingBytes());
    (void)DebugUart6_VofaPrintfBlocking(
        "MENU5_SWEEP_SUMMARY", "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%lu,%lu,%u",
        (unsigned int)CIRCUIT_SWEEP_POINTS,
        (unsigned int)stats.attempted_points,
        (unsigned int)stats.valid_points,
        (unsigned int)stats.missing_points,
        (unsigned int)stats.dds_fail_points,
        (unsigned int)stats.dds_retry_total,
        (unsigned int)stats.ads_fail_points,
        (unsigned int)stats.invalid_points,
        (unsigned int)stats.append_fail_points,
        (unsigned int)stats.retry_total,
        (unsigned int)stats.retry_exhausted,
        (unsigned int)stats.aborted,
        (unsigned long)stats.elapsed_ms,
        (unsigned long)DebugUart6_GetOverflowCount(),
        (unsigned int)DebugUart6_GetPendingBytes());

    (void)DebugUart6_Printf(
        "ADS_SWEEP_MODEL_CHECK,count=%u,complete=%u,note=menu5_does_not_overwrite_menu3_model\r\n",
        (unsigned int)stats.valid_points,
        (unsigned int)(stats.aborted == 0U &&
                       stats.valid_points == CIRCUIT_SWEEP_POINTS));

    measurement = s_ads_test_snapshot;
    measurement.event = UI_EVENT_ADS_TEST;
    measurement.sweep_index = stats.attempted_points;
    measurement.sweep_total = CIRCUIT_SWEEP_POINTS;
    measurement.valid = (stats.aborted == 0U &&
                         stats.missing_points == 0U) ? 1U : 0U;
    Instrument_PublishAdsProgress(&measurement);
}

/**
 * @brief Print the model and IIR design information used by menu 4.
 * @note  Uses a temporary filter, so the live menu 4 filter state is unchanged.
 */
static void Instrument_RunIirDebug(void)
{
    CircuitBiquad_t debug_filter;
    float filter_frequency;
    float filter_q;
    float linear_gain;
    float impulse_input;
    float impulse_output;
    uint8_t model_ready;
    uint8_t filter_ready;
    uint8_t index;

    model_ready = (s_circuit_model.type != CIRCUIT_MODEL_UNKNOWN &&
                   s_circuit_model.count > 0U) ? 1U : 0U;
    (void)DebugUart6_Printf(
        "IIR_DEBUG_BEGIN,model_ready=%u,model_type=%u,model_name=%s,count=%u,expected_points=%u\r\n",
        (unsigned int)model_ready,
        (unsigned int)s_circuit_model.type,
        CircuitModel_TypeName(s_circuit_model.type),
        (unsigned int)s_circuit_model.count,
        (unsigned int)CIRCUIT_SWEEP_POINTS);
    (void)DebugUart6_Printf(
        "IIR_MODEL_PARAM,f1_hz=%.6f,f2_hz=%.6f,center_hz=%.6f,"
        "q=%.6f,peak_gain_db=%.6f,natural_hz=%.6f,"
        "model_gain_db=%.6f,fit_rms=%.9f,fit_valid=%u\r\n",
        (double)s_circuit_model.f1_hz,
        (double)s_circuit_model.f2_hz,
        (double)s_circuit_model.center_hz,
        (double)s_circuit_model.q,
        (double)s_circuit_model.peak_gain_db,
        (double)s_circuit_model.natural_hz,
        (double)s_circuit_model.model_gain_db,
        (double)s_circuit_model.fit_rms_error,
        (unsigned int)s_circuit_model.fit_valid);
    (void)DebugUart6_Printf(
        "IIR_SWEEP_CONFIG,start_hz=%lu,stop_hz=%lu,points=%u,settle_ms=%u,dds_mode=%s,dds_ch=%u\r\n",
        (unsigned long)CIRCUIT_SWEEP_START_HZ,
        (unsigned long)CIRCUIT_SWEEP_STOP_HZ,
        (unsigned int)CIRCUIT_SWEEP_POINTS,
        (unsigned int)CIRCUIT_SETTLE_MS,
        CIRCUIT_DDS_SWEEP_MODE_TEXT,
        (unsigned int)CIRCUIT_DDS_SWEEP_CHANNEL);

    if (model_ready == 0U)
    {
        (void)DebugUart6_Printf("IIR_DEBUG_ERROR,reason=MODEL_NOT_READY\r\n");
        (void)DebugUart6_VofaPrintfBlocking("MENU5_IIR_STATUS", "0,0");
        (void)DebugUart6_Printf("IIR_DEBUG_END,valid=0\r\n");
        return;
    }

    /* Keep this selection identical to the menu 4 InstrumentTask path. */
    filter_frequency =
        ((s_circuit_model.fit_valid != 0U) &&
         (s_circuit_model.natural_hz > 0.0f)) ?
        s_circuit_model.natural_hz :
        ((s_circuit_model.center_hz > 0.0f) ?
         s_circuit_model.center_hz : s_circuit_model.f1_hz);
    filter_q = (s_circuit_model.q > 0.0f) ? s_circuit_model.q : 0.707f;
    linear_gain = powf(
        10.0f,
        ((s_circuit_model.fit_valid != 0U) ?
         s_circuit_model.model_gain_db :
         s_circuit_model.peak_gain_db) / 20.0f);
    (void)DebugUart6_Printf(
        "IIR_DESIGN_INPUT,type=%u,design_freq_hz=%.6f,design_q=%.6f,fs_hz=%.3f,linear_gain=%.9f\r\n",
        (unsigned int)s_circuit_model.type,
        (double)filter_frequency,
        (double)filter_q,
        (double)CIRCUIT_INFERENCE_FS_HZ,
        (double)linear_gain);

    (void)memset(&debug_filter, 0, sizeof(debug_filter));
    filter_ready = CircuitModel_BiquadInit(
        &debug_filter,
        s_circuit_model.type,
        CIRCUIT_INFERENCE_FS_HZ,
        filter_frequency,
        filter_q);
    if (filter_ready != 0U)
    {
        CircuitModel_BiquadSetGain(&debug_filter, linear_gain);
        (void)DebugUart6_VofaPrintfBlocking(
            "MENU5_IIR_CONFIG", "%u,%.6f,%.6f,%.3f,%.9f,%u",
            (unsigned int)s_circuit_model.type,
            (double)filter_frequency,
            (double)filter_q,
            (double)CIRCUIT_INFERENCE_FS_HZ,
            (double)debug_filter.gain,
            (unsigned int)filter_ready);
        (void)DebugUart6_Printf(
            "IIR_COEFF,b0=%.9f,b1=%.9f,b2=%.9f,a1=%.9f,a2=%.9f,gain=%.9f\r\n",
            (double)debug_filter.b0,
            (double)debug_filter.b1,
            (double)debug_filter.b2,
            (double)debug_filter.a1,
            (double)debug_filter.a2,
            (double)debug_filter.gain);
        (void)DebugUart6_Printf(
            "IIR_STATE,x1=%.9f,x2=%.9f,y1=%.9f,y2=%.9f\r\n",
            (double)debug_filter.x1,
            (double)debug_filter.x2,
            (double)debug_filter.y1,
            (double)debug_filter.y2);
        for (index = 0U; index < 8U; ++index)
        {
            impulse_input = (index == 0U) ? 1.0f : 0.0f;
            impulse_output = CircuitModel_BiquadProcess(
                &debug_filter, impulse_input);
            (void)DebugUart6_VofaPrintfBlocking(
                "MENU5_IIR", "%u,%.9f,%.9f",
                (unsigned int)index,
                (double)impulse_input,
                (double)impulse_output);
            (void)DebugUart6_Printf(
                "IIR_IMPULSE,index=%u,input=%.1f,output=%.9f\r\n",
                (unsigned int)index,
                (double)impulse_input,
                (double)impulse_output);
        }
    }
    else
    {
        (void)DebugUart6_VofaPrintfBlocking("MENU5_IIR_STATUS", "1,0");
        (void)DebugUart6_Printf(
            "IIR_DEBUG_ERROR,reason=BIQUAD_INIT_FAIL\r\n");
    }

    if (filter_ready != 0U)
    {
        (void)DebugUart6_VofaPrintfBlocking("MENU5_IIR_STATUS", "1,1");
    }

    (void)DebugUart6_Printf(
        "IIR_RUNTIME_CONFIG,fs_hz=%.3f,adc_vref=%.6f,adc_bias=%.6f,dac_bias=%.6f,block_len=%u,adc_buffer_len=%u\r\n",
        (double)CIRCUIT_INFERENCE_FS_HZ,
        (double)CIRCUIT_INFERENCE_ADC_VREF,
        (double)CIRCUIT_INFERENCE_ADC_BIAS,
        (double)CIRCUIT_INFERENCE_DAC_BIAS,
        (unsigned int)CIRCUIT_INFERENCE_BLOCK_LEN,
        (unsigned int)BSP_ADC_WAVEFORM_BUFFER_LENGTH);
    (void)DebugUart6_Printf(
        "IIR_RUNTIME_NOTE,menu4_trigger=TIM2_TRGO_SHARED,"
        "wake=ADC_DMA_TASK_NOTIFY\r\n");
    (void)DebugUart6_Printf(
        "IIR_DEBUG_END,valid=%u\r\n",
        (unsigned int)(filter_ready != 0U));
}

/* ================================================================
 * InstrumentTask：主仪器控制任务
 *
 * 独占 AD9959 和 ADS1256，保证串行化访问模拟外设。
 * 处理来自 UiTask 下发的命令，执行相应测量或控制逻辑。
 * ================================================================ */
static void InstrumentTask(void *argument)
{
    UI_Command_t command;
    UI_Measurement_t measurement;
    CircuitMeasurement_t circuit_measurement;
    uint8_t ads_ready;
    uint8_t dds_attempts;
    uint16_t index;

    (void)argument;
    /* 初始化 ADS1256 并标记就绪状态 */
    ads_ready = (ADS1256_Init() == HAL_OK) ? 1U : 0U;
    DDSDataInit();                           /* 初始化 AD9959 数据结构 */
    (void)AD9959_StopOutput();               /* 确保 DDS 输出关闭 */
    CircuitModel_Reset(&s_circuit_model);    /* 清空建模历史数据 */
    /* 题目要求上电后 5 s 内具备输出：默认 1 kHz、约 1 V 峰值 */
    (void)AD9959_SetOutput(UI_DEFAULT_FREQUENCY_HZ, 125.0f, 0U, 1U);

    for (;;)
    {
        /* 阻塞等待命令队列（无限等待） */
        if (xQueueReceive(s_command_queue, &command, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        /* ---------- 正弦信号输出 ---------- */
        if (command.type == UI_COMMAND_SET_SINE)
        {
            (void)AD9959_SetOutput(command.frequency_hz,
                                   command.amplitude_mv,
                                   command.phase_deg,
                                   1U);
        }
        /* ---------- ADS1256 测试 ---------- */
        else if (command.type == UI_COMMAND_TEST_ADS1256)
        {
            Instrument_ClearStopRequest();
            Instrument_RunAdsTest();
            Instrument_RunAdsDiagnosticSweep();
        }
        /* ---------- 菜单5四类串口校准测试 ---------- */
        else if (command.type == UI_COMMAND_RUN_CALIBRATION)
        {
            Instrument_ClearStopRequest();
            DebugUart6_SetOutputEnabled(1U);
            (void)DebugUart6_VofaPrintfBlocking(
                "MENU5_ITEM", "%u",
                (unsigned int)command.calibration_item);
            switch (command.calibration_item)
            {
                case UI_CALIBRATION_ADS_REFERENCE:
                    Instrument_RunCalibrationAdsReference();
                    break;
                case UI_CALIBRATION_ZERO_OFFSET:
                    Instrument_RunCalibrationZeroOffset();
                    break;
                case UI_CALIBRATION_CHANNEL_GAIN:
                    Instrument_RunCalibrationChannelGain();
                    break;
                case UI_CALIBRATION_SWEEP:
                    (void)DebugUart6_Printf(
                        "CAL_SWEEP_BEGIN,mode=FULL_CIRCUIT_MENU3_MIRROR\r\n");
                    Instrument_RunAdsDiagnosticSweep();
                    break;
                case UI_CALIBRATION_IIR_DEBUG:
                    Instrument_RunIirDebug();
                    break;
                case UI_CALIBRATION_DAC_TEST:
                    /* 菜单五功能6：DAC 连续输出，同时打印 PA4->PA2 ADC 数据。 */
                    Instrument_RunDacAdcMirrorTest();
                    break;
                default:
                    (void)DebugUart6_VofaPrintfBlocking(
                        "MENU5_ERROR", "%u",
                        (unsigned int)command.calibration_item);
                    (void)DebugUart6_Printf(
                        "CAL_ERROR,unknown_item=%u\r\n",
                        (unsigned int)command.calibration_item);
                    break;
            }
            DebugUart6_SetOutputEnabled(0U);
        }
        /* ---------- 停止输出 ---------- */
        else if (command.type == UI_COMMAND_STOP_OUTPUT)
        {
            (void)AD9959_StopOutput();
            Instrument_ClearStopRequest();
        }
        /* ---------- 菜单2：已知模型二维步进扫频 ---------- */
        else if (command.type == UI_COMMAND_RUN_KNOWN_MODEL)
        {
            Instrument_RunKnownModel(command.frequency_hz,
                                     command.target_output_peak_v,
                                     command.phase_deg, &ads_ready);
        }
        /* ---------- 未知电路对数扫频建模 ---------- */
        else if (command.type == UI_COMMAND_START_UNKNOWN_MODEL)
        {
            uint8_t aborted = 0U;

            Instrument_ClearStopRequest();
            CircuitModel_Reset(&s_circuit_model);
            for (index = 0U; index < CIRCUIT_SWEEP_POINTS; ++index)
            {
                uint32_t frequency_hz = Instrument_SweepFrequency(index);

                /* 检查是否收到停止命令 */
                if (Instrument_InferenceStopRequested() != 0U)
                {
                    aborted = 1U;
                    break;
                }

                (void)memset(&measurement, 0, sizeof(measurement));
                measurement.event = UI_EVENT_SWEEP_PROGRESS;
                measurement.frequency_hz = frequency_hz;
                measurement.sweep_index = index + 1U;
                measurement.sweep_total = CIRCUIT_SWEEP_POINTS;
                /* 无效频点使用明确的底值，不能让结构体清零值冒出 0 dB。 */
                measurement.gain_db = -120.0f;

                if (Instrument_SetDdsWithRetry(frequency_hz,
                                               CIRCUIT_SOURCE_AMPLITUDE_MV,
                                               0U,
                                               1U,
                                               &dds_attempts) == HAL_OK)
                {
                    vTaskDelay(pdMS_TO_TICKS(
                        Instrument_SweepSettleMs(frequency_hz)));
                    if (Instrument_ReadCircuitWithRetry(
                            &circuit_measurement, &ads_ready, NULL) == HAL_OK)
                    {
                        /* 将测量点加入模型历史 */
                        if (CircuitModel_Append(
                                &s_circuit_model, frequency_hz,
                                circuit_measurement.input_rms,
                                circuit_measurement.output_rms) != 0U)
                        {
                            measurement.input_rms =
                                circuit_measurement.input_rms;
                            measurement.output_rms =
                                circuit_measurement.output_rms;
                            measurement.gain_db = circuit_measurement.gain_db;
                            measurement.valid = 1U;
                        }
                    }
                }
                Instrument_PublishAdsProgress(&measurement);
            }
            (void)AD9959_StopOutput();
            /* 若未中止，则执行模型分析并发布结果 */
            if ((aborted == 0U) &&
                (s_circuit_model.count == CIRCUIT_SWEEP_POINTS))
            {
                (void)CircuitModel_Analyze(&s_circuit_model);
                Instrument_PublishModel(&s_circuit_model);
            }
            else if (aborted == 0U)
            {
                /* 存在未成功采集的频点时禁止用不完整数据生成模型。 */
                s_circuit_model.type = CIRCUIT_MODEL_UNKNOWN;
                Instrument_PublishModel(&s_circuit_model);
            }
        }
        /* ---------- 启动 IIR 推理输出 ---------- */
        else if (command.type == UI_COMMAND_START_INFERENCE)
        {
            Instrument_ClearStopRequest();
            (void)memset(&measurement, 0, sizeof(measurement));
            measurement.event = UI_EVENT_STATUS;
            measurement.model_type = s_circuit_model.type;
            measurement.f1_hz = s_circuit_model.f1_hz;
            measurement.f2_hz = s_circuit_model.f2_hz;
            measurement.center_hz = s_circuit_model.center_hz;
            measurement.q = s_circuit_model.q;
            if (s_circuit_model.type != CIRCUIT_MODEL_UNKNOWN)
            {
                /*
                 * A second-order prototype is designed from its pole/natural
                 * frequency, not from a -3 dB crossing unless Q = 1/sqrt(2).
                 */
                float filter_frequency =
                    ((s_circuit_model.fit_valid != 0U) &&
                     (s_circuit_model.natural_hz > 0.0f)) ?
                    s_circuit_model.natural_hz :
                    ((s_circuit_model.center_hz > 0.0f) ?
                     s_circuit_model.center_hz : s_circuit_model.f1_hz);
                float filter_q = (s_circuit_model.q > 0.0f) ?
                                  s_circuit_model.q : 0.707f; /* 默认 Butterworth Q */

                /* 初始化 IIR 系数 */
                measurement.valid = CircuitModel_BiquadInit(
                    &s_inference_filter, s_circuit_model.type,
                    CIRCUIT_INFERENCE_FS_HZ, filter_frequency, filter_q);
                if (measurement.valid != 0U)
                {
                    /* 设置滤波器增益 */
                    CircuitModel_BiquadSetGain(
                        &s_inference_filter,
                        powf(10.0f,
                             ((s_circuit_model.fit_valid != 0U) ?
                              s_circuit_model.model_gain_db :
                              s_circuit_model.peak_gain_db) / 20.0f));
                    (void)DebugUart6_Printf(
                        "MENU4_IIR_CONFIG,type=%u,design_freq_hz=%.6f,"
                        "design_q=%.6f,fs_hz=%.3f,linear_gain=%.9f,"
                        "fit_valid=%u\r\n",
                        (unsigned int)s_circuit_model.type,
                        (double)filter_frequency,
                        (double)filter_q,
                        (double)CIRCUIT_INFERENCE_FS_HZ,
                        (double)s_inference_filter.gain,
                        (unsigned int)s_circuit_model.fit_valid);
                    (void)DebugUart6_Printf(
                        "MENU4_IIR_COEFF,b0=%.9f,b1=%.9f,b2=%.9f,"
                        "a1=%.9f,a2=%.9f,gain=%.9f\r\n",
                        (double)s_inference_filter.b0,
                        (double)s_inference_filter.b1,
                        (double)s_inference_filter.b2,
                        (double)s_inference_filter.a1,
                        (double)s_inference_filter.a2,
                        (double)s_inference_filter.gain);
                    (void)AD9959_StopOutput();   /* 推理时关闭 DDS */
                    Instrument_Publish(&measurement);
                    Instrument_RunInference();    /* 进入持续推理循环，直到用户停止 */
                    continue;                    /* 推理结束后继续等待下一条命令 */
                }
            }
            Instrument_Publish(&measurement);
        }
    }
}
