/**
 * @file    DAC.c
 * @brief   DDS 信号发生器 DAC 驱动
 *
 * 基于 STM32 DAC + DMA + TIM6 触发，实现任意波形（正弦、三角、锯齿、方波）
 * 以及外部缓冲流式输出。支持动态波形切换和采样率自动分段选择。
 */

#include "DAC.h"
#include <math.h>

/* ================================================================
 * 硬件参数常量
 * ================================================================ */

/** TIM6 定时器时钟频率 (84MHz) */
#define DDS_TIMER_CLOCK_HZ 84000000U
/** DAC 最大峰峰值电压，等于参考电压 (3.3V) */
#define DDS_MAX_VPP DDS_DAC_VREF_VOLTS
/** Keep the DAC trigger rate within a conservative bench-test range. */
#define DDS_MAX_SAMPLE_RATE_HZ 4000000U
#define DDS_MIN_TABLE_LENGTH   32U
#define DDS_MAX_TABLE_LENGTH   128U

/* ================================================================
 * 外设句柄与全局状态
 * ================================================================ */

DAC_HandleTypeDef hdac;               /**< DAC 外设句柄 */
TIM_HandleTypeDef htim6;              /**< TIM6 定时器句柄（触发 DAC） */
ddsStruct ddsStructData;              /**< DDS 波形数据与参数结构体 */

static DMA_HandleTypeDef hdma_dac1;   /**< DAC 通道1 DMA 句柄 */
static uint8_t dds_initialized;       /**< DDS 数据结构是否已初始化 */
static volatile uint8_t dds_running;  /**< 波形输出是否正在运行 */
static uint8_t s_dac_external_timer;  /**< 1 when TIM8 (owned by ADC) triggers DAC */

/* ================================================================
 * 波形生成函数（归一化至 ±1 范围）
 * ================================================================ */

/**
 * @brief 正弦波生成
 * @param sample  当前采样点序号
 * @param period  周期总采样点数
 * @param duty_cycle 占空比（未使用）
 * @return 归一化幅值 [-1, 1]
 */
static double DDS_Sine(uint32_t sample, uint32_t period, double duty_cycle)
{
    (void)duty_cycle;
    return sin((2.0 * 3.14159265358979323846 * sample) / period);
}

/**
 * @brief 三角波生成
 * @param sample  当前采样点序号
 * @param period  周期总采样点数
 * @param duty_cycle 占空比（未使用）
 * @return 归一化幅值 [-1, 1]
 */
static double DDS_Triangle(uint32_t sample, uint32_t period, double duty_cycle)
{
    double phase;

    (void)duty_cycle;
    phase = (double)sample / period;
    if (phase < 0.25)
    {
        return phase * 4.0;            /* 上升沿 0→1 */
    }
    if (phase < 0.75)
    {
        return 2.0 - phase * 4.0;      /* 下降沿 1→-1 */
    }
    return phase * 4.0 - 4.0;          /* 上升沿 -1→0 */
}

/**
 * @brief 锯齿波生成
 * @param sample  当前采样点序号
 * @param period  周期总采样点数
 * @param duty_cycle 占空比（未使用）
 * @return 归一化幅值 [-1, 1]
 */
static double DDS_Sawtooth(uint32_t sample, uint32_t period, double duty_cycle)
{
    (void)duty_cycle;
    return (2.0 * sample / period) - 1.0;  /* 线性增长 -1→1 */
}

/**
 * @brief 方波生成
 * @param sample     当前采样点序号
 * @param period     周期总采样点数
 * @param duty_cycle 占空比 (%)，决定高电平时间比例
 * @return 高电平 1.0，低电平 -1.0
 */
static double DDS_Square(uint32_t sample, uint32_t period, double duty_cycle)
{
    return ((double)sample / period < duty_cycle / 100.0) ? 1.0 : -1.0;
}

/* ================================================================
 * 采样长度自动选择（频率自适应）
 * ================================================================ */

/**
 * @brief 根据输出频率选择最优波形表长度
 *
 * 高频使用较少点数以降低定时器频率，低频使用较多点数以保证波形平滑度。
 * @param frequency_hz 输出频率 (Hz)
 * @return 波形表采样点数 (32/64/128/512)
 */
static uint16_t DDS_SelectLength(uint32_t frequency_hz)
{
    uint16_t best_length = DDS_MIN_TABLE_LENGTH;
    uint32_t best_error = 0xFFFFFFFFU;
    uint16_t length;

    /*
     * The output frequency is timer_clock / divider / table_length. Search
     * the small practical table range instead of using a coarse fixed table;
     * this makes integer timer-divider error negligible at test frequencies.
     */
    for (length = DDS_MIN_TABLE_LENGTH;
         length <= DDS_MAX_TABLE_LENGTH;
         ++length)
    {
        uint32_t sample_rate_hz;
        uint32_t divider;
        uint32_t product;
        uint32_t error;

        if (frequency_hz >
            (DDS_MAX_SAMPLE_RATE_HZ / (uint32_t)length))
        {
            continue;
        }
        sample_rate_hz = frequency_hz * (uint32_t)length;

        divider = (DDS_TIMER_CLOCK_HZ + sample_rate_hz / 2U) /
                  sample_rate_hz;
        if (divider == 0U)
        {
            divider = 1U;
        }
        product = sample_rate_hz * divider;
        error = (product > DDS_TIMER_CLOCK_HZ) ?
                (product - DDS_TIMER_CLOCK_HZ) :
                (DDS_TIMER_CLOCK_HZ - product);

        /* Prefer more points at low frequency when the error is tied. */
        if ((error < best_error) ||
            ((error == best_error) && (length > best_length)))
        {
            best_error = error;
            best_length = length;
        }
    }

    return best_length;
}

/* ================================================================
 * DDS 数据结构初始化
 * ================================================================ */

/**
 * @brief 首次使用前初始化 DDS 全局数据结构
 *
 * 填充默认值并清零波形缓冲，仅执行一次。
 */
static void DDS_InitializeData(void)
{
    uint16_t index;

    if (dds_initialized != 0U)
    {
        return;                        /* 已初始化则直接返回 */
    }

    /* 清零缓冲区 */
    for (index = 0U; index < DDS_BUFFER_LENGTH; ++index)
    {
        ddsStructData.data[index] = 0U;
    }
    ddsStructData.length = 128U;       /* 默认 128 点 */
    ddsStructData.vpp = 2.0f;          /* 默认 2.0Vpp */
    ddsStructData.hz = 1000U;          /* 默认 1kHz */
    ddsStructData.wave = SINWAVE;      /* 默认正弦波 */
    ddsStructData.dutycycle = 50U;     /* 默认 50% 占空比 */
    ddsStructData.createWaveData = DDS_Sine;
    dds_initialized = 1U;
}

/* ================================================================
 * TIM6 定时器配置（产生 DAC 触发信号）
 * ================================================================ */

/**
 * @brief 配置 TIM6 产生指定采样率的更新事件（TRGO）用于触发 DAC DMA
 * @param sample_rate_hz 采样率 (Hz)
 * @return HAL_OK 或 HAL_ERROR
 */
static HAL_StatusTypeDef DDS_ConfigureTimer(uint32_t sample_rate_hz)
{
    TIM_MasterConfigTypeDef master_config = {0};
    uint32_t divider;
    uint32_t prescaler;
    uint32_t period;

    if (sample_rate_hz == 0U)
    {
        return HAL_ERROR;
    }

    /* 工程未提供 HAL_TIM_Base_MspInit，TIM6 时钟在此显式打开。 */
    __HAL_RCC_TIM6_CLK_ENABLE();
    /* 计算分频器与自动重载值，使定时器频率尽量接近采样率 */
    divider = (DDS_TIMER_CLOCK_HZ + sample_rate_hz / 2U) / sample_rate_hz;
    if (divider == 0U)
    {
        divider = 1U;
    }
    prescaler = (divider - 1U) / 65536U;   /* 预分频器值（0 表示不分频） */
    period = divider / (prescaler + 1U);    /* 自动重载值 */
    if (period == 0U)
    {
        period = 1U;
    }

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = prescaler;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = period - 1U;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 配置主模式：定时器更新事件作为 TRGO 输出，触发 DAC */
    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    return HAL_TIMEx_MasterConfigSynchronization(&htim6, &master_config);
}

/* ================================================================
 * DAC 底层初始化和波形控制 API
 * ================================================================ */

/**
 * @brief 初始化 DAC 及默认 DDS 数据，配置 TIM6 触发模式
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef BSP_DAC_Init(void)
{
    DAC_ChannelConfTypeDef channel_config = {0};

    DDS_InitializeData();
    hdac.Instance = DAC;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * 通道配置：TIM6 TRGO 触发，开启输出缓冲。
     * PA4 需要直接驱动 ADC1/PA1 的采样电容；关闭缓冲时示波器的高阻
     * 探头仍能看到波形，但 ADC 采样瞬间会把高阻 DAC 输出拉到接近 0V。
     */
    channel_config.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &channel_config, DAC_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 启动 DAC 通道（但不立即输出波形） */
    return HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
}

/**
 * @brief 停止当前波形输出（关闭定时器和 DMA）
 * @return HAL_OK
 */
HAL_StatusTypeDef BSP_DAC_StopWaveform(void)
{
    /* hdac 已初始化不等于 DAC DMA 已启动，避免复位后无条件 Abort DMA。 */
    if (dds_running == 0U)
    {
        return HAL_OK;
    }

    if ((s_dac_external_timer == 0U) && (htim6.Instance == TIM6))
    {
        (void)HAL_TIM_Base_Stop(&htim6);
    }
    if (hdac.Instance == DAC)
    {
        (void)HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    }
    dds_running = 0U;
    s_dac_external_timer = 0U;
    return HAL_OK;
}

/**
 * @brief 设置 DAC 输出固定电压值（停止波形模式）
 * @param value 12 位 DAC 码 (0-4095)
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef BSP_DAC_SetValue(uint16_t value)
{
    DAC_ChannelConfTypeDef channel_config = {0};
    HAL_StatusTypeDef status;

    if (value > 4095U)
    {
        value = 4095U;
    }
    status = BSP_DAC_StopWaveform();
    if (status != HAL_OK)
    {
        return status;
    }
    if (BSP_DAC_Init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * A fixed dc output must not retain the TIM6 trigger configured by
     * BSP_DAC_Init().  With trigger mode enabled, a DHR write is transferred
     * to the output only after a timer event; AD8367 VGAIN needs the new
     * voltage to appear immediately even when TIM6 is stopped.
     */
    channel_config.DAC_Trigger = DAC_TRIGGER_NONE;
    channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(
            &hdac, &channel_config, DAC_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    return HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
}

/**
 * @brief 启动指定参数的内置波形生成（正弦、三角、锯齿、方波）
 *
 * 根据频率自动选择采样点数，填充波形表并通过 DMA + TIM6 循环输出。
 * @param vpp          峰峰值电压 (0 ~ DDS_MAX_VPP)
 * @param frequency_hz 输出频率 (Hz)
 * @param duty_cycle   占空比 (%)
 * @param wave         波形类型枚举
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef BSP_DAC_StartWaveform(double vpp, uint32_t frequency_hz,
                                        double duty_cycle, waveEnum wave)
{
    uint16_t index;
    uint16_t code;
    double normalized;
    double sample;
    uint32_t sample_rate_hz;

    if ((frequency_hz == 0U) || (wave > SQUAREWAVE))
    {
        return HAL_ERROR;
    }

    DDS_InitializeData();

    /* 参数限幅 */
    if (vpp < 0.0)         vpp = 0.0;
    else if (vpp > DDS_MAX_VPP) vpp = DDS_MAX_VPP;
    if (duty_cycle < 0.0)  duty_cycle = 0.0;
    else if (duty_cycle > 100.0) duty_cycle = 100.0;

    /* 选择波形生成函数 */
    switch (wave)
    {
        case SINWAVE:      ddsStructData.createWaveData = DDS_Sine;      break;
        case TRIANGLEWAE:  ddsStructData.createWaveData = DDS_Triangle;  break;
        case SAWTOOTHWAVE: ddsStructData.createWaveData = DDS_Sawtooth;  break;
        case SQUAREWAVE:   ddsStructData.createWaveData = DDS_Square;    break;
        default:           return HAL_ERROR;
    }

    /* 根据频率确定采样点数并填充波形缓冲区 */
    ddsStructData.length = DDS_SelectLength(frequency_hz);
    ddsStructData.vpp = (float)vpp;
    ddsStructData.hz = frequency_hz;
    ddsStructData.wave = wave;
    ddsStructData.dutycycle = (uint16_t)duty_cycle;

    for (index = 0U; index < ddsStructData.length; ++index)
    {
        sample = ddsStructData.createWaveData(index, ddsStructData.length, duty_cycle);
        /* 将 ±1 归一化值映射到 0~Vpp 区间，再转为 12 位 DAC 码 */
        normalized = ((sample + 1.0) * 0.5) * vpp / DDS_DAC_VREF_VOLTS;
        if (normalized <= 0.0)          code = 0U;
        else if (normalized >= 1.0)     code = 4095U;
        else                            code = (uint16_t)(normalized * 4095.0 + 0.5);
        ddsStructData.data[index] = code;
    }

    sample_rate_hz = frequency_hz * ddsStructData.length;   /* DAC 触发频率 = 输出频率 × 每周期点数 */
    if ((sample_rate_hz / ddsStructData.length != frequency_hz) ||  /* 溢出检查 */
        (BSP_DAC_StopWaveform() != HAL_OK) || (BSP_DAC_Init() != HAL_OK))
    {
        return HAL_ERROR;
    }

    /* 配置定时器触发频率并启动 DMA 循环传输 */
    if (DDS_ConfigureTimer(sample_rate_hz) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)ddsStructData.data,
                          ddsStructData.length, DAC_ALIGN_12B_R) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
    {
        (void)HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        return HAL_ERROR;
    }

    dds_running = 1U;
    s_dac_external_timer = 0U;
    return HAL_OK;
}

/**
 * @brief 启动外部缓冲区流式输出（用户提供的波形数据）
 *
 * 使用 DMA 循环/单次模式，通过 TIM6 触发 DAC 连续输出。
 * @param sample_rate_hz 采样率 (Hz)，即 DAC 触发频率
 * @param buffer         波形数据缓冲区（12 位码值）
 * @param length         缓冲区长度
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef BSP_DAC_StartStream(uint32_t sample_rate_hz,
                                      uint16_t *buffer,
                                      uint16_t length)
{
    DAC_ChannelConfTypeDef channel_config = {0};

    if ((sample_rate_hz == 0U) || (buffer == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    (void)BSP_DAC_StopWaveform();
    if ((hdac.Instance != DAC) && (BSP_DAC_Init() != HAL_OK))
    {
        return HAL_ERROR;
    }
    /* Restore TIM6 after a synchronized TIM2 stream was used previously. */
    channel_config.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &channel_config, DAC_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (DDS_ConfigureTimer(sample_rate_hz) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)buffer,
                          length, DAC_ALIGN_12B_R) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
    {
        (void)HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
        return HAL_ERROR;
    }
    dds_running = 1U;
    s_dac_external_timer = 0U;
    return HAL_OK;
}

/*
 * Arm DAC DMA without starting a private timer. TIM8 TRGO is configured and
 * started by BSP_ADC_WaveformStart(), so ADC sampling and DAC updates have the
 * same clock edge and their ping-pong buffer halves cannot drift or be offset
 * by peripheral initialization latency.
 */
HAL_StatusTypeDef BSP_DAC_StartSynchronizedStream(uint16_t *buffer,
                                                  uint16_t length)
{
    DAC_ChannelConfTypeDef channel_config = {0};

    if ((buffer == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    (void)BSP_DAC_StopWaveform();
    if ((hdac.Instance != DAC) && (BSP_DAC_Init() != HAL_OK))
    {
        return HAL_ERROR;
    }

    channel_config.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
    channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &channel_config, DAC_CHANNEL_1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)buffer,
                          length, DAC_ALIGN_12B_R) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_dac_external_timer = 1U;
    dds_running = 1U;
    return HAL_OK;
}

/* ================================================================
 * HAL DAC MSP 回调（时钟、GPIO、DMA 初始化与反初始化）
 * ================================================================ */

/**
 * @brief DAC 外设 MSP 初始化（由 HAL_DAC_Init 调用）
 *
 * 使能时钟、配置 PA4 为模拟输出、初始化 DMA1_Stream5 循环模式。
 * @param handle DAC 句柄
 */
void HAL_DAC_MspInit(DAC_HandleTypeDef *handle)
{
    GPIO_InitTypeDef gpio = {0};

    if (handle->Instance != DAC)
    {
        return;
    }

    /* 使能时钟 */
    __HAL_RCC_DAC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* PA4: DAC 输出引脚，模拟模式 */
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* DMA 配置：存储器到外设，半字，循环模式 */
    hdma_dac1.Instance = DMA1_Stream5;
    hdma_dac1.Init.Channel = DMA_CHANNEL_7;
    hdma_dac1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1.Init.Mode = DMA_CIRCULAR;          /* 循环模式：自动重复传输 */
    hdma_dac1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dac1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_dac1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_dac1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_dac1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    (void)HAL_DMA_DeInit(&hdma_dac1);
    (void)HAL_DMA_Init(&hdma_dac1);

    __HAL_LINKDMA(handle, DMA_Handle1, hdma_dac1);
}

/**
 * @brief DAC 外设 MSP 反初始化
 * @param handle DAC 句柄
 */
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *handle)
{
    if (handle->Instance == DAC)
    {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
        (void)HAL_DMA_DeInit(&hdma_dac1);
        __HAL_RCC_DAC_CLK_DISABLE();
    }
}

/* ================================================================
 * 以下为兼容旧版接口的包装函数
 * ================================================================ */

void timer6Init(uint32_t sample_rate_hz)
{
    (void)DDS_ConfigureTimer(sample_rate_hz);
}

void DAC1_Init(void)
{
    (void)BSP_DAC_Init();
}

void DAC1_Vol_Set(uint16_t value)
{
    (void)BSP_DAC_SetValue(value);
}

/** 关闭 DAC 输出与定时器（不关闭 DAC 通道本身） */
void dacClose(void)
{
    if (htim6.Instance == TIM6)
    {
        (void)HAL_TIM_Base_Stop(&htim6);
    }
    if (hdac.Instance == DAC)
    {
        (void)HAL_DAC_Stop(&hdac, DAC_CHANNEL_1);
    }
}

/** 重新打开 DAC 通道，若之前为运行状态则恢复定时器 */
void dacOpen(void)
{
    if (hdac.Instance == DAC)
    {
        (void)HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    }
    if ((dds_running != 0U) && (htim6.Instance == TIM6))
    {
        (void)HAL_TIM_Base_Start(&htim6);
    }
}

void dacInit(void)
{
    (void)BSP_DAC_Init();
}

void setDDS(double vpp, uint32_t frequency_hz, double duty_cycle, waveEnum wave)
{
    (void)BSP_DAC_StartWaveform(vpp, frequency_hz, duty_cycle, wave);
}
