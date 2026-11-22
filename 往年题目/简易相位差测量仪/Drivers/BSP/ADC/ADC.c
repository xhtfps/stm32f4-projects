/**
 * @file    ADC.c
 * @brief   ADC驱动实现文件
 * @note    包含ADC初始化、单次/多次采样、波形捕获等功能
 */

#include "ADC.h"
#include <string.h>

/*==============================================================================
 * 宏定义
 *============================================================================*/

/** @brief ADC单次转换超时时间(毫秒) */
#define BSP_ADC_TIMEOUT_MS 10U

/** @brief 12位ADC满量程值(0~4095) */
#define BSP_ADC_FULL_SCALE 4095.0

/** @brief ADC参考电压(伏特) */
#define BSP_ADC_VREF_VOLTS 3.3

/** @brief 波形捕获使用的ADC通道(ADC2通道2，PA2) */
#define BSP_ADC_WAVEFORM_CHANNEL ADC_CHANNEL_2

/** @brief 定时器时钟频率(84MHz) */
#define BSP_ADC_TIMER_CLOCK_HZ 84000000U

/*==============================================================================
 * 全局变量
 *============================================================================*/

/** @brief ADC1句柄 */
ADC_HandleTypeDef hadc1;

/** @brief ADC2句柄 */
ADC_HandleTypeDef hadc2;

/** @brief ADC2波形捕获DMA句柄 */
DMA_HandleTypeDef hdma_adc2_waveform;

/** @brief 波形捕获定时器句柄(用于触发ADC) */
static TIM_HandleTypeDef s_adc_timer;

/** @brief ADC波形数据缓冲区(DMA循环写入) */
static uint16_t s_adc_waveform_buffer[BSP_ADC_WAVEFORM_BUFFER_LENGTH];

/**
 * @brief 波形数据就绪标志
 * @note  bit0: 前半个缓冲区就绪
 *        bit1: 后半个缓冲区就绪
 */
static volatile uint8_t s_adc_waveform_ready;

/** @brief DMA 双半缓冲取数时的下一优先半区，避免前半区长期饥饿 */
static uint8_t s_adc_waveform_next_half;

/** @brief 波形捕获运行状态(0:停止, 1:运行中) */
static volatile uint8_t s_adc_waveform_running;
static volatile uint32_t s_adc_waveform_overrun_count;

/*==============================================================================
 * 内部辅助函数
 *============================================================================*/

/**
 * @brief  通用ADC句柄初始化函数
 * @param  hadc    : ADC句柄指针
 * @param  instance: ADC外设基地址(如ADC1)
 * @return HAL_StatusTypeDef: HAL状态
 * @note   配置ADC为软件触发、单次转换模式
 */
static HAL_StatusTypeDef BSP_ADC_InitHandle(ADC_HandleTypeDef *hadc,
                                             ADC_TypeDef *instance)
{
    hadc->Instance = instance;

    /* 时钟分频: 4分频 */
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;

    /* 12位分辨率 */
    hadc->Init.Resolution = ADC_RESOLUTION_12B;

    /* 单通道模式(非扫描) */
    hadc->Init.ScanConvMode = DISABLE;

    /* 单次转换模式(非连续) */
    hadc->Init.ContinuousConvMode = DISABLE;

    /* 非间断模式 */
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.NbrOfDiscConversion = 0U;

    /* 软件触发(无外部触发) */
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;

    /* 数据右对齐 */
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;

    /* 单次转换通道数量为1 */
    hadc->Init.NbrOfConversion = 1U;

    /* DMA连续请求禁止(单次模式不使用) */
    hadc->Init.DMAContinuousRequests = DISABLE;

    /* 每次转换结束即置转换完成标志 */
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    return HAL_ADC_Init(hadc);
}

/*==============================================================================
 * 公共接口函数 - 基本ADC操作
 *============================================================================*/

/**
 * @brief  ADC1初始化
 * @return HAL_StatusTypeDef: HAL状态
 */
HAL_StatusTypeDef BSP_ADC1_Init(void)
{
    return BSP_ADC_InitHandle(&hadc1, ADC1);
}

/**
 * @brief  ADC2初始化
 * @return HAL_StatusTypeDef: HAL状态
 */
HAL_StatusTypeDef BSP_ADC2_Init(void)
{
    return BSP_ADC_InitHandle(&hadc2, ADC2);
}

/**
 * @brief  单次ADC采样(阻塞模式)
 * @param  hadc   : ADC句柄
 * @param  channel: ADC通道号
 * @param  value  : 输出采样值
 * @return HAL_StatusTypeDef: HAL状态
 * @note   操作流程: 配置通道 -> 启动ADC -> 等待转换完成 -> 读取结果 -> 停止ADC
 */
HAL_StatusTypeDef BSP_ADC_Read(ADC_HandleTypeDef *hadc, uint32_t channel,
                               uint32_t *value)
{
    ADC_ChannelConfTypeDef channel_config = {0};
    HAL_StatusTypeDef status;

    /* 参数有效性检查 */
    if ((hadc == NULL) || (value == NULL))
    {
        return HAL_ERROR;
    }

    /* 配置ADC通道 */
    channel_config.Channel = channel;
    channel_config.Rank = 1U;                         /* 规则序列排第1 */
    channel_config.SamplingTime = ADC_SAMPLETIME_84CYCLES; /* 采样时间84周期 */
    channel_config.Offset = 0U;
    status = HAL_ADC_ConfigChannel(hadc, &channel_config);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 启动ADC */
    status = HAL_ADC_Start(hadc);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 轮询等待转换完成 */
    status = HAL_ADC_PollForConversion(hadc, BSP_ADC_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        *value = HAL_ADC_GetValue(hadc);    /* 读取转换结果 */
    }

    /* 停止ADC(无论转换是否成功) */
    (void)HAL_ADC_Stop(hadc);
    return status;
}

/**
 * @brief  多次ADC采样求平均值
 * @param  hadc    : ADC句柄
 * @param  channel : ADC通道号
 * @param  samples : 采样次数
 * @param  value   : 输出平均值
 * @return HAL_StatusTypeDef: HAL状态
 */
HAL_StatusTypeDef BSP_ADC_ReadAverage(ADC_HandleTypeDef *hadc, uint32_t channel,
                                      uint16_t samples, uint32_t *value)
{
    uint64_t sum = 0U;           /* 累加和(64位防溢出) */
    uint32_t sample;
    uint16_t index;
    HAL_StatusTypeDef status;

    /* 参数有效性检查 */
    if ((samples == 0U) || (value == NULL))
    {
        return HAL_ERROR;
    }

    /* 进行samples次单次采样并累加 */
    for (index = 0U; index < samples; ++index)
    {
        status = BSP_ADC_Read(hadc, channel, &sample);
        if (status != HAL_OK)
        {
            return status;
        }
        sum += sample;
    }

    /* 计算算术平均值 */
    *value = (uint32_t)(sum / samples);
    return HAL_OK;
}

/*==============================================================================
 * 内部辅助函数 - 波形捕获相关
 *============================================================================*/

/**
 * @brief  配置波形捕获定时器(TIM2)用于触发ADC
 * @param  sample_rate_hz: 采样频率(Hz)
 * @return HAL_StatusTypeDef: HAL状态
 * @note   定时器输出TRGO更新事件, 作为ADC外部触发源
 */
static HAL_StatusTypeDef BSP_ADC_ConfigureWaveformTimer(uint32_t sample_rate_hz)
{
    TIM_MasterConfigTypeDef master_config = {0};
    uint32_t period;

    if (sample_rate_hz == 0U)
    {
        return HAL_ERROR;
    }

    /* 计算定时器自动重装载值: 定时器频率/采样频率 */
    period = BSP_ADC_TIMER_CLOCK_HZ / sample_rate_hz;
    if (period == 0U)
    {
        return HAL_ERROR;
    }

    /* TIM2位于APB1，波形采集必须显式开启定时器时钟。 */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 配置TIM2基本参数 */
    s_adc_timer.Instance = TIM2;
    s_adc_timer.Init.Prescaler = 0U;                       /* 无预分频 */
    s_adc_timer.Init.CounterMode = TIM_COUNTERMODE_UP;    /* 向上计数 */
    s_adc_timer.Init.Period = period - 1U;                 /* 自动重装值 */
    s_adc_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_adc_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_adc_timer) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 配置主模式: TRGO输出更新事件 */
    master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    return HAL_TIMEx_MasterConfigSynchronization(&s_adc_timer,
                                                 &master_config);
}

/**
 * @brief  配置ADC2 DMA传输(循环模式)
 * @return HAL_StatusTypeDef: HAL状态
 * @note   DMA2_Stream2, 通道1, 半字传输, 循环模式, 高优先级
 */
static HAL_StatusTypeDef BSP_ADC_ConfigureWaveformDma(void)
{
    /* 使能DMA2时钟 */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /*
     * 首次启动时 DMA 句柄尚未配置，Instance 为 0。HAL_DMA_DeInit()
     * 会先执行 IS_DMA_STREAM_ALL_INSTANCE(Instance) 断言，不能对这样
     * 的句柄直接调用，否则在开启断言的工程中会停在 assert_failed()。
     * 只有已经配置过 DMA2_Stream2 后才执行反初始化。
     */
    if (hdma_adc2_waveform.Instance == DMA2_Stream2)
    {
        (void)HAL_DMA_DeInit(&hdma_adc2_waveform);
    }

    /* 配置DMA2_Stream2 */
    hdma_adc2_waveform.Instance = DMA2_Stream2;
    hdma_adc2_waveform.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2_waveform.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2_waveform.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2_waveform.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2_waveform.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2_waveform.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2_waveform.Init.Mode = DMA_CIRCULAR;
    hdma_adc2_waveform.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_adc2_waveform.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2_waveform) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 关联DMA与ADC句柄 */
    __HAL_LINKDMA(&hadc2, DMA_Handle, hdma_adc2_waveform);

    /* 配置DMA中断优先级并使能 */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    return HAL_OK;
}

/*==============================================================================
 * 公共接口函数 - 波形捕获
 *============================================================================*/

/**
 * @brief  启动ADC波形捕获(DMA+定时器触发)
 * @param  sample_rate_hz: 采样频率(Hz)
 * @return HAL_StatusTypeDef: HAL状态
 * @note   工作流程:
 *         1. 停止当前波形捕获
 *         2. 重新配置ADC2为外部触发+DMA连续模式
 *         3. 配置DMA和定时器
 *         4. 启动DMA传输和定时器
 */
HAL_StatusTypeDef BSP_ADC_WaveformStart(uint32_t sample_rate_hz)
{
    ADC_ChannelConfTypeDef channel_config = {0};

    if (sample_rate_hz == 0U)
    {
        return HAL_ERROR;
    }

    /* 先停止之前的捕获(如果有) */
    (void)BSP_ADC_WaveformStop();

    /* 确保hadc2指向ADC2并重新配置 */
    if (hadc2.Instance == ADC2)
    {
        (void)HAL_ADC_DeInit(&hadc2);
    }
    hadc2.Instance = ADC2;

    /* ADC配置: 外部触发(定时器TRGO), DMA连续模式 */
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.NbrOfDiscConversion = 0U;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1U;
    hadc2.Init.DMAContinuousRequests = ENABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 配置采样通道 */
    channel_config.Channel = BSP_ADC_WAVEFORM_CHANNEL;
    channel_config.Rank = 1U;
    /*
     * ADC clock is PCLK2/4 = 21 MHz.  15 sampling cycles plus the 12-bit
     * conversion time fit safely inside a 2 us period (500 kS/s).  The PA2
     * source is an op-amp output, so the shorter acquisition window is
     * appropriate; 84 cycles could not complete before the next trigger.
     */
    channel_config.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    channel_config.Offset = 0U;

    /* 依次配置通道、DMA、定时器 */
    if ((HAL_ADC_ConfigChannel(&hadc2, &channel_config) != HAL_OK) ||
        (BSP_ADC_ConfigureWaveformDma() != HAL_OK) ||
        (BSP_ADC_ConfigureWaveformTimer(sample_rate_hz) != HAL_OK))
    {
        return HAL_ERROR;
    }

    /* 清空缓冲区并就绪标志 */
    (void)memset(s_adc_waveform_buffer, 0, sizeof(s_adc_waveform_buffer));
    s_adc_waveform_ready = 0U;
    s_adc_waveform_overrun_count = 0U;

    /* 启动DMA传输 */
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)s_adc_waveform_buffer,
                          BSP_ADC_WAVEFORM_BUFFER_LENGTH) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 启动定时器, 开始触发ADC */
    s_adc_waveform_running = 1U;
    if (HAL_TIM_Base_Start(&s_adc_timer) != HAL_OK)
    {
        (void)HAL_ADC_Stop_DMA(&hadc2);
        s_adc_waveform_running = 0U;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/**
 * @brief  停止ADC波形捕获
 * @return HAL_StatusTypeDef: HAL状态
 */
HAL_StatusTypeDef BSP_ADC_WaveformStop(void)
{
    if (s_adc_waveform_running != 0U)
    {
        /* 先停定时器, 再停DMA(顺序重要) */
        (void)HAL_TIM_Base_Stop(&s_adc_timer);
        (void)HAL_ADC_Stop_DMA(&hadc2);
        s_adc_waveform_running = 0U;
    }
    s_adc_waveform_ready = 0U;     /* 清除就绪标志 */
    s_adc_waveform_next_half = 0U;
    return HAL_OK;
}

/**
 * @brief  从波形缓冲区取出一块数据(双缓冲机制)
 * @param  block  : 输出数据块
 * @param  length : 数据块长度(必须等于BSP_ADC_WAVEFORM_BLOCK_LENGTH)
 * @param  offset : 输出该数据块在总缓冲区的偏移
 * @return HAL_StatusTypeDef: HAL_OK(成功), HAL_ERROR(参数错误), HAL_BUSY(无就绪数据)
 * @note   双缓冲原理:
 *         DMA在s_adc_waveform_buffer中循环写入, 半传输完成和传输完成回调分别
 *         设置就绪标志bit0和bit1。本函数原子操作取出就绪的半块数据并清除标志位。
 */
HAL_StatusTypeDef BSP_ADC_WaveformTakeBlock(uint16_t *block,
                                             uint16_t length,
                                             uint16_t *offset)
{
    uint8_t ready;
    uint16_t source_offset;

    /* 参数检查 */
    if ((block == NULL) || (offset == NULL) ||
        (length != BSP_ADC_WAVEFORM_BLOCK_LENGTH))
    {
        return HAL_ERROR;
    }

    /* 临界区: 原子判断并清除就绪标志 */
    __disable_irq();
    ready = s_adc_waveform_ready;

    if ((ready & 0x03U) == 0x03U)                       /* 两半均就绪：交替取 */
    {
        if (s_adc_waveform_next_half == 0U)
        {
            s_adc_waveform_ready = (uint8_t)(ready & (uint8_t)~0x01U);
            source_offset = 0U;
            s_adc_waveform_next_half = 1U;
        }
        else
        {
            s_adc_waveform_ready = (uint8_t)(ready & (uint8_t)~0x02U);
            source_offset = BSP_ADC_WAVEFORM_BLOCK_LENGTH;
            s_adc_waveform_next_half = 0U;
        }
    }
    else if ((ready & 0x01U) != 0U)                     /* 前半个缓冲区就绪 */
    {
        s_adc_waveform_ready = (uint8_t)(ready & (uint8_t)~0x01U);
        source_offset = 0U;
        s_adc_waveform_next_half = 1U;
    }
    else if ((ready & 0x02U) != 0U)                     /* 后半个缓冲区就绪 */
    {
        s_adc_waveform_ready = (uint8_t)(ready & (uint8_t)~0x02U);
        source_offset = BSP_ADC_WAVEFORM_BLOCK_LENGTH;
        s_adc_waveform_next_half = 0U;
    }
    else
    {
        __enable_irq();
        return HAL_BUSY;                                 /* 无就绪数据 */
    }

    /* 拷贝数据到用户缓冲区 */
    (void)memcpy(block, &s_adc_waveform_buffer[source_offset],
                 length * sizeof(uint16_t));
    __enable_irq();

    *offset = source_offset;   /* 返回偏移 */
    return HAL_OK;
}

uint32_t BSP_ADC_WaveformGetOverrunCount(void)
{
    return s_adc_waveform_overrun_count;
}

__weak void BSP_ADC_WaveformBlockReadyCallback(void)
{
}

/*==============================================================================
 * ADC中断回调函数
 *============================================================================*/

/**
 * @brief  ADC半传输完成回调(DMA传输过半)
 * @param  hadc: ADC句柄
 * @note   标记前半个缓冲区数据可用
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc == &hadc2) && (s_adc_waveform_running != 0U))
    {
        if ((s_adc_waveform_ready & 0x01U) != 0U)
        {
            ++s_adc_waveform_overrun_count;
        }
        s_adc_waveform_ready |= 0x01U;   /* 置位前半就绪标志 */
        BSP_ADC_WaveformBlockReadyCallback();
    }
}

/**
 * @brief  ADC传输完成回调(DMA一次完整传输结束)
 * @param  hadc: ADC句柄
 * @note   标记后半个缓冲区数据可用
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc == &hadc2) && (s_adc_waveform_running != 0U))
    {
        if ((s_adc_waveform_ready & 0x02U) != 0U)
        {
            ++s_adc_waveform_overrun_count;
        }
        s_adc_waveform_ready |= 0x02U;   /* 置位后半就绪标志 */
        BSP_ADC_WaveformBlockReadyCallback();
    }
}

/*==============================================================================
 * HAL层MSP回调函数(时钟和GPIO初始化/反初始化)
 *============================================================================*/

/**
 * @brief  ADC MSP初始化(使能时钟并配置GPIO为模拟模式)
 * @param  hadc: ADC句柄
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};

    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_ADC1_CLK_ENABLE();       /* 使能ADC1时钟 */
        __HAL_RCC_GPIOA_CLK_ENABLE();      /* 使能GPIOA时钟 */
        gpio.Pin = GPIO_PIN_1;             /* PA1 - ADC1通道1 */
    }
    else if (hadc->Instance == ADC2)
    {
        __HAL_RCC_ADC2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_2;             /* PA2 - ADC2通道2 */
    }
    else
    {
        return;
    }

    /* 配置GPIO为模拟模式 */
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/**
 * @brief  ADC MSP反初始化(释放GPIO并关闭时钟)
 * @param  hadc: ADC句柄
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);
        __HAL_RCC_ADC1_CLK_DISABLE();
    }
    else if (hadc->Instance == ADC2)
    {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
        __HAL_RCC_ADC2_CLK_DISABLE();
    }
}

/*==============================================================================
 * 兼容旧接口(直接映射到BSP函数)
 *============================================================================*/

/**
 * @brief  ADC1初始化(兼容接口)
 */
void ADC1_Init(void)
{
    (void)BSP_ADC1_Init();
}

/**
 * @brief  ADC2初始化(兼容接口)
 */
void ADC2_Init(void)
{
    (void)BSP_ADC2_Init();
}

/**
 * @brief  获取单次ADC转换结果(兼容接口)
 * @param  channel      : 通道号
 * @param  sampling_time: 采样时间(未使用, 保持兼容)
 * @return uint16_t: 12位ADC值
 * @note   ADC_CHANNEL_2对应ADC2, 其他通道使用ADC1
 */
uint16_t ADC_Result_Get(uint8_t channel, uint8_t sampling_time)
{
    /* 根据通道选择ADC外设 */
    ADC_HandleTypeDef *hadc = (channel == ADC_CHANNEL_2) ? &hadc2 : &hadc1;
    uint32_t value = 0U;

    (void)sampling_time;  /* 未使用 */

    if (BSP_ADC_Read(hadc, channel, &value) != HAL_OK)
    {
        return 0U;
    }
    return (uint16_t)value;
}

/**
 * @brief  多次采样取平均值(兼容接口)
 * @param  channel: 通道号
 * @param  times  : 采样次数
 * @return uint16_t: 平均ADC值
 */
uint16_t ADC_Average_Get(uint8_t channel, uint8_t times)
{
    ADC_HandleTypeDef *hadc = (channel == ADC_CHANNEL_2) ? &hadc2 : &hadc1;
    uint32_t value = 0U;

    if (BSP_ADC_ReadAverage(hadc, channel, times, &value) != HAL_OK)
    {
        return 0U;
    }
    return (uint16_t)value;
}

/**
 * @brief  对波形数据进行线性插值并转换为电压值
 * @param  source_buffer : 原始ADC数据
 * @param  source_length : 原始数据长度
 * @param  result_buffer : 输出缓冲区(电压值, 单位V)
 * @param  result_length : 输出数据长度
 * @return uint16_t: 实际处理的源数据点数(起始点到结束点的跨度), 0表示失败
 * @note   算法:
 *         1. 找出波形的最大值和最小值, 计算峰峰值
 *         2. 检测信号上升沿, 定位"测量区间"的起始点(电压下降到半峰以下)
 *         3. 检测第二个上升沿, 定位终点(电压再次越过半峰)
 *         4. 在[start, end]区间内等距取result_length个点, 线性插值, 转换为电压
 */
uint16_t ADC_Waveform_Get(const uint16_t *source_buffer, uint16_t source_length,
                          double *result_buffer, uint16_t result_length)
{
    uint16_t index;
    uint16_t max_value = 0U;
    uint16_t min_value = UINT16_MAX;
    uint16_t start = 0U;
    uint16_t end = 0U;
    uint16_t peak_to_peak;
    double source_position;
    double fraction;

    /* 参数有效性检查 */
    if ((source_buffer == NULL) || (result_buffer == NULL) ||
        (source_length < 2U) || (result_length == 0U))
    {
        return 0U;
    }

    /* 遍历原始数据, 找出最大值、最小值, 并检查数据有效性 */
    for (index = 0U; index < source_length; ++index)
    {
        if (source_buffer[index] > (uint16_t)BSP_ADC_FULL_SCALE)
        {
            return 0U;  /* 数据超过满量程, 无效 */
        }
        if (source_buffer[index] > max_value)
        {
            max_value = source_buffer[index];
        }
        if (source_buffer[index] < min_value)
        {
            min_value = source_buffer[index];
        }
    }

    peak_to_peak = max_value - min_value;
    if (peak_to_peak == 0U)  /* 峰峰值为0, 无波形变化 */
    {
        return 0U;
    }

    /* 从开始处略过10%长度, 查找第一个上升沿(幅度达到90%峰峰值) */
    for (index = (uint16_t)(source_length / 100U); index < source_length; ++index)
    {
        if ((uint32_t)source_buffer[index] - min_value >= (uint32_t)peak_to_peak * 9U / 10U)
        {
            break;  /* 找到高电平起始 */
        }
    }
    if (index == source_length)
    {
        return 0U;  /* 未找到有效高电平 */
    }

    /* 搜索高电平结束点(下降到50%以下) -> 作为测量起点 */
    while ((index < source_length) &&
           ((uint32_t)source_buffer[index] - min_value >= (uint32_t)peak_to_peak / 2U))
    {
        ++index;
    }
    start = index;  /* 记录起点 */

    /* 搜索下一个上升沿到90% */
    while ((index < source_length) &&
           ((uint32_t)source_buffer[index] - min_value < (uint32_t)peak_to_peak * 9U / 10U))
    {
        ++index;
    }
    /* 搜索该高电平结束点(再次下降到50%以下) -> 作为测量终点 */
    while ((index < source_length) &&
           ((uint32_t)source_buffer[index] - min_value >= (uint32_t)peak_to_peak / 2U))
    {
        ++index;
    }
    end = index;  /* 记录终点 */

    /* 起点和终点有效性检查 */
    if ((end <= start) || (end >= source_length))
    {
        return 0U;
    }

    /* 在[start, end]区间内, 线性插值生成result_length个电压数据点 */
    for (index = 0U; index < result_length; ++index)
    {
        /* 计算浮点源位置 */
        source_position = start + ((double)index * (end - start) / result_length);
        /* 小数部分 */
        fraction = source_position - (uint16_t)source_position;
        /* 线性插值并转换为电压值(ADC值 * Vref / 4095) */
        result_buffer[index] = (source_buffer[(uint16_t)source_position] +
                                (source_buffer[(uint16_t)source_position + 1U] -
                                 source_buffer[(uint16_t)source_position]) * fraction) *
                               BSP_ADC_VREF_VOLTS / BSP_ADC_FULL_SCALE;
    }

    /* 返回实际处理的数据跨度 */
    return end - start;
}
