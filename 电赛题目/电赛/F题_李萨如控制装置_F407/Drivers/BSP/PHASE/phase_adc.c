/**
 * @file    phase_adc.c
 * @brief   双通道相位检测ADC驱动（STM32F4）
 * @details 使用ADC1/ADC2同步模式，定时器8触发，DMA双缓冲半满/全满中断，
 *          实现1.2MSPS连续采样，数据经DMA循环传输至内存缓冲区。
 *          提供取块接口供上层进行I/Q解调和相位计算。
 *          支持过载（Overrun）计数和采样率设置。
 * @note    硬件要求：PA1->ADC1_IN1, PA2->ADC2_IN2，TIM8_TRGO触发。
 */

#include "phase_adc.h"

#include <stddef.h>
#include <string.h>

static volatile uint32_t s_phase_adc_rejected_block_count;
static volatile uint32_t s_phase_adc_generation[2];
static volatile uint32_t s_phase_adc_ready_sequence[2];
static volatile uint32_t s_phase_adc_completion_sequence;

/*---------------- 硬件句柄声明 ----------------*/
ADC_HandleTypeDef hadc_phase_master;    /* ADC1 主设备（触发源） */
ADC_HandleTypeDef hadc_phase_slave;     /* ADC2 从设备（同步跟随） */
DMA_HandleTypeDef hdma_phase_adc;       /* DMA2_Stream0 用于传输 */

/*---------------- 静态内部状态 ----------------*/
static TIM_HandleTypeDef s_phase_adc_timer;              /* TIM8 触发定时器 */
static uint32_t s_phase_adc_buffer[PHASE_ADC_SAMPLE_COUNT]; /* DMA缓冲区（双块） */
static volatile uint8_t s_phase_adc_ready_mask;          /* 半满(0x01)/全满(0x02)标志 */
static volatile uint32_t s_phase_adc_overrun_count;      /* 数据覆盖计数（溢出） */
static uint32_t s_phase_adc_sample_rate_hz;              /* 当前实际采样率 */
static uint8_t s_phase_adc_initialized;                  /* 初始化标志 */
static PhaseAdcReadyCallback_t s_phase_adc_ready_callback; /* 回调函数指针 */

/** Return the real TIM8 kernel clock derived from the live RCC divisors. */
static uint32_t PhaseAdc_GetTimerClockHz(void)
{
    uint32_t timer_clock_hz = HAL_RCC_GetPCLK2Freq();

    /* STM32F4 timers run at 2 x PCLK when the APB prescaler is not DIV1. */
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U)
    {
        timer_clock_hz *= 2U;
    }
    return timer_clock_hz;
}

/* Select the longest acquisition interval that leaves 20% conversion margin. */
static uint32_t PhaseAdc_SelectSamplingTime(uint32_t sample_rate_hz)
{
    static const uint16_t cycles[] = {480U, 144U, 112U, 84U,
                                      56U, 28U, 15U, 3U};
    static const uint32_t settings[] = {
        ADC_SAMPLETIME_480CYCLES, ADC_SAMPLETIME_144CYCLES,
        ADC_SAMPLETIME_112CYCLES, ADC_SAMPLETIME_84CYCLES,
        ADC_SAMPLETIME_56CYCLES, ADC_SAMPLETIME_28CYCLES,
        ADC_SAMPLETIME_15CYCLES, ADC_SAMPLETIME_3CYCLES
    };
    uint32_t index;

    for (index = 0U; index < (sizeof(cycles) / sizeof(cycles[0])); ++index)
    {
        if ((uint64_t)(cycles[index] + 12U) * sample_rate_hz * 5ULL <=
            21000000ULL * 4ULL)
        {
            return settings[index];
        }
    }
    return ADC_SAMPLETIME_3CYCLES;
}

static HAL_StatusTypeDef PhaseAdc_ConfigureSamplingTime(
    uint32_t sample_rate_hz)
{
    ADC_ChannelConfTypeDef channel = {0};

    channel.Rank = 1U;
    channel.Offset = 0U;
    channel.SamplingTime = PhaseAdc_SelectSamplingTime(sample_rate_hz);
    channel.Channel = PHASE_ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc_phase_master, &channel) != HAL_OK)
    {
        return HAL_ERROR;
    }
    channel.Channel = PHASE_ADC_CHANNEL_2;
    return HAL_ADC_ConfigChannel(&hadc_phase_slave, &channel);
}

/*---------------- 内部辅助函数 ----------------*/

/**
 * @brief 初始化ADC句柄（主/从共用）
 * @param hadc       ADC句柄指针
 * @param instance   ADC寄存器基址（ADC1/ADC2）
 * @param master     1=主设备(配置外部触发), 0=从设备(无触发)
 * @return HAL_OK / HAL_ERROR
 */
static HAL_StatusTypeDef PhaseAdc_InitHandle(ADC_HandleTypeDef *hadc,
                                             ADC_TypeDef *instance,
                                             uint8_t master)
{
    hadc->Instance = instance;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 84MHz/4=21MHz, 符合ADC时钟上限 */
    hadc->Init.Resolution = ADC_RESOLUTION_12B;             /* 12位精度 */
    hadc->Init.ScanConvMode = DISABLE;                      /* 单通道模式 */
    hadc->Init.ContinuousConvMode = DISABLE;                /* 由定时器触发，非连续 */
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.NbrOfDiscConversion = 0U;
    /* 主设备使用TIM8_TRGO上升沿触发，从设备无外部触发（由硬件同步） */
    hadc->Init.ExternalTrigConvEdge =
        (master != 0U) ? ADC_EXTERNALTRIGCONVEDGE_RISING :
                         ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;             /* 右对齐，方便提取 */
    hadc->Init.NbrOfConversion = 1U;                        /* 每个ADC只转换一个通道 */
    hadc->Init.DMAContinuousRequests = ENABLE;              /* DMA循环模式需要此位使能 */
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;          /* 单次转换结束即触发DMA */
    return HAL_ADC_Init(hadc);
}

/**
 * @brief ADC的底层GPIO和时钟初始化（HAL回调）
 * @param hadc   ADC句柄
 * @note  由HAL_ADC_Init()自动调用，使能ADC1/2时钟及PA1/PA2模拟输入
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};

    if (hadc == NULL)
    {
        return;
    }

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio.Pin = PHASE_ADC_GPIO_PIN_1 | PHASE_ADC_GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PHASE_ADC_GPIO_PORT, &gpio);
}

/**
 * @brief 初始化DMA（循环模式，双字传输）
 * @return HAL_OK / HAL_ERROR
 * @note  DMA2_Stream0, 通道0, 外设地址固定(ADC1_DR), 内存地址递增,
 *        传输字宽32位（包含两个ADC的12位数据），循环模式。
 */
static HAL_StatusTypeDef PhaseAdc_InitDma(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    (void)memset(&hdma_phase_adc, 0, sizeof(hdma_phase_adc));
    hdma_phase_adc.Instance = DMA2_Stream0;
    hdma_phase_adc.Init.Channel = DMA_CHANNEL_0;             /* ADC1映射到DMA2 Stream0 Channel0 */
    hdma_phase_adc.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_phase_adc.Init.PeriphInc = DMA_PINC_DISABLE;        /* 外设地址固定（ADC_DR） */
    hdma_phase_adc.Init.MemInc = DMA_MINC_ENABLE;            /* 内存地址递增 */
    hdma_phase_adc.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* 外设字宽32位 */
    hdma_phase_adc.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_phase_adc.Init.Mode = DMA_CIRCULAR;                 /* 循环模式，自动重填 */
    hdma_phase_adc.Init.Priority = DMA_PRIORITY_VERY_HIGH;   /* 最高优先级，降低溢出风险 */
    hdma_phase_adc.Init.FIFOMode = DMA_FIFOMODE_DISABLE;     /* 直接模式 */

    if (HAL_DMA_Init(&hdma_phase_adc) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_LINKDMA(&hadc_phase_master, DMA_Handle, hdma_phase_adc);
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5U, 0U);         /* 中断优先级5 */
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    return HAL_OK;
}

/**
 * @brief 初始化定时器TIM8作为ADC触发源
 * @param sample_rate_hz  期望采样率（Hz）
 * @return HAL_OK / HAL_ERROR
 * @note  TIM8是16位定时器，自动计算分频和周期，使实际采样率尽量接近目标。
 *        使用向上计数，更新事件（UEV）作为TRGO输出。
 */
static HAL_StatusTypeDef PhaseAdc_InitTimer(uint32_t sample_rate_hz)
{
    TIM_MasterConfigTypeDef master = {0};
    uint32_t prescaler_divider;
    uint32_t period;
    uint32_t timer_clock_hz;
    uint64_t minimum_divider;

    if ((sample_rate_hz == 0U) ||
        (sample_rate_hz > PHASE_ADC_MAX_SAMPLE_RATE_HZ))
    {
        return HAL_ERROR;
    }

    /* PSC >= ceil(timer_clock / (sample_rate * 65536)) keeps ARR in range. */
    timer_clock_hz = PhaseAdc_GetTimerClockHz();
    if (timer_clock_hz == 0U)
    {
        return HAL_ERROR;
    }

    minimum_divider = (uint64_t)sample_rate_hz * 65536ULL;
    prescaler_divider =
        (uint32_t)(((uint64_t)timer_clock_hz +
                    minimum_divider - 1ULL) / minimum_divider);
    if (prescaler_divider == 0U)
    {
        prescaler_divider = 1U;
    }
    if (prescaler_divider > 65536U)          /* 分频寄存器最大65536（对应0写入） */
    {
        return HAL_ERROR;
    }

    /* ARR + 1 = round(timer_clock / (prescaler * sample_rate)). */
    period = (uint32_t)(
        ((uint64_t)timer_clock_hz +
         ((uint64_t)prescaler_divider * sample_rate_hz) / 2ULL) /
        ((uint64_t)prescaler_divider * sample_rate_hz));
    if ((period == 0U) || (period > 65536U))
    {
        return HAL_ERROR;
    }

    __HAL_RCC_TIM8_CLK_ENABLE();
    (void)memset(&s_phase_adc_timer, 0, sizeof(s_phase_adc_timer));
    s_phase_adc_timer.Instance = PHASE_ADC_TIMER;
    s_phase_adc_timer.Init.Prescaler = prescaler_divider - 1U;
    s_phase_adc_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_phase_adc_timer.Init.Period = period - 1U;
    s_phase_adc_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_phase_adc_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&s_phase_adc_timer) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 配置TRGO为更新事件 */
    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(
            &s_phase_adc_timer, &master) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 记录实际采样率（可能因整数分频与目标略有偏差） */
    s_phase_adc_sample_rate_hz =
        timer_clock_hz / prescaler_divider / period;
    return HAL_OK;
}

/*---------------- 公共API ----------------*/

/**
 * @brief 初始化ADC模块（仅调用一次）
 * @return HAL_OK / HAL_ERROR
 * @note  配置ADC1/ADC2为同步双模式，DMA循环。采样保持3周期、转换12周期，
 *        总转换时间15/21MHz约0.714us；软件将触发率限制在1.2MSPS以内。
 *        初始化后处于待触发状态，需调用 PhaseAdc_Start() 启动采样。
 */
HAL_StatusTypeDef PhaseAdc_Init(void)
{
    ADC_ChannelConfTypeDef channel = {0};
    ADC_MultiModeTypeDef multimode = {0};

    if (s_phase_adc_initialized != 0U)
    {
        return HAL_OK;
    }

    /* 初始化主从ADC句柄 */
    if (PhaseAdc_InitHandle(&hadc_phase_master, ADC1, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (PhaseAdc_InitHandle(&hadc_phase_slave, ADC2, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 配置通道：ADC1_IN1, ADC2_IN2，采样时间3周期（快速） */
    channel.Rank = 1U;
    channel.SamplingTime = ADC_SAMPLETIME_3CYCLES;   /* 3周期 + 12周期 = 15周期 @21MHz = 0.714us */
    channel.Offset = 0U;
    channel.Channel = PHASE_ADC_CHANNEL_1;
    if (HAL_ADC_ConfigChannel(&hadc_phase_master, &channel) != HAL_OK)
    {
        return HAL_ERROR;
    }
    channel.Channel = PHASE_ADC_CHANNEL_2;
    if (HAL_ADC_ConfigChannel(&hadc_phase_slave, &channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 双ADC同步模式：同时采样，DMA读取模式2（32位双通道数据） */
    multimode.Mode = ADC_DUALMODE_REGSIMULT;         /* 同步注入/规则同时转换 */
    multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;   /* 32位打包：低16位ADC1，高16位ADC2 */
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES; /* 采样延迟，保证同步 */
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc_phase_master, &multimode) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (PhaseAdc_InitDma() != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_phase_adc_ready_mask = 0U;
    s_phase_adc_overrun_count = 0U;
    s_phase_adc_rejected_block_count = 0U;
    s_phase_adc_generation[0] = 0U;
    s_phase_adc_generation[1] = 0U;
    s_phase_adc_ready_sequence[0] = 0U;
    s_phase_adc_ready_sequence[1] = 0U;
    s_phase_adc_completion_sequence = 0U;
    s_phase_adc_sample_rate_hz = 0U;
    s_phase_adc_initialized = 1U;
    return HAL_OK;
}

/**
 * @brief 启动连续采样
 * @param sample_rate_hz  期望采样率（不超过PHASE_ADC_MAX_SAMPLE_RATE_HZ）
 * @return HAL_OK / HAL_ERROR
 * @note  先启动从设备（ADC2），再启动主设备（ADC1）的DMA多模式，
 *        最后启动定时器产生触发脉冲。
 *        采样开始后，DMA会在半满和全满时产生中断，通过回调通知上层。
 */
HAL_StatusTypeDef PhaseAdc_Start(uint32_t sample_rate_hz)
{
    if (s_phase_adc_initialized == 0U)
    {
        return HAL_ERROR;
    }
    s_phase_adc_overrun_count = 0U;
    s_phase_adc_rejected_block_count = 0U;
    if (PhaseAdc_InitTimer(sample_rate_hz) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (PhaseAdc_ConfigureSamplingTime(s_phase_adc_sample_rate_hz) != HAL_OK)
    {
        return HAL_ERROR;
    }

    PhaseAdc_Flush();   /* 清空缓冲区与标志 */
    /* 先启动从设备（必须早于主设备） */
    if (HAL_ADC_Start(&hadc_phase_slave) != HAL_OK)
    {
        return HAL_ERROR;
    }
    /* 启动主设备DMA传输（自动启动ADC1） */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc_phase_master,
                                     s_phase_adc_buffer,
                                     PHASE_ADC_SAMPLE_COUNT) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc_phase_slave);
        return HAL_ERROR;
    }
    /* 最后启动定时器，触发转换开始 */
    return HAL_TIM_Base_Start(&s_phase_adc_timer);
}

/**
 * @brief 停止采样，释放DMA和定时器
 * @return HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef PhaseAdc_Stop(void)
{
    if (s_phase_adc_initialized == 0U)
    {
        return HAL_ERROR;
    }

    (void)HAL_TIM_Base_Stop(&s_phase_adc_timer);
    (void)HAL_ADCEx_MultiModeStop_DMA(&hadc_phase_master);
    (void)HAL_ADC_Stop(&hadc_phase_slave);
    s_phase_adc_ready_mask = 0U;
    s_phase_adc_sample_rate_hz = 0U;
    return HAL_OK;
}

/**
 * @brief 清空内部状态和缓冲区（关闭中断保护）
 * @note  用于重新开始前或错误恢复。
 */
void PhaseAdc_Flush(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    s_phase_adc_ready_mask = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    (void)memset(s_phase_adc_buffer, 0, sizeof(s_phase_adc_buffer));
}

/**
 * @brief 内部函数：标记半块或全块就绪，并调用回调
 * @param mask   0x01（半满）或 0x02（全满）
 * @note  若已有对应标志未处理，则视为溢出（Overrun），计数器加1。
 */
static void PhaseAdc_MarkReady(uint8_t mask)
{
    uint32_t half = (mask == 0x01U) ? 0U : 1U;

    if ((s_phase_adc_ready_mask & mask) != 0U)
    {
        ++s_phase_adc_overrun_count;   /* 上次数据尚未取走，再次产生中断 => 覆盖 */
    }
    ++s_phase_adc_generation[half];
    ++s_phase_adc_completion_sequence;
    s_phase_adc_ready_sequence[half] = s_phase_adc_completion_sequence;
    s_phase_adc_ready_mask |= mask;
    if (s_phase_adc_ready_callback != NULL)
    {
        s_phase_adc_ready_callback();  /* 通知上层有新数据块可用 */
    }
}

/**
 * @brief 注册数据就绪回调函数
 * @param callback  函数指针（可为NULL）
 * @note  回调在中断上下文中执行，应尽量简短。
 */
void PhaseAdc_SetReadyCallback(PhaseAdcReadyCallback_t callback)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_phase_adc_ready_callback = callback;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/*---------------- DMA中断回调（由HAL调用） ----------------*/

/**
 * @brief DMA半传输完成中断（已处理半个缓冲区）
 * @param hadc   ADC句柄（此处固定为ADC1）
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1))
    {
        PhaseAdc_MarkReady(0x01U);  /* 标记前半块就绪 */
    }
}

/**
 * @brief DMA全传输完成中断（已填满整个缓冲区）
 * @param hadc   ADC句柄
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1))
    {
        PhaseAdc_MarkReady(0x02U);  /* 标记后半块就绪 */
    }
}

/*---------------- 数据取块接口 ----------------*/

/**
 * @brief 获取一个有效数据块（半块或全块）
 * @param channel1  输出数组（I分量，长度PHASE_ADC_BLOCK_COUNT）
 * @param channel2  输出数组（Q分量）
 * @param length    必须等于 PHASE_ADC_BLOCK_COUNT
 * @return HAL_OK       成功取到新块
 *         HAL_BUSY     无新数据（正在等待）
 *         HAL_ERROR    参数错误
 * @note  若半满/全满标志均未置位，返回HAL_BUSY。
 *        取块后自动清除对应标志，并释放缓冲区供DMA继续填充。
 *        数据已减去2048（12位ADC零电平偏移），得到有符号值。
 */
HAL_StatusTypeDef PhaseAdc_TakeBlock(int16_t *channel1,
                                     int16_t *channel2,
                                     uint16_t length)
{
    uint8_t ready_mask;
    uint8_t half;
    uint32_t generation;
    uint32_t completion_sequence;
    uint16_t offset;
    uint16_t index;

    if ((channel1 == NULL) || (channel2 == NULL) ||
        (length != PHASE_ADC_BLOCK_COUNT))
    {
        return HAL_ERROR;
    }

    /* 临界区读取并清除标志，选择偏移（0或半块长度） */
    {
        uint32_t primask = __get_PRIMASK();

        __disable_irq();
        ready_mask = s_phase_adc_ready_mask;
        if ((ready_mask == 0x03U) &&
            (s_phase_adc_ready_sequence[1] >
             s_phase_adc_ready_sequence[0]))
        {
            ready_mask &= (uint8_t)~0x01U;
        }
        if (s_phase_adc_ready_mask == 0x03U)
        {
            /* The older half is already being filled again; discard its flag. */
            s_phase_adc_ready_mask = 0U;
        }
        if ((ready_mask & 0x01U) != 0U)
        {
            half = 0U;
            generation = s_phase_adc_generation[0];
            completion_sequence = s_phase_adc_completion_sequence;
            s_phase_adc_ready_mask &= (uint8_t)~0x01U;
            offset = 0U;                        /* 前半块 */
        }
        else if ((ready_mask & 0x02U) != 0U)
        {
            half = 1U;
            generation = s_phase_adc_generation[1];
            completion_sequence = s_phase_adc_completion_sequence;
            s_phase_adc_ready_mask &= (uint8_t)~0x02U;
            offset = PHASE_ADC_BLOCK_COUNT;     /* 后半块 */
        }
        else
        {
            if (primask == 0U)
            {
                __enable_irq();
            }
            return HAL_BUSY;    /* 无新数据 */
        }
        if (primask == 0U)
        {
            __enable_irq();
        }
    }

    /* 提取并转换：低16位为ADC1，高16位为ADC2，每通道12位有效 */
    for (index = 0U; index < length; ++index)
    {
        uint32_t packed = s_phase_adc_buffer[offset + index];
        channel1[index] = (int16_t)(packed & 0x0FFFU) - 2048;   /* 带符号偏移 */
        channel2[index] = (int16_t)((packed >> 16U) & 0x0FFFU) - 2048;
    }
    if ((s_phase_adc_generation[half] != generation) ||
        (s_phase_adc_completion_sequence != completion_sequence))
    {
        ++s_phase_adc_rejected_block_count;
        return HAL_BUSY;
    }
    return HAL_OK;
}

/**
 * @brief 获取过载（Overrun）计数
 * @return 自启动以来的溢出次数
 * @note  溢出意味着采样数据被覆盖，可能导致数据丢失。
 *        若计数持续增长，需降低采样率或优化中断处理。
 */
uint32_t PhaseAdc_GetOverrunCount(void)
{
    return s_phase_adc_overrun_count;
}

uint32_t PhaseAdc_GetRejectedBlockCount(void)
{
    return s_phase_adc_rejected_block_count;
}

/**
 * @brief 获取当前实际采样率（可能与请求略有偏差）
 * @return 实际采样率（Hz）
 */
uint32_t PhaseAdc_GetSampleRateHz(void)
{
    return s_phase_adc_sample_rate_hz;
}
