/**
 * @file    phase_capture.c
 * @brief   相位/频率捕获模块
 *
 * 使用 TIM2 的 CH3 (PB10) 作为主信号捕获通道，CH4 (PB11) 作为反馈信号
 * 捕获通道。两路信号共享同一个定时器时基，因此计算频差时没有跨定时器
 * 偏差。模块支持两种采样方式：
 * - DMA 循环模式：对高频信号（快速边沿）连续采样，通过 DMA 缓冲区累
 *   加多个边沿计算平均周期。
 * - 中断模式：低频信号（慢速边沿）采用逐次捕获中断方式，在测得周期
 *   后自动尝试切回 DMA 模式。
 *
 * 硬件依赖：
 * - 定时器：TIM2
 * - DMA 流：主捕获通道使用 DMA1_StreamX，反馈通道使用另一个独立 DMA 流
 * - 输入引脚：PB10 (TIM2_CH3)，PB11 (TIM2_CH4)
 */

#include "phase_capture.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/*
 * Phase is estimated from several independently captured edge pairs.
 * Frequency measurement still uses the long-period DMA accumulation below;
 * these limits only bound the short history scan used by the DPLL.
 */
#define PHASE_CAPTURE_PAIR_INPUT_COUNT            8U
#define PHASE_CAPTURE_PAIR_FEEDBACK_SEARCH_COUNT 32U
#define PHASE_CAPTURE_PAIR_MIN_COUNT              4U
#define PHASE_CAPTURE_PAIR_MIN_RESULTANT       0.98f
#define PHASE_CAPTURE_PI_F             3.14159265358979323846f

/* -------------------------------------------------------------------------- */
/*                        全局实例与缓冲区定义                                 */
/* -------------------------------------------------------------------------- */

/** 全局相位捕获实例，供模块级 API 使用 */
PhaseCapture_t g_phase_capture;

/** 当前活跃的捕获实例指针，用于在回调中定位数据 */
static PhaseCapture_t *s_phase_capture_active;

/** 主信号通道 DMA 循环缓冲区，存储 TIM2->CCR3 的时间戳 */
static uint32_t s_phase_capture_dma_buffer[PHASE_CAPTURE_DMA_BUFFER_COUNT];

/** 反馈信号通道 DMA 循环缓冲区，存储 TIM2->CCR4 的时间戳 */
static uint32_t s_phase_capture_feedback_dma_buffer[
    PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT];

typedef struct
{
    uint32_t latest_index;
    uint32_t available_samples;
} PhaseCaptureDmaSnapshot_t;

/* -------------------------------------------------------------------------- */
/*                        内部静态辅助函数                                     */
/* -------------------------------------------------------------------------- */

static float PhaseCapture_NormalizeDegrees(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }
    while (degrees < -180.0f)
    {
        degrees += 360.0f;
    }
    return degrees;
}

/**
 * @brief Take a coherent snapshot of a circular DMA write position.
 *
 * The transfer-complete counter and NDTR are sampled together so a wrap
 * cannot silently turn the newest element into an unrelated buffer slot.
 */
static uint8_t PhaseCapture_GetDmaSnapshot(
    DMA_HandleTypeDef *dma,
    volatile uint32_t *wrap_count,
    uint32_t buffer_count,
    PhaseCaptureDmaSnapshot_t *snapshot)
{
    uint32_t wrap_before;
    uint32_t wrap_after;
    uint32_t remaining;
    uint32_t completed;
    uint32_t total_samples;
    uint32_t attempts;

    if ((dma == NULL) || (wrap_count == NULL) ||
        (snapshot == NULL) || (buffer_count == 0U))
    {
        return 0U;
    }

    for (attempts = 0U; attempts < 3U; ++attempts)
    {
        wrap_before = *wrap_count;
        remaining = __HAL_DMA_GET_COUNTER(dma);
        wrap_after = *wrap_count;
        if (wrap_before == wrap_after)
        {
            break;
        }
    }
    if ((wrap_before != wrap_after) || (remaining > buffer_count))
    {
        return 0U;
    }

    completed = buffer_count - remaining;
    total_samples = wrap_after * buffer_count + completed;
    if (total_samples == 0U)
    {
        return 0U;
    }

    snapshot->latest_index = (completed == 0U) ?
        (buffer_count - 1U) : (completed - 1U);
    snapshot->available_samples =
        (total_samples < buffer_count) ? total_samples : buffer_count;
    return 1U;
}

static uint32_t PhaseCapture_HistoryTimestamp(
    const uint32_t *buffer,
    uint32_t buffer_count,
    uint32_t latest_index,
    uint32_t history_offset)
{
    uint32_t index;

    index = (latest_index + buffer_count -
             (history_offset % buffer_count)) % buffer_count;
    return ((const volatile uint32_t *)buffer)[index];
}

/**
 * @brief  将需要的捕获边沿数量转换为输入捕获预分频器参数
 * @param  capture_cycles 要求的边沿数（必须是 1, 2, 4, 8）
 * @return 对应的 TIM_ICPSC 值；若不是合法值则返回 0xFFFFFFFFUL
 */
static uint32_t PhaseCapture_EdgeCountToPrescaler(uint32_t capture_cycles)
{
    switch (capture_cycles)
    {
        case 1U:  return TIM_ICPSC_DIV1;
        case 2U:  return TIM_ICPSC_DIV2;
        case 4U:  return TIM_ICPSC_DIV4;
        case 8U:  return TIM_ICPSC_DIV8;
        default:  return 0xFFFFFFFFUL;
    }
}

/**
 * @brief  将指定的边沿捕获配置应用到定时器硬件，并重置捕获状态
 * @param  capture        捕获实例
 * @param  capture_cycles 新的边沿数
 * @param  discard_ready  是否丢弃可能已就绪的捕获结果
 *                        非 0 时清空周期就绪标志
 */
static void PhaseCapture_ApplyEdgeCount(PhaseCapture_t *capture,
                                        uint32_t capture_cycles,
                                        uint8_t discard_ready)
{
    uint32_t ic_prescaler = PhaseCapture_EdgeCountToPrescaler(capture_cycles);

    /* 为两个捕获通道设置相同的边沿预分频器 */
    __HAL_TIM_SET_ICPRESCALER(&capture->timer,
                              PHASE_CAPTURE_TIMER_CHANNEL,
                              ic_prescaler);
    __HAL_TIM_SET_ICPRESCALER(&capture->timer,
                              PHASE_CAPTURE_FEEDBACK_CHANNEL,
                              ic_prescaler);
    capture->active_capture_cycles = capture_cycles;
    capture->last_capture = 0U;
    capture->has_last_capture = 0U;
    if (discard_ready != 0U)
    {
        capture->period_ticks = 0U;
        capture->period_capture_cycles = capture_cycles;
        capture->capture_ready = 0U;
    }
    /* 清除相关中断标志，防止残留中断引发误触发 */
    __HAL_TIM_CLEAR_IT(&capture->timer, PHASE_CAPTURE_TIMER_IT);
    __HAL_TIM_CLEAR_FLAG(&capture->timer, PHASE_CAPTURE_OVERCAPTURE_FLAG);
    __HAL_TIM_CLEAR_FLAG(&capture->timer, TIM_FLAG_CC4OF);
}

/**
 * @brief  初始化主捕获通道和反馈通道的 DMA 流，并链接到 TIM2 的 DMA 请求
 *         同时配置 NVIC 中断优先级并使能对应的 DMA 中断
 * @param  capture 捕获实例
 * @return HAL_OK 成功；HAL_ERROR 失败
 */
static HAL_StatusTypeDef PhaseCapture_InitDma(PhaseCapture_t *capture)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* ---------- 主捕获通道 DMA 初始化 ---------- */
    (void)memset(&capture->dma, 0, sizeof(capture->dma));
    capture->dma.Instance                 = PHASE_CAPTURE_DMA_STREAM;
    capture->dma.Init.Channel             = PHASE_CAPTURE_DMA_CHANNEL;
    capture->dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    capture->dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    capture->dma.Init.MemInc              = DMA_MINC_ENABLE;
    capture->dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    capture->dma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    capture->dma.Init.Mode                = DMA_CIRCULAR;
    capture->dma.Init.Priority            = DMA_PRIORITY_HIGH;
    capture->dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    capture->dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    capture->dma.Init.MemBurst            = DMA_MBURST_SINGLE;
    capture->dma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&capture->dma) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_LINKDMA(&capture->timer, hdma[TIM_DMA_ID_CC3], capture->dma);

    /*
     * PB11/TIM2_CH4 使用独立 DMA 流。两个 CCR 寄存器都由同一个
     * TIM2 计数器锁存，因此后续计算频差时没有跨定时器时基误差。
     */
    (void)memset(&capture->feedback_dma, 0, sizeof(capture->feedback_dma));
    capture->feedback_dma.Instance                 = PHASE_CAPTURE_FEEDBACK_DMA_STREAM;
    capture->feedback_dma.Init.Channel             = PHASE_CAPTURE_FEEDBACK_DMA_CHANNEL;
    capture->feedback_dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    capture->feedback_dma.Init.PeriphInc           = DMA_PINC_DISABLE;
    capture->feedback_dma.Init.MemInc              = DMA_MINC_ENABLE;
    capture->feedback_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    capture->feedback_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    capture->feedback_dma.Init.Mode                = DMA_CIRCULAR;
    capture->feedback_dma.Init.Priority            = DMA_PRIORITY_HIGH;
    capture->feedback_dma.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    capture->feedback_dma.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
    capture->feedback_dma.Init.MemBurst            = DMA_MBURST_SINGLE;
    capture->feedback_dma.Init.PeriphBurst         = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&capture->feedback_dma) != HAL_OK)
    {
        return HAL_ERROR;
    }
    __HAL_LINKDMA(&capture->timer, hdma[TIM_DMA_ID_CC4], capture->feedback_dma);

    /* NVIC 配置 */
    HAL_NVIC_SetPriority(PHASE_CAPTURE_DMA_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(PHASE_CAPTURE_DMA_IRQn);
    HAL_NVIC_SetPriority(PHASE_CAPTURE_FEEDBACK_DMA_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(PHASE_CAPTURE_FEEDBACK_DMA_IRQn);
    return HAL_OK;
}

/**
 * @brief  复位所有与 DMA 进度相关的统计变量
 *         包括包裹计数、上次样本数、上次时间戳和有效标志
 */
static void PhaseCapture_ResetDmaProgress(PhaseCapture_t *capture)
{
    capture->dma_wrap_count               = 0U;
    capture->dma_last_sample_count        = 0U;
    capture->dma_last_timestamp           = 0U;
    capture->dma_has_last_sample          = 0U;

    capture->feedback_dma_wrap_count      = 0U;
    capture->feedback_dma_last_sample_count = 0U;
    capture->feedback_dma_last_timestamp  = 0U;
    capture->feedback_dma_has_last_sample = 0U;

    capture->phase_last_input_timestamp    = 0U;
    capture->phase_last_feedback_timestamp = 0U;
}

/**
 * @brief  停止当前所有正在运行的捕获模式（DMA 及 IRQ）
 *         确保硬件资源被安全释放
 * @param  capture 捕获实例
 * @return HAL_OK 或 HAL_ERROR
 */
static HAL_StatusTypeDef PhaseCapture_StopCurrent(PhaseCapture_t *capture)
{
    HAL_StatusTypeDef status = HAL_OK;

    capture->dma_restart_requested = 0U;
    /* 先停反馈通道 DMA，再停主通道 DMA，最后停中断 */
    if (capture->feedback_dma_running != 0U)
    {
        __HAL_TIM_DISABLE_DMA(&capture->timer, TIM_DMA_CC4);
        if (capture->feedback_dma.State == HAL_DMA_STATE_BUSY)
        {
            if (HAL_DMA_Abort(&capture->feedback_dma) != HAL_OK)
            {
                status = HAL_ERROR;
            }
        }
        (void)HAL_TIM_IC_Stop(&capture->timer, PHASE_CAPTURE_FEEDBACK_CHANNEL);
        capture->feedback_dma_running = 0U;
    }
    if (capture->dma_running != 0U)
    {
        __HAL_TIM_DISABLE_DMA(&capture->timer, TIM_DMA_CC3);
        if (capture->dma.State == HAL_DMA_STATE_BUSY)
        {
            if (HAL_DMA_Abort(&capture->dma) != HAL_OK)
            {
                status = HAL_ERROR;
            }
        }
        (void)HAL_TIM_IC_Stop(&capture->timer, PHASE_CAPTURE_TIMER_CHANNEL);
        capture->dma_running = 0U;
    }
    if (capture->irq_running != 0U)
    {
        if (HAL_TIM_IC_Stop_IT(&capture->timer,
                               PHASE_CAPTURE_TIMER_CHANNEL) != HAL_OK)
        {
            status = HAL_ERROR;
        }
        capture->irq_running = 0U;
    }
    return status;
}

/**
 * @brief  启动两个通道的 DMA 循环捕获模式
 *         配置完成后禁止半传输中断，仅保留传输完成中断用于包裹计数
 * @param  capture 捕获实例
 * @return HAL_OK 成功；HAL_ERROR 失败（并回滚已启动的资源）
 */
static HAL_StatusTypeDef PhaseCapture_StartDma(PhaseCapture_t *capture)
{
    HAL_StatusTypeDef status;

    PhaseCapture_ResetDmaProgress(capture);
    status = HAL_TIM_IC_Start_DMA(
        &capture->timer, PHASE_CAPTURE_TIMER_CHANNEL,
        s_phase_capture_dma_buffer,
        (uint16_t)PHASE_CAPTURE_DMA_BUFFER_COUNT);
    if (status != HAL_OK)
    {
        capture->dma_running = 0U;
        return status;
    }

    /*
     * 通过轮询 NDTR 获取当前写入位置，因此只需要全缓冲完成中断来扩展
     * 样本计数（包裹计数），半传输中断被禁止以降低中断负载。
     */
    __HAL_DMA_DISABLE_IT(&capture->dma, DMA_IT_HT);

    status = HAL_TIM_IC_Start_DMA(
        &capture->timer, PHASE_CAPTURE_FEEDBACK_CHANNEL,
        s_phase_capture_feedback_dma_buffer,
        (uint16_t)PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT);
    if (status != HAL_OK)
    {
        __HAL_TIM_DISABLE_DMA(&capture->timer, TIM_DMA_CC3);
        (void)HAL_DMA_Abort(&capture->dma);
        (void)HAL_TIM_IC_Stop(&capture->timer,
                              PHASE_CAPTURE_TIMER_CHANNEL);
        capture->dma_running = 0U;
        capture->feedback_dma_running = 0U;
        return status;
    }
    __HAL_DMA_DISABLE_IT(&capture->feedback_dma, DMA_IT_HT);

    capture->dma_running          = 1U;
    capture->feedback_dma_running = 1U;
    capture->irq_running          = 0U;
    return HAL_OK;
}

/**
 * @brief  启动中断捕获模式（主通道用 IT，反馈通道继续用 DMA）
 *         用于低频信号，减少 DMA 缓冲区溢出的等待时间
 * @param  capture 捕获实例
 * @return HAL_OK 或 HAL_ERROR
 */
static HAL_StatusTypeDef PhaseCapture_StartIrq(PhaseCapture_t *capture)
{
    HAL_StatusTypeDef status;

    PhaseCapture_ResetDmaProgress(capture);
    status = HAL_TIM_IC_Start_IT(
        &capture->timer, PHASE_CAPTURE_TIMER_CHANNEL);

    if (status != HAL_OK)
    {
        return status;
    }

    status = HAL_TIM_IC_Start_DMA(
        &capture->timer, PHASE_CAPTURE_FEEDBACK_CHANNEL,
        s_phase_capture_feedback_dma_buffer,
        (uint16_t)PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT);
    if (status != HAL_OK)
    {
        (void)HAL_TIM_IC_Stop_IT(
            &capture->timer, PHASE_CAPTURE_TIMER_CHANNEL);
        return status;
    }
    __HAL_DMA_DISABLE_IT(&capture->feedback_dma, DMA_IT_HT);
    capture->irq_running          = 1U;
    capture->dma_running          = 0U;
    capture->feedback_dma_running = 1U;
    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/*                        公共 API 实现                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief  设置默认配置：预分频器 = 0，周期 = 最大，时钟源频率为 TIM2 的
 *         输入时钟
 * @param  config 配置结构体指针（不可为 NULL）
 */
void PhaseCaptureConfig_Default(PhaseCaptureConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->prescaler     = 0U;
    config->period        = 0xFFFFFFFFUL;
    config->timer_tick_hz = PHASE_CAPTURE_TIMER_CLOCK_HZ;
}

/**
 * @brief  TIM 外设 MSP 初始化回调（由 HAL 库调用）
 *         使能 TIM2 和 GPIOB 时钟，配置两个捕获引脚为复用功能
 * @param  htim 定时器句柄
 */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if ((htim == NULL) || (htim->Instance != PHASE_CAPTURE_TIMER))
    {
        return;
    }

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = PHASE_CAPTURE_INPUT_GPIO_PIN | PHASE_CAPTURE_FEEDBACK_GPIO_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = PHASE_CAPTURE_INPUT_GPIO_AF;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(PHASE_CAPTURE_TIMER_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(PHASE_CAPTURE_TIMER_IRQn);
}

/**
 * @brief  初始化相位捕获实例
 *         配置定时器时基、输入捕获通道、DMA 以及初始状态
 * @param  capture 捕获实例指针
 * @param  config  配置参数
 * @return HAL_OK 成功；HAL_ERROR 参数无效或硬件初始化失败
 */
HAL_StatusTypeDef PhaseCapture_Init(PhaseCapture_t *capture,
                                    const PhaseCaptureConfig_t *config)
{
    TIM_IC_InitTypeDef channel = {0};

    if ((capture == NULL) || (config == NULL) ||
        (config->period == 0U) || (config->timer_tick_hz == 0U))
    {
        return HAL_ERROR;
    }

    (void)memset(capture, 0, sizeof(*capture));
    capture->config = *config;
    capture->timer.Instance               = PHASE_CAPTURE_TIMER;
    capture->timer.Init.Prescaler         = config->prescaler;
    capture->timer.Init.CounterMode       = TIM_COUNTERMODE_UP;
    capture->timer.Init.Period            = config->period;
    capture->timer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    capture->timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_IC_Init(&capture->timer) != HAL_OK)
    {
        return HAL_ERROR;
    }

    channel.ICPolarity  = TIM_ICPOLARITY_RISING;
    channel.ICSelection = TIM_ICSELECTION_DIRECTTI;
    channel.ICPrescaler = PHASE_CAPTURE_IC_PRESCALER;
    /*
     * 输入滤波器要求两个连续定时器时钟采样为高电平，能够滤除极窄的耦合
     * 尖峰，同时保留 10 MHz 方波（50 ns 高/低电平）的完整性。
     */
    channel.ICFilter = PHASE_CAPTURE_IC_FILTER;
    if (HAL_TIM_IC_ConfigChannel(&capture->timer, &channel,
                                 PHASE_CAPTURE_TIMER_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_TIM_IC_ConfigChannel(&capture->timer, &channel,
                                 PHASE_CAPTURE_FEEDBACK_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (PhaseCapture_InitDma(capture) != HAL_OK)
    {
        return HAL_ERROR;
    }

    capture->initialized             = 1U;
    capture->active_capture_cycles   = PHASE_CAPTURE_FAST_EDGE_COUNT;
    capture->period_capture_cycles   = PHASE_CAPTURE_FAST_EDGE_COUNT;
    s_phase_capture_active           = capture;
    return HAL_OK;
}

/**
 * @brief  启动捕获（DMA 模式）
 *         会先停止当前任何活动模式，重置硬件，并清零计数器
 * @param  capture 捕获实例
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef PhaseCapture_Start(PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return HAL_ERROR;
    }

    (void)PhaseCapture_StopCurrent(capture);
    PhaseCapture_ApplyEdgeCount(capture, PHASE_CAPTURE_FAST_EDGE_COUNT, 1U);
    __HAL_TIM_SET_COUNTER(&capture->timer, 0U);
    __HAL_TIM_CLEAR_FLAG(&capture->timer, TIM_FLAG_UPDATE);
    return PhaseCapture_StartDma(capture);
}

/**
 * @brief  停止所有捕获活动
 * @param  capture 捕获实例
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef PhaseCapture_Stop(PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return HAL_ERROR;
    }

    return PhaseCapture_StopCurrent(capture);
}

/**
 * @brief  获取主通道的最新周期测量值（单位：定时器 tick）
 *         内部调用 TakeMeasurement，丢弃捕获边沿数信息
 * @param  capture      捕获实例
 * @param  period_ticks 输出参数：周期对应的 timer tick 数
 * @return 1 有新的测量结果可用；0 无有效测量
 */
uint8_t PhaseCapture_TakePeriod(PhaseCapture_t *capture,
                                uint32_t *period_ticks)
{
    uint32_t capture_cycles;

    return PhaseCapture_TakeMeasurement(capture, period_ticks, &capture_cycles);
}

/**
 * @brief  从中断捕获模式获取一次测量结果
 *         直接读取实例中由中断服务程序设置的已就绪标志和数据
 * @param  capture        捕获实例
 * @param  period_ticks   输出：周期 tick 数
 * @param  capture_cycles 输出：对应的捕获边沿数
 * @return 1 有数据；0 无数据
 */
static uint8_t PhaseCapture_TakeIrqMeasurement(
    PhaseCapture_t *capture,
    uint32_t *period_ticks,
    uint32_t *capture_cycles)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t ready;

    __disable_irq();
    ready = capture->capture_ready;
    if (ready != 0U)
    {
        *period_ticks    = capture->period_ticks;
        *capture_cycles  = capture->period_capture_cycles;
        capture->capture_ready = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return ready;
}

/**
 * @brief  从 DMA 循环缓冲区中提取主通道的累积周期测量值
 *
 * 采用“NDTR + 包裹计数”获取缓冲区写指针，读取最新 CCR 值，通过与上一次
 * 记录的差值得到经过的 tick 数和捕获边沿总数，从而计算平均周期。该算法
 * 可以避免锁存问题，且在信号频率较高时提供稳定的平均结果。
 *
 * @param  capture        捕获实例
 * @param  period_ticks   输出：周期 tick 数
 * @param  capture_cycles 输出：实际累计的边沿总数
 * @return 1 有新的有效数据；0 无数据或数据异常
 */
static uint8_t PhaseCapture_TakeDmaMeasurement(
    PhaseCapture_t *capture,
    uint32_t *period_ticks,
    uint32_t *capture_cycles)
{
    uint32_t wrap_before;
    uint32_t wrap_after;
    uint32_t remaining;
    uint32_t completed;
    uint32_t total_samples;
    uint32_t sample_delta;
    uint32_t latest_index;
    uint32_t captured;
    uint32_t delta_ticks;
    uint32_t attempts;

    /* 重复尝试读取 NDTR 和 wrap_count，确保两者一致，防止 DMA 在读取期间完成 */
    for (attempts = 0U; attempts < 3U; ++attempts)
    {
        wrap_before = capture->dma_wrap_count;
        remaining   = __HAL_DMA_GET_COUNTER(&capture->dma);
        wrap_after  = capture->dma_wrap_count;
        if (wrap_before == wrap_after)
        {
            break;
        }
    }
    if ((wrap_before != wrap_after) ||
        (remaining > PHASE_CAPTURE_DMA_BUFFER_COUNT))
    {
        return 0U;
    }

    completed = PHASE_CAPTURE_DMA_BUFFER_COUNT - remaining;
    total_samples = wrap_after * PHASE_CAPTURE_DMA_BUFFER_COUNT + completed;
    if ((total_samples == 0U) ||
        (total_samples == capture->dma_last_sample_count))
    {
        return 0U; /* 无新样本或首次无历史 */
    }

    /* 获取缓冲区中最新的时间戳 */
    latest_index = (completed == 0U) ?
                   (PHASE_CAPTURE_DMA_BUFFER_COUNT - 1U) :
                   (completed - 1U);
    captured = s_phase_capture_dma_buffer[latest_index];
    if (capture->dma_has_last_sample == 0U)
    {
        capture->dma_last_timestamp    = captured;
        capture->dma_last_sample_count = total_samples;
        capture->dma_has_last_sample   = 1U;
        return 0U;
    }

    sample_delta = total_samples - capture->dma_last_sample_count;
    /* 计算时间差，处理 32 位计数器回绕 */
    delta_ticks = (captured >= capture->dma_last_timestamp) ?
                  (captured - capture->dma_last_timestamp) :
                  (uint32_t)(0xFFFFFFFFUL - capture->dma_last_timestamp + 1UL + captured);
    capture->dma_last_timestamp    = captured;
    capture->dma_last_sample_count = total_samples;
    if ((sample_delta == 0U) || (delta_ticks == 0U) ||
        (sample_delta > (0xFFFFFFFFUL / capture->active_capture_cycles)))
    {
        return 0U; /* 数据不合理，丢弃 */
    }

    *period_ticks   = delta_ticks;
    *capture_cycles = sample_delta * capture->active_capture_cycles;
    return 1U;
}

/**
 * @brief  从反馈通道 DMA 缓冲区获取一次测量（原理与主通道完全相同）
 *
 * @param  capture        捕获实例
 * @param  period_ticks   输出：周期 tick 数
 * @param  capture_cycles 输出：累计边沿总数
 * @return 1 有有效数据；0 无数据
 */
static uint8_t PhaseCapture_TakeFeedbackDmaMeasurement(
    PhaseCapture_t *capture,
    uint32_t *period_ticks,
    uint32_t *capture_cycles)
{
    uint32_t wrap_before;
    uint32_t wrap_after;
    uint32_t remaining;
    uint32_t completed;
    uint32_t total_samples;
    uint32_t sample_delta;
    uint32_t latest_index;
    uint32_t captured;
    uint32_t delta_ticks;
    uint32_t attempts;

    for (attempts = 0U; attempts < 3U; ++attempts)
    {
        wrap_before = capture->feedback_dma_wrap_count;
        remaining   = __HAL_DMA_GET_COUNTER(&capture->feedback_dma);
        wrap_after  = capture->feedback_dma_wrap_count;
        if (wrap_before == wrap_after)
        {
            break;
        }
    }
    if ((wrap_before != wrap_after) ||
        (remaining > PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT))
    {
        return 0U;
    }

    completed = PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT - remaining;
    total_samples = wrap_after * PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT + completed;
    if ((total_samples == 0U) ||
        (total_samples == capture->feedback_dma_last_sample_count))
    {
        return 0U;
    }

    latest_index = (completed == 0U) ?
                   (PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT - 1U) :
                   (completed - 1U);
    captured = s_phase_capture_feedback_dma_buffer[latest_index];
    if (capture->feedback_dma_has_last_sample == 0U)
    {
        capture->feedback_dma_last_timestamp    = captured;
        capture->feedback_dma_last_sample_count = total_samples;
        capture->feedback_dma_has_last_sample   = 1U;
        return 0U;
    }

    sample_delta = total_samples - capture->feedback_dma_last_sample_count;
    delta_ticks = (captured >= capture->feedback_dma_last_timestamp) ?
                  (captured - capture->feedback_dma_last_timestamp) :
                  (uint32_t)(0xFFFFFFFFUL - capture->feedback_dma_last_timestamp + 1UL + captured);
    capture->feedback_dma_last_timestamp    = captured;
    capture->feedback_dma_last_sample_count = total_samples;
    if ((sample_delta == 0U) || (delta_ticks == 0U) ||
        (sample_delta > (0xFFFFFFFFUL / capture->active_capture_cycles)))
    {
        return 0U;
    }

    *period_ticks   = delta_ticks;
    *capture_cycles = sample_delta * capture->active_capture_cycles;
    return 1U;
}

/**
 * @brief  通用测量获取接口，根据当前运行模式调用对应的内部函数
 *         同时处理 DMA 重启请求（由中断回调触发）
 * @param  capture        捕获实例
 * @param  period_ticks   输出：周期 tick 数
 * @param  capture_cycles 输出：累计边沿数
 * @return 1 有新的测量；0 无测量
 */
uint8_t PhaseCapture_TakeMeasurement(PhaseCapture_t *capture,
                                     uint32_t *period_ticks,
                                     uint32_t *capture_cycles)
{
    uint8_t ready = 0U;
    uint8_t restart_dma;
    uint32_t primask;

    if ((capture == NULL) || (period_ticks == NULL) ||
        (capture_cycles == NULL) || (capture->initialized == 0U))
    {
        return 0U;
    }

    if (capture->dma_running != 0U)
    {
        ready = PhaseCapture_TakeDmaMeasurement(
            capture, period_ticks, capture_cycles);
    }
    else if (capture->irq_running != 0U)
    {
        ready = PhaseCapture_TakeIrqMeasurement(
            capture, period_ticks, capture_cycles);
    }

    /* 安全检查：处理中断回调中触发的 DMA 重启请求 */
    primask = __get_PRIMASK();
    __disable_irq();
    restart_dma = capture->dma_restart_requested;
    capture->dma_restart_requested = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }
    if (restart_dma != 0U)
    {
        (void)PhaseCapture_SetEdgeCount(
            capture, PHASE_CAPTURE_FAST_EDGE_COUNT);
    }
    return ready;
}

/**
 * @brief  获取反馈通道的最新测量值（仅 DMA 模式）
 * @param  capture        捕获实例
 * @param  period_ticks   输出：周期 tick 数
 * @param  capture_cycles 输出：累计边沿数
 * @return 1 有数据；0 无数据或模式不匹配
 */
uint8_t PhaseCapture_TakeFeedbackMeasurement(
    PhaseCapture_t *capture,
    uint32_t *period_ticks,
    uint32_t *capture_cycles)
{
    if ((capture == NULL) || (period_ticks == NULL) ||
        (capture_cycles == NULL) ||
        (capture->initialized == 0U) ||
        (capture->feedback_dma_running == 0U))
    {
        return 0U;
    }

    return PhaseCapture_TakeFeedbackDmaMeasurement(
        capture, period_ticks, capture_cycles);
}

uint8_t PhaseCapture_GetPhaseError(
    PhaseCapture_t *capture,
    uint32_t input_period_ticks,
    uint32_t input_capture_cycles,
    uint8_t frequency_ratio,
    float target_phase_deg,
    float *phase_error_deg)
{
    PhaseCaptureDmaSnapshot_t input_snapshot;
    PhaseCaptureDmaSnapshot_t feedback_snapshot;
    uint32_t input_pair_count;
    uint32_t feedback_search_count;
    uint32_t input_offset;
    uint32_t feedback_offset;
    uint32_t input_timestamp;
    uint32_t output_timestamp;
    uint32_t latest_input_timestamp;
    uint32_t latest_output_timestamp;
    uint32_t nearest_distance;
    uint32_t distance;
    uint32_t pair_count;
    int32_t delta_ticks;
    int32_t nearest_delta_ticks;
    float input_cycle_ticks;
    float output_cycle_ticks;
    float delta_cycle_ticks;
    float phase_relation_deg;
    float pair_error_deg;
    float radians;
    float sin_sum;
    float cos_sum;
    float resultant;

    if ((capture == NULL) || (phase_error_deg == NULL) ||
        (input_period_ticks == 0U) || (input_capture_cycles == 0U) ||
        ((frequency_ratio != 1U) && (frequency_ratio != 2U)) ||
        (capture->dma_running == 0U) ||
        (capture->feedback_dma_running == 0U))
    {
        return 0U;
    }

    if ((PhaseCapture_GetDmaSnapshot(
             &capture->dma, &capture->dma_wrap_count,
             PHASE_CAPTURE_DMA_BUFFER_COUNT,
             &input_snapshot) == 0U) ||
        (PhaseCapture_GetDmaSnapshot(
             &capture->feedback_dma,
             &capture->feedback_dma_wrap_count,
             PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT,
             &feedback_snapshot) == 0U))
    {
        return 0U;
    }

    latest_input_timestamp = PhaseCapture_HistoryTimestamp(
        s_phase_capture_dma_buffer,
        PHASE_CAPTURE_DMA_BUFFER_COUNT,
        input_snapshot.latest_index, 0U);
    latest_output_timestamp = PhaseCapture_HistoryTimestamp(
        s_phase_capture_feedback_dma_buffer,
        PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT,
        feedback_snapshot.latest_index, 0U);
    if (latest_input_timestamp ==
        capture->phase_last_input_timestamp)
    {
        return 0U;
    }
    capture->phase_last_input_timestamp = latest_input_timestamp;
    capture->phase_last_feedback_timestamp = latest_output_timestamp;

    input_cycle_ticks = (float)input_period_ticks /
                        (float)input_capture_cycles;
    if (input_cycle_ticks <= 0.0f)
    {
        return 0U;
    }
    output_cycle_ticks =
        input_cycle_ticks / (float)frequency_ratio;

    input_pair_count = input_snapshot.available_samples;
    if (input_pair_count > PHASE_CAPTURE_PAIR_INPUT_COUNT)
    {
        input_pair_count = PHASE_CAPTURE_PAIR_INPUT_COUNT;
    }
    feedback_search_count = feedback_snapshot.available_samples;
    if (feedback_search_count >
        PHASE_CAPTURE_PAIR_FEEDBACK_SEARCH_COUNT)
    {
        feedback_search_count =
            PHASE_CAPTURE_PAIR_FEEDBACK_SEARCH_COUNT;
    }
    if ((input_pair_count < PHASE_CAPTURE_PAIR_MIN_COUNT) ||
        (feedback_search_count == 0U))
    {
        return 0U;
    }

    sin_sum = 0.0f;
    cos_sum = 0.0f;
    pair_count = 0U;
    for (input_offset = 0U;
         input_offset < input_pair_count;
         ++input_offset)
    {
        input_timestamp = PhaseCapture_HistoryTimestamp(
            s_phase_capture_dma_buffer,
            PHASE_CAPTURE_DMA_BUFFER_COUNT,
            input_snapshot.latest_index, input_offset);
        nearest_distance = 0xFFFFFFFFUL;
        nearest_delta_ticks = 0;

        /*
         * CH3 and CH4 have independent DMA streams and input-capture
         * prescalers. Pair each recent CH3 event with the temporally nearest
         * CH4 event instead of subtracting two unrelated "latest" entries.
         */
        for (feedback_offset = 0U;
             feedback_offset < feedback_search_count;
             ++feedback_offset)
        {
            output_timestamp = PhaseCapture_HistoryTimestamp(
                s_phase_capture_feedback_dma_buffer,
                PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT,
                feedback_snapshot.latest_index, feedback_offset);
            delta_ticks =
                (int32_t)(output_timestamp - input_timestamp);
            distance = (delta_ticks < 0) ?
                (uint32_t)(-(int64_t)delta_ticks) :
                (uint32_t)delta_ticks;
            if (distance < nearest_distance)
            {
                nearest_distance = distance;
                nearest_delta_ticks = delta_ticks;
            }
        }

        /*
         * Remove whole output cycles after pairing. A later output edge
         * means the output lags, hence the minus sign.
         */
        delta_cycle_ticks = fmodf(
            (float)nearest_delta_ticks, output_cycle_ticks);
        if (delta_cycle_ticks > (0.5f * output_cycle_ticks))
        {
            delta_cycle_ticks -= output_cycle_ticks;
        }
        else if (delta_cycle_ticks <
                 (-0.5f * output_cycle_ticks))
        {
            delta_cycle_ticks += output_cycle_ticks;
        }
        phase_relation_deg =
            -360.0f * delta_cycle_ticks / output_cycle_ticks;
        pair_error_deg = PhaseCapture_NormalizeDegrees(
            phase_relation_deg - target_phase_deg);
        radians = pair_error_deg * PHASE_CAPTURE_PI_F / 180.0f;
        sin_sum += sinf(radians);
        cos_sum += cosf(radians);
        ++pair_count;
    }

    if (pair_count < PHASE_CAPTURE_PAIR_MIN_COUNT)
    {
        return 0U;
    }
    resultant = sqrtf(sin_sum * sin_sum + cos_sum * cos_sum) /
                (float)pair_count;
    if (resultant < PHASE_CAPTURE_PAIR_MIN_RESULTANT)
    {
        return 0U;
    }
    *phase_error_deg = atan2f(sin_sum, cos_sum) *
                       180.0f / PHASE_CAPTURE_PI_F;
    return 1U;
}

/**
 * @brief  动态切换边沿捕获数量，并在必要时改变运行模式
 *         - 若目标边沿数与当前相同且运行模式匹配，直接返回成功
 *         - 若切换到慢速（PHASE_CAPTURE_SLOW_EDGE_COUNT），使用中断模式
 *         - 否则使用 DMA 模式
 * @param  capture        捕获实例
 * @param  capture_cycles 目标边沿数
 * @return HAL_OK 成功；HAL_ERROR 参数错误或切换失败
 */
HAL_StatusTypeDef PhaseCapture_SetEdgeCount(PhaseCapture_t *capture,
                                            uint32_t capture_cycles)
{
    HAL_StatusTypeDef status;

    if ((capture == NULL) || (capture->initialized == 0U) ||
        (PhaseCapture_EdgeCountToPrescaler(capture_cycles) == 0xFFFFFFFFUL))
    {
        return HAL_ERROR;
    }
    /* 若已经处于目标边沿数且模式相符，无需操作 */
    if (((capture_cycles == PHASE_CAPTURE_SLOW_EDGE_COUNT) &&
         (capture->irq_running != 0U) &&
         (capture->active_capture_cycles == capture_cycles)) ||
        ((capture_cycles != PHASE_CAPTURE_SLOW_EDGE_COUNT) &&
         (capture->dma_running != 0U) &&
         (capture->active_capture_cycles == capture_cycles)))
    {
        return HAL_OK;
    }

    (void)PhaseCapture_StopCurrent(capture);
    PhaseCapture_ApplyEdgeCount(capture, capture_cycles, 1U);
    if (capture_cycles == PHASE_CAPTURE_SLOW_EDGE_COUNT)
    {
        status = PhaseCapture_StartIrq(capture);
    }
    else
    {
        status = PhaseCapture_StartDma(capture);
    }
    return status;
}

/**
 * @brief  获取当前有效捕获边沿数
 * @param  capture 捕获实例
 * @return 边沿数；若实例无效则返回 0
 */
uint32_t PhaseCapture_GetEdgeCount(const PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return 0U;
    }
    return capture->active_capture_cycles;
}

/**
 * @brief  获取主通道 DMA 错误计数（过捕获等）
 */
uint32_t PhaseCapture_GetDmaErrorCount(const PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return 0U;
    }
    return capture->dma_error_count;
}

/**
 * @brief  获取反馈通道 DMA 错误计数
 */
uint32_t PhaseCapture_GetFeedbackDmaErrorCount(
    const PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return 0U;
    }
    return capture->feedback_dma_error_count;
}

/* -------------------------------------------------------------------------- */
/*                    中断服务程序与 HAL 回调                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief  处理一次主通道捕获事件（由 DMA 完成中断或定时器捕获中断调用）
 *         计算相邻两次捕获的时间差，并将结果存入实例，置位就绪标志。
 *         若在慢速边沿模式下测得的周期足够短，则自动切换至快速 DMA 模式。
 * @param  htim 定时器句柄
 */
static void PhaseCapture_RecordCaptured(TIM_HandleTypeDef *htim)
{
    uint32_t captured;
    uint32_t capture_cycles;
    PhaseCapture_t *capture;
    uint32_t delta;

    if ((htim == NULL) || (htim->Instance != PHASE_CAPTURE_TIMER))
    {
        return;
    }

    capture = s_phase_capture_active;
    if ((capture == NULL) || (&capture->timer != htim))
    {
        capture = &g_phase_capture;
    }
    captured       = PHASE_CAPTURE_TIMER->CCR3;
    capture_cycles = capture->active_capture_cycles;
    if (capture->has_last_capture != 0U)
    {
        delta = (captured >= capture->last_capture) ?
                (captured - capture->last_capture) :
                (uint32_t)(0xFFFFFFFFUL - capture->last_capture + 1UL + captured);
        if (delta > 0U)
        {
            capture->period_ticks          = delta;
            capture->period_capture_cycles = capture_cycles;
            capture->capture_ready         = 1U;

            /*
             * 若当前为慢速边沿模式，且测量周期小于快速切换阈值，
             * 表明输入频率升高，此时立刻切回快速 DMA 模式，
             * 并请求主任务重新启动 DMA，防止丢失高频信号。
             */
            if ((capture_cycles == PHASE_CAPTURE_SLOW_EDGE_COUNT) &&
                (delta <= PHASE_CAPTURE_FAST_SWITCH_TICKS))
            {
                PhaseCapture_ApplyEdgeCount(
                    capture, PHASE_CAPTURE_FAST_EDGE_COUNT, 0U);
                capture->dma_restart_requested = 1U;
                __HAL_TIM_DISABLE_IT(
                    &capture->timer, PHASE_CAPTURE_TIMER_IT);
                return;
            }
        }
    }
    capture->last_capture     = captured;
    capture->has_last_capture = 1U;
}

/**
 * @brief  TIM 输入捕获完成回调（由 HAL 中断处理调用）
 *         根据通道和运行模式更新包裹计数或调用周期记录函数
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    PhaseCapture_t *capture = s_phase_capture_active;

    if ((htim == NULL) || (htim->Instance != PHASE_CAPTURE_TIMER))
    {
        return;
    }
    if ((capture == NULL) || (&capture->timer != htim))
    {
        capture = &g_phase_capture;
    }
    /* 主通道 DMA 模式：递增包裹计数 */
    if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) &&
        (capture->dma_running != 0U))
    {
        ++capture->dma_wrap_count;
        return;
    }
    /* 反馈通道 DMA 模式：递增反馈包裹计数 */
    if ((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) &&
        (capture->feedback_dma_running != 0U))
    {
        ++capture->feedback_dma_wrap_count;
        return;
    }
    /* 否则为中断模式捕获，进行记录 */
    PhaseCapture_RecordCaptured(htim);
}

/**
 * @brief  定时器错误回调，累加相应的 DMA 错误计数
 */
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    PhaseCapture_t *capture = s_phase_capture_active;

    if ((htim != NULL) && (htim->Instance == PHASE_CAPTURE_TIMER))
    {
        if ((capture == NULL) || (&capture->timer != htim))
        {
            capture = &g_phase_capture;
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            ++capture->feedback_dma_error_count;
        }
        else
        {
            ++capture->dma_error_count;
        }
    }
}

/**
 * @brief  TIM2 中断处理函数（覆盖弱定义）
 *         处理捕获中断与过捕获标志
 */
void TIM2_IRQHandler(void)
{
    TIM_HandleTypeDef *timer = &g_phase_capture.timer;

    if ((__HAL_TIM_GET_FLAG(timer, PHASE_CAPTURE_TIMER_FLAG) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(timer, PHASE_CAPTURE_TIMER_IT) != RESET))
    {
        __HAL_TIM_CLEAR_IT(timer, PHASE_CAPTURE_TIMER_IT);
        PhaseCapture_RecordCaptured(timer);
    }
    if (__HAL_TIM_GET_FLAG(timer, PHASE_CAPTURE_OVERCAPTURE_FLAG) != RESET)
    {
        __HAL_TIM_CLEAR_FLAG(timer, PHASE_CAPTURE_OVERCAPTURE_FLAG);
    }
}

/**
 * @brief  主通道 DMA 中断处理函数（由用户注册到中断向量表）
 */
void PhaseCapture_DmaIrqHandler(void)
{
    if (g_phase_capture.initialized != 0U)
    {
        HAL_DMA_IRQHandler(&g_phase_capture.dma);
    }
}

/**
 * @brief  反馈通道 DMA 中断处理函数
 */
void PhaseCapture_FeedbackDmaIrqHandler(void)
{
    if (g_phase_capture.initialized != 0U)
    {
        HAL_DMA_IRQHandler(&g_phase_capture.feedback_dma);
    }
}

/* -------------------------------------------------------------------------- */
/*                        简易全局访问函数                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief  使用默认配置初始化全局捕获实例
 */
void phase_capture_init(void)
{
    PhaseCaptureConfig_t config;

    PhaseCaptureConfig_Default(&config);
    (void)PhaseCapture_Init(&g_phase_capture, &config);
}

/**
 * @brief  启动全局实例捕获
 */
HAL_StatusTypeDef phase_capture_start(void)
{
    return PhaseCapture_Start(&g_phase_capture);
}

/**
 * @brief  获取全局实例的主信号周期
 * @param  period_ticks 输出：周期 tick 数
 * @return 1 成功获取；0 无数据
 */
uint8_t phase_capture_take_period(uint32_t *period_ticks)
{
    return PhaseCapture_TakePeriod(&g_phase_capture, period_ticks);
}
