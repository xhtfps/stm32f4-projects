#include "phase_capture.h"

#include <stddef.h>
#include <string.h>

PhaseCapture_t g_phase_capture;
static PhaseCapture_t *s_phase_capture_active;
static uint32_t
    s_phase_capture_dma_buffer[PHASE_CAPTURE_DMA_BUFFER_COUNT];

static uint32_t PhaseCapture_EdgeCountToPrescaler(uint32_t capture_cycles)
{
    switch (capture_cycles)
    {
        case 1U:
            return TIM_ICPSC_DIV1;
        case 2U:
            return TIM_ICPSC_DIV2;
        case 4U:
            return TIM_ICPSC_DIV4;
        case 8U:
            return TIM_ICPSC_DIV8;
        default:
            return 0xFFFFFFFFUL;
    }
}

static void PhaseCapture_ApplyEdgeCount(PhaseCapture_t *capture,
                                        uint32_t capture_cycles,
                                        uint8_t discard_ready)
{
    uint32_t ic_prescaler =
        PhaseCapture_EdgeCountToPrescaler(capture_cycles);

    __HAL_TIM_SET_ICPRESCALER(&capture->timer,
                              PHASE_CAPTURE_TIMER_CHANNEL,
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
    __HAL_TIM_CLEAR_IT(&capture->timer, PHASE_CAPTURE_TIMER_IT);
    __HAL_TIM_CLEAR_FLAG(&capture->timer,
                         PHASE_CAPTURE_OVERCAPTURE_FLAG);
}

static HAL_StatusTypeDef PhaseCapture_InitDma(PhaseCapture_t *capture)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    (void)memset(&capture->dma, 0, sizeof(capture->dma));
    capture->dma.Instance = PHASE_CAPTURE_DMA_STREAM;
    capture->dma.Init.Channel = PHASE_CAPTURE_DMA_CHANNEL;
    capture->dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
    capture->dma.Init.PeriphInc = DMA_PINC_DISABLE;
    capture->dma.Init.MemInc = DMA_MINC_ENABLE;
    capture->dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    capture->dma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    capture->dma.Init.Mode = DMA_CIRCULAR;
    capture->dma.Init.Priority = DMA_PRIORITY_HIGH;
    capture->dma.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    capture->dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    capture->dma.Init.MemBurst = DMA_MBURST_SINGLE;
    capture->dma.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&capture->dma) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_LINKDMA(&capture->timer, hdma[TIM_DMA_ID_CC3],
                  capture->dma);
    HAL_NVIC_SetPriority(PHASE_CAPTURE_DMA_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(PHASE_CAPTURE_DMA_IRQn);
    return HAL_OK;
}

static void PhaseCapture_ResetDmaProgress(PhaseCapture_t *capture)
{
    capture->dma_wrap_count = 0U;
    capture->dma_last_sample_count = 0U;
    capture->dma_last_timestamp = 0U;
    capture->dma_has_last_sample = 0U;
}

static HAL_StatusTypeDef PhaseCapture_StopCurrent(PhaseCapture_t *capture)
{
    HAL_StatusTypeDef status = HAL_OK;

    capture->dma_restart_requested = 0U;
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
        (void)HAL_TIM_IC_Stop(&capture->timer,
                              PHASE_CAPTURE_TIMER_CHANNEL);
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
     * Polling NDTR supplies the current write position. Only the full-buffer
     * interrupt is needed to extend the sample count across circular wraps.
     */
    __HAL_DMA_DISABLE_IT(&capture->dma, DMA_IT_HT);
    capture->dma_running = 1U;
    capture->irq_running = 0U;
    return HAL_OK;
}

static HAL_StatusTypeDef PhaseCapture_StartIrq(PhaseCapture_t *capture)
{
    HAL_StatusTypeDef status = HAL_TIM_IC_Start_IT(
        &capture->timer, PHASE_CAPTURE_TIMER_CHANNEL);

    if (status == HAL_OK)
    {
        capture->irq_running = 1U;
        capture->dma_running = 0U;
    }
    return status;
}

void PhaseCaptureConfig_Default(PhaseCaptureConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->prescaler = 0U;
    config->period = 0xFFFFFFFFUL;
    config->timer_tick_hz = PHASE_CAPTURE_TIMER_CLOCK_HZ;
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if ((htim == NULL) || (htim->Instance != PHASE_CAPTURE_TIMER))
    {
        return;
    }

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin = PHASE_CAPTURE_GPIO_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = PHASE_CAPTURE_GPIO_AF;
    HAL_GPIO_Init(PHASE_CAPTURE_GPIO_PORT, &gpio);

    HAL_NVIC_SetPriority(PHASE_CAPTURE_TIMER_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(PHASE_CAPTURE_TIMER_IRQn);
}

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
    capture->timer.Instance = PHASE_CAPTURE_TIMER;
    capture->timer.Init.Prescaler = config->prescaler;
    capture->timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    capture->timer.Init.Period = config->period;
    capture->timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    capture->timer.Init.AutoReloadPreload =
        TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_IC_Init(&capture->timer) != HAL_OK)
    {
        return HAL_ERROR;
    }

    channel.ICPolarity = TIM_ICPOLARITY_RISING;
    channel.ICSelection = TIM_ICSELECTION_DIRECTTI;
    channel.ICPrescaler = PHASE_CAPTURE_IC_PRESCALER;
    channel.ICFilter = 0U;
    if (HAL_TIM_IC_ConfigChannel(&capture->timer, &channel,
                                 PHASE_CAPTURE_TIMER_CHANNEL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (PhaseCapture_InitDma(capture) != HAL_OK)
    {
        return HAL_ERROR;
    }

    capture->initialized = 1U;
    capture->active_capture_cycles =
        PHASE_CAPTURE_FAST_EDGE_COUNT;
    capture->period_capture_cycles =
        PHASE_CAPTURE_FAST_EDGE_COUNT;
    s_phase_capture_active = capture;
    return HAL_OK;
}

HAL_StatusTypeDef PhaseCapture_Start(PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return HAL_ERROR;
    }

    (void)PhaseCapture_StopCurrent(capture);
    PhaseCapture_ApplyEdgeCount(capture,
                                PHASE_CAPTURE_FAST_EDGE_COUNT, 1U);
    __HAL_TIM_SET_COUNTER(&capture->timer, 0U);
    __HAL_TIM_CLEAR_FLAG(&capture->timer, TIM_FLAG_UPDATE);
    return PhaseCapture_StartDma(capture);
}

HAL_StatusTypeDef PhaseCapture_Stop(PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return HAL_ERROR;
    }

    return PhaseCapture_StopCurrent(capture);
}

uint8_t PhaseCapture_TakePeriod(PhaseCapture_t *capture,
                                uint32_t *period_ticks)
{
    uint32_t capture_cycles;

    return PhaseCapture_TakeMeasurement(capture, period_ticks,
                                        &capture_cycles);
}

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
        *period_ticks = capture->period_ticks;
        *capture_cycles = capture->period_capture_cycles;
        capture->capture_ready = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return ready;
}

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

    for (attempts = 0U; attempts < 3U; ++attempts)
    {
        wrap_before = capture->dma_wrap_count;
        remaining = __HAL_DMA_GET_COUNTER(&capture->dma);
        wrap_after = capture->dma_wrap_count;
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
    total_samples =
        wrap_after * PHASE_CAPTURE_DMA_BUFFER_COUNT + completed;
    if ((total_samples == 0U) ||
        (total_samples == capture->dma_last_sample_count))
    {
        return 0U;
    }

    latest_index = (completed == 0U) ?
                   (PHASE_CAPTURE_DMA_BUFFER_COUNT - 1U) :
                   (completed - 1U);
    captured = s_phase_capture_dma_buffer[latest_index];
    if (capture->dma_has_last_sample == 0U)
    {
        capture->dma_last_timestamp = captured;
        capture->dma_last_sample_count = total_samples;
        capture->dma_has_last_sample = 1U;
        return 0U;
    }

    sample_delta = total_samples - capture->dma_last_sample_count;
    delta_ticks = (captured >= capture->dma_last_timestamp) ?
                  (captured - capture->dma_last_timestamp) :
                  (uint32_t)(0xFFFFFFFFUL -
                             capture->dma_last_timestamp +
                             1UL + captured);
    capture->dma_last_timestamp = captured;
    capture->dma_last_sample_count = total_samples;
    if ((sample_delta == 0U) || (delta_ticks == 0U) ||
        (sample_delta >
         (0xFFFFFFFFUL / capture->active_capture_cycles)))
    {
        return 0U;
    }

    *period_ticks = delta_ticks;
    *capture_cycles =
        sample_delta * capture->active_capture_cycles;
    return 1U;
}

uint8_t PhaseCapture_TakeMeasurement(PhaseCapture_t *capture,
                                     uint32_t *period_ticks,
                                     uint32_t *capture_cycles)
{
    uint8_t ready = 0U;
    uint8_t restart_dma;
    uint32_t primask;

    if ((capture == NULL) || (period_ticks == NULL) ||
        (capture_cycles == NULL) ||
        (capture->initialized == 0U))
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

HAL_StatusTypeDef PhaseCapture_SetEdgeCount(PhaseCapture_t *capture,
                                            uint32_t capture_cycles)
{
    HAL_StatusTypeDef status;

    if ((capture == NULL) || (capture->initialized == 0U) ||
        (PhaseCapture_EdgeCountToPrescaler(capture_cycles) ==
         0xFFFFFFFFUL))
    {
        return HAL_ERROR;
    }
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

uint32_t PhaseCapture_GetEdgeCount(const PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return 0U;
    }
    return capture->active_capture_cycles;
}

uint32_t PhaseCapture_GetDmaErrorCount(const PhaseCapture_t *capture)
{
    if ((capture == NULL) || (capture->initialized == 0U))
    {
        return 0U;
    }
    return capture->dma_error_count;
}

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
    captured = PHASE_CAPTURE_TIMER->CCR3;
    capture_cycles = capture->active_capture_cycles;
    if (capture->has_last_capture != 0U)
    {
        delta = (captured >= capture->last_capture) ?
                (captured - capture->last_capture) :
                (uint32_t)(0xFFFFFFFFUL - capture->last_capture +
                           1UL + captured);
        if (delta > 0U)
        {
            capture->period_ticks = delta;
            capture->period_capture_cycles = capture_cycles;
            capture->capture_ready = 1U;

            /*
             * Preserve the first fast result, stop further CC interrupts and
             * let the task switch the channel back to circular DMA.
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
    capture->last_capture = captured;
    capture->has_last_capture = 1U;
}

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
    if (capture->dma_running != 0U)
    {
        ++capture->dma_wrap_count;
        return;
    }
    PhaseCapture_RecordCaptured(htim);
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    PhaseCapture_t *capture = s_phase_capture_active;

    if ((htim != NULL) && (htim->Instance == PHASE_CAPTURE_TIMER))
    {
        if ((capture == NULL) || (&capture->timer != htim))
        {
            capture = &g_phase_capture;
        }
        ++capture->dma_error_count;
    }
}

void TIM2_IRQHandler(void)
{
    TIM_HandleTypeDef *timer = &g_phase_capture.timer;

    if ((__HAL_TIM_GET_FLAG(timer, PHASE_CAPTURE_TIMER_FLAG) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(timer, PHASE_CAPTURE_TIMER_IT) != RESET))
    {
        __HAL_TIM_CLEAR_IT(timer, PHASE_CAPTURE_TIMER_IT);
        PhaseCapture_RecordCaptured(timer);
    }
    if (__HAL_TIM_GET_FLAG(timer,
                           PHASE_CAPTURE_OVERCAPTURE_FLAG) != RESET)
    {
        __HAL_TIM_CLEAR_FLAG(timer,
                             PHASE_CAPTURE_OVERCAPTURE_FLAG);
    }
}

void PhaseCapture_DmaIrqHandler(void)
{
    if (g_phase_capture.initialized != 0U)
    {
        HAL_DMA_IRQHandler(&g_phase_capture.dma);
    }
}

void phase_capture_init(void)
{
    PhaseCaptureConfig_t config;

    PhaseCaptureConfig_Default(&config);
    (void)PhaseCapture_Init(&g_phase_capture, &config);
}

HAL_StatusTypeDef phase_capture_start(void)
{
    return PhaseCapture_Start(&g_phase_capture);
}

uint8_t phase_capture_take_period(uint32_t *period_ticks)
{
    return PhaseCapture_TakePeriod(&g_phase_capture, period_ticks);
}
