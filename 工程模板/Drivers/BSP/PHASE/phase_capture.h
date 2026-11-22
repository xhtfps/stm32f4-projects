#ifndef PHASE_CAPTURE_H
#define PHASE_CAPTURE_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

/*
 * PB10/TIM2_CH3 leaves PA0/WKUP and PA1/PA2/PA3 ADC inputs untouched.
 * High frequency capture uses DMA1 Stream1/Channel3 in a circular buffer.
 * Slow acquisition falls back to one interrupt per edge. This keeps the
 * original PB10 wiring while protecting PS/2 and LCD responsiveness.
 */
#define PHASE_CAPTURE_TIMER             TIM2
#define PHASE_CAPTURE_TIMER_IRQn        TIM2_IRQn
#define PHASE_CAPTURE_TIMER_CHANNEL     TIM_CHANNEL_3
#define PHASE_CAPTURE_TIMER_CLOCK_HZ    84000000U
#define PHASE_CAPTURE_GPIO_PORT         GPIOB
#define PHASE_CAPTURE_GPIO_PIN          GPIO_PIN_10
#define PHASE_CAPTURE_GPIO_AF           GPIO_AF1_TIM2
#define PHASE_CAPTURE_FAST_EDGE_COUNT   8U
#define PHASE_CAPTURE_SLOW_EDGE_COUNT   1U
#define PHASE_CAPTURE_EDGE_COUNT        PHASE_CAPTURE_FAST_EDGE_COUNT
#define PHASE_CAPTURE_IC_PRESCALER      TIM_ICPSC_DIV8
/* DIV1 is changed back to DMA/DIV8 at approximately 20 kHz or above. */
#define PHASE_CAPTURE_FAST_SWITCH_TICKS 4200U
#define PHASE_CAPTURE_TIMER_FLAG        TIM_FLAG_CC3
#define PHASE_CAPTURE_TIMER_IT          TIM_IT_CC3
#define PHASE_CAPTURE_OVERCAPTURE_FLAG  TIM_FLAG_CC3OF
#define PHASE_CAPTURE_DMA_STREAM        DMA1_Stream1
#define PHASE_CAPTURE_DMA_CHANNEL       DMA_CHANNEL_3
#define PHASE_CAPTURE_DMA_IRQn          DMA1_Stream1_IRQn
#define PHASE_CAPTURE_DMA_BUFFER_COUNT  512U

typedef struct
{
    uint16_t prescaler;
    uint32_t period;
    uint32_t timer_tick_hz;
} PhaseCaptureConfig_t;

typedef struct
{
    TIM_HandleTypeDef timer;
    DMA_HandleTypeDef dma;
    PhaseCaptureConfig_t config;
    volatile uint32_t last_capture;
    volatile uint32_t period_ticks;
    volatile uint32_t period_capture_cycles;
    volatile uint32_t active_capture_cycles;
    volatile uint32_t dma_wrap_count;
    volatile uint32_t dma_error_count;
    uint32_t dma_last_sample_count;
    uint32_t dma_last_timestamp;
    volatile uint8_t capture_ready;
    volatile uint8_t has_last_capture;
    volatile uint8_t dma_restart_requested;
    volatile uint8_t dma_running;
    volatile uint8_t irq_running;
    uint8_t dma_has_last_sample;
    uint8_t initialized;
} PhaseCapture_t;

extern PhaseCapture_t g_phase_capture;

void PhaseCaptureConfig_Default(PhaseCaptureConfig_t *config);
HAL_StatusTypeDef PhaseCapture_Init(PhaseCapture_t *capture,
                                    const PhaseCaptureConfig_t *config);
HAL_StatusTypeDef PhaseCapture_Start(PhaseCapture_t *capture);
HAL_StatusTypeDef PhaseCapture_Stop(PhaseCapture_t *capture);
uint8_t PhaseCapture_TakePeriod(PhaseCapture_t *capture,
                                uint32_t *period_ticks);
uint8_t PhaseCapture_TakeMeasurement(PhaseCapture_t *capture,
                                     uint32_t *period_ticks,
                                     uint32_t *capture_cycles);
HAL_StatusTypeDef PhaseCapture_SetEdgeCount(PhaseCapture_t *capture,
                                            uint32_t capture_cycles);
uint32_t PhaseCapture_GetEdgeCount(const PhaseCapture_t *capture);
uint32_t PhaseCapture_GetDmaErrorCount(const PhaseCapture_t *capture);
void PhaseCapture_DmaIrqHandler(void);

void phase_capture_init(void);
HAL_StatusTypeDef phase_capture_start(void);
uint8_t phase_capture_take_period(uint32_t *period_ticks);

#endif /* PHASE_CAPTURE_H */
