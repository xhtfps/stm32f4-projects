#ifndef PHASE_ADC_H
#define PHASE_ADC_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

/* Hardware defaults from the PDF prototype. Change only these macros after PCB revision. */
#define PHASE_ADC_TIMER                 TIM8
#define PHASE_ADC_TIMER_CLOCK_HZ        168000000U
#define PHASE_ADC_TIMER_IRQn            TIM8_UP_TIM13_IRQn
#define PHASE_ADC_CHANNEL_1             ADC_CHANNEL_2 /* PA2, ADC1 */
#define PHASE_ADC_CHANNEL_2             ADC_CHANNEL_3 /* PA3, ADC2 */
#define PHASE_ADC_GPIO_PORT             GPIOA
#define PHASE_ADC_GPIO_PIN_1            GPIO_PIN_2
#define PHASE_ADC_GPIO_PIN_2            GPIO_PIN_3
#define PHASE_ADC_SAMPLE_COUNT          256U
#define PHASE_ADC_BLOCK_COUNT           (PHASE_ADC_SAMPLE_COUNT / 2U)

extern ADC_HandleTypeDef hadc_phase_master;
extern ADC_HandleTypeDef hadc_phase_slave;
extern DMA_HandleTypeDef hdma_phase_adc;

HAL_StatusTypeDef PhaseAdc_Init(void);
HAL_StatusTypeDef PhaseAdc_Start(uint32_t sample_rate_hz);
HAL_StatusTypeDef PhaseAdc_Stop(void);

/* Copies one ready half-buffer and removes the DC bias from both channels. */
HAL_StatusTypeDef PhaseAdc_TakeBlock(int16_t *channel1,
                                     int16_t *channel2,
                                     uint16_t length);
uint32_t PhaseAdc_GetOverrunCount(void);

#endif /* PHASE_ADC_H */
