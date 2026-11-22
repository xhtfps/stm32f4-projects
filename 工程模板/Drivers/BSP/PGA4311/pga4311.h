#ifndef PGA4311_H
#define PGA4311_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define PGA4311_CHANNEL_COUNT       4U
#define PGA4311_GAIN_MIN_DB         (-95.5f)
#define PGA4311_GAIN_MAX_DB         31.5f
#define PGA4311_GAIN_STEP_DB        0.5f
#define PGA4311_DEFAULT_GAIN_DB     0.0f

/*
 * Keep the source-project wiring: PE0 = CS, PE2 = SCLK, PE4 = SDI.
 * The actual board has no external SRAM and this firmware never calls
 * sram_init(), so PE0 is not configured as FSMC_NBL0.
 */
#define PGA4311_GPIO_PORT           GPIOE
#define PGA4311_CS_PIN              GPIO_PIN_0
#define PGA4311_SCLK_PIN            GPIO_PIN_2
#define PGA4311_SDI_PIN             GPIO_PIN_4

HAL_StatusTypeDef PGA4311_InitDevice(void);
HAL_StatusTypeDef PGA4311_SetGainDb(const float gain_db[PGA4311_CHANNEL_COUNT]);
/* channel uses the PGA4311 channel number 1..4. */
HAL_StatusTypeDef PGA4311_SetChannelGainDb(uint8_t channel, float gain_db);
HAL_StatusTypeDef PGA4311_MuteAll(void);
uint8_t PGA4311_GainDbToCode(float gain_db);
float PGA4311_GetChannelGainDb(uint8_t channel);

/* Reference-project compatible API. */
void PGA4311_Init(void);
void PGA4311_SetAV(float A1, float A2, float A3, float A4);

#endif /* PGA4311_H */
