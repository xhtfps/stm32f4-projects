#ifndef BSP_AD8367_H
#define BSP_AD8367_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

/*
 * AD8367 VGAIN is linear in dB over approximately 50 mV to 950 mV.
 * MODE high: gain increases with VGAIN.
 * MODE low : gain decreases with VGAIN.
 */
typedef enum
{
    AD8367_GAIN_DECREASES_WITH_VOLTAGE = 0U,
    AD8367_GAIN_INCREASES_WITH_VOLTAGE = 1U
} AD8367_GainMode_t;

#define AD8367_GAIN_MIN_DB             (-2.5f)
#define AD8367_GAIN_MAX_DB              42.5f
#define AD8367_VGAIN_MIN_MV               50U
#define AD8367_VGAIN_MAX_MV              950U
#define AD8367_DEFAULT_GAIN_DB            0.0f

HAL_StatusTypeDef AD8367_Init(float gain_db, AD8367_GainMode_t mode);
HAL_StatusTypeDef AD8367_SetGainDb(float gain_db);
HAL_StatusTypeDef AD8367_SetControlVoltageMv(uint16_t voltage_mv);

float AD8367_GetGainDb(void);
uint16_t AD8367_GetControlVoltageMv(void);
uint16_t AD8367_GetDacCode(void);
AD8367_GainMode_t AD8367_GetGainMode(void);
uint8_t AD8367_IsInitialized(void);
uint8_t AD8367_VerifyDacRegister(void);

#endif /* BSP_AD8367_H */
