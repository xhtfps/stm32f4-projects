#include "ad8367.h"

#include "../DAC/DAC.h"

#define AD8367_DAC_FULL_SCALE_CODE       4095U
#define AD8367_DAC_REFERENCE_MV          3300U
#define AD8367_GAIN_SLOPE_MV_PER_DB        20.0f
#define AD8367_GAIN_UP_INTERCEPT_DB        (-5.0f)
#define AD8367_GAIN_DOWN_INTERCEPT_DB       45.0f

static uint8_t s_initialized;
static AD8367_GainMode_t s_gain_mode =
    AD8367_GAIN_INCREASES_WITH_VOLTAGE;
static float s_gain_db = AD8367_DEFAULT_GAIN_DB;
static uint16_t s_control_voltage_mv = 100U;
static uint16_t s_dac_code;

static float AD8367_ClampGain(float gain_db)
{
    if (gain_db < AD8367_GAIN_MIN_DB)
    {
        return AD8367_GAIN_MIN_DB;
    }
    if (gain_db > AD8367_GAIN_MAX_DB)
    {
        return AD8367_GAIN_MAX_DB;
    }
    return gain_db;
}

static uint16_t AD8367_ClampVoltage(uint16_t voltage_mv)
{
    if (voltage_mv < AD8367_VGAIN_MIN_MV)
    {
        return AD8367_VGAIN_MIN_MV;
    }
    if (voltage_mv > AD8367_VGAIN_MAX_MV)
    {
        return AD8367_VGAIN_MAX_MV;
    }
    return voltage_mv;
}

static uint16_t AD8367_VoltageToDacCode(uint16_t voltage_mv)
{
    return (uint16_t)((((uint32_t)voltage_mv *
                        AD8367_DAC_FULL_SCALE_CODE) +
                       (AD8367_DAC_REFERENCE_MV / 2U)) /
                      AD8367_DAC_REFERENCE_MV);
}

static float AD8367_VoltageToGain(uint16_t voltage_mv)
{
    float voltage_term_db =
        (float)voltage_mv / AD8367_GAIN_SLOPE_MV_PER_DB;

    if (s_gain_mode == AD8367_GAIN_INCREASES_WITH_VOLTAGE)
    {
        return voltage_term_db + AD8367_GAIN_UP_INTERCEPT_DB;
    }
    return AD8367_GAIN_DOWN_INTERCEPT_DB - voltage_term_db;
}

HAL_StatusTypeDef AD8367_Init(float gain_db, AD8367_GainMode_t mode)
{
    if ((mode != AD8367_GAIN_INCREASES_WITH_VOLTAGE) &&
        (mode != AD8367_GAIN_DECREASES_WITH_VOLTAGE))
    {
        return HAL_ERROR;
    }

    s_gain_mode = mode;
    s_initialized = 1U;
    if (AD8367_SetGainDb(gain_db) != HAL_OK)
    {
        s_initialized = 0U;
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef AD8367_SetGainDb(float gain_db)
{
    float voltage_mv;

    gain_db = AD8367_ClampGain(gain_db);
    if (s_gain_mode == AD8367_GAIN_INCREASES_WITH_VOLTAGE)
    {
        voltage_mv = (gain_db - AD8367_GAIN_UP_INTERCEPT_DB) *
                     AD8367_GAIN_SLOPE_MV_PER_DB;
    }
    else
    {
        voltage_mv = (AD8367_GAIN_DOWN_INTERCEPT_DB - gain_db) *
                     AD8367_GAIN_SLOPE_MV_PER_DB;
    }

    return AD8367_SetControlVoltageMv(
        (uint16_t)(voltage_mv + 0.5f));
}

HAL_StatusTypeDef AD8367_SetControlVoltageMv(uint16_t voltage_mv)
{
    HAL_StatusTypeDef status;

    voltage_mv = AD8367_ClampVoltage(voltage_mv);
    s_dac_code = AD8367_VoltageToDacCode(voltage_mv);
    status = BSP_DAC_SetValue(s_dac_code);
    if (status != HAL_OK)
    {
        return status;
    }

    s_control_voltage_mv = voltage_mv;
    s_gain_db = AD8367_ClampGain(
        AD8367_VoltageToGain(voltage_mv));
    return HAL_OK;
}

float AD8367_GetGainDb(void)
{
    return s_gain_db;
}

uint16_t AD8367_GetControlVoltageMv(void)
{
    return s_control_voltage_mv;
}

uint16_t AD8367_GetDacCode(void)
{
    return s_dac_code;
}

AD8367_GainMode_t AD8367_GetGainMode(void)
{
    return s_gain_mode;
}

uint8_t AD8367_IsInitialized(void)
{
    return s_initialized;
}

uint8_t AD8367_VerifyDacRegister(void)
{
    if ((s_initialized == 0U) || (hdac.Instance != DAC))
    {
        return 0U;
    }
    return (HAL_DAC_GetValue(&hdac, DAC_CHANNEL_1) == s_dac_code) ?
           1U : 0U;
}
