#include "pga4311.h"

#include "./SYSTEM/delay/delay.h"

typedef struct
{
    float gain_db[PGA4311_CHANNEL_COUNT];
    uint8_t initialized;
} PGA4311_State_t;

static PGA4311_State_t s_pga4311;

static void pga4311_write_word(uint32_t data)
{
    uint8_t bit_index;

    HAL_GPIO_WritePin(PGA4311_GPIO_PORT, PGA4311_SCLK_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PGA4311_GPIO_PORT, PGA4311_CS_PIN, GPIO_PIN_RESET);
    delay_us(1U);

    for (bit_index = 0U; bit_index < 32U; bit_index++)
    {
        HAL_GPIO_WritePin(
            PGA4311_GPIO_PORT,
            PGA4311_SDI_PIN,
            ((data & 0x80000000UL) != 0UL) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        delay_us(1U);
        HAL_GPIO_WritePin(PGA4311_GPIO_PORT, PGA4311_SCLK_PIN, GPIO_PIN_SET);
        delay_us(1U);
        HAL_GPIO_WritePin(PGA4311_GPIO_PORT, PGA4311_SCLK_PIN, GPIO_PIN_RESET);
        data <<= 1U;
    }

    delay_us(1U);
    HAL_GPIO_WritePin(PGA4311_GPIO_PORT, PGA4311_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA4311_GPIO_PORT, PGA4311_SDI_PIN, GPIO_PIN_RESET);
}

uint8_t PGA4311_GainDbToCode(float gain_db)
{
    float code;

    if (gain_db <= PGA4311_GAIN_MIN_DB)
    {
        return 1U;
    }
    if (gain_db >= PGA4311_GAIN_MAX_DB)
    {
        return 255U;
    }

    code = 192.0f + (2.0f * gain_db);
    return (uint8_t)(code + 0.5f);
}

HAL_StatusTypeDef PGA4311_SetGainDb(
    const float gain_db[PGA4311_CHANNEL_COUNT])
{
    uint8_t code[PGA4311_CHANNEL_COUNT];
    uint32_t data;
    uint8_t channel;
    float clamped_gain;

    if ((gain_db == NULL) || (s_pga4311.initialized == 0U))
    {
        return HAL_ERROR;
    }

    for (channel = 0U; channel < PGA4311_CHANNEL_COUNT; channel++)
    {
        clamped_gain = gain_db[channel];
        if (clamped_gain < PGA4311_GAIN_MIN_DB)
        {
            clamped_gain = PGA4311_GAIN_MIN_DB;
        }
        else if (clamped_gain > PGA4311_GAIN_MAX_DB)
        {
            clamped_gain = PGA4311_GAIN_MAX_DB;
        }

        code[channel] = PGA4311_GainDbToCode(clamped_gain);
        s_pga4311.gain_db[channel] =
            ((float)code[channel] - 192.0f) * 0.5f;
    }

    /* PGA4311 serial order is channel 4, 3, 2, then 1. */
    data = ((uint32_t)code[3] << 24U) |
           ((uint32_t)code[2] << 16U) |
           ((uint32_t)code[1] << 8U) |
           (uint32_t)code[0];
    pga4311_write_word(data);
    return HAL_OK;
}

HAL_StatusTypeDef PGA4311_SetChannelGainDb(uint8_t channel, float gain_db)
{
    float gains[PGA4311_CHANNEL_COUNT];
    uint8_t index;

    if ((channel == 0U) || (channel > PGA4311_CHANNEL_COUNT) ||
        (s_pga4311.initialized == 0U))
    {
        return HAL_ERROR;
    }

    for (index = 0U; index < PGA4311_CHANNEL_COUNT; index++)
    {
        gains[index] = s_pga4311.gain_db[index];
    }
    gains[channel - 1U] = gain_db;
    return PGA4311_SetGainDb(gains);
}

HAL_StatusTypeDef PGA4311_MuteAll(void)
{
    uint8_t channel;

    if (s_pga4311.initialized == 0U)
    {
        return HAL_ERROR;
    }

    pga4311_write_word(0UL);
    for (channel = 0U; channel < PGA4311_CHANNEL_COUNT; channel++)
    {
        s_pga4311.gain_db[channel] = PGA4311_GAIN_MIN_DB;
    }
    return HAL_OK;
}

float PGA4311_GetChannelGainDb(uint8_t channel)
{
    if ((channel == 0U) || (channel > PGA4311_CHANNEL_COUNT))
    {
        return PGA4311_GAIN_MIN_DB;
    }
    return s_pga4311.gain_db[channel - 1U];
}

HAL_StatusTypeDef PGA4311_InitDevice(void)
{
    GPIO_InitTypeDef gpio;
    float default_gain[PGA4311_CHANNEL_COUNT];
    uint8_t channel;

    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Establish idle levels before switching the pins to output mode. */
    HAL_GPIO_WritePin(PGA4311_GPIO_PORT, PGA4311_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PGA4311_GPIO_PORT,
                      PGA4311_SCLK_PIN | PGA4311_SDI_PIN,
                      GPIO_PIN_RESET);

    gpio.Pin = PGA4311_CS_PIN | PGA4311_SCLK_PIN | PGA4311_SDI_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = 0U;
    HAL_GPIO_Init(PGA4311_GPIO_PORT, &gpio);

    s_pga4311.initialized = 1U;
    for (channel = 0U; channel < PGA4311_CHANNEL_COUNT; channel++)
    {
        default_gain[channel] = PGA4311_DEFAULT_GAIN_DB;
    }
    return PGA4311_SetGainDb(default_gain);
}

void PGA4311_Init(void)
{
    (void)PGA4311_InitDevice();
}

void PGA4311_SetAV(float A1, float A2, float A3, float A4)
{
    float gain_db[PGA4311_CHANNEL_COUNT];

    gain_db[0] = A1;
    gain_db[1] = A2;
    gain_db[2] = A3;
    gain_db[3] = A4;
    (void)PGA4311_SetGainDb(gain_db);
}
