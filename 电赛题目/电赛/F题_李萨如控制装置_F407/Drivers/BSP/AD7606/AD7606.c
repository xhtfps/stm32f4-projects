/**
 * @file    AD7606.c
 * @brief   AD7606 eight-channel serial-interface driver.
 *
 * The implementation follows the supplied AD7606 manual and serial example:
 * one conversion is started with CONVST A/B, BUSY is waited high then low,
 * and DOUTA is shifted out as eight 16-bit two's-complement samples.
 */

#include "AD7606.h"

#include <stddef.h>

#include "../SPI/spi3.h"
#include "./SYSTEM/delay/delay.h"

static uint8_t s_initialized;
static uint8_t s_transaction_active;

static HAL_StatusTypeDef AD7606_WaitBusyLevel(GPIO_PinState level,
                                               uint32_t timeout_us)
{
    uint32_t start_tick;
    uint32_t timeout_ms;

    start_tick = HAL_GetTick();
    timeout_ms = (timeout_us + 999U) / 1000U;
    if (timeout_ms == 0U)
    {
        timeout_ms = 1U;
    }

    while (HAL_GPIO_ReadPin(AD7606_BUSY_GPIO_PORT,
                            AD7606_BUSY_GPIO_PIN) != level)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        delay_us(1U);
    }
    return HAL_OK;
}

static void AD7606_SetConversionPins(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AD7606_CONVST_A_GPIO_PORT,
                      AD7606_CONVST_A_GPIO_PIN, state);
    HAL_GPIO_WritePin(AD7606_CONVST_B_GPIO_PORT,
                      AD7606_CONVST_B_GPIO_PIN, state);
}

static void AD7606_SetOversamplingPins(AD7606_Oversampling_t mode)
{
    HAL_GPIO_WritePin(AD7606_OS2_GPIO_PORT, AD7606_OS2_GPIO_PIN,
                      ((mode & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD7606_OS1_GPIO_PORT, AD7606_OS1_GPIO_PIN,
                      ((mode & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AD7606_OS0_GPIO_PORT, AD7606_OS0_GPIO_PIN,
                      ((mode & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void AD7606_ConfigureOutput(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &gpio);
}

static void AD7606_ConfigureInput(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &gpio);
}

HAL_StatusTypeDef AD7606_Init(void)
{
    HAL_StatusTypeDef status;

    if (s_initialized != 0U)
    {
        return HAL_OK;
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    status = SPI3_Init();
    if (status != HAL_OK)
    {
        return status;
    }
    status = SPI3_SetSpeed(AD7606_SPI3_PRESCALER);
    if (status != HAL_OK)
    {
        return status;
    }
    /* AD7606 timing is SPI mode 2: idle high, data sampled on falling edge. */
    status = SPI3_SetMode(SPI_POLARITY_HIGH, SPI_PHASE_1EDGE);
    if (status != HAL_OK)
    {
        return status;
    }

    AD7606_ConfigureOutput(AD7606_RST_GPIO_PORT, AD7606_RST_GPIO_PIN);
    AD7606_ConfigureOutput(AD7606_CONVST_A_GPIO_PORT,
                           AD7606_CONVST_A_GPIO_PIN);
    AD7606_ConfigureOutput(AD7606_CONVST_B_GPIO_PORT,
                           AD7606_CONVST_B_GPIO_PIN);
#if (AD7606_STBY_HARDWIRED_HIGH == 0U)
    AD7606_ConfigureOutput(AD7606_STBY_GPIO_PORT, AD7606_STBY_GPIO_PIN);
#endif
    AD7606_ConfigureOutput(AD7606_OS2_GPIO_PORT, AD7606_OS2_GPIO_PIN);
    AD7606_ConfigureOutput(AD7606_OS1_GPIO_PORT, AD7606_OS1_GPIO_PIN);
    AD7606_ConfigureOutput(AD7606_OS0_GPIO_PORT, AD7606_OS0_GPIO_PIN);
    AD7606_ConfigureInput(AD7606_FRSTDATA_GPIO_PORT,
                          AD7606_FRSTDATA_GPIO_PIN);
    AD7606_ConfigureInput(AD7606_BUSY_GPIO_PORT, AD7606_BUSY_GPIO_PIN);

    AD7606_SetConversionPins(GPIO_PIN_SET);
#if (AD7606_STBY_HARDWIRED_HIGH == 0U)
    HAL_GPIO_WritePin(AD7606_STBY_GPIO_PORT, AD7606_STBY_GPIO_PIN,
                      GPIO_PIN_SET);
#endif
    SPI3_CS_High();
    HAL_GPIO_WritePin(AD7606_RST_GPIO_PORT, AD7606_RST_GPIO_PIN,
                      GPIO_PIN_RESET);
    AD7606_SetOversamplingPins(AD7606_OVERSAMPLE_1);

    s_initialized = 1U;
    s_transaction_active = 0U;
    return AD7606_Reset();
}

HAL_StatusTypeDef AD7606_Reset(void)
{
    if (s_initialized == 0U)
    {
        return HAL_ERROR;
    }

    HAL_GPIO_WritePin(AD7606_RST_GPIO_PORT, AD7606_RST_GPIO_PIN,
                      GPIO_PIN_SET);
    delay_us(1U); /* datasheet tRESET is at least 50 ns */
    HAL_GPIO_WritePin(AD7606_RST_GPIO_PORT, AD7606_RST_GPIO_PIN,
                      GPIO_PIN_RESET);
    delay_us(1U); /* datasheet t7 is at least 25 ns */
    return HAL_OK;
}

HAL_StatusTypeDef AD7606_SetOversampling(AD7606_Oversampling_t mode)
{
    if ((s_initialized == 0U) || (mode > AD7606_OVERSAMPLE_64))
    {
        return HAL_ERROR;
    }

    AD7606_SetOversamplingPins(mode);
    return HAL_OK;
}

static void AD7606_StartConversionInternal(void)
{
    /* CONVST A and B are tied together in the supplied serial wiring. */
    AD7606_SetConversionPins(GPIO_PIN_RESET);
    delay_us(1U); /* t2 >= 25 ns */
    AD7606_SetConversionPins(GPIO_PIN_SET);
}

HAL_StatusTypeDef AD7606_StartConversion(void)
{
    if (s_initialized == 0U)
    {
        return HAL_ERROR;
    }
    if (s_transaction_active != 0U)
    {
        return HAL_BUSY;
    }

    AD7606_StartConversionInternal();
    return HAL_OK;
}

HAL_StatusTypeDef AD7606_WaitReady(uint32_t timeout_us)
{
    HAL_StatusTypeDef status;

    if (s_initialized == 0U)
    {
        return HAL_ERROR;
    }

    /* Waiting for the high edge first prevents a stale low BUSY from being
       mistaken for the result of the conversion just started. */
    status = AD7606_WaitBusyLevel(GPIO_PIN_SET, timeout_us);
    if (status != HAL_OK)
    {
        return status;
    }
    return AD7606_WaitBusyLevel(GPIO_PIN_RESET, timeout_us);
}

HAL_StatusTypeDef AD7606_ReadSamples(int16_t samples[AD7606_CHANNEL_COUNT])
{
    uint8_t channel;
    uint8_t tx_data[AD7606_CHANNEL_COUNT * 2U] = {0U};
    uint8_t rx_data[AD7606_CHANNEL_COUNT * 2U] = {0U};
    HAL_StatusTypeDef status;

    if ((s_initialized == 0U) || (samples == NULL))
    {
        return HAL_ERROR;
    }

    SPI3_CS_Low();
    delay_us(1U);
    status = SPI3_TransmitReceive(tx_data, rx_data,
                                  (uint16_t)sizeof(tx_data),
                                  SPI3_TIMEOUT_MS);
    SPI3_CS_High();
    if (status != HAL_OK)
    {
        return status;
    }

    for (channel = 0U; channel < AD7606_CHANNEL_COUNT; ++channel)
    {
        samples[channel] = (int16_t)(((uint16_t)rx_data[channel * 2U] << 8U) |
                                     rx_data[channel * 2U + 1U]);
    }
    return HAL_OK;
}

HAL_StatusTypeDef AD7606_ReadFrame(int16_t samples[AD7606_CHANNEL_COUNT])
{
    HAL_StatusTypeDef status;

    if ((s_initialized == 0U) || (samples == NULL))
    {
        return HAL_ERROR;
    }
    if (s_transaction_active != 0U)
    {
        return HAL_BUSY;
    }

    s_transaction_active = 1U;
    AD7606_StartConversionInternal();
    status = AD7606_WaitReady(AD7606_BUSY_TIMEOUT_US);
    if (status == HAL_OK)
    {
        status = AD7606_ReadSamples(samples);
    }
    s_transaction_active = 0U;
    return status;
}

HAL_StatusTypeDef AD7606_ReadChannel(uint8_t channel, int16_t *sample)
{
    int16_t samples[AD7606_CHANNEL_COUNT];
    HAL_StatusTypeDef status;

    if ((channel >= AD7606_CHANNEL_COUNT) || (sample == NULL))
    {
        return HAL_ERROR;
    }

    status = AD7606_ReadFrame(samples);
    if (status == HAL_OK)
    {
        *sample = samples[channel];
    }
    return status;
}

float AD7606_CodeToVoltage(int16_t code, float input_range_volts)
{
    if (input_range_volts <= 0.0f)
    {
        return 0.0f;
    }
    return ((float)code * input_range_volts) / 32768.0f;
}

void GPIO_AD7606_Configuration(void)
{
    (void)AD7606_Init();
}

void AD7606_startconvst(void)
{
    (void)AD7606_StartConversion();
}

void AD7606_reset(void)
{
    (void)AD7606_Reset();
}

void AD7606_read_data(int16_t *DB_data)
{
    (void)AD7606_ReadFrame(DB_data);
}
