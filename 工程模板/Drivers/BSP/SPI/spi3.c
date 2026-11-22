/**
 * @file    spi3.c
 * @brief   STM32F407 SPI3 硬件 SPI 驱动实现。
 */

#include "spi3.h"

SPI_HandleTypeDef g_spi3_handler;

static uint8_t s_spi3_initialized;

static void SPI3_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio_init;

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC10/PC11/PC12: SPI3_SCK/MISO/MOSI，AF6。 */
    gpio_init.Pin = SPI3_SCK_GPIO_PIN |
                    SPI3_MISO_GPIO_PIN |
                    SPI3_MOSI_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    /* PB6 作为软件片选，SPI 空闲时保持释放状态。 */
    gpio_init.Pin = SPI3_CS_GPIO_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = 0U;
    HAL_GPIO_Init(SPI3_CS_GPIO_PORT, &gpio_init);
    SPI3_CS_High();
}

static uint8_t SPI3_PrescalerIsValid(uint32_t prescaler)
{
    return (prescaler == SPI_BAUDRATEPRESCALER_2)   ||
           (prescaler == SPI_BAUDRATEPRESCALER_4)   ||
           (prescaler == SPI_BAUDRATEPRESCALER_8)   ||
           (prescaler == SPI_BAUDRATEPRESCALER_16)  ||
           (prescaler == SPI_BAUDRATEPRESCALER_32)  ||
           (prescaler == SPI_BAUDRATEPRESCALER_64)  ||
           (prescaler == SPI_BAUDRATEPRESCALER_128) ||
           (prescaler == SPI_BAUDRATEPRESCALER_256);
}

HAL_StatusTypeDef SPI3_Init(void)
{
    if (s_spi3_initialized != 0U)
    {
        return HAL_OK;
    }

    SPI3_GPIO_Init();
    __HAL_RCC_SPI3_CLK_ENABLE();

    g_spi3_handler.Instance = SPI3;
    g_spi3_handler.Init.Mode = SPI_MODE_MASTER;
    g_spi3_handler.Init.Direction = SPI_DIRECTION_2LINES;
    g_spi3_handler.Init.DataSize = SPI_DATASIZE_8BIT;
    g_spi3_handler.Init.CLKPolarity = SPI_POLARITY_LOW;
    g_spi3_handler.Init.CLKPhase = SPI_PHASE_1EDGE;
    g_spi3_handler.Init.NSS = SPI_NSS_SOFT;
    g_spi3_handler.Init.BaudRatePrescaler = SPI3_DEFAULT_PRESCALER;
    g_spi3_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;
    g_spi3_handler.Init.TIMode = SPI_TIMODE_DISABLE;
    g_spi3_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    g_spi3_handler.Init.CRCPolynomial = 7U;

    if (HAL_SPI_Init(&g_spi3_handler) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_spi3_initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef SPI3_SetSpeed(uint32_t prescaler)
{
    if ((s_spi3_initialized == 0U) || (SPI3_PrescalerIsValid(prescaler) == 0U))
    {
        return HAL_ERROR;
    }

    __HAL_SPI_DISABLE(&g_spi3_handler);
    MODIFY_REG(g_spi3_handler.Instance->CR1, SPI_CR1_BR, prescaler);
    __HAL_SPI_ENABLE(&g_spi3_handler);

    return HAL_OK;
}

HAL_StatusTypeDef SPI3_SetMode(uint32_t polarity, uint32_t phase)
{
    if ((s_spi3_initialized == 0U) ||
        ((polarity != SPI_POLARITY_LOW) &&
         (polarity != SPI_POLARITY_HIGH)) ||
        ((phase != SPI_PHASE_1EDGE) && (phase != SPI_PHASE_2EDGE)))
    {
        return HAL_ERROR;
    }

    __HAL_SPI_DISABLE(&g_spi3_handler);
    MODIFY_REG(g_spi3_handler.Instance->CR1,
               SPI_CR1_CPOL | SPI_CR1_CPHA,
               polarity | phase);
    __HAL_SPI_ENABLE(&g_spi3_handler);
    g_spi3_handler.Init.CLKPolarity = (uint32_t)polarity;
    g_spi3_handler.Init.CLKPhase = (uint32_t)phase;

    return HAL_OK;
}

HAL_StatusTypeDef SPI3_TransmitReceive(uint8_t *tx_data,
                                       uint8_t *rx_data,
                                       uint16_t size,
                                       uint32_t timeout)
{
    if ((s_spi3_initialized == 0U) ||
        ((tx_data == NULL) && (size != 0U)) ||
        ((rx_data == NULL) && (size != 0U)))
    {
        return HAL_ERROR;
    }

    if (size == 0U)
    {
        return HAL_OK;
    }

    return HAL_SPI_TransmitReceive(&g_spi3_handler,
                                   tx_data,
                                   rx_data,
                                   size,
                                   timeout);
}

HAL_StatusTypeDef SPI3_Transmit(uint8_t *data,
                                uint16_t size,
                                uint32_t timeout)
{
    if ((s_spi3_initialized == 0U) ||
        ((data == NULL) && (size != 0U)))
    {
        return HAL_ERROR;
    }

    return HAL_SPI_Transmit(&g_spi3_handler, data, size, timeout);
}

HAL_StatusTypeDef SPI3_Receive(uint8_t *data,
                               uint16_t size,
                               uint32_t timeout)
{
    if ((s_spi3_initialized == 0U) ||
        ((data == NULL) && (size != 0U)))
    {
        return HAL_ERROR;
    }

    return HAL_SPI_Receive(&g_spi3_handler, data, size, timeout);
}

uint8_t SPI3_ReadWriteByte(uint8_t tx_data)
{
    uint8_t rx_data = 0xFFU;

    if (SPI3_TransmitReceive(&tx_data,
                             &rx_data,
                             1U,
                             SPI3_TIMEOUT_MS) != HAL_OK)
    {
        return 0xFFU;
    }

    return rx_data;
}

void SPI3_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI3_CS_GPIO_PORT, SPI3_CS_GPIO_PIN, GPIO_PIN_RESET);
}

void SPI3_CS_High(void)
{
    HAL_GPIO_WritePin(SPI3_CS_GPIO_PORT, SPI3_CS_GPIO_PIN, GPIO_PIN_SET);
}
