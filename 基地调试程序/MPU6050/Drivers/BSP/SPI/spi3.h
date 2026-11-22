/**
 * @file    spi3.h
 * @brief   STM32F407 SPI3 硬件 SPI 驱动。
 *
 * 引脚定义（参考 STM32F407 数据手册复用功能表）：
 *   PC10 -> SPI3_SCK  (AF6)
 *   PC11 -> SPI3_MISO (AF6)
 *   PC12 -> SPI3_MOSI (AF6)
 *   PB6  -> 软件片选 CS，低有效（不是 SPI3 的硬件 NSS 复用脚）
 */

#ifndef __BSP_SPI3_H
#define __BSP_SPI3_H

#include "stm32f4xx_hal.h"

#define SPI3_CS_GPIO_PORT       GPIOB
#define SPI3_CS_GPIO_PIN        GPIO_PIN_6

#define SPI3_SCK_GPIO_PORT      GPIOC
#define SPI3_SCK_GPIO_PIN       GPIO_PIN_10
#define SPI3_MISO_GPIO_PORT     GPIOC
#define SPI3_MISO_GPIO_PIN      GPIO_PIN_11
#define SPI3_MOSI_GPIO_PORT     GPIOC
#define SPI3_MOSI_GPIO_PIN      GPIO_PIN_12

/* SPI3 的 APB1 时钟为 42 MHz 时，默认分频 16 得到约 2.625 MHz。 */
#define SPI3_DEFAULT_PRESCALER  SPI_BAUDRATEPRESCALER_16
#define SPI3_TIMEOUT_MS         100U

extern SPI_HandleTypeDef g_spi3_handler;

/** 初始化 SPI3、PC10/11/12 以及 PB6 软件片选。 */
HAL_StatusTypeDef SPI3_Init(void);

/** 修改 SPI3 波特率分频，参数使用 SPI_BAUDRATEPRESCALER_x 宏。 */
HAL_StatusTypeDef SPI3_SetSpeed(uint32_t prescaler);

/** Configure the SPI clock polarity and sampling phase after initialization. */
HAL_StatusTypeDef SPI3_SetMode(uint32_t polarity, uint32_t phase);

/** 发送并接收一段全双工 SPI 数据。 */
HAL_StatusTypeDef SPI3_TransmitReceive(uint8_t *tx_data,
                                       uint8_t *rx_data,
                                       uint16_t size,
                                       uint32_t timeout);

/** 仅发送一段数据。 */
HAL_StatusTypeDef SPI3_Transmit(uint8_t *data,
                                uint16_t size,
                                uint32_t timeout);

/** 仅接收一段数据。 */
HAL_StatusTypeDef SPI3_Receive(uint8_t *data,
                               uint16_t size,
                               uint32_t timeout);

/** 发送一个字节并返回同时收到的字节；失败时返回 0xFF。 */
uint8_t SPI3_ReadWriteByte(uint8_t tx_data);

/** PB6 片选控制，低有效。 */
void SPI3_CS_Low(void);
void SPI3_CS_High(void);

#endif /* __BSP_SPI3_H */
