/**
 * @file    iic.h
 * @brief   STM32F407 两路软件 I2C 主机驱动。
 *
 * 引脚按用户指定配置为开漏 GPIO：
 *   I2C1: PF10 = SCL，PC13 = SDA
 *   I2C2: PF14 = SCL，PF12 = SDA
 *
 * 两根总线都需要外部上拉电阻，建议 2.2 kOhm~4.7 kOhm。设备地址参数
 * 使用未左移的 7 位地址，例如 0x68，而不是 HAL 常用的 0xD0。
 */

#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "stm32f4xx_hal.h"

#define IIC1_SCL_GPIO_PORT       GPIOF
#define IIC1_SCL_GPIO_PIN        GPIO_PIN_10
#define IIC1_SDA_GPIO_PORT       GPIOC
#define IIC1_SDA_GPIO_PIN        GPIO_PIN_13

#define IIC2_SCL_GPIO_PORT       GPIOF
#define IIC2_SCL_GPIO_PIN        GPIO_PIN_14
#define IIC2_SDA_GPIO_PORT       GPIOF
#define IIC2_SDA_GPIO_PIN        GPIO_PIN_12

/* 约 100 kHz；实际频率还会受到 HAL_GPIO 调用开销影响。 */
#define IIC_SOFTWARE_DELAY_US    5U
#define IIC_SCL_TIMEOUT_US       1000U

/** 初始化指定软件 I2C 总线。 */
HAL_StatusTypeDef IIC1_Init(void);
HAL_StatusTypeDef IIC2_Init(void);

/** 以 7 位设备地址发送数据。 */
HAL_StatusTypeDef IIC1_MasterTransmit(uint8_t dev_addr,
                                      uint8_t *data,
                                      uint16_t size);
HAL_StatusTypeDef IIC2_MasterTransmit(uint8_t dev_addr,
                                      uint8_t *data,
                                      uint16_t size);

/** 以 7 位设备地址接收数据。 */
HAL_StatusTypeDef IIC1_MasterReceive(uint8_t dev_addr,
                                     uint8_t *data,
                                     uint16_t size);
HAL_StatusTypeDef IIC2_MasterReceive(uint8_t dev_addr,
                                     uint8_t *data,
                                     uint16_t size);

/** 8 位寄存器地址设备的写寄存器/读寄存器接口。 */
HAL_StatusTypeDef IIC1_MemWrite8(uint8_t dev_addr,
                                 uint8_t reg_addr,
                                 uint8_t *data,
                                 uint16_t size);
HAL_StatusTypeDef IIC2_MemWrite8(uint8_t dev_addr,
                                 uint8_t reg_addr,
                                 uint8_t *data,
                                 uint16_t size);
HAL_StatusTypeDef IIC1_MemRead8(uint8_t dev_addr,
                                uint8_t reg_addr,
                                uint8_t *data,
                                uint16_t size);
HAL_StatusTypeDef IIC2_MemRead8(uint8_t dev_addr,
                                uint8_t reg_addr,
                                uint8_t *data,
                                uint16_t size);

/** 探测设备是否响应，dev_addr 使用 7 位地址。 */
HAL_StatusTypeDef IIC1_IsDeviceReady(uint8_t dev_addr);
HAL_StatusTypeDef IIC2_IsDeviceReady(uint8_t dev_addr);

#endif /* __BSP_IIC_H */
