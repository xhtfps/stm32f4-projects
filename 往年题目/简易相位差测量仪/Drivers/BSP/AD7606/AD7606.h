#ifndef BSP_AD7606_H
#define BSP_AD7606_H

/*
 * AD7606 serial-interface driver for the STM32F407 project.
 *
 * The module must be strapped for serial mode:
 *   SER = 1, DB15/BYTE SEL = 0, and DOUTA (module DB7/DOUT) -> DOUT pin.
 * The default pin map uses this project's SPI3:
 *   PC10 = SCLK, PC11 = DOUTA, PC12 = unused MOSI, PB6 = CS.
 * Change only this section when the module is wired differently.
 */

/*
 * ======================== AD7606 接线表 ========================
 *
 * AD7606 模块                         STM32F407 工程
 * ---------------------------------------------------------------
 * +5V                                +5V
 * GND                                GND
 * SER                                +3.3V（串行模式）
 * DB15/BYTE SEL                      GND（SER=1、DB15=0）
 * STBY                               PE5
 * RESET                              PF1
 * CONVST B                           PF2
 * CONVST A                           PF3
 * OS2                                PF4
 * OS1                                PF5
 * OS0                                PE4
 * BUSY                               PE3
 * FRSTDATA                           PE2（可不接，驱动当前不依赖）
 * CS_N                               PB6（SPI3 软件片选）
 * RD/SCLK                            PC10（SPI3_SCK）
 * DB7/DOUTA                          PC11（SPI3_MISO）
 * PC12                               不接（SPI3_MOSI，AD7606 串行读不使用）
 * RANGE                              按模块量程设置：高=±10V，低=±5V
 *
 * 注意：PF1~PF5 同时是本工程外部 SRAM 的 FSMC 地址线。使用上述
 * AD7606 接线时，不要调用 sram_init()，否则会发生引脚复用冲突。
 * PE0/PE1 保留给 FSMC SRAM NBL0/NBL1，本驱动不使用。
 * ================================================================
 */

#include "stm32f4xx_hal.h"

#define AD7606_CHANNEL_COUNT              8U

/*
 * Control pin map for the available connector pins.
 * PF1..PF5 are also FSMC SRAM address pins in the board schematic. Do not
 * initialize external SRAM while this AD7606 mapping is in use.
 */
#define AD7606_RST_GPIO_PORT             GPIOF
#define AD7606_RST_GPIO_PIN              GPIO_PIN_1
#define AD7606_CONVST_B_GPIO_PORT        GPIOF
#define AD7606_CONVST_B_GPIO_PIN         GPIO_PIN_2
#define AD7606_CONVST_A_GPIO_PORT        GPIOF
#define AD7606_CONVST_A_GPIO_PIN         GPIO_PIN_3
#define AD7606_OS2_GPIO_PORT             GPIOF
#define AD7606_OS2_GPIO_PIN              GPIO_PIN_4
#define AD7606_OS1_GPIO_PORT             GPIOF
#define AD7606_OS1_GPIO_PIN              GPIO_PIN_5
#define AD7606_OS0_GPIO_PORT             GPIOE
#define AD7606_OS0_GPIO_PIN              GPIO_PIN_4
#define AD7606_STBY_HARDWIRED_HIGH       0U
#define AD7606_STBY_GPIO_PORT            GPIOE
#define AD7606_STBY_GPIO_PIN             GPIO_PIN_5

#define AD7606_FRSTDATA_GPIO_PORT        GPIOE
#define AD7606_FRSTDATA_GPIO_PIN         GPIO_PIN_2
#define AD7606_BUSY_GPIO_PORT            GPIOE
#define AD7606_BUSY_GPIO_PIN             GPIO_PIN_3
#define AD7606_CS_GPIO_PORT              GPIOB
#define AD7606_CS_GPIO_PIN               GPIO_PIN_6
#define AD7606_SCLK_GPIO_PORT            GPIOC
#define AD7606_SCLK_GPIO_PIN             GPIO_PIN_10
#define AD7606_DOUTA_GPIO_PORT           GPIOC
#define AD7606_DOUTA_GPIO_PIN            GPIO_PIN_11

/* The module connection diagram leaves these mode pins at fixed levels. */
#define AD7606_SERIAL_MODE_HARDWIRED     1U
#define AD7606_BYTE_SEL_HARDWIRED_LOW    1U

/* Default analog range is the module's +/-10 V setting. */
#ifndef AD7606_INPUT_RANGE_VOLTS
#define AD7606_INPUT_RANGE_VOLTS         10.0f
#endif

#ifndef AD7606_BUSY_TIMEOUT_US
#define AD7606_BUSY_TIMEOUT_US           1000U
#endif

#define AD7606_SPI3_PRESCALER            SPI_BAUDRATEPRESCALER_16

typedef enum
{
    AD7606_OVERSAMPLE_1  = 0U,
    AD7606_OVERSAMPLE_2  = 1U,
    AD7606_OVERSAMPLE_4  = 2U,
    AD7606_OVERSAMPLE_8  = 3U,
    AD7606_OVERSAMPLE_16 = 4U,
    AD7606_OVERSAMPLE_32 = 5U,
    AD7606_OVERSAMPLE_64 = 6U
} AD7606_Oversampling_t;

HAL_StatusTypeDef AD7606_Init(void);
HAL_StatusTypeDef AD7606_Reset(void);
HAL_StatusTypeDef AD7606_SetOversampling(AD7606_Oversampling_t mode);
HAL_StatusTypeDef AD7606_StartConversion(void);
HAL_StatusTypeDef AD7606_WaitReady(uint32_t timeout_us);
HAL_StatusTypeDef AD7606_ReadSamples(int16_t samples[AD7606_CHANNEL_COUNT]);
HAL_StatusTypeDef AD7606_ReadFrame(int16_t samples[AD7606_CHANNEL_COUNT]);
HAL_StatusTypeDef AD7606_ReadChannel(uint8_t channel, int16_t *sample);
float AD7606_CodeToVoltage(int16_t code, float input_range_volts);

/* Compatibility names used by the supplied reference program. */
void GPIO_AD7606_Configuration(void);
void AD7606_startconvst(void);
void AD7606_reset(void);
void AD7606_read_data(int16_t *DB_data);

#endif /* BSP_AD7606_H */
