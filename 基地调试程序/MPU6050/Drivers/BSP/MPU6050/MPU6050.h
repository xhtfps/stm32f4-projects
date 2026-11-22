/**
 * @file    MPU6050.h
 * @brief   MPU6050 六轴传感器驱动。
 *
 * 驱动复用 Drivers/BSP/IIC 下的软件 I2C 总线，默认使用 IIC1 和 7 位地址
 * 0x68。AD0 接高电平时可通过 MPU6050_InitWithBus() 使用地址 0x69。
 */

#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#include "stm32f4xx_hal.h"

#define MPU6050_IIC_BUS_1             1U
#define MPU6050_IIC_BUS_2             2U
#define MPU6050_DEFAULT_IIC_BUS      MPU6050_IIC_BUS_1

#define MPU6050_ADDRESS_AD0_LOW       0x68U
#define MPU6050_ADDRESS_AD0_HIGH      0x69U
#define MPU6050_DEFAULT_ADDRESS       MPU6050_ADDRESS_AD0_LOW
#define MPU6050_WHO_AM_I_VALUE        0x68U

typedef enum
{
    MPU6050_ACCEL_RANGE_2G = 0U,
    MPU6050_ACCEL_RANGE_4G = 1U,
    MPU6050_ACCEL_RANGE_8G = 2U,
    MPU6050_ACCEL_RANGE_16G = 3U
} Mpu6050AccelRange;

typedef enum
{
    MPU6050_GYRO_RANGE_250DPS = 0U,
    MPU6050_GYRO_RANGE_500DPS = 1U,
    MPU6050_GYRO_RANGE_1000DPS = 2U,
    MPU6050_GYRO_RANGE_2000DPS = 3U
} Mpu6050GyroRange;

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} Mpu6050RawData;

typedef struct
{
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float temperature_c;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} Mpu6050Data;

typedef struct
{
    uint8_t iic_bus;
    uint8_t address;
    uint8_t initialized;
    uint8_t sample_rate_divider;
    uint8_t dlpf_config;
    Mpu6050AccelRange accel_range;
    Mpu6050GyroRange gyro_range;
} Mpu6050Device;

extern Mpu6050Device g_mpu6050;

/** 使用默认 IIC1/0x68 配置初始化。 */
HAL_StatusTypeDef MPU6050_Init(void);

/** 选择软件 I2C 总线和 7 位设备地址后初始化。 */
HAL_StatusTypeDef MPU6050_InitWithBus(uint8_t iic_bus, uint8_t address);

/** 复位设备；复位后需要再次调用 MPU6050_InitWithBus() 完成配置。 */
HAL_StatusTypeDef MPU6050_Reset(void);
HAL_StatusTypeDef MPU6050_Wakeup(void);

/** 寄存器访问接口，reg_addr 为 8 位寄存器地址。 */
HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg_addr, uint8_t *data);
HAL_StatusTypeDef MPU6050_ReadRegs(uint8_t reg_addr,
                                   uint8_t *data,
                                   uint16_t size);
HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint8_t *who_am_i);

/** 运行参数配置。 */
HAL_StatusTypeDef MPU6050_SetSampleRateDivider(uint8_t divider);
HAL_StatusTypeDef MPU6050_SetDlpfConfig(uint8_t config);
HAL_StatusTypeDef MPU6050_SetAccelRange(Mpu6050AccelRange range);
HAL_StatusTypeDef MPU6050_SetGyroRange(Mpu6050GyroRange range);

/** 读取原始值或按当前量程换算后的物理值。 */
HAL_StatusTypeDef MPU6050_ReadRaw(Mpu6050RawData *data);
HAL_StatusTypeDef MPU6050_ReadData(Mpu6050Data *data);

uint8_t MPU6050_IsInitialized(void);

#endif /* __BSP_MPU6050_H */
