/**
 * @file    MPU6050.c
 * @brief   MPU6050 六轴传感器驱动实现。
 */

#include "MPU6050.h"
#include "./BSP/IIC/iic.h"
#include "./SYSTEM/delay/delay.h"

#define MPU6050_REG_SMPLRT_DIV       0x19U
#define MPU6050_REG_CONFIG           0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_PWR_MGMT_2      0x6CU
#define MPU6050_REG_WHO_AM_I        0x75U

#define MPU6050_CLOCK_PLL_XGYRO      0x01U
#define MPU6050_DEVICE_RESET         0x80U
#define MPU6050_DEFAULT_SAMPLE_DIV   4U
#define MPU6050_DEFAULT_DLPF_CONFIG  3U

typedef HAL_StatusTypeDef (*MpuIicInitFn)(void);
typedef HAL_StatusTypeDef (*MpuIicMemReadFn)(uint8_t,
                                             uint8_t,
                                             uint8_t *,
                                             uint16_t);
typedef HAL_StatusTypeDef (*MpuIicMemWriteFn)(uint8_t,
                                              uint8_t,
                                              uint8_t *,
                                              uint16_t);
typedef HAL_StatusTypeDef (*MpuIicReadyFn)(uint8_t);

typedef struct
{
    MpuIicInitFn init;
    MpuIicMemReadFn mem_read;
    MpuIicMemWriteFn mem_write;
    MpuIicReadyFn is_ready;
} MpuIicBusOps;

static const MpuIicBusOps s_iic1_ops =
{
    IIC1_Init,
    IIC1_MemRead8,
    IIC1_MemWrite8,
    IIC1_IsDeviceReady
};

static const MpuIicBusOps s_iic2_ops =
{
    IIC2_Init,
    IIC2_MemRead8,
    IIC2_MemWrite8,
    IIC2_IsDeviceReady
};

static const MpuIicBusOps *s_iic_ops;

Mpu6050Device g_mpu6050 =
{
    MPU6050_DEFAULT_IIC_BUS,
    MPU6050_DEFAULT_ADDRESS,
    0U,
    MPU6050_DEFAULT_SAMPLE_DIV,
    MPU6050_DEFAULT_DLPF_CONFIG,
    MPU6050_ACCEL_RANGE_2G,
    MPU6050_GYRO_RANGE_250DPS
};

static HAL_StatusTypeDef Mpu6050_SelectBus(uint8_t iic_bus)
{
    if (iic_bus == MPU6050_IIC_BUS_1)
    {
        s_iic_ops = &s_iic1_ops;
    }
    else if (iic_bus == MPU6050_IIC_BUS_2)
    {
        s_iic_ops = &s_iic2_ops;
    }
    else
    {
        s_iic_ops = NULL;
        return HAL_ERROR;
    }

    return HAL_OK;
}

static HAL_StatusTypeDef Mpu6050_ReadRegRaw(uint8_t reg_addr,
                                            uint8_t *data,
                                            uint16_t size)
{
    if ((s_iic_ops == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    return s_iic_ops->mem_read(g_mpu6050.address,
                               reg_addr,
                               data,
                               size);
}

static HAL_StatusTypeDef Mpu6050_WriteRegRaw(uint8_t reg_addr,
                                             uint8_t *data,
                                             uint16_t size)
{
    if ((s_iic_ops == NULL) || (data == NULL) || (size == 0U))
    {
        return HAL_ERROR;
    }

    return s_iic_ops->mem_write(g_mpu6050.address,
                                reg_addr,
                                data,
                                size);
}

static float Mpu6050_AccelScale(Mpu6050AccelRange range)
{
    switch (range)
    {
        case MPU6050_ACCEL_RANGE_4G:
            return 8192.0f;
        case MPU6050_ACCEL_RANGE_8G:
            return 4096.0f;
        case MPU6050_ACCEL_RANGE_16G:
            return 2048.0f;
        case MPU6050_ACCEL_RANGE_2G:
        default:
            return 16384.0f;
    }
}

static float Mpu6050_GyroScale(Mpu6050GyroRange range)
{
    switch (range)
    {
        case MPU6050_GYRO_RANGE_500DPS:
            return 65.5f;
        case MPU6050_GYRO_RANGE_1000DPS:
            return 32.8f;
        case MPU6050_GYRO_RANGE_2000DPS:
            return 16.4f;
        case MPU6050_GYRO_RANGE_250DPS:
        default:
            return 131.0f;
    }
}

static int16_t Mpu6050_CombineBytes(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}

HAL_StatusTypeDef MPU6050_Init(void)
{
    return MPU6050_InitWithBus(MPU6050_DEFAULT_IIC_BUS,
                                MPU6050_DEFAULT_ADDRESS);
}

HAL_StatusTypeDef MPU6050_InitWithBus(uint8_t iic_bus, uint8_t address)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i;
    uint8_t value;

    if ((address != MPU6050_ADDRESS_AD0_LOW) &&
        (address != MPU6050_ADDRESS_AD0_HIGH))
    {
        return HAL_ERROR;
    }

    status = Mpu6050_SelectBus(iic_bus);
    if (status != HAL_OK)
    {
        return status;
    }

    g_mpu6050.iic_bus = iic_bus;
    g_mpu6050.address = address;
    g_mpu6050.initialized = 0U;
    g_mpu6050.sample_rate_divider = MPU6050_DEFAULT_SAMPLE_DIV;
    g_mpu6050.dlpf_config = MPU6050_DEFAULT_DLPF_CONFIG;
    g_mpu6050.accel_range = MPU6050_ACCEL_RANGE_2G;
    g_mpu6050.gyro_range = MPU6050_GYRO_RANGE_250DPS;

    status = s_iic_ops->init();
    if (status != HAL_OK)
    {
        return status;
    }

    status = s_iic_ops->is_ready(g_mpu6050.address);
    if (status != HAL_OK)
    {
        return status;
    }

    status = Mpu6050_ReadRegRaw(MPU6050_REG_WHO_AM_I, &who_am_i, 1U);
    if ((status != HAL_OK) || (who_am_i != MPU6050_WHO_AM_I_VALUE))
    {
        return HAL_ERROR;
    }

    value = MPU6050_DEVICE_RESET;
    status = Mpu6050_WriteRegRaw(MPU6050_REG_PWR_MGMT_1, &value, 1U);
    if (status != HAL_OK)
    {
        return status;
    }
    delay_ms(100U);

    value = MPU6050_CLOCK_PLL_XGYRO;
    status = Mpu6050_WriteRegRaw(MPU6050_REG_PWR_MGMT_1, &value, 1U);
    if (status == HAL_OK)
    {
        value = 0U;
        status = Mpu6050_WriteRegRaw(MPU6050_REG_PWR_MGMT_2, &value, 1U);
    }
    if (status == HAL_OK)
    {
        value = g_mpu6050.sample_rate_divider;
        status = Mpu6050_WriteRegRaw(MPU6050_REG_SMPLRT_DIV, &value, 1U);
    }
    if (status == HAL_OK)
    {
        value = g_mpu6050.dlpf_config;
        status = Mpu6050_WriteRegRaw(MPU6050_REG_CONFIG, &value, 1U);
    }
    if (status == HAL_OK)
    {
        value = (uint8_t)(g_mpu6050.gyro_range << 3U);
        status = Mpu6050_WriteRegRaw(MPU6050_REG_GYRO_CONFIG, &value, 1U);
    }
    if (status == HAL_OK)
    {
        value = (uint8_t)(g_mpu6050.accel_range << 3U);
        status = Mpu6050_WriteRegRaw(MPU6050_REG_ACCEL_CONFIG, &value, 1U);
    }

    if (status == HAL_OK)
    {
        g_mpu6050.initialized = 1U;
    }

    return status;
}

HAL_StatusTypeDef MPU6050_Reset(void)
{
    uint8_t value = MPU6050_DEVICE_RESET;
    HAL_StatusTypeDef status;

    if (g_mpu6050.initialized == 0U)
    {
        return HAL_ERROR;
    }

    status = Mpu6050_WriteRegRaw(MPU6050_REG_PWR_MGMT_1, &value, 1U);
    if (status == HAL_OK)
    {
        g_mpu6050.initialized = 0U;
        delay_ms(100U);
    }

    return status;
}

HAL_StatusTypeDef MPU6050_Wakeup(void)
{
    uint8_t value = MPU6050_CLOCK_PLL_XGYRO;
    HAL_StatusTypeDef status;

    if ((g_mpu6050.initialized == 0U) || (s_iic_ops == NULL))
    {
        return HAL_ERROR;
    }

    status = Mpu6050_WriteRegRaw(MPU6050_REG_PWR_MGMT_1, &value, 1U);
    if (status == HAL_OK)
    {
        value = 0U;
        status = Mpu6050_WriteRegRaw(MPU6050_REG_PWR_MGMT_2, &value, 1U);
    }

    return status;
}

HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg_addr, uint8_t *data)
{
    return MPU6050_ReadRegs(reg_addr, data, 1U);
}

HAL_StatusTypeDef MPU6050_ReadRegs(uint8_t reg_addr,
                                   uint8_t *data,
                                   uint16_t size)
{
    if (g_mpu6050.initialized == 0U)
    {
        return HAL_ERROR;
    }

    return Mpu6050_ReadRegRaw(reg_addr, data, size);
}

HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg_addr, uint8_t data)
{
    if (g_mpu6050.initialized == 0U)
    {
        return HAL_ERROR;
    }

    return Mpu6050_WriteRegRaw(reg_addr, &data, 1U);
}

HAL_StatusTypeDef MPU6050_ReadWhoAmI(uint8_t *who_am_i)
{
    return MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, who_am_i);
}

HAL_StatusTypeDef MPU6050_SetSampleRateDivider(uint8_t divider)
{
    HAL_StatusTypeDef status;

    if (g_mpu6050.initialized == 0U)
    {
        return HAL_ERROR;
    }

    status = MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, divider);
    if (status == HAL_OK)
    {
        g_mpu6050.sample_rate_divider = divider;
    }

    return status;
}

HAL_StatusTypeDef MPU6050_SetDlpfConfig(uint8_t config)
{
    HAL_StatusTypeDef status;

    if ((g_mpu6050.initialized == 0U) || (config > 7U))
    {
        return HAL_ERROR;
    }

    status = MPU6050_WriteReg(MPU6050_REG_CONFIG, config);
    if (status == HAL_OK)
    {
        g_mpu6050.dlpf_config = config;
    }

    return status;
}

HAL_StatusTypeDef MPU6050_SetAccelRange(Mpu6050AccelRange range)
{
    HAL_StatusTypeDef status;
    uint8_t value;

    if ((g_mpu6050.initialized == 0U) || (range > MPU6050_ACCEL_RANGE_16G))
    {
        return HAL_ERROR;
    }

    value = (uint8_t)(range << 3U);
    status = MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, value);
    if (status == HAL_OK)
    {
        g_mpu6050.accel_range = range;
    }

    return status;
}

HAL_StatusTypeDef MPU6050_SetGyroRange(Mpu6050GyroRange range)
{
    HAL_StatusTypeDef status;
    uint8_t value;

    if ((g_mpu6050.initialized == 0U) || (range > MPU6050_GYRO_RANGE_2000DPS))
    {
        return HAL_ERROR;
    }

    value = (uint8_t)(range << 3U);
    status = MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, value);
    if (status == HAL_OK)
    {
        g_mpu6050.gyro_range = range;
    }

    return status;
}

HAL_StatusTypeDef MPU6050_ReadRaw(Mpu6050RawData *data)
{
    uint8_t buffer[14];
    HAL_StatusTypeDef status;

    if ((data == NULL) || (g_mpu6050.initialized == 0U))
    {
        return HAL_ERROR;
    }

    status = MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buffer, 14U);
    if (status != HAL_OK)
    {
        return status;
    }

    data->accel_x = Mpu6050_CombineBytes(buffer[0], buffer[1]);
    data->accel_y = Mpu6050_CombineBytes(buffer[2], buffer[3]);
    data->accel_z = Mpu6050_CombineBytes(buffer[4], buffer[5]);
    data->temperature = Mpu6050_CombineBytes(buffer[6], buffer[7]);
    data->gyro_x = Mpu6050_CombineBytes(buffer[8], buffer[9]);
    data->gyro_y = Mpu6050_CombineBytes(buffer[10], buffer[11]);
    data->gyro_z = Mpu6050_CombineBytes(buffer[12], buffer[13]);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadData(Mpu6050Data *data)
{
    Mpu6050RawData raw;
    float accel_scale;
    float gyro_scale;
    HAL_StatusTypeDef status;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    status = MPU6050_ReadRaw(&raw);
    if (status != HAL_OK)
    {
        return status;
    }

    accel_scale = Mpu6050_AccelScale(g_mpu6050.accel_range);
    gyro_scale = Mpu6050_GyroScale(g_mpu6050.gyro_range);

    data->accel_x_g = (float)raw.accel_x / accel_scale;
    data->accel_y_g = (float)raw.accel_y / accel_scale;
    data->accel_z_g = (float)raw.accel_z / accel_scale;
    data->temperature_c = ((float)raw.temperature / 340.0f) + 36.53f;
    data->gyro_x_dps = (float)raw.gyro_x / gyro_scale;
    data->gyro_y_dps = (float)raw.gyro_y / gyro_scale;
    data->gyro_z_dps = (float)raw.gyro_z / gyro_scale;

    return HAL_OK;
}

uint8_t MPU6050_IsInitialized(void)
{
    return g_mpu6050.initialized;
}
