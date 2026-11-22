/**
 * @file    iic.c
 * @brief   STM32F407 软件 I2C 主机驱动实现。
 */

#include "iic.h"
#include "./SYSTEM/delay/delay.h"

typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
    uint8_t initialized;
} IicBus;

static IicBus s_iic1_bus =
{
    IIC1_SCL_GPIO_PORT,
    IIC1_SCL_GPIO_PIN,
    IIC1_SDA_GPIO_PORT,
    IIC1_SDA_GPIO_PIN,
    0U
};

static IicBus s_iic2_bus =
{
    IIC2_SCL_GPIO_PORT,
    IIC2_SCL_GPIO_PIN,
    IIC2_SDA_GPIO_PORT,
    IIC2_SDA_GPIO_PIN,
    0U
};

static void Iic_EnablePortClock(GPIO_TypeDef *port)
{
    if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
}

static void Iic_ReleaseScl(const IicBus *bus)
{
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_SET);
}

static void Iic_HoldSclLow(const IicBus *bus)
{
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, GPIO_PIN_RESET);
}

static void Iic_ReleaseSda(const IicBus *bus)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_SET);
}

static void Iic_HoldSdaLow(const IicBus *bus)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, GPIO_PIN_RESET);
}

static GPIO_PinState Iic_ReadScl(const IicBus *bus)
{
    return HAL_GPIO_ReadPin(bus->scl_port, bus->scl_pin);
}

static GPIO_PinState Iic_ReadSda(const IicBus *bus)
{
    return HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin);
}

static HAL_StatusTypeDef Iic_WaitSclHigh(const IicBus *bus)
{
    uint32_t wait_us = 0U;

    Iic_ReleaseScl(bus);
    while (Iic_ReadScl(bus) == GPIO_PIN_RESET)
    {
        if (wait_us >= IIC_SCL_TIMEOUT_US)
        {
            return HAL_TIMEOUT;
        }

        delay_us(1U);
        wait_us++;
    }

    return HAL_OK;
}

static void Iic_Delay(void)
{
    delay_us(IIC_SOFTWARE_DELAY_US);
}

static HAL_StatusTypeDef Iic_BusInit(IicBus *bus)
{
    GPIO_InitTypeDef gpio_init;

    if (bus == NULL)
    {
        return HAL_ERROR;
    }

    Iic_EnablePortClock(bus->scl_port);
    Iic_EnablePortClock(bus->sda_port);

    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = 0U;

    gpio_init.Pin = bus->scl_pin;
    HAL_GPIO_Init(bus->scl_port, &gpio_init);
    gpio_init.Pin = bus->sda_pin;
    HAL_GPIO_Init(bus->sda_port, &gpio_init);

    Iic_ReleaseScl(bus);
    Iic_ReleaseSda(bus);
    Iic_Delay();
    bus->initialized = 1U;

    return HAL_OK;
}

static HAL_StatusTypeDef Iic_Start(IicBus *bus)
{
    if ((bus == NULL) || (bus->initialized == 0U))
    {
        return HAL_ERROR;
    }

    Iic_ReleaseSda(bus);
    if (Iic_WaitSclHigh(bus) != HAL_OK)
    {
        return HAL_TIMEOUT;
    }

    if (Iic_ReadSda(bus) == GPIO_PIN_RESET)
    {
        return HAL_BUSY;
    }

    Iic_Delay();
    Iic_HoldSdaLow(bus);
    Iic_Delay();
    Iic_HoldSclLow(bus);
    Iic_Delay();

    return HAL_OK;
}

static HAL_StatusTypeDef Iic_Stop(IicBus *bus)
{
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (bus->initialized == 0U))
    {
        return HAL_ERROR;
    }

    Iic_HoldSdaLow(bus);
    Iic_Delay();
    status = Iic_WaitSclHigh(bus);
    Iic_Delay();
    Iic_ReleaseSda(bus);
    Iic_Delay();

    return status;
}

static HAL_StatusTypeDef Iic_WriteByte(IicBus *bus, uint8_t data)
{
    uint8_t bit_index;
    GPIO_PinState ack;
    HAL_StatusTypeDef status;

    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        if ((data & 0x80U) != 0U)
        {
            Iic_ReleaseSda(bus);
        }
        else
        {
            Iic_HoldSdaLow(bus);
        }

        Iic_Delay();
        status = Iic_WaitSclHigh(bus);
        if (status != HAL_OK)
        {
            Iic_HoldSclLow(bus);
            return status;
        }

        Iic_Delay();
        Iic_HoldSclLow(bus);
        Iic_Delay();
        data <<= 1U;
    }

    /* 释放 SDA，由从机在第 9 个时钟周期拉低表示 ACK。 */
    Iic_ReleaseSda(bus);
    Iic_Delay();
    status = Iic_WaitSclHigh(bus);
    if (status != HAL_OK)
    {
        Iic_HoldSclLow(bus);
        return status;
    }

    Iic_Delay();
    ack = Iic_ReadSda(bus);
    Iic_HoldSclLow(bus);
    Iic_Delay();

    return (ack == GPIO_PIN_RESET) ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef Iic_ReadByte(IicBus *bus,
                                      uint8_t *data,
                                      uint8_t acknowledge)
{
    uint8_t bit_index;
    uint8_t received = 0U;
    HAL_StatusTypeDef status;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    Iic_ReleaseSda(bus);
    for (bit_index = 0U; bit_index < 8U; bit_index++)
    {
        received <<= 1U;
        status = Iic_WaitSclHigh(bus);
        if (status != HAL_OK)
        {
            Iic_HoldSclLow(bus);
            *data = 0xFFU;
            return status;
        }

        Iic_Delay();
        if (Iic_ReadSda(bus) == GPIO_PIN_SET)
        {
            received |= 1U;
        }

        Iic_HoldSclLow(bus);
        Iic_Delay();
    }

    /* acknowledge 非 0 发送 ACK，0 发送 NACK。 */
    if (acknowledge != 0U)
    {
        Iic_HoldSdaLow(bus);
    }
    else
    {
        Iic_ReleaseSda(bus);
    }

    Iic_Delay();
    status = Iic_WaitSclHigh(bus);
    Iic_Delay();
    Iic_HoldSclLow(bus);
    Iic_ReleaseSda(bus);
    Iic_Delay();

    *data = received;
    return status;
}

static HAL_StatusTypeDef Iic_Address(IicBus *bus,
                                     uint8_t dev_addr,
                                     uint8_t read)
{
    if (dev_addr > 0x7FU)
    {
        return HAL_ERROR;
    }

    return Iic_WriteByte(bus,
                         (uint8_t)((dev_addr << 1U) | (read != 0U ? 1U : 0U)));
}

static HAL_StatusTypeDef Iic_MasterTransmit(IicBus *bus,
                                            uint8_t dev_addr,
                                            uint8_t *data,
                                            uint16_t size)
{
    HAL_StatusTypeDef status;
    uint16_t index;

    if ((bus == NULL) || (bus->initialized == 0U) ||
        ((data == NULL) && (size != 0U)))
    {
        return HAL_ERROR;
    }

    status = Iic_Start(bus);
    if (status != HAL_OK)
    {
        return status;
    }

    status = Iic_Address(bus, dev_addr, 0U);
    for (index = 0U; (status == HAL_OK) && (index < size); index++)
    {
        status = Iic_WriteByte(bus, data[index]);
    }

    if (Iic_Stop(bus) != HAL_OK)
    {
        status = HAL_TIMEOUT;
    }

    return status;
}

static HAL_StatusTypeDef Iic_MasterReceive(IicBus *bus,
                                           uint8_t dev_addr,
                                           uint8_t *data,
                                           uint16_t size)
{
    HAL_StatusTypeDef status;
    uint16_t index;

    if ((bus == NULL) || (bus->initialized == 0U) ||
        ((data == NULL) && (size != 0U)))
    {
        return HAL_ERROR;
    }

    status = Iic_Start(bus);
    if (status != HAL_OK)
    {
        return status;
    }

    status = Iic_Address(bus, dev_addr, 1U);
    for (index = 0U; (status == HAL_OK) && (index < size); index++)
    {
        status = Iic_ReadByte(bus,
                              &data[index],
                              (index + 1U < size) ? 1U : 0U);
    }

    if (Iic_Stop(bus) != HAL_OK)
    {
        status = HAL_TIMEOUT;
    }

    return status;
}

static HAL_StatusTypeDef Iic_MemWrite8(IicBus *bus,
                                       uint8_t dev_addr,
                                       uint8_t reg_addr,
                                       uint8_t *data,
                                       uint16_t size)
{
    HAL_StatusTypeDef status;
    uint16_t index;

    if ((bus == NULL) || (bus->initialized == 0U) ||
        ((data == NULL) && (size != 0U)))
    {
        return HAL_ERROR;
    }

    status = Iic_Start(bus);
    if (status == HAL_OK)
    {
        status = Iic_Address(bus, dev_addr, 0U);
    }
    if (status == HAL_OK)
    {
        status = Iic_WriteByte(bus, reg_addr);
    }
    for (index = 0U; (status == HAL_OK) && (index < size); index++)
    {
        status = Iic_WriteByte(bus, data[index]);
    }

    if (Iic_Stop(bus) != HAL_OK)
    {
        status = HAL_TIMEOUT;
    }

    return status;
}

static HAL_StatusTypeDef Iic_MemRead8(IicBus *bus,
                                      uint8_t dev_addr,
                                      uint8_t reg_addr,
                                      uint8_t *data,
                                      uint16_t size)
{
    HAL_StatusTypeDef status;
    uint16_t index;

    if ((bus == NULL) || (bus->initialized == 0U) ||
        ((data == NULL) && (size != 0U)))
    {
        return HAL_ERROR;
    }

    status = Iic_Start(bus);
    if (status == HAL_OK)
    {
        status = Iic_Address(bus, dev_addr, 0U);
    }
    if (status == HAL_OK)
    {
        status = Iic_WriteByte(bus, reg_addr);
    }
    if (status == HAL_OK)
    {
        /* repeated START，不释放总线，切换到读方向。 */
        status = Iic_Start(bus);
    }
    if (status == HAL_OK)
    {
        status = Iic_Address(bus, dev_addr, 1U);
    }
    for (index = 0U; (status == HAL_OK) && (index < size); index++)
    {
        status = Iic_ReadByte(bus,
                              &data[index],
                              (index + 1U < size) ? 1U : 0U);
    }

    if (Iic_Stop(bus) != HAL_OK)
    {
        status = HAL_TIMEOUT;
    }

    return status;
}

static HAL_StatusTypeDef Iic_IsDeviceReady(IicBus *bus, uint8_t dev_addr)
{
    HAL_StatusTypeDef status;

    status = Iic_Start(bus);
    if (status == HAL_OK)
    {
        status = Iic_Address(bus, dev_addr, 0U);
    }

    if (Iic_Stop(bus) != HAL_OK)
    {
        status = HAL_TIMEOUT;
    }

    return status;
}

HAL_StatusTypeDef IIC1_Init(void)
{
    return Iic_BusInit(&s_iic1_bus);
}

HAL_StatusTypeDef IIC2_Init(void)
{
    return Iic_BusInit(&s_iic2_bus);
}

HAL_StatusTypeDef IIC1_MasterTransmit(uint8_t dev_addr,
                                      uint8_t *data,
                                      uint16_t size)
{
    return Iic_MasterTransmit(&s_iic1_bus, dev_addr, data, size);
}

HAL_StatusTypeDef IIC2_MasterTransmit(uint8_t dev_addr,
                                      uint8_t *data,
                                      uint16_t size)
{
    return Iic_MasterTransmit(&s_iic2_bus, dev_addr, data, size);
}

HAL_StatusTypeDef IIC1_MasterReceive(uint8_t dev_addr,
                                     uint8_t *data,
                                     uint16_t size)
{
    return Iic_MasterReceive(&s_iic1_bus, dev_addr, data, size);
}

HAL_StatusTypeDef IIC2_MasterReceive(uint8_t dev_addr,
                                     uint8_t *data,
                                     uint16_t size)
{
    return Iic_MasterReceive(&s_iic2_bus, dev_addr, data, size);
}

HAL_StatusTypeDef IIC1_MemWrite8(uint8_t dev_addr,
                                 uint8_t reg_addr,
                                 uint8_t *data,
                                 uint16_t size)
{
    return Iic_MemWrite8(&s_iic1_bus, dev_addr, reg_addr, data, size);
}

HAL_StatusTypeDef IIC2_MemWrite8(uint8_t dev_addr,
                                 uint8_t reg_addr,
                                 uint8_t *data,
                                 uint16_t size)
{
    return Iic_MemWrite8(&s_iic2_bus, dev_addr, reg_addr, data, size);
}

HAL_StatusTypeDef IIC1_MemRead8(uint8_t dev_addr,
                                uint8_t reg_addr,
                                uint8_t *data,
                                uint16_t size)
{
    return Iic_MemRead8(&s_iic1_bus, dev_addr, reg_addr, data, size);
}

HAL_StatusTypeDef IIC2_MemRead8(uint8_t dev_addr,
                                uint8_t reg_addr,
                                uint8_t *data,
                                uint16_t size)
{
    return Iic_MemRead8(&s_iic2_bus, dev_addr, reg_addr, data, size);
}

HAL_StatusTypeDef IIC1_IsDeviceReady(uint8_t dev_addr)
{
    return Iic_IsDeviceReady(&s_iic1_bus, dev_addr);
}

HAL_StatusTypeDef IIC2_IsDeviceReady(uint8_t dev_addr)
{
    return Iic_IsDeviceReady(&s_iic2_bus, dev_addr);
}
