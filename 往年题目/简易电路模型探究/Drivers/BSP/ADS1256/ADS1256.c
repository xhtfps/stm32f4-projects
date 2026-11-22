/**
 * @file    ADS1256.c
 * @brief   ADS1256 24位高精度ADC驱动
 *
 * 使用GPIO模拟SPI时序与ADS1256通信，提供初始化、寄存器读写、
 * 单次/多次平均采样以及码值转电压等功能。
 * 引脚映射参考已验证的AD DA+AD9959+ADS1256工程实物接线。
 */

#include "ADS1256.h"
#include "./SYSTEM/delay/delay.h"

/* ================================================================
 * 引脚定义
 *
 * PF8  = SCLK      (MCU -> ADS1256)
 * PF9  = DIN/MOSI  (MCU -> ADS1256 DIN)
 * PF11 = DOUT/MISO (ADS1256 DOUT -> MCU)
 * PF13 = DRDY      (ADS1256 DRDY -> MCU)
 * PF15 = CS        (MCU -> ADS1256)
 * PG0  = PDWN      (ADS1256 power-down 输入，外部上拉)
 * ================================================================ */
#define ADS1256_GPIO_PORT       GPIOF
#define ADS1256_GPIO_PDWN_PORT  GPIOG
#define ADS1256_PIN_SCLK        GPIO_PIN_8
#define ADS1256_PIN_MOSI        GPIO_PIN_9
#define ADS1256_PIN_MISO        GPIO_PIN_11
#define ADS1256_PIN_DRDY        GPIO_PIN_13
#define ADS1256_PIN_CS          GPIO_PIN_15
#define ADS1256_PIN_PDWN        GPIO_PIN_0

/* 超时与延时常量 */
#define ADS1256_READY_TIMEOUT_US 50000U   /**< DRDY 等待超时 (微秒) */
#if (ADS1256_CLKIN_HZ == 0U)
#error "ADS1256_CLKIN_HZ must be greater than zero"
#endif

/* 向上取整，保证满足数据手册以 τCLKIN 为单位的时序。 */
#define ADS1256_CLK_CYCLES_TO_US(cycles) \
    ((uint32_t)((((uint64_t)(cycles) * 1000000ULL) + \
                 (uint64_t)ADS1256_CLKIN_HZ - 1ULL) / \
                (uint64_t)ADS1256_CLKIN_HZ))

#define ADS1256_T6_DELAY_US            ADS1256_CLK_CYCLES_TO_US(50U)
#define ADS1256_CS_HOLD_DELAY_US       ADS1256_CLK_CYCLES_TO_US(8U)
#define ADS1256_COMMAND_DELAY_US       ADS1256_CLK_CYCLES_TO_US(4U)
#define ADS1256_SYNC_WAKEUP_DELAY_US  ADS1256_CLK_CYCLES_TO_US(24U)
#define ADS1256_SCLK_HALF_PERIOD_US    5U
#define ADS1256_CS_SETUP_DELAY_US      1U
#define ADS1256_DIN_SETUP_DELAY_US     1U

/* ---------- 静态函数声明 ---------- */
static HAL_StatusTypeDef ADS1256_WaitReady(uint32_t timeout_us);
static HAL_StatusTypeDef ADS1256_WaitDrdyLowAfterCommand(uint32_t timeout_us);
static void ADS1256_Select(void);
static void ADS1256_Deselect(void);
static void ADS1256_WriteByte(uint8_t data);
static uint8_t ADS1256_ReadByte(void);
static HAL_StatusTypeDef ADS1256_ReadRawData(uint32_t *raw_data);
static int32_t ADS1256_SignExtend(uint32_t raw_data);

/* ================================================================
 * 片选控制
 * ================================================================ */

/** 选中 ADS1256（CS 拉低） */
static void ADS1256_Select(void)
{
    HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_CS, GPIO_PIN_RESET);
    delay_us(ADS1256_CS_SETUP_DELAY_US);
}

/** 取消选中（CS 拉高） */
static void ADS1256_Deselect(void)
{
    /* t10: 最后一个 SCLK 下降沿后，CS 仍须保持低电平 8τCLKIN。 */
    delay_us(ADS1256_CS_HOLD_DELAY_US);
    HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_CS, GPIO_PIN_SET);
}

/* ================================================================
 * DRDY 等待函数
 * ================================================================ */

/**
 * @brief  等待 DRDY 引脚变低，表示数据准备就绪
 * @param  timeout_us 超时时间 (微秒)
 * @retval HAL_OK 就绪
 * @retval HAL_TIMEOUT 超时
 */
static HAL_StatusTypeDef ADS1256_WaitReady(uint32_t timeout_us)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t timeout_ms = (timeout_us + 999U) / 1000U;

    while (HAL_GPIO_ReadPin(ADS1256_GPIO_PORT, ADS1256_PIN_DRDY) != GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        delay_us(1U);
    }
    return HAL_OK;
}

/**
 * @brief  等待会强制 DRDY 拉高的命令完成
 * @note   适用于 SYNC/WAKEUP、SELFCAL 等会重启或暂停转换的命令。
 *         RDATA 不使用本函数：RDATA 后 DRDY 要等 24 位读完才会置高。
 */
static HAL_StatusTypeDef ADS1256_WaitDrdyLowAfterCommand(uint32_t timeout_us)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t timeout_ms = (timeout_us + 999U) / 1000U;

    while (HAL_GPIO_ReadPin(ADS1256_GPIO_PORT, ADS1256_PIN_DRDY) ==
           GPIO_PIN_RESET)
    {
        if ((HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        delay_us(1U);
    }

    return ADS1256_WaitReady(timeout_us);
}

static void ADS1256_WriteByte(uint8_t data)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; ++bit)
    {
        /* DIN 必须在 SCLK 下降沿前稳定，满足 t4/t5。 */
        HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_MOSI,
                          (data & 0x80U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1U;

        delay_us(ADS1256_DIN_SETUP_DELAY_US);
        HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_SCLK, GPIO_PIN_SET);
        delay_us(ADS1256_SCLK_HALF_PERIOD_US);
        HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_SCLK, GPIO_PIN_RESET);
        delay_us(ADS1256_SCLK_HALF_PERIOD_US);
    }
}

/**
 * @brief 从 ADS1256 读取一个字节
 * @return 读取到的数据
 */
static uint8_t ADS1256_ReadByte(void)
{
    uint8_t bit;
    uint8_t data = 0U;

    for (bit = 0U; bit < 8U; ++bit)
    {
        HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_SCLK, GPIO_PIN_SET);
        delay_us(ADS1256_SCLK_HALF_PERIOD_US);
        data = (uint8_t)(data << 1U);
        if (HAL_GPIO_ReadPin(ADS1256_GPIO_PORT, ADS1256_PIN_MISO) == GPIO_PIN_SET)
        {
            data |= 0x01U;
        }

        HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_SCLK, GPIO_PIN_RESET);
        delay_us(ADS1256_SCLK_HALF_PERIOD_US);
    }
    return data;
}

/* ================================================================
 * 原始数据读取与符号扩展
 * ================================================================ */

/**
 * @brief  发送 RDATA 命令并读取 24 位原始码
 * @return 24 位原始数据（右对齐）
 */
static HAL_StatusTypeDef ADS1256_ReadRawData(uint32_t *raw_data)
{
    if (raw_data == NULL)
    {
        return HAL_ERROR;
    }

    ADS1256_WriteByte(ADS1256_CMD_RDATA);
    /*
     * RDATA is issued after DRDY has already gone low.  RDATA does not
     * start a new conversion and DRDY only returns high after the 24 bits
     * have been shifted out, so waiting for another DRDY transition here
     * reads the timing sequence one conversion late.  Only t6 is required
     * between the command and the first data clock.
     */
    delay_us(ADS1256_T6_DELAY_US);

    *raw_data = ((uint32_t)ADS1256_ReadByte() << 16U);
    *raw_data |= ((uint32_t)ADS1256_ReadByte() << 8U);
    *raw_data |= (uint32_t)ADS1256_ReadByte();
    return HAL_OK;
}

/**
 * @brief  将 24 位偏移二进制码转换为有符号 32 位整数
 * @param  raw_data 24 位原始数据
 * @return 符号扩展后的 int32_t 值
 */
static int32_t ADS1256_SignExtend(uint32_t raw_data)
{
    if ((raw_data & 0x00800000UL) != 0U)
    {
        raw_data |= 0xFF000000UL;  /* 负数：高8位置1 */
    }
    return (int32_t)raw_data;
}

/* ================================================================
 * GPIO 初始化
 * ================================================================ */

/**
 * @brief 初始化 ADS1256 相关 GPIO 引脚
 */
void Init_ADS1256_GPIO(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* SCLK, MOSI, CS 推挽输出 */
    gpio.Pin = ADS1256_PIN_SCLK | ADS1256_PIN_MOSI | ADS1256_PIN_CS;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(ADS1256_GPIO_PORT, &gpio);

    /* MISO, DRDY 输入上拉 */
    gpio.Pin = ADS1256_PIN_MISO | ADS1256_PIN_DRDY;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ADS1256_GPIO_PORT, &gpio);

    /* PDWN 输入上拉 */
    gpio.Pin = ADS1256_PIN_PDWN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ADS1256_GPIO_PDWN_PORT, &gpio);

    /* 初始电平：SCLK/MOSI 低，CS 高（未选中） */
    HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_SCLK, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ADS1256_GPIO_PORT, ADS1256_PIN_MOSI, GPIO_PIN_RESET);
    ADS1256_Deselect();
}

/* ================================================================
 * 状态读取
 * ================================================================ */

/**
 * @brief  读取 DRDY 引脚状态
 * @return 1:数据就绪, 0:忙
 */
uint8_t ADS1256_IsDataReady(void)
{
    return (HAL_GPIO_ReadPin(ADS1256_GPIO_PORT, ADS1256_PIN_DRDY) ==
            GPIO_PIN_RESET) ? 1U : 0U;
}

/**
 * @brief  读取 PDWN 引脚状态
 * @return 1:高电平, 0:低电平
 */
uint8_t ADS1256_IsPowerDownHigh(void)
{
    return (HAL_GPIO_ReadPin(ADS1256_GPIO_PDWN_PORT, ADS1256_PIN_PDWN) ==
            GPIO_PIN_SET) ? 1U : 0U;
}

/* ================================================================
 * 寄存器读写
 * ================================================================ */

/**
 * @brief  写 ADS1256 寄存器
 * @param  address 寄存器地址 (低4位有效)
 * @param  value   待写入值
 * @return HAL_OK 或 HAL_TIMEOUT
 */
HAL_StatusTypeDef ADS1256_WriteRegister(uint8_t address, uint8_t value)
{
    HAL_StatusTypeDef status = ADS1256_WaitReady(ADS1256_READY_TIMEOUT_US);

    if (status != HAL_OK)
    {
        return status;
    }
    ADS1256_Select();
    ADS1256_WriteByte(ADS1256_CMD_WREG | (address & 0x0FU));
    delay_us(ADS1256_COMMAND_DELAY_US);
    ADS1256_WriteByte(0x00U);            /* 写入 1 个寄存器 (count = 0) */
    delay_us(ADS1256_COMMAND_DELAY_US);
    ADS1256_WriteByte(value);
    ADS1256_Deselect();
    return HAL_OK;
}

/**
 * @brief  读 ADS1256 寄存器
 * @param  address 寄存器地址
 * @param  value   输出参数，读取到的值
 * @return HAL_OK 或 HAL_TIMEOUT / HAL_ERROR
 */
HAL_StatusTypeDef ADS1256_ReadRegister(uint8_t address, uint8_t *value)
{
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }
    status = ADS1256_WaitReady(ADS1256_READY_TIMEOUT_US);
    if (status != HAL_OK)
    {
        return status;
    }
    ADS1256_Select();
    ADS1256_WriteByte(ADS1256_CMD_RREG | (address & 0x0FU));
    delay_us(ADS1256_COMMAND_DELAY_US);
    ADS1256_WriteByte(0x00U);            /* 读取 1 个寄存器 */
    delay_us(ADS1256_T6_DELAY_US);
    *value = ADS1256_ReadByte();
    ADS1256_Deselect();
    return HAL_OK;
}

/* ================================================================
 * 芯片初始化
 * ================================================================ */

/**
 * @brief  完整初始化 ADS1256
 *
 * 包含 GPIO 初始化、复位、停止连续读数、自校准，并配置
 * STATUS=0x04 (关闭输入缓冲), MUX=AIN0-AINCOM, ADCON=1 倍增益,
 * DRATE=1000SPS。
 * @return HAL_OK 或 HAL_TIMEOUT / HAL_ERROR
 */
HAL_StatusTypeDef ADS1256_Init(void)
{
    HAL_StatusTypeDef status;

    Init_ADS1256_GPIO();
    HAL_Delay(10U);

    /* 复位过程保持 CS 低（与参考驱动一致） */
    ADS1256_Select();
    delay_us(100U);
    status = ADS1256_WaitReady(ADS1256_READY_TIMEOUT_US);
    if (status != HAL_OK)
    {
        ADS1256_Deselect();
        return status;
    }
    ADS1256_WriteByte(ADS1256_CMD_RESET);  /* 发送复位命令 */
    HAL_Delay(10U);
    status = ADS1256_WaitReady(ADS1256_READY_TIMEOUT_US);
    if (status != HAL_OK)
    {
        ADS1256_Deselect();
        return status;
    }

    ADS1256_WriteByte(ADS1256_CMD_SDATAC); /* 停止连续读数模式 */
    delay_us(ADS1256_COMMAND_DELAY_US);
    status = ADS1256_WaitReady(ADS1256_READY_TIMEOUT_US);
    if (status != HAL_OK)
    {
        ADS1256_Deselect();
        return status;
    }

    ADS1256_WriteByte(ADS1256_CMD_SELFCAL); /* 自校准 */
    status = ADS1256_WaitDrdyLowAfterCommand(ADS1256_READY_TIMEOUT_US);
    if (status != HAL_OK)
    {
        ADS1256_Deselect();
        return status;
    }

    /* 同步并唤醒 */
    ADS1256_WriteByte(ADS1256_CMD_SYNC);
    delay_us(ADS1256_SYNC_WAKEUP_DELAY_US);
    ADS1256_WriteByte(ADS1256_CMD_WAKEUP);
    delay_us(ADS1256_SYNC_WAKEUP_DELAY_US);
    ADS1256_Deselect();

    /* 参考工程配置：输入缓冲关闭(STATUS=0x04), 增益=1, 数据率=1000SPS */
    if (ADS1256_WriteRegister(ADS1256_STATUS, 0x04U) != HAL_OK ||
        ADS1256_WriteRegister(ADS1256_MUX, ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM) != HAL_OK ||
        ADS1256_WriteRegister(ADS1256_ADCON, ADS1256_GAIN_1) != HAL_OK ||
        ADS1256_WriteRegister(ADS1256_DRATE, ADS1256_DRATE_1000SPS) != HAL_OK ||
        ADS1256_WriteRegister(ADS1256_IO, 0x00U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 最后再进行一次自校准，与参考工程流程一致 */
    ADS1256_Select();
    ADS1256_WriteByte(ADS1256_CMD_SELFCAL);
    status = ADS1256_WaitDrdyLowAfterCommand(ADS1256_READY_TIMEOUT_US);
    ADS1256_Deselect();
    return status;
}

/* ================================================================
 * 单次读取
 * ================================================================ */

/**
 * @brief  切换通道并读取一次转换结果
 * @param  mux   通道选择 (如 ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM)
 * @param  value 输出，有符号码值
 * @return HAL_OK 或 HAL_TIMEOUT / HAL_ERROR
 */
HAL_StatusTypeDef ADS1256_Read(uint8_t mux, int32_t *value)
{
    uint32_t raw_data;
    HAL_StatusTypeDef status;

    if (value == NULL)
    {
        return HAL_ERROR;
    }
    status = ADS1256_WriteRegister(ADS1256_MUX, mux);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 同步 + 唤醒以确保通道切换后首次转换有效 */
    ADS1256_Select();
    ADS1256_WriteByte(ADS1256_CMD_SYNC);
    delay_us(ADS1256_SYNC_WAKEUP_DELAY_US);
    ADS1256_WriteByte(ADS1256_CMD_WAKEUP);
    delay_us(ADS1256_SYNC_WAKEUP_DELAY_US);
    ADS1256_Deselect();
    status = ADS1256_WaitDrdyLowAfterCommand(ADS1256_READY_TIMEOUT_US);
    if (status != HAL_OK)
    {
        return status;
    }

    ADS1256_Select();
    status = ADS1256_ReadRawData(&raw_data);
    ADS1256_Deselect();
    if (status != HAL_OK)
    {
        return status;
    }

    *value = ADS1256_SignExtend(raw_data);
    return HAL_OK;
}

/* ================================================================
 * 多次平均读取（去极值平均）
 * ================================================================ */

/**
 * @brief  连续采样指定次数，去掉最大值和最小值后取平均
 * @param  mux     通道选择
 * @param  samples 采样次数 (3 ~ 100)
 * @param  value   输出，平均值（有符号）
 * @return HAL_OK 或 HAL_TIMEOUT / HAL_ERROR
 */
HAL_StatusTypeDef ADS1256_ReadAverage(uint8_t mux, uint16_t samples, int32_t *value)
{
    int64_t sum = 0;
    int32_t maximum = 0;
    int32_t minimum = 0;
    int32_t sample;
    uint32_t raw_data;
    uint16_t index;
    HAL_StatusTypeDef status;

    if ((samples == 0U) || (value == NULL))
    {
        return HAL_ERROR;
    }

    /* 限制采样次数 */
    if (samples > 100U)
    {
        samples = 100U;
    }
    if (samples < 3U)
    {
        return ADS1256_Read(mux, value);
    }

    /* 仅设置一次通道，然后连续读取多个转换值 */
    status = ADS1256_WriteRegister(ADS1256_MUX, mux);
    if (status != HAL_OK)
    {
        return status;
    }

    ADS1256_Select();
    ADS1256_WriteByte(ADS1256_CMD_SYNC);
    delay_us(ADS1256_SYNC_WAKEUP_DELAY_US);
    ADS1256_WriteByte(ADS1256_CMD_WAKEUP);
    delay_us(ADS1256_SYNC_WAKEUP_DELAY_US);

    for (index = 0U; index < samples; ++index)
    {
        if (index == 0U)
        {
            /* SYNC/WAKEUP 会强制 DRDY 拉高，首个转换必须等高到低。 */
            status = ADS1256_WaitDrdyLowAfterCommand(
                ADS1256_READY_TIMEOUT_US);
        }
        else
        {
            /* 上一次 RDATA 已读完 24 位，DRDY 被置高；等待下一次转换完成。 */
            status = ADS1256_WaitReady(ADS1256_READY_TIMEOUT_US);
        }
        if (status != HAL_OK)
        {
            ADS1256_Deselect();
            return status;
        }

        /* 仅执行串行移位，避免长时间关闭全局中断。 */
        status = ADS1256_ReadRawData(&raw_data);
        if (status != HAL_OK)
        {
            ADS1256_Deselect();
            return status;
        }
        sample = ADS1256_SignExtend(raw_data);

        if (index == 0U)
        {
            maximum = sample;
            minimum = sample;
        }
        else
        {
            if (sample > maximum)
            {
                maximum = sample;
            }
            if (sample < minimum)
            {
                minimum = sample;
            }
        }
        sum += sample;
    }

    ADS1256_Deselect();
    /* 去掉最大最小值后的平均值 */
    *value = (int32_t)((sum - (int64_t)maximum - (int64_t)minimum) /
                       (int64_t)(samples - 2U));
    return HAL_OK;
}

/* ================================================================
 * 码值转电压
 * ================================================================ */

/**
 * @brief  将 ADS1256 有符号码值转换为电压
 * @param  code             有符号 ADC 码
 * @param  reference_voltage 参考电压 (V)
 * @param  gain              PGA 增益 (1,2,4,8...)
 * @return 电压值 (V)
 *
 * 公式: V = (code * 2 * Vref) / (gain * (2^23 - 1))
 */
float ADS1256_CodeToVoltage(int32_t code, float reference_voltage, uint8_t gain)
{
    if ((gain == 0U) || (reference_voltage <= 0.0f))
    {
        return 0.0f;
    }
    return ((float)code * 2.0f * reference_voltage) / ((float)gain * 8388607.0f);
}

/* ================================================================
 * 以下函数为兼容旧版接口，内部直接调用上述 BSP 函数
 * ================================================================ */

void SPI_WriteByte(uint8_t data)
{
    ADS1256_WriteByte(data);
}

uint8_t SPI_ReadByte(void)
{
    return ADS1256_ReadByte();
}

void ADS1256WREG(uint8_t address, uint8_t value)
{
    (void)ADS1256_WriteRegister(address, value);
}

uint8_t ADS1256RREG(uint8_t address)
{
    uint8_t value = 0xFFU;

    (void)ADS1256_ReadRegister(address, &value);
    return value;
}

uint32_t ADS1256ReadData(uint8_t mux)
{
    int32_t value;

    if (ADS1256_Read(mux, &value) != HAL_OK)
    {
        return ADS1256_READ_ERROR;
    }
    return (uint32_t)value;
}

float Get_Val(uint32_t raw_data)
{
    return ADS1256_CodeToVoltage((int32_t)raw_data,
                                 ADS1256_REFERENCE_VOLTAGE_DEFAULT, 1U);
}

uint32_t Moving_Average_Filter(uint8_t mux, uint16_t samples)
{
    int32_t value;

    if (ADS1256_ReadAverage(mux, samples, &value) != HAL_OK)
    {
        return ADS1256_READ_ERROR;
    }
    return (uint32_t)value;
}
