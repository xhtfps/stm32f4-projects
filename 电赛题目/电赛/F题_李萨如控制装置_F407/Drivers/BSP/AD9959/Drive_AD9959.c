/**
 * @file    Drive_AD9959.c
 * @brief   AD9959 基地模板固定 GPIO 模拟串行驱动实现。
 *
 * 本文件提供 AD9959 DDS 芯片的完整驱动，包括：
 * - 基于基地模板固定引脚的 GPIO 模拟串行寄存器写入；
 * - 频率、相位、幅度的寄存器级暂存与批量提交 (IO_UPDATE)；
 * - 高级接口：单通道输出、多通道同步、校准对输出等；
 * - 兼容旧版模板的包装函数。
 *
 * 硬件连接严格使用模板：
 *   PE5=CS，PE3=SCLK，PB0=IO_UPDATE，PF2=RESET；
 *   PE4=SDIO0，PE2/PE0/PE1=SDIO1/2/3；
 *   PF4=PDC，PF5/PF3/PF1/PE6=PROFILE0/1/2/3。
 */

#include "Drive_AD9959.h"

#include <stddef.h>
#include <string.h>

#include "../../SYSTEM/delay/delay.h"

/* -------------------------------------------------------------------------- */
/*  硬件参数与寄存器常量                                                      */
/* -------------------------------------------------------------------------- */

/** FTW 计算公式中的 2^32 缩放因子 */
#define AD9959_FTW_SCALE_U64          4294967296ULL
/** 频率单位转换：1 Hz = 1000 mHz */
#define AD9959_MILLI_UNITS_PER_HZ     1000ULL
/** 相位单位转换：360° = 360000 毫度 */
#define AD9959_MILLIDEG_PER_TURN      360000UL

/** FR2[13] 自动清零相位累加器位，用于同步四通道相位起点 */
#define AD9959_FR2_AUTOCLEAR_PHASE    0x2000U
/** ACR 寄存器中使能幅度控制并设置 ASF 的控制字前缀 */
#define AD9959_ACR_AMPLITUDE_ENABLE   0x001000UL

/*
 * CFR 默认值 0x000301：
 * - CFR[9:8]=11：保持默认 DAC 满量程电流控制；
 * - CFR[2:1]=00：正常累加模式（不启动扫描/调制）；
 * - CFR[0]=1 ：正弦输出，与题目相位定义一致。
 */
#define AD9959_CFR_SINGLE_TONE_SINE   0x000301UL

/* -------------------------------------------------------------------------- */
/*  模块内部状态                                                              */
/* -------------------------------------------------------------------------- */

/** AD9959 全局状态，保存配置、系统时钟和各通道当前寄存器值 */
static AD9959_State_t s_ad9959;

/* -------------------------------------------------------------------------- */
/*  底层 GPIO 与模拟串行操作                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief 使能基地模板固定引脚所用 GPIOB/GPIOE/GPIOF 时钟。
 */
static void AD9959_EnableGpioClocks(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
}

/**
 * @brief 设置 SPI 片选信号 (CS)。
 * @param state GPIO 电平状态。
 */
static void AD9959_SetCs(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AD9959_CS_GPIO_PORT, AD9959_CS_GPIO_PIN, state);
}

/**
 * @brief 设置 IO_UPDATE 信号，用于将缓存寄存器内容传输至工作寄存器。
 * @param state GPIO 电平状态。
 */
static void AD9959_SetUpdate(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AD9959_UPDATE_GPIO_PORT,
                      AD9959_UPDATE_GPIO_PIN, state);
}

/**
 * @brief 设置 GPIO 模拟串行时钟。
 */
static void AD9959_SetSclk(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AD9959_SCLK_GPIO_PORT,
                      AD9959_SCLK_GPIO_PIN, state);
}

/**
 * @brief 设置单线串行数据 SDIO0。
 */
static void AD9959_SetSdio0(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AD9959_SDIO0_GPIO_PORT,
                      AD9959_SDIO0_GPIO_PIN, state);
}

/**
 * @brief 设置硬件复位信号 (MASTER_RESET)。
 * @param state GPIO 电平状态。
 */
static void AD9959_SetReset(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AD9959_RESET_GPIO_PORT,
                      AD9959_RESET_GPIO_PIN, state);
}

/**
 * @brief 设置PDC/PWR引脚，低电平为正常工作。
 */
static void AD9959_SetPowerDown(GPIO_PinState state)
{
    HAL_GPIO_WritePin(AD9959_POWER_GPIO_PORT,
                      AD9959_POWER_GPIO_PIN, state);
}

/**
 * @brief 按基地模板初始化全部AD9959固定引脚为推挽输出。
 */
static void AD9959_GpioInit(void)
{
    GPIO_InitTypeDef gpio;

    AD9959_EnableGpioClocks();

    /*
     * 先预置ODR，再配置为输出，避免CS/RESET/IO_UPDATE产生毛刺。
     * GPIOE的PE0~PE6严格对应模板SDIO2/3/1/SCLK/SDIO0/CS/PROFILE3。
     */
    HAL_GPIO_WritePin(
        GPIOE,
        AD9959_SDIO2_GPIO_PIN | AD9959_SDIO3_GPIO_PIN |
        AD9959_SDIO1_GPIO_PIN | AD9959_SCLK_GPIO_PIN |
        AD9959_SDIO0_GPIO_PIN | AD9959_CS_GPIO_PIN |
        AD9959_PROFILE3_GPIO_PIN,
        GPIO_PIN_RESET);
    AD9959_SetCs(GPIO_PIN_SET);

    /* GPIOF的PF1~PF5严格对应PROFILE2/RESET/PROFILE1/PDC/PROFILE0。 */
    HAL_GPIO_WritePin(
        GPIOF,
        AD9959_PROFILE2_GPIO_PIN | AD9959_RESET_GPIO_PIN |
        AD9959_PROFILE1_GPIO_PIN | AD9959_POWER_GPIO_PIN |
        AD9959_PROFILE0_GPIO_PIN,
        GPIO_PIN_RESET);
    AD9959_SetUpdate(GPIO_PIN_RESET);

    gpio.Pin = AD9959_SDIO2_GPIO_PIN |
               AD9959_SDIO3_GPIO_PIN |
               AD9959_SDIO1_GPIO_PIN |
               AD9959_SCLK_GPIO_PIN |
               AD9959_SDIO0_GPIO_PIN |
               AD9959_CS_GPIO_PIN |
               AD9959_PROFILE3_GPIO_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = 0U;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = AD9959_PROFILE2_GPIO_PIN |
               AD9959_RESET_GPIO_PIN |
               AD9959_PROFILE1_GPIO_PIN |
               AD9959_POWER_GPIO_PIN |
               AD9959_PROFILE0_GPIO_PIN;
    HAL_GPIO_Init(GPIOF, &gpio);

    gpio.Pin = AD9959_UPDATE_GPIO_PIN;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* 与模板Intserve()一致：PDC、Profile、SDIO1~3、SCLK均保持低。 */
    AD9959_SetPowerDown(GPIO_PIN_RESET);
    AD9959_SetReset(GPIO_PIN_RESET);
    AD9959_SetSclk(GPIO_PIN_RESET);
    AD9959_SetSdio0(GPIO_PIN_RESET);
}

/**
 * @brief 产生一次硬件复位脉冲 (MASTER_RESET)。
 *        时序：保持 RESET 低 -> 拉高 10us -> 拉低。
 */
static void AD9959_ResetPulse(void)
{
    AD9959_SetReset(GPIO_PIN_RESET);
    delay_us(2U);
    AD9959_SetReset(GPIO_PIN_SET);
    delay_us(10U);
    AD9959_SetReset(GPIO_PIN_RESET);
    delay_us(10U);
}

/**
 * @brief 产生一次 IO_UPDATE 脉冲，将缓存寄存器内容生效。
 *        要求高电平宽度 > 1 个 SYNC_CLK 周期（这里 1us 提供充足余量）。
 */
static void AD9959_UpdatePulse(void)
{
    /*
     * 数据手册要求 IO_UPDATE 高电平宽度大于一个 SYNC_CLK 周期。
     * 1 us 对 25 MHz 或 500 MHz 系统时钟都保留了充足余量。
     */
    AD9959_SetUpdate(GPIO_PIN_RESET);
    AD9959_SetUpdate(GPIO_PIN_SET);
    delay_us(1U);
    AD9959_SetUpdate(GPIO_PIN_RESET);
}

/**
 * @brief 将逻辑通道号转换为 CSR 通道选择掩码。
 * @param channel 通道号 (0~3) 或 AD9959_CHANNEL_ALL (0xF0)。
 * @return CSR[7:4] 对应位掩码，错误时返回 0。
 */
static uint8_t AD9959_ChannelMask(uint8_t channel)
{
    if (channel < AD9959_CHANNEL_COUNT)
    {
        return (uint8_t)(0x10U << channel);
    }
    if (channel == AD9959_CHANNEL_ALL)
    {
        return 0xF0U;
    }
    return 0U;
}

/**
 * @brief 按模板时序通过SDIO0写入一个字节，MSB先行。
 *
 * AD9959在SCLK上升沿采样。两次HAL_GPIO_WritePin调用之间的执行时间
 * 已大于数据手册要求的最小时钟高低电平宽度，不额外插入阻塞延时。
 */
static void AD9959_WriteByte(uint8_t value)
{
    uint8_t bit;

    for (bit = 0U; bit < 8U; ++bit)
    {
        AD9959_SetSclk(GPIO_PIN_RESET);
        AD9959_SetSdio0(
            ((value & 0x80U) != 0U) ?
            GPIO_PIN_SET : GPIO_PIN_RESET);
        AD9959_SetSclk(GPIO_PIN_SET);
        value <<= 1U;
    }
    AD9959_SetSclk(GPIO_PIN_RESET);
}

/**
 * @brief 通过GPIO模拟串行向指定寄存器地址写入数据。
 * @param register_address 寄存器地址 (7 位，最高位为 0 表示写)。
 * @param data 数据缓冲区。
 * @param length 数据长度 (1~4 字节)。
 * @return HAL_OK 成功，HAL_ERROR 参数/通信错误。
 */
static HAL_StatusTypeDef AD9959_WriteRegisterInternal(
    uint8_t register_address,
    const uint8_t *data,
    uint8_t length)
{
    uint8_t index;

    if ((data == NULL) || (length == 0U) || (length > 4U) ||
        (register_address > AD9959_REG_FDW))
    {
        return HAL_ERROR;
    }

    /* SCLK空闲低、MSB先行、上升沿采样，与基地模板完全一致。 */
    AD9959_SetSclk(GPIO_PIN_RESET);
    AD9959_SetCs(GPIO_PIN_RESET);
    AD9959_WriteByte((uint8_t)(register_address & 0x7FU));
    for (index = 0U; index < length; ++index)
    {
        AD9959_WriteByte(data[index]);
    }
    AD9959_SetSdio0(GPIO_PIN_RESET);
    AD9959_SetCs(GPIO_PIN_SET);

    return HAL_OK;
}

/**
 * @brief 选择当前操作的通道（写 CSR 寄存器，无需 IO_UPDATE）。
 * @param channel 通道号或 ALL。
 * @return HAL_OK 成功，HAL_ERROR 无效通道。
 */
static HAL_StatusTypeDef AD9959_SelectChannels(uint8_t channel)
{
    uint8_t csr;

    csr = AD9959_ChannelMask(channel);
    if (csr == 0U)
    {
        return HAL_ERROR;
    }

    /*
     * CSR[7:4] 通道选择位在字节写完后立即生效，不需要 IO_UPDATE。
     */
    return AD9959_WriteRegisterInternal(AD9959_REG_CSR, &csr, 1U);
}

/* 以下为不同位宽寄存器的便捷写入函数 */

static HAL_StatusTypeDef AD9959_WriteU16(uint8_t register_address,
                                        uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
    return AD9959_WriteRegisterInternal(register_address, data, 2U);
}

static HAL_StatusTypeDef AD9959_WriteU24(uint8_t register_address,
                                        uint32_t value)
{
    uint8_t data[3];

    data[0] = (uint8_t)(value >> 16U);
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)value;
    return AD9959_WriteRegisterInternal(register_address, data, 3U);
}

static HAL_StatusTypeDef AD9959_WriteU32(uint8_t register_address,
                                        uint32_t value)
{
    uint8_t data[4];

    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
    return AD9959_WriteRegisterInternal(register_address, data, 4U);
}

/**
 * @brief 校验配置参数合法性，并计算实际系统时钟。
 * @param config 用户配置。
 * @param[out] system_clock_hz 计算出的系统时钟 (Hz)。
 * @return 1 合法，0 非法。
 */
static uint8_t AD9959_ConfigIsValid(const AD9959_Config_t *config,
                                    uint32_t *system_clock_hz)
{
    uint64_t clock_hz;

    if ((config == NULL) || (system_clock_hz == NULL) ||
        (config->reference_clock_hz == 0U) ||
        (config->full_scale_mv <= 0.0f))
    {
        return 0U;
    }

    if (config->pll_multiplier == 0U)
    {
        /* 不使用 PLL，系统时钟等于参考时钟 */
        clock_hz = config->reference_clock_hz;
    }
    else
    {
        if ((config->pll_multiplier < 4U) ||
            (config->pll_multiplier > 20U))
        {
            return 0U;
        }
        clock_hz = (uint64_t)config->reference_clock_hz *
                   config->pll_multiplier;
        if ((clock_hz < 100000000ULL) || (clock_hz > 500000000ULL))
        {
            return 0U;
        }
    }

    /* 奈奎斯特约束：最高输出频率需小于系统时钟的一半 */
    if ((clock_hz == 0ULL) || (clock_hz > 500000000ULL) ||
        ((uint64_t)AD9959_PROJECT_MAX_FREQUENCY_HZ * 2ULL >= clock_hz))
    {
        return 0U;
    }

    *system_clock_hz = (uint32_t)clock_hz;
    return 1U;
}

/**
 * @brief 根据配置和系统时钟构建 FR1 寄存器值。
 *        包含 PLL 倍频系数和 VCO 范围选择。
 * @param config 配置。
 * @param system_clock_hz 系统时钟。
 * @return FR1 寄存器值。
 */
static uint32_t AD9959_BuildFr1(const AD9959_Config_t *config,
                               uint32_t system_clock_hz)
{
    uint32_t fr1 = 0U;

    if (config->pll_multiplier != 0U)
    {
        fr1 |= ((uint32_t)config->pll_multiplier << 18U);
        if (system_clock_hz > 255000000U)
        {
            fr1 |= (1UL << 23U);  // VCO 范围选择位
        }
    }
    return fr1;
}

/**
 * @brief 暂存幅度寄存器 (ACR) 并更新本地状态，但不触发 IO_UPDATE。
 * @param channel 通道号 (0~3)。
 * @param asf 幅度缩放因子 (0~1023)。
 * @param output 0 表示关闭输出（强制 ASF=0），非 0 正常。
 * @return HAL_OK/HAL_ERROR。
 */
static HAL_StatusTypeDef AD9959_StageAmplitudeRaw(uint8_t channel,
                                                  uint16_t asf,
                                                  uint8_t output)
{
    uint32_t acr;

    if ((channel >= AD9959_CHANNEL_COUNT) || (asf > AD9959_ASF_MAX))
    {
        return HAL_ERROR;
    }
    if (output == 0U)
    {
        asf = 0U;   // 关闭输出时强制幅度为 0
    }

    if (AD9959_SelectChannels(channel) != HAL_OK)
    {
        return HAL_ERROR;
    }

    acr = AD9959_ACR_AMPLITUDE_ENABLE | asf;
    if (AD9959_WriteU24(AD9959_REG_ACR, acr) != HAL_OK)
    {
        return HAL_ERROR;
    }

    s_ad9959.channel[channel].asf = asf;
    s_ad9959.channel[channel].output_enabled =
        ((output != 0U) && (asf != 0U)) ? 1U : 0U;
    return HAL_OK;
}

/**
 * @brief 暂存频率字 (FTW) 并更新本地状态，不触发 IO_UPDATE。
 * @param channel 通道号。
 * @param ftw 频率调谐字。
 * @return HAL_OK/HAL_ERROR。
 */
static HAL_StatusTypeDef AD9959_StageFrequencyRaw(uint8_t channel,
                                                  uint32_t ftw)
{
    if (channel >= AD9959_CHANNEL_COUNT)
    {
        return HAL_ERROR;
    }
    if (AD9959_SelectChannels(channel) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (AD9959_WriteU32(AD9959_REG_CFTW0, ftw) != HAL_OK)
    {
        return HAL_ERROR;
    }
    s_ad9959.channel[channel].ftw = ftw;
    return HAL_OK;
}

/**
 * @brief 暂存相位偏移字 (POW) 并更新本地状态，不触发 IO_UPDATE。
 * @param channel 通道号。
 * @param pow 相位偏移字 (0~16383)。
 * @return HAL_OK/HAL_ERROR。
 */
static HAL_StatusTypeDef AD9959_StagePhaseRaw(uint8_t channel,
                                              uint16_t pow)
{
    if ((channel >= AD9959_CHANNEL_COUNT) || (pow > AD9959_POW_MAX))
    {
        return HAL_ERROR;
    }
    if (AD9959_SelectChannels(channel) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (AD9959_WriteU16(AD9959_REG_CPOW0, pow) != HAL_OK)
    {
        return HAL_ERROR;
    }
    s_ad9959.channel[channel].pow = pow;
    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/*  公共 API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief 使用默认配置初始化 AD9959。
 * @return HAL_OK 成功，否则失败。
 */
HAL_StatusTypeDef AD9959_Init(void)
{
    AD9959_Config_t config;

    config.reference_clock_hz = AD9959_DEFAULT_REFERENCE_CLOCK_HZ;
    config.pll_multiplier = AD9959_DEFAULT_PLL_MULTIPLIER;
    config.full_scale_mv = AD9959_DEFAULT_FULL_SCALE_MV;
    return AD9959_InitWithConfig(&config);
}

/**
 * @brief 根据自定义配置初始化 AD9959。
 *        完成固定GPIO、复位、PLL配置和默认寄存器设置。
 *        若已初始化则直接返回 HAL_OK。
 * @param config 配置参数结构体指针。
 * @return HAL_OK 成功，否则失败。
 */
HAL_StatusTypeDef AD9959_InitWithConfig(const AD9959_Config_t *config)
{
    uint32_t system_clock_hz;
    uint32_t fr1;
    uint8_t channel;

    if (s_ad9959.initialized != 0U)
    {
        return HAL_OK;
    }
    if (AD9959_ConfigIsValid(config, &system_clock_hz) == 0U)
    {
        return HAL_ERROR;
    }

    /* 基地模板固定GPIO初始化；本驱动不使用硬件SPI3。 */
    AD9959_GpioInit();
    AD9959_ResetPulse();

    (void)memset(&s_ad9959, 0, sizeof(s_ad9959));
    s_ad9959.config = *config;
    s_ad9959.system_clock_hz = system_clock_hz;

    /* 配置 FR1，使能 PLL 及 VCO 范围 */
    fr1 = AD9959_BuildFr1(config, system_clock_hz);
    if (AD9959_WriteU24(AD9959_REG_FR1, fr1) != HAL_OK)
    {
        return HAL_ERROR;
    }
    AD9959_UpdatePulse();

    if (config->pll_multiplier != 0U)
    {
        /* 数据手册给出的 PLL 典型锁定时间约 1 ms，等待 2 ms 确保稳定。 */
        HAL_Delay(2U);
    }

    /* 所有通道配置为单音正弦输出模式 */
    if (AD9959_SelectChannels(AD9959_CHANNEL_ALL) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (AD9959_WriteU24(AD9959_REG_CFR,
                        AD9959_CFR_SINGLE_TONE_SINE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 初始化所有通道：频率 0，相位 0，输出关闭 */
    for (channel = 0U; channel < AD9959_CHANNEL_COUNT; ++channel)
    {
        if ((AD9959_StageFrequencyRaw(channel, 0U) != HAL_OK) ||
            (AD9959_StagePhaseRaw(channel, 0U) != HAL_OK) ||
            (AD9959_StageAmplitudeRaw(channel, 0U, 0U) != HAL_OK))
        {
            return HAL_ERROR;
        }
    }
    AD9959_UpdatePulse();
    s_ad9959.initialized = 1U;
    return HAL_OK;
}

/**
 * @brief 获取 AD9959 内部状态指针（只读）。
 * @return 状态结构指针。
 */
const AD9959_State_t *AD9959_GetState(void)
{
    return &s_ad9959;
}

/**
 * @brief 设置满量程输出电压 (mVpp)，用于后续幅度转换。
 * @param full_scale_mv 满量程值，必须 > 0。
 * @return HAL_OK 成功，HAL_ERROR 无效值。
 */
HAL_StatusTypeDef AD9959_SetFullScaleMilliVolt(float full_scale_mv)
{
    if (full_scale_mv <= 0.0f)
    {
        return HAL_ERROR;
    }
    s_ad9959.config.full_scale_mv = full_scale_mv;
    return HAL_OK;
}

/**
 * @brief 将毫赫兹频率值转换为 32 位频率调谐字 (FTW)。
 *        FTW = (freq_mHz * 2^32) / (SysClk_Hz * 1000)，结果四舍五入。
 * @param frequency_millihz 频率 (mHz)。
 * @return 32 位 FTW。
 */
uint32_t AD9959_FrequencyMilliHzToFtw(uint32_t frequency_millihz)
{
    uint64_t numerator;
    uint64_t denominator;
    uint32_t system_clock_hz = s_ad9959.system_clock_hz;

    if (system_clock_hz == 0U)
    {
        system_clock_hz = AD9959_DEFAULT_REFERENCE_CLOCK_HZ;
    }

    denominator = (uint64_t)system_clock_hz *
                  AD9959_MILLI_UNITS_PER_HZ;
    numerator = (uint64_t)frequency_millihz *
                AD9959_FTW_SCALE_U64;

    /* 四舍五入 */
    return (uint32_t)((numerator + denominator / 2ULL) / denominator);
}

/**
 * @brief 将毫度相位值转换为 14 位相位偏移字 (POW)。
 *        POW = (phase_millideg * 16384) / 360000，结果四舍五入。
 * @param phase_millideg 相位 (毫度)。
 * @return 14 位 POW (0~16383)。
 */
uint16_t AD9959_PhaseMilliDegreeToPow(uint32_t phase_millideg)
{
    uint64_t numerator;

    phase_millideg %= AD9959_MILLIDEG_PER_TURN;
    numerator = (uint64_t)phase_millideg * 16384ULL +
                (AD9959_MILLIDEG_PER_TURN / 2UL);
    return (uint16_t)((numerator / AD9959_MILLIDEG_PER_TURN) &
                      AD9959_POW_MAX);
}

/**
 * @brief 将毫伏幅度转换为幅度缩放因子 (ASF)。
 *        线性映射：ASF = (amplitude_mV / full_scale_mV) * 1023，并钳位。
 * @param amplitude_mv 幅度 (mVpp)。
 * @return 10 位 ASF (0~1023)。
 */
uint16_t AD9959_AmplitudeMilliVoltToAsf(float amplitude_mv)
{
    float full_scale_mv = s_ad9959.config.full_scale_mv;
    float scaled;

    if (full_scale_mv <= 0.0f)
    {
        full_scale_mv = AD9959_DEFAULT_FULL_SCALE_MV;
    }
    if (amplitude_mv <= 0.0f)
    {
        return 0U;
    }
    if (amplitude_mv >= full_scale_mv)
    {
        return AD9959_ASF_MAX;
    }

    scaled = amplitude_mv * (float)AD9959_ASF_MAX / full_scale_mv;
    return (uint16_t)(scaled + 0.5f);
}

/**
 * @brief 直接向 AD9959 寄存器写入数据（不封装通道选择，需自行控制）。
 * @param register_address 寄存器地址。
 * @param data 数据缓冲区。
 * @param length 数据长度 (字节)。
 * @return HAL_OK 成功，HAL_ERROR 未初始化或通信失败。
 */
HAL_StatusTypeDef AD9959_WriteRegister(uint8_t register_address,
                                       const uint8_t *data,
                                       uint8_t length)
{
    if (s_ad9959.initialized == 0U)
    {
        return HAL_ERROR;
    }
    return AD9959_WriteRegisterInternal(register_address, data, length);
}

/**
 * @brief 暂存单个通道的原始参数（FTW, POW, ASF），不触发 IO_UPDATE。
 *        可用于原子更新多个通道后再统一提交。
 * @param channel 通道号。
 * @param ftw 频率调谐字。
 * @param pow 相位偏移字。
 * @param asf 幅度因子。
 * @param output 非 0 开启输出，0 关闭。
 * @return HAL_OK 成功，否则失败。
 */
HAL_StatusTypeDef AD9959_StageChannelRaw(uint8_t channel,
                                        uint32_t ftw,
                                        uint16_t pow,
                                        uint16_t asf,
                                        uint8_t output)
{
    if ((s_ad9959.initialized == 0U) ||
        (channel >= AD9959_CHANNEL_COUNT))
    {
        return HAL_ERROR;
    }

    if ((AD9959_StageFrequencyRaw(channel, ftw) != HAL_OK) ||
        (AD9959_StagePhaseRaw(channel, pow) != HAL_OK) ||
        (AD9959_StageAmplitudeRaw(channel, asf, output) != HAL_OK))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief 触发 IO_UPDATE，将所有暂存寄存器值传输至工作寄存器。
 * @return HAL_OK 成功，HAL_ERROR 未初始化。
 */
HAL_StatusTypeDef AD9959_Commit(void)
{
    if (s_ad9959.initialized == 0U)
    {
        return HAL_ERROR;
    }
    AD9959_UpdatePulse();
    return HAL_OK;
}

/**
 * @brief 只设置单通道原始频率字并立即生效。
 *
 * 该函数不重写CPOW0和ACR，也不设置相位累加器清零位，因此适合在
 * 锁频过程中连续切换相邻FTW。
 *
 * @param channel 通道号。
 * @param ftw 32位频率调谐字。
 * @return HAL_OK 成功，否则失败。
 */
HAL_StatusTypeDef AD9959_SetChannelFrequencyRaw(uint8_t channel,
                                                uint32_t ftw)
{
    HAL_StatusTypeDef status;

    if (s_ad9959.initialized == 0U)
    {
        return HAL_ERROR;
    }

    status = AD9959_StageFrequencyRaw(channel, ftw);
    if (status == HAL_OK)
    {
        status = AD9959_Commit();
    }
    return status;
}

/**
 * @brief 设置单通道原始参数并立即生效（包含 IO_UPDATE）。
 * @param channel 通道号。
 * @param ftw 频率调谐字。
 * @param pow 相位偏移字。
 * @param asf 幅度因子。
 * @param output 非 0 开启输出，0 关闭。
 * @return HAL_OK 成功，否则失败。
 */
HAL_StatusTypeDef AD9959_SetChannelRaw(uint8_t channel,
                                      uint32_t ftw,
                                      uint16_t pow,
                                      uint16_t asf,
                                      uint8_t output)
{
    HAL_StatusTypeDef status;

    status = AD9959_StageChannelRaw(channel, ftw, pow, asf, output);
    if (status == HAL_OK)
    {
        status = AD9959_Commit();
    }
    return status;
}

/**
 * @brief 高精度设置单通道输出（单位：mHz, mVpp, 毫度）。
 * @param channel 通道号。
 * @param frequency_millihz 频率 (mHz)。
 * @param amplitude_mv 幅度 (mVpp)。
 * @param phase_millideg 相位 (毫度)。
 * @param output 非 0 开启输出。
 * @return HAL_OK 成功。
 */
HAL_StatusTypeDef AD9959_SetChannelOutputPrecise(
    uint8_t channel,
    uint32_t frequency_millihz,
    float amplitude_mv,
    uint32_t phase_millideg,
    uint8_t output)
{
    uint32_t max_frequency_millihz =
        AD9959_PROJECT_MAX_FREQUENCY_HZ * 1000UL;

    if ((channel >= AD9959_CHANNEL_COUNT) ||
        (frequency_millihz > max_frequency_millihz))
    {
        return HAL_ERROR;
    }

    return AD9959_SetChannelRaw(
        channel,
        AD9959_FrequencyMilliHzToFtw(frequency_millihz),
        AD9959_PhaseMilliDegreeToPow(phase_millideg),
        AD9959_AmplitudeMilliVoltToAsf(amplitude_mv),
        output);
}

/**
 * @brief 常用精度设置单通道输出（单位：Hz, mVpp, 度）。
 *        内部转换为毫单位后调用高精度版本。
 * @param channel 通道号。
 * @param frequency_hz 频率 (Hz)，范围 1..项目最大值。
 * @param amplitude_mv 幅度 (mVpp)。
 * @param phase_deg 相位 (度)。
 * @param output 非 0 开启。
 * @return HAL_OK 成功。
 */
HAL_StatusTypeDef AD9959_SetChannelOutput(uint8_t channel,
                                         uint32_t frequency_hz,
                                         float amplitude_mv,
                                         uint32_t phase_deg,
                                         uint8_t output)
{
    if (frequency_hz == 0U)
    {
        frequency_hz = 1U;   // 最小 1 Hz
    }
    if (frequency_hz > AD9959_PROJECT_MAX_FREQUENCY_HZ)
    {
        return HAL_ERROR;
    }

    return AD9959_SetChannelOutputPrecise(
        channel,
        frequency_hz * 1000UL,
        amplitude_mv,
        (phase_deg % 360UL) * 1000UL,
        output);
}

/**
 * @brief 设置所有通道为相同参数（暂存+提交）。
 * @param frequency_hz 频率 (Hz)。
 * @param amplitude_mv 幅度 (mVpp)。
 * @param phase_deg 相位 (度)。
 * @param output 输出使能。
 * @return HAL_OK 成功。
 */
HAL_StatusTypeDef AD9959_SetAllChannelsSame(uint32_t frequency_hz,
                                           float amplitude_mv,
                                           uint32_t phase_deg,
                                           uint8_t output)
{
    uint32_t ftw;
    uint16_t pow;
    uint16_t asf;
    uint8_t channel;

    if (s_ad9959.initialized == 0U)
    {
        return HAL_ERROR;
    }
    if (frequency_hz == 0U)
    {
        frequency_hz = 1U;
    }
    if (frequency_hz > AD9959_PROJECT_MAX_FREQUENCY_HZ)
    {
        return HAL_ERROR;
    }

    ftw = AD9959_FrequencyMilliHzToFtw(frequency_hz * 1000UL);
    pow = AD9959_PhaseMilliDegreeToPow(
        (phase_deg % 360UL) * 1000UL);
    asf = AD9959_AmplitudeMilliVoltToAsf(amplitude_mv);

    for (channel = 0U; channel < AD9959_CHANNEL_COUNT; ++channel)
    {
        if (AD9959_StageChannelRaw(channel, ftw, pow, asf,
                                   output) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    return AD9959_Commit();
}

/**
 * @brief 设置输出（别名，等同于 AD9959_SetAllChannelsSame）。
 * @param frequency_hz 频率 (Hz)。
 * @param amplitude_mv 幅度 (mVpp)。
 * @param phase_deg 相位 (度)。
 * @param output 输出使能。
 * @return HAL_OK 成功。
 */
HAL_StatusTypeDef AD9959_SetOutput(uint32_t frequency_hz,
                                  float amplitude_mv,
                                  uint32_t phase_deg,
                                  uint8_t output)
{
    return AD9959_SetAllChannelsSame(frequency_hz, amplitude_mv,
                                     phase_deg, output);
}

/**
 * @brief 设置校准对输出：通道0和通道1分别设不同相位，通道2/3关闭。
 *        用于有线测量中产生特定相位关系的两路信号。
 * @param frequency_hz 公共频率 (Hz)。
 * @param amplitude_mv 公共幅度 (mVpp)。
 * @param channel1_phase_deg 通道1相位 (度)。
 * @param channel0_phase_deg 通道0相位 (度)。
 * @return HAL_OK 成功。
 */
HAL_StatusTypeDef AD9959_SetCalibrationPair(
    uint32_t frequency_hz,
    float amplitude_mv,
    uint32_t channel1_phase_deg,
    uint32_t channel0_phase_deg)
{
    uint32_t ftw;
    uint16_t asf;

    if ((s_ad9959.initialized == 0U) ||
        (frequency_hz == 0U) ||
        (frequency_hz > AD9959_PROJECT_MAX_FREQUENCY_HZ))
    {
        return HAL_ERROR;
    }

    ftw = AD9959_FrequencyMilliHzToFtw(frequency_hz * 1000UL);
    asf = AD9959_AmplitudeMilliVoltToAsf(amplitude_mv);

    if (AD9959_StageChannelRaw(
            0U, ftw,
            AD9959_PhaseMilliDegreeToPow(
                (channel0_phase_deg % 360UL) * 1000UL),
            asf, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (AD9959_StageChannelRaw(
            1U, ftw,
            AD9959_PhaseMilliDegreeToPow(
                (channel1_phase_deg % 360UL) * 1000UL),
            asf, 1U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    /* 关闭未使用的通道 */
    if ((AD9959_StageChannelRaw(2U, ftw, 0U, 0U, 0U) != HAL_OK) ||
        (AD9959_StageChannelRaw(3U, ftw, 0U, 0U, 0U) != HAL_OK))
    {
        return HAL_ERROR;
    }
    return AD9959_Commit();
}

/**
 * @brief 停止所有通道输出（幅度设为零并提交）。
 * @return HAL_OK 成功。
 */
HAL_StatusTypeDef AD9959_StopOutput(void)
{
    uint8_t channel;

    if (s_ad9959.initialized == 0U)
    {
        return HAL_ERROR;
    }

    for (channel = 0U; channel < AD9959_CHANNEL_COUNT; ++channel)
    {
        if (AD9959_StageAmplitudeRaw(channel, 0U, 0U) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    return AD9959_Commit();
}

/**
 * @brief 同步所有通道的相位累加器起点。
 *        通过设置 FR2[13] 自动清零相位累加器一拍，然后清除该位。
 *        用于消除通道间随机相位偏移，建立确定的相位关系。
 * @return HAL_OK 成功。
 */
HAL_StatusTypeDef AD9959_SyncPhaseAll(void)
{
    if (s_ad9959.initialized == 0U)
    {
        return HAL_ERROR;
    }

    /*
     * FR2[13]=1：下一次 IO_UPDATE 时，四个相位累加器同步清零一拍。
     * 随后清除该控制位，避免以后每次更新都重新起相。
     */
    if (AD9959_WriteU16(AD9959_REG_FR2,
                        AD9959_FR2_AUTOCLEAR_PHASE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    AD9959_UpdatePulse();

    if (AD9959_WriteU16(AD9959_REG_FR2, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }
    AD9959_UpdatePulse();
    return HAL_OK;
}

/* -------------------------------------------------------------------------- */
/*  模板兼容层（供旧代码调用）                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief 旧版初始化函数，映射至 AD9959_Init()。
 */
void Init_AD9959(void)
{
    (void)AD9959_Init();
}

/**
 * @brief 旧版 DDS 数据初始化，映射至 AD9959_Init()。
 */
void DDSDataInit(void)
{
    (void)AD9959_Init();
}

/**
 * @brief 旧版硬件复位，调用后需重新初始化才能使用。
 */
void IntReset(void)
{
    if (s_ad9959.initialized != 0U)
    {
        AD9959_ResetPulse();
        s_ad9959.initialized = 0U;
    }
}

/**
 * @brief 旧版 IO_UPDATE 触发。
 */
void IO_Update(void)
{
    if (s_ad9959.initialized != 0U)
    {
        AD9959_UpdatePulse();
    }
}

/**
 * @brief 旧版 GPIO 初始化（仅复位引脚，实际已无用）。
 */
void Intserve(void)
{
    AD9959_GpioInit();
}

/**
 * @brief 旧版写入寄存器函数，根据 update 参数决定是否提交。
 * @param register_address 寄存器地址。
 * @param number_of_registers 字节数。
 * @param register_data 数据缓冲。
 * @param update 非 0 则调用 IO_UPDATE。
 */
void WriteData_AD9959(uint8_t register_address,
                      uint8_t number_of_registers,
                      uint8_t *register_data,
                      uint8_t update)
{
    if (AD9959_WriteRegister(register_address, register_data,
                             number_of_registers) == HAL_OK)
    {
        if (update != 0U)
        {
            (void)AD9959_Commit();
        }
    }
}

/**
 * @brief 旧版频率暂存（不更新），频率钳位至项目最大值。
 */
void Write_frequence_no_update(uint8_t channel, uint32_t frequency_hz)
{
    uint32_t max_frequency_millihz =
        AD9959_PROJECT_MAX_FREQUENCY_HZ * 1000UL;
    uint32_t frequency_millihz;

    if ((s_ad9959.initialized == 0U) ||
        (channel >= AD9959_CHANNEL_COUNT))
    {
        return;
    }
    frequency_millihz = frequency_hz * 1000UL;
    if (frequency_millihz > max_frequency_millihz)
    {
        frequency_millihz = max_frequency_millihz;
    }
    (void)AD9959_StageFrequencyRaw(
        channel, AD9959_FrequencyMilliHzToFtw(frequency_millihz));
}

/**
 * @brief 旧版频率设置并提交。
 */
void Write_frequence(uint8_t channel, uint32_t frequency_hz)
{
    Write_frequence_no_update(channel, frequency_hz);
    (void)AD9959_Commit();
}

/**
 * @brief 旧版幅度暂存（不更新），输入单位 mV。
 */
void Write_Amplitude_no_update(uint8_t channel, uint16_t amplitude_mv)
{
    if ((s_ad9959.initialized == 0U) ||
        (channel >= AD9959_CHANNEL_COUNT))
    {
        return;
    }
    (void)AD9959_StageAmplitudeRaw(
        channel, AD9959_AmplitudeMilliVoltToAsf((float)amplitude_mv), 1U);
}

/**
 * @brief 旧版幅度设置并提交。
 */
void Write_Amplitude(uint8_t channel, uint16_t amplitude_mv)
{
    Write_Amplitude_no_update(channel, amplitude_mv);
    (void)AD9959_Commit();
}

/**
 * @brief 旧版相位暂存（不更新），支持通道 ALL。
 */
void Write_Phase_no_update(uint8_t channel, uint16_t phase_deg)
{
    if (channel == AD9959_CHANNEL_ALL)
    {
        uint8_t index;
        uint16_t pow = AD9959_PhaseMilliDegreeToPow(
            ((uint32_t)phase_deg % 360UL) * 1000UL);

        for (index = 0U; index < AD9959_CHANNEL_COUNT; ++index)
        {
            (void)AD9959_StagePhaseRaw(index, pow);
        }
        return;
    }
    if ((s_ad9959.initialized == 0U) ||
        (channel >= AD9959_CHANNEL_COUNT))
    {
        return;
    }
    (void)AD9959_StagePhaseRaw(
        channel,
        AD9959_PhaseMilliDegreeToPow(
            ((uint32_t)phase_deg % 360UL) * 1000UL));
}

/**
 * @brief 旧版相位设置并提交。
 */
void Write_Phase(uint8_t channel, uint16_t phase_deg)
{
    Write_Phase_no_update(channel, phase_deg);
    (void)AD9959_Commit();
}

/**
 * @brief 旧版相位同步函数别名。
 */
void AD9959_SyncPhase_All(void)
{
    (void)AD9959_SyncPhaseAll();
}
