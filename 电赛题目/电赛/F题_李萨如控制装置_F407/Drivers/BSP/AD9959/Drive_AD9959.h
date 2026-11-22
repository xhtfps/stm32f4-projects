/**
 * @file    Drive_AD9959.h
 * @brief   STM32F407 按基地模板固定引脚控制 AD9959。
 *
 * 设计目标：
 * 1. 要求2~4继续使用整数 Hz、mVpp、整数度的简洁接口；
 * 2. 要求5提供毫赫兹和毫度接口，避免光学锁频精度被整数类型截断；
 * 3. 支持先缓存多个通道参数，再用一次 IO_UPDATE 同步生效；
 * 4. 默认关闭 AD9959 内部 PLL，25 MHz 晶体直接作为系统时钟。
 *
 * 固定接线（严格对应基地标准库 Drive_AD9959.h，不允许自行更换）：
 *   PE5 -> CS             PE3 -> SCLK
 *   PB0 -> IO_UPDATE      PF2 -> RESET
 *   PE4 -> SDIO0          PE2 -> SDIO1
 *   PE0 -> SDIO2          PE1 -> SDIO3
 *   PF4 -> PDC/PWR
 *   PF5 -> PROFILE0       PF3 -> PROFILE1
 *   PF1 -> PROFILE2       PE6 -> PROFILE3
 *
 * SCLK/SDIO0不是同一组硬件SPI复用脚，因此驱动与模板一样使用GPIO
 * 模拟串行时序。当前采用单线串行模式，SDIO1~3保持低电平。
 */

#ifndef DRIVE_AD9959_H
#define DRIVE_AD9959_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* AD9959 寄存器地址。 */
#define AD9959_REG_CSR                 0x00U
#define AD9959_REG_FR1                 0x01U
#define AD9959_REG_FR2                 0x02U
#define AD9959_REG_CFR                 0x03U
#define AD9959_REG_CFTW0               0x04U
#define AD9959_REG_CPOW0               0x05U
#define AD9959_REG_ACR                 0x06U
#define AD9959_REG_LSRR                0x07U
#define AD9959_REG_RDW                 0x08U
#define AD9959_REG_FDW                 0x09U

#define AD9959_CHANNEL_COUNT           4U
#define AD9959_CHANNEL_ALL             4U
#define AD9959_FTW_BITS                32U
#define AD9959_POW_MAX                 0x3FFFU
#define AD9959_ASF_MAX                 0x03FFU

/*
 * F题最高输出为200 kHz（要求3：100 kHz输入的二倍频）。
 * 这里限制驱动的工程级输出范围，避免控制算法异常时输出过高频率。
 */
#define AD9959_PROJECT_MAX_FREQUENCY_HZ    200000U

/* 默认使用25 MHz晶体，PLL=0表示关闭内部倍频器。 */
#define AD9959_DEFAULT_REFERENCE_CLOCK_HZ  25000000U
#define AD9959_DEFAULT_PLL_MULTIPLIER       0U
#define AD9959_DEFAULT_FULL_SCALE_MV        500.0f

/*
 * 基地模板固定GPIO。这里故意不提供板级重映射选项，防止代码引脚与
 * 已焊接硬件不一致。
 */
#define AD9959_CS_GPIO_PORT            GPIOE
#define AD9959_CS_GPIO_PIN             GPIO_PIN_5
#define AD9959_SCLK_GPIO_PORT          GPIOE
#define AD9959_SCLK_GPIO_PIN           GPIO_PIN_3
#define AD9959_UPDATE_GPIO_PORT        GPIOB
#define AD9959_UPDATE_GPIO_PIN         GPIO_PIN_0

#define AD9959_PROFILE0_GPIO_PORT      GPIOF
#define AD9959_PROFILE0_GPIO_PIN       GPIO_PIN_5
#define AD9959_PROFILE1_GPIO_PORT      GPIOF
#define AD9959_PROFILE1_GPIO_PIN       GPIO_PIN_3
#define AD9959_PROFILE2_GPIO_PORT      GPIOF
#define AD9959_PROFILE2_GPIO_PIN       GPIO_PIN_1
#define AD9959_PROFILE3_GPIO_PORT      GPIOE
#define AD9959_PROFILE3_GPIO_PIN       GPIO_PIN_6

#define AD9959_SDIO0_GPIO_PORT         GPIOE
#define AD9959_SDIO0_GPIO_PIN          GPIO_PIN_4
#define AD9959_SDIO1_GPIO_PORT         GPIOE
#define AD9959_SDIO1_GPIO_PIN          GPIO_PIN_2
#define AD9959_SDIO2_GPIO_PORT         GPIOE
#define AD9959_SDIO2_GPIO_PIN          GPIO_PIN_0
#define AD9959_SDIO3_GPIO_PORT         GPIOE
#define AD9959_SDIO3_GPIO_PIN          GPIO_PIN_1

#define AD9959_POWER_GPIO_PORT         GPIOF
#define AD9959_POWER_GPIO_PIN          GPIO_PIN_4
#define AD9959_RESET_GPIO_PORT         GPIOF
#define AD9959_RESET_GPIO_PIN          GPIO_PIN_2

/**
 * @brief AD9959时钟和幅度换算配置。
 */
typedef struct
{
    uint32_t reference_clock_hz; /**< 外部晶体/时钟频率。 */
    uint8_t pll_multiplier;      /**< 0=关闭PLL；4~20=PLL倍频。 */
    float full_scale_mv;         /**< ASF=1023对应的实测DDS输出mVpp。 */
} AD9959_Config_t;

/**
 * @brief 驱动保存的通道寄存器镜像。
 */
typedef struct
{
    uint32_t ftw;
    uint16_t pow;
    uint16_t asf;
    uint8_t output_enabled;
} AD9959_ChannelState_t;

/**
 * @brief 驱动运行状态，只读查询使用。
 */
typedef struct
{
    AD9959_Config_t config;
    uint32_t system_clock_hz;
    AD9959_ChannelState_t channel[AD9959_CHANNEL_COUNT];
    uint8_t initialized;
} AD9959_State_t;

/** 使用F题默认配置初始化AD9959。 */
HAL_StatusTypeDef AD9959_Init(void);

/** 使用指定参考时钟、PLL和满量程参数初始化AD9959。 */
HAL_StatusTypeDef AD9959_InitWithConfig(const AD9959_Config_t *config);

/** 返回驱动状态镜像，不允许调用者修改。 */
const AD9959_State_t *AD9959_GetState(void);

/** 修改ASF满量程标定值，不会立即改变当前输出。 */
HAL_StatusTypeDef AD9959_SetFullScaleMilliVolt(float full_scale_mv);

/** 毫赫兹频率换算为32位FTW，采用四舍五入而不是向下截断。 */
uint32_t AD9959_FrequencyMilliHzToFtw(uint32_t frequency_millihz);

/** 毫度相位换算为14位CPOW，360000毫度等价于0度。 */
uint16_t AD9959_PhaseMilliDegreeToPow(uint32_t phase_millideg);

/** mVpp换算为10位ASF，超过满量程时自动限幅。 */
uint16_t AD9959_AmplitudeMilliVoltToAsf(float amplitude_mv);

/** 写AD9959寄存器；数据仅进入I/O缓冲区，不自动IO_UPDATE。 */
HAL_StatusTypeDef AD9959_WriteRegister(uint8_t register_address,
                                       const uint8_t *data,
                                       uint8_t length);

/**
 * @brief 缓存一个通道的原始FTW/CPOW/ASF，不立即改变模拟输出。
 */
HAL_StatusTypeDef AD9959_StageChannelRaw(uint8_t channel,
                                        uint32_t ftw,
                                        uint16_t pow,
                                        uint16_t asf,
                                        uint8_t output);

/** 产生一次IO_UPDATE，使此前缓存的所有通道寄存器同步生效。 */
HAL_StatusTypeDef AD9959_Commit(void);

/**
 * @brief 只更新一个通道的原始FTW并立即生效，不改相位和幅度。
 *
 * 供精细锁频和相邻FTW抖动使用。该接口不会清零相位累加器。
 */
HAL_StatusTypeDef AD9959_SetChannelFrequencyRaw(uint8_t channel,
                                                uint32_t ftw);

/** 设置一个通道的原始寄存器值并立即生效。 */
HAL_StatusTypeDef AD9959_SetChannelRaw(uint8_t channel,
                                      uint32_t ftw,
                                      uint16_t pow,
                                      uint16_t asf,
                                      uint8_t output);

/**
 * @brief 精密通道设置接口，供要求5光学锁频使用。
 *
 * @param frequency_millihz 频率，单位0.001 Hz。
 * @param phase_millideg    相位，单位0.001°。
 */
HAL_StatusTypeDef AD9959_SetChannelOutputPrecise(
    uint8_t channel,
    uint32_t frequency_millihz,
    float amplitude_mv,
    uint32_t phase_millideg,
    uint8_t output);

/** 整数Hz/整数度兼容接口，供要求2~4使用。 */
HAL_StatusTypeDef AD9959_SetChannelOutput(uint8_t channel,
                                         uint32_t frequency_hz,
                                         float amplitude_mv,
                                         uint32_t phase_deg,
                                         uint8_t output);

/** 四通道设置为相同参数，并通过一次IO_UPDATE同步生效。 */
HAL_StatusTypeDef AD9959_SetAllChannelsSame(uint32_t frequency_hz,
                                           float amplitude_mv,
                                           uint32_t phase_deg,
                                           uint8_t output);

/** AD9959_SetAllChannelsSame的兼容别名。 */
HAL_StatusTypeDef AD9959_SetOutput(uint32_t frequency_hz,
                                  float amplitude_mv,
                                  uint32_t phase_deg,
                                  uint8_t output);

/** 同步配置CH0/CH1校准输出，并关闭CH2/CH3。 */
HAL_StatusTypeDef AD9959_SetCalibrationPair(
    uint32_t frequency_hz,
    float amplitude_mv,
    uint32_t channel1_phase_deg,
    uint32_t channel0_phase_deg);

/** 将四个通道ASF同步置零。 */
HAL_StatusTypeDef AD9959_StopOutput(void);

/** 显式同步清零四通道相位累加器；普通调频时不会自动调用。 */
HAL_StatusTypeDef AD9959_SyncPhaseAll(void);

/* -------------------------------------------------------------------------
 * 模板兼容API。新业务代码优先使用上面的AD9959_xxx接口。
 * ------------------------------------------------------------------------- */
void Init_AD9959(void);
void DDSDataInit(void);
void IntReset(void);
void IO_Update(void);
void Intserve(void);
void WriteData_AD9959(uint8_t register_address,
                      uint8_t number_of_registers,
                      uint8_t *register_data,
                      uint8_t update);
void Write_frequence(uint8_t channel, uint32_t frequency_hz);
void Write_frequence_no_update(uint8_t channel, uint32_t frequency_hz);
void Write_Amplitude(uint8_t channel, uint16_t amplitude_mv);
void Write_Amplitude_no_update(uint8_t channel, uint16_t amplitude_mv);
void Write_Phase(uint8_t channel, uint16_t phase_deg);
void Write_Phase_no_update(uint8_t channel, uint16_t phase_deg);
void AD9959_SyncPhase_All(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVE_AD9959_H */
