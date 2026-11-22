#ifndef BSP_ADS1256_H
#define BSP_ADS1256_H

/**
 * @file    BSP_ADS1256.h
 * @brief   ADS1256 24位高精度ADC驱动头文件
 *
 * 提供 ADS1256 的寄存器地址、命令宏定义、通道选择、
 * 增益与数据率选项等常量和 API 函数声明。
 * 同时保留旧版兼容接口以支持现有应用代码。
 */

#include "stm32f4xx_hal.h"

/*
 * ADS1256 CLKIN frequency used to derive all CLKIN-dependent serial
 * timing. This must match the clock actually connected to XTAL1/CLKIN.
 * The board schematic uses a 7.68 MHz crystal on XTAL1/XTAL2, so the
 * project value below must remain 7680000UL unless the hardware is changed.
 */
#ifndef ADS1256_CLKIN_HZ
#define ADS1256_CLKIN_HZ 7680000UL
#endif

/* ================================================================
 * ADS1256 命令字节宏
 * ================================================================ */

#define ADS1256_CMD_WAKEUP   0x00U  /**< 唤醒命令 (退出待机) */
#define ADS1256_CMD_RDATA    0x01U  /**< 读取单次转换数据 */
#define ADS1256_CMD_RDATAC   0x03U  /**< 启动连续读取模式 */
#define ADS1256_CMD_SDATAC   0x0FU  /**< 停止连续读取模式 */
#define ADS1256_CMD_RREG     0x10U  /**< 读寄存器 (起始地址 ORed) */
#define ADS1256_CMD_WREG     0x50U  /**< 写寄存器 (起始地址 ORed) */
#define ADS1256_CMD_SELFCAL  0xF0U  /**< 自校准 (偏移+增益) */
#define ADS1256_CMD_SELFOCAL 0xF1U  /**< 自偏移校准 */
#define ADS1256_CMD_SELFGCAL 0xF2U  /**< 自增益校准 */
#define ADS1256_CMD_SYSOCAL  0xF3U  /**< 系统偏移校准 */
#define ADS1256_CMD_SYSGCAL  0xF4U  /**< 系统增益校准 */
#define ADS1256_CMD_SYNC     0xFCU  /**< 同步命令 */
#define ADS1256_CMD_STANDBY  0xFDU  /**< 待机命令 */
#define ADS1256_CMD_RESET    0xFEU  /**< 复位命令 */

/* ================================================================
 * 寄存器地址
 * ================================================================ */

#define ADS1256_STATUS 0x00U  /**< 状态寄存器 (缓冲使能、数据顺序等) */
#define ADS1256_MUX    0x01U  /**< 输入通道选择寄存器 */
#define ADS1256_ADCON  0x02U  /**< 控制寄存器 (增益、传感器检测) */
#define ADS1256_DRATE  0x03U  /**< 数据率选择寄存器 */
#define ADS1256_IO     0x04U  /**< 数字 I/O 控制寄存器 */

/* ================================================================
 * 正输入端通道选择 (MUX 高 4 位)
 * ================================================================ */

#define ADS1256_MUXP_AIN0   0x00U  /**< 正端 AIN0 */
#define ADS1256_MUXP_AIN1   0x10U  /**< 正端 AIN1 */
#define ADS1256_MUXP_AIN2   0x20U  /**< 正端 AIN2 */
#define ADS1256_MUXP_AIN3   0x30U  /**< 正端 AIN3 */
#define ADS1256_MUXP_AIN4   0x40U  /**< 正端 AIN4 */
#define ADS1256_MUXP_AIN5   0x50U  /**< 正端 AIN5 */
#define ADS1256_MUXP_AIN6   0x60U  /**< 正端 AIN6 */
#define ADS1256_MUXP_AIN7   0x70U  /**< 正端 AIN7 */
#define ADS1256_MUXP_AINCOM 0x80U  /**< 正端 AINCOM (共模) */

/* ================================================================
 * 负输入端通道选择 (MUX 低 4 位)
 * ================================================================ */

#define ADS1256_MUXN_AIN0   0x00U  /**< 负端 AIN0 */
#define ADS1256_MUXN_AIN1   0x01U  /**< 负端 AIN1 */
#define ADS1256_MUXN_AIN2   0x02U  /**< 负端 AIN2 */
#define ADS1256_MUXN_AIN3   0x03U  /**< 负端 AIN3 */
#define ADS1256_MUXN_AIN4   0x04U  /**< 负端 AIN4 */
#define ADS1256_MUXN_AIN5   0x05U  /**< 负端 AIN5 */
#define ADS1256_MUXN_AIN6   0x06U  /**< 负端 AIN6 */
#define ADS1256_MUXN_AIN7   0x07U  /**< 负端 AIN7 */
#define ADS1256_MUXN_AINCOM 0x08U  /**< 负端 AINCOM (共模) */

/* ================================================================
 * 可编程增益放大器 (PGA) 增益值
 * ================================================================ */

#define ADS1256_GAIN_1  0x00U  /**< 增益 x1 */
#define ADS1256_GAIN_2  0x01U  /**< 增益 x2 */
#define ADS1256_GAIN_4  0x02U  /**< 增益 x4 */
#define ADS1256_GAIN_8  0x03U  /**< 增益 x8 */
#define ADS1256_GAIN_16 0x04U  /**< 增益 x16 */
#define ADS1256_GAIN_32 0x05U  /**< 增益 x32 */
#define ADS1256_GAIN_64 0x06U  /**< 增益 x64 */

/* ================================================================
 * 数据输出率 (DRATE) 选项
 * ================================================================ */

#define ADS1256_DRATE_30000SPS 0xF0U  /**< 30000 SPS (采样每秒) */
#define ADS1256_DRATE_15000SPS 0xE0U  /**< 15000 SPS */
#define ADS1256_DRATE_7500SPS  0xD0U  /**< 7500 SPS */
#define ADS1256_DRATE_3750SPS  0xC0U  /**< 3750 SPS */
#define ADS1256_DRATE_2000SPS  0xB0U  /**< 2000 SPS */
#define ADS1256_DRATE_1000SPS  0xA1U  /**< 1000 SPS */
#define ADS1256_DRATE_500SPS   0x92U  /**< 500 SPS */
#define ADS1256_DRATE_100SPS   0x82U  /**< 100 SPS */
#define ADS1256_DRATE_60SPS    0x72U  /**< 60 SPS */
#define ADS1256_DRATE_50SPS    0x63U  /**< 50 SPS */
#define ADS1256_DRATE_30SPS    0x53U  /**< 30 SPS */
#define ADS1256_DRATE_25SPS    0x43U  /**< 25 SPS */
#define ADS1256_DRATE_15SPS    0x33U  /**< 15 SPS */
#define ADS1256_DRATE_10SPS    0x23U  /**< 10 SPS */
#define ADS1256_DRATE_5SPS     0x13U  /**< 5 SPS */
#define ADS1256_DRATE_2_5SPS   0x03U  /**< 2.5 SPS */

/* ================================================================
 * 错误与默认参考电压
 * ================================================================ */

/** 读取失败时返回的错误码 (32 位全 1) */
#define ADS1256_READ_ERROR 0xFFFFFFFFU
/** 默认参考电压 (V) — 由硬件电路决定 */
#define ADS1256_REFERENCE_VOLTAGE_DEFAULT 2.491f

/* ================================================================
 * BSP 函数声明
 * ================================================================ */

HAL_StatusTypeDef ADS1256_Init(void);                                         /**< 完整初始化 ADS1256 */
uint8_t ADS1256_IsDataReady(void);                                            /**< 查询 DRDY 引脚状态 (1:就绪) */
uint8_t ADS1256_IsPowerDownHigh(void);                                        /**< 查询 PDWN 引脚电平 (1:高) */
HAL_StatusTypeDef ADS1256_WriteRegister(uint8_t address, uint8_t value);      /**< 写寄存器 */
HAL_StatusTypeDef ADS1256_ReadRegister(uint8_t address, uint8_t *value);      /**< 读寄存器 */
HAL_StatusTypeDef ADS1256_Read(uint8_t mux, int32_t *value);                  /**< 切换通道并读取一次转换值 */
HAL_StatusTypeDef ADS1256_ReadAverage(uint8_t mux, uint16_t samples, int32_t *value); /**< 多次采样去极值平均 */
float ADS1256_CodeToVoltage(int32_t code, float reference_voltage, uint8_t gain);     /**< 码值转电压 */

/* ================================================================
 * 旧版兼容接口（保留以支持已有应用代码）
 * ================================================================ */

void Init_ADS1256_GPIO(void);           /**< 仅初始化 GPIO（不配置 ADC） */
void SPI_WriteByte(uint8_t data);       /**< 旧版写字节 */
uint8_t SPI_ReadByte(void);             /**< 旧版读字节 */
void ADS1256WREG(uint8_t address, uint8_t value);  /**< 旧版写寄存器 */
uint8_t ADS1256RREG(uint8_t address);             /**< 旧版读寄存器 */
uint32_t ADS1256ReadData(uint8_t mux);            /**< 旧版读取原始数据（不进行符号扩展） */
float Get_Val(uint32_t raw_data);                 /**< 旧版码值转电压 */
uint32_t Moving_Average_Filter(uint8_t mux, uint16_t samples); /**< 旧版多次平均 */

#endif /* BSP_ADS1256_H */
