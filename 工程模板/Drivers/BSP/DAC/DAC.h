#ifndef BSP_DAC_H
#define BSP_DAC_H

/**
 * @file    BSP_DAC.h
 * @brief   DDS 信号发生器 DAC 驱动头文件
 *
 * 定义基于 STM32 DAC + DMA + TIM6 的波形生成接口，支持
 * 正弦、三角、锯齿、方波以及用户自定义缓冲区流式输出。
 * 同时提供旧版兼容函数入口。
 */

#include "stm32f4xx_hal.h"

/* ================================================================
 * 配置常量
 * ================================================================ */

/** DDS 波形缓冲区最大长度（16 位半字） */
#define DDS_BUFFER_LENGTH 1000U
/** DAC 参考电压 (V)，决定输出电压范围 */
#define DDS_DAC_VREF_VOLTS 3.3

/* ================================================================
 * 波形类型枚举
 * ================================================================ */

/**
 * @brief 内置波形类型
 */
typedef enum
{
    SINWAVE = 0,        /**< 正弦波 */
    TRIANGLEWAE,        /**< 三角波 (注意：原拼写保留) */
    SAWTOOTHWAVE,       /**< 锯齿波 */
    SQUAREWAVE          /**< 方波 */
} waveEnum;

/* ================================================================
 * DDS 数据结构
 * ================================================================ */

/**
 * @brief DDS 波形参数与数据集合
 *
 * 包含当前波形类型、频率、幅值、占空比，以及存储波形样点的
 * 缓冲区和生成函数指针。
 */
typedef struct
{
    uint16_t data[DDS_BUFFER_LENGTH];   /**< 波形采样数据缓冲（12 位 DAC 码） */
    uint16_t length;                    /**< 当前使用的有效数据点数 */
    float vpp;                          /**< 峰峰值电压 (V) */
    uint32_t hz;                        /**< 输出频率 (Hz) */
    waveEnum wave;                      /**< 当前波形类型 */
    uint16_t dutycycle;                 /**< 占空比 (%)，仅方波有效 */
    /**
     * @brief 波形生成函数指针
     * @param t           当前采样点序号 (0 .. period-1)
     * @param period      周期总采样点数
     * @param duty_cycle  占空比 (%)
     * @return 归一化幅值 [-1.0, 1.0]
     */
    double (*createWaveData)(uint32_t t, uint32_t period, double duty_cycle);
} ddsStruct;

/* ================================================================
 * 全局外设句柄与数据结构（声明于 DAC.c）
 * ================================================================ */

extern DAC_HandleTypeDef hdac;          /**< DAC 外设句柄 */
extern TIM_HandleTypeDef htim6;         /**< TIM6 触发定时器句柄 */
extern ddsStruct ddsStructData;         /**< 全局 DDS 实例 */

/* ================================================================
 * 新版 BSP 接口
 * ================================================================ */

HAL_StatusTypeDef BSP_DAC_Init(void);                                   /**< 初始化 DAC 及默认 DDS 配置 */
HAL_StatusTypeDef BSP_DAC_SetValue(uint16_t value);                     /**< 输出固定电压值（停止波形） */
HAL_StatusTypeDef BSP_DAC_StartWaveform(double vpp, uint32_t frequency_hz,
                                        double duty_cycle, waveEnum wave); /**< 启动内置波形输出 */
HAL_StatusTypeDef BSP_DAC_StartStream(uint32_t sample_rate_hz,
                                      uint16_t *buffer,
                                      uint16_t length);                 /**< 启动外部缓冲区流式输出 */
HAL_StatusTypeDef BSP_DAC_StartSynchronizedStream(
                                      uint16_t *buffer,
                                      uint16_t length);                 /**< DMA armed for TIM8 TRGO; ADC starts the shared timer */
HAL_StatusTypeDef BSP_DAC_StopWaveform(void);                           /**< 停止波形输出并关闭定时器 */

/* ================================================================
 * 旧版兼容接口（保留以支持已有代码）
 * ================================================================ */

void timer6Init(uint32_t sample_rate_hz);                               /**< 初始化 TIM6 采样率 */
void DAC1_Init(void);                                                   /**< 旧版 DAC 初始化 */
void DAC1_Vol_Set(uint16_t value);                                      /**< 旧版设置 DAC 电压 */
void dacClose(void);                                                    /**< 关闭 DAC 输出与定时器 */
void dacOpen(void);                                                     /**< 重新开启 DAC 输出 */
void dacInit(void);                                                     /**< 旧版 DAC 初始化别名 */
void setDDS(double vpp, uint32_t frequency_hz, double duty_cycle, waveEnum wave); /**< 旧版设置 DDS 波形 */

#endif /* BSP_DAC_H */
