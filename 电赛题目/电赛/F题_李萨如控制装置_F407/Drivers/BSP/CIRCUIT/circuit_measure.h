#ifndef CIRCUIT_MEASURE_H
#define CIRCUIT_MEASURE_H

#include "stm32f4xx_hal.h"
#include "../ADS1256/ADS1256.h"

/* ============================================================
 *  电路幅频特性测量模块
 *  硬件链路：待测电路 → OPA运放 → AD637有效值检波 → ADS1256采集
 *  功能：同时采集输入/输出两路交流信号有效值，计算电压增益
 * ============================================================ */

/**
 * @brief  测量配置结构体：保存两路采集通道的全部校准参数
 * @note   上电初始化时赋值一次，测量时直接传入复用
 */
typedef struct
{
    uint8_t  input_mux;            // 输入通道ADS1256多路选择器配置（AINx+AINCOM组合）
    uint8_t  output_mux;           // 输出通道ADS1256多路选择器配置
    uint16_t average_samples;      // 单次测量平均采样次数（次数越多越稳，耗时越长）
    float    reference_voltage;    // ADS1256参考电压（单位：V，一般2.5V）
    uint8_t  ads_gain;             // ADS1256内部PGA增益倍数（1/2/4/8/16/32/64）

    /* 前端运放增益校准值（OPA1612放大倍数） */
    float    input_opa_gain;       // 输入通道OPA1612运放增益倍数
    float    output_opa_gain;      // 输出通道OPA1612运放增益倍数

    /* AD637检波器传输增益校准值 */
    float    input_ad637_gain;     // 输入通道AD637有效值检波电路增益
    float    output_ad637_gain;    // 输出通道AD637有效值检波电路增益

    /* 零点漂移校准（空载时测得的静态偏移，用于消除检波器零漂误差） */
    float    input_offset_voltage; // 输入通道零点偏移电压（单位：V）
    float    output_offset_voltage;// 输出通道零点偏移电压（单位：V）
} CircuitMeasureConfig_t;

/**
 * @brief  测量结果结构体：一次测量返回的全部数据
 */
typedef struct
{
    float   input_rms;    // 输入端交流信号有效值（单位：V，已还原真实电压）
    float   output_rms;   // 输出端交流信号有效值（单位：V，已还原真实电压）
    float   gain;         // 线性电压增益 = 输出/输入（倍数）
    float   gain_db;      // 对数电压增益（单位：dB，=20×log10(gain)）
    int32_t input_raw_code;       // AIN0平均原始码
    int32_t output_raw_code;      // AIN1平均原始码
    float   input_detector_voltage;  // AIN0对应ADS1256输入电压
    float   output_detector_voltage; // AIN1对应ADS1256输入电压
    uint8_t valid;        // 结果有效标志：1=测量成功数据有效，0=无效
} CircuitMeasurement_t;

/**
 * @brief  执行一次电路幅频测量（采集输入输出，计算增益）
 * @param  config  测量配置（通道、增益、零点校准等参数）
 * @param  result  输出测量结果（输入RMS、输出RMS、增益、dB增益）
 * @retval HAL状态：HAL_OK=测量成功，HAL_ERROR=参数错误或采集失败
 */
HAL_StatusTypeDef CircuitMeasure_Read(const CircuitMeasureConfig_t *config,
                                      CircuitMeasurement_t *result);

#endif /* CIRCUIT_MEASURE_H */
