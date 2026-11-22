#include "circuit_measure.h"
#include <math.h>

/* ============================================================
 *  内部辅助函数：ADS1256原始采样码 → 实际电路RMS电压换算
 *  信号链路反向还原：ADC码 → AD637检波输出 → 除以运放增益 → 原始信号有效值
 * ============================================================ */

/**
 * @brief  将ADS1256采集的原始数字码，还原为被测点真实交流有效值电压
 * @param  code              ADS1256 24位原始采样码
 * @param  reference_voltage ADS1256参考电压（V）
 * @param  ads_gain          ADS1256内部PGA增益倍数
 * @param  offset_voltage    零点偏移电压（空载校准值，用于消除检波器零漂）
 * @param  ad637_gain        AD637有效值检波电路增益（检波传输系数）
 * @param  opa_gain          前端OPA1612运放放大倍数
 * @return  还原后的真实交流信号有效值（V）
 * @note    换算链路：ADC电压 - 零点偏移 = AD637输出电压
 *          真实输入电压 = AD637输出 × 检波增益 / 运放增益
 */
static float CircuitMeasure_Convert(int32_t code,
                                    float reference_voltage,
                                    uint8_t ads_gain,
                                    float offset_voltage,
                                    float ad637_gain,
                                    float opa_gain)
{
    float detector_voltage;

    /* 第一步：24位数字码 → ADS1256输入引脚实际电压（AD637输出的直流有效值） */
    detector_voltage = ADS1256_CodeToVoltage(code, reference_voltage,
                                              ads_gain);

    /* 第二步：减去零点漂移电压，消除检波器静态偏移误差 */
    detector_voltage -= offset_voltage;

    /* 有效值不能为负，噪声导致的负值直接钳位到0 */
    if (detector_voltage < 0.0f)
    {
        detector_voltage = 0.0f;
    }

    /* 增益参数非法保护，避免除零或负增益 */
    if ((ad637_gain <= 0.0f) || (opa_gain <= 0.0f))
    {
        return 0.0f;
    }

    /* 第三步：反向还原真实信号电压
       信号流向：被测信号 → OPA运放放大 → AD637检波 → ADC采集
       反向推导：真实电压 = 检波输出 × 检波增益 / 运放增益 */
    return detector_voltage * ad637_gain / opa_gain;
}

/* ============================================================
 *  对外主接口：一次读取电路输入+输出两路有效值，并计算增益
 * ============================================================ */

/**
 * @brief  采集电路输入输出两路有效值，计算电压增益与dB增益
 * @param  config  测量配置结构体（通道选择、采样次数、各级增益、零点偏移）
 * @param  result  输出测量结果：输入RMS、输出RMS、线性增益、dB增益、有效标志
 * @retval HAL状态：HAL_OK=测量成功，HAL_ERROR=参数非法或采集失败
 * @note   硬件链路：AD9959激励 → 待测电路 → 两路OPA1612 → 两路AD637有效值检波 → ADS1256双通道采集
 */
HAL_StatusTypeDef CircuitMeasure_Read(const CircuitMeasureConfig_t *config,
                                      CircuitMeasurement_t *result)
{
    int32_t input_code;    // 输入通道ADS1256原始采样码
    int32_t output_code;   // 输出通道ADS1256原始采样码
    HAL_StatusTypeDef status;

    /* ---------- 参数合法性检查 ---------- */
    if ((config == NULL) || (result == NULL) ||
        (config->average_samples == 0U) ||
        (config->ads_gain == 0U) ||
        (config->reference_voltage <= 0.0f))
    {
        return HAL_ERROR;
    }

    /* 结果结构体先清零赋默认值，防止失败时残留脏数据 */
    result->input_rms = 0.0f;
    result->output_rms = 0.0f;
    result->gain = 0.0f;
    result->gain_db = -120.0f;   // 默认极小增益（-120dB表示无信号）
    result->input_raw_code = 0;
    result->output_raw_code = 0;
    result->input_detector_voltage = 0.0f;
    result->output_detector_voltage = 0.0f;
    result->valid = 0U;

    /* ---------- 第一步：采集输入通道（多次平均滤波） ---------- */
    status = ADS1256_ReadAverage(config->input_mux,
                                 config->average_samples,
                                 &input_code);
    if (status != HAL_OK)
    {
        return status;
    }

    /* ---------- 第二步：采集输出通道（多次平均滤波） ---------- */
    status = ADS1256_ReadAverage(config->output_mux,
                                 config->average_samples,
                                 &output_code);
    if (status != HAL_OK)
    {
        return status;
    }

    result->input_raw_code = input_code;
    result->output_raw_code = output_code;
    result->input_detector_voltage = ADS1256_CodeToVoltage(
        input_code, config->reference_voltage, config->ads_gain);
    result->output_detector_voltage = ADS1256_CodeToVoltage(
        output_code, config->reference_voltage, config->ads_gain);

    /* AD637 RMS 输出为单极性直流。负码说明本次 SPI/采样结果无效，
       不能继续换算，否则会被下方钳位成 0 并伪装成 0 增益。 */
    /* ---------- 第三步：两路原始码分别换算为真实RMS电压 ---------- */
    result->input_rms = CircuitMeasure_Convert(
        input_code, config->reference_voltage, config->ads_gain,
        config->input_offset_voltage, config->input_ad637_gain,
        config->input_opa_gain);

    result->output_rms = CircuitMeasure_Convert(
        output_code, config->reference_voltage, config->ads_gain,
        config->output_offset_voltage, config->output_ad637_gain,
        config->output_opa_gain);

    /* 输入信号过小或为0，无法计算增益（避免除零） */
    if (result->input_rms <= 0.000001f)
    {
        return HAL_ERROR;
    }

    /* ---------- 第四步：计算电压增益与dB增益 ---------- */
    result->gain = result->output_rms / result->input_rms;  // 线性增益 = 输出/输入

    if (result->gain <= 0.000001f)
    {
        /* 增益极小，钳位为0并赋值-120dB底噪值 */
        result->gain = 0.0f;
        result->gain_db = -120.0f;
    }
    else
    {
        /* 电压增益转dB公式：dB = 20 × log10(增益倍数) */
        result->gain_db = 20.0f * (float)log10((double)result->gain);
    }

    /* 标记本次测量结果有效 */
    result->valid = 1U;
    return HAL_OK;
}
