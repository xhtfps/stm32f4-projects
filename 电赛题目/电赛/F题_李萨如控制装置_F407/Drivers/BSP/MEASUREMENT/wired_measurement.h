/**
 * @file    wired_measurement.h
 * @brief   F题要求1~4的有线双通道幅值/相位测量服务。
 *
 * 硬件边界：
 *   PA1/ADC1_IN1 = 信号源输入，经0.78倍衰减并偏置到1.635V；
 *   PA2/ADC2_IN2 = OPA1612后最终Y输出，经约0.77倍检测并偏置到1.635V；
 *   示波器Y输出 = PA2测量值除以0.77，再应用ADC2台架校正系数；
 *   TIM8 TRGO触发ADC1/ADC2规则同步采样，DMA2_Stream0搬运。
 *
 * 本模块只处理采样率规划、单频DFT、幅值换算和相位关系，不控制DDS、
 * 不访问LCD，也不使用K230视觉结果。
 */

#ifndef WIRED_MEASUREMENT_H
#define WIRED_MEASUREMENT_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "./BSP/PHASE/phase_adc.h"
#include "./BSP/PHASE/phase_fft.h"

typedef enum
{
    WIRED_MEASUREMENT_MODE_NONE = 0U,
    WIRED_MEASUREMENT_MODE_REQUIREMENT1,
    WIRED_MEASUREMENT_MODE_REQUIREMENT2,
    WIRED_MEASUREMENT_MODE_REQUIREMENT3,
    WIRED_MEASUREMENT_MODE_REQUIREMENT4,
    WIRED_MEASUREMENT_MODE_DEBUG
} WiredMeasurementMode_t;

typedef struct
{
    uint32_t adc_reference_mv;
    uint16_t adc_full_scale_count;
    uint16_t input_attenuation_permille;  /* PA1输入链路增益，默认780‰ */
    uint16_t output_attenuation_permille; /* PA2检测比例，默认770‰ */
    float output_post_gain;               /* ADC2幅值台架校正，当前5.733 */
    uint16_t expected_bias_mv;
    uint16_t bias_tolerance_mv;
    uint16_t clipping_low_count;
    uint16_t clipping_high_count;
    uint16_t minimum_signal_vpp_mv;
    float requirement2_phase_offset_deg;
    float requirement3_phase_offset_deg;
} WiredMeasurementConfig_t;

typedef struct
{
    uint32_t input_frequency_hz;
    uint32_t output_analysis_frequency_hz;
    uint32_t adc_sample_rate_hz;
    uint32_t adc_overrun_count;
    uint32_t adc_rejected_block_count;
    uint32_t block_count;
    uint32_t input_vpp_mv;
    uint32_t output_sense_vpp_mv;  /* 由PA2按0.77反算的检测Vpp */
    uint32_t output_vpp_mv;        /* 应用ADC2台架校正后的最终Y输出Vpp */
    int32_t amplitude_error_mv;
    uint32_t input_bias_mv;
    uint32_t output_bias_mv;
    float input_phase_deg;
    float output_phase_deg;
    float phase_relation_deg;
    float phase_error_deg;
    float crossing_phase_relation_deg;
    float phase_estimator_difference_deg;
    uint16_t input_min_count;
    uint16_t input_max_count;
    uint16_t output_min_count;
    uint16_t output_max_count;
    uint8_t input_amplitude_valid;
    uint8_t output_amplitude_valid;
    uint8_t amplitude_valid;
    uint8_t phase_valid;
    uint8_t phase_agreement_count;
    uint8_t input_clipped;
    uint8_t output_clipped;
    uint8_t clipped;
    uint8_t input_bias_valid;
    uint8_t output_bias_valid;
    uint8_t bias_valid;
    uint8_t valid;
} WiredMeasurementResult_t;

typedef struct
{
    WiredMeasurementConfig_t config;
    WiredMeasurementResult_t result;
    int16_t *input_samples;
    int16_t *output_samples;
    WiredMeasurementMode_t mode;
    uint32_t active_frequency_hz;
    uint8_t initialized;
    uint8_t running;
    uint8_t adc_running;
    uint8_t phase_agreement_count;
} WiredMeasurement_t;

extern WiredMeasurement_t g_wired_measurement;

void WiredMeasurementConfig_Default(WiredMeasurementConfig_t *config);
HAL_StatusTypeDef WiredMeasurement_Init(
    WiredMeasurement_t *measurement,
    const WiredMeasurementConfig_t *config);
HAL_StatusTypeDef WiredMeasurement_Start(
    WiredMeasurement_t *measurement,
    WiredMeasurementMode_t mode);
HAL_StatusTypeDef WiredMeasurement_SetFrequency(
    WiredMeasurement_t *measurement,
    uint32_t input_frequency_hz);
uint8_t WiredMeasurement_Process(
    WiredMeasurement_t *measurement,
    float reference_frequency_hz);
HAL_StatusTypeDef WiredMeasurement_Stop(
    WiredMeasurement_t *measurement);
const WiredMeasurementResult_t *WiredMeasurement_GetResult(
    const WiredMeasurement_t *measurement);

/* 单实例固件使用的简洁包装接口。 */
HAL_StatusTypeDef wired_measurement_init(void);
HAL_StatusTypeDef wired_measurement_start(WiredMeasurementMode_t mode);
HAL_StatusTypeDef wired_measurement_set_frequency(uint32_t frequency_hz);
uint8_t wired_measurement_process(float reference_frequency_hz);
HAL_StatusTypeDef wired_measurement_stop(void);
const WiredMeasurementResult_t *wired_measurement_get_result(void);

#endif /* WIRED_MEASUREMENT_H */
