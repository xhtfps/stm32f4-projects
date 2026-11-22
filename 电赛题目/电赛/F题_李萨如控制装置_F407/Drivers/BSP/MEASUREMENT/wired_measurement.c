/**
 * @file    wired_measurement.c
 * @brief   F题有线双ADC幅值与相位测量服务实现。
 */

#include "wired_measurement.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "./MALLOC/malloc.h"
#include "./BSP/PHASE/phase_vofa.h"

#define WIRED_ADC_MID_COUNT              2048
#define WIRED_MIN_INPUT_FREQUENCY_HZ      900U
#define WIRED_MAX_INPUT_FREQUENCY_HZ     100000U
#define WIRED_FREQUENCY_STEP_HZ          100U
#define WIRED_LOW_BAND_MAX_FREQUENCY_HZ  4500U
#define WIRED_LOW_BAND_MIN_SAMPLE_RATE_HZ 64000U
#define WIRED_PHASE_AGREEMENT_LIMIT_DEG  12.0f
#define WIRED_PHASE_AGREEMENT_BLOCKS     3U
#define WIRED_PHASE_MIN_PEAK_COUNTS       16.0f

WiredMeasurement_t g_wired_measurement;

/** Select the requested rate from the approved analysis-frequency bands. */
static uint32_t WiredMeasurement_SelectSampleRate(
    uint32_t analysis_frequency_hz)
{
    uint32_t multiplier;
    uint64_t requested_rate;

    if (analysis_frequency_hz < 10U)
    {
        multiplier = 1024U;
    }
    else if (analysis_frequency_hz < 100U)
    {
        multiplier = 256U;
    }
    else if (analysis_frequency_hz < 1000U)
    {
        multiplier = 64U;
    }
    else if (analysis_frequency_hz < 10000U)
    {
        multiplier = 16U;
    }
    else if (analysis_frequency_hz < 100000U)
    {
        multiplier = 8U;
    }
    else
    {
        multiplier = 4U;
    }

    requested_rate = (uint64_t)analysis_frequency_hz * multiplier;
    if ((analysis_frequency_hz >= WIRED_MIN_INPUT_FREQUENCY_HZ) &&
        (analysis_frequency_hz < WIRED_LOW_BAND_MAX_FREQUENCY_HZ) &&
        (requested_rate < WIRED_LOW_BAND_MIN_SAMPLE_RATE_HZ))
    {
        requested_rate = WIRED_LOW_BAND_MIN_SAMPLE_RATE_HZ;
    }
    if (requested_rate < PHASE_ADC_MIN_SAMPLE_RATE_HZ)
    {
        return PHASE_ADC_MIN_SAMPLE_RATE_HZ;
    }
    if (requested_rate > PHASE_ADC_MAX_SAMPLE_RATE_HZ)
    {
        return PHASE_ADC_MAX_SAMPLE_RATE_HZ;
    }
    return (uint32_t)requested_rate;
}

static float WiredMeasurement_NormalizeDegrees(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }
    while (degrees < -180.0f)
    {
        degrees += 360.0f;
    }
    return degrees;
}

static uint32_t WiredMeasurement_RoundPositive(float value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= 4294967040.0f)
    {
        return 0xFFFFFFFFUL;
    }
    return (uint32_t)(value + 0.5f);
}

static uint32_t WiredMeasurement_PeakCountsToSourceVppMv(
    float peak_counts,
    const WiredMeasurementConfig_t *config,
    uint16_t attenuation_permille)
{
    float adc_vpp_mv;
    float source_vpp_mv;

    if ((config == NULL) || (attenuation_permille == 0U) ||
        (config->adc_full_scale_count == 0U))
    {
        return 0U;
    }

    adc_vpp_mv =
        peak_counts * 2.0f * (float)config->adc_reference_mv /
        (float)config->adc_full_scale_count;
    source_vpp_mv =
        adc_vpp_mv * 1000.0f / (float)attenuation_permille;
    return WiredMeasurement_RoundPositive(source_vpp_mv);
}

static uint32_t WiredMeasurement_CountToMv(
    int32_t count,
    const WiredMeasurementConfig_t *config)
{
    if ((config == NULL) || (config->adc_full_scale_count == 0U) ||
        (count <= 0))
    {
        return 0U;
    }
    return (uint32_t)(
        ((uint64_t)(uint32_t)count * config->adc_reference_mv +
         config->adc_full_scale_count / 2U) /
        config->adc_full_scale_count);
}

static void WiredMeasurement_BlockStats(
    const int16_t *samples,
    uint16_t count,
    const WiredMeasurementConfig_t *config,
    uint32_t *bias_mv,
    uint16_t *minimum,
    uint16_t *maximum)
{
    int64_t sum = 0;
    int32_t raw;
    uint16_t index;
    uint16_t min_value = 4095U;
    uint16_t max_value = 0U;

    for (index = 0U; index < count; ++index)
    {
        raw = (int32_t)samples[index] + WIRED_ADC_MID_COUNT;
        if (raw < 0)
        {
            raw = 0;
        }
        else if (raw > 4095)
        {
            raw = 4095;
        }
        sum += raw;
        if ((uint16_t)raw < min_value)
        {
            min_value = (uint16_t)raw;
        }
        if ((uint16_t)raw > max_value)
        {
            max_value = (uint16_t)raw;
        }
    }

    *minimum = min_value;
    *maximum = max_value;
    *bias_mv = WiredMeasurement_CountToMv(
        (int32_t)((sum + count / 2U) / count), config);
}

static uint8_t WiredMeasurement_BiasIsValid(
    uint32_t bias_mv,
    const WiredMeasurementConfig_t *config)
{
    uint32_t difference;

    difference = (bias_mv >= config->expected_bias_mv) ?
                 (bias_mv - config->expected_bias_mv) :
                 (config->expected_bias_mv - bias_mv);
    return (difference <= config->bias_tolerance_mv) ? 1U : 0U;
}

void WiredMeasurementConfig_Default(WiredMeasurementConfig_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->adc_reference_mv = 3300U;
    config->adc_full_scale_count = 4095U;
    /* PA1和PA2位于两条不同的模拟调理链路，必须分别校准。 */
    config->input_attenuation_permille = 780U;
    /*
     * PA2直接检测OPA1612后的最终Y输出，硬件检测比例为0.77。
     * 台架实测：X/Y物理幅值相等时，原17.2 / 0.77尺度把Y显示为X的2倍，
     * 说明0.385→0.77的×2重归一化与实际硬件采样信号不符。
     * 后级增益按实测3/4比例修正为5.733（4.3×4/3）。
     * 5.733仍非精确硬件增益，待PA2采样链路校准后应重新标定。
     */
    config->output_attenuation_permille = 770U;
    config->output_post_gain = 5.733f;
    config->expected_bias_mv = 1635U;
    config->bias_tolerance_mv = 200U;
    config->clipping_low_count = 16U;
    config->clipping_high_count = 4079U;
    config->minimum_signal_vpp_mv = 300U;
    config->requirement2_phase_offset_deg = 0.0f;
    config->requirement3_phase_offset_deg = 0.0f;
}

HAL_StatusTypeDef WiredMeasurement_Init(
    WiredMeasurement_t *measurement,
    const WiredMeasurementConfig_t *config)
{
    if ((measurement == NULL) || (config == NULL) ||
        (config->input_attenuation_permille == 0U) ||
        (config->output_attenuation_permille == 0U) ||
        (config->output_post_gain <= 0.0f))
    {
        return HAL_ERROR;
    }

    (void)memset(measurement, 0, sizeof(*measurement));
    measurement->config = *config;
    measurement->input_samples = (int16_t *)mymalloc(
        SRAMCCM, PHASE_ADC_BLOCK_COUNT * sizeof(int16_t));
    measurement->output_samples = (int16_t *)mymalloc(
        SRAMCCM, PHASE_ADC_BLOCK_COUNT * sizeof(int16_t));
    if ((measurement->input_samples == NULL) ||
        (measurement->output_samples == NULL))
    {
        return HAL_ERROR;
    }
    if (PhaseAdc_Init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    measurement->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef WiredMeasurement_Start(
    WiredMeasurement_t *measurement,
    WiredMeasurementMode_t mode)
{
    if ((measurement == NULL) || (measurement->initialized == 0U) ||
        (mode == WIRED_MEASUREMENT_MODE_NONE))
    {
        return HAL_ERROR;
    }

    if (measurement->adc_running != 0U)
    {
        (void)PhaseAdc_Stop();
    }
    (void)memset(&measurement->result, 0,
                 sizeof(measurement->result));
    measurement->mode = mode;
    measurement->active_frequency_hz = 0U;
    measurement->phase_agreement_count = 0U;
    measurement->adc_running = 0U;
    measurement->running = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef WiredMeasurement_SetFrequency(
    WiredMeasurement_t *measurement,
    uint32_t input_frequency_hz)
{
    uint32_t analysis_frequency_hz;
    uint32_t sample_rate_hz;

    if ((measurement == NULL) || (measurement->initialized == 0U) ||
        (measurement->running == 0U) ||
        (input_frequency_hz < WIRED_MIN_INPUT_FREQUENCY_HZ) ||
        (input_frequency_hz > WIRED_MAX_INPUT_FREQUENCY_HZ) ||
        ((input_frequency_hz % WIRED_FREQUENCY_STEP_HZ) != 0U))
    {
        return HAL_ERROR;
    }
    if ((measurement->adc_running != 0U) &&
        (measurement->active_frequency_hz == input_frequency_hz))
    {
        return HAL_OK;
    }

    if (measurement->adc_running != 0U)
    {
        (void)PhaseAdc_Stop();
        measurement->adc_running = 0U;
    }

    analysis_frequency_hz =
        (measurement->mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3) ?
        input_frequency_hz * 2U : input_frequency_hz;
    sample_rate_hz =
        WiredMeasurement_SelectSampleRate(analysis_frequency_hz);
    if ((sample_rate_hz < PHASE_ADC_MIN_SAMPLE_RATE_HZ) ||
        (sample_rate_hz > PHASE_ADC_MAX_SAMPLE_RATE_HZ))
    {
        return HAL_ERROR;
    }
    if (PhaseAdc_Start(sample_rate_hz) != HAL_OK)
    {
        return HAL_ERROR;
    }

    measurement->active_frequency_hz = input_frequency_hz;
    measurement->phase_agreement_count = 0U;
    measurement->result.input_frequency_hz = input_frequency_hz;
    measurement->result.output_analysis_frequency_hz =
        (measurement->mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3) ?
        input_frequency_hz * 2U : input_frequency_hz;
    measurement->result.adc_sample_rate_hz =
        PhaseAdc_GetSampleRateHz();
    measurement->adc_running = 1U;
    return HAL_OK;
}

uint8_t WiredMeasurement_Process(
    WiredMeasurement_t *measurement,
    float reference_frequency_hz)
{
    PhaseFftConfig_t input_config;
    PhaseFftConfig_t output_config;
    PhaseFftResult_t fft_result;
    WiredMeasurementResult_t *result;
    float target_phase_deg;

    if ((measurement == NULL) || (measurement->running == 0U) ||
        (measurement->adc_running == 0U))
    {
        return 0U;
    }
    if (PhaseAdc_TakeBlock(measurement->input_samples,
                           measurement->output_samples,
                           PHASE_ADC_BLOCK_COUNT) != HAL_OK)
    {
        return 0U;
    }

    /* VOFA FireWater waveform path: two columns, PA1 and PA2, already DC-centered. */
    PhaseVofa_SubmitBlock(measurement->input_samples,
                          measurement->output_samples,
                          PHASE_ADC_BLOCK_COUNT);

    result = &measurement->result;
    PhaseFftConfig_Default(&input_config);
    input_config.sample_rate_hz = PhaseAdc_GetSampleRateHz();
    if ((reference_frequency_hz < (float)WIRED_MIN_INPUT_FREQUENCY_HZ) ||
        (reference_frequency_hz > (float)WIRED_MAX_INPUT_FREQUENCY_HZ))
    {
        reference_frequency_hz = (float)measurement->active_frequency_hz;
    }
    input_config.signal_frequency_hz = reference_frequency_hz;
    input_config.point_count = PHASE_ADC_BLOCK_COUNT;
    output_config = input_config;
    output_config.signal_frequency_hz =
        (measurement->mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3) ?
        reference_frequency_hz * 2.0f : reference_frequency_hz;

    WiredMeasurement_BlockStats(
        measurement->input_samples, PHASE_ADC_BLOCK_COUNT,
        &measurement->config, &result->input_bias_mv,
        &result->input_min_count, &result->input_max_count);
    WiredMeasurement_BlockStats(
        measurement->output_samples, PHASE_ADC_BLOCK_COUNT,
        &measurement->config, &result->output_bias_mv,
        &result->output_min_count, &result->output_max_count);

    result->adc_sample_rate_hz = input_config.sample_rate_hz;
    result->adc_overrun_count = PhaseAdc_GetOverrunCount();
    result->adc_rejected_block_count = PhaseAdc_GetRejectedBlockCount();
    result->input_clipped =
        ((result->input_min_count <=
          measurement->config.clipping_low_count) ||
         (result->input_max_count >=
          measurement->config.clipping_high_count)) ? 1U : 0U;
    result->output_clipped =
        ((result->output_min_count <=
          measurement->config.clipping_low_count) ||
         (result->output_max_count >=
          measurement->config.clipping_high_count)) ? 1U : 0U;
    result->clipped =
        ((result->input_clipped != 0U) ||
         (result->output_clipped != 0U)) ? 1U : 0U;
    result->input_bias_valid = WiredMeasurement_BiasIsValid(
        result->input_bias_mv, &measurement->config);
    result->output_bias_valid = WiredMeasurement_BiasIsValid(
        result->output_bias_mv, &measurement->config);
    result->bias_valid =
        ((result->input_bias_valid != 0U) &&
         (result->output_bias_valid != 0U)) ? 1U : 0U;

    if (PhaseFft_AnalyzeDual(
            measurement->input_samples,
            measurement->output_samples,
            &input_config, &output_config,
            &fft_result) == 0U)
    {
        result->input_amplitude_valid = 0U;
        result->output_amplitude_valid = 0U;
        result->amplitude_valid = 0U;
        result->phase_valid = 0U;
        measurement->phase_agreement_count = 0U;
        result->phase_agreement_count = 0U;
        result->valid = 0U;
        ++result->block_count;
        return 1U;
    }

    result->input_vpp_mv =
        WiredMeasurement_PeakCountsToSourceVppMv(
            fft_result.channel1_amplitude,
            &measurement->config,
            measurement->config.input_attenuation_permille);
    result->output_sense_vpp_mv =
        WiredMeasurement_PeakCountsToSourceVppMv(
            fft_result.channel2_amplitude,
            &measurement->config,
            measurement->config.output_attenuation_permille);
    result->output_vpp_mv = WiredMeasurement_RoundPositive(
        (float)result->output_sense_vpp_mv *
        measurement->config.output_post_gain);
    result->amplitude_error_mv =
        (int32_t)result->output_vpp_mv -
        (int32_t)result->input_vpp_mv;
    result->input_phase_deg = fft_result.channel1_phase_deg;
    result->output_phase_deg = fft_result.channel2_phase_deg;

    if (measurement->mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3)
    {
        result->phase_relation_deg = WiredMeasurement_NormalizeDegrees(
            result->output_phase_deg -
            2.0f * result->input_phase_deg -
            measurement->config.requirement3_phase_offset_deg);
        target_phase_deg = 0.0f;
    }
    else
    {
        result->phase_relation_deg = WiredMeasurement_NormalizeDegrees(
            result->output_phase_deg -
            result->input_phase_deg -
            measurement->config.requirement2_phase_offset_deg);
        target_phase_deg =
            (measurement->mode ==
             WIRED_MEASUREMENT_MODE_REQUIREMENT2) ? 90.0f : 0.0f;
    }
    result->phase_error_deg = WiredMeasurement_NormalizeDegrees(
        result->phase_relation_deg - target_phase_deg);
    if (measurement->mode == WIRED_MEASUREMENT_MODE_REQUIREMENT3)
    {
        result->crossing_phase_relation_deg =
            WiredMeasurement_NormalizeDegrees(
                fft_result.channel2_crossing_phase_deg -
                2.0f * fft_result.channel1_crossing_phase_deg -
                measurement->config.requirement3_phase_offset_deg);
    }
    else
    {
        result->crossing_phase_relation_deg =
            WiredMeasurement_NormalizeDegrees(
                fft_result.channel2_crossing_phase_deg -
                fft_result.channel1_crossing_phase_deg -
                measurement->config.requirement2_phase_offset_deg);
    }
    result->phase_estimator_difference_deg =
        WiredMeasurement_NormalizeDegrees(
            result->crossing_phase_relation_deg -
            result->phase_relation_deg);
    result->input_amplitude_valid =
        ((result->input_vpp_mv >=
          measurement->config.minimum_signal_vpp_mv) &&
         (result->input_clipped == 0U)) ? 1U : 0U;
    result->output_amplitude_valid =
        ((result->output_vpp_mv >=
          measurement->config.minimum_signal_vpp_mv) &&
         (result->output_clipped == 0U)) ? 1U : 0U;
    result->amplitude_valid =
        ((result->input_amplitude_valid != 0U) &&
         (result->output_amplitude_valid != 0U)) ? 1U : 0U;
    if ((fft_result.valid != 0U) &&
        (fft_result.crossing_valid != 0U) &&
        (fft_result.channel1_amplitude >= WIRED_PHASE_MIN_PEAK_COUNTS) &&
        (fft_result.channel2_amplitude >= WIRED_PHASE_MIN_PEAK_COUNTS) &&
        (result->clipped == 0U) &&
        (result->bias_valid != 0U) &&
        (fabsf(result->phase_estimator_difference_deg) <=
         WIRED_PHASE_AGREEMENT_LIMIT_DEG))
    {
        if (measurement->phase_agreement_count <
            WIRED_PHASE_AGREEMENT_BLOCKS)
        {
            ++measurement->phase_agreement_count;
        }
    }
    else
    {
        measurement->phase_agreement_count = 0U;
    }
    result->phase_agreement_count = measurement->phase_agreement_count;
    result->phase_valid =
        (measurement->phase_agreement_count >=
         WIRED_PHASE_AGREEMENT_BLOCKS) ? 1U : 0U;
    result->valid =
        ((result->phase_valid != 0U) &&
         (result->bias_valid != 0U)) ? 1U : 0U;
    ++result->block_count;
    return 1U;
}

HAL_StatusTypeDef WiredMeasurement_Stop(
    WiredMeasurement_t *measurement)
{
    HAL_StatusTypeDef status = HAL_OK;

    if ((measurement == NULL) || (measurement->initialized == 0U))
    {
        return HAL_ERROR;
    }
    if (measurement->adc_running != 0U)
    {
        status = PhaseAdc_Stop();
    }
    measurement->adc_running = 0U;
    measurement->running = 0U;
    measurement->mode = WIRED_MEASUREMENT_MODE_NONE;
    measurement->active_frequency_hz = 0U;
    return status;
}

const WiredMeasurementResult_t *WiredMeasurement_GetResult(
    const WiredMeasurement_t *measurement)
{
    if ((measurement == NULL) || (measurement->initialized == 0U))
    {
        return NULL;
    }
    return &measurement->result;
}

HAL_StatusTypeDef wired_measurement_init(void)
{
    WiredMeasurementConfig_t config;

    WiredMeasurementConfig_Default(&config);
    return WiredMeasurement_Init(&g_wired_measurement, &config);
}

HAL_StatusTypeDef wired_measurement_start(WiredMeasurementMode_t mode)
{
    return WiredMeasurement_Start(&g_wired_measurement, mode);
}

HAL_StatusTypeDef wired_measurement_set_frequency(uint32_t frequency_hz)
{
    return WiredMeasurement_SetFrequency(
        &g_wired_measurement, frequency_hz);
}

uint8_t wired_measurement_process(float reference_frequency_hz)
{
    return WiredMeasurement_Process(
        &g_wired_measurement, reference_frequency_hz);
}

HAL_StatusTypeDef wired_measurement_stop(void)
{
    return WiredMeasurement_Stop(&g_wired_measurement);
}

const WiredMeasurementResult_t *wired_measurement_get_result(void)
{
    return WiredMeasurement_GetResult(&g_wired_measurement);
}
