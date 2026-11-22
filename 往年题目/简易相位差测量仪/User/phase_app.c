/**
 * @file    phase_app.c
 * @brief   Phase-amplitude instrument state machine.
 *
 * The capture route (raw/external-divide-by-32) and ADC route
 * (direct/mixed) are
 * deliberately independent.  A relay change stops ADC DMA, performs OUT3
 * break-before-make, waits for analogue settling, invalidates old samples and
 * remeasures whenever OUT1 changed.
 */

#include "phase_app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "./BSP/AD9959/Drive_Communication.h"
#include "./BSP/AD8367/ad8367.h"
#include "./BSP/DEBUG/debug_uart6.h"
#include "./BSP/PHASE/phase_adc.h"
#include "./BSP/PHASE/phase_calibration.h"
#include "./BSP/PHASE/phase_capture.h"
#include "./BSP/PHASE/phase_measure.h"
#include "./BSP/PHASE/phase_pipeline.h"
#include "./BSP/PHASE/phase_route.h"
#include "./BSP/PHASE/phase_vofa.h"
#include "./MALLOC/malloc.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PHASE_APP_IF_FREQUENCY_HZ          10000U
#define PHASE_APP_MIXED_SAMPLE_RATE_HZ    256000U
#define PHASE_APP_DIRECT_SAMPLES_PER_CYCLE    64U
#define PHASE_APP_DDS_AMPLITUDE_MV          250.0f
#define PHASE_APP_DDS_LO_CHANNEL              2U /* AD9959 CH2 / 第3路输出 */
#define PHASE_APP_DDS_MIN_GAP_MS             100U
#define PHASE_APP_DDS_DEADBAND_HZ             20U
#define PHASE_APP_UI_PUBLISH_MS               50U
#define PHASE_APP_CAPTURE_PROCESS_PERIOD_MS     5U
#define PHASE_APP_ANALYSIS_PERIOD_MS            5U
#define PHASE_APP_VOFA_BLOCK_PERIOD_MS         20U
#define PHASE_APP_NO_SIGNAL_MS              2500U
#define PHASE_APP_SLOW_FALLBACK_MS           300U
#define PHASE_APP_M1_VERIFY_MIN_HZ        4000000U
#define PHASE_APP_M1_VERIFY_MAX_HZ        6500000U
#define PHASE_APP_M1_VERIFY_TOLERANCE_PERCENT 10U
#define PHASE_APP_M1_VERIFY_TIMEOUT_MS        200U
#define PHASE_APP_M1_VERIFY_RETRY_MS         5000U
#define PHASE_APP_T_BREAK_MS                   5U
#define PHASE_APP_T_MAKE_MS                    5U
#define PHASE_APP_T_ANALOG_SETTLE_MS           5U
#define PHASE_APP_T_LO_SETTLE_MS              10U
#define PHASE_APP_T_SOURCE_SETTLE_MS           20U
#define PHASE_APP_LEARNING_AMPLITUDE_MV       250.0f
#define PHASE_APP_WIRING_CASE_COUNT             4U
#define PHASE_APP_WIRING_REQUIRED_SAMPLES       3U
#define PHASE_APP_WIRING_CASE_TIMEOUT_MS     2500U
#define PHASE_APP_WIRING_DIRECT_FREQUENCY_HZ 100000U
#define PHASE_APP_WIRING_MIXED_FREQUENCY_HZ 1000000U
#define PHASE_APP_WIRING_AMPLITUDE_MV         250.0f
#define PHASE_APP_WIRING_MIN_AMPLITUDE_COUNTS   8.0f
#define PHASE_APP_WIRING_FREQUENCY_TOLERANCE_HZ 5000U
#define PHASE_APP_WIRING_PHASE_TOLERANCE_DEG   30.0f
#define PHASE_APP_WIRING_ADC_CLIP_COUNTS       2000U
#define PHASE_APP_MIN_VALID_AMPLITUDE_COUNTS   4.0f
#define PHASE_APP_AD8367_DEFAULT_GAIN_DB        0.0f
#define PHASE_APP_AD8367_MODE \
    AD8367_GAIN_INCREASES_WITH_VOLTAGE

#if (PHASE_ADC_BLOCK_COUNT > PHASE_FFT_MAX_POINTS)
#error "PHASE_ADC_BLOCK_COUNT exceeds the DFT work-buffer size"
#endif

typedef enum
{
    PHASE_APP_MODE_IDLE = 0U,
    PHASE_APP_MODE_FREQUENCY = 1U,
    PHASE_APP_MODE_PHASE = 2U,
    PHASE_APP_MODE_FFT = 3U,
    PHASE_APP_MODE_LEARNING = 4U,
    PHASE_APP_MODE_WIRING = 5U,
    PHASE_APP_MODE_VOFA = 6U,
    PHASE_APP_MODE_SYSTEM = 7U
} PhaseAppMode_t;

typedef struct
{
    const char *name;
    uint32_t frequency_hz;
    uint32_t channel1_phase_deg;
    uint32_t channel0_phase_deg;
    float expected_phase_deg;
    uint8_t check_phase;
    PhaseCaptureRoute_t capture_route;
    PhaseAdcMode_t adc_mode;
} PhaseAppWiringCase_t;

static const PhaseAppWiringCase_t s_wiring_cases[
    PHASE_APP_WIRING_CASE_COUNT] =
{
    {"RAW_DIRECT", PHASE_APP_WIRING_DIRECT_FREQUENCY_HZ,
     0U, 0U, 0.0f, 0U,
     PHASE_CAPTURE_ROUTE_RAW,
     PHASE_ADC_MODE_DIRECT},
    {"DIV32_DIRECT", PHASE_APP_WIRING_DIRECT_FREQUENCY_HZ,
     0U, 0U, 0.0f, 0U,
     PHASE_CAPTURE_ROUTE_DIV32,
     PHASE_ADC_MODE_DIRECT},
    {"MIXED_HIGH_LO", PHASE_APP_WIRING_MIXED_FREQUENCY_HZ,
     0U, 0U, 0.0f, 0U,
     PHASE_CAPTURE_ROUTE_RAW,
     PHASE_ADC_MODE_MIXED},
    {"CHANNEL_PHASE_MAP", PHASE_APP_WIRING_DIRECT_FREQUENCY_HZ,
     0U, 90U, 90.0f, 1U,
     PHASE_CAPTURE_ROUTE_RAW,
     PHASE_ADC_MODE_DIRECT}
};

typedef enum
{
    PHASE_APP_STATE_IDLE = 0U,
    PHASE_APP_STATE_MEASURE_FREQUENCY = 1U,
    PHASE_APP_STATE_DECIDE_ROUTE = 2U,
    PHASE_APP_STATE_SWITCH_RELAYS = 3U,
    PHASE_APP_STATE_WAIT_RELAY_SETTLE = 4U,
    PHASE_APP_STATE_REMEASURE = 5U,
    PHASE_APP_STATE_SET_LOCAL_OSC = 6U,
    PHASE_APP_STATE_FLUSH_ADC = 7U,
    PHASE_APP_STATE_ACQUIRE_PHASE = 8U,
    PHASE_APP_STATE_ERROR = 9U
} PhaseAppState_t;

typedef struct
{
    float channel1_zero_counts;
    float channel2_zero_counts;
    float channel1_gain;
    float channel2_gain;
    float phase_offset_deg;
} PhaseAppCalibration_t;

/*
 * Separate calibration storage is intentional.  Replace these defaults with
 * values obtained from direct-low-frequency and mixed/high-frequency
 * calibration; the processing path already applies all five terms.
 */
static const PhaseAppCalibration_t s_calibration_direct =
{
    0.0f, 0.0f, 1.0f, 1.0f, 0.0f
};
static const PhaseAppCalibration_t s_calibration_mixed =
{
    0.0f, 0.0f, 1.0f, 1.0f, 0.0f
};

/*
 * These are CPU-only analysis copies, not DMA targets.  Allocate them from
 * CCM so the DMA-capable 0x20000000 SRAM remains available for FreeRTOS,
 * peripheral buffers and stacks.
 */
static int16_t *s_channel1;
static int16_t *s_channel2;
static uint32_t s_last_dds_frequency_hz;
static TickType_t s_last_dds_update_tick;
static uint32_t s_adc_requested_rate_hz;
static uint8_t s_initialized;
static uint8_t s_adc_running;
static uint8_t s_capture_running;
static uint8_t s_local_oscillator_enabled;
static uint8_t s_discard_adc_blocks;
static uint8_t s_no_signal_published;
static TickType_t s_last_ui_publish_tick;
static TickType_t s_last_frequency_tick;
static TickType_t s_last_capture_process_tick;
static TickType_t s_last_analysis_tick;
static TickType_t s_last_vofa_block_tick;
static TickType_t s_menu_test_start_tick;
static uint8_t s_menu_test_result_reported;
static uint8_t s_capture_recovery_attempted;
static uint8_t s_capture_watchdog_restart_attempted;
static uint8_t s_m1_verify_pending;
static uint8_t s_m1_div32_verified;
static uint8_t s_m1_verify_failed;
static uint32_t s_m1_raw_candidate_hz;
static TickType_t s_m1_verify_start_tick;
static TickType_t s_m1_verify_failure_tick;
static PhaseCaptureRoute_t s_capture_route;
static PhaseAdcMode_t s_adc_mode;
static PhaseAppMode_t s_mode;
static PhaseAppState_t s_state;
static PhaseAppMeasurementPublisher_t s_publish;
static UI_Measurement_t s_measurement;
static TaskHandle_t s_phase_app_task;
static uint8_t s_wiring_active;
static uint8_t s_wiring_case_index;
static uint8_t s_wiring_sample_count;
static uint8_t s_wiring_valid_sample_count;
static uint8_t s_wiring_pass_count;
static uint8_t s_wiring_fail_count;
static uint64_t s_wiring_frequency_sum;
static float s_wiring_channel1_sum;
static float s_wiring_channel2_sum;
static float s_wiring_phase_sum;
static TickType_t s_wiring_case_start_tick;
static uint32_t s_wiring_overrun_start;
static uint16_t s_wiring_channel1_peak;
static uint16_t s_wiring_channel2_peak;
static uint8_t s_wiring_adc_raw_reported;
static uint8_t s_wiring_frequency_sampled;
static float s_wiring_direct_phase_reference_deg;
static uint8_t s_wiring_direct_phase_reference_valid;

static void phase_app_invalidate_phase(void);
static void phase_app_set_state(PhaseAppState_t state);
static void phase_app_publish_measurement(uint8_t force);
static uint8_t phase_app_restart_capture(void);
static uint8_t phase_app_restart_capture_keep_result(void);
static void phase_app_wait_dds_gap(void);
static uint8_t phase_app_disable_local_oscillator(void);
static uint8_t phase_app_prepare_acquisition(
    uint32_t input_frequency_hz);
static uint8_t phase_app_apply_route(
    PhaseCaptureRoute_t target_capture_route,
    PhaseAdcMode_t target_adc_mode);
static uint8_t phase_app_apply_frequency_route(
    PhaseCaptureRoute_t target_capture_route);
static uint8_t phase_app_learning_start_current_point(void);
static uint8_t phase_app_ad9959_test_channel(
    uint8_t channel,
    uint32_t frequency_hz,
    uint32_t phase_deg,
    float amplitude_mv,
    uint8_t output);
static uint8_t phase_app_ad9959_output_test(void);
static uint8_t phase_app_wiring_start_case(void);
static void phase_app_wiring_finish(uint8_t aborted);
static void phase_app_wiring_check_sample(void);
static void phase_app_wiring_check_timeout(void);
static void phase_app_menu_test_begin(PhaseAppMode_t mode);
static void phase_app_menu_test_report_measurement(void);
static uint8_t phase_app_m1_needs_div32_verification(
    uint32_t frequency_hz);
static uint8_t phase_app_m1_frequency_matches(
    uint32_t raw_frequency_hz,
    uint32_t div32_frequency_hz);
static void phase_app_m1_publish_wait_signal(void);

static void phase_app_adc_ready_from_isr(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (s_phase_app_task != NULL)
    {
        vTaskNotifyGiveFromISR(
            s_phase_app_task, &higher_priority_task_woken);
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }
}

static float phase_app_normalize_degrees(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }
    while (degrees <= -180.0f)
    {
        degrees += 360.0f;
    }
    return degrees;
}

static uint8_t phase_app_m1_needs_div32_verification(
    uint32_t frequency_hz)
{
    return ((frequency_hz >= PHASE_APP_M1_VERIFY_MIN_HZ) &&
            (frequency_hz <= PHASE_APP_M1_VERIFY_MAX_HZ)) ? 1U : 0U;
}

static uint8_t phase_app_m1_frequency_matches(
    uint32_t raw_frequency_hz,
    uint32_t div32_frequency_hz)
{
    uint32_t difference_hz;
    uint32_t tolerance_hz;

    difference_hz = (raw_frequency_hz >= div32_frequency_hz) ?
                    (raw_frequency_hz - div32_frequency_hz) :
                    (div32_frequency_hz - raw_frequency_hz);
    tolerance_hz =
        (raw_frequency_hz /
         (100U / PHASE_APP_M1_VERIFY_TOLERANCE_PERCENT));
    return (difference_hz <= tolerance_hz) ? 1U : 0U;
}

/*
 * Menu 1 has no ADC amplitude information.  In the 4-6.5 MHz region an
 * unloaded LTC6752/raw path can oscillate or pick up LCD/FSMC activity and
 * still look like a valid TIM2 waveform.  Clear that unconfirmed candidate
 * instead of publishing it as a signal.
 */
static void phase_app_m1_publish_wait_signal(void)
{
    s_measurement.frequency_hz = 0U;
    s_measurement.divided_frequency_hz = 0U;
    s_measurement.frequency_hz_precise = 0.0f;
    s_measurement.divided_frequency_hz_precise = 0.0f;
    s_measurement.period_ticks = 0U;
    s_measurement.frequency_valid = 0U;
    s_measurement.status_code = 0U;
    (void)strcpy(s_measurement.status_text, "WAIT SIGNAL");
    s_no_signal_published = 1U;
    phase_app_set_state(PHASE_APP_STATE_MEASURE_FREQUENCY);
    phase_app_publish_measurement(1U);
}

static void phase_app_print_relay_state(const char *prefix)
{
    PhaseRouteDiagnostics_t diagnostics;

    (void)memset(&diagnostics, 0, sizeof(diagnostics));
    PhaseRoute_GetDiagnostics(&diagnostics);
    (void)DebugUart6_TextPrintfBlocking(
        "%s RELAY PG9=%u PG10=%u PG11=%u PG12=%u PG13=%u PG14=%u PG15=%u OUT3=%u",
        prefix,
        (unsigned)diagnostics.out1_ctrl,
        (unsigned)diagnostics.out2_ctrl,
        (unsigned)diagnostics.out4_ctrl,
        (unsigned)diagnostics.out3_in6,
        (unsigned)diagnostics.out5_ctrl,
        (unsigned)diagnostics.out3_in5,
        (unsigned)diagnostics.out3_in7,
        (unsigned)diagnostics.out3_input);
}

static void phase_app_menu_test_begin(PhaseAppMode_t mode)
{
    s_menu_test_start_tick = xTaskGetTickCount();
    s_menu_test_result_reported = 0U;
    s_capture_recovery_attempted = 0U;
    s_capture_watchdog_restart_attempted = 0U;
    s_m1_verify_pending = 0U;
    s_m1_div32_verified = 0U;
    s_m1_verify_failed = 0U;
    s_m1_raw_candidate_hz = 0U;
    s_m1_verify_start_tick = 0U;
    s_m1_verify_failure_tick = 0U;

    if (mode == PHASE_APP_MODE_FREQUENCY)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M1 BEGIN FUNCTION=FREQUENCY CAPTURE_EVENT=10..1250000Hz EXT_DIV=32 SWITCH=4.9/5.1MHz VERIFY=RAW_DIV32 ADC=OFF LO=OFF OUT3=NONE");
    }
    else if (mode == PHASE_APP_MODE_PHASE)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M2 BEGIN FUNCTION=PHASE ADC=ON AUTO_ROUTE=ON");
    }
    else if (mode == PHASE_APP_MODE_FFT)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M3 BEGIN FUNCTION=ADC_FFT ADC=ON AUTO_ROUTE=ON");
    }
    else if (mode == PHASE_APP_MODE_LEARNING)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M4 BEGIN FUNCTION=LEARNING POINTS=13 SAMPLES_PER_POINT=8");
    }
    else if (mode == PHASE_APP_MODE_WIRING)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M5 BEGIN FUNCTION=FULL_WIRING_TEST");
    }
}

static void phase_app_menu_test_report_measurement(void)
{
    const char *menu_name;

    if (s_menu_test_result_reported != 0U)
    {
        return;
    }
    if (s_mode == PHASE_APP_MODE_PHASE)
    {
        menu_name = "M2";
    }
    else if (s_mode == PHASE_APP_MODE_FFT)
    {
        menu_name = "M3";
    }
    else
    {
        return;
    }

    s_menu_test_result_reported = 1U;
    (void)DebugUart6_TextPrintfBlocking(
        "MENU_TEST %s RESULT PASS FREQ=%lu ADC_MODE=%u RATE=%lu CH1=%.2f CH2=%.2f PHASE=%.2f GAIN=%.2f LO=%lu OVR=%lu",
        menu_name,
        (unsigned long)s_measurement.frequency_hz,
        (unsigned)s_adc_mode,
        (unsigned long)s_measurement.adc_sample_rate_hz,
        (double)s_measurement.channel1_amplitude,
        (double)s_measurement.channel2_amplitude,
        (double)s_measurement.phase_difference_deg,
        (double)s_measurement.gain_db,
        (unsigned long)s_measurement.local_oscillator_hz,
        (unsigned long)PhaseAdc_GetOverrunCount());
    phase_app_print_relay_state("MENU_TEST");
}

static uint16_t phase_app_adc_peak_abs(const int16_t *samples)
{
    uint16_t index;
    uint16_t peak = 0U;

    if (samples == NULL)
    {
        return 0U;
    }
    for (index = 0U; index < PHASE_ADC_BLOCK_COUNT; ++index)
    {
        int32_t value = (int32_t)samples[index];
        uint32_t magnitude = (value < 0) ?
                             (uint32_t)(-value) :
                             (uint32_t)value;
        if (magnitude > (uint32_t)peak)
        {
            peak = (magnitude > 65535U) ? 65535U :
                   (uint16_t)magnitude;
        }
    }
    return peak;
}

static int16_t phase_app_clamp_i16(float value)
{
    if (value > 32767.0f)
    {
        return 32767;
    }
    if (value < -32768.0f)
    {
        return -32768;
    }
    return (int16_t)value;
}

static const PhaseAppCalibration_t *phase_app_get_calibration(void)
{
    return (s_adc_mode == PHASE_ADC_MODE_MIXED) ?
           &s_calibration_mixed : &s_calibration_direct;
}

static void phase_app_apply_sample_calibration(
    const PhaseAppCalibration_t *calibration)
{
    uint16_t index;

    for (index = 0U; index < PHASE_ADC_BLOCK_COUNT; ++index)
    {
        s_channel1[index] = phase_app_clamp_i16(
            (float)s_channel1[index] -
            calibration->channel1_zero_counts);
        s_channel2[index] = phase_app_clamp_i16(
            (float)s_channel2[index] -
            calibration->channel2_zero_counts);
    }
}

static void phase_app_refresh_calibration_status(void)
{
    PhaseCalibrationStatus_t status;

    PhaseCalibration_GetStatus(&status);
    s_measurement.calibration_valid = status.calibration_valid;
    s_measurement.learning_active = status.learning_active;
    s_measurement.learning_save_ok = status.save_ok;
    s_measurement.learning_point_index = status.point_index;
    s_measurement.learning_point_total = status.point_total;
    s_measurement.learning_sample_count = status.sample_count;
    s_measurement.learning_target_frequency_hz =
        status.target_frequency_hz;
    s_measurement.learned_phase_error_deg =
        status.last_phase_error_deg;
    s_measurement.learned_gain_error_db =
        status.last_gain_error_db;
}

static void phase_app_refresh_wiring_status(void)
{
    s_measurement.wiring_active = s_wiring_active;
    s_measurement.wiring_case_index = s_wiring_case_index;
    s_measurement.wiring_case_total = PHASE_APP_WIRING_CASE_COUNT;
    s_measurement.wiring_pass_count = s_wiring_pass_count;
    s_measurement.wiring_fail_count = s_wiring_fail_count;
    s_measurement.wiring_done =
        ((s_wiring_active == 0U) &&
         ((s_wiring_pass_count + s_wiring_fail_count) != 0U)) ?
        1U : 0U;
}

static void phase_app_submit_vofa_snapshot(void)
{
    PhaseVofaSnapshot_t snapshot;

    phase_app_refresh_calibration_status();
    phase_app_refresh_wiring_status();
    (void)memset(&snapshot, 0, sizeof(snapshot));
    snapshot.menu = (uint8_t)s_mode;
    snapshot.measure_state = (uint8_t)s_state;
    snapshot.capture_route = (uint8_t)s_capture_route;
    snapshot.adc_mode = (uint8_t)s_adc_mode;
    snapshot.frequency_valid = s_measurement.frequency_valid;
    snapshot.phase_valid = s_measurement.phase_valid;
    snapshot.frequency_hz = s_measurement.frequency_hz_precise;
    snapshot.capture_frequency_hz =
        s_measurement.divided_frequency_hz_precise;
    snapshot.channel1_amplitude =
        s_measurement.channel1_amplitude;
    snapshot.channel2_amplitude =
        s_measurement.channel2_amplitude;
    snapshot.phase_deg = s_measurement.phase_difference_deg;
    snapshot.gain_db = s_measurement.gain_db;
    snapshot.local_oscillator_hz =
        (s_local_oscillator_enabled != 0U) ?
        s_last_dds_frequency_hz : 0U;
    snapshot.adc_sample_rate_hz =
        s_measurement.adc_sample_rate_hz;
    snapshot.adc_overrun_count = PhaseAdc_GetOverrunCount();
    snapshot.calibration_valid =
        s_measurement.calibration_valid;
    snapshot.learning_active =
        s_measurement.learning_active;
    snapshot.learning_point_index =
        s_measurement.learning_point_index;
    snapshot.learning_point_total =
        s_measurement.learning_point_total;
    PhaseVofa_SubmitSnapshot(&snapshot);
}

static void phase_app_set_state(PhaseAppState_t state)
{
    s_state = state;
    s_measurement.measure_state = (uint8_t)state;
    s_measurement.capture_route = (uint8_t)s_capture_route;
    s_measurement.adc_mode = (uint8_t)s_adc_mode;
    s_measurement.active_menu = (uint8_t)s_mode;
    phase_app_submit_vofa_snapshot();
}

static void phase_app_publish_status(uint8_t status_code,
                                     const char *text)
{
    s_measurement.event = UI_EVENT_STATUS;
    s_measurement.status_code = status_code;
    (void)strncpy(s_measurement.status_text, text,
                  sizeof(s_measurement.status_text) - 1U);
    s_measurement.status_text[
        sizeof(s_measurement.status_text) - 1U] = '\0';
    phase_app_submit_vofa_snapshot();
    if (s_publish != NULL)
    {
        s_publish(&s_measurement);
    }
}

static void phase_app_publish_measurement(uint8_t force)
{
    TickType_t now = xTaskGetTickCount();

    if ((force == 0U) &&
        ((TickType_t)(now - s_last_ui_publish_tick) <
         pdMS_TO_TICKS(PHASE_APP_UI_PUBLISH_MS)))
    {
        phase_app_submit_vofa_snapshot();
        return;
    }

    s_last_ui_publish_tick = now;
    s_measurement.event = UI_EVENT_MEASUREMENT;
    s_measurement.adc_overrun_count =
        PhaseAdc_GetOverrunCount();
    phase_app_submit_vofa_snapshot();
    if (s_publish != NULL)
    {
        s_publish(&s_measurement);
    }
}

static void phase_app_configure_pipeline(void)
{
    PhasePipelineConfig_t config;

    PhasePipelineConfig_Default(&config);
    config.frequency.timer_tick_hz =
        PHASE_CAPTURE_TIMER_CLOCK_HZ;
    config.frequency.capture_cycles =
        PHASE_CAPTURE_EDGE_COUNT;
    config.frequency.input_min_hz = PHASE_MEASURE_MIN_HZ;
    config.frequency.input_max_hz = PHASE_MEASURE_MAX_HZ;
    config.fft.sample_rate_hz =
        PHASE_APP_MIXED_SAMPLE_RATE_HZ;
    config.fft.signal_frequency_hz =
        PHASE_APP_IF_FREQUENCY_HZ;
    config.fft.point_count = PHASE_ADC_BLOCK_COUNT;
    (void)PhasePipeline_Init(&g_phase_pipeline, &config);
}

static uint8_t phase_app_mode_requires_adc(void)
{
    return ((s_mode == PHASE_APP_MODE_PHASE) ||
            (s_mode == PHASE_APP_MODE_FFT) ||
            (s_mode == PHASE_APP_MODE_LEARNING) ||
            (s_mode == PHASE_APP_MODE_WIRING) ||
            (s_mode == PHASE_APP_MODE_VOFA)) ? 1U : 0U;
}

static uint8_t phase_app_set_adc_running(uint8_t enabled,
                                         uint32_t sample_rate_hz)
{
    if (enabled == 0U)
    {
        if (s_adc_running != 0U)
        {
            (void)PhaseAdc_Stop();
        }
        s_adc_running = 0U;
        s_adc_requested_rate_hz = 0U;
        s_measurement.adc_sample_rate_hz = 0U;
        s_discard_adc_blocks = 0U;
        return 1U;
    }

    if ((s_adc_running != 0U) &&
        (s_adc_requested_rate_hz == sample_rate_hz))
    {
        return 1U;
    }
    if (s_adc_running != 0U)
    {
        (void)PhaseAdc_Stop();
        s_adc_running = 0U;
    }

    phase_app_invalidate_phase();
    PhaseAdc_Flush();
    if (PhaseAdc_Start(sample_rate_hz) != HAL_OK)
    {
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "ADC START ERROR");
        return 0U;
    }

    s_adc_running = 1U;
    s_adc_requested_rate_hz = sample_rate_hz;
    s_measurement.adc_sample_rate_hz =
        PhaseAdc_GetSampleRateHz();
    PhasePipeline_SetSampleRate(
        &g_phase_pipeline, s_measurement.adc_sample_rate_hz);
    s_discard_adc_blocks = 1U;
    s_last_analysis_tick = 0U;
    s_last_vofa_block_tick = 0U;
    s_measurement.first_frame_discarded = 0U;
    phase_app_set_state(PHASE_APP_STATE_FLUSH_ADC);
    return 1U;
}

static void phase_app_wiring_reset_accumulators(void)
{
    s_wiring_sample_count = 0U;
    s_wiring_valid_sample_count = 0U;
    s_wiring_frequency_sum = 0ULL;
    s_wiring_channel1_sum = 0.0f;
    s_wiring_channel2_sum = 0.0f;
    s_wiring_phase_sum = 0.0f;
    s_wiring_channel1_peak = 0U;
    s_wiring_channel2_peak = 0U;
    s_wiring_adc_raw_reported = 0U;
    s_wiring_frequency_sampled = 0U;
}

static void phase_app_wiring_advance_case(void)
{
    if ((s_wiring_case_index + 1U) >= PHASE_APP_WIRING_CASE_COUNT)
    {
        phase_app_wiring_finish(0U);
        return;
    }

    ++s_wiring_case_index;
    if (phase_app_wiring_start_case() == 0U)
    {
        phase_app_wiring_finish(1U);
    }
}

static uint8_t phase_app_wiring_start_case(void)
{
    const PhaseAppWiringCase_t *test_case;
    PhaseRouteDiagnostics_t route_diagnostics;

    if ((s_wiring_active == 0U) ||
        (s_wiring_case_index >= PHASE_APP_WIRING_CASE_COUNT))
    {
        return 0U;
    }
    test_case = &s_wiring_cases[s_wiring_case_index];
    phase_app_wiring_reset_accumulators();

    (void)phase_app_set_adc_running(0U, 0U);
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    if (phase_app_disable_local_oscillator() == 0U)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "WIRE_TEST CASE %u/%u %s LO_OFF_ERROR",
            (unsigned)(s_wiring_case_index + 1U),
            (unsigned)PHASE_APP_WIRING_CASE_COUNT,
            test_case->name);
        return 0U;
    }

    (void)phase_app_apply_route(
        test_case->capture_route, test_case->adc_mode);
    if (s_state == PHASE_APP_STATE_ERROR)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "WIRE_TEST CASE %u/%u %s ROUTE_ERROR",
            (unsigned)(s_wiring_case_index + 1U),
            (unsigned)PHASE_APP_WIRING_CASE_COUNT,
            test_case->name);
        return 0U;
    }
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    PhaseRoute_GetDiagnostics(&route_diagnostics);
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST RELAY_GPIO PG9=%u PG10=%u PG11=%u PG12=%u PG13=%u PG14=%u PG15=%u OUT3=%u",
        (unsigned)route_diagnostics.out1_ctrl,
        (unsigned)route_diagnostics.out2_ctrl,
        (unsigned)route_diagnostics.out4_ctrl,
        (unsigned)route_diagnostics.out3_in6,
        (unsigned)route_diagnostics.out5_ctrl,
        (unsigned)route_diagnostics.out3_in5,
        (unsigned)route_diagnostics.out3_in7,
        (unsigned)route_diagnostics.out3_input);

    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_SET_LOCAL_OSC);
    phase_app_wait_dds_gap();
    if (AD9959_SetCalibrationPair(
            test_case->frequency_hz,
            PHASE_APP_WIRING_AMPLITUDE_MV,
            test_case->channel1_phase_deg,
            test_case->channel0_phase_deg) != HAL_OK)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "WIRE_TEST CASE %u/%u %s DDS_ERROR",
            (unsigned)(s_wiring_case_index + 1U),
            (unsigned)PHASE_APP_WIRING_CASE_COUNT,
            test_case->name);
        return 0U;
    }
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST DDS_PAIR TX=OK MODE=NORMAL BATCH_IO_UPDATE=1 CH1=%luHz/%ludeg CH0=%luHz/%ludeg CH2=OFF",
        (unsigned long)test_case->frequency_hz,
        (unsigned long)test_case->channel1_phase_deg,
        (unsigned long)test_case->frequency_hz,
        (unsigned long)test_case->channel0_phase_deg);

    s_last_dds_update_tick = xTaskGetTickCount();
    s_last_dds_frequency_hz = 0U;
    s_local_oscillator_enabled = 0U;
    s_measurement.local_oscillator_hz = 0U;
    vTaskDelay(pdMS_TO_TICKS(PHASE_APP_T_SOURCE_SETTLE_MS));

    /*
     * Route selection stops the ADC before switching relays.  The wiring
     * test used to restart only the frequency capture here, leaving
     * s_adc_running == 0 for every case.  That made the frequency counter
     * report valid data while the ADC/phase path could never produce a
     * sample (ADC_VALID=0, both amplitudes=0).  Prepare the acquisition now;
     * this also enables the 1.01 MHz high-side LO for the mixed case.
     */
    if (phase_app_prepare_acquisition(test_case->frequency_hz) == 0U)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "WIRE_TEST CASE %u/%u %s ADC_START_ERROR",
            (unsigned)(s_wiring_case_index + 1U),
            (unsigned)PHASE_APP_WIRING_CASE_COUNT,
            test_case->name);
        return 0U;
    }
    s_wiring_overrun_start = PhaseAdc_GetOverrunCount();
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST ADC_START OK RATE=%lu",
        (unsigned long)PhaseAdc_GetSampleRateHz());

    phase_app_set_state(PHASE_APP_STATE_MEASURE_FREQUENCY);
    if (phase_app_restart_capture() == 0U)
    {
        return 0U;
    }
    s_wiring_case_start_tick = xTaskGetTickCount();
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST CASE %u/%u %s START ROUTE=%u ADC=%u FREQ=%u PHASE1=%u PHASE0=%u",
        (unsigned)(s_wiring_case_index + 1U),
        (unsigned)PHASE_APP_WIRING_CASE_COUNT,
        test_case->name,
        (unsigned)test_case->capture_route,
        (unsigned)test_case->adc_mode,
        (unsigned)test_case->frequency_hz,
        (unsigned)test_case->channel1_phase_deg,
        (unsigned)test_case->channel0_phase_deg);
    phase_app_publish_status(1U, "WIRE CASE");
    return 1U;
}

static void phase_app_wiring_check_sample(void)
{
    const PhaseAppWiringCase_t *test_case;
    uint32_t frequency_average;
    uint32_t frequency_delta;
    uint32_t expected_lo_hz;
    uint32_t lo_delta;
    float channel1_average;
    float channel2_average;
    float phase_average;
    float phase_delta;
    float phase_step;
    uint8_t frequency_ok;
    uint8_t channel1_ok;
    uint8_t channel2_ok;
    uint8_t route_ok;
    uint8_t lo_ok;
    uint8_t phase_ok;
    uint8_t clipping_ok;
    uint8_t pass;

    if ((s_wiring_active == 0U) ||
        (s_wiring_frequency_sampled == 0U))
    {
        return;
    }

    ++s_wiring_sample_count;
    if (s_measurement.phase_valid != 0U)
    {
        ++s_wiring_valid_sample_count;
    }
    s_wiring_frequency_sum +=
        (uint64_t)s_measurement.frequency_hz;
    s_wiring_channel1_sum +=
        s_measurement.channel1_amplitude;
    s_wiring_channel2_sum +=
        s_measurement.channel2_amplitude;
    s_wiring_phase_sum +=
        s_measurement.phase_difference_deg;
    if (s_wiring_sample_count < PHASE_APP_WIRING_REQUIRED_SAMPLES)
    {
        return;
    }

    test_case = &s_wiring_cases[s_wiring_case_index];
    frequency_average = (uint32_t)(
        s_wiring_frequency_sum /
        (uint64_t)PHASE_APP_WIRING_REQUIRED_SAMPLES);
    frequency_delta = (frequency_average >=
                       test_case->frequency_hz) ?
                      (frequency_average - test_case->frequency_hz) :
                      (test_case->frequency_hz -
                       frequency_average);
    channel1_average = s_wiring_channel1_sum /
                       (float)PHASE_APP_WIRING_REQUIRED_SAMPLES;
    channel2_average = s_wiring_channel2_sum /
                       (float)PHASE_APP_WIRING_REQUIRED_SAMPLES;
    phase_average = s_wiring_phase_sum /
                    (float)PHASE_APP_WIRING_REQUIRED_SAMPLES;
    clipping_ok = ((s_wiring_channel1_peak <
                    PHASE_APP_WIRING_ADC_CLIP_COUNTS) &&
                   (s_wiring_channel2_peak <
                    PHASE_APP_WIRING_ADC_CLIP_COUNTS)) ? 1U : 0U;
    if ((s_wiring_case_index == 0U) &&
        (s_wiring_valid_sample_count ==
         PHASE_APP_WIRING_REQUIRED_SAMPLES) &&
        (clipping_ok != 0U))
    {
        s_wiring_direct_phase_reference_deg = phase_average;
        s_wiring_direct_phase_reference_valid = 1U;
    }
    phase_step = 0.0f;
    phase_delta = 0.0f;
    if (test_case->check_phase != 0U)
    {
        if (s_wiring_direct_phase_reference_valid != 0U)
        {
            phase_step = phase_app_normalize_degrees(
                phase_average -
                s_wiring_direct_phase_reference_deg);
            phase_delta = phase_app_normalize_degrees(
                phase_step - test_case->expected_phase_deg);
        }
        else
        {
            phase_delta = 360.0f;
        }
    }
    expected_lo_hz = (test_case->adc_mode == PHASE_ADC_MODE_MIXED) ?
                     (test_case->frequency_hz +
                      PHASE_APP_IF_FREQUENCY_HZ) : 0U;
    lo_delta = (s_measurement.local_oscillator_hz >= expected_lo_hz) ?
               (s_measurement.local_oscillator_hz - expected_lo_hz) :
               (expected_lo_hz - s_measurement.local_oscillator_hz);
    frequency_ok = (frequency_delta <=
                    PHASE_APP_WIRING_FREQUENCY_TOLERANCE_HZ) ? 1U : 0U;
    channel1_ok = (channel1_average >=
                   PHASE_APP_WIRING_MIN_AMPLITUDE_COUNTS) ? 1U : 0U;
    channel2_ok = (channel2_average >=
                   PHASE_APP_WIRING_MIN_AMPLITUDE_COUNTS) ? 1U : 0U;
    route_ok = ((s_capture_route == test_case->capture_route) &&
                (s_adc_mode == test_case->adc_mode)) ? 1U : 0U;
    lo_ok = (lo_delta <= PHASE_APP_WIRING_FREQUENCY_TOLERANCE_HZ) ?
            1U : 0U;
    if (test_case->check_phase == 0U)
    {
        phase_ok = 1U;
    }
    else
    {
        phase_ok =
            ((s_wiring_direct_phase_reference_valid != 0U) &&
             (fabsf(phase_delta) <=
              PHASE_APP_WIRING_PHASE_TOLERANCE_DEG)) ? 1U : 0U;
    }
    pass = (frequency_ok != 0U) &&
           (s_wiring_valid_sample_count ==
            PHASE_APP_WIRING_REQUIRED_SAMPLES) &&
           (channel1_ok != 0U) &&
           (channel2_ok != 0U) &&
           (route_ok != 0U) &&
           (lo_ok != 0U) &&
           (phase_ok != 0U) &&
           (clipping_ok != 0U);

    if (pass != 0U)
    {
        ++s_wiring_pass_count;
    }
    else
    {
        ++s_wiring_fail_count;
    }
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST RESULT %u/%u %s %s FREQ=%lu EXPECT=%lu FREQ_OK=%u ADC_VALID=%u/%u CH1_ADC=%s(%.1f) CH2_ADC=%s(%.1f) PHASE=%.1f STEP=%.1f PHASE_OK=%u PEAK=%u/%u CLIP=%s LO=%lu ROUTE=%s ADC_RATE=%lu OVR_DELTA=%lu",
        (unsigned)(s_wiring_case_index + 1U),
        (unsigned)PHASE_APP_WIRING_CASE_COUNT,
        test_case->name,
        (pass != 0U) ? "PASS" : "FAIL",
        (unsigned long)frequency_average,
        (unsigned long)test_case->frequency_hz,
        (unsigned)frequency_ok,
        (unsigned)s_wiring_valid_sample_count,
        (unsigned)PHASE_APP_WIRING_REQUIRED_SAMPLES,
        (channel1_ok != 0U) ? "PASS" : "FAIL",
        (double)channel1_average,
        (channel2_ok != 0U) ? "PASS" : "FAIL",
        (double)channel2_average,
        (double)phase_average,
        (double)phase_step,
        (unsigned)phase_ok,
        (unsigned)s_wiring_channel1_peak,
        (unsigned)s_wiring_channel2_peak,
        (clipping_ok != 0U) ? "PASS" : "FAIL",
        (unsigned long)s_measurement.local_oscillator_hz,
        (route_ok != 0U) ? "PASS" : "FAIL",
        (unsigned long)PhaseAdc_GetSampleRateHz(),
        (unsigned long)(
            PhaseAdc_GetOverrunCount() - s_wiring_overrun_start));
    phase_app_publish_status(pass, (pass != 0U) ?
                             "WIRE PASS" : "WIRE FAIL");
    phase_app_wiring_advance_case();
}

static void phase_app_wiring_finish(uint8_t aborted)
{
    HAL_StatusTypeDef dds_status;

    (void)phase_app_set_adc_running(0U, 0U);
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    if (phase_app_disable_local_oscillator() == 0U)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "WIRE_TEST LO_STOP_ERROR");
    }
    phase_app_wait_dds_gap();
    dds_status = AD9959_StopOutput();
    s_last_dds_update_tick = xTaskGetTickCount();
    s_last_dds_frequency_hz = 0U;
    s_local_oscillator_enabled = 0U;
    s_measurement.local_oscillator_hz = 0U;
    PhaseRoute_Out3DisconnectAll();
    PhaseRoute_SetMainRelays(
        PHASE_CAPTURE_ROUTE_RAW, PHASE_ADC_MODE_DIRECT);
    s_capture_route = PHASE_CAPTURE_ROUTE_RAW;
    s_adc_mode = PHASE_ADC_MODE_DIRECT;
    s_wiring_active = 0U;
    s_mode = PHASE_APP_MODE_IDLE;
    s_measurement.active_menu = (uint8_t)s_mode;
    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_IDLE);

    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST SUMMARY %s PASS=%u FAIL=%u DDS_STOP=%s",
        (aborted != 0U) ? "ABORT" : "DONE",
        (unsigned)s_wiring_pass_count,
        (unsigned)s_wiring_fail_count,
        (dds_status == HAL_OK) ? "PASS" : "FAIL");
    phase_app_publish_status(
        ((aborted == 0U) && (dds_status == HAL_OK) &&
         (s_wiring_fail_count == 0U)) ? 1U : 0U,
        (aborted != 0U) ? "WIRE TEST ABORT" : "WIRE TEST DONE");
}

static void phase_app_wiring_check_timeout(void)
{
    if ((s_wiring_active == 0U) ||
        (s_wiring_sample_count >= PHASE_APP_WIRING_REQUIRED_SAMPLES) ||
        ((TickType_t)(xTaskGetTickCount() -
                      s_wiring_case_start_tick) <
         pdMS_TO_TICKS(PHASE_APP_WIRING_CASE_TIMEOUT_MS)))
    {
        return;
    }

    ++s_wiring_fail_count;
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST RESULT %u/%u %s FAIL TIMEOUT FREQ_VALID=%u FREQ=%lu EXPECT=%lu ADC_VALID=%u CH1=%.1f CH2=%.1f ADC_RATE=%lu OVR_DELTA=%lu",
        (unsigned)(s_wiring_case_index + 1U),
        (unsigned)PHASE_APP_WIRING_CASE_COUNT,
        s_wiring_cases[s_wiring_case_index].name,
        (unsigned)s_measurement.frequency_valid,
        (unsigned long)s_measurement.frequency_hz,
        (unsigned long)s_wiring_cases[s_wiring_case_index].frequency_hz,
        (unsigned)s_measurement.phase_valid,
        (double)s_measurement.channel1_amplitude,
        (double)s_measurement.channel2_amplitude,
        (unsigned long)PhaseAdc_GetSampleRateHz(),
        (unsigned long)(
            PhaseAdc_GetOverrunCount() - s_wiring_overrun_start));
    phase_app_publish_status(0U, "WIRE TIMEOUT");
    phase_app_wiring_advance_case();
}

static void phase_app_invalidate_phase(void)
{
    s_measurement.phase_valid = 0U;
    s_measurement.first_frame_discarded = 0U;
    s_measurement.channel1_amplitude = 0.0f;
    s_measurement.channel2_amplitude = 0.0f;
    s_measurement.phase_difference_deg = 0.0f;
    s_measurement.gain_db = 0.0f;
    s_measurement.selected_bin = 0U;
}

static uint8_t phase_app_restart_capture_internal(
    uint8_t clear_frequency)
{
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    if (PhaseCapture_Start(&g_phase_capture) != HAL_OK)
    {
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "CAPTURE ERROR");
        return 0U;
    }

    s_capture_running = 1U;
    s_no_signal_published = 0U;
    s_last_frequency_tick = xTaskGetTickCount();
    s_last_capture_process_tick = 0U;
    if (clear_frequency != 0U)
    {
        s_measurement.frequency_hz = 0U;
        s_measurement.divided_frequency_hz = 0U;
        s_measurement.frequency_hz_precise = 0.0f;
        s_measurement.divided_frequency_hz_precise = 0.0f;
        s_measurement.period_ticks = 0U;
        s_measurement.capture_cycles = PHASE_CAPTURE_EDGE_COUNT;
        s_measurement.divider =
            PhaseRoute_GetDivider(s_capture_route);
        s_measurement.frequency_valid = 0U;
    }
    phase_app_invalidate_phase();
    phase_app_submit_vofa_snapshot();
    return 1U;
}

static uint8_t phase_app_restart_capture(void)
{
    return phase_app_restart_capture_internal(1U);
}

/*
 * OUT1 switching and capture-watchdog recovery are remeasurement
 * transactions.  Keep the last valid display value until the new route
 * produces a replacement instead of flashing an artificial 0 Hz.
 */
static uint8_t phase_app_restart_capture_keep_result(void)
{
    return phase_app_restart_capture_internal(0U);
}

static void phase_app_wait_dds_gap(void)
{
    TickType_t now;
    TickType_t elapsed;
    TickType_t minimum_gap;

    if (s_last_dds_update_tick == 0U)
    {
        return;
    }

    now = xTaskGetTickCount();
    elapsed = now - s_last_dds_update_tick;
    minimum_gap = pdMS_TO_TICKS(PHASE_APP_DDS_MIN_GAP_MS);
    if (elapsed < minimum_gap)
    {
        vTaskDelay(minimum_gap - elapsed);
    }
}

static uint8_t phase_app_update_local_oscillator(
    uint32_t input_frequency_hz)
{
    uint32_t local_frequency_hz;
    uint32_t difference_hz;

    if (input_frequency_hz == 0U)
    {
        return 0U;
    }
    /*
     * AD9959 CH2 reaches both AD835 LO inputs through AD8367. Reassert VGAIN
     * if another DAC user or a peripheral reset changed PA4.
     */
    if ((AD8367_VerifyDacRegister() == 0U) &&
        (AD8367_SetGainDb(
             PHASE_APP_AD8367_DEFAULT_GAIN_DB) != HAL_OK))
    {
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "AD8367 GAIN ERROR");
        return 0U;
    }
    local_frequency_hz =
        input_frequency_hz + PHASE_APP_IF_FREQUENCY_HZ;
    difference_hz =
        (local_frequency_hz >= s_last_dds_frequency_hz) ?
        (local_frequency_hz - s_last_dds_frequency_hz) :
        (s_last_dds_frequency_hz - local_frequency_hz);
    if ((s_local_oscillator_enabled != 0U) &&
        (difference_hz <= PHASE_APP_DDS_DEADBAND_HZ))
    {
        return 1U;
    }

    (void)phase_app_set_adc_running(0U, 0U);
    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_SET_LOCAL_OSC);
    phase_app_wait_dds_gap();
    if (AD9959_SetChannelOutput(
            PHASE_APP_DDS_LO_CHANNEL, local_frequency_hz,
            PHASE_APP_DDS_AMPLITUDE_MV, 0U, 1U) != HAL_OK)
    {
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "DDS LO ERROR");
        return 0U;
    }

    s_last_dds_frequency_hz = local_frequency_hz;
    s_last_dds_update_tick = xTaskGetTickCount();
    s_local_oscillator_enabled = 1U;
    s_measurement.local_oscillator_hz = local_frequency_hz;
    if (s_mode == PHASE_APP_MODE_WIRING)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "WIRE_TEST LO_CHAIN CH2=%luHz AD8367_VGAIN=%umV GAIN=%.1fdB DAC=%s",
            (unsigned long)local_frequency_hz,
            (unsigned)AD8367_GetControlVoltageMv(),
            (double)AD8367_GetGainDb(),
            (AD8367_VerifyDacRegister() != 0U) ?
                "PASS" : "FAIL");
    }
    vTaskDelay(pdMS_TO_TICKS(PHASE_APP_T_LO_SETTLE_MS));
    return 1U;
}

static uint8_t phase_app_disable_local_oscillator(void)
{
    uint32_t frequency_hz;

    if (s_local_oscillator_enabled == 0U)
    {
        s_measurement.local_oscillator_hz = 0U;
        return 1U;
    }

    frequency_hz = (s_last_dds_frequency_hz != 0U) ?
                   s_last_dds_frequency_hz : 1000U;
    (void)phase_app_set_adc_running(0U, 0U);
    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_SET_LOCAL_OSC);
    phase_app_wait_dds_gap();
    if (AD9959_SetChannelOutput(
            PHASE_APP_DDS_LO_CHANNEL, frequency_hz,
            0.0f, 0U, 0U) != HAL_OK)
    {
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "DDS STOP ERROR");
        return 0U;
    }

    s_last_dds_update_tick = xTaskGetTickCount();
    s_local_oscillator_enabled = 0U;
    s_measurement.local_oscillator_hz = 0U;
    return 1U;
}

static uint32_t phase_app_choose_direct_sample_rate(
    uint32_t frequency_hz)
{
    uint64_t desired;
    uint32_t quantized;

    desired = (uint64_t)frequency_hz *
              PHASE_APP_DIRECT_SAMPLES_PER_CYCLE;
    if (desired < PHASE_ADC_MIN_SAMPLE_RATE_HZ)
    {
        desired = PHASE_ADC_MIN_SAMPLE_RATE_HZ;
    }
    if (desired > PHASE_ADC_MAX_SAMPLE_RATE_HZ)
    {
        desired = PHASE_ADC_MAX_SAMPLE_RATE_HZ;
    }

    quantized = (uint32_t)(((desired + 500ULL) / 1000ULL) *
                           1000ULL);
    if (quantized < PHASE_ADC_MIN_SAMPLE_RATE_HZ)
    {
        quantized = PHASE_ADC_MIN_SAMPLE_RATE_HZ;
    }
    if (quantized > PHASE_ADC_MAX_SAMPLE_RATE_HZ)
    {
        quantized = PHASE_ADC_MAX_SAMPLE_RATE_HZ;
    }

    /*
     * Avoid restarting DMA for small capture jitter.  Exact-frequency DFT
     * remains valid with the existing sample clock.
     */
    if ((s_adc_requested_rate_hz != 0U) &&
        ((uint64_t)quantized * 100ULL >=
         (uint64_t)s_adc_requested_rate_hz * 90ULL) &&
        ((uint64_t)quantized * 100ULL <=
         (uint64_t)s_adc_requested_rate_hz * 110ULL))
    {
        return s_adc_requested_rate_hz;
    }
    return quantized;
}

static uint8_t phase_app_prepare_acquisition(
    uint32_t input_frequency_hz)
{
    uint32_t sample_rate_hz;
    uint32_t analysis_frequency_hz;

    if (phase_app_mode_requires_adc() == 0U)
    {
        (void)phase_app_set_adc_running(0U, 0U);
        if (phase_app_disable_local_oscillator() == 0U)
        {
            return 0U;
        }
        phase_app_set_state(PHASE_APP_STATE_MEASURE_FREQUENCY);
        return 1U;
    }

    if (s_adc_mode == PHASE_ADC_MODE_MIXED)
    {
        if (phase_app_update_local_oscillator(
                input_frequency_hz) == 0U)
        {
            return 0U;
        }
        analysis_frequency_hz =
            (input_frequency_hz >= s_last_dds_frequency_hz) ?
            (input_frequency_hz - s_last_dds_frequency_hz) :
            (s_last_dds_frequency_hz - input_frequency_hz);
        sample_rate_hz = PHASE_APP_MIXED_SAMPLE_RATE_HZ;
    }
    else
    {
        if (phase_app_disable_local_oscillator() == 0U)
        {
            return 0U;
        }
        analysis_frequency_hz = input_frequency_hz;
        sample_rate_hz =
            phase_app_choose_direct_sample_rate(input_frequency_hz);
    }

    if ((analysis_frequency_hz == 0U) ||
        ((uint64_t)analysis_frequency_hz * 2ULL >=
         (uint64_t)sample_rate_hz))
    {
        (void)phase_app_set_adc_running(0U, 0U);
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "ADC NYQUIST ERROR");
        return 0U;
    }

    PhasePipeline_SetSignalFrequency(
        &g_phase_pipeline, analysis_frequency_hz);
    if (phase_app_set_adc_running(
            1U, sample_rate_hz) == 0U)
    {
        return 0U;
    }

    if (s_discard_adc_blocks == 0U)
    {
        phase_app_set_state(PHASE_APP_STATE_ACQUIRE_PHASE);
    }
    return 1U;
}

/*
 * Returns 1 when OUT1 changed and a fresh capture is now mandatory.
 */
static uint8_t phase_app_apply_route(
    PhaseCaptureRoute_t target_capture_route,
    PhaseAdcMode_t target_adc_mode)
{
    uint8_t capture_changed;
    uint8_t main_route_changed;
    uint8_t out3_changed;
    PhaseOut3Input_t target_out3;

    capture_changed =
        (target_capture_route != s_capture_route) ? 1U : 0U;
    main_route_changed =
        ((capture_changed != 0U) ||
         (target_adc_mode != s_adc_mode)) ? 1U : 0U;
    target_out3 = PhaseRoute_GetOut3Target(target_adc_mode);
    out3_changed =
        (PhaseRoute_GetOut3Input() != target_out3) ? 1U : 0U;
    if ((main_route_changed == 0U) && (out3_changed == 0U))
    {
        return 0U;
    }

    (void)phase_app_set_adc_running(0U, 0U);
    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_SWITCH_RELAYS);
    if ((capture_changed != 0U) && (s_capture_running != 0U))
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }

    if (out3_changed != 0U)
    {
        PhaseRoute_Out3DisconnectAll();
        phase_app_set_state(
            PHASE_APP_STATE_WAIT_RELAY_SETTLE);
        vTaskDelay(pdMS_TO_TICKS(PHASE_APP_T_BREAK_MS));
    }

    PhaseRoute_SetMainRelays(
        target_capture_route, target_adc_mode);
    if (out3_changed != 0U)
    {
        if (PhaseRoute_Out3ConnectOnly(target_out3) != HAL_OK)
        {
            phase_app_set_state(PHASE_APP_STATE_ERROR);
            phase_app_publish_status(0U, "OUT3 ROUTE ERROR");
            return 0U;
        }
    }

    phase_app_set_state(PHASE_APP_STATE_WAIT_RELAY_SETTLE);
    vTaskDelay(pdMS_TO_TICKS(
        PHASE_APP_T_MAKE_MS +
        PHASE_APP_T_ANALOG_SETTLE_MS));

    PhaseAdc_Flush();
    (void)memset(
        s_channel1, 0,
        PHASE_ADC_BLOCK_COUNT * sizeof(s_channel1[0]));
    (void)memset(
        s_channel2, 0,
        PHASE_ADC_BLOCK_COUNT * sizeof(s_channel2[0]));
    s_capture_route = target_capture_route;
    s_adc_mode = target_adc_mode;
    s_measurement.capture_route = (uint8_t)s_capture_route;
    s_measurement.adc_mode = (uint8_t)s_adc_mode;
    s_measurement.divider =
        PhaseRoute_GetDivider(s_capture_route);

    if (capture_changed != 0U)
    {
        phase_app_set_state(PHASE_APP_STATE_REMEASURE);
        if (phase_app_restart_capture() == 0U)
        {
            return 0U;
        }
        phase_app_set_state(PHASE_APP_STATE_REMEASURE);
        return 1U;
    }
    return 0U;
}

/*
 * Menu 1 owns OUT1 only.  It must never energize OUT3 or run the
 * 190/210 kHz ADC route decision while measuring frequency.
 */
static uint8_t phase_app_apply_frequency_route(
    PhaseCaptureRoute_t target_capture_route)
{
    if (target_capture_route == s_capture_route)
    {
        return 0U;
    }

    (void)phase_app_set_adc_running(0U, 0U);
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_SWITCH_RELAYS);
    PhaseRoute_SetMainRelays(
        target_capture_route, PHASE_ADC_MODE_DIRECT);
    phase_app_set_state(PHASE_APP_STATE_WAIT_RELAY_SETTLE);
    vTaskDelay(pdMS_TO_TICKS(
        PHASE_APP_T_MAKE_MS +
        PHASE_APP_T_ANALOG_SETTLE_MS));

    s_capture_route = target_capture_route;
    s_adc_mode = PHASE_ADC_MODE_DIRECT;
    s_measurement.capture_route = (uint8_t)s_capture_route;
    s_measurement.adc_mode = (uint8_t)s_adc_mode;
    s_measurement.divider =
        PhaseRoute_GetDivider(s_capture_route);
    phase_app_set_state(PHASE_APP_STATE_REMEASURE);
    if (phase_app_restart_capture() == 0U)
    {
        return 0U;
    }
    s_measurement.status_code = 1U;
    (void)strcpy(s_measurement.status_text, "REMEASURE");
    phase_app_publish_measurement(1U);
    phase_app_print_relay_state("MENU_TEST M1 ROUTE_CHANGE");
    return 1U;
}

static void phase_app_start_measurement_mode(
    PhaseAppMode_t mode, const char *status)
{
    if (s_wiring_active != 0U)
    {
        phase_app_wiring_finish(1U);
    }
    if (PhaseCalibration_IsLearning() != 0U)
    {
        PhaseCalibration_AbortLearning();
    }
    /* VOFA is enabled only by ADC-backed measurement menus below. */
    PhaseVofa_SetOutputEnabled(0U);
    (void)phase_app_set_adc_running(0U, 0U);
    if (((mode == PHASE_APP_MODE_FREQUENCY) ||
         (mode == PHASE_APP_MODE_SYSTEM)) &&
        (phase_app_disable_local_oscillator() == 0U))
    {
        return;
    }
    if ((mode == PHASE_APP_MODE_FREQUENCY) ||
        (mode == PHASE_APP_MODE_SYSTEM))
    {
        PhaseRoute_Out3DisconnectAll();
        PhaseRoute_SetMainRelays(
            s_capture_route, PHASE_ADC_MODE_DIRECT);
        s_adc_mode = PHASE_ADC_MODE_DIRECT;
        s_measurement.adc_mode = (uint8_t)s_adc_mode;
    }
    s_mode = mode;
    s_measurement.active_menu = (uint8_t)mode;
    phase_app_menu_test_begin(mode);
    if ((mode == PHASE_APP_MODE_PHASE) ||
        (mode == PHASE_APP_MODE_FFT))
    {
        PhaseVofa_SetOutputEnabled(1U);
    }
    phase_app_set_state(PHASE_APP_STATE_MEASURE_FREQUENCY);
    if (phase_app_restart_capture() == 0U)
    {
        return;
    }
    phase_app_publish_status(1U, status);
    phase_app_print_relay_state("MENU_TEST");
}

static uint8_t phase_app_learning_start_current_point(void)
{
    uint32_t target_frequency_hz =
        PhaseCalibration_GetTargetFrequency();
    PhaseCalibrationStatus_t learning_status;
    PhaseCaptureRoute_t target_capture_route;
    PhaseAdcMode_t target_adc_mode;

    if (target_frequency_hz == 0U)
    {
        return 0U;
    }

    target_capture_route =
        (target_frequency_hz > PHASE_ROUTE_CAPTURE_UP_HZ) ?
        PHASE_CAPTURE_ROUTE_DIV32 : PHASE_CAPTURE_ROUTE_RAW;
    target_adc_mode =
        (target_frequency_hz > PHASE_ROUTE_MIX_UP_HZ) ?
        PHASE_ADC_MODE_MIXED : PHASE_ADC_MODE_DIRECT;
    PhaseCalibration_GetStatus(&learning_status);
    (void)DebugUart6_TextPrintfBlocking(
        "MENU_TEST M4 POINT %u/%u START FREQ=%lu ROUTE=%s/%s",
        (unsigned)(learning_status.point_index + 1U),
        (unsigned)learning_status.point_total,
        (unsigned long)target_frequency_hz,
        (target_capture_route == PHASE_CAPTURE_ROUTE_DIV32) ?
            "DIV32" : "RAW",
        (target_adc_mode == PHASE_ADC_MODE_MIXED) ?
            "MIXED" : "DIRECT");

    (void)phase_app_set_adc_running(0U, 0U);
    (void)phase_app_apply_route(
        target_capture_route, target_adc_mode);
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }

    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_SET_LOCAL_OSC);
    phase_app_wait_dds_gap();
    if (AD9959_SetCalibrationPair(
            target_frequency_hz,
            PHASE_APP_LEARNING_AMPLITUDE_MV,
            0U, 0U) != HAL_OK)
    {
        PhaseCalibration_AbortLearning();
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "LEARN DDS ERROR");
        return 0U;
    }

    s_last_dds_update_tick = xTaskGetTickCount();
    s_last_dds_frequency_hz = 0U;
    s_local_oscillator_enabled = 0U;
    s_measurement.local_oscillator_hz = 0U;
    vTaskDelay(pdMS_TO_TICKS(PHASE_APP_T_SOURCE_SETTLE_MS));

    phase_app_set_state(PHASE_APP_STATE_MEASURE_FREQUENCY);
    if (phase_app_restart_capture() == 0U)
    {
        PhaseCalibration_AbortLearning();
        return 0U;
    }
    phase_app_publish_status(1U, "LEARNING");
    return 1U;
}

static void phase_app_start_learning_mode(void)
{
    if (s_wiring_active != 0U)
    {
        phase_app_wiring_finish(1U);
    }
    PhaseCalibration_StartLearning();
    s_mode = PHASE_APP_MODE_LEARNING;
    s_measurement.active_menu = (uint8_t)s_mode;
    phase_app_menu_test_begin(s_mode);
    PhaseVofa_SetOutputEnabled(1U);
    if (phase_app_learning_start_current_point() == 0U)
    {
        PhaseCalibration_AbortLearning();
    }
}

/*
 * The F4 does not have an AD9959 read-back or analogue feedback pin.  This
 * helper therefore verifies the complete F4-side UART transaction and prints
 * a deterministic probe plan; an oscilloscope/frequency counter is still
 * required to prove that the three physical SMA outputs are present.
 */
static uint8_t phase_app_ad9959_test_channel(
    uint8_t channel,
    uint32_t frequency_hz,
    uint32_t phase_deg,
    float amplitude_mv,
    uint8_t output)
{
    HAL_StatusTypeDef status;

    status = AD9959_SetChannelOutput(
        channel, frequency_hz, amplitude_mv, phase_deg, output);
    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST CH%u TX=%s FREQ=%lu PHASE=%lu AMP=%.0fmV OUT=%u",
        (unsigned)channel,
        (status == HAL_OK) ? "OK" : "ERROR",
        (unsigned long)frequency_hz,
        (unsigned long)phase_deg,
        (double)amplitude_mv,
        (unsigned)output);
    return (status == HAL_OK) ? 1U : 0U;
}

static uint8_t phase_app_ad9959_output_test(void)
{
    uint8_t pass_count = 0U;

    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST BEGIN F4_USART1->F1_USART3 BAUD=115200");
    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST MAP PHYSICAL_CH0=100kHz PHYSICAL_CH1=200kHz PHYSICAL_CH2=300kHz");
    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST NOTE TX=F4_UART_FRAME_ONLY NO_DDS_READBACK");
    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST NOTE F1_REQUIRES_BATCH_FLAG_FIRMWARE; OLD_F1_BOOT_WAVE=1MHz");

    pass_count += phase_app_ad9959_test_channel(
        0U, 100000U, 0U, 250.0f, 1U);
    HAL_Delay(AD9959_FRAME_GAP_MS);
    pass_count += phase_app_ad9959_test_channel(
        1U, 200000U, 90U, 250.0f, 1U);
    HAL_Delay(AD9959_FRAME_GAP_MS);
    pass_count += phase_app_ad9959_test_channel(
        2U, 300000U, 180U, 250.0f, 1U);
    HAL_Delay(AD9959_FRAME_GAP_MS);
    pass_count += phase_app_ad9959_test_channel(
        3U, 100000U, 0U, 0.0f, 0U);

    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST SUMMARY TX_PASS=%u TX_FAIL=%u",
        (unsigned)pass_count,
        (unsigned)(4U - pass_count));
    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST PROBE CH0=100kHz CH1=200kHz CH2=300kHz CH3=OFF");
    (void)DebugUart6_TextPrintfBlocking(
        "AD9959_TEST CHECK SCOPE_OR_COUNTER_AT_EACH_DDS_OUTPUT");
    return (pass_count == 4U) ? 1U : 0U;
}

static void phase_app_start_wiring_test(void)
{
    if (PhaseCalibration_IsLearning() != 0U)
    {
        PhaseCalibration_AbortLearning();
    }
    if (s_wiring_active != 0U)
    {
        phase_app_wiring_finish(1U);
    }

    (void)phase_app_set_adc_running(0U, 0U);
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    (void)phase_app_disable_local_oscillator();
    s_mode = PHASE_APP_MODE_WIRING;
    s_measurement.active_menu = (uint8_t)s_mode;
    phase_app_menu_test_begin(s_mode);
    s_wiring_active = 1U;
    s_wiring_case_index = 0U;
    s_wiring_pass_count = 0U;
    s_wiring_fail_count = 0U;
    s_wiring_direct_phase_reference_deg = 0.0f;
    s_wiring_direct_phase_reference_valid = 0U;
    PhaseVofa_SetOutputEnabled(0U);

    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST BEGIN USART6=OK");
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST MAP AD9959_CH1->INPUT_CH1 AD9959_CH0->INPUT_CH2 AD9959_CH2=HIGH_LO");
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST ADC_INPUTS ADC1=PA1 ADC2=PA2");
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST ROUTES OUT1=IN1_RAW/IN2_DIV32 OUT2=IN3/IN4 OUT3=IN5/IN6 OUT4=IN8/IN9 OUT5=IN10/IN11");
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST RELAY_PINS OUT1=PG9 OUT2=PG10 OUT3_IN6=PG12 OUT4=PG11 OUT5=PG13 OUT3_IN5=PG14 OUT3_IN7=PG15");
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST WARNING RELAY_PINMAP_AND_ACTIVE_LEVEL_REQUIRE_SCHEMATIC_CONFIRMATION");
    (void)DebugUart6_TextPrintfBlocking(
        "WIRE_TEST NOTE OUT3_IN7_AND_AD9959_CH3=UNUSED_BY_DESIGN");
    (void)DebugUart6_TextPrintfBlocking(
        "AD8367_TEST DAC=%s PIN=PA4 VGAIN=%umV CODE=%u MODE=%s TARGET_GAIN=%.1fdB",
        (AD8367_VerifyDacRegister() != 0U) ? "PASS" : "FAIL",
        (unsigned)AD8367_GetControlVoltageMv(),
        (unsigned)AD8367_GetDacCode(),
        (AD8367_GetGainMode() ==
         AD8367_GAIN_INCREASES_WITH_VOLTAGE) ? "GAIN_UP" : "GAIN_DOWN",
        (double)AD8367_GetGainDb());
    (void)DebugUart6_TextPrintfBlocking(
        "AD8367_TEST NOTE REGISTER_PASS_REQUIRES_MULTIMETER_PA4_OR_VGAIN_FOR_ANALOG_CONFIRM");
    (void)DebugUart6_TextPrintfBlocking(
        "AD8367_TEST NOTE VGAIN=MCU_PA4 ENBL_AND_MODE=HARDWARE_LEVELS_REQUIRE_CONFIRMATION");
    (void)phase_app_ad9959_output_test();
    if (phase_app_wiring_start_case() == 0U)
    {
        ++s_wiring_fail_count;
        phase_app_wiring_finish(1U);
    }
}

static void phase_app_finish_learning(void)
{
    PhaseCalibrationStatus_t status;
    HAL_StatusTypeDef dds_status;

    (void)phase_app_set_adc_running(0U, 0U);
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    phase_app_wait_dds_gap();
    dds_status = AD9959_StopOutput();
    s_last_dds_update_tick = xTaskGetTickCount();
    s_last_dds_frequency_hz = 0U;
    s_local_oscillator_enabled = 0U;
    s_measurement.local_oscillator_hz = 0U;
    PhaseVofa_SetOutputEnabled(0U);
    PhaseRoute_Out3DisconnectAll();
    PhaseRoute_SetMainRelays(
        PHASE_CAPTURE_ROUTE_RAW, PHASE_ADC_MODE_DIRECT);
    s_capture_route = PHASE_CAPTURE_ROUTE_RAW;
    s_adc_mode = PHASE_ADC_MODE_DIRECT;
    s_mode = PHASE_APP_MODE_IDLE;
    s_measurement.active_menu = (uint8_t)s_mode;
    (void)PhaseCalibration_Save();
    phase_app_invalidate_phase();
    phase_app_set_state(PHASE_APP_STATE_IDLE);

    PhaseCalibration_GetStatus(&status);
    if ((dds_status == HAL_OK) && (status.save_ok != 0U))
    {
        s_menu_test_result_reported = 1U;
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M4 RESULT PASS POINTS=%u SAVE=PASS CAL_VALID=%u",
            (unsigned)status.point_total,
            (unsigned)status.calibration_valid);
        phase_app_publish_status(1U, "LEARNING SAVED");
    }
    else if (dds_status != HAL_OK)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M4 RESULT FAIL REASON=DDS_STOP");
        phase_app_publish_status(0U, "DDS STOP ERROR");
    }
    else
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M4 RESULT FAIL REASON=FLASH_SAVE");
        phase_app_publish_status(0U, "CAL SAVE ERROR");
    }
}

static void phase_app_process_frequency(uint32_t period_ticks,
                                        uint32_t capture_cycles)
{
    uint32_t divider;
    uint32_t acquisition_frequency_hz;
    uint32_t measured_frequency_hz;
    PhaseCaptureRoute_t target_capture_route;
    PhaseAdcMode_t target_adc_mode;
    const PhasePipelineResult_t *result;
    TickType_t now;

    divider = PhaseRoute_GetDivider(s_capture_route);
    (void)PhasePipeline_ProcessCapturedPeriod(
        &g_phase_pipeline, period_ticks, capture_cycles, divider);
    result = PhasePipeline_GetResult(&g_phase_pipeline);
    if ((result == NULL) || (result->frequency_valid == 0U))
    {
        return;
    }
    measured_frequency_hz =
        result->frequency.input_frequency_hz;
    now = xTaskGetTickCount();

    /*
     * The unloaded raw comparator path has been observed producing a stable
     * 5-6 MHz waveform even with no generator attached.  A real CH1 signal
     * must also appear after OUT1 selects the physical /32 path.  Treat the
     * raw result as a candidate and publish it only after that independent
     * route agrees.
     */
    if ((s_mode == PHASE_APP_MODE_FREQUENCY) &&
        (s_capture_route == PHASE_CAPTURE_ROUTE_RAW) &&
        (phase_app_m1_needs_div32_verification(
             measured_frequency_hz) != 0U))
    {
        if ((s_m1_verify_failed != 0U) &&
            ((TickType_t)(now - s_m1_verify_failure_tick) <
             pdMS_TO_TICKS(PHASE_APP_M1_VERIFY_RETRY_MS)))
        {
            /*
             * Keep TIM2 in fast DMA mode while rejecting the known raw-only
             * oscillation.  A different-frequency source is still accepted
             * immediately; this band is retried periodically for hot-plug.
             */
            s_last_frequency_tick = now;
            if (s_no_signal_published == 0U)
            {
                phase_app_m1_publish_wait_signal();
            }
            return;
        }

        s_m1_verify_pending = 1U;
        s_m1_div32_verified = 0U;
        s_m1_verify_failed = 0U;
        s_m1_raw_candidate_hz = measured_frequency_hz;
        s_m1_verify_start_tick = now;
        s_last_frequency_tick = now;
        s_no_signal_published = 0U;
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M1 VERIFY BEGIN RAW=%luHz ROUTE=DIV32",
            (unsigned long)s_m1_raw_candidate_hz);
        phase_app_set_state(PHASE_APP_STATE_DECIDE_ROUTE);
        (void)phase_app_apply_frequency_route(
            PHASE_CAPTURE_ROUTE_DIV32);
        if (s_state != PHASE_APP_STATE_ERROR)
        {
            s_measurement.status_code = 1U;
            (void)strcpy(
                s_measurement.status_text, "VERIFY SIGNAL");
            phase_app_publish_measurement(1U);
        }
        return;
    }

    if ((s_mode == PHASE_APP_MODE_FREQUENCY) &&
        (s_capture_route == PHASE_CAPTURE_ROUTE_DIV32) &&
        (s_m1_verify_pending != 0U))
    {
        if (phase_app_m1_frequency_matches(
                s_m1_raw_candidate_hz,
                measured_frequency_hz) == 0U)
        {
            (void)DebugUart6_TextPrintfBlocking(
                "MENU_TEST M1 VERIFY FAIL REASON=MISMATCH RAW=%luHz DIV32=%luHz",
                (unsigned long)s_m1_raw_candidate_hz,
                (unsigned long)measured_frequency_hz);
            s_m1_verify_pending = 0U;
            s_m1_div32_verified = 0U;
            s_m1_verify_failed = 1U;
            s_m1_verify_failure_tick = now;
            (void)phase_app_apply_frequency_route(
                PHASE_CAPTURE_ROUTE_RAW);
            s_last_frequency_tick = now;
            phase_app_m1_publish_wait_signal();
            return;
        }

        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M1 VERIFY PASS RAW=%luHz DIV32=%luHz",
            (unsigned long)s_m1_raw_candidate_hz,
            (unsigned long)measured_frequency_hz);
        s_m1_verify_pending = 0U;
        s_m1_div32_verified = 1U;
        s_m1_verify_failed = 0U;
    }
    else if ((s_mode == PHASE_APP_MODE_FREQUENCY) &&
             (s_capture_route == PHASE_CAPTURE_ROUTE_DIV32) &&
             (phase_app_m1_needs_div32_verification(
                  measured_frequency_hz) != 0U))
    {
        /* Starting Menu 1 while OUT1 is already /32 is itself confirmation. */
        s_m1_div32_verified = 1U;
        s_m1_verify_failed = 0U;
    }
    else if ((s_mode == PHASE_APP_MODE_FREQUENCY) &&
             (phase_app_m1_needs_div32_verification(
                  measured_frequency_hz) == 0U))
    {
        s_m1_div32_verified = 0U;
    }

    s_capture_recovery_attempted = 0U;
    s_capture_watchdog_restart_attempted = 0U;

    s_measurement.frequency_hz =
        result->frequency.input_frequency_hz;
    s_measurement.divided_frequency_hz =
        result->frequency.divided_frequency_hz;
    s_measurement.frequency_hz_precise =
        result->frequency.input_frequency_hz_f;
    s_measurement.divided_frequency_hz_precise =
        result->frequency.divided_frequency_hz_f;
    s_measurement.divider = result->frequency.divider;
    s_measurement.period_ticks =
        result->frequency.period_ticks;
    s_measurement.capture_cycles = capture_cycles;
    s_measurement.frequency_valid = 1U;
    s_measurement.status_code = 1U;
    (void)strcpy(s_measurement.status_text, "SIGNAL VALID");
    s_no_signal_published = 0U;
    s_last_frequency_tick = xTaskGetTickCount();

    phase_app_set_state(PHASE_APP_STATE_DECIDE_ROUTE);
    target_capture_route = PhaseRoute_DecideCapture(
        s_capture_route, result->frequency.input_frequency_hz);
    if ((s_mode == PHASE_APP_MODE_FREQUENCY) &&
        (s_m1_div32_verified != 0U) &&
        (phase_app_m1_needs_div32_verification(
             result->frequency.input_frequency_hz) != 0U))
    {
        /*
         * Keep the independently verified 4.0-4.9 MHz signal on /32.  Moving
         * straight back to RAW here would immediately start another probe.
         */
        target_capture_route = PHASE_CAPTURE_ROUTE_DIV32;
    }
    if ((s_mode == PHASE_APP_MODE_FREQUENCY) ||
        (s_mode == PHASE_APP_MODE_SYSTEM))
    {
        if (phase_app_apply_frequency_route(
                target_capture_route) != 0U)
        {
            return;
        }
        phase_app_set_state(PHASE_APP_STATE_MEASURE_FREQUENCY);
        phase_app_publish_measurement(0U);
        if ((s_mode == PHASE_APP_MODE_FREQUENCY) &&
            (s_menu_test_result_reported == 0U))
        {
            s_menu_test_result_reported = 1U;
            (void)DebugUart6_TextPrintfBlocking(
                "MENU_TEST M1 RESULT PASS FREQ=%lu CAPTURE=%lu DIV=%lu ADC=OFF LO=OFF",
                (unsigned long)s_measurement.frequency_hz,
                (unsigned long)s_measurement.divided_frequency_hz,
                (unsigned long)s_measurement.divider);
            phase_app_print_relay_state("MENU_TEST");
        }
        return;
    }
    target_adc_mode = PhaseRoute_DecideAdc(
        s_adc_mode, result->frequency.input_frequency_hz);
    acquisition_frequency_hz =
        result->frequency.input_frequency_hz;
    if ((s_wiring_active != 0U) &&
        (s_wiring_case_index < PHASE_APP_WIRING_CASE_COUNT))
    {
        s_wiring_frequency_sampled = 1U;
        target_capture_route =
            s_wiring_cases[s_wiring_case_index].capture_route;
        target_adc_mode =
            s_wiring_cases[s_wiring_case_index].adc_mode;
        /*
         * Menu 5 drives a known DDS test frequency.  A wrong capture value is
         * itself a wiring-test failure and must not be allowed to stop or
         * retune the ADC before its signal path has been evaluated.
         */
        acquisition_frequency_hz =
            s_wiring_cases[s_wiring_case_index].frequency_hz;
    }

    if (phase_app_apply_route(
            target_capture_route, target_adc_mode) != 0U)
    {
        return;
    }

    if (phase_app_prepare_acquisition(
            acquisition_frequency_hz) == 0U)
    {
        return;
    }
    phase_app_publish_measurement(0U);
}

static uint8_t phase_app_process_adc(void)
{
    const PhaseAppCalibration_t *calibration;
    const PhasePipelineResult_t *result;
    PhaseCalibrationLearnEvent_t learn_event =
        PHASE_CALIBRATION_LEARN_NONE;
    TickType_t now;
    uint8_t submit_vofa_block = 0U;
    uint16_t channel1_peak;
    uint16_t channel2_peak;

    if ((s_adc_running == 0U) ||
        (PhaseAdc_TakeBlock(
            s_channel1, s_channel2,
            PHASE_ADC_BLOCK_COUNT) != HAL_OK))
    {
        return 0U;
    }

    if (s_mode == PHASE_APP_MODE_WIRING)
    {
        channel1_peak = phase_app_adc_peak_abs(s_channel1);
        channel2_peak = phase_app_adc_peak_abs(s_channel2);
        if (channel1_peak > s_wiring_channel1_peak)
        {
            s_wiring_channel1_peak = channel1_peak;
        }
        if (channel2_peak > s_wiring_channel2_peak)
        {
            s_wiring_channel2_peak = channel2_peak;
        }
        if (s_wiring_adc_raw_reported == 0U)
        {
            (void)DebugUart6_TextPrintfBlocking(
                "WIRE_TEST ADC_RAW CH1_PEAK=%u CH2_PEAK=%u S0=%d,%d CLIP_LIMIT=%u",
                (unsigned)channel1_peak,
                (unsigned)channel2_peak,
                (int)s_channel1[0], (int)s_channel2[0],
                (unsigned)PHASE_APP_WIRING_ADC_CLIP_COUNTS);
            s_wiring_adc_raw_reported = 1U;
        }
    }

    if (s_discard_adc_blocks != 0U)
    {
        --s_discard_adc_blocks;
        s_measurement.first_frame_discarded = 1U;
        phase_app_set_state(PHASE_APP_STATE_ACQUIRE_PHASE);
        return 1U;
    }

    now = xTaskGetTickCount();
    if ((s_last_analysis_tick != 0U) &&
        ((TickType_t)(now - s_last_analysis_tick) <
         pdMS_TO_TICKS(PHASE_APP_ANALYSIS_PERIOD_MS)))
    {
        return 1U;
    }
    s_last_analysis_tick = now;

    calibration = phase_app_get_calibration();
    phase_app_apply_sample_calibration(calibration);
    if ((s_last_vofa_block_tick == 0U) ||
        ((TickType_t)(now - s_last_vofa_block_tick) >=
         pdMS_TO_TICKS(PHASE_APP_VOFA_BLOCK_PERIOD_MS)))
    {
        s_last_vofa_block_tick = now;
        submit_vofa_block = 1U;
    }
    if (PhasePipeline_ProcessAdcBlock(
            &g_phase_pipeline,
            s_channel1, s_channel2) == 0U)
    {
        s_measurement.phase_valid = 0U;
        phase_app_submit_vofa_snapshot();
        if (submit_vofa_block != 0U)
        {
            PhaseVofa_SubmitBlock(
                s_channel1, s_channel2,
                PHASE_ADC_BLOCK_COUNT);
        }
        return 1U;
    }

    result = PhasePipeline_GetResult(&g_phase_pipeline);
    if (result == NULL)
    {
        if (submit_vofa_block != 0U)
        {
            PhaseVofa_SubmitBlock(
                s_channel1, s_channel2,
                PHASE_ADC_BLOCK_COUNT);
        }
        return 1U;
    }

    s_measurement.channel1_amplitude =
        result->phase.channel1_amplitude *
        calibration->channel1_gain;
    s_measurement.channel2_amplitude =
        result->phase.channel2_amplitude *
        calibration->channel2_gain;
    /*
     * CH2 uses high-side injection: f_LO = f_input + 10 kHz.
     * A real mixer therefore reverses the phase of the low-pass difference
     * term. Negate the analyzed phase so the public result remains CH2-CH1.
     */
    s_measurement.phase_difference_deg =
        phase_app_normalize_degrees(
            ((s_adc_mode == PHASE_ADC_MODE_MIXED) ?
             -result->phase.phase_difference_deg :
             result->phase.phase_difference_deg) -
            calibration->phase_offset_deg);
    s_measurement.selected_bin =
        result->phase.selected_bin;
    s_measurement.phase_valid =
        ((result->phase.valid != 0U) &&
         (s_measurement.channel1_amplitude >=
          PHASE_APP_MIN_VALID_AMPLITUDE_COUNTS) &&
         (s_measurement.channel2_amplitude >=
          PHASE_APP_MIN_VALID_AMPLITUDE_COUNTS)) ? 1U : 0U;
    if (s_measurement.phase_valid != 0U)
    {
        s_measurement.gain_db =
            20.0f * log10f(
                s_measurement.channel2_amplitude /
                s_measurement.channel1_amplitude);
        if (s_mode == PHASE_APP_MODE_LEARNING)
        {
            learn_event = PhaseCalibration_AddSample(
                s_measurement.frequency_hz,
                s_adc_mode,
                s_measurement.phase_difference_deg,
                s_measurement.gain_db);
        }
        else
        {
            PhaseCalibration_Apply(
                s_measurement.frequency_hz,
                s_adc_mode,
                &s_measurement.phase_difference_deg,
                &s_measurement.gain_db);
        }
        s_measurement.status_code = 1U;
        (void)strcpy(s_measurement.status_text,
                     (s_mode == PHASE_APP_MODE_LEARNING) ?
                     "LEARNING" : "PHASE VALID");
        phase_app_menu_test_report_measurement();
    }
    else
    {
        s_measurement.gain_db = 0.0f;
        s_measurement.status_code = 0U;
        (void)strcpy(
            s_measurement.status_text, "ADC LEVEL LOW");
    }
    phase_app_set_state(PHASE_APP_STATE_ACQUIRE_PHASE);
    phase_app_publish_measurement(
        (learn_event != PHASE_CALIBRATION_LEARN_NONE) ?
        1U : 0U);
    if (submit_vofa_block != 0U)
    {
        PhaseVofa_SubmitBlock(
            s_channel1, s_channel2,
            PHASE_ADC_BLOCK_COUNT);
    }
    if (learn_event == PHASE_CALIBRATION_LEARN_NEXT_POINT)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M4 POINT DONE FREQ=%lu PHASE_ERR=%.3f GAIN_ERR=%.3f",
            (unsigned long)s_measurement.frequency_hz,
            (double)s_measurement.phase_difference_deg,
            (double)s_measurement.gain_db);
        (void)phase_app_learning_start_current_point();
    }
    else if (learn_event == PHASE_CALIBRATION_LEARN_COMPLETE)
    {
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M4 POINT DONE FREQ=%lu PHASE_ERR=%.3f GAIN_ERR=%.3f",
            (unsigned long)s_measurement.frequency_hz,
            (double)s_measurement.phase_difference_deg,
            (double)s_measurement.gain_db);
        phase_app_finish_learning();
    }
    if (s_mode == PHASE_APP_MODE_WIRING)
    {
        phase_app_wiring_check_sample();
    }
    return 1U;
}

void phase_app_set_measurement_publisher(
    PhaseAppMeasurementPublisher_t publisher)
{
    s_publish = publisher;
}

uint8_t phase_app_init(void)
{
    PhaseCaptureConfig_t capture_config;

    if (s_initialized != 0U)
    {
        return 1U;
    }

    (void)memset(&s_measurement, 0, sizeof(s_measurement));
    if (s_channel1 == NULL)
    {
        s_channel1 = (int16_t *)mymalloc(
            SRAMCCM,
            PHASE_ADC_BLOCK_COUNT * sizeof(s_channel1[0]));
    }
    if (s_channel2 == NULL)
    {
        s_channel2 = (int16_t *)mymalloc(
            SRAMCCM,
            PHASE_ADC_BLOCK_COUNT * sizeof(s_channel2[0]));
    }
    if ((s_channel1 == NULL) || (s_channel2 == NULL))
    {
        phase_app_publish_status(0U, "CCM BUFFER ERROR");
        return 0U;
    }
    (void)memset(
        s_channel1, 0,
        PHASE_ADC_BLOCK_COUNT * sizeof(s_channel1[0]));
    (void)memset(
        s_channel2, 0,
        PHASE_ADC_BLOCK_COUNT * sizeof(s_channel2[0]));
    s_capture_route = PHASE_CAPTURE_ROUTE_RAW;
    s_adc_mode = PHASE_ADC_MODE_DIRECT;
    s_mode = PHASE_APP_MODE_IDLE;
    s_state = PHASE_APP_STATE_IDLE;
    s_measurement.capture_cycles = PHASE_CAPTURE_EDGE_COUNT;
    s_measurement.divider = 1U;
    s_measurement.active_menu = (uint8_t)s_mode;
    (void)strcpy(s_measurement.status_text, "READY");

    phase_app_configure_pipeline();
    (void)PhaseCalibration_Init();
    phase_app_refresh_calibration_status();
    DDSDataInit();
    /*
     * The F1 demo firmware boots with a four-channel 1 MHz waveform and the
     * DDS controller may also survive an F4-only reset.  Force all channels
     * off with one coherent four-frame batch before any relay can connect a
     * measurement path.
     */
    if (AD9959_StopOutput() != HAL_OK)
    {
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "DDS INIT ERROR");
        return 0U;
    }
    s_last_dds_frequency_hz = 0U;
    s_last_dds_update_tick = xTaskGetTickCount();
    s_local_oscillator_enabled = 0U;
    PhaseCaptureConfig_Default(&capture_config);

    if ((AD8367_Init(
             PHASE_APP_AD8367_DEFAULT_GAIN_DB,
             PHASE_APP_AD8367_MODE) != HAL_OK) ||
        (PhaseRoute_Init() != HAL_OK) ||
        (PhaseAdc_Init() != HAL_OK) ||
        (PhaseCapture_Init(
            &g_phase_capture, &capture_config) != HAL_OK))
    {
        phase_app_set_state(PHASE_APP_STATE_ERROR);
        phase_app_publish_status(0U, "INIT ERROR");
        return 0U;
    }
    (void)DebugUart6_TextPrintfBlocking(
        "AD8367 INIT PASS PA4=%umV CODE=%u MODE=%s GAIN=%.1fdB",
        (unsigned)AD8367_GetControlVoltageMv(),
        (unsigned)AD8367_GetDacCode(),
        (AD8367_GetGainMode() ==
         AD8367_GAIN_INCREASES_WITH_VOLTAGE) ? "GAIN_UP" : "GAIN_DOWN",
        (double)AD8367_GetGainDb());

    s_phase_app_task = xTaskGetCurrentTaskHandle();
    PhaseAdc_SetReadyCallback(phase_app_adc_ready_from_isr);

    /*
     * Power-up uses SAFE_IDLE.  OUT3 stays disconnected until an ADC-backed
     * menu has measured the input frequency and selected a valid route.
     */
    PhaseRoute_Out3DisconnectAll();
    PhaseRoute_SetMainRelays(
        PHASE_CAPTURE_ROUTE_RAW, PHASE_ADC_MODE_DIRECT);
    vTaskDelay(pdMS_TO_TICKS(PHASE_APP_T_BREAK_MS));

    s_adc_running = 0U;
    s_capture_running = 0U;
    s_local_oscillator_enabled = 0U;
    s_no_signal_published = 0U;
    s_last_ui_publish_tick = 0U;
    s_last_frequency_tick = 0U;
    s_capture_watchdog_restart_attempted = 0U;
    s_initialized = 1U;
    phase_app_set_state(PHASE_APP_STATE_IDLE);
    phase_app_publish_status(1U, "READY");
    (void)DebugUart6_TextPrintfBlocking(
        "MENU_TEST READY M1=FREQUENCY M2=PHASE M3=ADC_FFT M4=LEARNING M5=WIRING");
    return 1U;
}

void phase_app_step(void)
{
    uint32_t period_ticks;
    uint32_t capture_cycles;
    uint8_t adc_processed;
    TickType_t now;

    if (s_initialized == 0U)
    {
        vTaskDelay(pdMS_TO_TICKS(10U));
        return;
    }

    now = xTaskGetTickCount();
    if ((s_capture_running != 0U) &&
        ((s_measurement.frequency_valid == 0U) ||
         (s_last_capture_process_tick == 0U) ||
         ((TickType_t)(now - s_last_capture_process_tick) >=
          pdMS_TO_TICKS(
              PHASE_APP_CAPTURE_PROCESS_PERIOD_MS))) &&
        (PhaseCapture_TakeMeasurement(
            &g_phase_capture, &period_ticks,
            &capture_cycles) != 0U))
    {
        s_last_capture_process_tick = now;
        phase_app_process_frequency(
            period_ticks, capture_cycles);
    }

    if ((s_mode == PHASE_APP_MODE_FREQUENCY) &&
        (s_m1_verify_pending != 0U) &&
        (s_capture_route == PHASE_CAPTURE_ROUTE_DIV32) &&
        ((TickType_t)(now - s_m1_verify_start_tick) >=
         pdMS_TO_TICKS(PHASE_APP_M1_VERIFY_TIMEOUT_MS)))
    {
        /*
         * RAW had edges but /32 did not.  This is the exact signature of the
         * observed no-input 5-6 MHz oscillation, so reject it before the
         * generic 2.5 s capture watchdog can restore and publish RAW again.
         */
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M1 VERIFY FAIL REASON=DIV32_NO_EDGE RAW=%luHz TIMEOUT_MS=%u",
            (unsigned long)s_m1_raw_candidate_hz,
            (unsigned)PHASE_APP_M1_VERIFY_TIMEOUT_MS);
        s_m1_verify_pending = 0U;
        s_m1_div32_verified = 0U;
        s_m1_verify_failed = 1U;
        s_m1_verify_failure_tick = now;
        s_capture_recovery_attempted = 1U;
        s_capture_watchdog_restart_attempted = 1U;
        (void)phase_app_apply_frequency_route(
            PHASE_CAPTURE_ROUTE_RAW);
        s_last_frequency_tick = now;
        phase_app_m1_publish_wait_signal();
        return;
    }

    if ((s_capture_running != 0U) &&
        (PhaseCapture_GetEdgeCount(&g_phase_capture) !=
         PHASE_CAPTURE_SLOW_EDGE_COUNT) &&
        ((TickType_t)(xTaskGetTickCount() -
                      s_last_frequency_tick) >=
         pdMS_TO_TICKS(PHASE_APP_SLOW_FALLBACK_MS)))
    {
        if (PhaseCapture_SetEdgeCount(
                &g_phase_capture,
                PHASE_CAPTURE_SLOW_EDGE_COUNT) == HAL_OK)
        {
            s_measurement.capture_cycles =
                PHASE_CAPTURE_SLOW_EDGE_COUNT;
            phase_app_submit_vofa_snapshot();
        }
    }

    if ((s_capture_running != 0U) &&
        (s_no_signal_published == 0U) &&
        ((TickType_t)(xTaskGetTickCount() -
                      s_last_frequency_tick) >=
         pdMS_TO_TICKS(PHASE_APP_NO_SIGNAL_MS)))
    {
        /*
         * A large downward frequency step can leave OUT1 on /32, making a
         * 10 Hz input only 0.3125 Hz at TIM2.  Recover to RAW once before
         * declaring a timeout so the low end is reachable without a reboot.
         */
        if ((s_capture_route == PHASE_CAPTURE_ROUTE_DIV32) &&
            (s_capture_recovery_attempted == 0U) &&
            ((s_mode == PHASE_APP_MODE_FREQUENCY) ||
             (s_mode == PHASE_APP_MODE_PHASE) ||
             (s_mode == PHASE_APP_MODE_FFT)))
        {
            s_capture_recovery_attempted = 1U;
            s_capture_watchdog_restart_attempted = 1U;
            (void)DebugUart6_TextPrintfBlocking(
                "MENU_TEST M%u CAPTURE_RECOVERY DIV32->RAW REASON=NO_EDGE",
                (unsigned)s_mode);
            if (s_mode == PHASE_APP_MODE_FREQUENCY)
            {
                (void)phase_app_apply_frequency_route(
                    PHASE_CAPTURE_ROUTE_RAW);
            }
            else
            {
                (void)phase_app_apply_route(
                    PHASE_CAPTURE_ROUTE_RAW, s_adc_mode);
            }
            return;
        }

        /*
         * A temporary DMA/edge-mode stall must not erase a valid reading.
         * Restart once and retain the last result.  If no new edge-derived
         * result arrives during the next full timeout window, the normal
         * no-signal branch below is allowed to publish 0 Hz.
         */
        if ((s_measurement.frequency_valid != 0U) &&
            (s_capture_watchdog_restart_attempted == 0U) &&
            ((s_mode == PHASE_APP_MODE_FREQUENCY) ||
             (s_mode == PHASE_APP_MODE_PHASE) ||
             (s_mode == PHASE_APP_MODE_FFT)))
        {
            s_capture_watchdog_restart_attempted = 1U;
            (void)DebugUart6_TextPrintfBlocking(
                "MENU_TEST M%u CAPTURE_WATCHDOG RESTART KEEP=%luHz ROUTE=%u EDGE=%lu DMA_ERR=%lu",
                (unsigned)s_mode,
                (unsigned long)s_measurement.frequency_hz,
                (unsigned)s_capture_route,
                (unsigned long)PhaseCapture_GetEdgeCount(
                    &g_phase_capture),
                (unsigned long)PhaseCapture_GetDmaErrorCount(
                    &g_phase_capture));
            phase_app_set_state(PHASE_APP_STATE_REMEASURE);
            if (phase_app_restart_capture_keep_result() != 0U)
            {
                s_measurement.status_code = 1U;
                (void)strcpy(
                    s_measurement.status_text, "CAPTURE RETRY");
                phase_app_publish_measurement(1U);
            }
            return;
        }

        if ((s_mode == PHASE_APP_MODE_FREQUENCY) ||
            (s_mode == PHASE_APP_MODE_PHASE) ||
            (s_mode == PHASE_APP_MODE_FFT))
        {
            (void)DebugUart6_TextPrintfBlocking(
                "MENU_TEST M%u CAPTURE_TIMEOUT CLEAR LAST=%luHz ROUTE=%u EDGE=%lu DMA_ERR=%lu PB10=%u",
                (unsigned)s_mode,
                (unsigned long)s_measurement.frequency_hz,
                (unsigned)s_capture_route,
                (unsigned long)PhaseCapture_GetEdgeCount(
                    &g_phase_capture),
                (unsigned long)PhaseCapture_GetDmaErrorCount(
                    &g_phase_capture),
                (unsigned)HAL_GPIO_ReadPin(
                    PHASE_CAPTURE_GPIO_PORT,
                    PHASE_CAPTURE_GPIO_PIN));
        }
        (void)phase_app_set_adc_running(0U, 0U);
        (void)phase_app_disable_local_oscillator();
        s_measurement.frequency_hz = 0U;
        s_measurement.divided_frequency_hz = 0U;
        s_measurement.frequency_hz_precise = 0.0f;
        s_measurement.divided_frequency_hz_precise = 0.0f;
        s_measurement.period_ticks = 0U;
        s_measurement.frequency_valid = 0U;
        s_measurement.status_code = 0U;
        phase_app_invalidate_phase();
        (void)strcpy(
            s_measurement.status_text, "WAIT SIGNAL");
        s_no_signal_published = 1U;
        phase_app_set_state(
            PHASE_APP_STATE_MEASURE_FREQUENCY);
        phase_app_publish_measurement(1U);
        if ((s_menu_test_result_reported == 0U) &&
            ((s_mode == PHASE_APP_MODE_FREQUENCY) ||
             (s_mode == PHASE_APP_MODE_PHASE) ||
             (s_mode == PHASE_APP_MODE_FFT)))
        {
            s_menu_test_result_reported = 1U;
            (void)DebugUart6_TextPrintfBlocking(
                "MENU_TEST M%u RESULT FAIL REASON=NO_FREQUENCY TIMEOUT_MS=%u",
                (unsigned)s_mode,
                (unsigned)PHASE_APP_NO_SIGNAL_MS);
        }
    }

    if ((s_menu_test_result_reported == 0U) &&
        ((s_mode == PHASE_APP_MODE_PHASE) ||
         (s_mode == PHASE_APP_MODE_FFT)) &&
        (s_measurement.frequency_valid != 0U) &&
        ((TickType_t)(now - s_menu_test_start_tick) >=
         pdMS_TO_TICKS(PHASE_APP_NO_SIGNAL_MS)))
    {
        s_menu_test_result_reported = 1U;
        (void)DebugUart6_TextPrintfBlocking(
            "MENU_TEST M%u RESULT FAIL REASON=ADC_TIMEOUT FREQ=%lu ADC_MODE=%u RATE=%lu CH1=%.2f CH2=%.2f OVR=%lu",
            (unsigned)s_mode,
            (unsigned long)s_measurement.frequency_hz,
            (unsigned)s_adc_mode,
            (unsigned long)s_measurement.adc_sample_rate_hz,
            (double)s_measurement.channel1_amplitude,
            (double)s_measurement.channel2_amplitude,
            (unsigned long)PhaseAdc_GetOverrunCount());
        phase_app_print_relay_state("MENU_TEST");
    }

    adc_processed = phase_app_process_adc();
    phase_app_wiring_check_timeout();
    if ((s_adc_running != 0U) && (adc_processed == 0U))
    {
        (void)ulTaskNotifyTake(
            pdTRUE, pdMS_TO_TICKS(1U));
    }
    else if (s_adc_running == 0U)
    {
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

void phase_app_handle_command(const UI_Command_t *command)
{
    HAL_StatusTypeDef dds_status;

    if (command == NULL)
    {
        return;
    }

    if (command->type == UI_COMMAND_STOP_OUTPUT)
    {
        if (s_wiring_active != 0U)
        {
            phase_app_wiring_finish(1U);
            PhaseVofa_SetOutputEnabled(0U);
            return;
        }
        if (PhaseCalibration_IsLearning() != 0U)
        {
            PhaseCalibration_AbortLearning();
        }
        (void)phase_app_set_adc_running(0U, 0U);
        if (s_capture_running != 0U)
        {
            (void)PhaseCapture_Stop(&g_phase_capture);
            s_capture_running = 0U;
        }
        PhaseRoute_Out3DisconnectAll();
        PhaseRoute_SetMainRelays(
            PHASE_CAPTURE_ROUTE_RAW, PHASE_ADC_MODE_DIRECT);
        s_capture_route = PHASE_CAPTURE_ROUTE_RAW;
        s_adc_mode = PHASE_ADC_MODE_DIRECT;
        phase_app_invalidate_phase();
        s_mode = PHASE_APP_MODE_IDLE;
        phase_app_set_state(PHASE_APP_STATE_IDLE);

        phase_app_wait_dds_gap();
        dds_status = AD9959_StopOutput();
        s_last_dds_update_tick = xTaskGetTickCount();
        s_last_dds_frequency_hz = 0U;
        s_local_oscillator_enabled = 0U;
        s_measurement.local_oscillator_hz = 0U;
        PhaseVofa_SetOutputEnabled(0U);
        if (dds_status == HAL_OK)
        {
            (void)DebugUart6_TextPrintfBlocking(
                "MENU_TEST STOP PASS SAFE_IDLE=1 ADC=OFF DDS=OFF");
            phase_app_print_relay_state("MENU_TEST");
            phase_app_publish_status(1U, "STOPPED");
        }
        else
        {
            phase_app_publish_status(0U, "DDS STOP ERROR");
        }
    }
    else if (command->type ==
             UI_COMMAND_START_FREQUENCY_MEASUREMENT)
    {
        phase_app_start_measurement_mode(
            PHASE_APP_MODE_FREQUENCY, "WAIT SIGNAL");
    }
    else if (command->type == UI_COMMAND_SET_SINE)
    {
        (void)AD9959_SetOutput(
            command->frequency_hz,
            command->amplitude_mv,
            command->phase_deg, 1U);
    }
    else if (command->type == UI_COMMAND_RUN_KNOWN_MODEL)
    {
        phase_app_start_measurement_mode(
            PHASE_APP_MODE_PHASE, "PHASE MEASURING");
    }
    else if (command->type ==
             UI_COMMAND_START_UNKNOWN_MODEL)
    {
        phase_app_start_measurement_mode(
            PHASE_APP_MODE_FFT, "DFT MEASURING");
    }
    else if (command->type == UI_COMMAND_START_LEARNING)
    {
        phase_app_start_learning_mode();
    }
    else if (command->type == UI_COMMAND_START_WIRING_TEST)
    {
        phase_app_start_wiring_test();
    }
    else if (command->type ==
             UI_COMMAND_START_INFERENCE)
    {
        DebugUart6_SetOutputEnabled(1U);
        phase_app_start_measurement_mode(
            PHASE_APP_MODE_VOFA, "VOFA OUTPUT");
    }
    else if (command->type == UI_COMMAND_RUN_CALIBRATION)
    {
        s_mode = PHASE_APP_MODE_SYSTEM;
        phase_app_set_state(
            PHASE_APP_STATE_MEASURE_FREQUENCY);
        phase_app_publish_status(1U, "SYSTEM READY");
    }
}
