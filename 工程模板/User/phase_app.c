#include "phase_app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "./BSP/AD9959/Drive_Communication.h"
#include "./BSP/DEBUG/debug_uart6.h"
#include "./BSP/PHASE/phase_adc.h"
#include "./BSP/PHASE/phase_capture.h"
#include "./BSP/PHASE/phase_measure.h"
#include "./BSP/PHASE/phase_pipeline.h"
#include "./BSP/PHASE/phase_vofa.h"
#include "./BSP/PGA4311/pga4311.h"

#include <stdlib.h>
#include <string.h>

#define PHASE_APP_IF_FREQUENCY_HZ     10000U
#define PHASE_APP_ADC_SAMPLE_RATE_HZ  256000U
#define PHASE_APP_INPUT_DIVIDER       1U
#define PHASE_APP_DDS_AMPLITUDE_MV    250.0f
#define PHASE_APP_UI_PUBLISH_MS       50U
#define PHASE_APP_NO_SIGNAL_MS        2500U
#define PHASE_APP_SLOW_FALLBACK_MS    300U

typedef enum
{
    PHASE_APP_MODE_IDLE = 0U,
    PHASE_APP_MODE_FREQUENCY,
    PHASE_APP_MODE_PHASE,
    PHASE_APP_MODE_FFT,
    PHASE_APP_MODE_VOFA,
    PHASE_APP_MODE_SYSTEM
} PhaseAppMode_t;

static int16_t s_channel1[PHASE_ADC_BLOCK_COUNT];
static int16_t s_channel2[PHASE_ADC_BLOCK_COUNT];
static uint32_t s_last_dds_frequency_hz;
static uint8_t s_initialized;
static uint8_t s_adc_running;
static uint8_t s_capture_running;
static uint8_t s_no_signal_published;
static TickType_t s_last_ui_publish_tick;
static TickType_t s_last_frequency_tick;
static PhaseAppMode_t s_mode;
static PhaseAppMeasurementPublisher_t s_publish;
static UI_Measurement_t s_measurement;

static void phase_app_publish_status(uint8_t status_code, const char *text)
{
    s_measurement.event = UI_EVENT_STATUS;
    s_measurement.status_code = status_code;
    (void)strncpy(s_measurement.status_text, text,
                  sizeof(s_measurement.status_text) - 1U);
    s_measurement.status_text[sizeof(s_measurement.status_text) - 1U] = '\0';
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
        return;
    }
    s_last_ui_publish_tick = now;
    s_measurement.event = UI_EVENT_MEASUREMENT;
    s_measurement.adc_overrun_count = PhaseAdc_GetOverrunCount();
    if (s_publish != NULL)
    {
        s_publish(&s_measurement);
    }
}

static void phase_app_configure_pipeline(void)
{
    PhasePipelineConfig_t config;

    PhasePipelineConfig_Default(&config);
    config.frequency.timer_tick_hz = PHASE_CAPTURE_TIMER_CLOCK_HZ;
    config.frequency.capture_cycles = PHASE_CAPTURE_EDGE_COUNT;
    config.fft.sample_rate_hz = PHASE_APP_ADC_SAMPLE_RATE_HZ;
    config.fft.signal_frequency_hz = PHASE_APP_IF_FREQUENCY_HZ;
    (void)PhasePipeline_Init(&g_phase_pipeline, &config);
}

static uint8_t phase_app_set_adc_running(uint8_t enabled)
{
    if (enabled != 0U)
    {
        if (s_adc_running != 0U)
        {
            return 1U;
        }
        if (PhaseAdc_Start(PHASE_APP_ADC_SAMPLE_RATE_HZ) != HAL_OK)
        {
            phase_app_publish_status(0U, "ADC START ERROR");
            return 0U;
        }
        s_adc_running = 1U;
        return 1U;
    }

    if (s_adc_running != 0U)
    {
        (void)PhaseAdc_Stop();
        s_adc_running = 0U;
    }
    return 1U;
}

static uint8_t phase_app_restart_capture(void)
{
    if (s_capture_running != 0U)
    {
        (void)PhaseCapture_Stop(&g_phase_capture);
        s_capture_running = 0U;
    }
    if (PhaseCapture_Start(&g_phase_capture) != HAL_OK)
    {
        phase_app_publish_status(0U, "CAPTURE ERROR");
        return 0U;
    }

    s_capture_running = 1U;
    s_no_signal_published = 0U;
    s_last_frequency_tick = xTaskGetTickCount();
    s_measurement.frequency_hz = 0U;
    s_measurement.divided_frequency_hz = 0U;
    s_measurement.frequency_hz_precise = 0.0f;
    s_measurement.divided_frequency_hz_precise = 0.0f;
    s_measurement.period_ticks = 0U;
    s_measurement.capture_cycles = PHASE_CAPTURE_EDGE_COUNT;
    s_measurement.divider = PHASE_APP_INPUT_DIVIDER;
    s_measurement.frequency_valid = 0U;
    PhaseVofa_SubmitTimerCapture(0.0f, 0U, 0U);
    return 1U;
}

static void phase_app_start_measurement_mode(PhaseAppMode_t mode,
                                             uint8_t adc_required,
                                             const char *status)
{
    if (phase_app_set_adc_running(adc_required) == 0U)
    {
        return;
    }
    if (phase_app_restart_capture() == 0U)
    {
        return;
    }
    s_mode = mode;
    phase_app_publish_status(1U, status);
}

static void phase_app_update_local_oscillator(uint32_t input_frequency_hz)
{
    uint32_t local_frequency_hz;

    if (input_frequency_hz > (PHASE_MEASURE_MAX_HZ -
                              PHASE_APP_IF_FREQUENCY_HZ))
    {
        return;
    }
    local_frequency_hz = input_frequency_hz + PHASE_APP_IF_FREQUENCY_HZ;
    if ((s_last_dds_frequency_hz == 0U) ||
        (abs((int32_t)local_frequency_hz -
             (int32_t)s_last_dds_frequency_hz) > 1))
    {
        if (AD9959_SetOutput(local_frequency_hz,
                             PHASE_APP_DDS_AMPLITUDE_MV,
                             0U, 1U) == HAL_OK)
        {
            s_last_dds_frequency_hz = local_frequency_hz;
        }
    }
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
    s_measurement.capture_cycles = PHASE_CAPTURE_EDGE_COUNT;
    s_measurement.divider = PHASE_APP_INPUT_DIVIDER;
    phase_app_configure_pipeline();
    DDSDataInit();
    PhaseCaptureConfig_Default(&capture_config);
    if ((PGA4311_InitDevice() != HAL_OK) ||
        (PhaseAdc_Init() != HAL_OK) ||
        (PhaseCapture_Init(&g_phase_capture, &capture_config) != HAL_OK))
    {
        (void)DebugUart6_Printf("PHASE_APP_ERROR,reason=INIT\r\n");
        phase_app_publish_status(0U, "INIT ERROR");
        return 0U;
    }
    if (PhaseCapture_Start(&g_phase_capture) != HAL_OK)
    {
        (void)DebugUart6_Printf("PHASE_APP_ERROR,reason=START\r\n");
        phase_app_publish_status(0U, "START ERROR");
        return 0U;
    }

    s_adc_running = 0U;
    s_capture_running = 1U;
    s_no_signal_published = 0U;
    s_last_ui_publish_tick = 0U;
    s_last_frequency_tick = xTaskGetTickCount();
    s_mode = PHASE_APP_MODE_FREQUENCY;
    s_initialized = 1U;
    phase_app_publish_status(1U, "WAIT SIGNAL");
    (void)DebugUart6_Printf(
        "PHASE_APP_READY,range=%u-%lu,capture_edges=%u,adc_fs=%lu,divider=%u,pga_db=0.0\r\n",
        PHASE_MEASURE_MIN_HZ,
        (unsigned long)PHASE_MEASURE_MAX_HZ,
        PHASE_CAPTURE_EDGE_COUNT,
        (unsigned long)PHASE_APP_ADC_SAMPLE_RATE_HZ,
        PHASE_APP_INPUT_DIVIDER);
    return 1U;
}

void phase_app_step(void)
{
    uint32_t period_ticks;
    uint32_t capture_cycles;
    const PhasePipelineResult_t *result;

    if (s_initialized == 0U)
    {
        vTaskDelay(pdMS_TO_TICKS(10U));
        return;
    }

    if ((s_capture_running != 0U) &&
        (PhaseCapture_TakeMeasurement(&g_phase_capture, &period_ticks,
                                      &capture_cycles) != 0U))
    {
        (void)PhasePipeline_ProcessCapturedPeriod(
            &g_phase_pipeline, period_ticks, capture_cycles,
            PHASE_APP_INPUT_DIVIDER);
        result = PhasePipeline_GetResult(&g_phase_pipeline);
        if ((result != NULL) && (result->frequency_valid != 0U))
        {
            if (s_mode == PHASE_APP_MODE_PHASE)
            {
                phase_app_update_local_oscillator(
                    result->frequency.input_frequency_hz);
            }
            s_measurement.frequency_hz =
                result->frequency.input_frequency_hz;
            s_measurement.divided_frequency_hz =
                result->frequency.divided_frequency_hz;
            s_measurement.frequency_hz_precise =
                result->frequency.input_frequency_hz_f;
            s_measurement.divided_frequency_hz_precise =
                result->frequency.divided_frequency_hz_f;
            s_measurement.divider = result->frequency.divider;
            s_measurement.period_ticks = result->frequency.period_ticks;
            s_measurement.capture_cycles = capture_cycles;
            s_measurement.frequency_valid = 1U;
            s_measurement.status_code = 1U;
            (void)strcpy(s_measurement.status_text, "SIGNAL VALID");
            s_no_signal_published = 0U;
            s_last_frequency_tick = xTaskGetTickCount();
            PhaseVofa_SubmitTimerCapture(
                result->frequency.input_frequency_hz_f,
                result->frequency.period_ticks,
                result->frequency.valid);
            (void)DebugUart6_Printf(
                "PHASE_FREQ,input=%lu,divided=%lu,divider=%lu,period_ticks=%lu,capture_cycles=%lu,dma_errors=%lu\r\n",
                (unsigned long)result->frequency.input_frequency_hz,
                (unsigned long)result->frequency.divided_frequency_hz,
                (unsigned long)result->frequency.divider,
                (unsigned long)result->frequency.period_ticks,
                (unsigned long)capture_cycles,
                (unsigned long)PhaseCapture_GetDmaErrorCount(
                    &g_phase_capture));
            phase_app_publish_measurement(0U);
        }
    }

    /*
     * With no recent result, DIV8 makes a 10 Hz source unnecessarily slow to
     * acquire. Probe every edge after 300 ms. The ISR promotes itself back to
     * DIV8 immediately if this is actually a fast source.
     */
    if ((s_capture_running != 0U) &&
        (PhaseCapture_GetEdgeCount(&g_phase_capture) !=
         PHASE_CAPTURE_SLOW_EDGE_COUNT) &&
        ((TickType_t)(xTaskGetTickCount() - s_last_frequency_tick) >=
         pdMS_TO_TICKS(PHASE_APP_SLOW_FALLBACK_MS)))
    {
        if (PhaseCapture_SetEdgeCount(
                &g_phase_capture,
                PHASE_CAPTURE_SLOW_EDGE_COUNT) == HAL_OK)
        {
            s_measurement.capture_cycles =
                PHASE_CAPTURE_SLOW_EDGE_COUNT;
            (void)DebugUart6_Printf(
                "PHASE_CAPTURE_MODE,capture_cycles=%u,reason=SLOW_PROBE\r\n",
                PHASE_CAPTURE_SLOW_EDGE_COUNT);
        }
    }

    if ((s_capture_running != 0U) &&
        (s_no_signal_published == 0U) &&
        ((TickType_t)(xTaskGetTickCount() - s_last_frequency_tick) >=
         pdMS_TO_TICKS(PHASE_APP_NO_SIGNAL_MS)))
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
        PhaseVofa_SubmitTimerCapture(0.0f, 0U, 0U);
        phase_app_publish_measurement(1U);
    }

    if ((s_adc_running != 0U) &&
        (PhaseAdc_TakeBlock(s_channel1, s_channel2,
                            PHASE_ADC_BLOCK_COUNT) == HAL_OK))
    {
        PhaseVofa_SubmitBlock(s_channel1, s_channel2,
                              PHASE_ADC_BLOCK_COUNT);
        if (PhasePipeline_ProcessAdcBlock(&g_phase_pipeline,
                                           s_channel1, s_channel2) != 0U)
        {
            result = PhasePipeline_GetResult(&g_phase_pipeline);
            if (result != NULL)
            {
                s_measurement.channel1_amplitude =
                    result->phase.channel1_amplitude;
                s_measurement.channel2_amplitude =
                    result->phase.channel2_amplitude;
                s_measurement.phase_difference_deg =
                    result->phase.phase_difference_deg;
                s_measurement.selected_bin = result->phase.selected_bin;
                s_measurement.phase_valid = result->phase.valid;
                (void)DebugUart6_Printf(
                    "PHASE_RESULT,bin=%u,ch1_amp=%.4f,ch2_amp=%.4f,phase_deg=%.3f,overrun=%lu\r\n",
                    (unsigned int)result->phase.selected_bin,
                    (double)result->phase.channel1_amplitude,
                    (double)result->phase.channel2_amplitude,
                    (double)result->phase.phase_difference_deg,
                    (unsigned long)PhaseAdc_GetOverrunCount());
                phase_app_publish_measurement(0U);
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(1U));
}

void phase_app_handle_command(const UI_Command_t *command)
{
    if (command == NULL)
    {
        return;
    }
    if (command->type == UI_COMMAND_STOP_OUTPUT)
    {
        (void)AD9959_StopOutput();
        s_last_dds_frequency_hz = 0U;
        (void)phase_app_set_adc_running(0U);
        if (s_capture_running != 0U)
        {
            (void)PhaseCapture_Stop(&g_phase_capture);
            s_capture_running = 0U;
        }
        s_mode = PHASE_APP_MODE_IDLE;
        phase_app_publish_status(1U, "STOPPED");
    }
    else if (command->type == UI_COMMAND_START_FREQUENCY_MEASUREMENT)
    {
        (void)AD9959_StopOutput();
        s_last_dds_frequency_hz = 0U;
        phase_app_start_measurement_mode(PHASE_APP_MODE_FREQUENCY, 0U,
                                         "WAIT SIGNAL");
    }
    else if (command->type == UI_COMMAND_SET_SINE)
    {
        (void)AD9959_SetOutput(command->frequency_hz,
                                command->amplitude_mv,
                                command->phase_deg, 1U);
    }
    else if (command->type == UI_COMMAND_RUN_KNOWN_MODEL)
    {
        phase_app_start_measurement_mode(PHASE_APP_MODE_PHASE, 1U,
                                         "PHASE MEASURING");
    }
    else if (command->type == UI_COMMAND_START_UNKNOWN_MODEL)
    {
        phase_app_start_measurement_mode(PHASE_APP_MODE_FFT, 1U,
                                         "FFT MEASURING");
    }
    else if (command->type == UI_COMMAND_START_INFERENCE)
    {
        DebugUart6_SetOutputEnabled(1U);
        phase_app_start_measurement_mode(PHASE_APP_MODE_VOFA, 1U,
                                         "VOFA OUTPUT");
    }
    else if (command->type == UI_COMMAND_SET_PGA_GAIN)
    {
        if (PGA4311_SetChannelGainDb(command->pga_channel,
                                     command->pga_gain_db) == HAL_OK)
        {
            phase_app_publish_status(1U, "PGA GAIN UPDATED");
        }
        else
        {
            phase_app_publish_status(0U, "PGA GAIN ERROR");
        }
    }
    else if (command->type == UI_COMMAND_RESET_PGA_GAIN)
    {
        PGA4311_SetAV(0.0f, 0.0f, 0.0f, 0.0f);
        phase_app_publish_status(1U, "PGA ALL 0dB");
    }
    else if (command->type == UI_COMMAND_RUN_CALIBRATION)
    {
        s_mode = PHASE_APP_MODE_SYSTEM;
        phase_app_publish_status(1U, "SYSTEM READY");
    }
}
