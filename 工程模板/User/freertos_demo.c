#include "freertos_demo.h"

#include "./BSP/KEY/key.h"
#include "./BSP/PS2/ps2.h"
#include "./BSP/PHASE/phase_vofa.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "phase_app.h"
#include "user.h"

#define START_TASK_PRIO             1U
#define START_TASK_STACK_SIZE       128U
#define KEY_TASK_PRIO               4U
#define KEY_TASK_STACK_SIZE         128U
#define UI_TASK_PRIO                3U
#define UI_TASK_STACK_SIZE          512U
#define INSTRUMENT_TASK_PRIO        5U
#define INSTRUMENT_TASK_STACK_SIZE  512U
#define UI_REFRESH_PERIOD_MS         5U
/* Phase ADC produces results much faster than the LCD can render them.
 * Keep only the newest snapshot so a high-priority producer cannot starve
 * the UI task before it services the key queue. */
#define MEASUREMENT_QUEUE_LENGTH    1U

static QueueHandle_t s_key_queue;
static QueueHandle_t s_command_queue;
static QueueHandle_t s_measurement_queue;
static QueueHandle_t s_ads_progress_queue;
static TaskHandle_t s_instrument_task_handle;
static volatile uint8_t s_stop_requested;

static void StartTask(void *argument);
static void KeyTask(void *argument);
static void UiTask(void *argument);
static void InstrumentTask(void *argument);
static void Instrument_PublishAdsProgress(
    const UI_Measurement_t *measurement);

static PS2_KeyValue_t KeyTask_MapBoardKey(uint8_t board_key)
{
    switch (board_key)
    {
        case KEY1_PRES: return PS2_KEY_VALUE_1;
        case KEY2_PRES: return PS2_KEY_VALUE_2;
        case KEY3_PRES: return PS2_KEY_VALUE_3;
        case WKUP_PRES: return PS2_KEY_VALUE_ENTER;
        default:        return PS2_KEY_VALUE_NULL;
    }
}

void Instrument_RequestStop(void)
{
    s_stop_requested = 1U;
    if (s_instrument_task_handle != NULL)
    {
        (void)xTaskNotifyGive(s_instrument_task_handle);
    }
}

static void Instrument_ClearStopRequest(void)
{
    s_stop_requested = 0U;
}

static void Instrument_Publish(const UI_Measurement_t *measurement)
{
    if ((measurement == NULL) || (s_measurement_queue == NULL))
    {
        return;
    }
    if ((measurement->event == UI_EVENT_SWEEP_PROGRESS) ||
        (measurement->event == UI_EVENT_ADS_TEST))
    {
        Instrument_PublishAdsProgress(measurement);
        return;
    }
    (void)xQueueOverwrite(s_measurement_queue, measurement);
}

static void Instrument_PublishAdsProgress(
    const UI_Measurement_t *measurement)
{
    if ((measurement != NULL) && (s_ads_progress_queue != NULL))
    {
        (void)xQueueOverwrite(s_ads_progress_queue, measurement);
    }
}

void freertos_demo(void)
{
    s_key_queue = xQueueCreate(8U, sizeof(PS2_KeyValue_t));
    s_command_queue = xQueueCreate(4U, sizeof(UI_Command_t));
    s_measurement_queue = xQueueCreate(MEASUREMENT_QUEUE_LENGTH,
                                       sizeof(UI_Measurement_t));
    s_ads_progress_queue = xQueueCreate(1U, sizeof(UI_Measurement_t));
    if ((s_key_queue == NULL) || (s_command_queue == NULL) ||
        (s_measurement_queue == NULL) || (s_ads_progress_queue == NULL))
    {
        return;
    }

    (void)xTaskCreate(StartTask, "start", START_TASK_STACK_SIZE,
                      NULL, START_TASK_PRIO, NULL);
    vTaskStartScheduler();
    for (;;)
    {
    }
}

static void StartTask(void *argument)
{
    (void)argument;
    taskENTER_CRITICAL();
    (void)xTaskCreate(KeyTask, "key", KEY_TASK_STACK_SIZE,
                      NULL, KEY_TASK_PRIO, NULL);
    (void)xTaskCreate(UiTask, "ui", UI_TASK_STACK_SIZE,
                      NULL, UI_TASK_PRIO, NULL);
    (void)xTaskCreate(InstrumentTask, "instrument",
                      INSTRUMENT_TASK_STACK_SIZE, NULL,
                      INSTRUMENT_TASK_PRIO, &s_instrument_task_handle);
    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}

static void KeyTask(void *argument)
{
    PS2_KeyValue_t key;
    PS2_KeyValue_t board_key_value;
    uint8_t board_key;

    (void)argument;
    for (;;)
    {
        PS2_KeyScan();
        key = PS2_GetCurrentKeyValue();
        if (key != PS2_KEY_VALUE_NULL)
        {
            (void)xQueueSend(s_key_queue, &key, 0U);
        }

        board_key = key_scan(0U);
        board_key_value = KeyTask_MapBoardKey(board_key);
        if (board_key_value != PS2_KEY_VALUE_NULL)
        {
            (void)xQueueSend(s_key_queue, &board_key_value, 0U);
        }
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

static void UiTask(void *argument)
{
    PS2_KeyValue_t key;
    UI_Command_t command;
    UI_Measurement_t measurement;

    (void)argument;
    UI_Init();
    for (;;)
    {
        if (xQueueReceive(s_ads_progress_queue, &measurement, 0U) == pdPASS)
        {
            UI_UpdateMeasurement(&measurement);
        }
        if (xQueueReceive(s_measurement_queue, &measurement, 0U) == pdPASS)
        {
            UI_UpdateMeasurement(&measurement);
        }

        if (xQueueReceive(s_key_queue, &key,
                          pdMS_TO_TICKS(UI_REFRESH_PERIOD_MS)) == pdPASS)
        {
            if (key == PS2_KEY_VALUE_BACKSPACE)
            {
                Instrument_RequestStop();
            }
            if (UI_HandleKey(key, &command) &&
                (command.type != UI_COMMAND_NONE))
            {
                if (command.type == UI_COMMAND_STOP_OUTPUT)
                {
                    Instrument_RequestStop();
                }
                (void)xQueueSend(s_command_queue, &command, 0U);
            }
        }
    }
}

static void InstrumentTask(void *argument)
{
    UI_Command_t command;
    uint8_t initialized;

    (void)argument;
    (void)PhaseVofa_CreateTask();
    Instrument_ClearStopRequest();
    phase_app_set_measurement_publisher(Instrument_Publish);
    initialized = phase_app_init();
    if (initialized == 0U)
    {
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    for (;;)
    {
        if (xQueueReceive(s_command_queue, &command,
                          pdMS_TO_TICKS(1U)) == pdPASS)
        {
            phase_app_handle_command(&command);
        }
        phase_app_step();
    }
}
