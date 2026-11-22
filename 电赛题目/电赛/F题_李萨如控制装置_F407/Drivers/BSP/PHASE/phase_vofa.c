#include "phase_vofa.h"

#include "./BSP/DEBUG/debug_uart6.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include <string.h>

#define PHASE_VOFA_TASK_PRIORITY     2U
#define PHASE_VOFA_TASK_STACK_WORDS  512U
#define PHASE_VOFA_QUEUE_LENGTH      1U
#define PHASE_VOFA_STATUS_PERIOD_MS  50U

typedef struct
{
    int16_t channel1[PHASE_VOFA_POINT_COUNT];
    int16_t channel2[PHASE_VOFA_POINT_COUNT];
} PhaseVofaFrame_t;

static QueueHandle_t s_phase_vofa_queue;
static volatile uint32_t s_phase_vofa_frame_count;
static volatile uint32_t s_phase_vofa_overwrite_count;
/* VOFA is opt-in: ordinary startup and non-ADC menus stay silent. */
static volatile uint8_t s_phase_vofa_output_enabled = 0U;
static PhaseVofaSnapshot_t s_phase_vofa_snapshot;

static void PhaseVofa_SendFireWater(int16_t channel1, int16_t channel2)
{
    if (s_phase_vofa_output_enabled == 0U)
    {
        return;
    }

    (void)DebugUart6_VofaPrintfBlocking(
        "ADC",
        "%d,%d",
        (int)channel1,
        (int)channel2);
}

static void PhaseVofa_Task(void *argument)
{
    PhaseVofaFrame_t frame;
    uint16_t index;
    BaseType_t has_frame;

    (void)argument;
    for (;;)
    {
        has_frame = xQueueReceive(
            s_phase_vofa_queue, &frame,
            pdMS_TO_TICKS(PHASE_VOFA_STATUS_PERIOD_MS));

        if (has_frame == pdPASS)
        {
            for (index = 0U; index < PHASE_VOFA_POINT_COUNT; ++index)
            {
                PhaseVofa_SendFireWater(
                    frame.channel1[index],
                    frame.channel2[index]);
            }
            ++s_phase_vofa_frame_count;
        }
    }
}

uint8_t PhaseVofa_CreateTask(void)
{
    if (s_phase_vofa_queue != NULL)
    {
        return 1U;
    }

    s_phase_vofa_queue = xQueueCreate(PHASE_VOFA_QUEUE_LENGTH,
                                      sizeof(PhaseVofaFrame_t));
    if (s_phase_vofa_queue == NULL)
    {
        return 0U;
    }

    s_phase_vofa_frame_count = 0U;
    s_phase_vofa_overwrite_count = 0U;
    s_phase_vofa_output_enabled = 0U;
    (void)memset(&s_phase_vofa_snapshot, 0,
                 sizeof(s_phase_vofa_snapshot));
    return (xTaskCreate(PhaseVofa_Task, "vofa",
                        PHASE_VOFA_TASK_STACK_WORDS, NULL,
                        PHASE_VOFA_TASK_PRIORITY, NULL) == pdPASS) ?
           1U : 0U;
}

void PhaseVofa_SetOutputEnabled(uint8_t enabled)
{
    s_phase_vofa_output_enabled = (enabled != 0U) ? 1U : 0U;
    if ((s_phase_vofa_output_enabled == 0U) &&
        (s_phase_vofa_queue != NULL))
    {
        /* Do not leak a block captured by the previous menu. */
        (void)xQueueReset(s_phase_vofa_queue);
    }
}

void PhaseVofa_SubmitBlock(const int16_t *channel1,
                           const int16_t *channel2,
                           uint16_t length)
{
    PhaseVofaFrame_t frame;
    int32_t channel1_sum = 0;
    int32_t channel2_sum = 0;
    int32_t channel1_mean;
    int32_t channel2_mean;
    int32_t centered1;
    int32_t centered2;
    int32_t magnitude;
    int32_t shared_peak = 0;
    uint16_t index;

    if ((s_phase_vofa_queue == NULL) || (channel1 == NULL) ||
        (channel2 == NULL) || (length == 0U))
    {
        return;
    }

    for (index = 0U; index < length; ++index)
    {
        channel1_sum += channel1[index];
        channel2_sum += channel2[index];
    }
    channel1_mean = channel1_sum / (int32_t)length;
    channel2_mean = channel2_sum / (int32_t)length;

    /* One shared scale keeps the real PA1/PA2 amplitude ratio visible. */
    for (index = 0U; index < length; ++index)
    {
        centered1 = (int32_t)channel1[index] - channel1_mean;
        centered2 = (int32_t)channel2[index] - channel2_mean;
        magnitude = (centered1 < 0) ? -centered1 : centered1;
        if (magnitude > shared_peak)
        {
            shared_peak = magnitude;
        }
        magnitude = (centered2 < 0) ? -centered2 : centered2;
        if (magnitude > shared_peak)
        {
            shared_peak = magnitude;
        }
    }

    for (index = 0U; index < PHASE_VOFA_POINT_COUNT; ++index)
    {
        uint16_t source_index = (index < length) ? index :
                                (uint16_t)(length - 1U);

        centered1 = (int32_t)channel1[source_index] - channel1_mean;
        centered2 = (int32_t)channel2[source_index] - channel2_mean;
        if (shared_peak > 0)
        {
            frame.channel1[index] = (int16_t)(
                centered1 * PHASE_VOFA_NORMALIZED_PEAK / shared_peak);
            frame.channel2[index] = (int16_t)(
                centered2 * PHASE_VOFA_NORMALIZED_PEAK / shared_peak);
        }
        else
        {
            frame.channel1[index] = 0;
            frame.channel2[index] = 0;
        }
    }

    if (uxQueueMessagesWaiting(s_phase_vofa_queue) != 0U)
    {
        ++s_phase_vofa_overwrite_count;
    }
    (void)xQueueOverwrite(s_phase_vofa_queue, &frame);
}

void PhaseVofa_SubmitTimerCapture(float frequency_hz,
                                  uint32_t period_ticks,
                                  uint8_t valid)
{
    (void)period_ticks;
    taskENTER_CRITICAL();
    s_phase_vofa_snapshot.capture_frequency_hz =
        (valid != 0U) ? frequency_hz : 0.0f;
    s_phase_vofa_snapshot.frequency_valid =
        (valid != 0U) ? 1U : 0U;
    taskEXIT_CRITICAL();
}

void PhaseVofa_SubmitSnapshot(const PhaseVofaSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    s_phase_vofa_snapshot = *snapshot;
    taskEXIT_CRITICAL();
}

uint32_t PhaseVofa_GetFrameCount(void)
{
    return s_phase_vofa_frame_count;
}

uint32_t PhaseVofa_GetOverwriteCount(void)
{
    return s_phase_vofa_overwrite_count;
}
