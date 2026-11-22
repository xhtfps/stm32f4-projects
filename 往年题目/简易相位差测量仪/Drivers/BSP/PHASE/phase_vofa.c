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

static void PhaseVofa_CopySnapshot(PhaseVofaSnapshot_t *snapshot)
{
    taskENTER_CRITICAL();
    *snapshot = s_phase_vofa_snapshot;
    taskEXIT_CRITICAL();
}

static void PhaseVofa_SendFireWater(float channel1, float channel2)
{
    if (s_phase_vofa_output_enabled == 0U)
    {
        return;
    }

    (void)DebugUart6_VofaPrintfBlocking(
        "ADC",
        "%.0f,%.0f",
        (double)channel1,
        (double)channel2);
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
                    (float)frame.channel1[index],
                    (float)frame.channel2[index]);
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
    PhaseVofaSnapshot_t snapshot;
    uint32_t analysis_frequency_hz;
    uint32_t decimation;
    uint64_t decimation_denominator;
    uint16_t index;
    uint16_t source_index;

    if ((s_phase_vofa_queue == NULL) || (channel1 == NULL) ||
        (channel2 == NULL) || (length == 0U))
    {
        return;
    }

    PhaseVofa_CopySnapshot(&snapshot);
    analysis_frequency_hz =
        (snapshot.adc_mode != 0U) ?
        ((snapshot.frequency_hz >=
          (float)snapshot.local_oscillator_hz) ?
         (uint32_t)(snapshot.frequency_hz -
                    (float)snapshot.local_oscillator_hz) :
         (uint32_t)((float)snapshot.local_oscillator_hz -
                    snapshot.frequency_hz)) :
        (uint32_t)snapshot.frequency_hz;
    decimation = 1U;
    if ((analysis_frequency_hz != 0U) &&
        (snapshot.adc_sample_rate_hz != 0U))
    {
        decimation_denominator =
            (uint64_t)analysis_frequency_hz * 8ULL;
        decimation = (uint32_t)(
            (uint64_t)snapshot.adc_sample_rate_hz /
            decimation_denominator);
        if (decimation == 0U)
        {
            decimation = 1U;
        }
    }

    for (index = 0U; index < PHASE_VOFA_POINT_COUNT; ++index)
    {
        source_index =
            (uint16_t)((uint32_t)index * decimation);
        if (source_index >= length)
        {
            source_index = (uint16_t)(length - 1U);
        }
        frame.channel1[index] = channel1[source_index];
        frame.channel2[index] = channel2[source_index];
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
