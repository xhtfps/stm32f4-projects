#include "phase_vofa.h"

#include "./BSP/DEBUG/debug_uart6.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define PHASE_VOFA_TASK_PRIORITY     2U
#define PHASE_VOFA_TASK_STACK_WORDS  384U
#define PHASE_VOFA_QUEUE_LENGTH      1U

typedef struct
{
    int16_t channel1[PHASE_VOFA_POINT_COUNT];
    int16_t channel2[PHASE_VOFA_POINT_COUNT];
} PhaseVofaFrame_t;

static QueueHandle_t s_phase_vofa_queue;
static volatile uint32_t s_phase_vofa_frame_count;
static volatile uint32_t s_phase_vofa_overwrite_count;
static volatile float s_phase_vofa_timer_frequency_hz;
static volatile uint32_t s_phase_vofa_timer_period_ticks;
static volatile uint8_t s_phase_vofa_timer_valid;

static void PhaseVofa_Task(void *argument)
{
    PhaseVofaFrame_t frame;
    uint16_t index;
    float timer_frequency;
    float timer_period;
    float timer_wave;

    (void)argument;
    for (;;)
    {
        if (xQueueReceive(s_phase_vofa_queue, &frame, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        for (index = 0U; index < PHASE_VOFA_POINT_COUNT; ++index)
        {
            timer_frequency = (s_phase_vofa_timer_valid != 0U) ?
                              s_phase_vofa_timer_frequency_hz : 0.0f;
            timer_period = (s_phase_vofa_timer_valid != 0U) ?
                           (float)s_phase_vofa_timer_period_ticks : 0.0f;
            /* TIM_WAVE is a reconstructed 50%% square wave from capture edges. */
            timer_wave = (s_phase_vofa_timer_valid != 0U) &&
                         (index >= (PHASE_VOFA_POINT_COUNT / 2U)) ?
                         1.0f : 0.0f;

            /* FireWater: CH1:value1,value2,value3,value4,value5. */
            (void)DebugUart6_VofaFivePrintfBlocking(
                "CH1", (float)frame.channel1[index],
                "CH2", (float)frame.channel2[index],
                "TIM_FREQ", timer_frequency,
                "TIM_PERIOD", timer_period,
                "TIM_WAVE", timer_wave);
        }
        ++s_phase_vofa_frame_count;
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
    s_phase_vofa_timer_frequency_hz = 0.0f;
    s_phase_vofa_timer_period_ticks = 0U;
    s_phase_vofa_timer_valid = 0U;
    return (xTaskCreate(PhaseVofa_Task, "vofa",
                        PHASE_VOFA_TASK_STACK_WORDS, NULL,
                        PHASE_VOFA_TASK_PRIORITY, NULL) == pdPASS) ?
           1U : 0U;
}

void PhaseVofa_SubmitBlock(const int16_t *channel1,
                           const int16_t *channel2,
                           uint16_t length)
{
    PhaseVofaFrame_t frame;
    uint16_t index;
    uint16_t source_index;

    if ((s_phase_vofa_queue == NULL) || (channel1 == NULL) ||
        (channel2 == NULL) || (length == 0U))
    {
        return;
    }

    for (index = 0U; index < PHASE_VOFA_POINT_COUNT; ++index)
    {
        source_index = (uint16_t)(((uint32_t)index * length) /
                                  PHASE_VOFA_POINT_COUNT);
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
    s_phase_vofa_timer_frequency_hz = frequency_hz;
    s_phase_vofa_timer_period_ticks = period_ticks;
    s_phase_vofa_timer_valid = (valid != 0U) ? 1U : 0U;
}

uint32_t PhaseVofa_GetFrameCount(void)
{
    return s_phase_vofa_frame_count;
}

uint32_t PhaseVofa_GetOverwriteCount(void)
{
    return s_phase_vofa_overwrite_count;
}
