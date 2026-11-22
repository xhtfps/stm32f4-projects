/**
 * @file    freertos_demo.c
 * @brief   F 题 FreeRTOS 任务框架：按键、LCD 与控制状态机。
 *          本文件创建并管理系统中所有 FreeRTOS 任务和队列，实现
 *          PS/2 键盘与板载按键输入、LCD 界面显示刷新、仪器测量流程
 *          的协同工作。
 */

#include "freertos_demo.h"

#include "./BSP/KEY/key.h"            /* 板载按键驱动 */
#include "./BSP/PS2/ps2.h"            /* PS/2 键盘驱动 */
#include "FreeRTOS.h"
#include "queue.h"                    /* FreeRTOS 队列 */
#include "task.h"                     /* FreeRTOS 任务 */

#include "f_app.h"                    /* F 题测频/视觉控制应用层 */
#include "user.h"                     /* 用户相关定义（按键值、事件等） */

/* ---------- 任务优先级定义 ---------- */
#define START_TASK_PRIO             1U   /* 启动任务优先级（最低，创建完其他任务后自删） */
#define KEY_TASK_PRIO               4U   /* 按键扫描任务优先级 */
#define UI_TASK_PRIO                3U   /* 用户界面刷新任务优先级 */
#define INSTRUMENT_TASK_PRIO        5U   /* 仪器测量任务优先级（最高，保证测量实时性） */

/* ---------- 任务栈大小（字） ---------- */
#define START_TASK_STACK_SIZE       128U
#define KEY_TASK_STACK_SIZE         128U
#define UI_TASK_STACK_SIZE          512U
#define INSTRUMENT_TASK_STACK_SIZE  512U

/* UI 任务轮询按键队列的阻塞时间，同时也是界面刷新间隔 */
#define UI_REFRESH_PERIOD_MS         5U

/**
 * 测量结果队列长度。
 * 由于 ADC 采样速率远高于 LCD 刷新速率，这里只保留最新一个测量快照，
 * 防止高优先级生产者饿死 UI 任务（UI 任务可能正忙于处理按键）。
 */
#define MEASUREMENT_QUEUE_LENGTH    1U

/* ---------- 全局队列句柄 ---------- */
static QueueHandle_t s_key_queue;             /* 按键事件队列，元素类型 PS2_KeyValue_t */
static QueueHandle_t s_command_queue;         /* UI 发出的控制命令队列，元素类型 UI_Command_t */
static QueueHandle_t s_measurement_queue;     /* 主测量结果队列（仅保留最新值） */
static QueueHandle_t s_control_progress_queue;/* 自动控制进度队列（仅保留最新值） */

/* ---------- 全局任务句柄 ---------- */
static TaskHandle_t s_instrument_task_handle; /* 仪器任务句柄，用于通知停止 */

/* ---------- 全局标志 ---------- */
static volatile uint8_t s_stop_requested;     /* 仪器任务停止请求标志 */

/* ---------- 函数声明 ---------- */
static void StartTask(void *argument);
static void KeyTask(void *argument);
static void UiTask(void *argument);
static void InstrumentTask(void *argument);
static void Instrument_PublishControlProgress(
    const UI_Measurement_t *measurement);

/**
 * @brief  将板载按键的硬件键值映射为统一按键值（PS/2 键值空间）。
 * @param  board_key 板载按键原始值（来自 key_scan()）
 * @return 对应的 PS/2 键值，如果没有有效按键则返回 PS2_KEY_VALUE_NULL
 */
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

/**
 * @brief  请求停止当前仪器测量（例如扫频或 ADS 测试）。
 *         设置停止标志，并通过任务通知唤醒仪器任务，使其快速响应。
 */
void Instrument_RequestStop(void)
{
    s_stop_requested = 1U;
    if (s_instrument_task_handle != NULL)
    {
        /* 直接给仪器任务一个通知，即使它在等待队列也能立刻退出阻塞 */
        (void)xTaskNotifyGive(s_instrument_task_handle);
    }
}

/**
 * @brief  清除停止请求标志（通常在新一轮测量启动时调用）。
 */
static void Instrument_ClearStopRequest(void)
{
    s_stop_requested = 0U;
}

/**
 * @brief  测量结果发布回调，由 F 题应用层调用。
 *         根据事件类型选择不同的队列投递数据：
 *         - 自动控制进度 → control_progress_queue
 *         - 其他测量结果 → measurement_queue（覆盖最新值）
 * @param  measurement 测量结果结构体指针
 */
static void Instrument_Publish(const UI_Measurement_t *measurement)
{
    if ((measurement == NULL) || (s_measurement_queue == NULL))
    {
        return;
    }

    /* 自动扫频/锁定进度使用专用覆盖队列。 */
    if (measurement->event == UI_EVENT_CONTROL_PROGRESS)
    {
        Instrument_PublishControlProgress(measurement);
        return;
    }

    /* 覆盖写入，保证队列里始终是最新的测量值 */
    (void)xQueueOverwrite(s_measurement_queue, measurement);
}

/**
 * @brief  将自动控制进度写入专用队列（覆盖最新值）。
 * @param  measurement 测量结果结构体指针
 */
static void Instrument_PublishControlProgress(
    const UI_Measurement_t *measurement)
{
    if ((measurement != NULL) &&
        (s_control_progress_queue != NULL))
    {
        (void)xQueueOverwrite(
            s_control_progress_queue, measurement);
    }
}

/**
 * @brief  FreeRTOS 启动入口：创建队列、启动任务和调度器。
 *         该函数由 main() 调用，此后不会返回。
 */
void freertos_demo(void)
{
    /* 创建按键队列（最多缓存 8 次按键） */
    s_key_queue = xQueueCreate(8U, sizeof(PS2_KeyValue_t));
    /* 创建命令队列（最多缓存 4 条 UI 指令） */
    s_command_queue = xQueueCreate(4U, sizeof(UI_Command_t));
    /* 测量结果队列（只保留最新） */
    s_measurement_queue = xQueueCreate(MEASUREMENT_QUEUE_LENGTH,
                                       sizeof(UI_Measurement_t));
    /* 自动控制进度队列（只保留最新） */
    s_control_progress_queue =
        xQueueCreate(1U, sizeof(UI_Measurement_t));

    /* 任一队列创建失败则直接返回（实际工程中可增加错误处理） */
    if ((s_key_queue == NULL) || (s_command_queue == NULL) ||
        (s_measurement_queue == NULL) ||
        (s_control_progress_queue == NULL))
    {
        return;
    }

    /* 创建启动任务，该任务负责创建其他所有任务 */
    (void)xTaskCreate(StartTask, "start", START_TASK_STACK_SIZE,
                      NULL, START_TASK_PRIO, NULL);

    /* 启动调度器，此后控制权交给 FreeRTOS，不会返回 */
    vTaskStartScheduler();

    /* 正常情况下不会执行到这里 */
    for (;;)
    {
    }
}

/**
 * @brief  启动任务：一次性创建按键、UI、仪器三个任务，然后自删除。
 *         使用临界区保证任务创建过程的原子性。
 */
static void StartTask(void *argument)
{
    (void)argument;

    taskENTER_CRITICAL();
    /* 创建按键扫描任务 */
    (void)xTaskCreate(KeyTask, "key", KEY_TASK_STACK_SIZE,
                      NULL, KEY_TASK_PRIO, NULL);
    /* 创建 UI 界面任务 */
    (void)xTaskCreate(UiTask, "ui", UI_TASK_STACK_SIZE,
                      NULL, UI_TASK_PRIO, NULL);
    /* 创建仪器控制任务，并保存其任务句柄以便后续通知 */
    (void)xTaskCreate(InstrumentTask, "instrument",
                      INSTRUMENT_TASK_STACK_SIZE, NULL,
                      INSTRUMENT_TASK_PRIO, &s_instrument_task_handle);
    taskEXIT_CRITICAL();

    /* 启动任务使命完成，删除自身 */
    vTaskDelete(NULL);
}

/**
 * @brief  按键扫描任务：周期性扫描 PS/2 键盘和板载按键，
 *         将有效按键事件发送到按键队列。
 */
static void KeyTask(void *argument)
{
    PS2_KeyValue_t key;
    PS2_KeyValue_t board_key_value;
    uint8_t board_key;

    (void)argument;

    for (;;)
    {
        /* 扫描一次 PS/2 键盘 */
        PS2_KeyScan();
        key = PS2_GetCurrentKeyValue();
        if (key != PS2_KEY_VALUE_NULL)
        {
            /* 非空键值立即入队（非阻塞） */
            (void)xQueueSend(s_key_queue, &key, 0U);
        }

        /* 扫描板载按键（参数 0 表示支持连按） */
        board_key = key_scan(0U);
        board_key_value = KeyTask_MapBoardKey(board_key);
        if (board_key_value != PS2_KEY_VALUE_NULL)
        {
            (void)xQueueSend(s_key_queue, &board_key_value, 0U);
        }

        /* 10ms 扫描周期，平衡响应速度与 CPU 占用 */
        vTaskDelay(pdMS_TO_TICKS(10U));
    }
}

/**
 * @brief  UI 任务：处理按键输入、更新 LCD 显示。
 *         优先处理自动控制进度，再处理常规测量结果，
 *         最后以固定间隔检查按键队列并执行相应的界面逻辑。
 */
static void UiTask(void *argument)
{
    PS2_KeyValue_t key;
    UI_Command_t command;
    UI_Measurement_t measurement;

    (void)argument;

    /* 初始化用户界面（LCD、状态机等） */
    UI_Init();

    for (;;)
    {
        /* 1. 非阻塞处理扫频/锁定进度，保证状态显示实时性 */
        if (xQueueReceive(
                s_control_progress_queue,
                &measurement, 0U) == pdPASS)
        {
            UI_UpdateMeasurement(&measurement);
        }

        /* 2. 非阻塞处理最新常规测量结果 */
        if (xQueueReceive(s_measurement_queue, &measurement, 0U) == pdPASS)
        {
            UI_UpdateMeasurement(&measurement);
        }

        /* 3. 以 UI_REFRESH_PERIOD_MS 超时等待按键，
         *    超时即进入下一次循环，保证定时刷新界面。 */
        if (xQueueReceive(s_key_queue, &key,
                          pdMS_TO_TICKS(UI_REFRESH_PERIOD_MS)) == pdPASS)
        {
            /* 退格键 = 全局停止测量 */
            if (key == PS2_KEY_VALUE_BACKSPACE)
            {
                Instrument_RequestStop();
            }

            /* 将按键交给界面逻辑处理，可能会产生 UI 命令 */
            if (UI_HandleKey(key, &command) &&
                (command.type != UI_COMMAND_NONE))
            {
                /* 如果 UI 逻辑发出停止输出命令，也请求停止仪器 */
                if (command.type == UI_COMMAND_STOP_OUTPUT)
                {
                    Instrument_RequestStop();
                }
                /* 将有效命令发送给仪器任务 */
                (void)xQueueSend(s_command_queue, &command, 0U);
            }
        }
    }
}

/**
 * @brief  仪器任务：初始化 F 题应用，循环接收 UI 命令并执行状态机。
 *         使用回调函数 Instrument_Publish 将结果推送给 UI。
 */
static void InstrumentTask(void *argument)
{
    UI_Command_t command;
    uint8_t initialized;

    (void)argument;

    /* 清除可能遗留的停止请求 */
    Instrument_ClearStopRequest();

    /* 注册测量结果发布回调 */
    f_app_set_measurement_publisher(Instrument_Publish);

    /* 初始化 F 题应用层及 TIM2 输入捕获。 */
    initialized = f_app_init();
    if (initialized == 0U)
    {
        /* 初始化失败时稍作延迟，避免错误日志刷屏 */
        vTaskDelay(pdMS_TO_TICKS(100U));
    }

    for (;;)
    {
        /* 以极短超时等待 UI 命令，非阻塞，保证测量步进不被长时间阻塞 */
        if (xQueueReceive(s_command_queue, &command, 0U) == pdPASS)
        {
            f_app_handle_command(&command);
        }

        /* 执行一次测频/自动控制状态机步进。 */
        f_app_step();
    }
}
