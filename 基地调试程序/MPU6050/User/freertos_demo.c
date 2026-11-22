/**
 * @file    freertos_demo.c
 * @brief   FreeRTOS 模板演示程序
 *
 * 本模块创建多个 FreeRTOS 演示任务：
 *   - LedTask：LED 闪烁，验证任务调度
 *   - LcdTask：LCD 显示演示，验证显示驱动
 *   - KeyTask：按键扫描演示（使用标准 GPIO 按键）
 *
 * 任务间通过 FreeRTOS 队列进行通信。
 * 开发新项目时，在此添加自定义任务。
 */

#include "freertos_demo.h"

#include "./BSP/LED/led.h"              /* LED 驱动 */
#include "./BSP/KEY/key.h"              /* 按键驱动 */
#include "./BSP/PS2/ps2.h"              /* PS/2 键盘驱动 */
#include "user.h"                       /* UI 模块接口 */

#include "FreeRTOS.h"
#include "queue.h"                      /* 消息队列 */
#include "task.h"                       /* 任务管理 */

/* ================================================================
 * 任务优先级与栈大小（用户可根据需要调整）
 * ================================================================ */

/** 启动任务优先级（最低，仅用于创建其他任务） */
#define START_TASK_PRIO        1U
/** 启动任务栈大小（字） */
#define START_TASK_STACK_SIZE  128U

/** LED 闪烁任务优先级 */
#define LED_TASK_PRIO          2U
/** LED 任务栈大小 */
#define LED_TASK_STACK_SIZE    128U

/** LCD 显示任务优先级 */
#define LCD_TASK_PRIO          3U
/** LCD 任务栈大小（含字符串操作） */
#define LCD_TASK_STACK_SIZE    256U

/** 按键扫描任务优先级 */
#define KEY_TASK_PRIO          4U
/** 按键任务栈大小 */
#define KEY_TASK_STACK_SIZE    128U

/* ================================================================
 * 队列句柄
 * ================================================================ */

/** 按键消息来源 */
typedef enum
{
    KEY_SOURCE_GPIO = 0U,
    KEY_SOURCE_PS2  = 1U
} KeySource_t;

/** 按键消息：KeyTask -> LcdTask */
typedef struct
{
    KeySource_t source;
    uint8_t value;
} KeyMessage_t;

static QueueHandle_t s_key_queue;

/* ================================================================
 * 内部任务函数声明
 * ================================================================ */
static void StartTask(void *argument);
static void LedTask(void *argument);
static void LcdTask(void *argument);
static void KeyTask(void *argument);

/* ================================================================
 * freertos_demo：入口函数，创建队列与启动任务
 * ================================================================ */
void freertos_demo(void)
{
    /* 创建队列 */
    s_key_queue = xQueueCreate(8U, sizeof(KeyMessage_t));
    /* 队列创建失败则直接返回 */
    if (s_key_queue == NULL)
    {
        return;
    }

    /* 创建启动任务，由该任务负责创建其余任务 */
    (void)xTaskCreate(StartTask, "start", START_TASK_STACK_SIZE, NULL,
                      START_TASK_PRIO, NULL);
    /* 启动调度器，此后由 FreeRTOS 接管 */
    vTaskStartScheduler();
}

/* ================================================================
 * StartTask：初始化其他任务后自删除
 * ================================================================ */
static void StartTask(void *argument)
{
    (void)argument;
    taskENTER_CRITICAL();   /* 临界区保护，避免任务创建被打断 */

    (void)xTaskCreate(LedTask, "led", LED_TASK_STACK_SIZE, NULL,
                      LED_TASK_PRIO, NULL);
    (void)xTaskCreate(LcdTask, "lcd", LCD_TASK_STACK_SIZE, NULL,
                      LCD_TASK_PRIO, NULL);
    (void)xTaskCreate(KeyTask, "key", KEY_TASK_STACK_SIZE, NULL,
                      KEY_TASK_PRIO, NULL);

    taskEXIT_CRITICAL();
    vTaskDelete(NULL);      /* 启动任务自身不再需要 */
}

/* ================================================================
 * LedTask：LED 闪烁演示
 * ================================================================ */
static void LedTask(void *argument)
{
    uint8_t led_state = 0U;

    (void)argument;
    for (;;)
    {
        led_state ^= 1U;                   /* 翻转 LED 状态 */
        if (led_state != 0U)
        {
            LED1(0);                       /* 点亮 LED1（红灯，低电平有效） */
        }
        else
        {
            LED1(1);                       /* 熄灭 LED1 */
        }
        vTaskDelay(pdMS_TO_TICKS(500U));   /* 500ms 周期闪烁 */
    }
}

/* ================================================================
 * LcdTask：LCD 显示演示
 * ================================================================ */
static void LcdTask(void *argument)
{
    KeyMessage_t key_message;
    UI_Command_t command;

    (void)argument;
    UI_Init();                              /* 初始化 UI，绘制主界面 */

    for (;;)
    {
        /* 检查是否有按键消息 */
        if (xQueueReceive(s_key_queue, &key_message, pdMS_TO_TICKS(100U)) == pdPASS)
        {
            if (key_message.source == KEY_SOURCE_PS2)
            {
                /* PS/2 数字键 1~5 进入对应菜单，退格键返回主页 */
                (void)UI_HandleKey((PS2_KeyValue_t)key_message.value, &command);
            }
            else
            {
                /* 保留原有板载 GPIO 按键行为 */
                switch (key_message.value)
                {
                    case KEY1_PRES:             /* KEY1 → 菜单 1 (MPU6050 测试) */
                        Change_Menu(1U);
                        break;

                    case KEY2_PRES:             /* KEY2 → 菜单 2 */
                        Change_Menu(2U);
                        break;

                    case KEY3_PRES:             /* KEY3 → 返回主菜单 */
                        Change_Menu(0U);
                        break;

                    case WKUP_PRES:             /* WK_UP 预留 */
                    default:
                        break;
                }
            }
        }

        /* 如果当前在 MPU6050 测试页（菜单 1），则周期性刷新传感器数据 */
        if (UI_GetCurrentPage() == 1U)  /* PAGE_SINE */
        {
            UI_Mpu6050_UpdateData();
        }
        else if (UI_GetCurrentPage() == 2U)  /* AD7606 测试页 */
        {
            UI_Ad7606_UpdateData();
        }
    }
}

/* ================================================================
 * KeyTask：按键扫描
 * ================================================================ */
static void KeyTask(void *argument)
{
    uint8_t gpio_key;
    PS2_KeyValue_t ps2_key;
    KeyMessage_t key_message;

    (void)argument;
    for (;;)
    {
        /* 扫描按键 */
        gpio_key = key_scan(0U);             /* 0=不等待，立即返回 */

        if (gpio_key != 0U)
        {
            key_message.source = KEY_SOURCE_GPIO;
            key_message.value = gpio_key;
            (void)xQueueSend(s_key_queue, &key_message, 0U);
        }

        /* 读取由 1ms TIM4 状态机解析出的 PS/2 短按事件 */
        PS2_KeyScan();
        ps2_key = PS2_GetCurrentKeyValue();
        if (ps2_key != PS2_KEY_VALUE_NULL)
        {
            key_message.source = KEY_SOURCE_PS2;
            key_message.value = (uint8_t)ps2_key;
            (void)xQueueSend(s_key_queue, &key_message, 0U);
        }

        vTaskDelay(pdMS_TO_TICKS(20U));     /* 20ms 轮询周期 */
    }
}
