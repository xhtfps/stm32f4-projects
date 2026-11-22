/**
 * @file    main.c
 * @brief   电路模型探究装置主程序入口
 *
 * 完成 STM32 外设与硬件模块的初始化，然后启动 FreeRTOS 多任务演示。
 */

#include "./SYSTEM/sys/sys.h"           /* 系统时钟与基础配置 */
#include "./SYSTEM/usart/usart.h"       /* 串口通信 */
#include "./SYSTEM/delay/delay.h"       /* 微秒/毫秒延时 */
#include "./BSP/LED/led.h"              /* LED 指示灯驱动 */
#include "./BSP/TFT_LCD/TFT_LCD.h"      /* TFT LCD 显示驱动 */
#include "./BSP/TFT_LCD/fontupd.h"      /* 字库更新与检测 */
#include "./BSP/PS2/ps2.h"              /* PS/2 键盘驱动 */
#include "./MALLOC/malloc.h"            /* 内存管理 */
#include "./BSP/TIMER/btim.h"           /* 基本定时器驱动 */
#include "./BSP/DEBUG/debug_uart6.h"    /* USART6 调试输出：PC6/PC7 */
#include "freertos_demo.h"              /* FreeRTOS 演示任务声明 */
#include "user.h"                       /* UI 模块接口与常量 */

/**
 * @brief  主函数
 * @return int (实际不会返回)
 */
int main(void)
{
    /* ---------- HAL 库与系统时钟 ---------- */
    HAL_Init();                                    /* 初始化 HAL 库 */
    sys_stm32_clock_init(336, 8, 2, 7);            /* 配置系统时钟：336MHz PLL, 8分频AHB, 2分频APB1, 7分频APB2 */
    delay_init(168);                               /* 初始化延时函数（系统时钟 168MHz） */
    usart_init(115200);                            /* 初始化串口，波特率 115200 */
    DebugUart6_Init(230400U);                      /* USART6：PC6=TX，PC7=RX */

    /* ---------- 板级外设 ---------- */
    led_init();                                    /* 初始化 LED 指示灯 */
    TFT_LCD_Init();                                /* 初始化 TFT LCD 屏幕 */

    /* 检测并初始化字库，根据结果设置 UI 中文显示能力 */
    UI_SetChineseFontAvailable((font_init() == 0U) ? 1U : 0U);

    PS2_Keyboard_Init();                           /* 初始化 PS/2 键盘 */

    /* 基本定时器 TIM4 中断初始化：1ms 周期
       参数：(1000-1) 预分频值，(72-1) 自动重载值 -> 72MHz/72=1MHz -> 1us计数，
       重载 1000-1 得到 1ms 中断 */
    btim_tim4_int_init(1000 - 1, 72 - 1);

    /* ---------- 内存管理 ---------- */
    my_mem_init(SRAMIN);                           /* 初始化内部 SRAM 内存池 */
    my_mem_init(SRAMCCM);                          /* 初始化 CCM 内存池（核心耦合内存） */

    /* ---------- 启动 FreeRTOS 演示程序 ---------- */
    freertos_demo();                               /* 创建任务并启动调度器，函数不会返回 */

    /* 正常情况下不会执行到这里，为完整性保留无限循环 */
    while (1)
    {
    }
}
