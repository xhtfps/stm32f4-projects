/**
 * @file    main.c
 * @brief   工程模板主程序入口
 *
 * 完成 STM32 外设与硬件模块的初始化，然后启动 FreeRTOS 多任务演示。
 * 开发新项目时，根据需要在此添加/删除外设初始化步骤。
 */

#include "./SYSTEM/sys/sys.h"           /* 系统时钟与基础配置 */
#include "./SYSTEM/usart/usart.h"       /* 串口通信 */
#include "./SYSTEM/delay/delay.h"       /* 微秒/毫秒延时 */
#include "./BSP/LED/led.h"              /* LED 指示灯驱动 */
#include "./BSP/TFT_LCD/TFT_LCD.h"      /* TFT LCD 显示驱动 */
#include "./BSP/TFT_LCD/fontupd.h"      /* 字库更新与检测 */
#include "./BSP/KEY/key.h"              /* 按键驱动 */
#include "./BSP/PS2/ps2.h"              /* PS/2 键盘驱动 */
#include "./BSP/TIMER/btim.h"           /* 基本定时器驱动 */
#include "./MALLOC/malloc.h"            /* 内存管理 */
#include "user.h"                       /* UI 初始化接口 */
#include "freertos_demo.h"              /* FreeRTOS 演示任务声明 */

/**
 * @brief  主函数
 * @return int (实际不会返回)
 */
int main(void)
{
    /* ---------- HAL 库与系统时钟 ---------- */
    HAL_Init();                                    /* 初始化 HAL 库 */
    sys_stm32_clock_init(336, 8, 2, 7);            /* 配置系统时钟：336MHz PLL */
    delay_init(168);                               /* 初始化延时函数（系统时钟 168MHz） */
    usart_init(115200);                            /* 初始化串口，波特率 115200 */

    /* ---------- 板级外设 ---------- */
    led_init();                                    /* 初始化 LED 指示灯 */
    TFT_LCD_Init();                                /* 初始化 TFT LCD 屏幕 */

    /* 检测并初始化字库 */
    UI_SetChineseFontAvailable((font_init() == 0U) ? 1U : 0U);

    key_init();                                    /* 初始化按键 */
    PS2_Keyboard_Init();                           /* 初始化 PS/2 键盘及 CLK 外部中断 */

    /* 基本定时器 TIM4 中断初始化：1ms 周期 */
    btim_tim4_int_init(1000 - 1, 72 - 1);

    /* ---------- 内存管理 ---------- */
    my_mem_init(SRAMIN);                           /* 初始化内部 SRAM 内存池 */
    my_mem_init(SRAMCCM);                          /* 初始化 CCM 内存池 */

    /* ---------- 启动 FreeRTOS 演示程序 ---------- */
    freertos_demo();                               /* 创建任务并启动调度器 */

    /* 正常情况下不会执行到这里，为完整性保留无限循环 */
    while (1)
    {
    }
}
