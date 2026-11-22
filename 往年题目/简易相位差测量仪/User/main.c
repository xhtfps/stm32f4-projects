/**
 * @file    main.c
 * @brief   简易相位差测量仪主程序入口
 */

#include "./SYSTEM/sys/sys.h"           /* 系统时钟与基础配置 */
#include "./SYSTEM/usart/usart.h"       /* 串口通信 */
#include "./SYSTEM/delay/delay.h"       /* 微秒/毫秒延时 */
#include "./BSP/LED/led.h"              /* LED 指示灯 */
#include "./BSP/TFT_LCD/TFT_LCD.h"      /* TFT LCD */
#include "./BSP/TFT_LCD/fontupd.h"      /* 外部字库 */
#include "./BSP/KEY/key.h"              /* 板载按键 */
#include "./BSP/PS2/ps2.h"              /* PS/2 键盘 */
#include "./MALLOC/malloc.h"            /* 内存池 */
#include "./BSP/TIMER/btim.h"            /* TIM4 1 ms 基准 */
#include "./BSP/DEBUG/debug_uart6.h"    /* USART6 调试输出：PC6/PC7 */
#include "freertos_demo.h"              /* 原工程 FreeRTOS/UI 任务框架 */
#include "user.h"                       /* 参考工程 UI 接口 */

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
    DebugUart6_SetOutputEnabled(1U);                /* 开启 USART6 底层发送；VOFA 由 ADC 菜单控制 */

    /* 保持参考工程的板级初始化顺序。 */
    led_init();
    TFT_LCD_Init();
    UI_SetChineseFontAvailable((font_init() == 0U) ? 1U : 0U);
    key_init();
    PS2_Keyboard_Init();
    btim_tim4_int_init(1000U - 1U, 72U - 1U);
    my_mem_init(SRAMIN);
    my_mem_init(SRAMCCM);

    freertos_demo();                               /* 创建 UI/按键/测量任务并启动调度器 */

    /* 正常情况下不会执行到这里，为完整性保留无限循环 */
    while (1)
    {
    }
}
