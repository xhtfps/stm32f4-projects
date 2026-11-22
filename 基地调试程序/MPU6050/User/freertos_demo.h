#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H

/**
 * @file    freertos_demo.h
 * @brief   FreeRTOS 演示任务头文件
 *
 * 声明用于创建 FreeRTOS 演示任务的入口函数。
 * 开发新项目时，在此添加新的任务声明。
 */

/**
 * @brief  FreeRTOS 演示任务入口
 *
 * 该函数会创建多个 FreeRTOS 演示任务（如 LED 闪烁、LCD 显示、按键扫描等），
 * 用于验证系统多任务运行环境。
 */
void freertos_demo(void);

#endif /* __FREERTOS_DEMO_H */
