#ifndef __FREERTOS_DEMO_H
#define __FREERTOS_DEMO_H

/**
 * @file    freertos_demo.h
 * @brief   FreeRTOS 演示任务头文件
 *
 * 声明用于创建 FreeRTOS 综合演示任务的入口函数。
 */

/**
 * @brief  FreeRTOS 演示任务入口
 *
 * 该函数会创建并启动多个 FreeRTOS 任务（如 UI 刷新、数据采集、
 * 控制逻辑等），用于验证系统多任务运行环境。
 */
void freertos_demo(void);
void Instrument_RequestStop(void);

#endif /* __FREERTOS_DEMO_H */
