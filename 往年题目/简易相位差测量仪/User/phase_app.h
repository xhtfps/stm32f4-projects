/**
 * @file    phase_app.h
 * @brief   相位测量应用层接口
 *
 * 提供相位测量系统的初始化、主循环步进、命令处理以及测量结果发布回调的注册。
 * 上层任务管理模块通过调用这些接口驱动整个测量流程。
 */

#ifndef PHASE_APP_H
#define PHASE_APP_H

#include <stdint.h>

#include "user.h"   /* UI_Measurement_t, UI_Command_t 等用户定义类型 */

/**
 * @brief 测量结果发布回调类型
 * @param measurement 指向待发布的测量结果结构体的指针
 */
typedef void (*PhaseAppMeasurementPublisher_t)(
    const UI_Measurement_t *measurement);

/**
 * @brief  注册测量结果发布回调函数
 * @param  publisher 回调函数指针，phase_app 内部会在产生新测量结果时调用
 */
void phase_app_set_measurement_publisher(
    PhaseAppMeasurementPublisher_t publisher);

/**
 * @brief  初始化相位测量应用（硬件、流水线、捕获等）
 * @return 1: 初始化成功；0: 失败
 */
uint8_t phase_app_init(void);

/**
 * @brief  应用主循环步进函数，应由 FreeRTOS 任务周期性调用
 *
 * 内部依次处理输入捕获结果、ADC 数据块、信号超时检测及慢速回退逻辑。
 */
void phase_app_step(void);

/**
 * @brief  处理来自 UI 的控制命令（启动/停止测量、设置参数等）
 * @param  command 指向命令结构体的指针，不能为 NULL
 */
void phase_app_handle_command(const UI_Command_t *command);

#endif /* PHASE_APP_H */
