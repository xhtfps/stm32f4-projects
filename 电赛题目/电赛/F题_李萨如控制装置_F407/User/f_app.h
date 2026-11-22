/**
 * @file    f_app.h
 * @brief   F 题应用层统一入口。
 *
 * 提供应用层初始化、主循环步进、UI 命令处理以及测量数据发布器注册接口。
 * 所有对外类型通过 user.h 中的 UI_Measurement_t / UI_Command_t 传递。
 */

#ifndef F_APP_H
#define F_APP_H

#include <stdint.h>

#include "user.h"

/**
 * @brief 测量数据发布回调函数类型。
 *
 * 由 UI 或其他模块注册，当有新的测量数据时由 f_app 模块调用。
 *
 * @param measurement 指向最新测量数据的指针，不可为 NULL。
 */
typedef void (*FAppMeasurementPublisher_t)(
    const UI_Measurement_t *measurement);

/**
 * @brief 注册测量数据发布回调。
 *
 * 仅允许注册一个发布器，后注册的会覆盖之前的。
 *
 * @param publisher 回调函数指针，传入 NULL 可取消注册。
 */
void f_app_set_measurement_publisher(
    FAppMeasurementPublisher_t publisher);

/**
 * @brief 初始化 F 题应用模块。
 *
 * 依次完成数据结构初始化、频率计配置、DDS 初始化以及双 ADC 测量模块启动。
 * 任一外设初始化失败即进入错误状态。
 *
 * @return 1 表示初始化成功，0 表示失败。
 */
uint8_t f_app_init(void);

/**
 * @brief 应用层主循环步进函数。
 *
 * 需要在 RTOS 任务中周期性调用。负责：
 * - 运行频率计处理并更新测量数据；
 * - 根据当前模式驱动 DDS 输出；
 * - 运行有线测量（双 ADC）处理；
 * - 按固定周期将测量结果推送给注册的发布器。
 *
 * 内部通过 vTaskDelay 释放 CPU，确保不影响其他任务调度。
 */
void f_app_step(void);

/**
 * @brief 处理 UI 模块下发的命令。
 *
 * 支持的命令包括：
 *   - 启动要求 1 / 2 / 3 / 4 / 5
 *   - 启动调试模式
 *   - 启动/停止菜单5中的AD9959 CH0测试
 *   - 停止所有输出
 *
 * @param command 指向 UI 命令的指针，不可为 NULL。
 *                若模块尚未初始化，该函数不执行任何操作。
 */
void f_app_handle_command(const UI_Command_t *command);

#endif /* F_APP_H */
