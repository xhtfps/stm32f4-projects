/**
 * @file    frequency_meter.h
 * @brief   F 题调试用定时器测频服务。
 *
 * 输入信号由 PB10/TIM2_CH3 捕获，AD9959 输出反馈由
 * PB11/TIM2_CH4 捕获。两路共用 84 MHz、32 位 TIM2 时间戳，
 * 分别通过循环 DMA 采集。本模块负责把时间戳转换为频率，并完成
 * 中值滤波、指数平滑、抖动估计和无信号超时判断。
 *
 * PB10 输入频率按题目要求另行生成 100 Hz 整数频点；PB11 反馈频率
 * 不量化，保留浮点精度供后续 DPLL/PI 闭环使用。
 *
 * 注意：本模块用于前期硬件联调和 DDS 校准。题目“自动模式”要求
 * 信号源与装置之间无电气连接，因此最终自动搜索仍必须使用 K230
 * 的纯视觉测量结果，不能把本测频输入接到未知信号源上。
 */

#ifndef FREQUENCY_METER_H
#define FREQUENCY_METER_H

#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "./BSP/PHASE/phase_capture.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FREQUENCY_METER_MAX_WINDOW  9U

typedef enum
{
    FREQUENCY_METER_STATUS_STOPPED = 0U,
    FREQUENCY_METER_STATUS_WAIT_SIGNAL,
    FREQUENCY_METER_STATUS_VALID,
    FREQUENCY_METER_STATUS_OUT_OF_RANGE,
    FREQUENCY_METER_STATUS_HARDWARE_ERROR
} FrequencyMeterStatus_t;

typedef struct
{
    uint32_t valid_min_hz;       /* 最低频点，固定为 900 Hz */
    uint32_t valid_max_hz;       /* 最高频点，固定为 100 kHz */
    uint32_t frequency_step_hz;  /* 离散频点步进，固定为 100 Hz */
    float ema_alpha;             /* 指数平滑系数，范围 (0, 1] */
    uint32_t signal_timeout_ms;  /* 多久没有新边沿后判定无信号 */
    uint8_t median_window;       /* 中值窗口，必须为 1、3、5、7 或 9 */
} FrequencyMeterConfig_t;

typedef struct
{
    FrequencyMeterStatus_t status;
    float raw_frequency_hz;       /* 最近一次未滤波频率 */
    float filtered_frequency_hz;  /* 中值 + EMA 后供显示/控制使用 */
    uint32_t frequency_hz;        /* 量化后的频率，恒为 100 Hz 整数倍 */
    uint32_t frequency_100hz;     /* 频点序号：frequency_hz / 100 Hz */
    float window_min_hz;          /* 当前滑动窗口最小值 */
    float window_max_hz;          /* 当前滑动窗口最大值 */
    float jitter_ppm;             /* 窗口峰峰值 / 中心频率，单位 ppm */
    uint32_t period_ticks;        /* 本次统计所覆盖的 TIM2 计数值 */
    uint32_t capture_cycles;      /* 本次统计实际覆盖的输入周期数 */
    uint32_t sample_count;        /* 已接受的测量次数 */
    uint32_t signal_age_ms;       /* 距最后一次有效边沿的时间 */
    uint32_t dma_error_count;     /* TIM2 捕获 DMA 累计错误数 */
    uint8_t valid;

    float feedback_raw_frequency_hz;      /* PB11 最近一次原始频率 */
    float feedback_filtered_frequency_hz; /* PB11 中值+EMA精确频率 */
    uint32_t feedback_period_ticks;       /* PB11 统计覆盖的TIM2 tick */
    uint32_t feedback_capture_cycles;     /* PB11 统计覆盖的输出周期 */
    uint32_t feedback_sample_count;       /* PB11 已接受样本数 */
    uint32_t feedback_signal_age_ms;      /* PB11 距最后有效边沿时间 */
    uint32_t feedback_dma_error_count;    /* PB11 DMA累计错误 */
    uint8_t feedback_valid;               /* PB11 精确频率有效 */
} FrequencyMeterResult_t;

typedef struct FrequencyMeter FrequencyMeter_t;

typedef struct
{
    HAL_StatusTypeDef (*init)(FrequencyMeter_t *self,
                              const FrequencyMeterConfig_t *config);
    HAL_StatusTypeDef (*start)(FrequencyMeter_t *self);
    HAL_StatusTypeDef (*stop)(FrequencyMeter_t *self);
    uint8_t (*process)(FrequencyMeter_t *self);
    const FrequencyMeterResult_t *(*get_result)(
        const FrequencyMeter_t *self);
} FrequencyMeterOps_t;

struct FrequencyMeter
{
    PhaseCapture_t *capture;
    FrequencyMeterConfig_t config;
    FrequencyMeterResult_t result;
    float median_buffer[FREQUENCY_METER_MAX_WINDOW];
    float feedback_median_buffer[FREQUENCY_METER_MAX_WINDOW];
    uint32_t last_valid_tick_ms;
    uint32_t feedback_last_valid_tick_ms;
    uint8_t median_count;
    uint8_t median_write_index;
    uint8_t feedback_median_count;
    uint8_t feedback_median_write_index;
    uint8_t initialized;
    uint8_t running;
    const FrequencyMeterOps_t *ops;
};

extern FrequencyMeter_t g_frequency_meter;

void FrequencyMeterConfig_Default(FrequencyMeterConfig_t *config);
void FrequencyMeter_Construct(FrequencyMeter_t *self);

/* 便于普通业务代码调用的默认对象封装。 */
HAL_StatusTypeDef frequency_meter_init(
    const FrequencyMeterConfig_t *config);
HAL_StatusTypeDef frequency_meter_start(void);
HAL_StatusTypeDef frequency_meter_stop(void);
uint8_t frequency_meter_process(void);
const FrequencyMeterResult_t *frequency_meter_get_result(void);

#ifdef __cplusplus
}
#endif

#endif /* FREQUENCY_METER_H */
