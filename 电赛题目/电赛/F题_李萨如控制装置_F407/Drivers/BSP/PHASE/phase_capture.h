/**
 * @file    phase_capture.h
 * @brief   相位/频率捕获模块头文件
 *
 * 该模块使用 TIM2 的 CH3 (PB10) 和 CH4 (PB11) 分别捕获外部输入信号
 * 和 AD9959 反馈信号。两路信号共享同一 32 位自由运行计数器，计算
 * 频差时无跨定时器时基误差。
 *
 * 支持两种捕获模式：
 * - 快速 DMA 模式：每 8 个边沿触发一次 DMA 传输，适用于高频信号。
 * - 慢速中断模式：每 1 个边沿触发一次中断，适用于低频信号，并在
 *   检测到频率升高后自动切回 DMA 模式。
 *
 * 上层使用 PB10 进行频点识别（结果量化到 100Hz），PB11 的反馈频率
 * 保留原始分辨率，供精确频差闭环控制。
 */

#ifndef PHASE_CAPTURE_H
#define PHASE_CAPTURE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*                        硬件相关宏定义                                       */
/* -------------------------------------------------------------------------- */

/** 使用的定时器外设 */
#define PHASE_CAPTURE_TIMER             TIM2

/** 定时器全局中断号 */
#define PHASE_CAPTURE_TIMER_IRQn        TIM2_IRQn

/** TIM2 输入时钟频率（84MHz） */
#define PHASE_CAPTURE_TIMER_CLOCK_HZ    84000000U

/* ---------- 外部输入信号通道 (PB10) ---------- */
#define PHASE_CAPTURE_INPUT_CHANNEL     TIM_CHANNEL_3
#define PHASE_CAPTURE_INPUT_GPIO_PORT   GPIOB
#define PHASE_CAPTURE_INPUT_GPIO_PIN    GPIO_PIN_10
#define PHASE_CAPTURE_INPUT_GPIO_AF     GPIO_AF1_TIM2

/* ---------- 反馈信号通道 (PB11) ---------- */
#define PHASE_CAPTURE_FEEDBACK_CHANNEL   TIM_CHANNEL_4
#define PHASE_CAPTURE_FEEDBACK_GPIO_PORT GPIOB
#define PHASE_CAPTURE_FEEDBACK_GPIO_PIN  GPIO_PIN_11
#define PHASE_CAPTURE_FEEDBACK_GPIO_AF   GPIO_AF1_TIM2

/*
 * 保留旧版兼容宏，避免已有代码（频率计、测试代码）大规模修改。
 * 这些宏直接映射到输入通道的定义。
 */
#define PHASE_CAPTURE_TIMER_CHANNEL     PHASE_CAPTURE_INPUT_CHANNEL
#define PHASE_CAPTURE_GPIO_PORT         PHASE_CAPTURE_INPUT_GPIO_PORT
#define PHASE_CAPTURE_GPIO_PIN          PHASE_CAPTURE_INPUT_GPIO_PIN
#define PHASE_CAPTURE_GPIO_AF           PHASE_CAPTURE_INPUT_GPIO_AF

/* ---------- 捕获边沿配置 ---------- */
/** 快速模式：每 8 个边沿捕获一次（提高高频信号精度） */
#define PHASE_CAPTURE_FAST_EDGE_COUNT   8U
/** 慢速模式：每 1 个边沿捕获一次（减少低频等待时间） */
#define PHASE_CAPTURE_SLOW_EDGE_COUNT   1U
/** 默认使用的边沿数（即快速模式） */
#define PHASE_CAPTURE_EDGE_COUNT        PHASE_CAPTURE_FAST_EDGE_COUNT

/**
 * 输入捕获预分频器（初始配置）。
 * 注意：这是 TIM2 内部的“每 N 个边沿捕获一次”预分频器，而非
 * 外部 AD9959 OUT1 的 /32 分频。当频率低于约 20kHz 时，切换
 * 到 DIV1（慢速模式），否则保持 DIV8（快速模式）。
 */
#define PHASE_CAPTURE_IC_PRESCALER      TIM_ICPSC_DIV8

/** 输入滤波器采样数：需连续两个定时器时钟高电平才认为有效边沿 */
#define PHASE_CAPTURE_IC_FILTER         1U

/**
 * 慢速模式下自动切回 DMA 的周期阈值（timer ticks）。
 * 若在慢速模式测得周期 <= 4200 ticks（即频率 >= 20kHz），
 * 说明信号频率较高，立即切回快速 DMA 模式。
 */
#define PHASE_CAPTURE_FAST_SWITCH_TICKS 4200U

/* ---------- 定时器标志与中断 ---------- */
/** 输入捕获事件标志（CH3） */
#define PHASE_CAPTURE_TIMER_FLAG        TIM_FLAG_CC3
/** 输入捕获中断使能位（CH3） */
#define PHASE_CAPTURE_TIMER_IT          TIM_IT_CC3
/** 输入通道过捕获标志（CH3） */
#define PHASE_CAPTURE_OVERCAPTURE_FLAG  TIM_FLAG_CC3OF

/* ---------- 主通道 DMA 配置 ---------- */
/** DMA 流 */
#define PHASE_CAPTURE_DMA_STREAM        DMA1_Stream1
/** DMA 通道 */
#define PHASE_CAPTURE_DMA_CHANNEL       DMA_CHANNEL_3
/** DMA 中断号 */
#define PHASE_CAPTURE_DMA_IRQn          DMA1_Stream1_IRQn
/** 循环缓冲区深度（减少高频中断负载） */
#define PHASE_CAPTURE_DMA_BUFFER_COUNT  512U

/* ---------- 反馈通道 DMA 配置 ---------- */
#define PHASE_CAPTURE_FEEDBACK_DMA_STREAM       DMA1_Stream6
#define PHASE_CAPTURE_FEEDBACK_DMA_CHANNEL      DMA_CHANNEL_3
#define PHASE_CAPTURE_FEEDBACK_DMA_IRQn         DMA1_Stream6_IRQn
/** 反馈通道缓冲区较小，降低内存占用 */
#define PHASE_CAPTURE_FEEDBACK_DMA_BUFFER_COUNT 128U

/* -------------------------------------------------------------------------- */
/*                        类型定义                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief 定时器配置参数
 */
typedef struct
{
    uint16_t prescaler;       /**< 定时器预分频器值（最终计数频率 = timer_tick_hz / (prescaler+1)） */
    uint32_t period;          /**< 自动重装载值（决定定时器溢出周期） */
    uint32_t timer_tick_hz;   /**< 定时器输入时钟频率（Hz） */
} PhaseCaptureConfig_t;

/**
 * @brief 相位捕获模块实例结构
 *
 * 维护了定时器、DMA、测量数据及运行状态。由于 DMA 在后台持续工作，
 * 读取测量结果时需要结合包裹计数和缓冲区指针，计算多边沿平均周期。
 */
typedef struct
{
    TIM_HandleTypeDef  timer;                       /**< 定时器句柄 */
    DMA_HandleTypeDef  dma;                         /**< 主通道 DMA 句柄 */
    DMA_HandleTypeDef  feedback_dma;                /**< 反馈通道 DMA 句柄 */
    PhaseCaptureConfig_t config;                    /**< 当前配置副本 */

    volatile uint32_t last_capture;                 /**< 上一次捕获的 CCR 值（用于计算差量） */
    volatile uint32_t period_ticks;                 /**< 最新测量得到的周期（tick 数） */
    volatile uint32_t period_capture_cycles;        /**< 对应周期所覆盖的边沿总数 */
    volatile uint32_t active_capture_cycles;        /**< 当前激活的边沿捕获数（1,2,4,8） */

    volatile uint32_t dma_wrap_count;               /**< 主 DMA 完成（半/全）中断次数，即缓冲区包裹计数 */
    volatile uint32_t dma_error_count;              /**< 主通道 DMA 错误计数 */
    volatile uint32_t feedback_dma_wrap_count;      /**< 反馈 DMA 包裹计数 */
    volatile uint32_t feedback_dma_error_count;     /**< 反馈通道 DMA 错误计数 */

    uint32_t dma_last_sample_count;                 /**< 上次读取时主通道的总样本数 */
    uint32_t dma_last_timestamp;                    /**< 上次读取时主通道最后一个时间戳 */
    uint32_t feedback_dma_last_sample_count;        /**< 上次读取时反馈通道总样本数 */
    uint32_t feedback_dma_last_timestamp;           /**< 上次读取时反馈通道最后一个时间戳 */
    uint32_t phase_last_input_timestamp;            /**< 上次鉴相使用的输入时间戳 */
    uint32_t phase_last_feedback_timestamp;         /**< 上次鉴相使用的反馈时间戳 */

    volatile uint8_t capture_ready;                 /**< 中断模式下新周期数据已就绪 */
    volatile uint8_t has_last_capture;              /**< 是否有上一次捕获值可供计算差量 */
    volatile uint8_t dma_restart_requested;         /**< 中断回调请求主任务重启 DMA 模式 */
    volatile uint8_t dma_running;                   /**< 主通道 DMA 是否正在运行 */
    volatile uint8_t feedback_dma_running;          /**< 反馈通道 DMA 是否正在运行 */
    volatile uint8_t irq_running;                   /**< 中断捕获模式是否正在运行 */

    uint8_t dma_has_last_sample;                    /**< 主通道是否已有历史样本（用于初次对齐） */
    uint8_t feedback_dma_has_last_sample;           /**< 反馈通道是否已有历史样本 */
    uint8_t initialized;                            /**< 模块已初始化标志 */
} PhaseCapture_t;

/** 全局捕获实例，供简易 API 和中断服务使用 */
extern PhaseCapture_t g_phase_capture;

/* -------------------------------------------------------------------------- */
/*                        公开函数声明                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief  初始化默认配置（预分频=0，周期=最大，时钟=84MHz）
 * @param  config 配置结构体指针
 */
void PhaseCaptureConfig_Default(PhaseCaptureConfig_t *config);

/**
 * @brief  初始化捕获模块（配置定时器、DMA、通道）
 * @param  capture 实例指针
 * @param  config  配置参数
 * @return HAL_OK 成功，HAL_ERROR 失败
 */
HAL_StatusTypeDef PhaseCapture_Init(PhaseCapture_t *capture,
                                    const PhaseCaptureConfig_t *config);

/**
 * @brief  启动 DMA 捕获（默认使用快速边沿数）
 * @param  capture 实例指针
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef PhaseCapture_Start(PhaseCapture_t *capture);

/**
 * @brief  停止所有捕获活动
 * @param  capture 实例指针
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef PhaseCapture_Stop(PhaseCapture_t *capture);

/**
 * @brief  获取主通道的周期（仅 tick 数，丢弃边沿计数）
 * @param  capture      实例指针
 * @param  period_ticks 输出周期（timer ticks）
 * @return 1 有新的有效数据，0 无数据
 */
uint8_t PhaseCapture_TakePeriod(PhaseCapture_t *capture,
                                uint32_t *period_ticks);

/**
 * @brief  获取主通道的完整测量值（周期与边沿数）
 * @param  capture        实例指针
 * @param  period_ticks   输出周期
 * @param  capture_cycles 输出累计边沿数
 * @return 1 有数据，0 无数据
 */
uint8_t PhaseCapture_TakeMeasurement(PhaseCapture_t *capture,
                                     uint32_t *period_ticks,
                                     uint32_t *capture_cycles);

/**
 * @brief  获取反馈通道的测量值（仅支持 DMA 运行状态）
 * @param  capture        实例指针
 * @param  period_ticks   输出周期
 * @param  capture_cycles 输出边沿数
 * @return 1 有数据，0 无数据
 */
uint8_t PhaseCapture_TakeFeedbackMeasurement(
    PhaseCapture_t *capture,
    uint32_t *period_ticks,
    uint32_t *capture_cycles);

/**
 * @brief Calculate 1x/2x phase error from paired CH3/CH4 DMA histories.
 *
 * Recent CH3 events are paired with the temporally nearest CH4 events and
 * circularly averaged. Both histories and the input period use the same
 * TIM2 counter, so the STM32 reference-clock tolerance cancels.
 */
uint8_t PhaseCapture_GetPhaseError(
    PhaseCapture_t *capture,
    uint32_t input_period_ticks,
    uint32_t input_capture_cycles,
    uint8_t frequency_ratio,
    float target_phase_deg,
    float *phase_error_deg);

/**
 * @brief  动态设置捕获边沿数（快速/DMA 或 慢速/中断）
 * @param  capture        实例指针
 * @param  capture_cycles 目标边沿数 (1/2/4/8)
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef PhaseCapture_SetEdgeCount(PhaseCapture_t *capture,
                                            uint32_t capture_cycles);

/**
 * @brief  获取当前有效捕获边沿数
 * @param  capture 实例指针
 * @return 边沿数，无效时返回0
 */
uint32_t PhaseCapture_GetEdgeCount(const PhaseCapture_t *capture);

/**
 * @brief  获取主通道 DMA 错误计数
 */
uint32_t PhaseCapture_GetDmaErrorCount(const PhaseCapture_t *capture);

/**
 * @brief  获取反馈通道 DMA 错误计数
 */
uint32_t PhaseCapture_GetFeedbackDmaErrorCount(
    const PhaseCapture_t *capture);

/**
 * @brief  主通道 DMA 中断处理函数（需在中断向量表中注册）
 */
void PhaseCapture_DmaIrqHandler(void);

/**
 * @brief  反馈通道 DMA 中断处理函数
 */
void PhaseCapture_FeedbackDmaIrqHandler(void);

/* -------------------------------------------------------------------------- */
/*                   简易全局 API（直接操作 g_phase_capture）                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief 使用默认配置初始化全局实例
 */
void phase_capture_init(void);

/**
 * @brief 启动全局实例
 */
HAL_StatusTypeDef phase_capture_start(void);

/**
 * @brief 读取全局实例的周期
 */
uint8_t phase_capture_take_period(uint32_t *period_ticks);

#endif /* PHASE_CAPTURE_H */
