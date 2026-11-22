/**
 * @file    phase_adc.h
 * @brief   双通道相位ADC采集模块（基于STM32F4）
 * @details 使用定时器触发ADC1和ADC2同步采样，DMA双缓冲模式，支持回调通知。
 *          提供启动/停止、数据取出、过载计数和采样率查询接口。
 *          硬件引脚和定时器参数通过宏定义配置，便于PCB改版时调整。
 *
 * @note    依赖STM32 HAL库，需预先初始化对应的ADC、DMA和定时器句柄。
 *          外部变量 hadc_phase_master, hadc_phase_slave, hdma_phase_adc
 *          由用户或HAL初始化代码提供。
 *
 * @version 1.0
 * @date    2026-07-28
 * @author  AI Assistant
 */

#ifndef PHASE_ADC_H
#define PHASE_ADC_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*============================== 硬件配置宏 ==============================*/
/**
 * @brief 用于触发ADC转换的定时器（高级定时器）
 * @note  该定时器产生PWM或比较输出事件，触发ADC注入或规则组转换。
 *        此处假设使用TIM8的更新事件或触发输出。
 */
#define PHASE_ADC_TIMER                 TIM8

/**
 * @brief 定时器中断号，用于处理DMA半满/全满中断或定时器溢出
 * @note  通常与DMA中断配合，本模块可能使用DMA中断而非定时器中断，
 *        但保留宏定义以备不时之需。
 */
#define PHASE_ADC_TIMER_IRQn            TIM8_UP_TIM13_IRQn

/**
 * @brief 通道1的ADC通道号（对应PA1引脚，ADC1）
 */
#define PHASE_ADC_CHANNEL_1             ADC_CHANNEL_1

/**
 * @brief 通道2的ADC通道号（对应PA2引脚，ADC2）
 */
#define PHASE_ADC_CHANNEL_2             ADC_CHANNEL_2

/**
 * @brief 两个ADC通道共用的GPIO端口
 */
#define PHASE_ADC_GPIO_PORT             GPIOA

/**
 * @brief 通道1对应的GPIO引脚
 */
#define PHASE_ADC_GPIO_PIN_1            GPIO_PIN_1

/**
 * @brief 通道2对应的GPIO引脚
 */
#define PHASE_ADC_GPIO_PIN_2            GPIO_PIN_2

/**
 * @brief 双ADC DMA总缓冲点数；两个半缓冲各提供1024组同步样本
 * @note  DMA存储的是ADC1/ADC2打包后的32位数据
 */
#define PHASE_ADC_SAMPLE_COUNT          2048U

/**
 * @brief 每个半缓冲区的样本数（即每次可取出的块长度）
 * @note  等于总采样点数的一半
 */
#define PHASE_ADC_BLOCK_COUNT           (PHASE_ADC_SAMPLE_COUNT / 2U)

/**
 * @brief 硬件支持的最大采样率（Hz）
 * @note  受ADC转换时间和定时器最大频率限制
 */
#define PHASE_ADC_MAX_SAMPLE_RATE_HZ    1200000U

/**
 * @brief 硬件支持的最小采样率（Hz）
 */
#define PHASE_ADC_MIN_SAMPLE_RATE_HZ       1000U

/*============================== 外部HAL句柄声明 ==============================*/
/**
 * @brief ADC1的句柄（主通道）
 * @note  需在外部定义并初始化，包括ADC配置、通道、DMA等
 */
extern ADC_HandleTypeDef hadc_phase_master;

/**
 * @brief ADC2的句柄（从通道）
 * @note  需在外部定义并初始化，同步触发模式
 */
extern ADC_HandleTypeDef hadc_phase_slave;

/**
 * @brief DMA句柄，用于ADC1和ADC2的数据传输（通常使用DMA2的流）
 * @note  配置为双缓冲循环模式，数据宽度为半字（16位）
 */
extern DMA_HandleTypeDef hdma_phase_adc;

/*============================== 回调函数类型 ==============================*/
/**
 * @brief 相位ADC数据就绪回调函数类型
 * @details 当DMA半缓冲或全缓冲传输完成时调用，通知用户取走数据。
 *          回调函数内不应执行耗时操作，建议仅设置标志或发送信号量。
 */
typedef void (*PhaseAdcReadyCallback_t)(void);

/*============================== 模块接口函数 ==============================*/

/**
 * @brief 初始化相位ADC模块
 * @details 配置GPIO、ADC、定时器、DMA等外设，使能中断但未启动转换。
 *          必须在使用其他功能前调用一次。
 * @retval HAL状态：HAL_OK 成功，否则失败
 */
HAL_StatusTypeDef PhaseAdc_Init(void);

/**
 * @brief 以指定采样率启动ADC连续采集
 * @param[in] sample_rate_hz 期望采样率（Hz），必须在 [PHASE_ADC_MIN_SAMPLE_RATE_HZ, PHASE_ADC_MAX_SAMPLE_RATE_HZ] 范围内
 * @details 计算定时器分频系数，启动定时器输出触发，并使能DMA双缓冲传输。
 *          采集过程中，每当DMA半缓冲或全缓冲填满，会触发回调（若已设置）。
 * @retval HAL状态：HAL_OK 启动成功，HAL_ERROR 参数无效或外设状态错误
 */
HAL_StatusTypeDef PhaseAdc_Start(uint32_t sample_rate_hz);

/**
 * @brief 停止ADC采集并关闭定时器
 * @details 停止定时器触发，禁用DMA传输，但保留配置以便后续再次启动。
 * @retval HAL状态：HAL_OK 停止成功，否则失败
 */
HAL_StatusTypeDef PhaseAdc_Stop(void);

/**
 * @brief 清空DMA内部缓冲区，重置缓冲区状态
 * @details 通常在停止后调用，以丢弃遗留数据，准备下一次启动。
 *          不影响外设配置。
 */
void PhaseAdc_Flush(void);

/**
 * @brief 设置数据就绪回调函数
 * @param[in] callback 函数指针，传NULL可取消回调
 * @note  回调在DMA传输完成中断上下文中执行，应尽量简短。
 */
void PhaseAdc_SetReadyCallback(PhaseAdcReadyCallback_t callback);

/**
 * @brief 取出一个完整的半缓冲数据块，并去除直流偏置
 * @param[out] channel1 通道1数据数组，长度至少为 length（单位：16位有符号整型）
 * @param[out] channel2 通道2数据数组，长度至少为 length
 * @param[in]  length   要取出的样本数，通常应等于 PHASE_ADC_BLOCK_COUNT
 * @details 该函数将当前就绪的半缓冲数据（原始ADC值）拷贝到用户缓冲区，
 *          同时减去各自通道的直流偏置（即整个缓冲区的平均值）。
 *          注意：调用前需确保有就绪的半缓冲，否则可能返回错误。
 * @retval HAL状态：HAL_OK 成功，HAL_BUSY 无就绪数据，HAL_ERROR 参数无效
 */
HAL_StatusTypeDef PhaseAdc_TakeBlock(int16_t *channel1,
                                     int16_t *channel2,
                                     uint16_t length);

/**
 * @brief 获取DMA或ADC的过载（溢出）计数
 * @details 当ADC转换速度过快或CPU未能及时取走数据时，可能发生溢出。
 *          该函数返回累积的溢出次数，可用于监测系统负载。
 * @return 过载计数值
 */
uint32_t PhaseAdc_GetOverrunCount(void);

/* Blocks discarded because DMA rewrote a half-buffer during the copy. */
uint32_t PhaseAdc_GetRejectedBlockCount(void);

/**
 * @brief 获取当前实际采样率（Hz）
 * @return 当前采样率，若未启动则返回0
 */
uint32_t PhaseAdc_GetSampleRateHz(void);

#endif /* PHASE_ADC_H */
