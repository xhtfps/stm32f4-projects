#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "stm32f4xx_hal.h"

/* ============================================================
 *  ADC波形采集缓冲区配置（DMA双缓冲/半满中断机制）
 *  采用乒乓缓冲设计：整缓冲256点，分成前后两半各128点
 *  DMA填前半段→触发半满中断取走前半块→DMA继续填后半段
 *  实现连续无断点的波形采集
 * ============================================================ */
#define BSP_ADC_WAVEFORM_BUFFER_LENGTH  256U   // DMA波形采集总缓冲区长度（采样点数）
#define BSP_ADC_WAVEFORM_BLOCK_LENGTH   (BSP_ADC_WAVEFORM_BUFFER_LENGTH / 2U)
                                               // 单次可读取的数据块长度=128点（半缓冲区）

/* ---------- HAL句柄全局声明（定义在.c文件中） ---------- */
extern ADC_HandleTypeDef hadc1;               // ADC1外设句柄（用于波形采集、高速采样）
extern ADC_HandleTypeDef hadc2;               // ADC2外设句柄（用于普通单点电压采集）
extern DMA_HandleTypeDef hdma_adc2_waveform;  // ADC2对应的DMA句柄（搬运波形数据到内存）

/* ============================================================
 *  新版BSP标准API（推荐新代码使用）
 * ============================================================ */

/**
 * @brief  初始化ADC1（配置为DMA连续采样模式，用于波形采集）
 * @retval HAL状态：HAL_OK=初始化成功
 */
HAL_StatusTypeDef BSP_ADC1_Init(void);

/**
 * @brief  初始化ADC2（配置为单次转换模式，用于普通电压读取）
 * @retval HAL状态：HAL_OK=初始化成功
 */
HAL_StatusTypeDef BSP_ADC2_Init(void);

/**
 * @brief  单次读取指定ADC通道的原始采样值
 * @param  hadc    ADC外设句柄（&hadc1 或 &hadc2）
 * @param  channel 目标通道号（如 ADC_CHANNEL_0）
 * @param  value   输出参数：返回12位原始ADC码值（0~4095）
 * @retval HAL状态：HAL_OK=读取成功
 */
HAL_StatusTypeDef BSP_ADC_Read(ADC_HandleTypeDef *hadc, uint32_t channel,
                               uint32_t *value);

/**
 * @brief  多次采样取平均值，降低噪声提高精度
 * @param  hadc    ADC外设句柄
 * @param  channel 目标通道号
 * @param  samples 采样次数（采样越多越稳定，耗时越长）
 * @param  value   输出参数：多次采样平均后的原始码值
 * @retval HAL状态：HAL_OK=读取成功
 */
HAL_StatusTypeDef BSP_ADC_ReadAverage(ADC_HandleTypeDef *hadc, uint32_t channel,
                                      uint16_t samples, uint32_t *value);

/**
 * @brief  启动ADC2 DMA连续波形采集（PA2，开启定时器触发+DMA循环搬运）
 * @param  sample_rate_hz 采样率，单位：Hz（每秒采样点数）
 * @retval HAL状态：HAL_OK=启动成功
 * @note   采集数据自动写入内部环形缓冲区，通过TakeBlock逐块取出
 */
HAL_StatusTypeDef BSP_ADC_WaveformStart(uint32_t sample_rate_hz);

/**
 * @brief  停止ADC波形采集（关闭DMA、停止ADC转换）
 * @retval HAL状态：HAL_OK=停止成功
 */
HAL_StatusTypeDef BSP_ADC_WaveformStop(void);

/**
 * @brief  从波形缓冲区取出一块就绪的采样数据（非阻塞）
 * @param  block  输出数组：拷贝出的采样数据原始码（12位）
 * @param  length 期望读取的数据长度（一般传BSP_ADC_WAVEFORM_BLOCK_LENGTH）
 * @param  offset 输出参数：该块数据在总缓冲区中的起始下标（用于DAC同步回放对齐）
 * @retval HAL状态：HAL_OK=有新数据已取出，HAL_BUSY=暂无新数据就绪
 * @note   基于DMA半满/全满中断实现乒乓缓冲，保证数据不丢失不重叠
 */
HAL_StatusTypeDef BSP_ADC_WaveformTakeBlock(uint16_t *block,
                                             uint16_t length,
                                             uint16_t *offset);

/** Return the number of DMA half-buffers overwritten before software consumed them. */
uint32_t BSP_ADC_WaveformGetOverrunCount(void);

/**
 * Application hook called from the ADC DMA half/full-complete ISR.
 * The default implementation is weak and does nothing.
 */
void BSP_ADC_WaveformBlockReadyCallback(void);

/* ============================================================
 *  旧版兼容接口（保留给已有应用代码，新代码请勿使用）
 * ============================================================ */

/** @deprecated 旧版ADC1初始化，推荐改用 BSP_ADC1_Init() */
void ADC1_Init(void);

/** @deprecated 旧版ADC2初始化，推荐改用 BSP_ADC2_Init() */
void ADC2_Init(void);

/** @deprecated 旧版单次采样，推荐改用 BSP_ADC_Read() */
uint16_t ADC_Result_Get(uint8_t channel, uint8_t sampling_time);

/** @deprecated 旧版多次平均采样，推荐改用 BSP_ADC_ReadAverage() */
uint16_t ADC_Average_Get(uint8_t channel, uint8_t times);

/**
 * @deprecated 旧版波形重采样接口
 * @brief  将原始采样波形通过插值重采样到目标长度
 */
uint16_t ADC_Waveform_Get(const uint16_t *source_buffer, uint16_t source_length,
                          double *result_buffer, uint16_t result_length);

#endif /* BSP_ADC_H */
