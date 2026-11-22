#ifndef DRIVE_COMMUNICATION_H
#define DRIVE_COMMUNICATION_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* ============================================================
 *  F4主控 <-> F1从机 AD9959 串口通信协议定义
 *  架构：STM32F4 通过UART发送指令给STM32F1，由F1直接驱动AD9959芯片
 * ============================================================ */

/* ---------- 串口帧协议固定字段 ---------- */
#define AD9959_FRAME_HEAD_0       0xAAU    // 帧头字节1（固定0xAA，用于帧同步识别）
#define AD9959_FRAME_HEAD_1       0xA5U    // 帧头字节2（固定0xA5，双字节帧头防止误识别）
#define AD9959_FRAME_TAIL         0x55U    // 帧尾字节（固定0x55，标记一帧数据结束）
#define AD9959_FRAME_LENGTH       38U      // 单帧数据总长度（字节）

/* ---------- 硬件与通信参数 ---------- */
#define AD9959_MAX_OUTPUT_MV      500.0f   // AD9959单通道最大输出幅值（单位：mV，硬件上限）
#define AD9959_CHANNEL_COUNT      4U
#define AD9959_UPDATE_BATCH_FLAG  1U
#define AD9959_UPDATE_SINGLE_FLAG 2U
#define AD9959_UART_BAUDRATE      115200U  // F4与F1之间串口波特率（115200 bps）
#define AD9959_FRAME_GAP_MS       100U     // 两帧数据之间最小间隔（ms，防止F1处理不过来丢包）
#define AD9959_UART_TIMEOUT_MS    100U     // UART 单帧超时保护（ms）

/* ---------- DDS工作模式位掩码（F1固件识别） ---------- */
/* F1普通输出解析器要求 mode 严格等于 NORMAL；相干更新由批量标志完成。 */
#define NORMAL                    0x0001U  // 普通单频输出模式（固定频率/幅值/相位）
#define SWEEP                     0x0002U  // 扫频模式（频率在起止区间内步进变化）
#define MODULATION                0x0004U  // 调制模式（支持ASK/FSK/PSK数字调制）
#define INPHASE                   0x0008U  // 多通道同相位同步输出模式

/* ---------- 旧版代码兼容别名（不推荐新代码使用） ---------- */
#define RX_LENGTH                 AD9959_FRAME_LENGTH  // 接收缓冲区长度（兼容旧命名）
#define TX_LENGTH                 AD9959_FRAME_LENGTH  // 发送缓冲区长度（兼容旧命名）

/* ============================================================
 *  通信数据结构体：管理串口收发原始字节缓存
 * ============================================================ */
typedef struct
{
    uint8_t rxBuffer[AD9959_FRAME_LENGTH];  // 串口接收缓冲区，存放F1回传的原始字节
    uint8_t txBuffer[AD9959_FRAME_LENGTH];  // 串口发送缓冲区，存放待发送给F1的指令帧
    uint8_t flag;                           // 通信完成标志位（1=一帧收发完成，待处理）
} communicationDataStruct;

/* ============================================================
 *  DDS通道参数结构体：描述单个AD9959通道的完整配置
 *  4通道AD9959对应4份该结构体实例
 * ============================================================ */
typedef struct
{
    uint16_t mode;              // 工作模式（位掩码，组合NORMAL/SWEEP/MODULATION/INPHASE）
    uint8_t  output;            // 输出使能，1=开启射频输出，0=关闭输出

    /* 普通模式基础参数 */
    float    range;             // 输出幅值（单位：mV，F1端已做校准映射）
    uint32_t phase;             // 输出相位（单位：度，0~360°）
    uint32_t fre;               // 输出中心频率（单位：Hz，普通模式下的固定频率）

    /* 扫频模式专用参数 */
    uint32_t fre_start;         // 扫频起始频率（Hz）
    uint32_t fre_stop;          // 扫频终止频率（Hz）
    uint32_t step;              // 扫频步进值（每一步频率增量，单位Hz）
    uint32_t step_time;         // 每步停留时间（ms，控制扫频速度）

    /* 调制模式专用参数（ASK/FSK/PSK） */
    uint8_t  modulation_mode;   // 调制类型选择：幅移键控/频移键控/相移键控
    uint8_t  modulation_levels; // 调制电平数（如2FSK=2级、16FSK=16级，最大16）
    float    ask[16];           // ASK各档位幅值表（mV，共16级）
    float    fsk[16];           // FSK各档位频率表（Hz，共16级）
    float    psk[16];           // PSK各档位相位表（度，共16级）

    uint8_t  flag;              // 数据更新标志，1=参数已修改，需要同步下发给F1
} DDSDataStruct;

/* ---------- 全局变量声明（定义在.c文件中） ---------- */
extern DDSDataStruct dds[4];                  // AD9959四个通道的参数数组（通道0~3）
extern communicationDataStruct communicationData; // 串口通信收发管理结构体

/* ============================================================
 *  底层兼容API（保留自原始F4驱动，上层一般不直接调用）
 * ============================================================ */

/**
 * @brief  向指定通道下发完整DDS参数帧（单通道底层发送接口）
 * @param  data    该通道完整配置参数结构体
 * @param  channel 目标通道号（0~3，对应AD9959的4个通道）
 * @retval HAL状态：HAL_OK=发送成功，其他=串口发送失败
 */
HAL_StatusTypeDef sendData(DDSDataStruct data, uint8_t channel);

/**
 * @brief  初始化与F1通信的UART外设
 * @param  baudrate 串口波特率，一般传 AD9959_UART_BAUDRATE
 */
void Init_Uart(uint32_t baudrate);

/**
 * @brief  DDS参数结构体全局初始化（上电调用一次，给所有通道赋默认值）
 */
void DDSDataInit(void);

/* ============================================================
 *  应用层高级API（业务代码直接调用，推荐使用）
 *  一次调用自动同步配置全部4个通道
 * ============================================================ */

/**
 * @brief  设置AD9959输出参数（四通道同步配置）
 * @param  frequency_hz 输出频率，单位：Hz
 * @param  amplitude_mv 输出幅值，单位：mV（最大不超过AD9959_MAX_OUTPUT_MV）
 * @param  phase_deg    输出相位，单位：度（0~360）
 * @param  output       输出使能，1=开启输出，0=关闭
 * @retval HAL状态：HAL_OK=配置下发成功
 */
HAL_StatusTypeDef AD9959_SetOutput(uint32_t frequency_hz,
                                   float amplitude_mv,
                                   uint32_t phase_deg,
                                   uint8_t output);

/* Update only one DDS channel. Used by the single-path sweep source. */
HAL_StatusTypeDef AD9959_SetChannelOutput(uint8_t channel,
                                          uint32_t frequency_hz,
                                          float amplitude_mv,
                                          uint32_t phase_deg,
                                          uint8_t output);

HAL_StatusTypeDef AD9959_SetAllChannelsSame(uint32_t frequency_hz,
                                            float amplitude_mv,
                                            uint32_t phase_deg,
                                            uint8_t output);

/*
 * Calibration source mapping:
 *   AD9959 CH1 -> instrument CH1
 *   AD9959 CH0 -> instrument CH2
 * CH0/CH1 are staged with the batch/in-phase flags. CH2/CH3 are disabled
 * by the same batch; the measurement state machine may then enable CH2 as
 * the high-frequency mixer LO.
 */
HAL_StatusTypeDef AD9959_SetCalibrationPair(
    uint32_t frequency_hz,
    float amplitude_mv,
    uint32_t channel1_phase_deg,
    uint32_t channel0_phase_deg);

/**
 * @brief  停止全部通道输出（关闭AD9959射频输出，进入待机）
 * @retval HAL状态：HAL_OK=停止指令下发成功
 */
HAL_StatusTypeDef AD9959_StopOutput(void);

#endif /* DRIVE_COMMUNICATION_H */
