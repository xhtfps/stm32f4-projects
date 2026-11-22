/**
 * @file    Drive_Communication.c
 * @brief   AD9959 驱动通信层
 *
 * 负责将上层设置的频率、幅值、相位等参数打包为自定义协议帧，
 * 通过串口（USART1）发送给协处理器（STM32F1）以控制 AD9959 DDS 模块。
 * 包含 CRC16 校验、帧格式组装、串口发送以及全局 DDS 状态维护。
 */

#include "Drive_Communication.h"

#include <string.h>

#include "./SYSTEM/usart/usart.h"

/* ================================================================
 * 全局数据结构
 * ================================================================ */

communicationDataStruct communicationData;  /**< 通信帧缓冲区 */
DDSDataStruct dds[4];                       /**< 四个通道的当前 DDS 参数 */

/* ================================================================
 * CRC16 校验 (MODBUS 协议多项式)
 * ================================================================ */

/**
 * @brief  计算 CRC16 (MODBUS) 校验值
 * @param  data   数据缓冲区指针
 * @param  length 数据字节数
 * @return 16 位 CRC 校验值
 */
static uint16_t AD9959_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

/* ================================================================
 * 大端字节序写入
 * ================================================================ */

/**
 * @brief  将 32 位无符号整数以大端格式写入缓冲区指定偏移处
 * @param  buffer 目标缓冲区
 * @param  offset 写入起始偏移
 * @param  value  待写入的值
 */
static void AD9959_PutU32BE(uint8_t *buffer, uint8_t offset, uint32_t value)
{
    buffer[offset]     = (uint8_t)(value >> 24U);
    buffer[offset + 1U] = (uint8_t)(value >> 16U);
    buffer[offset + 2U] = (uint8_t)(value >> 8U);
    buffer[offset + 3U] = (uint8_t)value;
}

/* ================================================================
 * 串口发送封装
 * ================================================================ */

/**
 * @brief  通过 UART1 发送一帧数据（阻塞方式）
 * @param  buffer 数据缓冲区指针
 * @param  length 发送字节数
 * @return HAL_OK 或 HAL_ERROR（如串口未初始化）
 */
static HAL_StatusTypeDef AD9959_Transmit(const uint8_t *buffer, uint16_t length)
{
    if ((buffer == NULL) || (length == 0U) ||
        (g_uart1_handle.Instance != USART1))
    {
        return HAL_ERROR;
    }

    {
        HAL_StatusTypeDef status = HAL_UART_Transmit(
            &g_uart1_handle, (uint8_t *)buffer, length,
            AD9959_UART_TIMEOUT_MS);

        /* HAL_UART_Transmit 超时后可能保留 BUSY 状态，必须恢复后才能重试。 */
        if (status != HAL_OK)
        {
            (void)HAL_UART_AbortTransmit(&g_uart1_handle);
        }
        return status;
    }
}

/* ================================================================
 * 帧组装与发送
 * ================================================================ */

/**
 * @brief  将 DDS 参数按自定义协议打包并发送给协处理器
 * @param  data    DDS 参数结构体
 * @param  channel 通道号 (0-3)
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef sendData(DDSDataStruct data, uint8_t channel)
{
    uint32_t range_scaled;   /* 幅值×1000，用于整数传输 */
    uint32_t phase_scaled;   /* 相位 ×100，用于整数传输 */
    uint16_t crc;

    if (channel >= 4U)
    {
        return HAL_ERROR;
    }

    (void)memset(communicationData.txBuffer, 0,
                 sizeof(communicationData.txBuffer));

    /* 帧头 (2 字节) */
    communicationData.txBuffer[0] = AD9959_FRAME_HEAD_0;
    communicationData.txBuffer[1] = AD9959_FRAME_HEAD_1;

    /* 工作模式 (2 字节，大端) */
    communicationData.txBuffer[2] = (uint8_t)(data.mode >> 8U);
    communicationData.txBuffer[3] = (uint8_t)data.mode;

    /* 通道号 */
    communicationData.txBuffer[4] = channel;

    /* 输出使能标志 */
    communicationData.txBuffer[5] = data.output;

    /* 频率 (4 字节，大端) */
    AD9959_PutU32BE(communicationData.txBuffer, 6U, data.fre);

    /* 幅值 (mV)，缩放为 ×1000 后传输 */
    if (data.range <= 0.0f)
    {
        range_scaled = 0U;
    }
    else if (data.range >= 4294967.0f)
    {
        range_scaled = UINT32_MAX;
    }
    else
    {
        range_scaled = (uint32_t)(data.range * 1000.0f + 0.5f);
    }
    AD9959_PutU32BE(communicationData.txBuffer, 10U, range_scaled);

    /* 相位 (°)，缩放为 ×100 后传输 */
    phase_scaled = data.phase * 100U;
    AD9959_PutU32BE(communicationData.txBuffer, 14U, phase_scaled);

    /* 扫频步进、步进时间、起始频率、终止频率（均未使用，占位） */
    AD9959_PutU32BE(communicationData.txBuffer, 18U, data.step);
    AD9959_PutU32BE(communicationData.txBuffer, 22U, data.step_time);
    AD9959_PutU32BE(communicationData.txBuffer, 26U, data.fre_start);
    AD9959_PutU32BE(communicationData.txBuffer, 30U, data.fre_stop);

    /* 标志字节（如扫频使能） */
    communicationData.txBuffer[34] = data.flag;

    /* CRC16 校验（覆盖除帧尾外的全部数据） */
    crc = AD9959_Crc16(communicationData.txBuffer, AD9959_FRAME_LENGTH - 3U);
    communicationData.txBuffer[35] = (uint8_t)(crc >> 8U);
    communicationData.txBuffer[36] = (uint8_t)crc;

    /* 帧尾 */
    communicationData.txBuffer[37] = AD9959_FRAME_TAIL;

    /* 通过串口发送整帧 */
    return AD9959_Transmit(communicationData.txBuffer,
                           AD9959_FRAME_LENGTH);
}

/* ================================================================
 * 串口初始化（封装用户串口初始化）
 * ================================================================ */

/**
 * @brief  初始化通信串口（USART1）
 * @param  baudrate 波特率
 */
void Init_Uart(uint32_t baudrate)
{
    usart_init(baudrate);
}

/* ================================================================
 * DDS 数据结构初始化
 * ================================================================ */

/**
 * @brief  初始化全局 DDS 参数数组
 *
 * 将所有通道设为正常模式，频率默认 1kHz，幅值 0，输出关闭。
 */
void DDSDataInit(void)
{
    uint8_t channel;

    (void)memset(dds, 0, sizeof(dds));
    for (channel = 0U; channel < 4U; ++channel)
    {
        dds[channel].mode = NORMAL;    /* 正常连续输出模式 */
        dds[channel].fre = 1000U;      /* 默认频率 1kHz */
        dds[channel].range = 0.0f;     /* 幅值 0 */
        dds[channel].output = 0U;      /* 关闭输出 */
        dds[channel].flag = 0U;        /* 无特殊标志 */
    }
}

/* ================================================================
 * 统一输出设置（四个通道同步）
 * ================================================================ */

/**
 * @brief  设置所有四个通道的输出参数并发送帧
 *
 * 将频率、幅值（mV）、相位（°）及输出使能标志写入全局 DDS 结构体，
 * 然后依次向四个通道发送通信帧。由于协处理器需要时间解析，各通道间
 * 插入少量延时（最后一个通道除外）。
 *
 * @param  frequency_hz 频率 (Hz) [1..500000000]
 * @param  amplitude_mv 幅值 (mV) [0..AD9959_MAX_OUTPUT_MV]
 * @param  phase_deg    相位 (°) [0..360]
 * @param  output       输出使能 (1:开启 0:关闭)
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef AD9959_SetChannelOutput(uint8_t channel,
                                          uint32_t frequency_hz,
                                          float amplitude_mv,
                                          uint32_t phase_deg,
                                          uint8_t output)
{
    if (channel >= AD9959_CHANNEL_COUNT)
    {
        return HAL_ERROR;
    }

    if (frequency_hz == 0U)
    {
        frequency_hz = 1U;
    }
    if (frequency_hz > 500000000U)
    {
        frequency_hz = 500000000U;
    }
    if (amplitude_mv < 0.0f)
    {
        amplitude_mv = 0.0f;
    }
    if (amplitude_mv > AD9959_MAX_OUTPUT_MV)
    {
        amplitude_mv = AD9959_MAX_OUTPUT_MV;
    }
    if (phase_deg > 360U)
    {
        phase_deg = 360U;
    }

    dds[channel].mode = NORMAL;
    dds[channel].output = output;
    dds[channel].range = amplitude_mv;
    dds[channel].phase = phase_deg;
    dds[channel].fre = frequency_hz;
    dds[channel].flag = AD9959_UPDATE_SINGLE_FLAG;

    return sendData(dds[channel], channel);
}

HAL_StatusTypeDef AD9959_SetAllChannelsSame(uint32_t frequency_hz,
                                            float amplitude_mv,
                                            uint32_t phase_deg,
                                            uint8_t output)
{
    uint8_t channel;
    HAL_StatusTypeDef status = HAL_OK;

    /* 参数范围限制 */
    if (frequency_hz == 0U)
    {
        frequency_hz = 1U;
    }
    if (frequency_hz > 500000000U)
    {
        frequency_hz = 500000000U;
    }
    if (amplitude_mv < 0.0f)
    {
        amplitude_mv = 0.0f;
    }
    if (amplitude_mv > AD9959_MAX_OUTPUT_MV)
    {
        amplitude_mv = AD9959_MAX_OUTPUT_MV;
    }
    if (phase_deg > 360U)
    {
        phase_deg = 360U;
    }

    /* 向四个通道写入相同参数并发送 */
    for (channel = 0U; channel < AD9959_CHANNEL_COUNT; ++channel)
    {
        dds[channel].mode = NORMAL;
        dds[channel].output = output;
        dds[channel].range = amplitude_mv;
        dds[channel].phase = phase_deg;
        dds[channel].fre = frequency_hz;
        dds[channel].flag = AD9959_UPDATE_BATCH_FLAG;

        status = sendData(dds[channel], channel);
        if (status != HAL_OK)
        {
            return status;
        }

        /* 协处理器（F1）在串口中断中解析帧，需要间隔时间防止帧冲突 */
        if (channel + 1U < AD9959_CHANNEL_COUNT)
        {
            HAL_Delay(AD9959_FRAME_GAP_MS);
        }
    }
    return status;
}

HAL_StatusTypeDef AD9959_SetOutput(uint32_t frequency_hz,
                                   float amplitude_mv,
                                   uint32_t phase_deg,
                                   uint8_t output)
{
    return AD9959_SetAllChannelsSame(frequency_hz, amplitude_mv,
                                      phase_deg, output);
}

HAL_StatusTypeDef AD9959_SetCalibrationPair(
    uint32_t frequency_hz,
    float amplitude_mv,
    uint32_t channel1_phase_deg,
    uint32_t channel0_phase_deg)
{
    uint8_t channel;
    HAL_StatusTypeDef status;

    if (frequency_hz == 0U)
    {
        frequency_hz = 1U;
    }
    if (frequency_hz > 500000000U)
    {
        frequency_hz = 500000000U;
    }
    if (amplitude_mv < 0.0f)
    {
        amplitude_mv = 0.0f;
    }
    if (amplitude_mv > AD9959_MAX_OUTPUT_MV)
    {
        amplitude_mv = AD9959_MAX_OUTPUT_MV;
    }
    if (channel1_phase_deg > 360U)
    {
        channel1_phase_deg %= 360U;
    }
    if (channel0_phase_deg > 360U)
    {
        channel0_phase_deg %= 360U;
    }

    for (channel = 0U; channel < AD9959_CHANNEL_COUNT; ++channel)
    {
        /*
         * The deployed F1 parser accepts ordinary output frames only when
         * command == NORMAL.  Sending NORMAL | INPHASE (0x0009) makes it
         * discard the CH0/CH1 frames, so the subsequent four-channel batch
         * never reaches IO_Update().  Phase-coherent application is already
         * provided by the batch flag and the single IO_Update on the F1.
         */
        dds[channel].mode = NORMAL;
        dds[channel].output =
            ((channel == 0U) || (channel == 1U)) ? 1U : 0U;
        dds[channel].range =
            ((channel == 0U) || (channel == 1U)) ?
            amplitude_mv : 0.0f;
        dds[channel].phase =
            (channel == 0U) ? channel0_phase_deg :
            ((channel == 1U) ? channel1_phase_deg : 0U);
        dds[channel].fre = frequency_hz;
        dds[channel].flag = AD9959_UPDATE_BATCH_FLAG;

        status = sendData(dds[channel], channel);
        if (status != HAL_OK)
        {
            return status;
        }
        if (channel + 1U < AD9959_CHANNEL_COUNT)
        {
            HAL_Delay(AD9959_FRAME_GAP_MS);
        }
    }
    return HAL_OK;
}

/* ================================================================
 * 关闭所有通道输出
 * ================================================================ */

/**
 * @brief  停止 AD9959 输出（幅值置零，输出使能清零）
 * @return HAL_OK 或 HAL_ERROR
 */
HAL_StatusTypeDef AD9959_StopOutput(void)
{
    uint32_t frequency_hz = dds[0].fre;

    if (frequency_hz == 0U)
    {
        frequency_hz = 1000U;
    }
    return AD9959_SetOutput(frequency_hz, 0.0f, 0U, 0U);
}
