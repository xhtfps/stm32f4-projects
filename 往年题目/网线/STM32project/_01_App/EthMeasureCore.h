#ifndef ETH_MEASURE_CORE_H
#define ETH_MEASURE_CORE_H

#include "User_header.h"

typedef enum
{
    ETH_WINDOW_RECTANGULAR = 0U,
    ETH_WINDOW_HANN = 1U,
    ETH_WINDOW_KAISER = 2U
} EthernetWindowType;

typedef enum
{
    ETH_PORT_CAL_OPEN = 0U,
    ETH_PORT_CAL_SHORT = 1U,
    ETH_PORT_CAL_LOAD = 2U
} EthernetPortCalStandard;

/**
 * @brief 网线TDR时域反射测量全局配置结构体
 * 存储扫频、IFFT、线缆NVP、校准相关固定参数
 */
typedef struct
{
    uint32_t start_freq_hz;     // 扫频起始频率，单位Hz
    uint32_t stop_freq_hz;      // 扫频终止频率，单位Hz
    uint32_t step_hz;           // DDS频率步进，单位Hz
    uint16_t settle_ms;          // 切换频点后射频稳定等待时间，单位ms
    uint16_t nvp_x1000;         // 线缆传播速率NVP，实际值 = nvp_x1000 / 1000 倍光速
    uint16_t ifft_points;        // IFFT 点数，支持 256/512/1024
    uint16_t calib_length_cm;    // 校准时标准线缆长度，单位cm
    uint16_t fine_search_span_cm;// 相位细搜相对 IFFT 粗定位的半宽，单位cm
    uint8_t frames_per_point;    // 每个频点的相干 ADC 帧数
    uint8_t window_type;         // EthernetWindowType
    uint8_t kaiser_beta_x10;     // Kaiser beta 乘以 10
} EthernetMeasureConfig;

/**
 * @brief TDR测量实时运行状态结构体
 * 保存硬件就绪标志、单次测量结果、电压采样值、日志提示信息
 */
typedef struct
{
    float last_length_m;            // 上一次测量得到的线缆长度，单位m
    float last_resolution_m;        // 当前扫频配置下的理论测距分辨率，单位m
    float last_max_range_m;         // 当前步进下可测量最大线缆长度，单位m
    float last_peak_metric;         // IFFT时域反射峰值幅值，用于判断反射强弱
    float last_coarse_length_m;     // IFFT 粗定位长度，单位m
    float last_phase_metric;        // 复数相位匹配细搜指标
    float last_peak_snr;            // IFFT 峰值与噪声底比值
    float detector_voltage_v;       // 主检波通道ADC采集直流电压，单位V
    float detector_aux_voltage_v;   // 辅助检波通道ADC采集直流电压，单位V
    uint32_t dds_freq_hz;           // DDS当前正在输出的射频频率，单位Hz
    uint32_t sweep_count;           // 累计完成扫频测量总次数
    uint16_t last_sweep_points;      // 上一轮扫频实际采集频点总数
    uint16_t last_peak_bin;          // 粗搜索得到的反射峰值IFFT索引
    uint16_t last_valid_points;      // 上一轮扫频有效频点数
    uint16_t last_rejected_frames;   // 上一轮扫频拒绝的 ADC 帧数
    uint16_t last_clipped_frames;    // 上一轮扫频疑似饱和的 ADC 帧数
    uint8_t dds_ready;              // DDS(AD9959)芯片就绪标志：1就绪 0未就绪
    uint8_t adc_ready;              // 检波ADC采集模块就绪标志：1就绪 0未就绪
    uint8_t adc_capture_ready;      // 片内ADC一次DMA帧采集是否成功
    uint8_t calib_ready;            // NVP线缆校准完成标志
    uint8_t baseline_ready;         // 100Ω逐频复数基线已采集
    uint8_t port_cal_valid;         // Open/Short/Load 一端口校准有效
    char last_message[64];          // 运行状态/故障/进度提示字符串缓存
} EthernetMeasureRuntime;

// 全局外部变量声明
extern EthernetMeasureConfig g_eth_cfg;     // 测量全局配置参数实例
extern EthernetMeasureRuntime g_eth_runtime;// 测量实时运行状态实例

/**
 * @brief 应用层初始化接口，上电调用一次
 * 完成屏幕清空、状态清零、DDS+ADC信号链路初始化
 */
void EthApp_Init(void);

/**
 * @brief DDS输出复位至配置起始频率
 * 测量结束/初始化后刷新射频输出频点
 */
void EthRefreshDdsPreview(void);

/**
 * @brief 配置参数规范化函数
 * 对非法、越界参数做限幅修正，保证所有参数在有效工作区间
 */
void EthNormalizeConfig(void);

/**
 * @brief 执行一次完整TDR扫频测距流程
 * @retval 1：测量成功并解算出线缆长度；0：测量失败(点数不足/无反射峰)
 */
uint8_t EthMeasureOnce(void);

/**
 * @brief 使用标准已知长度线缆校准NVP传播速率
 * @retval 1：校准成功更新NVP参数；0：校准失败(无有效测量长度)
 */
uint8_t EthApplyCalibration(void);

/** 在当前 RJ45 测试面采集 Open、Short 或 100Ω Load 标准件。 */
uint8_t EthCapturePortCalibration(EthernetPortCalStandard standard);

/** 清除 Open/Short/Load 与逐频点基线校准。 */
void EthClearPortCalibration(void);

/**
 * @brief 片内双ADC同步DMA采集自检。
 * @retval 1=成功取得一帧参考/响应采样；0=DMA采集超时。
 */
uint8_t EthProbeAdcOnce(void);

#endif
