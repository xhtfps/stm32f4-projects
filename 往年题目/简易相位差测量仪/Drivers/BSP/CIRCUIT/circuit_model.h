#ifndef CIRCUIT_MODEL_H
#define CIRCUIT_MODEL_H

#include <stdint.h>

/* ============================================================
 *  电路模型辨识与数字滤波模块
 *  功能1：扫频采集数据 → 自动识别滤波器类型 → 提取特征参数
 *  功能2：基于识别结果生成等效双二阶（Biquad）IIR数字滤波器
 *  支持类型：低通、高通、带通、带阻（陷波）四种经典二阶系统
 * ============================================================ */

/* 必须覆盖当前 256 点扫频，并预留少量余量，禁止静默丢点。 */
#define CIRCUIT_MODEL_MAX_POINTS 272U // 扫频数据点最大数量（模型缓冲区容量）

/**
 * @brief  电路滤波器类型枚举
 */
typedef enum
{
    CIRCUIT_MODEL_UNKNOWN = 0U,  // 未知类型（无法识别或数据不足）
    CIRCUIT_MODEL_LOWPASS,       // 低通滤波器
    CIRCUIT_MODEL_HIGHPASS,      // 高通滤波器
    CIRCUIT_MODEL_BANDPASS,      // 带通滤波器
    CIRCUIT_MODEL_BANDSTOP       // 带阻/陷波滤波器
} CircuitModelType_t;

/**
 * @brief  单个扫频数据点：一个频率下的完整测量结果
 */
typedef struct
{
    uint32_t frequency_hz;  // 测量频率点（Hz）
    float    input_rms;     // 输入端交流有效值（V）
    float    output_rms;    // 输出端交流有效值（V）
    float    gain;          // 线性电压增益 = 输出/输入（倍数）
    float    gain_db;       // 对数电压增益（dB）
} CircuitModelPoint_t;

/**
 * @brief  电路模型结构体：扫频数据 + 分析识别结果
 */
typedef struct
{
    CircuitModelPoint_t points[CIRCUIT_MODEL_MAX_POINTS]; // 扫频数据点数组
    uint16_t            count;                            // 已采集数据点数量

    /* ---- 分析结果字段（CircuitModel_Analyze后填充） ---- */
    CircuitModelType_t  type;        // 识别出的滤波器类型
    float               f1_hz;       // 下限截止频率（Hz），低通/高通为唯一截止频率
    float               f2_hz;       // 上限截止频率（Hz），仅带通/带阻有效
    float               center_hz;   // 中心频率（Hz，几何平均=√(f1×f2)），仅带通/带阻有效
    float               q;           // 品质因数Q = 中心频率/带宽，值越大谐振峰越尖锐
    float               peak_gain_db;// 通带峰值增益（dB）
    /*
     * Least-squares second-order prototype fit. natural_hz is the
     * pole/natural frequency, which differs from the -3 dB frequency when
     * Q is not 1/sqrt(2). model_gain_db is the fitted pass-band gain.
     */
    float               natural_hz;
    float               model_gain_db;
    float               fit_rms_error;
    uint8_t             fit_valid;
} CircuitModel_t;

/**
 * @brief  双二阶（Biquad）IIR数字滤波器结构体（直接I型）
 * @note   差分方程：y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
 */
typedef struct
{
    /* 滤波系数 */
    float b0;   // 分子系数（当前输入加权）
    float b1;   // 分子系数（前一拍输入加权）
    float b2;   // 分子系数（前两拍输入加权）
    float a1;   // 分母系数（前一拍输出加权，负反馈）
    float a2;   // 分母系数（前两拍输出加权，负反馈）
    float gain; // 整体增益倍数（输出端统一缩放）

    /* 状态变量（延迟线，保存历史采样值） */
    float x1;   // 输入延迟1拍：x[n-1]
    float x2;   // 输入延迟2拍：x[n-2]
    float y1;   // 输出延迟1拍：y[n-1]
    float y2;   // 输出延迟2拍：y[n-2]
} CircuitBiquad_t;

/* ============================================================
 *  电路模型分析API
 * ============================================================ */

/**
 * @brief  重置电路模型，清空所有数据恢复初始状态
 */
void CircuitModel_Reset(CircuitModel_t *model);

/**
 * @brief  向模型追加一个扫频测量点
 * @param  frequency_hz 测量频率
 * @param  input_rms    输入有效值
 * @param  output_rms   输出有效值
 * @retval 1=成功，0=缓冲区满或参数非法
 */
uint8_t CircuitModel_Append(CircuitModel_t *model,
                             uint32_t frequency_hz,
                             float input_rms,
                             float output_rms);

/**
 * @brief  分析全部扫频数据，自动识别滤波器类型并计算特征参数
 * @retval 1=识别成功，0=数据不足或无法分类
 */
uint8_t CircuitModel_Analyze(CircuitModel_t *model);

/**
 * @brief  获取滤波器类型名称字符串（用于UI显示）
 */
const char *CircuitModel_TypeName(CircuitModelType_t type);

/* ============================================================
 *  双二阶数字滤波器API（等效仿真用）
 * ============================================================ */

/**
 * @brief  初始化双二阶IIR数字滤波器（RBJ标准公式）
 * @param  filter              滤波器结构体
 * @param  type                滤波器类型
 * @param  sample_rate_hz      数字系统采样率（Hz）
 * @param  center_frequency_hz 截止/中心频率（Hz）
 * @param  q                   品质因数Q
 * @retval 1=成功，0=参数非法
 */
uint8_t CircuitModel_BiquadInit(CircuitBiquad_t *filter,
                                 CircuitModelType_t type,
                                 float sample_rate_hz,
                                 float center_frequency_hz,
                                 float q);

/**
 * @brief  设置滤波器整体增益倍数
 */
void CircuitModel_BiquadSetGain(CircuitBiquad_t *filter, float gain);

/**
 * @brief  单样本实时滤波处理（输入一个点，输出一个点）
 * @param  input  当前输入采样值
 * @return  滤波后输出采样值
 */
float CircuitModel_BiquadProcess(CircuitBiquad_t *filter, float input);

#endif /* CIRCUIT_MODEL_H */
