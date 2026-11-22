#include "circuit_model.h"
#include <math.h>
#include <string.h>

#define CIRCUIT_BANDPASS_MIN_DROP_DB 3.0f

/*
 * Magnitude-domain identification bounds.  The four DUTs are represented by
 * the standard second-order analog prototypes.  A coarse-to-fine grid is
 * deliberately used instead of a gradient solver: it is deterministic on the
 * MCU, needs no heap, and a complete fit is still small compared with a sweep.
 */
#define CIRCUIT_FIT_Q_MIN          0.10f
#define CIRCUIT_FIT_Q_MAX          3.00f
#define CIRCUIT_FIT_COARSE_F_STEPS 40U
#define CIRCUIT_FIT_COARSE_Q_STEPS 32U
#define CIRCUIT_FIT_REFINE_STEPS   16U
#define CIRCUIT_FIT_REFINE_ROUNDS  3U
#define CIRCUIT_FIT_MAX_NRMSE      0.08f

/* ---------- 常量定义 ---------- */
#define CIRCUIT_DB_3             3.0f                    // -3dB截止点阈值（标准半功率点）
#define CIRCUIT_DB_CLASSIFY      6.0f                    // 滤波器类型分类判定阈值（两端落差≥6dB才认定）
#define CIRCUIT_PI               3.14159265358979323846f // 圆周率高精度值
#define CIRCUIT_EDGE_WINDOW      3U                     // 端点通带平均窗口

/* ============================================================
 *  内部工具函数1：对数坐标线性插值
 *  用途：已知两个扫频点的增益，插值计算增益恰好等于阈值时对应的精确频率
 *  说明：频率轴是对数分布的，所以插值在log10空间计算后再还原
 * ============================================================ */

/**
 * @brief  在两个扫频点之间，对数插值求出指定dB增益对应的精确截止频率
 * @param  left         左侧频点（频率较低）
 * @param  right        右侧频点（频率较高）
 * @param  threshold_db 目标增益阈值（dB）
 * @return  增益等于阈值时对应的频率（Hz）
 */
static float CircuitModel_InterpolateFrequency(
    const CircuitModelPoint_t *left,
    const CircuitModelPoint_t *right,
    float threshold_db)
{
    float left_gain_db = left->gain_db;
    float right_gain_db = right->gain_db;
    float ratio;     // 阈值在两点增益之间的位置比例（0~1）
    float log_left;  // 左侧频率的对数值
    float log_right; // 右侧频率的对数值

    /* 两点增益相同或频率非法，直接返回左端点频率 */
    if ((right_gain_db == left_gain_db) ||
        (left->frequency_hz == 0U) || (right->frequency_hz == 0U))
    {
        return (float)left->frequency_hz;
    }

    /* 计算阈值在两点之间的线性比例 */
    ratio = (threshold_db - left_gain_db) /
            (right_gain_db - left_gain_db);

    /* 钳位到0~1区间，防止外推溢出 */
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    else if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    /* 频率轴按对数分布插值：先取log10，线性插值后再用10^x还原 */
    log_left = log10f((float)left->frequency_hz);
    log_right = log10f((float)right->frequency_hz);
    return powf(10.0f, log_left + ratio * (log_right - log_left));
}


/* ============================================================
 *  内部工具函数2：寻找-3dB穿越点
 *  用途：在指定扫频区间内，搜索增益曲线穿过阈值dB的那一点
 *  direction参数控制方向：上升沿/下降沿/任意穿越
 * ============================================================ */

/**
 * @brief  在扫频数据区间内查找增益穿越阈值的频率点
 * @param  model        电路模型（含全部扫频点）
 * @param  threshold_db 穿越阈值（dB，一般为峰值-3dB）
 * @param  begin        搜索起始下标
 * @param  end          搜索结束下标
 * @param  direction    穿越方向：>0只找上升沿，<0只找下降沿，0任意方向
 * @return  穿越点精确频率（Hz），未找到返回0
 */
static float CircuitModel_FindCrossing(const CircuitModel_t *model,
                                       float threshold_db,
                                       uint16_t begin,
                                       uint16_t end,
                                       int8_t direction)
{
    uint16_t index;

    if ((model == NULL) || (model->count < 2U))
    {
        return 0.0f;
    }

    /* 逐段扫描相邻两点，检测是否跨越了阈值线 */
    for (index = begin; index < end; ++index)
    {
        float left_gain = model->points[index].gain_db;
        float right_gain = model->points[index + 1U].gain_db;
        float left  = left_gain - threshold_db;
        float right = right_gain - threshold_db;

        /* 两点分别在阈值两侧（一正一负），说明发生了穿越 */
        if (((left <= 0.0f) && (right >= 0.0f)) ||
            ((left >= 0.0f) && (right <= 0.0f)))
        {
            /* 按方向筛选：上升沿增益变大，下降沿增益变小 */
            if ((direction == 0) ||
                ((direction < 0) && (right < left)) ||   // 下降沿
                ((direction > 0) && (right > left)))     // 上升沿
            {
                /* 找到穿越段，插值计算精确截止频率 */
                return CircuitModel_InterpolateFrequency(
                    &model->points[index], &model->points[index + 1U],
                    threshold_db);
            }
        }
    }
    return 0.0f; // 区间内未找到穿越点
}

typedef struct
{
    float frequency_hz;
    float q;
    float gain;
    float rms_error;
    float score;
    uint8_t valid;
} CircuitModelFit_t;

/*
 * Return the magnitude of a unity-gain normalized analog prototype at one
 * frequency.  Working with magnitude (rather than dB) naturally reduces the
 * influence of the AD637 noise floor at deep high-pass/notch attenuation.
 */
static float CircuitModel_PrototypeMagnitude(CircuitModelType_t type,
                                             float frequency_hz,
                                             float natural_hz,
                                             float q)
{
    float ratio;
    float ratio2;
    float delta;
    float denominator2;
    float numerator2;

    if ((frequency_hz <= 0.0f) || (natural_hz <= 0.0f) || (q <= 0.0f))
    {
        return 0.0f;
    }

    ratio = frequency_hz / natural_hz;
    ratio2 = ratio * ratio;
    delta = 1.0f - ratio2;
    denominator2 = delta * delta + ratio2 / (q * q);
    if (denominator2 <= 0.0f)
    {
        return 0.0f;
    }

    switch (type)
    {
        case CIRCUIT_MODEL_LOWPASS:
            numerator2 = 1.0f;
            break;
        case CIRCUIT_MODEL_HIGHPASS:
            numerator2 = ratio2 * ratio2;
            break;
        case CIRCUIT_MODEL_BANDPASS:
            numerator2 = ratio2 / (q * q);
            break;
        case CIRCUIT_MODEL_BANDSTOP:
            numerator2 = delta * delta;
            break;
        default:
            return 0.0f;
    }

    return sqrtf(numerator2 / denominator2);
}

/*
 * Evaluate one (natural frequency, Q) candidate.  For that shape, the gain
 * minimizing sum((gain * shape - measurement)^2) has the closed form below.
 * Residuals are accumulated in a second pass to avoid catastrophic
 * cancellation when an almost exact fit is evaluated with float precision.
 */
static void CircuitModel_EvaluateFit(const CircuitModel_t *model,
                                     CircuitModelType_t type,
                                     float natural_hz,
                                     float q,
                                     CircuitModelFit_t *best)
{
    uint16_t index;
    float shape;
    float sum_shape2 = 0.0f;
    float sum_gain_shape = 0.0f;
    float sum_gain2 = 0.0f;
    float fitted_gain;
    float residual;
    float sse;
    float score;

    for (index = 0U; index < model->count; ++index)
    {
        shape = CircuitModel_PrototypeMagnitude(
            type, (float)model->points[index].frequency_hz, natural_hz, q);
        sum_shape2 += shape * shape;
        sum_gain_shape += model->points[index].gain * shape;
        sum_gain2 += model->points[index].gain * model->points[index].gain;
    }

    if ((sum_shape2 <= 0.000001f) || (sum_gain2 <= 0.000001f))
    {
        return;
    }

    fitted_gain = sum_gain_shape / sum_shape2;
    if ((fitted_gain <= 0.0f) || (fitted_gain > 10.0f))
    {
        return;
    }

    sse = 0.0f;
    for (index = 0U; index < model->count; ++index)
    {
        shape = CircuitModel_PrototypeMagnitude(
            type, (float)model->points[index].frequency_hz, natural_hz, q);
        residual = fitted_gain * shape - model->points[index].gain;
        sse += residual * residual;
    }
    score = sse / sum_gain2;
    if ((best->valid == 0U) || (score < best->score))
    {
        best->frequency_hz = natural_hz;
        best->q = q;
        best->gain = fitted_gain;
        best->rms_error = sqrtf(sse / (float)model->count);
        best->score = score;
        best->valid = 1U;
    }
}

static uint8_t CircuitModel_FitSecondOrder(const CircuitModel_t *model,
                                           CircuitModelType_t type,
                                           CircuitModelFit_t *fit)
{
    uint16_t fi;
    uint16_t qi;
    uint8_t round;
    float frequency_min;
    float frequency_max;
    float q_min = CIRCUIT_FIT_Q_MIN;
    float q_max = CIRCUIT_FIT_Q_MAX;
    float frequency_step;
    float q_step;
    float previous_frequency_step;
    float previous_q_step;

    if ((model == NULL) || (fit == NULL) || (model->count < 8U))
    {
        return 0U;
    }

    (void)memset(fit, 0, sizeof(*fit));
    fit->score = 1.0e30f;
    frequency_min = (float)model->points[0].frequency_hz;
    frequency_max =
        (float)model->points[model->count - 1U].frequency_hz * 2.0f;

    frequency_step = (frequency_max - frequency_min) /
                     (float)CIRCUIT_FIT_COARSE_F_STEPS;
    q_step = (q_max - q_min) / (float)CIRCUIT_FIT_COARSE_Q_STEPS;
    for (fi = 0U; fi <= CIRCUIT_FIT_COARSE_F_STEPS; ++fi)
    {
        float candidate_frequency = frequency_min +
                                    frequency_step * (float)fi;
        for (qi = 0U; qi <= CIRCUIT_FIT_COARSE_Q_STEPS; ++qi)
        {
            CircuitModel_EvaluateFit(
                model, type, candidate_frequency,
                q_min + q_step * (float)qi, fit);
        }
    }

    for (round = 0U;
         (round < CIRCUIT_FIT_REFINE_ROUNDS) && (fit->valid != 0U);
         ++round)
    {
        previous_frequency_step = frequency_step;
        previous_q_step = q_step;
        frequency_min = fit->frequency_hz - previous_frequency_step;
        frequency_max = fit->frequency_hz + previous_frequency_step;
        q_min = fit->q - previous_q_step;
        q_max = fit->q + previous_q_step;
        if (frequency_min < (float)model->points[0].frequency_hz)
        {
            frequency_min = (float)model->points[0].frequency_hz;
        }
        if (q_min < CIRCUIT_FIT_Q_MIN)
        {
            q_min = CIRCUIT_FIT_Q_MIN;
        }
        if (q_max > CIRCUIT_FIT_Q_MAX)
        {
            q_max = CIRCUIT_FIT_Q_MAX;
        }

        frequency_step = (frequency_max - frequency_min) /
                         (float)CIRCUIT_FIT_REFINE_STEPS;
        q_step = (q_max - q_min) / (float)CIRCUIT_FIT_REFINE_STEPS;
        for (fi = 0U; fi <= CIRCUIT_FIT_REFINE_STEPS; ++fi)
        {
            float candidate_frequency = frequency_min +
                                        frequency_step * (float)fi;
            for (qi = 0U; qi <= CIRCUIT_FIT_REFINE_STEPS; ++qi)
            {
                CircuitModel_EvaluateFit(
                    model, type, candidate_frequency,
                    q_min + q_step * (float)qi, fit);
            }
        }
    }

    return ((fit->valid != 0U) &&
            (sqrtf(fit->score) <= CIRCUIT_FIT_MAX_NRMSE)) ? 1U : 0U;
}

static void CircuitModel_SetCutoffsFromFit(CircuitModel_t *model,
                                           const CircuitModelFit_t *fit)
{
    float inverse_q;
    float coefficient;
    float root;
    float ratio_low;
    float ratio_high;

    model->natural_hz = fit->frequency_hz;
    model->q = fit->q;
    model->model_gain_db = 20.0f * log10f(fit->gain);
    model->fit_rms_error = fit->rms_error;
    model->fit_valid = 1U;

    if (model->type == CIRCUIT_MODEL_LOWPASS)
    {
        coefficient = 1.0f / (fit->q * fit->q) - 2.0f;
        root = (-coefficient +
                sqrtf(coefficient * coefficient + 4.0f)) * 0.5f;
        model->f1_hz = fit->frequency_hz * sqrtf(root);
    }
    else if (model->type == CIRCUIT_MODEL_HIGHPASS)
    {
        coefficient = 2.0f - 1.0f / (fit->q * fit->q);
        root = (-coefficient +
                sqrtf(coefficient * coefficient + 4.0f)) * 0.5f;
        model->f1_hz = fit->frequency_hz * sqrtf(root);
    }
    else
    {
        inverse_q = 1.0f / fit->q;
        root = sqrtf(4.0f + inverse_q * inverse_q);
        ratio_low = (root - inverse_q) * 0.5f;
        ratio_high = (root + inverse_q) * 0.5f;
        model->f1_hz = fit->frequency_hz * ratio_low;
        model->f2_hz = fit->frequency_hz * ratio_high;
        model->center_hz = fit->frequency_hz;
    }
}

/* ============================================================
 *  对外接口1：重置模型数据
 * ============================================================ */

/**
 * @brief  清空电路模型全部数据，恢复初始未知状态
 * @param  model  电路模型结构体指针
 */
void CircuitModel_Reset(CircuitModel_t *model)
{
    if (model != NULL)
    {
        (void)memset(model, 0, sizeof(*model));       // 全部字段清零
        model->type = CIRCUIT_MODEL_UNKNOWN;          // 类型设为未知
        model->peak_gain_db = -120.0f;                // 峰值增益初始设为极小值
        model->model_gain_db = -120.0f;
    }
}

/* ============================================================
 *  对外接口2：追加一个扫频数据点
 *  扫频过程中每测完一个频点，调用本函数存入模型
 * ============================================================ */

/**
 * @brief  向模型追加一个扫频测量点（频率+输入输出RMS）
 * @param  model         电路模型
 * @param  frequency_hz  当前测量频率（Hz）
 * @param  input_rms     输入端有效值电压
 * @param  output_rms    输出端有效值电压
 * @retval 1=添加成功，0=参数非法或缓冲区已满
 */
uint8_t CircuitModel_Append(CircuitModel_t *model,
                             uint32_t frequency_hz,
                             float input_rms,
                             float output_rms)
{
    CircuitModelPoint_t *point;

    /* 参数合法性与缓冲区剩余空间检查 */
    if ((model == NULL) || (model->count >= CIRCUIT_MODEL_MAX_POINTS) ||
        (frequency_hz == 0U) || (input_rms <= 0.000001f) ||
        (output_rms < 0.0f))
    {
        return 0U;
    }

    /* 写入新数据点 */
    point = &model->points[model->count];
    point->frequency_hz = frequency_hz;
    point->input_rms    = input_rms;
    point->output_rms   = output_rms;

    /* 计算该频点的线性增益与dB增益 */
    point->gain    = output_rms / input_rms;
    point->gain_db = 20.0f * log10f(point->gain > 0.000001f ?
                                    point->gain : 0.000001f);
    /* 更新全局峰值增益（边扫频边记录最大值） */
    if (point->gain_db > model->peak_gain_db)
    {
        model->peak_gain_db = point->gain_db;
    }

    model->count++;
    return 1U;
}

/* ============================================================
 *  对外接口3：分析扫频数据，自动识别滤波器类型并计算参数
 *  核心算法：根据幅频曲线形状分类 → 找-3dB截止点 → 算中心频率和Q值
 *  支持识别：低通、高通、带通、带阻（陷波）四种经典二阶滤波器
 * ============================================================ */

/**
 * @brief  对全部扫频点进行分析，自动识别滤波器类型并提取特征参数
 * @param  model  电路模型（已填充扫频数据）
 * @retval 1=分析成功识别出类型，0=数据不足或无法识别
 * @note   识别逻辑：
 *         - 中间凹陷+两端高 → 带阻
 *         - 中间凸起+两端低 → 带通
 *         - 左高右低单调下降 → 低通
 *         - 左低右高单调上升 → 高通
 */
uint8_t CircuitModel_Analyze(CircuitModel_t *model)
{
    CircuitModelFit_t fit;
    uint16_t index;
    uint16_t edge_index;
    uint16_t edge_window;
    uint16_t peak_index = 0U;    // 峰值点下标
    uint16_t min_index  = 0U;    // 谷值点下标
    float    min_gain   = 120.0f;// 最小增益初始值
    float    first_gain;         // 低频端通带平均增益
    float    last_gain;          // 高频端通带平均增益
    float    edge_average;       // 两端通带平均值（用于带阻判定）
    float    threshold_db;       // -3dB截止阈值

    if ((model == NULL) || (model->count < 3U))
    {
        return 0U;
    }

    /* ---------- 第一步：遍历找出峰值点和谷值点 ---------- */
    model->peak_gain_db = model->points[0].gain_db;
    min_gain = model->peak_gain_db;

    for (index = 1U; index < model->count; ++index)
    {
        /* 更新最大值（峰值） */
        if (model->points[index].gain_db > model->peak_gain_db)
        {
            model->peak_gain_db = model->points[index].gain_db;
            peak_index = index;
        }
        /* 更新最小值（谷值） */
        if (model->points[index].gain_db < min_gain)
        {
            min_gain = model->points[index].gain_db;
            min_index = index;
        }
    }

    /*
     * 不直接使用单个端点判断类型。扫频端点容易受 AD637 建立时间、
     * ADC 噪声和 DDS 切频瞬态影响；使用端点附近少量点的平均值，
     * 仍保持对数扫频的整体形状，但不会因一个端点异常而四类全判未知。
     */
    edge_window = (model->count < CIRCUIT_EDGE_WINDOW) ?
                  model->count : CIRCUIT_EDGE_WINDOW;
    first_gain = 0.0f;
    last_gain = 0.0f;
    for (edge_index = 0U; edge_index < edge_window; ++edge_index)
    {
        first_gain += model->points[edge_index].gain_db;
        last_gain += model->points[model->count - 1U - edge_index].gain_db;
    }
    first_gain /= (float)edge_window;
    last_gain /= (float)edge_window;
    edge_average   = (first_gain + last_gain) * 0.5f;
    model->type   = CIRCUIT_MODEL_UNKNOWN;

    /* ---------- 第二步：根据幅频曲线形状分类识别滤波器类型 ---------- */

    /* 带阻（陷波）判定：中间有明显凹陷，且凹陷深度≥6dB */
    if ((peak_index > 0U) && (peak_index + 1U < model->count) &&
             (model->peak_gain_db - first_gain >= CIRCUIT_BANDPASS_MIN_DROP_DB) &&
             (model->peak_gain_db - last_gain  >= CIRCUIT_BANDPASS_MIN_DROP_DB))
    {
        model->type = CIRCUIT_MODEL_BANDPASS;
    }
    /* 低通判定：左端比右端高≥6dB，整体单调下降 */
    else if (first_gain - last_gain >= CIRCUIT_DB_CLASSIFY)
    {
        model->type = CIRCUIT_MODEL_LOWPASS;
    }
    /* 高通判定：右端比左端高≥6dB，整体单调上升 */
    else if (last_gain - first_gain >= CIRCUIT_DB_CLASSIFY)
    {
        model->type = CIRCUIT_MODEL_HIGHPASS;
    }
    else if ((min_index > 0U) && (min_index + 1U < model->count) &&
             (edge_average - min_gain >= CIRCUIT_DB_CLASSIFY))
    {
        model->type = CIRCUIT_MODEL_BANDSTOP;
    }

    /* 参数初始化清零 */
    model->f1_hz     = 0.0f;
    model->f2_hz     = 0.0f;
    model->center_hz = 0.0f;
    model->q         = 0.0f;
    model->natural_hz = 0.0f;
    model->model_gain_db = model->peak_gain_db;
    model->fit_rms_error = 0.0f;
    model->fit_valid = 0U;

    /*
     * 通带类滤波器的-3dB基准是峰值往下3dB；
     * 陷波（带阻）滤波器的-3dB基准是通带电平往下3dB，而不是谷值往上3dB。
     * 这里先用峰值统一计算，带阻类型后面会单独修正阈值。
     */
    threshold_db = model->peak_gain_db - CIRCUIT_DB_3;

    /* ---------- 第三步：按类型分别计算截止频率、中心频率、Q值 ---------- */

    if (model->type == CIRCUIT_MODEL_LOWPASS)
    {
        /* 低通：从峰值向右（高频方向）找下降沿的-3dB点 */
        model->f1_hz = CircuitModel_FindCrossing(
            model, threshold_db, peak_index, model->count - 1U, -1);
    }
    else if (model->type == CIRCUIT_MODEL_HIGHPASS)
    {
        /* 高通：从左端向峰值找上升沿的-3dB点 */
        model->f1_hz = CircuitModel_FindCrossing(
            model, threshold_db, 0U, peak_index, 1);
    }
    else if (model->type == CIRCUIT_MODEL_BANDPASS)
    {
        /* 带通：左侧找上升沿截止频率f1，右侧找下降沿截止频率f2 */
        model->f1_hz = CircuitModel_FindCrossing(
            model, threshold_db, 0U, peak_index, 1);
        model->f2_hz = CircuitModel_FindCrossing(
            model, threshold_db, peak_index, model->count - 1U, -1);

        /* 两个截止点都有效时，计算几何中心频率和品质因数Q */
        if ((model->f1_hz > 0.0f) && (model->f2_hz > model->f1_hz))
        {
            model->center_hz = sqrtf(model->f1_hz * model->f2_hz); // 几何中心频率
            model->q = model->center_hz / (model->f2_hz - model->f1_hz); // Q=中心频率/带宽
        }
    }
    else if (model->type == CIRCUIT_MODEL_BANDSTOP)
    {
        /* 带阻特殊处理：阈值改为通带电平往下3dB（不是谷值往上） */
        threshold_db = edge_average - CIRCUIT_DB_3;

        /* 左侧找下降沿（进入陷波），右侧找上升沿（离开陷波） */
        model->f1_hz = CircuitModel_FindCrossing(
            model, threshold_db, 0U, min_index, -1);
        model->f2_hz = CircuitModel_FindCrossing(
            model, threshold_db, min_index, model->count - 1U, 1);

        if ((model->f1_hz > 0.0f) && (model->f2_hz > model->f1_hz))
        {
            model->center_hz = sqrtf(model->f1_hz * model->f2_hz);
            model->q = model->center_hz / (model->f2_hz - model->f1_hz);
        }
    }

    /*
     * Threshold crossings are retained as a safe fallback and for
     * classification.  The fit then replaces them with the parameters of the
     * complete response.  This is essential for LP/HP, where a single cutoff
     * does not determine Q, and for a notch whose upper pass band is not flat
     * yet at the end of a 50 kHz sweep.
     */
    if ((model->type != CIRCUIT_MODEL_UNKNOWN) &&
        (CircuitModel_FitSecondOrder(model, model->type, &fit) != 0U))
    {
        CircuitModel_SetCutoffsFromFit(model, &fit);
    }

    return (model->type != CIRCUIT_MODEL_UNKNOWN) ? 1U : 0U;
}

/* ============================================================
 *  对外接口4：获取滤波器类型名称字符串
 * ============================================================ */

/**
 * @brief  将滤波器类型枚举转换为可读字符串（用于UI显示）
 * @param  type  滤波器类型枚举
 * @return  类型名称字符串
 */
const char *CircuitModel_TypeName(CircuitModelType_t type)
{
    switch (type)
    {
        case CIRCUIT_MODEL_LOWPASS:  return "LOWPASS";   // 低通
        case CIRCUIT_MODEL_HIGHPASS: return "HIGHPASS";  // 高通
        case CIRCUIT_MODEL_BANDPASS: return "BANDPASS";  // 带通
        case CIRCUIT_MODEL_BANDSTOP: return "BANDSTOP";  // 带阻/陷波
        default:                     return "UNKNOWN";   // 未知
    }
}

/* ============================================================
 *  对外接口5：双二阶（Biquad）数字滤波器初始化
 *  用途：用识别出的模拟电路参数，生成等效数字滤波器
 *  算法：标准RBJ双二阶滤波器设计公式（模拟s域→数字z域双线性变换）
 *  支持：低通、高通、带通、带阻四种类型
 * ============================================================ */

/**
 * @brief  初始化双二阶IIR数字滤波器（根据模拟电路参数生成等效数字滤波系数）
 * @param  filter              滤波器结构体
 * @param  type                滤波器类型（低通/高通/带通/带阻）
 * @param  sample_rate_hz      数字系统采样率（Hz）
 * @param  center_frequency_hz 截止/中心频率（Hz）
 * @param  q                   品质因数Q（控制谐振峰陡峭程度）
 * @retval 1=初始化成功，0=参数非法
 * @note   采用标准RBJ音频EQ双二阶公式，直接型I结构
 */
uint8_t CircuitModel_BiquadInit(CircuitBiquad_t *filter,
                                CircuitModelType_t type,
                                float sample_rate_hz,
                                float center_frequency_hz,
                                float q)
{
    float omega;    // 归一化角频率 = 2πf/Fs
    float sine;     // sin(ω)
    float cosine;   // cos(ω)
    float alpha;    // 中间变量 = sin(ω)/(2Q)
    float a0, a1, a2; // 分母系数（反馈端）
    float b0, b1, b2; // 分子系数（前馈端）

    /* 参数合法性检查：频率不能超过奈奎斯特频率（采样率一半） */
    if ((filter == NULL) || (sample_rate_hz <= 0.0f) ||
        (center_frequency_hz <= 0.0f) ||
        (center_frequency_hz >= sample_rate_hz * 0.5f) || (q <= 0.0f))
    {
        return 0U;
    }

    /* RBJ双二阶标准中间变量计算 */
    omega  = 2.0f * CIRCUIT_PI * center_frequency_hz / sample_rate_hz;
    sine   = sinf(omega);
    cosine = cosf(omega);
    alpha  = sine / (2.0f * q);

    /* 按滤波器类型计算分子b系数（前馈） */
    if (type == CIRCUIT_MODEL_LOWPASS)
    {
        b0 = (1.0f - cosine) * 0.5f;
        b1 = 1.0f - cosine;
        b2 = b0;
    }
    else if (type == CIRCUIT_MODEL_HIGHPASS)
    {
        b0 = (1.0f + cosine) * 0.5f;
        b1 = -(1.0f + cosine);
        b2 = b0;
    }
    else if (type == CIRCUIT_MODEL_BANDPASS)
    {
        /*
         * Constant-0-dB-peak form.  Using sin(omega)/2 here implements the
         * constant-skirt form whose peak is Q; multiplying that by the fitted
         * peak gain a second time made every Q != 1 band-pass amplitude wrong.
         */
        b0 = alpha;
        b1 = 0.0f;
        b2 = -b0;
    }
    else if (type == CIRCUIT_MODEL_BANDSTOP)
    {
        b0 = 1.0f;
        b1 = -2.0f * cosine;
        b2 = 1.0f;
    }
    else
    {
        return 0U;
    }

    /* 分母a系数（反馈），四种类型完全相同 */
    a0 = 1.0f + alpha;
    a1 = -2.0f * cosine;
    a2 = 1.0f - alpha;

    /* 全部系数除以a0归一化（使y[n]系数为1），存入滤波器结构体 */
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    /* 增益与状态变量初始化 */
    filter->gain = 1.0f;
    filter->x1 = 0.0f; filter->x2 = 0.0f; // 输入延迟线
    filter->y1 = 0.0f; filter->y2 = 0.0f; // 输出延迟线
    return 1U;
}

/* ============================================================
 *  对外接口6：设置数字滤波器整体增益
 * ============================================================ */

/**
 * @brief  设置双二阶滤波器的整体增益倍数
 * @param  filter  滤波器结构体
 * @param  gain    增益倍数（非负）
 */
void CircuitModel_BiquadSetGain(CircuitBiquad_t *filter, float gain)
{
    if (filter == NULL)
    {
        return;
    }
    if (gain < 0.0f)
    {
        gain = 0.0f;
    }
    filter->gain = gain;
}

/* ============================================================
 *  对外接口7：双二阶滤波器单样本实时处理
 *  差分方程：y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
 *  直接I型结构，每输入一个采样点输出一个滤波结果
 * ============================================================ */

/**
 * @brief  输入一个采样点，经过双二阶滤波后返回输出值（实时处理）
 * @param  filter  滤波器结构体（内部维护状态变量）
 * @param  input   当前输入采样值
 * @return  滤波后的输出采样值
 * @note   直接I型二阶IIR差分方程
 */
float CircuitModel_BiquadProcess(CircuitBiquad_t *filter, float input)
{
    float output;

    if (filter == NULL)
    {
        return 0.0f;
    }

    /* 双二阶差分方程运算 */
    output = filter->b0 * input + filter->b1 * filter->x1 +
             filter->b2 * filter->x2 - filter->a1 * filter->y1 -
             filter->a2 * filter->y2;

    /* 移位更新延迟线（为下一个采样点准备历史状态） */
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    /* 乘以整体增益后返回 */
    return output * filter->gain;
}
