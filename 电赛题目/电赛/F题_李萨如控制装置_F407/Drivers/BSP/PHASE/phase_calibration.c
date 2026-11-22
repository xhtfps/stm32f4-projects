/**
 * @file    phase_calibration.c
 * @brief   相位校准模块实现
 * @details 负责相位和增益误差的校准数据存储（Flash）、在线学习、插值应用。
 *          支持多个频率点的校准，并针对直接采样和混频采样两种ADC模式分别校准。
 *          采用CRC校验确保Flash中数据的完整性，学习过程采用向量平均法抑制噪声。
 *
 * @version 1.0
 * @date    2026-07-28
 * @author  AI Assistant
 */

#include "phase_calibration.h"

#include "stm32f4xx_hal.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/*============================== 内部宏定义 ==============================*/

/**
 * @brief 校准记录的魔数，用于识别有效数据
 * @note  值 "PHCL" 的ASCII编码
 */
#define PHASE_CALIBRATION_MAGIC          0x5048434CUL /* "PHCL" */

/**
 * @brief 校准记录版本号
 * @note  版本2表示使用AD9959 CH2高侧注入（输入+10kHz）方式。
 *        版本号用于向前兼容。
 */
#define PHASE_CALIBRATION_VERSION        2U

/**
 * @brief 校准数据在Flash中的存储起始地址
 * @note  0x080E0000 对应STM32F4的扇区11（256KB），需确保不与其他数据重叠。
 */
#define PHASE_CALIBRATION_FLASH_ADDRESS  0x080E0000UL

/**
 * @brief 存储校准数据所使用的Flash扇区编号
 * @note  扇区11（Sector 11）大小为128KB（对于STM32F42x/43x系列）
 */
#define PHASE_CALIBRATION_FLASH_SECTOR   FLASH_SECTOR_11

/**
 * @brief 圆周率π的近似值（单精度）
 */
#define PHASE_CALIBRATION_PI             3.14159265358979323846f

/**
 * @brief 频率匹配容差百分比
 * @details 当学习样本的频率与目标频率相差此百分比以内时，才接受该样本。
 */
#define PHASE_CALIBRATION_FREQ_TOL_PCT   2U

/*============================== 内部数据结构 ==============================*/

/**
 * @brief 单个频率点的校准数据
 */
typedef struct
{
    uint32_t frequency_hz;      /**< 频率点（Hz） */
    float phase_error_deg;      /**< 相位误差（度），范围 -180 ~ +180 */
    float gain_error_db;        /**< 增益误差（dB） */
    uint8_t adc_mode;           /**< 对应的ADC模式（PhaseAdcMode_t） */
    uint8_t reserved[3];        /**< 保留字节，保持对齐 */
} PhaseCalibrationPoint_t;

/**
 * @brief Flash中存储的完整校准记录
 */
typedef struct
{
    uint32_t magic;             /**< 魔数，用于识别有效记录 */
    uint16_t version;           /**< 版本号 */
    uint8_t point_count;        /**< 实际存储的频率点数量（应等于PHASE_CALIBRATION_POINT_COUNT） */
    uint8_t reserved;           /**< 保留字节 */
    PhaseCalibrationPoint_t points[PHASE_CALIBRATION_POINT_COUNT]; /**< 各频率点数据 */
    uint32_t crc32;             /**< 整个记录的CRC32校验值（不包括自身） */
} PhaseCalibrationRecord_t;

/**
 * @brief 模块内部运行上下文
 */
typedef struct
{
    PhaseCalibrationRecord_t active;   /**< 当前生效的校准数据（从Flash加载或学习完成后的数据） */
    PhaseCalibrationRecord_t working;  /**< 学习过程中的工作区数据 */
    PhaseCalibrationStatus_t status;   /**< 状态信息（对外可见） */
    float phase_sin_sum;               /**< 相位正弦值累加和（用于向量平均） */
    float phase_cos_sum;               /**< 相位余弦值累加和（用于向量平均） */
    float gain_sum;                    /**< 增益累加和（用于算术平均） */
} PhaseCalibrationContext_t;

/*============================== 内部全局变量 ==============================*/

/**
 * @brief 预定义的学习频率点列表（单位：Hz）
 * @note  这些频率点覆盖了从低频到高频的典型范围，每个点均适用于直接或混频模式。
 */
static const uint32_t s_learning_frequencies_hz[
    PHASE_CALIBRATION_POINT_COUNT] =
{
    10U,
    100U,
    1000U,
    10000U,
    50000U,
    100000U,
    180000U,
    220000U,
    500000U,
    1000000U,
    2000000U,
    5000000U,
    10000000U
};

/**
 * @brief 模块全局上下文（静态变量）
 */
static PhaseCalibrationContext_t s_calibration;

/*============================== 内部函数原型 ==============================*/

/**
 * @brief 将角度归一化到 (-180, 180] 度范围
 * @param degrees 输入角度（度）
 * @return 归一化后的角度（度）
 */
static float PhaseCalibration_NormalizeDegrees(float degrees);

/**
 * @brief 根据频率确定ADC模式（直接或混频）
 * @param frequency_hz 频率值
 * @return 对应的PhaseAdcMode_t枚举值
 */
static PhaseAdcMode_t PhaseCalibration_ModeForFrequency(
    uint32_t frequency_hz);

/**
 * @brief 计算CRC32校验值（兼容标准CRC-32）
 * @param data 数据缓冲区指针
 * @param length 数据长度（字节）
 * @return 计算出的CRC32值
 */
static uint32_t PhaseCalibration_Crc32(const uint8_t *data,
                                       uint32_t length);

/**
 * @brief 初始化一个校准记录结构，填充魔数、版本、点数和各频率点的ADC模式
 * @param record 指向要初始化的记录结构
 */
static void PhaseCalibration_PrepareRecord(
    PhaseCalibrationRecord_t *record);

/**
 * @brief 校验校准记录的有效性（魔数、版本、点数量、CRC、频率匹配）
 * @param record 指向待校验记录
 * @return 1有效，0无效
 */
static uint8_t PhaseCalibration_RecordIsValid(
    const PhaseCalibrationRecord_t *record);

/**
 * @brief 将校准记录写入Flash（擦除扇区，编程，并验证）
 * @param record 指向要保存的记录
 * @return 1成功，0失败
 */
static uint8_t PhaseCalibration_SaveRecord(
    PhaseCalibrationRecord_t *record);

/**
 * @brief 在当前激活的校准数据中，查找不高于指定频率且模式匹配的最近低频点
 * @param frequency_hz 目标频率
 * @param adc_mode 目标模式
 * @return 指向频率最低且<=frequency_hz的点，若不存在则返回NULL
 */
static const PhaseCalibrationPoint_t *PhaseCalibration_FindLower(
    uint32_t frequency_hz,
    PhaseAdcMode_t adc_mode);

/**
 * @brief 在当前激活的校准数据中，查找不低于指定频率且模式匹配的最近高频点
 * @param frequency_hz 目标频率
 * @param adc_mode 目标模式
 * @return 指向频率最高且>=frequency_hz的点，若不存在则返回NULL
 */
static const PhaseCalibrationPoint_t *PhaseCalibration_FindUpper(
    uint32_t frequency_hz,
    PhaseAdcMode_t adc_mode);

/*============================== 函数实现 ==============================*/

/**
 * @brief 归一化角度到 (-180, 180] 度
 * @param degrees 输入角度（度）
 * @return 归一化后的角度
 */
static float PhaseCalibration_NormalizeDegrees(float degrees)
{
    while (degrees > 180.0f)
    {
        degrees -= 360.0f;
    }
    while (degrees <= -180.0f)
    {
        degrees += 360.0f;
    }
    return degrees;
}

/**
 * @brief 根据频率选择ADC模式（直接或混频）
 * @param frequency_hz 频率
 * @return 模式值
 */
static PhaseAdcMode_t PhaseCalibration_ModeForFrequency(
    uint32_t frequency_hz)
{
    return (frequency_hz > PHASE_ROUTE_MIX_UP_HZ) ?
           PHASE_ADC_MODE_MIXED : PHASE_ADC_MODE_DIRECT;
}

/**
 * @brief 标准CRC-32计算（使用常用多项式0xEDB88320）
 * @param data 数据指针
 * @param length 数据长度（字节）
 * @return CRC32值
 */
static uint32_t PhaseCalibration_Crc32(const uint8_t *data,
                                       uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 1UL) != 0UL) ?
                  ((crc >> 1U) ^ 0xEDB88320UL) :
                  (crc >> 1U);
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

/**
 * @brief 初始化校准记录，填入魔数、版本、点数和各点的ADC模式
 * @param record 待初始化的记录指针
 */
static void PhaseCalibration_PrepareRecord(
    PhaseCalibrationRecord_t *record)
{
    uint8_t index;

    (void)memset(record, 0, sizeof(*record));
    record->magic = PHASE_CALIBRATION_MAGIC;
    record->version = PHASE_CALIBRATION_VERSION;
    record->point_count = PHASE_CALIBRATION_POINT_COUNT;
    for (index = 0U; index < PHASE_CALIBRATION_POINT_COUNT; ++index)
    {
        record->points[index].frequency_hz =
            s_learning_frequencies_hz[index];
        record->points[index].adc_mode = (uint8_t)
            PhaseCalibration_ModeForFrequency(
                s_learning_frequencies_hz[index]);
    }
}

/**
 * @brief 检查校准记录是否有效
 * @details 验证魔数、版本、点数、CRC，并确保各频率点与预定义列表一致
 * @param record 记录指针
 * @return 1有效，0无效
 */
static uint8_t PhaseCalibration_RecordIsValid(
    const PhaseCalibrationRecord_t *record)
{
    uint32_t expected_crc;
    uint8_t index;

    if ((record == NULL) ||
        (record->magic != PHASE_CALIBRATION_MAGIC) ||
        (record->version != PHASE_CALIBRATION_VERSION) ||
        (record->point_count != PHASE_CALIBRATION_POINT_COUNT))
    {
        return 0U;
    }

    /* 计算不包括crc32字段的CRC */
    expected_crc = PhaseCalibration_Crc32(
        (const uint8_t *)record,
        (uint32_t)offsetof(PhaseCalibrationRecord_t, crc32));
    if (expected_crc != record->crc32)
    {
        return 0U;
    }

    /* 验证每个点的频率和ADC模式是否与预定义一致 */
    for (index = 0U; index < PHASE_CALIBRATION_POINT_COUNT; ++index)
    {
        if ((record->points[index].frequency_hz !=
             s_learning_frequencies_hz[index]) ||
            (record->points[index].adc_mode != (uint8_t)
             PhaseCalibration_ModeForFrequency(
                 s_learning_frequencies_hz[index])))
        {
            return 0U;
        }
    }
    return 1U;
}

/**
 * @brief 将校准记录写入Flash（擦除、编程、校验）
 * @param record 要保存的记录
 * @return 1成功，0失败
 */
static uint8_t PhaseCalibration_SaveRecord(
    PhaseCalibrationRecord_t *record)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t sector_error = 0U;
    const uint32_t *words;
    uint32_t offset;
    HAL_StatusTypeDef status;

    /* 记录结构体大小必须为4字节的整数倍，确保编程时按字对齐 */
    if ((record == NULL) || ((sizeof(*record) & 3U) != 0U))
    {
        return 0U;
    }

    /* 在写入前先计算并填充CRC */
    record->crc32 = PhaseCalibration_Crc32(
        (const uint8_t *)record,
        (uint32_t)offsetof(PhaseCalibrationRecord_t, crc32));

    /* 配置扇区擦除 */
    (void)memset(&erase, 0, sizeof(erase));
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = PHASE_CALIBRATION_FLASH_SECTOR;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    /* 解锁Flash并清除错误标志 */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return 0U;
    }
    __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    /* 擦除扇区 */
    status = HAL_FLASHEx_Erase(&erase, &sector_error);

    /* 按字（32位）编程记录数据 */
    words = (const uint32_t *)record;
    for (offset = 0U;
         (status == HAL_OK) && (offset < sizeof(*record));
         offset += sizeof(uint32_t))
    {
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_WORD,
            PHASE_CALIBRATION_FLASH_ADDRESS + offset,
            words[offset / sizeof(uint32_t)]);
    }
    (void)HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        return 0U;
    }

    /* 验证写入结果（字节比较） */
    return (memcmp(
        (const void *)PHASE_CALIBRATION_FLASH_ADDRESS,
        record, sizeof(*record)) == 0) ? 1U : 0U;
}

/**
 * @brief 模块初始化函数，从Flash加载校准数据
 * @return 1如果有效的校准数据存在且加载成功，否则0
 */
uint8_t PhaseCalibration_Init(void)
{
    const PhaseCalibrationRecord_t *stored =
        (const PhaseCalibrationRecord_t *)
        PHASE_CALIBRATION_FLASH_ADDRESS;

    /* 清空上下文 */
    (void)memset(&s_calibration, 0, sizeof(s_calibration));

    /* 检查Flash中存储的记录是否有效，若有效则复制到active区 */
    if (PhaseCalibration_RecordIsValid(stored) != 0U)
    {
        s_calibration.active = *stored;
        s_calibration.status.calibration_valid = 1U;
    }
    s_calibration.status.point_total =
        PHASE_CALIBRATION_POINT_COUNT;
    return s_calibration.status.calibration_valid;
}

/**
 * @brief 开始学习过程
 * @details 初始化工作记录，重置学习状态，第一个目标频率为预定义列表的第一个点。
 *          学习激活标志置位。
 */
void PhaseCalibration_StartLearning(void)
{
    PhaseCalibration_PrepareRecord(&s_calibration.working);
    s_calibration.status.learning_active = 1U;
    s_calibration.status.save_ok = 0U;
    s_calibration.status.point_index = 0U;
    s_calibration.status.point_total =
        PHASE_CALIBRATION_POINT_COUNT;
    s_calibration.status.sample_count = 0U;
    s_calibration.status.target_frequency_hz =
        s_learning_frequencies_hz[0];
    s_calibration.status.last_phase_error_deg = 0.0f;
    s_calibration.status.last_gain_error_db = 0.0f;
    s_calibration.phase_sin_sum = 0.0f;
    s_calibration.phase_cos_sum = 0.0f;
    s_calibration.gain_sum = 0.0f;
}

/**
 * @brief 中止当前学习过程
 * @details 清除学习标志，重置累加器，但保留已学习的部分数据（working记录保留）。
 */
void PhaseCalibration_AbortLearning(void)
{
    s_calibration.status.learning_active = 0U;
    s_calibration.status.sample_count = 0U;
    s_calibration.status.target_frequency_hz = 0U;
    s_calibration.phase_sin_sum = 0.0f;
    s_calibration.phase_cos_sum = 0.0f;
    s_calibration.gain_sum = 0.0f;
}

/**
 * @brief 查询是否正在学习
 * @return 1正在学习，0未学习
 */
uint8_t PhaseCalibration_IsLearning(void)
{
    return s_calibration.status.learning_active;
}

/**
 * @brief 获取当前学习目标频率
 * @return 目标频率（Hz），若无学习则返回0
 */
uint32_t PhaseCalibration_GetTargetFrequency(void)
{
    return s_calibration.status.target_frequency_hz;
}

/**
 * @brief 向学习过程添加一个样本
 * @param frequency_hz  当前测量的频率（Hz）
 * @param adc_mode      当前使用的ADC模式
 * @param raw_phase_deg 原始相位误差（度）
 * @param raw_gain_db   原始增益误差（dB）
 * @return 学习事件类型，指示是否需要切换频率或学习完成
 */
PhaseCalibrationLearnEvent_t PhaseCalibration_AddSample(
    uint32_t frequency_hz,
    PhaseAdcMode_t adc_mode,
    float raw_phase_deg,
    float raw_gain_db)
{
    uint32_t target_hz;
    uint32_t difference_hz;
    uint32_t tolerance_hz;
    float radians;
    PhaseCalibrationPoint_t *point;

    if (s_calibration.status.learning_active == 0U)
    {
        return PHASE_CALIBRATION_LEARN_NONE;
    }

    /* 检查频率是否匹配目标频率（在容差范围内）以及ADC模式是否正确 */
    target_hz = s_calibration.status.target_frequency_hz;
    difference_hz = (frequency_hz >= target_hz) ?
                    (frequency_hz - target_hz) :
                    (target_hz - frequency_hz);
    tolerance_hz =
        (target_hz * PHASE_CALIBRATION_FREQ_TOL_PCT) / 100U;
    if (tolerance_hz < 2U)
    {
        tolerance_hz = 2U;
    }
    if ((difference_hz > tolerance_hz) ||
        (adc_mode != PhaseCalibration_ModeForFrequency(target_hz)))
    {
        return PHASE_CALIBRATION_LEARN_NONE;
    }

    /* 归一化相位并累加向量（sin/cos）和增益 */
    raw_phase_deg = PhaseCalibration_NormalizeDegrees(raw_phase_deg);
    radians = raw_phase_deg * PHASE_CALIBRATION_PI / 180.0f;
    s_calibration.phase_sin_sum += sinf(radians);
    s_calibration.phase_cos_sum += cosf(radians);
    s_calibration.gain_sum += raw_gain_db;
    ++s_calibration.status.sample_count;

    /* 若样本数量不足，继续累积 */
    if (s_calibration.status.sample_count <
        PHASE_CALIBRATION_SAMPLES_PER_POINT)
    {
        return PHASE_CALIBRATION_LEARN_NONE;
    }

    /* 样本数达标，计算该频率点的平均相位和增益误差 */
    point = &s_calibration.working.points[
        s_calibration.status.point_index];
    point->phase_error_deg = PhaseCalibration_NormalizeDegrees(
        atan2f(s_calibration.phase_sin_sum,
               s_calibration.phase_cos_sum) *
        180.0f / PHASE_CALIBRATION_PI);
    point->gain_error_db =
        s_calibration.gain_sum /
        (float)PHASE_CALIBRATION_SAMPLES_PER_POINT;
    s_calibration.status.last_phase_error_deg =
        point->phase_error_deg;
    s_calibration.status.last_gain_error_db =
        point->gain_error_db;

    /* 判断是否所有频率点都已学习完毕 */
    if ((uint8_t)(s_calibration.status.point_index + 1U) >=
        PHASE_CALIBRATION_POINT_COUNT)
    {
        /* 学习完成，将working复制到active，并清除学习标志 */
        s_calibration.status.learning_active = 0U;
        s_calibration.status.sample_count =
            PHASE_CALIBRATION_SAMPLES_PER_POINT;
        s_calibration.status.target_frequency_hz = target_hz;
        s_calibration.active = s_calibration.working;
        s_calibration.status.calibration_valid = 1U;
        s_calibration.status.save_ok = 0U;
        return PHASE_CALIBRATION_LEARN_COMPLETE;
    }

    /* 准备下一个频率点 */
    ++s_calibration.status.point_index;
    s_calibration.status.sample_count = 0U;
    s_calibration.status.target_frequency_hz =
        s_learning_frequencies_hz[
            s_calibration.status.point_index];
    s_calibration.phase_sin_sum = 0.0f;
    s_calibration.phase_cos_sum = 0.0f;
    s_calibration.gain_sum = 0.0f;
    return PHASE_CALIBRATION_LEARN_NEXT_POINT;
}

/**
 * @brief 将当前激活的校准数据保存到Flash
 * @return 1保存成功，0失败
 */
uint8_t PhaseCalibration_Save(void)
{
    if (s_calibration.status.calibration_valid == 0U)
    {
        s_calibration.status.save_ok = 0U;
        return 0U;
    }

    s_calibration.status.save_ok =
        PhaseCalibration_SaveRecord(&s_calibration.active);
    return s_calibration.status.save_ok;
}

/**
 * @brief 在激活校准数据中查找不高于目标频率且模式匹配的最低频率点
 * @param frequency_hz 目标频率
 * @param adc_mode 模式
 * @return 指向匹配点的指针，不存在则NULL
 */
static const PhaseCalibrationPoint_t *PhaseCalibration_FindLower(
    uint32_t frequency_hz,
    PhaseAdcMode_t adc_mode)
{
    const PhaseCalibrationPoint_t *lower = NULL;
    uint8_t index;

    for (index = 0U; index < PHASE_CALIBRATION_POINT_COUNT; ++index)
    {
        const PhaseCalibrationPoint_t *point =
            &s_calibration.active.points[index];

        if (point->adc_mode != (uint8_t)adc_mode)
        {
            continue;
        }
        if (point->frequency_hz <= frequency_hz)
        {
            lower = point;
        }
        else
        {
            break;  /* 由于点是按频率升序排列的，超出则终止 */
        }
    }
    return lower;
}

/**
 * @brief 在激活校准数据中查找不低于目标频率且模式匹配的最高频率点
 * @param frequency_hz 目标频率
 * @param adc_mode 模式
 * @return 指向匹配点的指针，不存在则NULL
 */
static const PhaseCalibrationPoint_t *PhaseCalibration_FindUpper(
    uint32_t frequency_hz,
    PhaseAdcMode_t adc_mode)
{
    const PhaseCalibrationPoint_t *upper = NULL;
    uint8_t index;

    for (index = 0U; index < PHASE_CALIBRATION_POINT_COUNT; ++index)
    {
        const PhaseCalibrationPoint_t *point =
            &s_calibration.active.points[index];

        if (point->adc_mode != (uint8_t)adc_mode)
        {
            continue;
        }
        upper = point;
        if (point->frequency_hz >= frequency_hz)
        {
            break;
        }
    }
    return upper;
}

/**
 * @brief 对给定的相位和增益误差应用校准修正（线性插值）
 * @param frequency_hz 当前频率
 * @param adc_mode 当前ADC模式
 * @param phase_deg 输入/输出：原始相位误差（度），将被修正
 * @param gain_db   输入/输出：原始增益误差（dB），将被修正
 * @note  若校准无效或正在学习，则不做任何修改。
 */
void PhaseCalibration_Apply(uint32_t frequency_hz,
                            PhaseAdcMode_t adc_mode,
                            float *phase_deg,
                            float *gain_db)
{
    const PhaseCalibrationPoint_t *lower;
    const PhaseCalibrationPoint_t *upper;
    float ratio;
    float phase_delta;
    float phase_error;
    float gain_error;

    if ((phase_deg == NULL) || (gain_db == NULL) ||
        (s_calibration.status.calibration_valid == 0U) ||
        (s_calibration.status.learning_active != 0U))
    {
        return;
    }

    /* 查找插值上下界 */
    lower = PhaseCalibration_FindLower(frequency_hz, adc_mode);
    upper = PhaseCalibration_FindUpper(frequency_hz, adc_mode);
    if (lower == NULL)
    {
        lower = upper;
    }
    if (upper == NULL)
    {
        upper = lower;
    }
    if ((lower == NULL) || (upper == NULL))
    {
        return;  /* 无有效校准点，不做修正 */
    }

    /* 计算插值比例（若上下界相同则比值为0） */
    if (upper->frequency_hz == lower->frequency_hz)
    {
        ratio = 0.0f;
    }
    else
    {
        ratio = (float)(frequency_hz - lower->frequency_hz) /
                (float)(upper->frequency_hz -
                        lower->frequency_hz);
    }

    /* 线性插值获取相位误差和增益误差 */
    phase_delta = PhaseCalibration_NormalizeDegrees(
        upper->phase_error_deg - lower->phase_error_deg);
    phase_error = PhaseCalibration_NormalizeDegrees(
        lower->phase_error_deg + ratio * phase_delta);
    gain_error = lower->gain_error_db +
                 ratio * (upper->gain_error_db -
                          lower->gain_error_db);

    /* 修正输入值：减去估计的误差 */
    *phase_deg = PhaseCalibration_NormalizeDegrees(
        *phase_deg - phase_error);
    *gain_db -= gain_error;
}

/**
 * @brief 获取当前校准状态
 * @param status 输出状态结构指针
 */
void PhaseCalibration_GetStatus(PhaseCalibrationStatus_t *status)
{
    if (status != NULL)
    {
        *status = s_calibration.status;
    }
}
