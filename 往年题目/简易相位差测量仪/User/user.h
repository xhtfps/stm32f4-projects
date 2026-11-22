/**
 * @file    user.h
 * @brief   相位差测量仪用户界面（UI）头文件
 *
 * 定义了 UI 命令类型、校准项、测量结果统一结构体以及界面相关的常量、
 * 字符串资源和 API 函数声明。所有界面交互均通过此接口与底层硬件隔离。
 */

#ifndef USER_H
#define USER_H

#include <stdbool.h>
#include <stdint.h>

#include "./BSP/PS2/ps2.h"    /* PS/2 键盘键值定义 */

/* ===================================================================
 *                           系统信息与菜单常量
 * =================================================================== */

#define TitleLength   5U
/* 标题字符串 "相位差测量仪"（GBK 编码） */
#define TitleStr      "\xCF\xE0\xCE\xBB\xB2\xEE\xB2\xE2\xC1\xBF\xD2\xC7"

/* 左下角硬件型号与固件版本（ASCII） */
#define ModelVerStr   "F4 10Hz-10MHz"
#define UserVerStr    "V1.1"

/* 主菜单项数量 */
#define MenuChoiceNum 5U

/* 各菜单项中文名称（GBK 编码）：
 * 1. 频率测量  2. 相位差测量  3. ADC/FFT 测量
 * 4. 学习校准    5. 接线自检 */
#define Menu1Choice1  "1 \xC6\xB5\xC2\xCA\xB2\xE2\xC1\xBF"
#define Menu1Choice2  "2 \xCF\xE0\xCE\xBB\xB2\xEE\xB2\xE2\xC1\xBF"
#define Menu1Choice3  "3 ADC/FFT\xB2\xE2\xC1\xBF"
#define Menu1Choice4  "4 \xD1\xA7\xCF\xB0\xD0\xA3\xD7\xBC"
#define Menu1Choice5  "5 \xD7\xD4\xBC\xEC\xBA\xCF\xB8\xF1"

/* ===================================================================
 *                        用户可调参数范围定义
 * =================================================================== */

#define UI_MENU_COUNT              5U       /* 菜单项总数 */
#define UI_DEFAULT_FREQUENCY_HZ    1000U     /* 默认频率 1 kHz */
#define UI_MIN_FREQUENCY_HZ        10U       /* 最低频率 10 Hz */
#define UI_MAX_FREQUENCY_HZ       10000000U  /* 最高频率 10 MHz */
#define UI_KNOWN_MAX_FREQUENCY_HZ 10000000U  /* 已知模型最高频率 */
#define UI_KNOWN_FREQUENCY_STEP_HZ 100U      /* 已知模型频率步进 100 Hz */
#define UI_DEFAULT_AMPLITUDE_MV    250.0f    /* 默认 DDS 幅度 250 mVpp */
#define UI_MAX_AMPLITUDE_MV        500.0f    /* 最大 DDS 幅度 500 mVpp */
#define UI_DEFAULT_PHASE_DEG       0U        /* 默认相位差 0° */
#define UI_DEFAULT_KNOWN_TARGET_PEAK_V 0.0f  /* 已知模型目标峰值电压默认 0 V */
#define UI_KNOWN_TARGET_MIN_VPP    0.0f      /* 目标 Vpp 最小值 */
#define UI_KNOWN_TARGET_MAX_VPP    0.0f      /* 目标 Vpp 最大值（无效占位） */
#define UI_KNOWN_TARGET_STEP_VPP   0.1f      /* 目标 Vpp 步进 */

/* ===================================================================
 *                             枚举类型定义
 * =================================================================== */

/**
 * @brief UI 命令类型枚举
 *
 * 由界面按键处理函数产生，通过命令队列传递给仪器任务执行。
 */
typedef enum
{
    UI_COMMAND_NONE = 0,                    /* 无有效命令 */
    UI_COMMAND_START_FREQUENCY_MEASUREMENT, /* 启动频率测量 */
    UI_COMMAND_SET_SINE,                    /* 设置 DDS 输出正弦波 */
    UI_COMMAND_STOP_OUTPUT,                 /* 停止所有输出 */
    UI_COMMAND_RUN_KNOWN_MODEL,             /* 运行已知模型相位测量 */
    UI_COMMAND_START_UNKNOWN_MODEL,         /* 运行未知模型 FFT 分析 */
    UI_COMMAND_START_LEARNING,              /* 启动硬件误差学习 */
    UI_COMMAND_START_WIRING_TEST,           /* 启动全链路接线自检 */
    UI_COMMAND_START_INFERENCE,             /* 旧 VOFA 命令（由既有 ADC 菜单控制） */
    UI_COMMAND_TEST_ADS1256,                /* ADS1256 测试（保留） */
    UI_COMMAND_RUN_CALIBRATION              /* 运行校准流程（保留） */
} UI_CommandType_t;

/**
 * @brief 校准项枚举（保留，部分未实现）
 */
typedef enum
{
    UI_CALIBRATION_ADS_REFERENCE = 1U, /* ADS1256 参考电压校准 */
    UI_CALIBRATION_ZERO_OFFSET,        /* 零点偏移校准 */
    UI_CALIBRATION_CHANNEL_GAIN,       /* 通道增益校准 */
    UI_CALIBRATION_SWEEP,              /* 扫频校准 */
    UI_CALIBRATION_IIR_DEBUG,          /* IIR 滤波器调试 */
    UI_CALIBRATION_DAC_TEST            /* DAC 测试 */
} UI_CalibrationItem_t;

/* ===================================================================
 *                             命令结构体
 * =================================================================== */

/**
 * @brief UI 命令结构体
 *
 * 包含命令类型及其携带的参数，通过队列传递到测量任务。
 */
typedef struct
{
    UI_CommandType_t type;          /* 命令类型 */
    uint32_t frequency_hz;          /* 频率参数（Hz） */
    float amplitude_mv;             /* 幅度参数（mVpp） */
    uint32_t phase_deg;             /* 相位差参数（°） */
    float target_output_peak_v;     /* 已知模型目标峰值电压（V） */
    uint8_t calibration_item;       /* 校准项目索引 */
} UI_Command_t;

/* ===================================================================
 *                        测量事件与统一载体
 * =================================================================== */

/**
 * @brief 测量事件类型枚举
 *
 * 指示 UI_Measurement_t 结构体中有效数据的种类，用于界面选择性刷新。
 */
typedef enum
{
    UI_EVENT_NONE = 0U,             /* 无事件 */
    UI_EVENT_MEASUREMENT,           /* 常规测量数据更新 */
    UI_EVENT_SWEEP_PROGRESS,        /* 扫频进度更新 */
    UI_EVENT_MODEL_RESULT,          /* 模型计算结果 */
    UI_EVENT_STATUS,                /* 状态/信息文本更新 */
    UI_EVENT_ADS_TEST               /* ADS1256 测试数据 */
} UI_EventType_t;

/** @brief 电路模型类型（占位，未实际使用） */
typedef uint8_t CircuitModelType_t;

/**
 * @brief 统一测量结果 / 状态结构体
 *
 * 承载所有测量模式下产生的数据，通过发布回调推送到 UI。
 * 由于历史原因，内部包含了频率/相位测量字段、ADS1256 测试字段等，
 * 不同模式下只会使用其中的一个子集。
 */
typedef struct
{
    UI_EventType_t event;             /* 当前事件类型 */

    /* ---------- 原始工程遗留字段（部分未使用） ---------- */
    uint32_t frequency_hz;            /* 测量频率（整数 Hz） */
    float input_rms;                  /* 输入 RMS 值（预留） */
    float output_rms;                 /* 输出 RMS 值（预留） */
    float gain_db;                    /* 增益 dB（预留） */
    float expected_output_rms;        /* 期望输出 RMS（预留） */
    float source_amplitude_mv;        /* 信号源幅度 mV（预留） */
    float target_output_peak_v;       /* 目标峰值电压（预留） */
    float output_peak_v;              /* 实际峰值电压（预留） */
    float expected_output_peak_v;     /* 期望峰值电压（预留） */
    uint16_t sweep_index;             /* 扫频当前步进序号 */
    uint16_t sweep_total;             /* 扫频总步数 */
    CircuitModelType_t model_type;    /* 电路模型类型（预留） */
    float f1_hz;                      /* 模型参数 f1 */
    float f2_hz;                      /* 模型参数 f2 */
    float center_hz;                  /* 中心频率 */
    float q;                          /* Q 值 */
    uint8_t valid;                    /* 通用有效标志（预留） */

    /* ---------- ADS1256 测试相关字段（预留） ---------- */
    uint8_t ads_init_ok;
    uint8_t ads_drdy_ready;
    uint8_t ads_pdwn_high;
    uint8_t ads_status_reg;
    uint8_t ads_mux_reg;
    uint8_t ads_adcon_reg;
    uint8_t ads_drate_reg;
    uint8_t ads_reg_read_ok;
    uint8_t ads_ain0_samples_ok;
    uint8_t ads_ain1_samples_ok;
    uint8_t ads_test_repeat_count;
    int32_t ads_ain0_raw;
    int32_t ads_ain1_raw;
    int32_t ads_ain0_min_raw;
    int32_t ads_ain0_max_raw;
    int32_t ads_ain1_min_raw;
    int32_t ads_ain1_max_raw;
    float ads_ain0_voltage;
    float ads_ain1_voltage;

    /* ---------- 相位/频率测量扩展字段 ---------- */
    uint32_t divided_frequency_hz;          /* 分频后的频率（Hz） */
    float frequency_hz_precise;             /* 高精度频率（浮点 Hz） */
    float divided_frequency_hz_precise;     /* 分频后高精度频率 */
    uint32_t period_ticks;                  /* 测得的周期（定时器计数值） */
    uint32_t capture_cycles;                /* 捕获周期数（边沿数） */
    uint32_t divider;                       /* 外部预分频系数 */
    uint32_t adc_overrun_count;             /* ADC DMA 溢出次数 */
    uint32_t adc_sample_rate_hz;             /* ADC 实际采样率 */
    uint32_t local_oscillator_hz;            /* AD9959 混频本振频率 */
    uint16_t selected_bin;                  /* FFT 选中的频率 bin */
    float channel1_amplitude;               /* 通道 1 幅度（归一化） */
    float channel2_amplitude;               /* 通道 2 幅度（归一化） */
    float phase_difference_deg;             /* 相位差（°） */
    uint8_t frequency_valid;                /* 频率测量有效标志 */
    uint8_t phase_valid;                    /* 相位测量有效标志 */
    uint8_t capture_route;                  /* 0=OUT1/IN1, 1=OUT1/IN2 */
    uint8_t adc_mode;                       /* 0=直采, 1=混频 */
    uint8_t measure_state;                  /* 测量状态机状态 */
    uint8_t active_menu;                    /* 当前仪器菜单编号 */
    uint8_t first_frame_discarded;          /* 继电器/DDS 后首帧已丢弃 */
    uint8_t calibration_valid;              /* 已加载有效校准表 */
    uint8_t learning_active;                /* 学习流程正在运行 */
    uint8_t learning_save_ok;               /* 最近一次学习表保存成功 */
    uint8_t learning_point_index;           /* 当前学习点，0 起 */
    uint8_t learning_point_total;           /* 学习点总数 */
    uint8_t learning_sample_count;          /* 当前点已接受样本数 */
    uint32_t learning_target_frequency_hz;  /* 当前学习目标频率 */
    float learned_phase_error_deg;          /* 最近完成点的相位误差 */
    float learned_gain_error_db;            /* 最近完成点的增益误差 */
    uint8_t wiring_active;                  /* 接线自检正在运行 */
    uint8_t wiring_case_index;              /* 当前自检项，0 起 */
    uint8_t wiring_case_total;              /* 自检项总数 */
    uint8_t wiring_pass_count;              /* 已通过项目数 */
    uint8_t wiring_fail_count;              /* 已失败项目数 */
    uint8_t wiring_done;                    /* 自检已结束 */
    uint8_t status_code;                    /* 状态码（0 异常，1 正常） */
    char status_text[32];                   /* 状态描述字符串 */
} UI_Measurement_t;

/* ===================================================================
 *                    旧版/兼容函数声明
 * =================================================================== */

void User_main(void);             /* 旧版裸机主循环，使用 FreeRTOS 后不再调用 */
void Init_All(void);              /* 全局初始化（等同于 UI_Init） */
void Disp_Main(void);             /* 强制返回主菜单并重绘 */
void Show_Val(uint8_t location, float value, char *format); /* 在固定位置显示数值 */
void Change_Menu(uint8_t menu_sign); /* 按编号切换菜单页面 */
void MenuHaddler_1(void);         /* 菜单 1 回调 */
void MenuHaddler_2(void);         /* 菜单 2 回调 */
void MenuHaddler_3(void);         /* 菜单 3 回调 */
void MenuHaddler_4(void);         /* 菜单 4 回调 */
void MenuHaddler_5(void);         /* 菜单 5 回调 */
float PS2_ReadNum(float num);    /* PS/2 数字输入解析（占位） */

/* ===================================================================
 *                   FreeRTOS 架构下核心 UI 接口
 * =================================================================== */

/**
 * @brief  UI 初始化：清屏、绘制框架与主菜单
 */
void UI_Init(void);

/**
 * @brief  设置中文字库是否可用，影响字符串显示方式
 * @param  available 非 0 表示字库可用，0 表示使用 ASCII 回退
 */
void UI_SetChineseFontAvailable(uint8_t available);

/**
 * @brief  处理按键输入，根据当前页面状态生成 UI 命令
 * @param  key     按键值
 * @param  command 输出命令结构体指针
 * @return true: 生成了有效命令；false: 无命令
 */
bool UI_HandleKey(PS2_KeyValue_t key, UI_Command_t *command);

/**
 * @brief  更新界面上的测量/状态数据
 * @param  measurement 指向最新测量结果结构体的指针
 */
void UI_UpdateMeasurement(const UI_Measurement_t *measurement);

#endif /* USER_H */
