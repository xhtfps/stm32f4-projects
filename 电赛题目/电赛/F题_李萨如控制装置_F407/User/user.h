/**
 * @file    user.h
 * @brief   F 题李萨如图形显示控制装置用户界面头文件
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

#define TitleLength   7U
/* 标题字符串“李萨如控制装置”（GBK 编码）。 */
#define TitleStr      "\xC0\xEE\xC8\xF8\xC8\xE7\xBF\xD8\xD6\xC6\xD7\xB0\xD6\xC3"

/* 左下角硬件型号与固件版本（ASCII） */
#define ModelVerStr   "F407 + K230"
#define UserVerStr    "F-FRAME V0.1"

/* 主菜单项数量 */
#define MenuChoiceNum 4U

/*
 * 六个顶层菜单依次对应六项功能：
 * 1=对角直线，2=正交圆形，3=无穷图形，
 * 4=幅值档位，5=自动控制，6=系统调试。
 */
#define Menu1Choice1  "\xB2\xA8\xD0\xCE\xBF\xD8\xD6\xC6" /* 波形控制 */
#define Menu1Choice2  "\xB7\xF9\xD6\xB5\xBF\xD8\xD6\xC6" /* 幅值控制 */
#define Menu1Choice3  "\xD7\xD4\xB6\xAF\xBF\xD8\xD6\xC6" /* 自动控制 */
#define Menu1Choice4  "\xCF\xB5\xCD\xB3\xB5\xF7\xCA\xD4" /* 系统调试 */

/* ===================================================================
 *                        用户可调参数范围定义
 * =================================================================== */

#define UI_MENU_COUNT              4U       /* 菜单项总数 */
#define UI_DEFAULT_FREQUENCY_HZ    1000U     /* 默认频率 1 kHz */
#define UI_MIN_FREQUENCY_HZ        1000U     /* 题目输入最低频率 1 kHz */
#define UI_MAX_FREQUENCY_HZ       100000U    /* 题目输入最高频率 100 kHz */
#define UI_KNOWN_MAX_FREQUENCY_HZ 200000U    /* 无穷形 DDS 最大 2 倍频 */
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
    UI_COMMAND_NONE = 0,             /* 无有效命令 */
    UI_COMMAND_START_WAVEFORM_CONTROL,  /* 0/1/3选择三种有线波形 */
    UI_COMMAND_START_AMPLITUDE_CONTROL, /* 三种波形加固定Y轴幅值 */
    UI_COMMAND_UPDATE_AMPLITUDE_TARGET, /* 运行中更新2/4/6/8div */
    UI_COMMAND_START_AUTOMATIC_CONTROL, /* 断线：K230视觉自动模式 */
    UI_COMMAND_START_DEBUG,          /* 测频、双 ADC、FFT 调试 */
    UI_COMMAND_START_AD9959_TEST,    /* 菜单6：启动AD9959 CH0单频测试 */
    UI_COMMAND_STOP_AD9959_TEST,     /* 菜单6：关闭AD9959测试输出 */
    UI_COMMAND_AD9959_ADJUST_FTW,    /* 菜单6：原始FTW加减1 */
    UI_COMMAND_AD9959_ADJUST_DITHER, /* 菜单6：相邻FTW混合比例加减1级 */
    UI_COMMAND_AD9959_TOGGLE_DITHER, /* 菜单6：开关相邻FTW抖动 */
    UI_COMMAND_TOGGLE_COMPLETE_OUTPUT, /* 菜单3：翻转PG0声光完成测试输出 */
    UI_COMMAND_STOP_OUTPUT           /* 停止测量/控制并回到空闲 */
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
    uint8_t target_span_y_div;       /* 要求4目标：2/4/6/8 div */
    uint8_t target_shape;            /* 要求5目标：UI_TargetShape_t */
    uint8_t calibration_item;       /* 校准项目索引 */
    int8_t dds_fine_step;            /* 菜单6精调方向：-1或+1 */
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
    UI_EVENT_MEASUREMENT,           /* 定时器测频数据更新 */
    UI_EVENT_CONTROL_PROGRESS,      /* 自动控制状态更新 */
    UI_EVENT_VISION_MEASUREMENT,    /* K230 视觉数据更新 */
    UI_EVENT_STATUS                 /* 状态/信息文本更新 */
} UI_EventType_t;

typedef enum
{
    UI_TARGET_SHAPE_NONE = 0U,
    UI_TARGET_SHAPE_DIAGONAL,
    UI_TARGET_SHAPE_ELLIPSE,
    UI_TARGET_SHAPE_INFINITY
} UI_TargetShape_t;

typedef enum
{
    UI_APP_MODE_IDLE = 0U,
    UI_APP_MODE_WAVEFORM_CONTROL,
    UI_APP_MODE_AMPLITUDE_CONTROL,
    UI_APP_MODE_AUTOMATIC_CONTROL,
    UI_APP_MODE_DEBUG
} UI_AppMode_t;

/**
 * @brief 与 read.md 第 14 节相对应的主控状态。
 *
 * 有线模式使用 IDLE/WIRED_*，菜单3使用 WAIT_VISION 之后的视觉闭环
 * 状态；这些枚举值是 UI 与控制器之间的稳定接口。
 */
typedef enum
{
    UI_CONTROL_STATE_IDLE = 0U,
    UI_CONTROL_STATE_WIRED_WAIT_FREQUENCY,
    UI_CONTROL_STATE_WIRED_MEASUREMENT,
    UI_CONTROL_STATE_WAIT_VISION,
    UI_CONTROL_STATE_COARSE_SWEEP_1X,
    UI_CONTROL_STATE_LOCAL_SWEEP_1X,
    UI_CONTROL_STATE_FREQUENCY_LOCK_1X,
    UI_CONTROL_STATE_SWITCH_TO_2X,
    UI_CONTROL_STATE_AMPLITUDE_CONTROL,
    UI_CONTROL_STATE_PHASE_CONTROL,
    UI_CONTROL_STATE_STRICT_VERIFY,
    UI_CONTROL_STATE_HOLD,
    UI_CONTROL_STATE_ERROR
} UI_ControlState_t;

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

    /* ---------- F 题新增：定时器测频 ---------- */
    float frequency_jitter_ppm;             /* 窗口峰峰抖动 ppm */
    uint32_t frequency_sample_count;        /* 已接受测频样本数 */
    uint32_t frequency_100hz;               /* 100 Hz 为单位的频点序号 */
    uint32_t signal_age_ms;                 /* 距最后有效边沿时间 */
    uint32_t capture_dma_error_count;       /* 捕获 DMA 错误计数 */
    float feedback_frequency_hz_precise;    /* PB11输出反馈精确频率，不量化 */
    float feedback_frequency_error_hz;      /* fout-N*fin，N由当前要求决定 */
    uint32_t feedback_period_ticks;         /* PB11累计TIM2计数 */
    uint32_t feedback_capture_cycles;       /* PB11累计输出周期数 */
    uint32_t feedback_signal_age_ms;        /* PB11距最后有效边沿时间 */
    uint32_t feedback_dma_error_count;      /* PB11捕获DMA错误计数 */
    uint8_t feedback_frequency_valid;       /* PB11反馈频率有效 */

    /* ---------- F 题新增：控制与视觉接口 ---------- */
    uint8_t target_shape;                   /* UI_TargetShape_t */
    uint8_t control_state;                  /* UI_ControlState_t */
    uint8_t vision_online;                  /* K230 链路在线 */
    uint8_t vision_valid;                   /* 最新视觉测量有效 */
    uint8_t dds_ready;                      /* AD9959 通信层已初始化 */
    uint8_t dds_channel;                    /* 当前DDS输出通道，0表示CH0 */
    float vision_confidence;
    float vision_span_x_div;
    float vision_span_y_div;
    float vision_center_x_div;
    float vision_center_y_div;
    float vision_phase_error_deg;          /* K230给出的目标相位误差 */
    float vision_phase_rate_deg_s;         /* 相位误差斜率，用于光学DPLL */
    uint16_t vision_probe_id;              /* 当前探测点编号 */
    uint8_t vision_ratio_y;                /* 约分后的fy:fx分子 */
    uint8_t vision_ratio_x;                /* 约分后的fy:fx分母 */
    float target_span_x_div;
    float target_span_y_div;
    uint32_t dds_frequency_hz;
    uint32_t dds_frequency_millihz;        /* 有线PI输出的精确DDS频率命令 */
    float dds_amplitude_mv;
    uint16_t dds_phase_deg;
    uint32_t dds_base_ftw;                 /* 菜单6精调使用的低端FTW */
    uint8_t dds_dither_numerator;           /* 相邻FTW混合分子，分母固定16 */
    uint8_t dds_dither_enabled;             /* 相邻FTW抖动已启用 */
    uint8_t dds_test_active;                /* 菜单6独立DDS测试正在运行 */
    uint32_t control_elapsed_ms;
    float frequency_pi_correction_hz;       /* 频率粗锁PI修正量 */
    float phase_pi_correction_hz;           /* 相位DPLL-PI产生的频率修正量 */
    float amplitude_pi_correction_mv;       /* 幅值PI产生的DDS幅度修正量 */
    uint8_t wired_control_locked;            /* 要求1~4闭环已进入HOLD */

    /* ---------- F 题有线测量：PA1 输入、PA2 输出 ---------- */
    uint32_t input_vpp_mv;             /* 输入端实际峰峰值，单位 mV */
    uint32_t output_vpp_mv;            /* OPA1612 后输出峰峰值，单位 mV */
    int32_t amplitude_error_mv;        /* 输出减输入，单位 mV */
    uint32_t input_bias_mv;            /* PA1 直流偏置，单位 mV */
    uint32_t output_bias_mv;           /* PA2 直流偏置，单位 mV */
    uint16_t output_min_count;         /* PA2 本块最小 ADC 码值 */
    uint16_t output_max_count;         /* PA2 本块最大 ADC 码值 */
    float input_phase_deg;             /* 输入在 fin 上的相位 */
    float output_phase_deg;            /* 输出在 fout 上的相位 */
    float phase_relation_deg;          /* 要求2: φy-φx；要求3: φy-2φx */
    float phase_error_deg;             /* 相对目标相位的误差 */
    uint32_t wired_block_count;        /* 已处理的双 ADC 数据块数 */
    uint8_t amplitude_valid;           /* 两路幅值有效 */
    uint8_t wired_phase_valid;         /* 有线相位关系有效 */
    uint8_t adc_clipped;               /* 任一路 ADC 接近上下电源轨 */
    uint8_t amplitude_target_div;      /* 当前要求4目标档位 */
    uint32_t adc_rejected_block_count;
    float crossing_phase_relation_deg;
    float phase_estimator_difference_deg;
    uint8_t phase_agreement_count;
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
void MenuHaddler_6(void);         /* 菜单 6 回调 */
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
