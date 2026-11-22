#ifndef USER_H
#define USER_H

/**
 * @file    user.h
 * @brief   电路模型测量系统 UI 层头文件
 *
 * 包含界面宏定义、命令结构体、测量数据结构体及对外函数声明
 * FreeRTOS 任务接口也在此声明
 */

#include <stdbool.h>
#include <stdint.h>

#include "./BSP/PS2/ps2.h"
#include "./BSP/CIRCUIT/circuit_model.h"

/* ================================================================
 * 菜单与标题常量定义
 * ================================================================ */

/** 标题字符长度 GBK 编码汉字数 */
#define TitleLength   5U
/** 系统标题字符串 */
#define TitleStr      "\xB5\xE7\xC2\xB7\xC4\xA3\xD0\xCD\xB2\xE2\xC1\xBF\xCF\xB5\xCD\xB3"
/** 硬件版本字符串 */
#define ModelVerStr   "F4 AD9959/ADS1256"
/** 软件版本字符串 */
#define UserVerStr    "V1.0"
/** 菜单项总数 */
#define MenuChoiceNum 5U
/** 菜单第 1 项文本 */
#define Menu1Choice1  "1 \xD5\xFD\xCF\xD2\xD0\xC5\xBA\xC5\xD4\xB4"
/** 菜单第 2 项文本 */
#define Menu1Choice2  "2 \xD2\xD1\xD6\xAA\xB5\xE7\xC2\xB7\xB2\xE2\xCA\xD4"
/** 菜单第 3 项文本 */
#define Menu1Choice3  "3 \xCE\xB4\xD6\xAA\xB5\xE7\xC2\xB7\xB1\xE6\xCA\xB6"
/** 菜单第 4 项文本 */
#define Menu1Choice4  "4 IIR\xC2\xCB\xB2\xA8\xCA\xE4\xB3\xF6"
/** 菜单第 5 项文本 */
#define Menu1Choice5  "5 ADS1256\xB5\xF7\xCA\xD4"

/* ================================================================
 * 参数默认值与范围
 * ================================================================ */

/** 菜单项总数 与 MenuChoiceNum 一致 */
#define UI_MENU_COUNT             5U
/** 默认输出频率 单位 Hz */
#define UI_DEFAULT_FREQUENCY_HZ   1000U
#define UI_MIN_FREQUENCY_HZ       100U
#define UI_MAX_FREQUENCY_HZ       1000000U
#define UI_KNOWN_MAX_FREQUENCY_HZ 3000U
#define UI_KNOWN_FREQUENCY_STEP_HZ 100U
/** 默认输出幅值 单位 mV */
#define UI_DEFAULT_AMPLITUDE_MV   125.0f
/** 最大输出幅值 单位 mV */
#define UI_MAX_AMPLITUDE_MV       500.0f
/** 默认初始相位 单位 度 */
#define UI_DEFAULT_PHASE_DEG      0U
/** 菜单2默认的已知模型目标输出峰峰值（旧宏名为兼容保留） */
#define UI_DEFAULT_KNOWN_TARGET_PEAK_V 2.0f
#define UI_KNOWN_TARGET_MIN_VPP   1.0f
#define UI_KNOWN_TARGET_MAX_VPP   2.0f
#define UI_KNOWN_TARGET_STEP_VPP  0.1f

/* ================================================================
 * UI 命令类型定义
 * ================================================================ */

/** UI 层按键解析后产生的底层执行命令类型 */
typedef enum
{
    UI_COMMAND_NONE = 0,               /**< 无有效命令 */
    UI_COMMAND_SET_SINE,               /**< 设置正弦信号输出参数 */
    UI_COMMAND_STOP_OUTPUT,            /**< 停止信号输出 */
    UI_COMMAND_RUN_KNOWN_MODEL,        /**< 运行已知电路测量 */
    UI_COMMAND_START_UNKNOWN_MODEL,    /**< 启动未知电路辨识扫频 */
    UI_COMMAND_START_INFERENCE,        /**< 启动 IIR 滤波推理 */
    UI_COMMAND_TEST_ADS1256,           /**< 兼容旧版 ADS1256 测试命令 */
    UI_COMMAND_RUN_CALIBRATION         /**< 运行菜单5校准测试 */
} UI_CommandType_t;

/** 菜单5四类串口校准测试 */
typedef enum
{
    UI_CALIBRATION_ADS_REFERENCE = 1U, /**< ADS参考/原始码测试 */
    UI_CALIBRATION_ZERO_OFFSET,        /**< 完整链路零点偏置测试 */
    UI_CALIBRATION_CHANNEL_GAIN,       /**< 输入输出通道一致性测试 */
    UI_CALIBRATION_SWEEP,              /**< Full circuit sweep test */
    UI_CALIBRATION_IIR_DEBUG,          /**< Menu4 IIR inference debug */
    UI_CALIBRATION_DAC_TEST            /**< DAC1/PA4 standalone output test */
} UI_CalibrationItem_t;

/**
 * @brief UI命令结构体
 *
 * 封装 UI 层按键解析后输出的完整执行参数
 */
typedef struct
{
    UI_CommandType_t type;    /**< 命令类型 */
    uint32_t frequency_hz;    /**< 输出频率参数 (Hz) */
    float amplitude_mv;       /**< 输出幅值参数 (mV) */
    uint32_t phase_deg;       /**< 输出相位参数 (度)  */
    float target_output_peak_v;/**< 菜单2目标输出Vpp (V)，旧字段名兼容保留 */
    uint8_t calibration_item; /**< 菜单5校准项目编号 */
} UI_Command_t;

/* ================================================================
 * 测量数据事件类型
 * ================================================================ */

/** 底层测量模块上报给 UI 层的事件类型 */
typedef enum
{
    UI_EVENT_NONE = 0U,           /**< 无事件 */
    UI_EVENT_MEASUREMENT,         /**< 单次测量数据更新 */
    UI_EVENT_SWEEP_PROGRESS,      /**< 扫频过程中的进度与数据更新 */
    UI_EVENT_MODEL_RESULT,        /**< 未知电路辨识完成参数结果 */
    UI_EVENT_STATUS,              /**< 滤波输出状态更新 */
    UI_EVENT_ADS_TEST             /**< ADS1256 测试数据更新 */
} UI_EventType_t;

/**
 * @brief 测量数据结构体
 *
 * 底层测量模块 / 辨识模块向 UI 层上报的统一数据结构
 * 根据 event 字段判断当前哪些参数有效
 */
typedef struct
{
    UI_EventType_t event;           /**< 当前数据事件类型 */

    /* ---------- 通用测量字段 ---------- */
    uint32_t frequency_hz;          /**< 当前频率 (Hz) */
    float input_rms;                /**< 输入电压有效值 (V) */
    float output_rms;               /**< 输出电压有效值 (V) */
    float gain_db;                  /**< 电路增益 (dB) */
    float expected_output_rms;      /**< 理论输出有效值 (V) */
    float source_amplitude_mv;      /**< 信号源输出幅值 AD9959 设置 (mV) */
    float target_output_peak_v;     /**< 菜单2目标输出Vpp (V)，旧字段名兼容保留 */
    float output_peak_v;            /**< 菜单2实测输出Vpp (V)，旧字段名兼容保留 */
    float expected_output_peak_v;   /**< 菜单2模型输出Vpp (V)，旧字段名兼容保留 */

    /* ---------- 扫频相关字段 ---------- */
    uint16_t sweep_index;           /**< 当前扫频点索引 */
    uint16_t sweep_total;           /**< 扫频总点数 */

    /* ---------- 辨识结果字段 ---------- */
    CircuitModelType_t model_type;  /**< 辨识出的电路类型 */
    float f1_hz;                    /**< 下/截止频率 1 (Hz) */
    float f2_hz;                    /**< 上/截止频率 2 (Hz) */
    float center_hz;                /**< 中心频率 (Hz) */
    float q;                        /**< 品质因数 Q */
    uint8_t valid;                  /**< 辨识结果有效 (0:无效) */

    /* ---------- ADS1256 测试字段 ---------- */
    uint8_t ads_init_ok;            /**< 初始化是否成功 */
    uint8_t ads_drdy_ready;         /**< DRDY 引脚是否就绪 */
    uint8_t ads_pdwn_high;          /**< PDWN 引脚电平 (高有效) */
    uint8_t ads_status_reg;         /**< ADS1256 STATUS 寄存器值 */
    uint8_t ads_mux_reg;            /**< ADS1256 MUX 寄存器值 */
    uint8_t ads_adcon_reg;          /**< ADS1256 ADCON 寄存器值 */
    uint8_t ads_drate_reg;          /**< ADS1256 DRATE 寄存器值 */
    uint8_t ads_reg_read_ok;        /**< ADS1256寄存器读取是否全部成功 */
    uint8_t ads_ain0_samples_ok;    /**< AIN0重复采样成功次数 */
    uint8_t ads_ain1_samples_ok;    /**< AIN1重复采样成功次数 */
    uint8_t ads_test_repeat_count;  /**< 菜单5重复采样总次数 */
    int32_t ads_ain0_raw;           /**< AIN0 通道原始 ADC 码 */
    int32_t ads_ain1_raw;           /**< AIN1 通道原始 ADC 码 */
    int32_t ads_ain0_min_raw;      /**< AIN0重复采样最小值 */
    int32_t ads_ain0_max_raw;      /**< AIN0重复采样最大值 */
    int32_t ads_ain1_min_raw;      /**< AIN1重复采样最小值 */
    int32_t ads_ain1_max_raw;      /**< AIN1重复采样最大值 */
    float ads_ain0_voltage;         /**< AIN0 换算电压 (V) */
    float ads_ain1_voltage;         /**< AIN1 换算电压 (V) */


/* ================================================================
 * 兼容旧版对外接口函数
 * 这些函数为旧框架保留，新代码推荐使用下方UI_系列接口
 * ================================================================ */

} UI_Measurement_t;

void User_main(void);                  /**< 主循环入口 */
void Init_All(void);                   /**< 全局初始化 */
void Disp_Main(void);                  /**< 显示主界面 */
void Show_Val(uint8_t location, float value, char *format);  /**< 指定位置显示数值 */
void Change_Menu(uint8_t menu_sign);   /**< 外部直接切换菜单项 */
void MenuHaddler_1(void);              /**< 菜单 1 处理 */
void MenuHaddler_2(void);              /**< 菜单 2 处理 */
void MenuHaddler_3(void);              /**< 菜单 3 处理 */
void MenuHaddler_4(void);              /**< 菜单 4 处理 */
void MenuHaddler_5(void);              /**< 菜单 5 处理 */
float PS2_ReadNum(float num);          /**< 预留PS2数值读取接口 */

/* ================================================================
 * FreeRTOS UI 层标准接口
 * 在 freertos_demo.c 中被任务调度调用
 * ================================================================ */

/**
 * @brief  初始化 UI 界面
 *
 * 重置所有状态变量并绘制首页框架与菜单
 */
void UI_Init(void);

/**
 * @brief  设置汉字字库是否可用
 * @param  available 非零表示启用GBK中文显示，零则强制英文界面
 */
void UI_SetChineseFontAvailable(uint8_t available);

/**
 * @brief  处理 PS/2 按键事件并输出命令
 * @param  key     按键值
 * @param  command 输出解析后的命令结构体
 * @return true    产生了有效执行命令
 *         false   未产生命令（仅页面切换等）
 */
bool UI_HandleKey(PS2_KeyValue_t key, UI_Command_t *command);

/**
 * @brief  更新界面测量显示数据
 * @param  measurement 上报的测量/辨识数据结构体
 *
 * 内部根据 measurement->event 字段判断是否刷新对应页面
 */
void UI_UpdateMeasurement(const UI_Measurement_t *measurement);

#endif /* USER_H */
