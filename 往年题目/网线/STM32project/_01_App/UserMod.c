#include "User.h"
#include "EthMeasureCore.h"
#include <string.h>

/*==================== 设备及版本信息 ====================*/
#define TITLE_STR       "电缆测量仪"
#define MODEL_VER_STR   "型号: DDR"
#define USER_VER_STR    "用户版本: M0"

/*==================== 主菜单选项文本 ====================*/
#define MENU1_CHOICE1   "1. 单端测量"
#define MENU1_CHOICE2   "2. 参数设置"
#define MENU1_CHOICE3   "3. 校准"
#define MENU1_CHOICE4   "4. 状态"
#define MENU_CHOICE_NUM 4U            /* 菜单项总数 */

/* 各功能界面标题 */
#define TITLE_MEASURE   "单端测量"
#define TITLE_PARAMS    "参数设置"
#define TITLE_CALIB     "校准"
#define TITLE_STATUS    "状态"

/*==================== 底部操作提示 ====================*/
#define TIP_MEASURE_1   "按Enter开始测量，按Back返回"
#define TIP_MEASURE_2   "DDS扫频 + IFFT频域反射测量"
#define TIP_PARAMS_1    "按Enter切换选择项，按+/-调整数值"
#define TIP_PARAMS_2    "参数范围内调整，校准可修正NVP"
#define TIP_CAL_1       "Enter:NVP校准  +:开路  -:短路"
#define TIP_CAL_2       "*:100欧负载  SOL校准"      /* 已修复乱码 */
#define TIP_STATUS_1    "按Enter触发一次ADC采样检测"
#define TIP_STATUS_2    "显示DDS和ADC状态"

/*==================== 测量界面标签 ====================*/
#define L_SWEEP_COUNT   "扫描次数"
#define L_LENGTH_M      "测量长度(m)"
#define L_RESOLUTION_M  "分辨率(m)"
#define L_START_MHZ     "起始频率(MHz)"
#define L_STOP_MHZ      "终止频率(MHz)"
#define L_STEP_KHZ      "步进(kHz)"
#define L_MESSAGE       "消息"
#define L_DET_V         "检波电压(V)"
#define L_PEAK_MAG      "峰值幅度"

/*==================== 参数设置界面标签 ====================*/
#define L_START_MHZ2    "起始频率 MHz"
#define L_STOP_MHZ2     "终止频率 MHz"
#define L_STEP_KHZ2     "步进 kHz"
#define L_SETTLE_MS     "稳定时间 ms"
#define L_NVP           "速度因子 NVPx1000"
#define L_IFFT_POINTS   "IFFT点数"
#define L_CABLE_REF_CM  "参考长度(cm)"
#define L_CURSOR        "光标(序号)"

/*==================== 校准界面标签 ====================*/
#define L_MODE          "模式"
#define L_REF_CABLE_M   "参考线缆长度(m)"
#define L_CUR_NVP       "当前NVP值"
#define L_MEASURED_M    "测量长度(m)"
#define L_CAL_STATUS    "校准状态"
#define L_PEAK_BIN      "峰值点序号"

/*==================== 状态界面标签 ====================*/
#define L_DDS           "DDS状态"
#define L_ADC           "ADC状态"
#define L_ADC_FRAME     "ADC DMA帧状态"
#define L_CALIBRATION   "校准状态"
#define L_DDS_FREQ_HZ   "DDS当前频率(Hz)"
#define L_REF_V         "参考端IF电压(V)"

/*==================== 状态文本 ====================*/
#define V_SINGLE_END_FDR "单端FDR模式"
#define V_CAL_ARMED      "校准就绪"
#define V_IDLE           "空闲"
#define V_READY          "就绪"
#define V_OFF            "关闭"
#define V_UNCHECKED      "未检测"
#define V_TIMEOUT        "超时"
#define V_ARMED          "已就绪"
#define V_IDLE2          "未校准"

/*==================== UI 参数常量 ====================*/
#define UI_MIN_NVP_X1000        500U    /* NVP最小值 */
#define UI_CALIB_LENGTH_STEP_CM 50U     /* 校准参考长度步进50厘米 */

/* 紧凑型9行Y坐标数组，用于快速定位显示行 */
static const uint16_t k_compact_line_y[9] =
{
    150U, 174U, 198U, 222U, 246U, 270U, 294U, 318U, 342U
};

static uint8_t g_menu_sign = 0U;                    /* 当前菜单标识：0=主菜单，1~4=对应功能 */
static char g_value_cache[9][64] = {{0}};           /* 数值显示缓存，用于防止闪烁 */

/*==================== 内部函数声明 ====================*/
static void Disp_Main(void);                        /* 绘制主菜单界面 */
static void Change_Menu(uint8_t menu_sign);         /* 切换菜单项并显示箭头 */
static void Clear_Work_Area(void);                  /* 清除工作区并初始化缓存 */
static void Draw_Work_Title(const char *title);     /* 绘制工作区标题 */
static void Draw_Key_Tips(const char *tip1, const char *tip2); /* 绘制底部两行按键提示 */
static void Show_Compact_Value_Only(uint8_t line, const char *text); /* 仅更新行数值，无闪烁刷新 */
static void Wait_Ps2KeyRelease(uint8_t key_value);  /* 等待按键释放，防止重复触发 */
static void MenuHandler_Measure(void);              /* 单端测量菜单处理 */
static void MenuHandler_Params(void);               /* 参数设置菜单处理 */
static void MenuHandler_Calibrate(void);            /* 校准菜单处理 */
static void MenuHandler_Status(void);               /* 状态查看菜单处理 */

/**
 * @brief 主循环入口，初始化后进入菜单调度
 */
void User_main(void)
{
    EthApp_Init();          /* 初始化测量核心模块 */
    Disp_Main();            /* 显示主菜单 */

    while(1)
    {
        switch(g_menu_sign)
        {
            case 0U:        /* 主菜单：检测数字键1~4进入对应功能 */
                if((Ps2KeyValue >= KeyValue_1) && (Ps2KeyValue <= KeyValue_4))
                {
                    Change_Menu((uint8_t)(Ps2KeyValue - KeyValue_0));
                }
                break;

            case 1U:        /* 单端测量界面 */
                MenuHandler_Measure();
                break;

            case 2U:        /* 参数设置界面 */
                MenuHandler_Params();
                break;

            case 3U:        /* 校准界面 */
                MenuHandler_Calibrate();
                break;

            case 4U:        /* 状态查看界面 */
                MenuHandler_Status();
                break;

            default:        /* 异常状态强制返回主菜单 */
                g_menu_sign = 0U;
                break;
        }

        delay_ms(10);       /* 10ms调度间隔，降低CPU占用 */
    }
}

/**
 * @brief 绘制主菜单界面：标题、分隔线、选项、版本信息
 */
static void Disp_Main(void)
{
    uint8_t count;

    /* 标题居中 */
    OS_String_Show(240, 16, 32, 1, TITLE_STR);

    /* 上下分隔线与左侧竖线 */
    LCD_Appoint_Clear(0, 64, 800, 72, White);
    LCD_Appoint_Clear(0, 440, 800, 448, White);
    LCD_Appoint_Clear(250, 72, 252, 440, White);

    /* 右侧版本信息 */
    OS_String_Show(32, 456, 16, 1, MODEL_VER_STR);
    OS_String_Show(650, 456, 16, 1, USER_VER_STR);

    /* 初始显示菜单项占位符（无箭头） */
    for(count = 1U; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32U + 64U * count), 32, 1, "-");
    }

    /* 菜单选项文字 */
    OS_String_Show(60, 96, 32, 1, MENU1_CHOICE1);
    OS_String_Show(60, 160, 32, 1, MENU1_CHOICE2);
    OS_String_Show(60, 224, 32, 1, MENU1_CHOICE3);
    OS_String_Show(60, 288, 32, 1, MENU1_CHOICE4);
}

/**
 * @brief 切换菜单项，在选中项前显示箭头">"
 * @param menu_sign 选中项1~4，0表示取消选中
 */
static void Change_Menu(uint8_t menu_sign)
{
    uint8_t count;

    /* 清除所有箭头 */
    for(count = 1U; count <= MENU_CHOICE_NUM; count++)
    {
        OS_String_Show(32, (uint16_t)(32U + 64U * count), 32, 1, "-");
    }

    /* 在选中项前显示箭头 */
    if((menu_sign >= 1U) && (menu_sign <= MENU_CHOICE_NUM))
    {
        OS_String_Show(32, (uint16_t)(32U + 64U * menu_sign), 32, 1, ">");
        g_menu_sign = menu_sign;
    }
    else
    {
        g_menu_sign = 0U;
    }

    Ps2KeyValue = KeyValue_Null;    /* 清除按键状态，防止误触发 */
}

/**
 * @brief 清除右侧工作区，并清空显示缓存
 */
static void Clear_Work_Area(void)
{
    memset(g_value_cache, 0, sizeof(g_value_cache));
    LCD_Appoint_Clear(260, 80, 798, 438, Black);
}

/**
 * @brief 在工作区顶部绘制标题
 * @param title 标题字符串
 */
static void Draw_Work_Title(const char *title)
{
    OS_String_Show(280, 88, 32, 1, (char *)title);
}

/**
 * @brief 在工作区底部绘制两行按键提示
 * @param tip1 第一行提示
 * @param tip2 第二行提示
 */
static void Draw_Key_Tips(const char *tip1, const char *tip2)
{
    OS_String_Show(280, 400, 16, 1, (char *)tip1);
    OS_String_Show(280, 420, 16, 1, (char *)tip2);
}

/**
 * @brief 防闪烁更新单行数值，内容未变时不刷新LCD
 * @param line 行号（0~8）
 * @param text 新数值字符串
 */
static void Show_Compact_Value_Only(uint8_t line, const char *text)
{
    if(line < 9U)
    {
        char display_buf[64];

        /* 格式化为固定宽度，便于比较 */
        sprintf(display_buf, "%-31s", text);
        if(strcmp(g_value_cache[line], display_buf) != 0)
        {
            OS_String_Show(500, k_compact_line_y[line], 24, 1, display_buf);
            strcpy(g_value_cache[line], display_buf);
        }
    }
}

/**
 * @brief 按键消抖与释放等待，防止一次按下多次触发
 * @param key_value 当前键值（未使用）
 */
static void Wait_Ps2KeyRelease(uint8_t key_value)
{
    (void)key_value;
    delay_ms(120);                   /* 120ms消抖延时 */
    Ps2KeyValue = KeyValue_Null;     /* 清除按键标志 */
}

/**
 * @brief 单端测量菜单处理：循环刷新测量结果，Enter触发单次测量，Back返回
 */
static void MenuHandler_Measure(void)
{
    char value_text[24];

    /* 初始化界面 */
    Draw_Work_Title(TITLE_MEASURE);
    Draw_Key_Tips(TIP_MEASURE_1, TIP_MEASURE_2);

    /* 绘制9行静态标签 */
    OS_String_Show(280, k_compact_line_y[0], 24, 1, L_SWEEP_COUNT);
    OS_String_Show(280, k_compact_line_y[1], 24, 1, L_LENGTH_M);
    OS_String_Show(280, k_compact_line_y[2], 24, 1, L_RESOLUTION_M);
    OS_String_Show(280, k_compact_line_y[3], 24, 1, L_START_MHZ);
    OS_String_Show(280, k_compact_line_y[4], 24, 1, L_STOP_MHZ);
    OS_String_Show(280, k_compact_line_y[5], 24, 1, L_STEP_KHZ);
    OS_String_Show(280, k_compact_line_y[6], 24, 1, L_MESSAGE);
    OS_String_Show(280, k_compact_line_y[7], 24, 1, L_DET_V);
    OS_String_Show(280, k_compact_line_y[8], 24, 1, L_PEAK_MAG);

    while(Ps2KeyValue != KeyValue_Back)
    {
        /* Enter键触发一次完整测量 */
        if(Ps2KeyValue == KeyValue_Enter)
        {
            Ps2KeyValue = KeyValue_Null;
            EthMeasureOnce();
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }

        /* 更新所有动态参数 */
        sprintf(value_text, "%lu", (unsigned long)g_eth_runtime.sweep_count);
        Show_Compact_Value_Only(0, value_text);

        sprintf(value_text, "%.2f", g_eth_runtime.last_length_m);
        Show_Compact_Value_Only(1, value_text);

        sprintf(value_text, "%.3f", g_eth_runtime.last_resolution_m);
        Show_Compact_Value_Only(2, value_text);

        sprintf(value_text, "%lu", (unsigned long)(g_eth_cfg.start_freq_hz / 1000000U));
        Show_Compact_Value_Only(3, value_text);

        sprintf(value_text, "%lu", (unsigned long)(g_eth_cfg.stop_freq_hz / 1000000U));
        Show_Compact_Value_Only(4, value_text);

        sprintf(value_text, "%lu", (unsigned long)(g_eth_cfg.step_hz / 1000U));
        Show_Compact_Value_Only(5, value_text);

        Show_Compact_Value_Only(6, g_eth_runtime.last_message);

        sprintf(value_text, "%.4f", g_eth_runtime.detector_voltage_v);
        Show_Compact_Value_Only(7, value_text);

        sprintf(value_text, "%.4f", g_eth_runtime.last_peak_metric);
        Show_Compact_Value_Only(8, value_text);

        delay_ms(300);
    }

    /* 返回主菜单 */
    Ps2KeyValue = KeyValue_Null;
    g_menu_sign = 0U;
    Clear_Work_Area();
}

/**
 * @brief 参数设置菜单处理：切换光标行，加减调整参数值，Back返回
 */
static void MenuHandler_Params(void)
{
    uint8_t cursor = 0U;     /* 当前选中行（0~6） */
    char value_text[24];
    char marker[8];

    Draw_Work_Title(TITLE_PARAMS);
    Draw_Key_Tips(TIP_PARAMS_1, TIP_PARAMS_2);

    /* 绘制参数标签 */
    OS_String_Show(280, k_compact_line_y[0], 24, 1, L_START_MHZ2);
    OS_String_Show(280, k_compact_line_y[1], 24, 1, L_STOP_MHZ2);
    OS_String_Show(280, k_compact_line_y[2], 24, 1, L_STEP_KHZ2);
    OS_String_Show(280, k_compact_line_y[3], 24, 1, L_SETTLE_MS);
    OS_String_Show(280, k_compact_line_y[4], 24, 1, L_NVP);
    OS_String_Show(280, k_compact_line_y[5], 24, 1, L_IFFT_POINTS);
    OS_String_Show(280, k_compact_line_y[6], 24, 1, L_CABLE_REF_CM);
    OS_String_Show(280, k_compact_line_y[7], 24, 1, L_CURSOR);

    while(Ps2KeyValue != KeyValue_Back)
    {
        /* Enter键切换当前参数行 */
        if(Ps2KeyValue == KeyValue_Enter)
        {
            cursor = (uint8_t)((cursor + 1U) % 7U);
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }
        /* '+' 键增加当前参数 */
        else if(Ps2KeyValue == KeyValue_Add)
        {
            switch(cursor)
            {
                case 0U: g_eth_cfg.start_freq_hz += 1000000U; break;  /* 起始频率 +1MHz */
                case 1U: g_eth_cfg.stop_freq_hz += 1000000U; break;   /* 终止频率 +1MHz */
                case 2U: g_eth_cfg.step_hz += 100000U; break;         /* 步进 +100kHz */
                case 3U: g_eth_cfg.settle_ms += 1U; break;            /* 稳定时间 +1ms */
                case 4U: g_eth_cfg.nvp_x1000 += 5U; break;           /* NVP +5 */
                case 5U: g_eth_cfg.ifft_points = (g_eth_cfg.ifft_points > 512U) ? 512U : 256U; break;
                case 6U: g_eth_cfg.calib_length_cm += UI_CALIB_LENGTH_STEP_CM; break; /* 参考长度 +50cm */
                default: break;
            }
            EthNormalizeConfig();    /* 确保参数合法 */
            EthRefreshDdsPreview();  /* 更新DDS至新频率 */
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Add);
        }
        /* '-' 键减小当前参数 */
        else if(Ps2KeyValue == KeyValue_Minus)
        {
            switch(cursor)
            {
                case 0U: if(g_eth_cfg.start_freq_hz > 1000000U) g_eth_cfg.start_freq_hz -= 1000000U; break;
                case 1U: if(g_eth_cfg.stop_freq_hz > (g_eth_cfg.start_freq_hz + 1000000U)) g_eth_cfg.stop_freq_hz -= 1000000U; break;
                case 2U: if(g_eth_cfg.step_hz > 100000U) g_eth_cfg.step_hz -= 100000U; break;
                case 3U: if(g_eth_cfg.settle_ms > 1U) g_eth_cfg.settle_ms -= 1U; break;
                case 4U: if(g_eth_cfg.nvp_x1000 > UI_MIN_NVP_X1000) g_eth_cfg.nvp_x1000 -= 5U; break;
                case 5U: g_eth_cfg.ifft_points = 256U; break;
                case 6U: if(g_eth_cfg.calib_length_cm > UI_CALIB_LENGTH_STEP_CM) g_eth_cfg.calib_length_cm -= UI_CALIB_LENGTH_STEP_CM; break;
                default: break;
            }
            EthNormalizeConfig();
            EthRefreshDdsPreview();
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Minus);
        }

        /* 刷新所有参数显示 */
        sprintf(value_text, "%lu", (unsigned long)(g_eth_cfg.start_freq_hz / 1000000U));
        Show_Compact_Value_Only(0, value_text);

        sprintf(value_text, "%lu", (unsigned long)(g_eth_cfg.stop_freq_hz / 1000000U));
        Show_Compact_Value_Only(1, value_text);

        sprintf(value_text, "%lu", (unsigned long)(g_eth_cfg.step_hz / 1000U));
        Show_Compact_Value_Only(2, value_text);

        sprintf(value_text, "%u", g_eth_cfg.settle_ms);
        Show_Compact_Value_Only(3, value_text);

        sprintf(value_text, "%u", g_eth_cfg.nvp_x1000);
        Show_Compact_Value_Only(4, value_text);

        sprintf(value_text, "%u", g_eth_cfg.ifft_points);
        Show_Compact_Value_Only(5, value_text);

        sprintf(value_text, "%u", g_eth_cfg.calib_length_cm);
        Show_Compact_Value_Only(6, value_text);

        sprintf(marker, "%u", (unsigned int)(cursor + 1U));
        Show_Compact_Value_Only(7, marker);

        delay_ms(300);
    }

    Ps2KeyValue = KeyValue_Null;
    g_menu_sign = 0U;
    Clear_Work_Area();
}

/**
 * @brief 校准菜单处理：Enter执行测量并校准NVP，+/-/*分别采集开路/短路/负载标准
 */
static void MenuHandler_Calibrate(void)
{
    char value_text[24];

    Draw_Work_Title(TITLE_CALIB);
    Draw_Key_Tips(TIP_CAL_1, TIP_CAL_2);  /* 提示已修复为清晰文字 */

    /* 绘制校准界面标签 */
    OS_String_Show(280, k_compact_line_y[0], 24, 1, L_MODE);
    OS_String_Show(280, k_compact_line_y[1], 24, 1, L_REF_CABLE_M);
    OS_String_Show(280, k_compact_line_y[2], 24, 1, L_CUR_NVP);
    OS_String_Show(280, k_compact_line_y[3], 24, 1, L_MEASURED_M);
    OS_String_Show(280, k_compact_line_y[4], 24, 1, L_CAL_STATUS);
    OS_String_Show(280, k_compact_line_y[5], 24, 1, L_DET_V);
    OS_String_Show(280, k_compact_line_y[6], 24, 1, L_PEAK_BIN);
    OS_String_Show(280, k_compact_line_y[7], 24, 1, L_MESSAGE);

    while(Ps2KeyValue != KeyValue_Back)
    {
        /* Enter键：测量并更新NVP */
        if(Ps2KeyValue == KeyValue_Enter)
        {
            if(EthMeasureOnce() != 0U)
            {
                EthApplyCalibration();
            }
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }
        /* '+' 键：采集开路校准数据 */
        else if(Ps2KeyValue == KeyValue_Add)
        {
            EthCapturePortCalibration(ETH_PORT_CAL_OPEN);
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Add);
        }
        /* '-' 键：采集短路校准数据 */
        else if(Ps2KeyValue == KeyValue_Minus)
        {
            EthCapturePortCalibration(ETH_PORT_CAL_SHORT);
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Minus);
        }
        /* '*' 键：采集负载校准数据（修复后的提示） */
        else if(Ps2KeyValue == KeyValue_Ride)
        {
            EthCapturePortCalibration(ETH_PORT_CAL_LOAD);
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Ride);
        }

        Show_Compact_Value_Only(0, V_SINGLE_END_FDR);

        sprintf(value_text, "%.2f", (float)g_eth_cfg.calib_length_cm / 100.0f);
        Show_Compact_Value_Only(1, value_text);

        sprintf(value_text, "%u", g_eth_cfg.nvp_x1000);
        Show_Compact_Value_Only(2, value_text);

        sprintf(value_text, "%.2f", g_eth_runtime.last_length_m);
        Show_Compact_Value_Only(3, value_text);

        /* 根据校准状态显示对应文本 */
        if(g_eth_runtime.port_cal_valid != 0U)
        {
            Show_Compact_Value_Only(4, "SOL Ready");
        }
        else if(g_eth_runtime.baseline_ready != 0U)
        {
            Show_Compact_Value_Only(4, "100R Baseline");
        }
        else if(g_eth_runtime.calib_ready != 0U)
        {
            Show_Compact_Value_Only(4, V_CAL_ARMED);
        }
        else
        {
            Show_Compact_Value_Only(4, V_IDLE);
        }

        sprintf(value_text, "%.4f", g_eth_runtime.detector_voltage_v);
        Show_Compact_Value_Only(5, value_text);

        sprintf(value_text, "%u", g_eth_runtime.last_peak_bin);
        Show_Compact_Value_Only(6, value_text);

        Show_Compact_Value_Only(7, g_eth_runtime.last_message);
        delay_ms(300);
    }

    Ps2KeyValue = KeyValue_Null;
    g_menu_sign = 0U;
    Clear_Work_Area();
}

/**
 * @brief 状态查看菜单处理：显示DDS、ADC及校准状态，Enter触发一次ADC采样检测
 */
static void MenuHandler_Status(void)
{
    char value_text[24];

    Draw_Work_Title(TITLE_STATUS);
    Draw_Key_Tips(TIP_STATUS_1, TIP_STATUS_2);

    /* 绘制状态标签 */
    OS_String_Show(280, k_compact_line_y[0], 24, 1, L_DDS);
    OS_String_Show(280, k_compact_line_y[1], 24, 1, L_ADC);
    OS_String_Show(280, k_compact_line_y[2], 24, 1, L_ADC_FRAME);
    OS_String_Show(280, k_compact_line_y[3], 24, 1, L_CALIBRATION);
    OS_String_Show(280, k_compact_line_y[4], 24, 1, L_DDS_FREQ_HZ);
    OS_String_Show(280, k_compact_line_y[5], 24, 1, L_DET_V);
    OS_String_Show(280, k_compact_line_y[6], 24, 1, L_REF_V);
    OS_String_Show(280, k_compact_line_y[7], 24, 1, L_SWEEP_COUNT);
    OS_String_Show(280, k_compact_line_y[8], 24, 1, L_MESSAGE);

    while(Ps2KeyValue != KeyValue_Back)
    {
        /* Enter键：执行一次ADC探测 */
        if(Ps2KeyValue == KeyValue_Enter)
        {
            EthProbeAdcOnce();
            Ps2KeyValue = KeyValue_Null;
            Wait_Ps2KeyRelease(KeyValue_Enter);
        }

        /* 逐行显示状态 */
        Show_Compact_Value_Only(0, (g_eth_runtime.dds_ready != 0U) ? V_READY : V_OFF);
        Show_Compact_Value_Only(1, (g_eth_runtime.adc_ready != 0U) ? V_READY : V_OFF);
        Show_Compact_Value_Only(2, (g_eth_runtime.adc_capture_ready != 0U) ? V_READY : V_UNCHECKED);
        Show_Compact_Value_Only(3, (g_eth_runtime.calib_ready != 0U) ? V_ARMED : V_IDLE2);

        sprintf(value_text, "%lu", (unsigned long)g_eth_runtime.dds_freq_hz);
        Show_Compact_Value_Only(4, value_text);

        sprintf(value_text, "%.4f", g_eth_runtime.detector_voltage_v);
        Show_Compact_Value_Only(5, value_text);

        sprintf(value_text, "%.4f", g_eth_runtime.detector_aux_voltage_v);
        Show_Compact_Value_Only(6, value_text);

        sprintf(value_text, "%lu", (unsigned long)g_eth_runtime.sweep_count);
        Show_Compact_Value_Only(7, value_text);

        Show_Compact_Value_Only(8, g_eth_runtime.last_message);

        delay_ms(300);
    }

    Ps2KeyValue = KeyValue_Null;
    g_menu_sign = 0U;
    Clear_Work_Area();
}
