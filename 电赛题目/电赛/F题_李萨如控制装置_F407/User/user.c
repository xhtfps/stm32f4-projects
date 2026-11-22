/**
 * @file    user.c
 * @brief   F 题李萨如图形显示控制装置用户界面实现
 *
 * 负责 LCD 显示、按键交互、页面切换及测量数据刷新。
 * 界面框架沿用“简易电路模型探究”的四级菜单结构，支持中英文双语显示。
 * 所有显示操作通过底层 TFT_LCD 和字库驱动完成。
 */

#include "user.h"

#include <stdio.h>
#include <string.h>

#include "./BSP/TFT_LCD/TFT_LCD.h"
#include "./BSP/TFT_LCD/text.h"

/* ---------- 界面布局常量（像素） ---------- */
#define CONTENT_LEFT    314U    /* 右侧内容区起始 X 坐标 */
#define DATA_VALUE_X    564U    /* 第二列数据（如 FFT BIN）的 X 坐标 */

/* ---------- 页面索引：严格对应题目要求，不交叉复用 ---------- */
#define PAGE_MAIN              0U
#define PAGE_WAVEFORM_CONTROL  1U
#define PAGE_AMPLITUDE_CONTROL 2U
#define PAGE_AUTOMATIC_CONTROL 3U
#define PAGE_DEBUG             4U

/* ---------- 字号 ---------- */
#define UI_MENU_SIZE    24U     /* 左侧菜单文字大小 */
#define UI_PRIMARY_SIZE 24U     /* 内容区标题 / 主值大小 */
#define UI_HELP_SIZE    16U     /* 帮助提示文字大小 */

typedef uint8_t UiPage;        /* 页面类型 */

/* ===== GBK 编码的中文字符串常量 =====
 * 采用 GBK 编码的十六进制字节序列，用于不支持字库时直接存储，
 * 支持字库时则调用 LCD_ShowGBKFont 显示。
 * 以下是各字符串对应的中文内容：
 */
static const uint8_t gbk_title[] =       /* “李萨如控制装置” */
    "\xC0\xEE\xC8\xF8\xC8\xE7\xBF\xD8\xD6\xC6\xD7\xB0\xD6\xC3";
static const uint8_t gbk_dash[] =        /* "——"（菜单项前缀） */
    "\xA1\xAA";
static const uint8_t gbk_selected[] =    /* "◆"（选中标记） */
    "\xA1\xF4";
static const uint8_t gbk_select[] =      /* "请选择菜单" */
    "\xC7\xEB\xD1\xA1\xD4\xF1\xB2\xCB\xB5\xA5";
static const uint8_t gbk_run[] =         /* "按下回车启动" */
    "\xB0\xB4\xCF\xC2\xBB\xD8\xB3\xB5\xC6\xF4\xB6\xAF";
static const uint8_t gbk_back[] =        /* "返回主菜单" */
    "\xB7\xB5\xBB\xD8\xD6\xF7\xB2\xCB\xB5\xA5";

/* ---------- 静态全局变量 ---------- */
static UiPage s_page = PAGE_MAIN;             /* 当前显示页面 */
static uint32_t s_frequency_hz = UI_DEFAULT_FREQUENCY_HZ;   /* 用户设置的频率 */
static float s_amplitude_mv = UI_DEFAULT_AMPLITUDE_MV;      /* 用户设置的幅度 */
static uint32_t s_phase_deg = UI_DEFAULT_PHASE_DEG;         /* 用户设置的相位差 */
static float s_known_target_peak_v = UI_DEFAULT_KNOWN_TARGET_PEAK_V; /* 已知模型目标峰值电压 */
static uint8_t s_chinese_font_available;     /* 中文字库是否可用标志 */
static UI_Measurement_t s_measurement;       /* 当前测量结果缓存 */
static uint8_t s_calibration_item = UI_CALIBRATION_ADS_REFERENCE; /* 校准项 */
static uint8_t s_target_span_y_div = 8U;     /* 要求4默认输出8div */
static uint8_t s_wired_target_shape = UI_TARGET_SHAPE_DIAGONAL;
static uint8_t s_auto_target_shape = UI_TARGET_SHAPE_DIAGONAL;

/**
 * @brief  在指定位置显示单个 ASCII 字符（支持 16/24/32 点阵）
 * @param  x    起始横坐标
 * @param  y    起始纵坐标
 * @param  num  字符 ASCII 码
 * @param  size 字号（16/24/32）
 * @param  mode 0: 非叠加；1: 叠加（背景部分不绘制，仅绘制前景）
 */
static void OS_Char_Show(uint16_t x, uint16_t y, uint8_t num,
                         uint8_t size, uint8_t mode)
{
    uint8_t temp;
    uint8_t t1;
    uint8_t t;
    uint16_t y0 = y;
    uint8_t csize;

    /* 计算字符点阵数据的字节数 */
    csize = (uint8_t)((size / 8U + ((size % 8U) ? 1U : 0U)) *
                      (size / 2U));
    /* 可显示字符范围 ' ' ~ '~'，超出则替换为空格 */
    if ((num < (uint8_t)' ') || (num > (uint8_t)'~'))
    {
        num = (uint8_t)' ';
    }
    else
    {
        num = (uint8_t)(num - (uint8_t)' '); /* 偏移到字库索引 */
    }

    for (t = 0U; t < csize; ++t)
    {
        /* 根据字号选择对应点阵表 */
        if (size == 16U)
        {
            temp = asc2_1608[num][t];
        }
        else if (size == 24U)
        {
            temp = asc2_2412[num][t];
        }
        else if (size == 32U)
        {
            temp = asc2_3216[num][t];
        }
        else
        {
            return; /* 不支持的字号 */
        }

        /* 逐位绘制 8 个像素点 */
        for (t1 = 0U; t1 < 8U; ++t1)
        {
            if ((temp & 0x80U) != 0U)
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(TextColor);       /* 前景色 */
            }
            else if (mode == 1U)              /* 叠加模式才绘制背景 */
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(BackColor);       /* 背景色 */
            }
            temp <<= 1U;
            ++y;
            if (y >= 480U) return;
            if ((uint16_t)(y - y0) == size)   /* 完成一列，换到下一列 */
            {
                y = y0;
                ++x;
                if (x >= 800U) return;
                break;
            }
        }
    }
}

/**
 * @brief  显示 GBK 编码的中文字符（两个字节）
 * @param  x,y   起始坐标
 * @param  font  GBK 编码字符串指针（至少 2 字节）
 * @param  size  字号
 * @param  mode  显示模式
 */
static void OS_Font_Show(uint16_t x, uint16_t y, char *font,
                         uint8_t size, uint8_t mode)
{
    LCD_ShowGBKFont(x, y, (uint8_t *)font, size, mode);
}

/**
 * @brief  显示混合中英文的字符串（以 GBK 编码存储，自动识别中英文）
 * @param  x,y   起始坐标
 * @param  size  字号
 * @param  mode  显示模式
 * @param  str   字符串（GBK 编码，中文占 2 字节，英文占 1 字节）
 */
static void OS_String_Show(uint16_t x, uint16_t y, uint8_t size,
                           uint8_t mode, char *str)
{
    uint16_t x0 = x;
    uint8_t chinese = 0U;

    if ((str == NULL) || (s_chinese_font_available == 0U))
    {
        return;
    }
    while (*str != '\0')
    {
        if (chinese == 0U)
        {
            if ((int8_t)*str < 0)          /* 最高位为 1，是中文字节 */
            {
                chinese = 1U;
            }
            else if (*str == '\n')         /* 换行符 */
            {
                y = (uint16_t)(y + size);
                x = x0;
                ++str;
                continue;
            }
            else                            /* ASCII 字符 */
            {
                OS_Char_Show(x, y, (uint8_t)*str, size, mode);
                ++str;
                x = (uint16_t)(x + size / 2U);
            }
        }
        else                                /* 已识别为中文，显示两个字节 */
        {
            chinese = 0U;
            OS_Font_Show(x, y, str, size, mode);
            str += 2;
            x = (uint16_t)(x + size);
        }
    }
}

/**
 * @brief  显示纯 ASCII 字符串（忽略中文字库）
 * @param  x,y   起始坐标
 * @param  size  字号
 * @param  mode  显示模式
 * @param  str   以 '\0' 结尾的 ASCII 字符串
 */
static void OS_Ascii_Show(uint16_t x, uint16_t y, uint8_t size,
                          uint8_t mode, const char *str)
{
    if (str == NULL) return;
    while (*str != '\0')
    {
        if (*str == '\n')
        {
            y = (uint16_t)(y + size);
            x = 0U;
            ++str;
            continue;
        }
        OS_Char_Show(x, y, (uint8_t)*str, size, mode);
        ++str;
        x = (uint16_t)(x + size / 2U);
    }
}

/**
 * @brief  智能字符串显示：根据中文字库是否可用选择显示方式
 * @param  gbk_text     GBK 编码的中文文本（字库可用时使用）
 * @param  ascii_text   纯 ASCII 备用文本（字库不可用时使用）
 */
static void UI_String_Show(uint16_t x, uint16_t y, uint8_t size,
                           uint8_t mode, const uint8_t *gbk_text,
                           const char *ascii_text)
{
    if (s_chinese_font_available != 0U)
    {
        OS_String_Show(x, y, size, mode, (char *)gbk_text);
    }
    else
    {
        OS_Ascii_Show(x, y, size, mode, ascii_text);
    }
}

/**
 * @brief  显示浮点数值，支持中英文格式字符串
 * @param  x,y          坐标
 * @param  size         字号
 * @param  mode         显示模式
 * @param  value        数值
 * @param  format       中文格式字符串（字库可用时）
 * @param  ascii_format 英文格式字符串（字库不可用时）
 */
static void OS_Num_Show(uint16_t x, uint16_t y, uint8_t size,
                        uint8_t mode, double value, char *format,
                        const char *ascii_format)
{
    char buffer[100];

    if (format == NULL)
    {
        (void)sprintf(buffer, "%g", value);
    }
    else if (s_chinese_font_available != 0U)
    {
        (void)sprintf(buffer, format, value);
    }
    else
    {
        (void)sprintf(buffer, (ascii_format != NULL) ? ascii_format : "%g",
                      value);
    }
    UI_String_Show(x, y, size, mode, (const uint8_t *)buffer, buffer);
}

/**
 * @brief  清除右侧内容区域（252 ≤ x ≤ 800, 72 ≤ y ≤ 440）
 */
/**
 * @brief  绘制固定参考框架：标题、分隔线、版本信息、左侧菜单项
 */
static void draw_reference_frame(void)
{
    uint8_t count;

    LCD_SetColors(White, Black);
    /* 顶部标题 */
    UI_String_Show(272U, 16U, 32U, 0U, gbk_title, TitleStr);
    /* 水平分隔线（上、下） */
    LCD_Appoint_Clear(0U, 64U, 800U, 72U, White);
    LCD_Appoint_Clear(0U, 440U, 800U, 448U, White);
    /* 垂直分隔线（左右区域） */
    LCD_Appoint_Clear(250U, 72U, 252U, 440U, White);
    /* 左下角模型/用户版本号 */
    UI_String_Show(32U, 456U, 16U, 0U,
                   (const uint8_t *)ModelVerStr, ModelVerStr);
    UI_String_Show(632U, 456U, 16U, 0U,
                   (const uint8_t *)UserVerStr, UserVerStr);

    /* 左侧菜单项（带“——”前缀） */
    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 0U,
                       gbk_dash, "-");
    }
    /* 菜单项统一使用四字中文标题；ASCII 仅用于字库不可用时降级显示。 */
    UI_String_Show(80U, 96U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice1, "WAVE");
    UI_String_Show(80U, 160U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice2, "AMPL");
    UI_String_Show(80U, 224U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice3, "AUTO");
    UI_String_Show(80U, 288U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice4, "TEST");
}

/**
 * @brief  更新左侧菜单项高亮状态（当前页面用“◆”标记）
 */
static void draw_menu_selection(void)
{
    uint8_t count;

    /* 先清除所有标记（用“——”覆盖） */
    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 1U,
                       gbk_dash, "-");
    }
    /* 非主菜单时，在当前页对应项绘制选中标记 */
    if (s_page != PAGE_MAIN)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * s_page), 32U, 1U,
                       gbk_selected, ">");
    }
}

/**
 * @brief  绘制主菜单页面内容（右侧提示信息）
 */
static void draw_main_page(void)
{
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   gbk_select, "SELECT MENU");
    UI_String_Show(CONTENT_LEFT, 160U, UI_HELP_SIZE, 1U,
                   gbk_run, "ENTER RUN");
    UI_String_Show(CONTENT_LEFT, 184U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

static void refresh_wired_common(uint8_t requirement3);

static uint8_t wired_shape_key(void)
{
    if (s_wired_target_shape == UI_TARGET_SHAPE_ELLIPSE)
        return 1U;
    if (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY)
        return 3U;
    return 0U;
}

static void draw_waveform_control_page(void)
{
    refresh_wired_common(
        (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY) ? 1U : 0U);
    OS_Num_Show(CONTENT_LEFT, 292U, UI_HELP_SIZE, 1U,
                (double)wired_shape_key(),
                "SHAPE KEY %.0f        ", "SHAPE KEY %.0f        ");
    UI_String_Show(CONTENT_LEFT, 324U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"0=DIAG 1=CIRCLE 3=INFINITY",
                   "0=DIAG 1=CIRCLE 3=INFINITY");
    UI_String_Show(CONTENT_LEFT, 356U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

static void refresh_wired_common(uint8_t requirement3)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.frequency_hz_precise,
                "FIN %.3f Hz         ", "FIN %.3f Hz         ");
    OS_Num_Show(DATA_VALUE_X, 96U, 20U, 1U,
                (double)s_measurement.feedback_frequency_hz_precise,
                "FOUT %.3f       ", "FOUT %.3f       ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.input_vpp_mv,
                "VIN %.0f mVpp        ", "VIN %.0f mVpp        ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.output_vpp_mv,
                "VOUT %.0f       ", "VOUT %.0f       ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.amplitude_error_mv,
                "AMP ERR %.0f mV      ", "AMP ERR %.0f mV      ");
    OS_Num_Show(CONTENT_LEFT, 192U, 24U, 1U,
                (double)s_measurement.phase_relation_deg,
                requirement3 ? "PHI y-2x %.2f deg    " :
                               "PHI y-x %.2f deg     ",
                requirement3 ? "PHI y-2x %.2f deg    " :
                               "PHI y-x %.2f deg     ");
    OS_Num_Show(CONTENT_LEFT, 224U, 24U, 1U,
                (double)s_measurement.phase_error_deg,
                "PHASE ERR %.2f deg   ", "PHASE ERR %.2f deg   ");
    UI_String_Show(CONTENT_LEFT, 260U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)s_measurement.status_text,
                   s_measurement.status_text);
}

static void draw_amplitude_control_page(void)
{
    refresh_wired_common(
        (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY) ? 1U : 0U);
    OS_Num_Show(CONTENT_LEFT, 284U, UI_HELP_SIZE, 1U,
                (double)wired_shape_key(),
                "SHAPE %.0f          ", "SHAPE %.0f          ");
    OS_Num_Show(DATA_VALUE_X, 284U, UI_HELP_SIZE, 1U,
                (double)s_target_span_y_div,
                "Y %.0f DIV       ", "Y %.0f DIV       ");
    UI_String_Show(CONTENT_LEFT, 316U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"0=DIAG 1=CIRCLE 3=INFINITY",
                   "0=DIAG 1=CIRCLE 3=INFINITY");
    UI_String_Show(CONTENT_LEFT, 340U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"2/4/6/8=Y VPP DIV (LIVE)",
                   "2/4/6/8=Y VPP DIV (LIVE)");
    UI_String_Show(CONTENT_LEFT, 372U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

static void refresh_requirement5_values(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_auto_target_shape,
                "AUTO TARGET %.0f       ", "AUTO TARGET %.0f       ");
    OS_Num_Show(CONTENT_LEFT, 124U, 20U, 1U,
                (double)s_measurement.vision_confidence,
                "VISION CONF %.3f      ", "VISION CONF %.3f      ");
    OS_Num_Show(DATA_VALUE_X, 124U, 20U, 1U,
                (double)s_measurement.vision_online,
                "LINK %.0f      ", "LINK %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 152U, 20U, 1U,
                (double)s_measurement.vision_ratio_y,
                "RATIO Y %.0f       ", "RATIO Y %.0f       ");
    OS_Num_Show(DATA_VALUE_X, 152U, 20U, 1U,
                (double)s_measurement.vision_ratio_x,
                "X %.0f      ", "X %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 180U, 20U, 1U,
                (double)s_measurement.vision_span_x_div,
                "SPAN X %.3f div      ", "SPAN X %.3f div      ");
    OS_Num_Show(DATA_VALUE_X, 180U, 20U, 1U,
                (double)s_measurement.vision_span_y_div,
                "Y %.3f      ", "Y %.3f      ");
    OS_Num_Show(CONTENT_LEFT, 208U, 20U, 1U,
                (double)s_measurement.dds_frequency_hz,
                "DDS %.0f Hz         ", "DDS %.0f Hz         ");
    OS_Num_Show(DATA_VALUE_X, 208U, 20U, 1U,
                (double)s_measurement.vision_phase_error_deg,
                "PH %.1f      ", "PH %.1f      ");
    OS_Num_Show(CONTENT_LEFT, 236U, 20U, 1U,
                (double)s_measurement.sweep_index,
                "PROBE %.0f       ", "PROBE %.0f       ");
    OS_Num_Show(DATA_VALUE_X, 236U, 20U, 1U,
                (double)s_measurement.sweep_total,
                "/ %.0f      ", "/ %.0f      ");
    UI_String_Show(CONTENT_LEFT, 260U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)s_measurement.status_text,
                   s_measurement.status_text);
}

static void draw_requirement5_page(void)
{
    refresh_requirement5_values();
    UI_String_Show(CONTENT_LEFT, 292U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"SOURCE-DEVICE MUST BE OPEN",
                   "SOURCE-DEVICE MUST BE OPEN");
    UI_String_Show(CONTENT_LEFT, 316U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"1=LINE 2=CIRCLE 3=INFINITY",
                   "1=LINE 2=CIRCLE 3=INFINITY");
    UI_String_Show(CONTENT_LEFT, 348U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

static void refresh_debug_values(void)
{
    char *mix_format;

    OS_Num_Show(CONTENT_LEFT, 96U, 20U, 1U,
                (double)s_measurement.frequency_hz_precise,
                "PB10 %.3f Hz       ", "PB10 %.3f Hz       ");
    OS_Num_Show(DATA_VALUE_X, 96U, 20U, 1U,
                (double)s_measurement.feedback_frequency_hz_precise,
                "PB11 %.3f       ", "PB11 %.3f       ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.adc_sample_rate_hz,
                "ADC FS %.0f Hz       ", "ADC FS %.0f Hz       ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.adc_overrun_count,
                "OVR %.0f       ", "OVR %.0f       ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.input_vpp_mv,
                "VIN %.0f mVpp       ", "VIN %.0f mVpp       ");
    OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                (double)s_measurement.output_vpp_mv,
                "VOUT %.0f       ", "VOUT %.0f       ");
    OS_Num_Show(CONTENT_LEFT, 192U, 20U, 1U,
                (double)s_measurement.phase_relation_deg,
                "DFT %.2f deg      ", "DFT %.2f deg      ");
    OS_Num_Show(DATA_VALUE_X, 192U, 20U, 1U,
                (double)s_measurement.crossing_phase_relation_deg,
                "ZC %.2f       ", "ZC %.2f       ");
    OS_Num_Show(CONTENT_LEFT, 220U, 20U, 1U,
                (double)s_measurement.phase_estimator_difference_deg,
                "D-Z %.2f deg      ", "D-Z %.2f deg      ");
    if (s_measurement.dds_test_active != 0U)
    {
        OS_Num_Show(DATA_VALUE_X, 220U, 20U, 1U,
                    (double)s_measurement.dds_frequency_hz,
                    "CH0 %.0f Hz       ", "CH0 %.0f Hz       ");
        OS_Num_Show(CONTENT_LEFT, 252U, 20U, 1U,
                    (double)s_measurement.dds_base_ftw,
                    "FTW %.0f          ", "FTW %.0f          ");
        mix_format =
            (s_measurement.dds_dither_enabled != 0U) ?
            "MIX %.0f/16 ON  " : "MIX %.0f/16 OFF ";
        OS_Num_Show(DATA_VALUE_X, 252U, 20U, 1U,
                    (double)s_measurement.dds_dither_numerator,
                    mix_format, mix_format);
    }
    else
    {
        OS_Num_Show(DATA_VALUE_X, 220U, 20U, 1U,
                    (double)s_measurement.adc_rejected_block_count,
                    "REJ %.0f       ", "REJ %.0f       ");
        OS_Num_Show(CONTENT_LEFT, 252U, 20U, 1U,
                    (double)s_measurement.input_bias_mv,
                    "BIAS1 %.0f mV       ", "BIAS1 %.0f mV       ");
        OS_Num_Show(DATA_VALUE_X, 252U, 20U, 1U,
                    (double)s_measurement.output_bias_mv,
                    "B2 %.0f       ", "B2 %.0f       ");
        OS_Num_Show(CONTENT_LEFT, 280U, 20U, 1U,
                    (double)s_measurement.output_min_count,
                    "PA2 MIN %.0f      ", "PA2 MIN %.0f      ");
        OS_Num_Show(DATA_VALUE_X, 280U, 20U, 1U,
                    (double)s_measurement.output_max_count,
                    "MAX %.0f      ", "MAX %.0f      ");
    }
    UI_String_Show(CONTENT_LEFT, 304U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)s_measurement.status_text,
                   s_measurement.status_text);
}

static void draw_debug_page(void)
{
    refresh_debug_values();
    UI_String_Show(CONTENT_LEFT, 328U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"ENTER=WIRED 0=OFF 1-4=FREQ",
                   "ENTER=WIRED 0=OFF 1-4=FREQ");
    UI_String_Show(CONTENT_LEFT, 352U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"5/6=FTW -1/+1",
                   "5/6=FTW -1/+1");
    UI_String_Show(CONTENT_LEFT, 376U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"7/9=MIX -/+  8=MIX ON/OFF",
                   "7/9=MIX -/+  8=MIX ON/OFF");
    UI_String_Show(CONTENT_LEFT, 400U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

static void refresh_page_static(void)
{
    if (s_page == PAGE_WAVEFORM_CONTROL)
    {
        OS_Num_Show(CONTENT_LEFT, 292U, UI_HELP_SIZE, 1U,
                    (double)wired_shape_key(),
                    "SHAPE KEY %.0f        ", "SHAPE KEY %.0f        ");
        OS_Num_Show(CONTENT_LEFT, 192U, 24U, 1U,
                    (double)s_measurement.phase_relation_deg,
                    (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY) ?
                    "PHI y-2x %.2f deg    " : "PHI y-x %.2f deg     ",
                    (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY) ?
                    "PHI y-2x %.2f deg    " : "PHI y-x %.2f deg     ");
        UI_String_Show(CONTENT_LEFT, 324U, UI_HELP_SIZE, 1U,
                       (const uint8_t *)"0=DIAG 1=CIRCLE 3=INFINITY",
                       "0=DIAG 1=CIRCLE 3=INFINITY");
        UI_String_Show(CONTENT_LEFT, 356U, UI_HELP_SIZE, 1U,
                       gbk_back, "BACK STOP/MAIN");
    }
    else if (s_page == PAGE_AMPLITUDE_CONTROL)
    {
        OS_Num_Show(CONTENT_LEFT, 284U, UI_HELP_SIZE, 1U,
                    (double)wired_shape_key(),
                    "SHAPE %.0f          ", "SHAPE %.0f          ");
        OS_Num_Show(DATA_VALUE_X, 284U, UI_HELP_SIZE, 1U,
                    (double)s_target_span_y_div,
                    "Y %.0f DIV       ", "Y %.0f DIV       ");
        OS_Num_Show(CONTENT_LEFT, 192U, 24U, 1U,
                    (double)s_measurement.phase_relation_deg,
                    (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY) ?
                    "PHI y-2x %.2f deg    " : "PHI y-x %.2f deg     ",
                    (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY) ?
                    "PHI y-2x %.2f deg    " : "PHI y-x %.2f deg     ");
        UI_String_Show(CONTENT_LEFT, 316U, UI_HELP_SIZE, 1U,
                       (const uint8_t *)"0=DIAG 1=CIRCLE 3=INFINITY",
                       "0=DIAG 1=CIRCLE 3=INFINITY");
        UI_String_Show(CONTENT_LEFT, 340U, UI_HELP_SIZE, 1U,
                       (const uint8_t *)"2/4/6/8=Y VPP DIV (LIVE)",
                       "2/4/6/8=Y VPP DIV (LIVE)");
        UI_String_Show(CONTENT_LEFT, 372U, UI_HELP_SIZE, 1U,
                       gbk_back, "BACK STOP/MAIN");
    }
    else if (s_page == PAGE_AUTOMATIC_CONTROL)
    {
        OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                    (double)s_auto_target_shape,
                    "AUTO TARGET %.0f       ", "AUTO TARGET %.0f       ");
        refresh_requirement5_values();
        UI_String_Show(CONTENT_LEFT, 292U, UI_HELP_SIZE, 1U,
                       (const uint8_t *)"SOURCE-DEVICE MUST BE OPEN",
                       "SOURCE-DEVICE MUST BE OPEN");
        UI_String_Show(CONTENT_LEFT, 316U, UI_HELP_SIZE, 1U,
                       (const uint8_t *)"1=LINE 2=CIRCLE 3=INFINITY",
                       "1=LINE 2=CIRCLE 3=INFINITY");
        UI_String_Show(CONTENT_LEFT, 348U, UI_HELP_SIZE, 1U,
                       gbk_back, "BACK STOP/MAIN");
    }
}

/**
 * @brief  根据当前页面索引绘制右侧内容区域
 */
static void draw_page_content(void)
{
    if (s_page == PAGE_MAIN)
        draw_main_page();
    else if (s_page == PAGE_WAVEFORM_CONTROL)
        draw_waveform_control_page();
    else if (s_page == PAGE_AMPLITUDE_CONTROL)
        draw_amplitude_control_page();
    else if (s_page == PAGE_AUTOMATIC_CONTROL)
        draw_requirement5_page();
    else
        draw_debug_page();
}

/**
 * @brief  完整重绘当前页面（清除右侧 + 绘制内容 + 更新菜单高亮）
 */
static void render_page(void)
{
    draw_page_content();
    draw_menu_selection();
}

static void render_main_page_after_exit(void)
{
    LCD_Appoint_Clear(252U, 72U, 800U, 439U, Black);
    draw_page_content();
    draw_menu_selection();
}

/**
 * @brief  局部刷新当前页面的动态数据（不重绘静态框架）
 */
static void refresh_page_data(void)
{
    draw_page_content();
}

/**
 * @brief  根据当前页面类型刷新测量数据（频率、相位、FFT 等）
 */
static void refresh_measurement_data(void)
{
    if (s_page == PAGE_WAVEFORM_CONTROL)
        refresh_wired_common(
            (s_wired_target_shape == UI_TARGET_SHAPE_INFINITY) ? 1U : 0U);
    else if (s_page == PAGE_AMPLITUDE_CONTROL)
        draw_amplitude_control_page();
    else if (s_page == PAGE_AUTOMATIC_CONTROL)
        refresh_requirement5_values();
    else if (s_page == PAGE_DEBUG)
        refresh_debug_values();
}

/**
 * @brief  切换到指定页面，必要时进行参数限幅和状态重置
 * @param  next_page 目标页面索引
 */
static void switch_page(UiPage next_page)
{
    UiPage previous_page;

    if (next_page <= PAGE_DEBUG)
    {
        /* 进入有线测量页面前确保频率参数在题目范围内。 */
        if ((next_page == PAGE_WAVEFORM_CONTROL) ||
            (next_page == PAGE_AMPLITUDE_CONTROL) ||
            (next_page == PAGE_DEBUG))
        {
            if (s_frequency_hz < UI_MIN_FREQUENCY_HZ)
                s_frequency_hz = UI_MIN_FREQUENCY_HZ;
            if (s_frequency_hz > UI_MAX_FREQUENCY_HZ)
                s_frequency_hz = UI_MAX_FREQUENCY_HZ;
        }
        previous_page = s_page;
        s_page = next_page;

        if ((next_page == PAGE_MAIN) && (previous_page != PAGE_MAIN))
            render_main_page_after_exit();
        else
            render_page();
    }
}

/**
 * @brief  设置中文字库是否可用，影响后续字符串显示行为
 * @param  available 非 0 表示字库可用，0 表示不可用
 */
void UI_SetChineseFontAvailable(uint8_t available)
{
    s_chinese_font_available = (available != 0U) ? 1U : 0U;
}

/**
 * @brief  用户界面初始化：清屏、绘制框架、显示主菜单
 */
void UI_Init(void)
{
    /* 默认参数复位 */
    s_page = PAGE_MAIN;
    s_frequency_hz = UI_DEFAULT_FREQUENCY_HZ;
    s_amplitude_mv = UI_DEFAULT_AMPLITUDE_MV;
    s_phase_deg = UI_DEFAULT_PHASE_DEG;
    s_known_target_peak_v = UI_DEFAULT_KNOWN_TARGET_PEAK_V;
    s_calibration_item = UI_CALIBRATION_ADS_REFERENCE;
    s_target_span_y_div = 8U;
    s_wired_target_shape = UI_TARGET_SHAPE_DIAGONAL;
    s_auto_target_shape = UI_TARGET_SHAPE_DIAGONAL;
    (void)memset(&s_measurement, 0, sizeof(s_measurement));
    s_measurement.frequency_hz = s_frequency_hz;
    s_measurement.frequency_hz_precise = 0.0f;
    s_measurement.divider = 1U;
    s_measurement.amplitude_target_div = s_target_span_y_div;
    (void)strcpy(s_measurement.status_text, "WAIT SIGNAL");

    LCD_Clear(Black);
    draw_reference_frame();
    draw_page_content();
    draw_menu_selection();
}

/**
 * @brief  处理按键输入，根据当前页面状态生成对应的 UI 命令
 * @param  key     按键值（来自 PS/2 键盘或板载按键）
 * @param  command 输出的命令结构体指针
 * @return true: 生成了有效命令；false: 无命令产生
 */
bool UI_HandleKey(PS2_KeyValue_t key, UI_Command_t *command)
{
    if (command == NULL)
        return false;

    /* 默认填充命令参数 */
    command->type = UI_COMMAND_NONE;
    command->frequency_hz = s_frequency_hz;
    command->amplitude_mv = s_amplitude_mv;
    command->phase_deg = s_phase_deg;
    command->target_output_peak_v = s_known_target_peak_v;
    command->target_span_y_div = s_target_span_y_div;
    command->target_shape = s_auto_target_shape;
    command->calibration_item = s_calibration_item;
    command->dds_fine_step = 0;

    /* 退格键：停止当前操作并返回主菜单 */
    if (key == PS2_KEY_VALUE_BACKSPACE)
    {
        command->type = (s_page == PAGE_MAIN) ?
                        UI_COMMAND_NONE : UI_COMMAND_STOP_OUTPUT;
        switch_page(PAGE_MAIN);
        return (command->type != UI_COMMAND_NONE);
    }

    /* 主菜单：数字键 1~6 直接进入对应页面并产生启动命令 */
    if ((s_page == PAGE_MAIN) &&
        (key >= PS2_KEY_VALUE_1) && (key <= PS2_KEY_VALUE_4))
    {
        switch_page((UiPage)key);
        if (key == PS2_KEY_VALUE_1)
        {
            command->target_shape = s_wired_target_shape;
            command->type = UI_COMMAND_START_WAVEFORM_CONTROL;
        }
        else if (key == PS2_KEY_VALUE_2)
        {
            command->target_shape = s_wired_target_shape;
            command->target_span_y_div = s_target_span_y_div;
            command->type = UI_COMMAND_START_AMPLITUDE_CONTROL;
        }
        else if (key == PS2_KEY_VALUE_4)
            command->type = UI_COMMAND_START_DEBUG;
        /*
         * 要求5必须先明确三种独立自动目标。数字5只进入页面，
         * 进入后再按1/2/3直接启动对角线/圆/无穷模式。
         */
        return (command->type != UI_COMMAND_NONE);
    }

    /* 各子页面按键处理 */
    if (s_page == PAGE_WAVEFORM_CONTROL)
    {
        if ((key == PS2_KEY_VALUE_0) ||
            (key == PS2_KEY_VALUE_1) ||
            (key == PS2_KEY_VALUE_3))
        {
            s_wired_target_shape =
                (key == PS2_KEY_VALUE_0) ? UI_TARGET_SHAPE_DIAGONAL :
                (key == PS2_KEY_VALUE_1) ? UI_TARGET_SHAPE_ELLIPSE :
                                           UI_TARGET_SHAPE_INFINITY;
            command->target_shape = s_wired_target_shape;
            command->type = UI_COMMAND_START_WAVEFORM_CONTROL;
            refresh_page_static();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
        {
            command->target_shape = s_wired_target_shape;
            command->type = UI_COMMAND_START_WAVEFORM_CONTROL;
        }
    }
    else if (s_page == PAGE_AMPLITUDE_CONTROL)
    {
        if ((key == PS2_KEY_VALUE_0) ||
            (key == PS2_KEY_VALUE_1) ||
            (key == PS2_KEY_VALUE_3))
        {
            s_wired_target_shape =
                (key == PS2_KEY_VALUE_0) ? UI_TARGET_SHAPE_DIAGONAL :
                (key == PS2_KEY_VALUE_1) ? UI_TARGET_SHAPE_ELLIPSE :
                                           UI_TARGET_SHAPE_INFINITY;
            command->target_shape = s_wired_target_shape;
            command->target_span_y_div = s_target_span_y_div;
            command->type = UI_COMMAND_START_AMPLITUDE_CONTROL;
            refresh_page_static();
        }
        else if ((key == PS2_KEY_VALUE_2) ||
            (key == PS2_KEY_VALUE_4) ||
            (key == PS2_KEY_VALUE_6) ||
            (key == PS2_KEY_VALUE_8))
        {
            s_target_span_y_div = (uint8_t)key;
            s_measurement.amplitude_target_div = s_target_span_y_div;
            command->target_span_y_div = s_target_span_y_div;
            command->type = UI_COMMAND_UPDATE_AMPLITUDE_TARGET;
            refresh_page_static();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
        {
            command->target_shape = s_wired_target_shape;
            command->target_span_y_div = s_target_span_y_div;
            command->type = UI_COMMAND_START_AMPLITUDE_CONTROL;
        }
    }
    else if (s_page == PAGE_AUTOMATIC_CONTROL)
    {
        if (key == PS2_KEY_VALUE_5)
        {
            command->type = UI_COMMAND_TOGGLE_COMPLETE_OUTPUT;
        }
        else if ((key >= PS2_KEY_VALUE_1) && (key <= PS2_KEY_VALUE_3))
        {
            s_auto_target_shape = (uint8_t)key;
            command->target_shape = s_auto_target_shape;
            command->type = UI_COMMAND_START_AUTOMATIC_CONTROL;
            refresh_page_static();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
        {
            command->target_shape = s_auto_target_shape;
            command->type = UI_COMMAND_START_AUTOMATIC_CONTROL;
        }
    }
    else if (s_page == PAGE_DEBUG)
    {
        /*
         * 回车保留原来的PB10+PA1/PA2调试。
         * 数字1~4只控制AD9959 CH0，分别覆盖F题所需频段；
         * 数字0关闭DDS但仍停留在调试页面；
         * 数字5/6调整低端FTW，7/9调整相邻FTW混合比例，
         * 数字8开关相邻FTW抖动。
         */
        if (key == PS2_KEY_VALUE_ENTER)
        {
            command->type = UI_COMMAND_START_DEBUG;
        }
        else if ((key >= PS2_KEY_VALUE_1) &&
                 (key <= PS2_KEY_VALUE_4))
        {
            static const uint32_t test_frequency_hz[4] =
            {
                1000U, 10000U, 100000U, 200000U
            };

            command->frequency_hz =
                test_frequency_hz[(uint8_t)key - 1U];
            command->amplitude_mv = 118.0f;
            command->phase_deg = 0U;
            command->type = UI_COMMAND_START_AD9959_TEST;
        }
        else if (key == PS2_KEY_VALUE_0)
        {
            command->type = UI_COMMAND_STOP_AD9959_TEST;
        }
        else if ((key == PS2_KEY_VALUE_5) ||
                 (key == PS2_KEY_VALUE_6))
        {
            command->dds_fine_step =
                (key == PS2_KEY_VALUE_5) ? -1 : 1;
            command->type = UI_COMMAND_AD9959_ADJUST_FTW;
        }
        else if ((key == PS2_KEY_VALUE_7) ||
                 (key == PS2_KEY_VALUE_9))
        {
            command->dds_fine_step =
                (key == PS2_KEY_VALUE_7) ? -1 : 1;
            command->type =
                UI_COMMAND_AD9959_ADJUST_DITHER;
        }
        else if (key == PS2_KEY_VALUE_8)
        {
            command->type = UI_COMMAND_AD9959_TOGGLE_DITHER;
        }
    }

    return (command->type != UI_COMMAND_NONE);
}

/**
 * @brief  更新界面上的测量数据（由 InstrumentTask 通过回调触发）
 * @param  measurement 最新的测量结果结构体指针
 */
void UI_UpdateMeasurement(const UI_Measurement_t *measurement)
{
    if (measurement == NULL)
        return;

    s_measurement = *measurement;  /* 复制整个结构体 */

    /* 根据事件类型和当前页面决定刷新方式 */
    if (((measurement->event == UI_EVENT_MEASUREMENT) ||
         (measurement->event == UI_EVENT_CONTROL_PROGRESS)) &&
        (s_page >= PAGE_WAVEFORM_CONTROL) &&
        (s_page <= PAGE_DEBUG))
    {
        refresh_measurement_data();   /* 测量事件 → 刷新数字区域 */
    }
    else if ((measurement->event == UI_EVENT_STATUS) &&
             ((s_page == PAGE_MAIN) || (s_page == PAGE_DEBUG)))
    {
        refresh_page_data();          /* 状态事件 → 全页刷新 */
    }
    else if ((measurement->event == UI_EVENT_STATUS) &&
        (s_page >= PAGE_WAVEFORM_CONTROL) &&
        (s_page <= PAGE_DEBUG))
    {
        refresh_measurement_data();   /* 状态事件在测量页也刷新数据 */
    }
}

/**
 * @brief  旧版用户主循环（轮询 PS/2 键盘），在新 FreeRTOS 架构中已由 KeyTask 替代
 */
void User_main(void)
{
    PS2_KeyValue_t key;
    UI_Command_t command;

    UI_Init();
    for (;;)
    {
        PS2_KeyScan();
        key = PS2_GetCurrentKeyValue();
        if (key != PS2_KEY_VALUE_NULL)
        {
            (void)UI_HandleKey(key, &command);
        }
        HAL_Delay(10U);  /* 10 ms 轮询周期 */
    }
}

/**
 * @brief  初始化所有界面（等价于 UI_Init）
 */
void Init_All(void)
{
    UI_Init();
}

/**
 * @brief  强制返回主菜单并重绘整个屏幕
 */
void Disp_Main(void)
{
    s_page = PAGE_MAIN;
    render_main_page_after_exit();
}

/**
 * @brief  在指定位置显示一个浮点数值
 * @param  location 位置编号 1~10 左侧列，11~20 右侧列
 * @param  value    数值
 * @param  format   sprintf 格式字符串（与中文无关时中英文共用）
 */
void Show_Val(uint8_t location, float value, char *format)
{
    uint16_t x, y;

    if ((location > 0U) && (location <= 10U))
    {
        x = CONTENT_LEFT;
        y = (uint16_t)(96U + 32U * (location - 1U));
    }
    else if ((location > 10U) && (location <= 20U))
    {
        x = DATA_VALUE_X;
        y = (uint16_t)(96U + 32U * (location - 11U));
    }
    else
    {
        UI_String_Show(CONTENT_LEFT, 96U, 32U, 1U,
                       (const uint8_t *)"ERROR", "ERROR");
        return;
    }
    OS_Num_Show(x, y, 24U, 1U, (double)value, format, format);
}

/**
 * @brief  根据菜单编号切换页面（1~MenuChoiceNum 有效，其他返回主菜单）
 * @param  menu_sign 菜单项编号（1 起）
 */
void Change_Menu(uint8_t menu_sign)
{
    if ((menu_sign >= 1U) && (menu_sign <= MenuChoiceNum))
        switch_page((UiPage)menu_sign);
    else
        switch_page(PAGE_MAIN);
}

/* 以下六个函数为旧版菜单直接跳转接口，在新架构中可被按键处理替代 */

/** @brief 菜单项1：要求1软件闭环对角线。 */
void MenuHaddler_1(void) { switch_page(PAGE_WAVEFORM_CONTROL); }
/** @brief 菜单项2：要求2同频正交。 */
void MenuHaddler_2(void) { switch_page(PAGE_AMPLITUDE_CONTROL); }
/** @brief 菜单项3：要求3二倍频。 */
void MenuHaddler_3(void) { switch_page(PAGE_AUTOMATIC_CONTROL); }
/** @brief 菜单项4：要求4幅值档位。 */
void MenuHaddler_4(void) { switch_page(PAGE_DEBUG); }
/** @brief 菜单项5：要求5纯视觉自动。 */
void MenuHaddler_5(void) { switch_page(PAGE_MAIN); }
void MenuHaddler_6(void) { switch_page(PAGE_MAIN); }

/**
 * @brief  占位函数：PS/2 数字键盘输入解析（直接返回原值，未实现实际解析逻辑）
 * @param  num 输入数值
 * @return 原样返回
 */
float PS2_ReadNum(float num)
{
    return num;
}
