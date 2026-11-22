/**
 * @file    user.c
 * @brief   相位差测量仪用户界面（UI）实现
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

/* ---------- 页面索引 ---------- */
#define PAGE_MAIN       0U      /* 主菜单 */
#define PAGE_SINE       1U      /* 频率测量（正弦/方波） */
#define PAGE_KNOWN      2U      /* 已知模型相位测量 */
#define PAGE_UNKNOWN    3U      /* 未知模型 FFT 分析 */
#define PAGE_LEARNING   4U      /* 硬件误差学习与校准 */
#define PAGE_DIAGNOSTIC 5U      /* 全工程接线自检 */

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
static const uint8_t gbk_title[] =       /* "相位差测量仪" */
    "\xCF\xE0\xCE\xBB\xB2\xEE\xB2\xE2\xC1\xBF\xD2\xC7";
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
static void clear_right_panel(void)
{
    LCD_Appoint_Clear(252U, 72U, 800U, 440U, Black);
}

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
    /* 菜单项文字（中英文） */
    UI_String_Show(80U, 96U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice1, "1 FREQUENCY");
    UI_String_Show(80U, 160U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice2, "2 PHASE");
    UI_String_Show(80U, 224U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice3, "3 ADC/FFT");
    UI_String_Show(80U, 288U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice4, "4 LEARN CAL");
    UI_String_Show(80U, 352U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice5, "5 WIRING TEST");
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

/**
 * @brief  刷新频率测量页面的动态数据（频率、周期等）
 */
static void refresh_sine_measurement_values(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.frequency_hz_precise,
                "\xB2\xE2\xC1\xBF\xC6\xB5\xC2\xCA %.3f Hz      ",  // 测量频率
                "MEASURED FREQ %.3f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.period_ticks,
                "TIM2 CH3 %.0f ticks      ",
                "TIM2 CH3 CAPTURE %.0f ticks      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.capture_cycles,
                "\xB2\xB6\xBB\xF1\xD6\xDC\xC6\xDA %.0f      ",    // 捕获周期数
                "CAPTURE PERIODS %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 192U, 24U, 1U,
                (double)s_measurement.divider,
                "\xCD\xE2\xB2\xBF\xB7\xD6\xC6\xB5 1:%.0f      ",   // 外部分频
                "EXTERNAL DIVIDER 1:%.0f      ");
    /* 信号有效状态 */
    UI_String_Show(CONTENT_LEFT, 224U, 24U, 1U,
                   (const uint8_t *)((s_measurement.frequency_valid != 0U) ?
                       "STATUS SIGNAL VALID        " :
                       "STATUS WAIT SIGNAL         "),
                   (s_measurement.frequency_valid != 0U) ?
                       "STATUS SIGNAL VALID        " :
                       "STATUS WAIT SIGNAL         ");
}

/**
 * @brief  绘制频率测量页面（含静态提示）
 */
static void draw_sine_page(void)
{
    refresh_sine_measurement_values();
    UI_String_Show(CONTENT_LEFT, 276U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"INPUT PB10 / TIM2_CH3",
                   "INPUT PB10 / TIM2_CH3");
    UI_String_Show(CONTENT_LEFT, 300U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"ENTER RESTART CAPTURE",
                   "ENTER RESTART CAPTURE");
    UI_String_Show(CONTENT_LEFT, 324U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"USE 0-3.3V DIGITAL/SQUARE",
                   "USE 0-3.3V DIGITAL/SQUARE");
    UI_String_Show(CONTENT_LEFT, 348U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

/**
 * @brief  刷新已知模型相位测量页面的动态数据（相位差、幅度、FFT 等）
 */
static void refresh_known_measurement_values(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.phase_difference_deg,
                "\xCF\xE0\xCE\xBB\xB2\xEE%.3f deg      ",           // 相位差
                "PHASE %.3f deg      ");
    OS_Num_Show(DATA_VALUE_X, 96U, 24U, 1U,
                (double)s_measurement.frequency_hz,
                "\xC6\xB5\xC2\xCA%.0f Hz      ",                   // 频率
                "FREQ %.0f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.channel1_amplitude,
                "CH1 \xB7\xF9\xD6\xB5%.4f      ",                  // CH1 幅值
                "CH1 AMP %.4f      ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.channel2_amplitude,
                "CH2 \xB7\xF9\xD6\xB5%.4f      ",                  // CH2 幅值
                "CH2 AMP %.4f      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.selected_bin,
                "FFT\xC6\xB5\xC2\xCA\xB5\xA5\xD4\xAA %.0f      ",  // FFT 频率单元
                "FFT BIN %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                (double)s_measurement.phase_valid,
                "\xD3\xD0\xD0\xA7 %.0f      ", "VALID %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 192U, 20U, 1U,
                (double)s_measurement.adc_sample_rate_hz,
                "ADC FS %.0f Hz      ", "ADC FS %.0f Hz      ");
    OS_Num_Show(DATA_VALUE_X, 192U, 20U, 1U,
                (double)s_measurement.local_oscillator_hz,
                "LO %.0f Hz      ", "LO %.0f Hz      ");
}

/**
 * @brief  绘制已知模型测量页面（含提示）
 */
static void draw_known_page(void)
{
    refresh_known_measurement_values();
    UI_String_Show(CONTENT_LEFT, 220U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"ADC1+ADC2\xCD\xAC\xB2\xBD DMA", // ADC1+ADC2同步
                   "ADC1+ADC2 SYNCHRONIZED DMA");
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP");
}

/**
 * @brief  刷新未知模型 FFT 测量页面的动态数据
 */
static void refresh_unknown_measurement_values(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.adc_overrun_count,
                "DMA\xD2\xE7\xB3\xF6 %.0f      ",            // DMA溢出
                "DMA OVERRUN %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 96U, 24U, 1U,
                (double)s_measurement.selected_bin,
                "FFT BIN %.0f      ", "FFT BIN %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.channel1_amplitude,
                "CH1 %.4f      ", "CH1 AMP %.4f      ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.channel2_amplitude,
                "CH2 %.4f      ", "CH2 AMP %.4f      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.frequency_hz,
                "\xC6\xB5\xC2\xCA%.0f Hz      ", "FREQ %.0f Hz      ");
    OS_Num_Show(DATA_VALUE_X, 160U, 20U, 1U,
                (double)s_measurement.adc_sample_rate_hz,
                "FS %.0f Hz      ", "FS %.0f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 192U, 20U, 1U,
                (double)s_measurement.adc_mode,
                "ADC MODE %.0f      ", "ADC MODE %.0f      ");
}

/**
 * @brief  绘制未知模型测量页面（含提示）
 */
static void draw_unknown_page(void)
{
    refresh_unknown_measurement_values();
    UI_String_Show(CONTENT_LEFT, 220U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"AUTO FS / 256 POINT DFT",
                   "AUTO FS / 256 POINT DFT");
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP");
}

static void refresh_learning_values(void)
{
    uint32_t display_point =
        (uint32_t)s_measurement.learning_point_index + 1U;

    if (display_point >
        (uint32_t)s_measurement.learning_point_total)
    {
        display_point =
            (uint32_t)s_measurement.learning_point_total;
    }

    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.learning_target_frequency_hz,
                "TARGET %.0f Hz      ", "TARGET %.0f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)display_point,
                "POINT %.0f      ", "POINT %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.learning_point_total,
                "TOTAL %.0f      ", "TOTAL %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.learning_sample_count,
                "SAMPLES %.0f/8      ", "SAMPLES %.0f/8      ");
    OS_Num_Show(CONTENT_LEFT, 192U, 20U, 1U,
                (double)s_measurement.learned_phase_error_deg,
                "PHASE ERR %.3f deg      ",
                "PHASE ERR %.3f deg      ");
    OS_Num_Show(DATA_VALUE_X, 192U, 20U, 1U,
                (double)s_measurement.learned_gain_error_db,
                "GAIN ERR %.3f dB      ",
                "GAIN ERR %.3f dB      ");
    OS_Num_Show(CONTENT_LEFT, 224U, 20U, 1U,
                (double)s_measurement.calibration_valid,
                "CAL VALID %.0f      ", "CAL VALID %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 224U, 20U, 1U,
                (double)s_measurement.learning_save_ok,
                "SAVED %.0f      ", "SAVED %.0f      ");
}

static void draw_learning_page(void)
{
    refresh_learning_values();
    UI_String_Show(CONTENT_LEFT, 268U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"AD9959 CH1 -> INPUT CH1",
                   "AD9959 CH1 -> INPUT CH1");
    UI_String_Show(CONTENT_LEFT, 292U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"AD9959 CH0 -> INPUT CH2",
                   "AD9959 CH0 -> INPUT CH2");
    UI_String_Show(CONTENT_LEFT, 316U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"ENTER START/RESTART",
                   "ENTER START/RESTART");
    UI_String_Show(CONTENT_LEFT, 340U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK ABORT/MAIN");
}

/**
 * @brief  绘制全工程接线自检页面
 */
static void refresh_diagnostic_values(void)
{
    uint32_t display_case =
        (uint32_t)s_measurement.wiring_case_index + 1U;

    if (display_case > (uint32_t)s_measurement.wiring_case_total)
    {
        display_case = s_measurement.wiring_case_total;
    }
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)display_case,
                "CASE %.0f      ", "CASE %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 96U, 24U, 1U,
                (double)s_measurement.wiring_case_total,
                "TOTAL %.0f      ", "TOTAL %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.wiring_pass_count,
                "PASS %.0f      ", "PASS %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.wiring_fail_count,
                "FAIL %.0f      ", "FAIL %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.wiring_active,
                "ACTIVE %.0f      ", "ACTIVE %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                (double)s_measurement.wiring_done,
                "DONE %.0f      ", "DONE %.0f      ");
    UI_String_Show(CONTENT_LEFT, 200U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)s_measurement.status_text,
                   s_measurement.status_text);
}

static void draw_diagnostic_page(void)
{
    refresh_diagnostic_values();
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"USART6 PC6/PC7 230400",
                   "SERIAL USART6 PC6/PC7 230400");
    UI_String_Show(CONTENT_LEFT, 276U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"4 CASES: RAW DIV32 MIXED MAP",
                   "4 CASES: RAW DIV32 MIXED MAP");
    UI_String_Show(CONTENT_LEFT, 308U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"ENTER START/RESTART",
                   "ENTER START/RESTART");
    UI_String_Show(CONTENT_LEFT, 340U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

/**
 * @brief  根据当前页面索引绘制右侧内容区域
 */
static void draw_page_content(void)
{
    if (s_page == PAGE_MAIN)
        draw_main_page();
    else if (s_page == PAGE_SINE)
        draw_sine_page();
    else if (s_page == PAGE_KNOWN)
        draw_known_page();
    else if (s_page == PAGE_UNKNOWN)
        draw_unknown_page();
    else if (s_page == PAGE_LEARNING)
        draw_learning_page();
    else
        draw_diagnostic_page(); /* PAGE_DIAGNOSTIC */
}

/**
 * @brief  完整重绘当前页面（清除右侧 + 绘制内容 + 更新菜单高亮）
 */
static void render_page(void)
{
    clear_right_panel();
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
    if (s_page == PAGE_SINE)
        refresh_sine_measurement_values();
    else if (s_page == PAGE_KNOWN)
        refresh_known_measurement_values();
    else if (s_page == PAGE_UNKNOWN)
        refresh_unknown_measurement_values();
    else if (s_page == PAGE_LEARNING)
        refresh_learning_values();
    else if (s_page == PAGE_DIAGNOSTIC)
        refresh_diagnostic_values();
}

/**
 * @brief  切换到指定页面，必要时进行参数限幅和状态重置
 * @param  next_page 目标页面索引
 */
static void switch_page(UiPage next_page)
{
    if (next_page <= PAGE_DIAGNOSTIC)
    {
        /* 进入频率/相位页面前确保频率在有效范围内 */
        if ((next_page == PAGE_KNOWN) || (next_page == PAGE_SINE))
        {
            if (s_frequency_hz < UI_MIN_FREQUENCY_HZ)
                s_frequency_hz = UI_MIN_FREQUENCY_HZ;
            if (s_frequency_hz > UI_MAX_FREQUENCY_HZ)
                s_frequency_hz = UI_MAX_FREQUENCY_HZ;
        }
        s_page = next_page;
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
    (void)memset(&s_measurement, 0, sizeof(s_measurement));
    s_measurement.frequency_hz = s_frequency_hz;
    s_measurement.frequency_hz_precise = 0.0f;
    s_measurement.divider = 1U;
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
    command->calibration_item = s_calibration_item;

    /* 退格键：停止当前操作并返回主菜单 */
    if (key == PS2_KEY_VALUE_BACKSPACE)
    {
        command->type = (s_page == PAGE_MAIN) ?
                        UI_COMMAND_NONE : UI_COMMAND_STOP_OUTPUT;
        switch_page(PAGE_MAIN);
        return (command->type != UI_COMMAND_NONE);
    }

    /* 主菜单：数字键 1~5 直接进入对应页面并产生启动命令 */
    if ((s_page == PAGE_MAIN) &&
        (key >= PS2_KEY_VALUE_1) && (key <= PS2_KEY_VALUE_5))
    {
        switch_page((UiPage)key);
        if (key == PS2_KEY_VALUE_1)
            command->type = UI_COMMAND_START_FREQUENCY_MEASUREMENT;
        else if (key == PS2_KEY_VALUE_2)
            command->type = UI_COMMAND_RUN_KNOWN_MODEL;
        else if (key == PS2_KEY_VALUE_3)
            command->type = UI_COMMAND_START_UNKNOWN_MODEL;
        else if (key == PS2_KEY_VALUE_4)
            command->type = UI_COMMAND_START_LEARNING;
        else if (key == PS2_KEY_VALUE_5)
            command->type = UI_COMMAND_START_WIRING_TEST;
        return (command->type != UI_COMMAND_NONE);
    }

    /* 各子页面按键处理 */
    if (s_page == PAGE_SINE)
    {
        if (key == PS2_KEY_VALUE_ENTER)
            command->type = UI_COMMAND_START_FREQUENCY_MEASUREMENT;
    }
    else if (s_page == PAGE_KNOWN)
    {
        if (key == PS2_KEY_VALUE_ADD)
        {
            /* 增加相位设置值（步进 10°） */
            s_phase_deg = (s_phase_deg < 350U) ? s_phase_deg + 10U : 360U;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MINUS)
        {
            s_phase_deg = (s_phase_deg > 10U) ? s_phase_deg - 10U : 0U;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
            command->type = UI_COMMAND_RUN_KNOWN_MODEL;
    }
    /* 其他页面中回车键的处理 */
    else if (key == PS2_KEY_VALUE_ENTER)
    {
        if (s_page == PAGE_UNKNOWN)
            command->type = UI_COMMAND_START_UNKNOWN_MODEL;
        else if (s_page == PAGE_LEARNING)
            command->type = UI_COMMAND_START_LEARNING;
        else if (s_page == PAGE_DIAGNOSTIC)
            command->type = UI_COMMAND_START_WIRING_TEST;
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
    if ((measurement->event == UI_EVENT_MEASUREMENT) &&
        ((s_page == PAGE_SINE) || (s_page == PAGE_KNOWN) ||
         (s_page == PAGE_UNKNOWN) || (s_page == PAGE_LEARNING) ||
         (s_page == PAGE_DIAGNOSTIC)))
    {
        refresh_measurement_data();   /* 测量事件 → 刷新数字区域 */
    }
    else if ((measurement->event == UI_EVENT_STATUS) &&
             ((s_page == PAGE_MAIN) || (s_page == PAGE_DIAGNOSTIC)))
    {
        refresh_page_data();          /* 状态事件 → 全页刷新 */
    }
    else if ((measurement->event == UI_EVENT_STATUS) &&
        ((s_page == PAGE_SINE) || (s_page == PAGE_KNOWN) ||
         (s_page == PAGE_UNKNOWN) || (s_page == PAGE_LEARNING) ||
         (s_page == PAGE_DIAGNOSTIC)))
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
    LCD_Clear(Black);
    draw_reference_frame();
    draw_page_content();
    draw_menu_selection();
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

/* 以下五个函数为旧版菜单直接跳转接口，在新架构中可被按键处理替代 */

/** @brief 菜单项 1：进入频率测量页面 */
void MenuHaddler_1(void) { switch_page(PAGE_SINE); }
/** @brief 菜单项 2：进入相位测量页面 */
void MenuHaddler_2(void) { switch_page(PAGE_KNOWN); }
/** @brief 菜单项 3：进入 FFT 分析页面 */
void MenuHaddler_3(void) { switch_page(PAGE_UNKNOWN); }
/** @brief 菜单项 4：进入学习校准页面 */
void MenuHaddler_4(void) { switch_page(PAGE_LEARNING); }
/** @brief 菜单项 5：进入接线自检页面 */
void MenuHaddler_5(void) { switch_page(PAGE_DIAGNOSTIC); }

/**
 * @brief  占位函数：PS/2 数字键盘输入解析（直接返回原值，未实现实际解析逻辑）
 * @param  num 输入数值
 * @return 原样返回
 */
float PS2_ReadNum(float num)
{
    return num;
}
