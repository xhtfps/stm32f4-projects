/* Circuit-model UI using the original four-page menu framework. */
#include "user.h"

#include <stdio.h>
#include <string.h>

#include "./BSP/TFT_LCD/TFT_LCD.h"
#include "./BSP/TFT_LCD/text.h"

/* ================================================================
 * 界面宏定义
 * ================================================================ */
#define CONTENT_LEFT    314U          /* 右侧内容区域起始 X 坐标 */
#define DATA_VALUE_X    564U          /* 数据数值靠右 X 坐标   */
#define PAGE_MAIN       0U            /* 主菜单首页 */
#define PAGE_SINE       1U            /* 正弦信号设置页 */
#define PAGE_KNOWN      2U            /* 已知电路测量页 */
#define PAGE_UNKNOWN    3U            /* 未知电路辨识页 */
#define PAGE_OUTPUT     4U            /* IIR滤波输出页 */
#define PAGE_ADS_TEST   5U            /* ADS1256 调试页面 */
#define UI_MENU_SIZE    24U           /* 侧边菜单字号 */
#define UI_PRIMARY_SIZE 24U           /* 主内容字号 */
#define UI_HELP_SIZE    16U           /* 提示小字字号 */

typedef uint8_t UiPage;               /* 页面类型枚举 */

/* ================================================================
 * GBK 编码中文字符串常量（替换所有问号乱码）
 * ================================================================ */
static const u8 gbk_title[]        = "\xB5\xE7\xC2\xB7\xC4\xA3\xD0\xCD\xB2\xE2\xC1\xBF\xCF\xB5\xCD\xB3";
static const u8 gbk_sine[]         = "1 \xD5\xFD\xCF\xD2\xD0\xC5\xBA\xC5\xD4\xB4";
static const u8 gbk_known[]        = "2 \xD2\xD1\xD6\xAA\xB5\xE7\xC2\xB7\xB2\xE2\xCA\xD4";
static const u8 gbk_unknown[]      = "3 \xCE\xB4\xD6\xAA\xB5\xE7\xC2\xB7\xB1\xE6\xCA\xB6";
static const u8 gbk_output[]       = "4 IIR\xC2\xCB\xB2\xA8\xCA\xE4\xB3\xF6";
static const u8 gbk_ads_test[]     = "\xD0\xA3\xD7\xBC\xB2\xE2\xCA\xD4";
static const u8 gbk_dash[]         = "\xA1\xAA";
static const u8 gbk_selected[]     = "\xA1\xF4";
static const u8 gbk_select[]       = "\xC7\xEB\xD1\xA1\xD4\xF1\xB9\xA6\xC4\xDC\xB2\xCB\xB5\xA5";
static const u8 gbk_inference[]    = "\xC2\xCB\xB2\xA8IIR";
static const u8 gbk_wait[]         = "\xB5\xC8\xB4\xFD\xBC\xC6\xCB\xE3...";
static const u8 gbk_run[]          = "\xB0\xB4\xCF\xC2\xBB\xD8\xB3\xB5\xC6\xF4\xB6\xAF\xD4\xCB\xD0\xD0";
static const u8 gbk_adjust_freq[]  = "+/- \xB5\xF7\xBD\xDA\xC6\xB5\xC2\xCA""100Hz";
static const u8 gbk_adjust_amp[]   = "*/ \xB5\xF7\xBD\xDA\xB7\xF9\xD6\xB5""10mV";
static const u8 gbk_back[]         = "\xB7\xB5\xBB\xD8\xD6\xF7\xB2\xCB\xB5\xA5\xCD\xA3\xD6\xB9\xCA\xE4\xB3\xF6";

/* ================================================================
 * 全局状态变量
 * ================================================================ */
static UiPage s_page = PAGE_MAIN;               /* 当前页面 */
static uint32_t s_frequency_hz = UI_DEFAULT_FREQUENCY_HZ; /* 输出频率 (Hz) */
static float s_amplitude_mv = UI_DEFAULT_AMPLITUDE_MV;    /* 输出幅值 (mV) */
static uint32_t s_phase_deg = UI_DEFAULT_PHASE_DEG;       /* 相位 (度)  */
static float s_known_target_peak_v = UI_DEFAULT_KNOWN_TARGET_PEAK_V;
static uint8_t s_chinese_font_available;                 /* 汉字字库可用标志 */
static UI_Measurement_t s_measurement;                   /* 测量数据结构体 */
static uint8_t s_calibration_item = UI_CALIBRATION_ADS_REFERENCE;

/* ================================================================
 * 纯 ASCII 字符点阵显示底层函数
 * 参考 OS_UI.c 中的 asc2_1608/2412/3216 字库
 * ================================================================ */
static void OS_Char_Show(uint16_t x, uint16_t y, uint8_t num,
                         uint8_t size, uint8_t mode)
{
    uint8_t temp;
    uint8_t t1;
    uint8_t t;
    uint16_t y0 = y;
    uint8_t csize;

    /* 计算点阵占用字节数 */
    csize = (uint8_t)((size / 8U + ((size % 8U) ? 1U : 0U)) *
                      (size / 2U));
    /* 校验ASCII可打印范围 */
    if ((num < (uint8_t)' ') || (num > (uint8_t)'~'))
    {
        num = (uint8_t)' ';
    }
    else
    {
        num = (uint8_t)(num - (uint8_t)' ');
    }

    for (t = 0U; t < csize; ++t)
    {
        /* 根据字号选对应ASCII点阵 */
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
            return;
        }

        /* 逐位打点绘制字符 */
        for (t1 = 0U; t1 < 8U; ++t1)
        {
            if ((temp & 0x80U) != 0U)
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(TextColor);      /* 前景色 */
            }
            else if (mode == 1U)               /* mode=1 反色背景 */
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(BackColor);      /* 背景色 */
            }
            temp <<= 1U;
            ++y;
            if (y >= 480U)
            {
                return;
            }
            if ((uint16_t)(y - y0) == size)
            {
                y = y0;
                ++x;
                if (x >= 800U)
                {
                    return;
                }
                break;
            }
        }
    }
}

/* 汉字/GBK 调用底层字库绘制 */
static void OS_Font_Show(uint16_t x, uint16_t y, char *font,
                         uint8_t size, uint8_t mode)
{
    LCD_ShowGBKFont(x, y, (u8 *)font, size, mode);
}

/* 中英文混排字符串解析显示 自动识别 GBK 汉字 */
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
            /* 判断是否为 GBK 汉字首字节（最高位为1即负数） */
            if ((int8_t)*str < 0)
            {
                chinese = 1U;
            }
            else
            {
                /* ASCII字符 */
                if (*str == '\n')
                {
                    y = (uint16_t)(y + size);
                    x = x0;
                    ++str;
                    continue;
                }
                OS_Char_Show(x, y, (uint8_t)*str, size, mode);
                ++str;
                x = (uint16_t)(x + size / 2U);
            }
        }
        else
        {
            chinese = 0U;
            OS_Font_Show(x, y, str, size, mode);
            str += 2;                         /* 汉字占两字节 */
            x = (uint16_t)(x + size);
        }
    }
}

/* 纯 ASCII 英文字符串显示 */
static void OS_Ascii_Show(uint16_t x, uint16_t y, uint8_t size,
                          uint8_t mode, const char *str)
{
    if (str == NULL)
    {
        return;
    }

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

/* 双模式显示：有汉字库显示 GBK 中文，无则显示 ASCII 英文兜底 */
static void UI_String_Show(uint16_t x, uint16_t y, uint8_t size,
                           uint8_t mode, const u8 *gbk_text,
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

/* 浮点数格式化数字显示 */
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

    if (s_chinese_font_available != 0U)
    {
        OS_String_Show(x, y, size, mode, buffer);
    }
    else
    {
        OS_Ascii_Show(x, y, size, mode, buffer);
    }
}

/* 清空右侧内容面板 */
static void clear_right_panel(void)
{
    LCD_Appoint_Clear(252U, 72U, 800U, 440U, Black);
}

/* 绘制参考框架与固定界面 */
static void draw_reference_frame(void)
{
    uint8_t count;

    LCD_SetColors(White, Black);
    UI_String_Show(272U, 16U, 32U, 0U, gbk_title, "CIRCUIT MODEL");
    /* 绘制分割线边框 */
    LCD_Appoint_Clear(0U, 64U, 800U, 72U, White);
    LCD_Appoint_Clear(0U, 440U, 800U, 448U, White);
    LCD_Appoint_Clear(250U, 72U, 252U, 440U, White);
    /* 底部版本号 */
    UI_String_Show(32U, 456U, 16U, 0U,
                   (const u8 *)ModelVerStr, ModelVerStr);
    UI_String_Show(632U, 456U, 16U, 0U,
                   (const u8 *)UserVerStr, UserVerStr);

    /* 侧边菜单前缀横线 */
    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 0U,
                       gbk_dash, "-");
    }
    UI_String_Show(80U, 96U, UI_MENU_SIZE, 0U,
                   gbk_sine, "1 SINE");
    UI_String_Show(80U, 160U, UI_MENU_SIZE, 0U,
                   gbk_known, "2 KNOWN");
    UI_String_Show(80U, 224U, UI_MENU_SIZE, 0U,
                   gbk_unknown, "3 UNKNOWN");
    UI_String_Show(80U, 288U, UI_MENU_SIZE, 0U,
                   gbk_output, "4 IIR OUTPUT");
    UI_String_Show(80U, 352U, UI_MENU_SIZE, 0U,
                   gbk_ads_test, "5 CALIBRATION");
}

/* 绘制侧边菜单选中标记 箭头 反色 */
static void draw_menu_selection(void)
{
    uint8_t count;

    /* 重绘侧边横线 */
    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 1U,
                       gbk_dash, "-");
    }
    /* 当前页选中箭头 */
    if (s_page != PAGE_MAIN)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * s_page), 32U, 1U,
                       gbk_selected, ">");
    }
}

/* ---------- 各页面内容绘制函数 ---------- */
static void draw_main_page(void)
{
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   gbk_select, "SELECT MENU");
    UI_String_Show(CONTENT_LEFT, 160U, UI_HELP_SIZE, 1U,
                   gbk_run, "ENTER RUN");
    UI_String_Show(CONTENT_LEFT, 184U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

static void draw_sine_page(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_amplitude_mv,
                "\xCA\xE4\xB3\xF6\xB7\xF9\xD6\xB5%.1f mV      ", "SOURCE %.1f mV      ");
    OS_Num_Show(DATA_VALUE_X, 96U, 24U, 1U,
                (double)s_frequency_hz,
                "\xC6\xB5\xC2\xCA%.0f Hz      ", "FREQ %.0f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_phase_deg,
                "\xCF\xE0\xCE\xBB%.0f deg      ", "PHASE %.0f deg      ");
    UI_String_Show(CONTENT_LEFT, 220U, UI_HELP_SIZE, 1U,
                   gbk_adjust_freq, "+/- FREQ 100Hz");
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   gbk_adjust_amp, "*/ AMPL 10mV");
    UI_String_Show(CONTENT_LEFT, 268U, UI_HELP_SIZE, 1U,
                   gbk_run, "ENTER OUTPUT");
    UI_String_Show(CONTENT_LEFT, 292U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP");
}

static void draw_known_page(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_known_target_peak_v,
                "\xC4\xBF\xB1\xEAVpp %.1f V      ", "TARGET VPP %.1f V      ");
    OS_Num_Show(DATA_VALUE_X, 96U, 24U, 1U,
                (double)s_measurement.output_peak_v,
                "\xCA\xB5\xB2\xE2Vpp %.3f V      ", "MEASURED VPP %.3f V      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.input_rms,
                "\xCA\xE4\xC8\xEB\xD3\xD0\xD0\xA7\xD6\xB5%.3f V      ", "INPUT RMS %.3f V      ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.output_rms,
                "\xCA\xE4\xB3\xF6\xD3\xD0\xD0\xA7\xD6\xB5%.3f V      ", "OUTPUT RMS %.3f V      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.expected_output_peak_v,
                "\xC0\xED\xC2\xDBVpp %.3f V      ", "MODEL VPP %.3f V      ");
    OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                (double)s_measurement.gain_db,
                "\xD4\xF6\xD2\xE6%.2f dB      ", "GAIN %.2f dB      ");
    OS_Num_Show(CONTENT_LEFT, 192U, 24U, 1U,
                (double)s_measurement.source_amplitude_mv,
                "DDS\xB7\xF9\xD6\xB5%.1f mV      ", "DDS %.1f mV      ");
    OS_Num_Show(DATA_VALUE_X, 192U, 24U, 1U,
                (double)s_frequency_hz,
                "\xC6\xB5\xC2\xCA%.0f Hz      ", "FREQ %.0f Hz      ");
    UI_String_Show(CONTENT_LEFT, 248U, UI_HELP_SIZE, 1U,
                   (const u8 *)"\xC6\xB5\xC2\xCA""100~3000Hz\xA3\xAC\xB2\xBD\xBD\xF8""100Hz",
                   "FREQ 100..3000Hz STEP 100Hz");
    UI_String_Show(CONTENT_LEFT, 272U, UI_HELP_SIZE, 1U,
                   (const u8 *)"\xC4\xBF\xB1\xEAVpp 1.0~2.0V\xA3\xAC\xB2\xBD\xBD\xF8""0.1V",
                   "TARGET VPP 1.0..2.0V STEP 0.1V");
    UI_String_Show(CONTENT_LEFT, 296U, UI_HELP_SIZE, 1U,
                   (const u8 *)"\xBB\xD8\xB3\xB5\xC6\xF4\xB6\xAF\xA3\xAC\xB7\xB5\xBB\xD8\xCD\xA3\xD6\xB9",
                   "ENTER START; BACK STOP");
    UI_String_Show(CONTENT_LEFT, 320U, UI_HELP_SIZE, 1U,
                   (const u8 *)"PLUS/MINUS FREQ; STAR/SLASH TARGET VPP",
                   "PLUS/MINUS FREQ; STAR/SLASH TARGET VPP");
}

static void draw_unknown_page(void)
{
    char progress[24];
    char model_type[40];

    /* 辨识完成显示电路参数结果 */
    if (s_measurement.event == UI_EVENT_MODEL_RESULT)
    {
        (void)sprintf(model_type, "%s        ",
                      CircuitModel_TypeName(s_measurement.model_type));
        UI_String_Show(CONTENT_LEFT, 96U, 24U, 1U,
                       (const u8 *)"\xB5\xE7\xC2\xB7\xC0\xE0\xD0\xCD\xA3\xBA", "TYPE:");
        UI_String_Show(DATA_VALUE_X, 96U, 24U, 1U,
                       (const u8 *)model_type, model_type);
        OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                    (double)s_measurement.f1_hz,
                    "\xBD\xD8\xD6\xB9\xC6\xB5\xC2\xCA""1:%.1f Hz      ", "F1 %.1f Hz      ");
        OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                    (double)s_measurement.f2_hz,
                    "\xBD\xD8\xD6\xB9\xC6\xB5\xC2\xCA""2:%.1f Hz      ", "F2 %.1f Hz      ");
        OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                    (double)s_measurement.center_hz,
                    "\xD6\xD0\xD0\xC4\xC6\xB5\xC2\xCA:%.1f Hz      ", "CENTER %.1f Hz      ");
        OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                    (double)s_measurement.q,
                    "\xC6\xB7\xD6\xCA\xD2\xF2\xCA\xFDQ:%.3f      ", "Q %.3f      ");
    }
    else  /* 扫频进行中显示进度 */
    {
        (void)sprintf(progress, "%3u/%3u      ",
                      (unsigned int)s_measurement.sweep_index,
                      (unsigned int)s_measurement.sweep_total);
        UI_String_Show(CONTENT_LEFT, 96U, 24U, 1U,
                       ((const u8 *)"SWEEP"), "SWEEP");
        UI_String_Show(DATA_VALUE_X, 96U, 24U, 1U,
                       (const u8 *)progress, progress);
        OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                    (double)s_measurement.input_rms,
                    "AIN0\xD3\xD0\xD0\xA7\xD6\xB5%.3f V      ", "AIN0 %.3f V      ");
        OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                    (double)s_measurement.output_rms,
                    "AIN1\xD3\xD0\xD0\xA7\xD6\xB5%.3f V      ", "AIN1 %.3f V      ");
        OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                    (double)s_measurement.gain_db,
                    "\xD4\xF6\xD2\xE6%.2f dB      ", "GAIN %.2f dB      ");
        OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                    (double)s_measurement.frequency_hz,
                    "\xC6\xB5\xC2\xCA%.0f Hz      ", "FREQ %.0f Hz      ");
    }
    UI_String_Show(CONTENT_LEFT, 220U, UI_HELP_SIZE, 1U,
                   gbk_run, "ENTER START SWEEP");
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP");
}

static void draw_output_page(void)
{
    char model_type[40];

    (void)sprintf(model_type, "%s        ",
                  CircuitModel_TypeName(s_measurement.model_type));
    UI_String_Show(CONTENT_LEFT, 96U, 24U, 1U,
                   (const u8 *)"\xB5\xE7\xC2\xB7\xC0\xE0\xD0\xCD\xA3\xBA", "TYPE:");
    UI_String_Show(DATA_VALUE_X, 96U, 24U, 1U,
                   (const u8 *)model_type, model_type);
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.center_hz,
                "\xD6\xD0\xD0\xC4\xC6\xB5\xC2\xCA:%.1f Hz      ", "CENTER %.1f Hz      ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.f1_hz,
                "\xBD\xD8\xD6\xB9\xC6\xB5\xC2\xCA""1:%.1f Hz      ", "F1 %.1f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.f2_hz,
                "\xBD\xD8\xD6\xB9\xC6\xB5\xC2\xCA""2:%.1f Hz      ", "F2 %.1f Hz      ");
    OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                (double)s_measurement.q,
                "\xC6\xB7\xD6\xCA\xD2\xF2\xCA\xFDQ:%.3f      ", "Q %.3f      ");
    UI_String_Show(CONTENT_LEFT, 220U, UI_HELP_SIZE, 1U,
                   (s_measurement.valid != 0U) ? gbk_inference : gbk_wait,
                   (s_measurement.valid != 0U) ? "FILTER READY" : "WAIT MODEL");
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

static void draw_ads_calibration_page(void)
{
    uint8_t item;
    uint16_t y;
    char buffer[64];
    const char *label;

    UI_String_Show(CONTENT_LEFT, 76U, 24U, 1U,
                   (const u8 *)"CALIBRATION MENU", "CALIBRATION MENU");
    for (item = 1U; item <= 6U; ++item)
    {
        switch (item)
        {
            case UI_CALIBRATION_ADS_REFERENCE:
                label = "ADS REF/RAW TEST";
                break;
            case UI_CALIBRATION_ZERO_OFFSET:
                label = "ZERO OFFSET TEST";
                break;
            case UI_CALIBRATION_CHANNEL_GAIN:
                label = "CHANNEL GAIN TEST";
                break;
            case UI_CALIBRATION_SWEEP:
                label = "FULL CIRCUIT SWEEP";
                break;
            case UI_CALIBRATION_IIR_DEBUG:
                label = "MENU4 IIR DEBUG";
                break;
            case UI_CALIBRATION_DAC_TEST:
                 label = "DAC/ADC MIRROR TEST";
                break;
            default:
                label = "UNKNOWN ITEM";
                break;
        }

        y = (uint16_t)(108U + (uint16_t)(item - 1U) * 34U);
        (void)sprintf(buffer, "%c %u %s",
                      (item == s_calibration_item) ? '>' : ' ',
                      (unsigned int)item, label);
        UI_String_Show(CONTENT_LEFT, y, 16U, 1U,
                       (const u8 *)buffer, buffer);
    }

    UI_String_Show(CONTENT_LEFT, 320U, 16U, 1U,
                   (const u8 *)"UART6 PC6/PC7 230400", "UART6 PC6/PC7 230400");
    UI_String_Show(CONTENT_LEFT, 350U, 16U, 1U,
                   (const u8 *)"+/- SELECT", "+/- SELECT");
    UI_String_Show(CONTENT_LEFT, 372U, 16U, 1U,
                   (const u8 *)"ENTER RUN UART TEST", "ENTER RUN UART TEST");
    UI_String_Show(CONTENT_LEFT, 394U, 16U, 1U,
                   (const u8 *)"BACK MAIN", "BACK MAIN");
}

/* Legacy ADS display retained for source compatibility; menu5 no longer calls it. */
#if 0
static void draw_ads_test_page(void)
{
    char buffer[80];
    char result_text[32];
    int32_t raw_delta;
    int32_t ain0_span;
    int32_t ain1_span;
    uint8_t registers_read_ok;
    uint8_t registers_match;
    uint8_t overall_ok;

    registers_read_ok =
        (s_measurement.ads_status_reg != 0xFFU) &&
        (s_measurement.ads_mux_reg != 0xFFU) &&
        (s_measurement.ads_adcon_reg != 0xFFU) &&
        (s_measurement.ads_drate_reg != 0xFFU);
    /* STATUS checks ID=3 and ORDER/ACAL/BUFEN=0/1/0; DRDY is dynamic. */
    registers_match =
        ((s_measurement.ads_status_reg & 0xF0U) == 0x30U) &&
        ((s_measurement.ads_status_reg & 0x0EU) == 0x04U) &&
        (s_measurement.ads_mux_reg == 0x08U) &&
        (s_measurement.ads_adcon_reg == 0x00U) &&
        (s_measurement.ads_drate_reg == 0xA1U);
    raw_delta = s_measurement.ads_ain1_raw - s_measurement.ads_ain0_raw;
    ain0_span = (int32_t)(s_measurement.ads_ain0_max_raw - s_measurement.ads_ain0_min_raw);
    ain1_span = (int32_t)(s_measurement.ads_ain1_max_raw - s_measurement.ads_ain1_min_raw);
    overall_ok = (s_measurement.ads_init_ok != 0U &&
                  registers_read_ok != 0U && registers_match != 0U &&
                  s_measurement.valid != 0U) ? 1U : 0U;

    if (s_measurement.ads_init_ok == 0U)
    {
        (void)sprintf(result_text, "INIT FAIL");
    }
    else if (registers_read_ok == 0U)
    {
        (void)sprintf(result_text, "REG READ FAIL");
    }
    else if (registers_match == 0U)
    {
        (void)sprintf(result_text, "REG VALUE FAIL");
    }
    else if (s_measurement.valid == 0U)
    {
        if (s_measurement.ads_ain0_samples_ok == 0U)
        {
            (void)sprintf(result_text, "AIN0 READ FAIL");
        }
        else if (s_measurement.ads_ain1_samples_ok == 0U)
        {
            (void)sprintf(result_text, "AIN1 READ FAIL");
        }
        else
        {
            (void)sprintf(result_text, "ADC REPEAT FAIL");
        }
    }
    else if ((s_measurement.ads_ain0_raw > -100000L) &&
             (s_measurement.ads_ain0_raw < 100000L) &&
             (s_measurement.ads_ain1_raw > -100000L) &&
             (s_measurement.ads_ain1_raw < 100000L))
    {
        (void)sprintf(result_text, "ADC OK LOW LEVEL");
    }
    else
    {
        (void)sprintf(result_text, "ADC SPI OK");
    }

    UI_String_Show(CONTENT_LEFT, 76U, 16U, 1U,
                   (const u8 *)"ADS1256 SELF TEST", "ADS1256 SELF TEST");
    UI_String_Show(DATA_VALUE_X, 76U, 16U, 1U,
                   (const u8 *)((overall_ok != 0U) ? "PASS" : "CHECK"),
                   (overall_ok != 0U) ? "PASS" : "CHECK");

    UI_String_Show(CONTENT_LEFT, 98U, 16U, 1U,
                   (const u8 *)"INIT", "INIT");
    UI_String_Show(DATA_VALUE_X, 98U, 16U, 1U,
                   (const u8 *)((s_measurement.ads_init_ok != 0U) ? "OK" : "FAIL"),
                   (s_measurement.ads_init_ok != 0U) ? "OK" : "FAIL");

    UI_String_Show(CONTENT_LEFT, 120U, 16U, 1U,
                   (const u8 *)"GPIO", "GPIO");
    (void)sprintf(buffer, "DRDY %s PDWN %s      ",
                  (s_measurement.ads_drdy_ready != 0U) ? "READY" : "BUSY",
                  (s_measurement.ads_pdwn_high != 0U) ? "HIGH" : "LOW");
    UI_String_Show(DATA_VALUE_X, 120U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 142U, 16U, 1U,
                   (const u8 *)"REG READ", "REG READ");
    UI_String_Show(DATA_VALUE_X, 142U, 16U, 1U,
                   (const u8 *)(registers_read_ok ? "OK" : "FAIL"),
                   registers_read_ok ? "OK" : "FAIL");
    UI_String_Show(CONTENT_LEFT, 164U, 16U, 1U,
                   (const u8 *)"REG CHECK", "REG CHECK");
    UI_String_Show(DATA_VALUE_X, 164U, 16U, 1U,
                   (const u8 *)(registers_match ? "MATCH" : "MISMATCH"),
                   registers_match ? "MATCH" : "MISMATCH");

    UI_String_Show(CONTENT_LEFT, 186U, 16U, 1U,
                   (const u8 *)"STATUS exp34/35", "STATUS exp34/35");
    OS_Hex8_Show(DATA_VALUE_X, 186U, 16U, 1U,
                 s_measurement.ads_status_reg);
    UI_String_Show(CONTENT_LEFT, 207U, 16U, 1U,
                   (const u8 *)"MUX exp08", "MUX exp08");
    OS_Hex8_Show(DATA_VALUE_X, 207U, 16U, 1U,
                 s_measurement.ads_mux_reg);
    UI_String_Show(CONTENT_LEFT, 228U, 16U, 1U,
                   (const u8 *)"ADCON exp00", "ADCON exp00");
    OS_Hex8_Show(DATA_VALUE_X, 228U, 16U, 1U,
                 s_measurement.ads_adcon_reg);
    UI_String_Show(CONTENT_LEFT, 249U, 16U, 1U,
                   (const u8 *)"DRATE expA1", "DRATE expA1");
    OS_Hex8_Show(DATA_VALUE_X, 249U, 16U, 1U,
                 s_measurement.ads_drate_reg);

    UI_String_Show(CONTENT_LEFT, 270U, 16U, 1U,
                   (const u8 *)"AIN0 AVG/OK", "AIN0 AVG/OK");
    (void)sprintf(buffer, "%ld 0x%06lX %u/%u      ",
                  (long)s_measurement.ads_ain0_raw,
                  (unsigned long)((uint32_t)s_measurement.ads_ain0_raw &
                                  0x00FFFFFFUL),
                  (unsigned int)s_measurement.ads_ain0_samples_ok,
                  (unsigned int)s_measurement.ads_test_repeat_count);
    UI_String_Show(DATA_VALUE_X, 270U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 291U, 16U, 1U,
                   (const u8 *)"AIN0 MIN/MAX", "AIN0 MIN/MAX");
    (void)sprintf(buffer, "%.0f/%.0f      ",
                  (double)s_measurement.ads_ain0_min_raw,
                  (double)s_measurement.ads_ain0_max_raw);
    UI_String_Show(DATA_VALUE_X, 291U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 312U, 16U, 1U,
                   (const u8 *)"AIN0 SPAN/V", "AIN0 SPAN/V");
    (void)sprintf(buffer, "%ld %.5f V      ", (long)ain0_span,
                  (double)s_measurement.ads_ain0_voltage);
    UI_String_Show(DATA_VALUE_X, 312U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 333U, 16U, 1U,
                   (const u8 *)"AIN1 AVG/OK", "AIN1 AVG/OK");
    (void)sprintf(buffer, "%ld 0x%06lX %u/%u      ",
                  (long)s_measurement.ads_ain1_raw,
                  (unsigned long)((uint32_t)s_measurement.ads_ain1_raw &
                                  0x00FFFFFFUL),
                  (unsigned int)s_measurement.ads_ain1_samples_ok,
                  (unsigned int)s_measurement.ads_test_repeat_count);
    UI_String_Show(DATA_VALUE_X, 333U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 354U, 16U, 1U,
                   (const u8 *)"AIN1 MIN/MAX", "AIN1 MIN/MAX");
    (void)sprintf(buffer, "%.0f/%.0f      ",
                  (double)s_measurement.ads_ain1_min_raw,
                  (double)s_measurement.ads_ain1_max_raw);
    UI_String_Show(DATA_VALUE_X, 354U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 375U, 16U, 1U,
                   (const u8 *)"AIN1 SPAN/V", "AIN1 SPAN/V");
    (void)sprintf(buffer, "%ld %.5f V      ", (long)ain1_span,
                  (double)s_measurement.ads_ain1_voltage);
    UI_String_Show(DATA_VALUE_X, 375U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 396U, 16U, 1U,
                   (const u8 *)"DELTA RAW", "DELTA RAW");
    OS_Num_Show(DATA_VALUE_X, 396U, 16U, 1U, (double)raw_delta,
                "%+.0f      ", "%+.0f      ");

    UI_String_Show(CONTENT_LEFT, 417U, 16U, 1U,
                   (const u8 *)"RESULT", "RESULT");
    UI_String_Show(DATA_VALUE_X, 417U, 16U, 1U,
                   (const u8 *)result_text, result_text);
    UI_String_Show(CONTENT_LEFT, 438U, 16U, 1U,
                   (const u8 *)"SWEEP", "SWEEP");
    (void)sprintf(buffer, "%u/%u      ",
                  (unsigned int)s_measurement.sweep_index,
                  (unsigned int)s_measurement.sweep_total);
    UI_String_Show(DATA_VALUE_X, 438U, 16U, 1U,
                   (const u8 *)buffer, buffer);
}
#endif
/* ???????????????????????? */
#if 0
static void draw_ads_sweep_progress(void)
{
    char buffer[40];

        /* Known-model page: adjust one point, then start output. */

    UI_String_Show(CONTENT_LEFT, 417U, 16U, 1U,
                   (const u8 *)"SWEEP GAIN", "SWEEP GAIN");
    (void)sprintf(buffer, "%+.2f dB      ",
                  (double)s_measurement.gain_db);
    UI_String_Show(DATA_VALUE_X, 417U, 16U, 1U,
                   (const u8 *)buffer, buffer);

    UI_String_Show(CONTENT_LEFT, 438U, 16U, 1U,
                   (const u8 *)"SWEEP", "SWEEP");
    (void)sprintf(buffer, "%luHz %u/%u      ",
                  (unsigned long)s_measurement.frequency_hz,
                  (unsigned int)s_measurement.sweep_index,
                  (unsigned int)s_measurement.sweep_total);
    UI_String_Show(DATA_VALUE_X, 438U, 16U, 1U,
                   (const u8 *)buffer, buffer);
}
#endif

static void draw_page_content(void)
{
    if (s_page == PAGE_MAIN)
    {
        draw_main_page();
    }
    else if (s_page == PAGE_SINE)
    {
        draw_sine_page();
    }
    else if (s_page == PAGE_KNOWN)
    {
        draw_known_page();
    }
    else if (s_page == PAGE_UNKNOWN)
    {
        draw_unknown_page();
    }
    else if (s_page == PAGE_ADS_TEST)
    {
        draw_ads_calibration_page();
    }
    else
    {
        draw_output_page();            /* PAGE_OUTPUT */
    }
}

/* ???????????? ?????????? */
static void render_page(void)
{
    clear_right_panel();
    draw_page_content();
    draw_menu_selection();
}

/* ?????????????? ???????? */
static void refresh_page_data(void)
{
    draw_page_content();
}

/* ???????? ?????????? */
static void switch_page(UiPage next_page)
{
    if (next_page <= PAGE_ADS_TEST)
    {
        if (next_page == PAGE_KNOWN)
        {
            if (s_frequency_hz < UI_MIN_FREQUENCY_HZ)
            {
                s_frequency_hz = UI_MIN_FREQUENCY_HZ;
            }
            if (s_frequency_hz > UI_KNOWN_MAX_FREQUENCY_HZ)
            {
                s_frequency_hz = UI_KNOWN_MAX_FREQUENCY_HZ;
            }
        }
        else if (next_page == PAGE_SINE)
        {
            if (s_frequency_hz < UI_MIN_FREQUENCY_HZ)
            {
                s_frequency_hz = UI_MIN_FREQUENCY_HZ;
            }
            if (s_frequency_hz > UI_MAX_FREQUENCY_HZ)
            {
                s_frequency_hz = UI_MAX_FREQUENCY_HZ;
            }
        }
        else if (next_page == PAGE_KNOWN)
        {
            if (s_known_target_peak_v < UI_KNOWN_TARGET_MIN_VPP)
            {
                s_known_target_peak_v = UI_KNOWN_TARGET_MIN_VPP;
            }
            if (s_known_target_peak_v > UI_KNOWN_TARGET_MAX_VPP)
            {
                s_known_target_peak_v = UI_KNOWN_TARGET_MAX_VPP;
            }
        }
        else if (next_page == PAGE_ADS_TEST)
        {
            s_calibration_item = UI_CALIBRATION_ADS_REFERENCE;
        }
        s_page = next_page;
        render_page();
    }
}

/* ================================================================
 * 对外接口函数
 * ================================================================ */

/* 设置汉字字库可用标志 */
void UI_SetChineseFontAvailable(uint8_t available)
{
    s_chinese_font_available = (available != 0U) ? 1U : 0U;
}

/* UI 初始化 重置所有状态并绘制首页 */
void UI_Init(void)
{
    s_page = PAGE_MAIN;
    s_frequency_hz = UI_DEFAULT_FREQUENCY_HZ;
    s_amplitude_mv = UI_DEFAULT_AMPLITUDE_MV;
    s_phase_deg = UI_DEFAULT_PHASE_DEG;
    s_known_target_peak_v = UI_DEFAULT_KNOWN_TARGET_PEAK_V;
    s_calibration_item = UI_CALIBRATION_ADS_REFERENCE;
    (void)memset(&s_measurement, 0, sizeof(s_measurement));
    s_measurement.frequency_hz = s_frequency_hz;
    s_measurement.target_output_peak_v = s_known_target_peak_v;
    LCD_Clear(Black);
    draw_reference_frame();
    draw_page_content();
    draw_menu_selection();
}

/* 按键事件处理 返回是否产生有效命令 */
bool UI_HandleKey(PS2_KeyValue_t key, UI_Command_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    command->type = UI_COMMAND_NONE;
    command->frequency_hz = s_frequency_hz;
    command->amplitude_mv = s_amplitude_mv;
    command->phase_deg = s_phase_deg;
    command->target_output_peak_v = s_known_target_peak_v;
    command->calibration_item = s_calibration_item;

    /* 返回键退回主菜单并停止输出 */
    if (key == PS2_KEY_VALUE_BACKSPACE)
    {
        command->type = (s_page == PAGE_MAIN) ?
                        UI_COMMAND_NONE : UI_COMMAND_STOP_OUTPUT;
        switch_page(PAGE_MAIN);
        return (command->type != UI_COMMAND_NONE);
    }

    /* 主页面数字 1~5 切换页面 */
    if ((s_page == PAGE_MAIN) &&
        (key >= PS2_KEY_VALUE_1) && (key <= PS2_KEY_VALUE_5))
    {
        switch_page((UiPage)key);
        return false;
    }

    /* 正弦页：频率/幅值加减调节 */
    if (s_page == PAGE_SINE)
    {
        if (key == PS2_KEY_VALUE_ADD)
        {
            if (s_frequency_hz < UI_MAX_FREQUENCY_HZ)
            {
                s_frequency_hz += 100U;
                if (s_frequency_hz > UI_MAX_FREQUENCY_HZ)
                {
                    s_frequency_hz = UI_MAX_FREQUENCY_HZ;
                }
            }
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MINUS)
        {
            s_frequency_hz = (s_frequency_hz > UI_MIN_FREQUENCY_HZ) ?
                             s_frequency_hz - 100U : UI_MIN_FREQUENCY_HZ;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MULTIPLY)
        {
            s_amplitude_mv = (s_amplitude_mv < (UI_MAX_AMPLITUDE_MV - 10.0f)) ?
                             s_amplitude_mv + 10.0f : UI_MAX_AMPLITUDE_MV;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_DIVIDE)
        {
            s_amplitude_mv = (s_amplitude_mv > 10.0f) ?
                             s_amplitude_mv - 10.0f : 0.0f;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
        {
            command->type = UI_COMMAND_SET_SINE;
        }
    }
    /* 已知电路页：回车启动固定的 330 点二维步进扫频 */
    else if (s_page == PAGE_KNOWN)
    {
        if (key == PS2_KEY_VALUE_ADD)
        {
            if (s_frequency_hz < UI_KNOWN_MAX_FREQUENCY_HZ)
            {
                s_frequency_hz += UI_KNOWN_FREQUENCY_STEP_HZ;
            }
            if (s_frequency_hz > UI_KNOWN_MAX_FREQUENCY_HZ)
            {
                s_frequency_hz = UI_KNOWN_MAX_FREQUENCY_HZ;
            }
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MINUS)
        {
            s_frequency_hz = (s_frequency_hz > UI_MIN_FREQUENCY_HZ) ?
                             s_frequency_hz - UI_KNOWN_FREQUENCY_STEP_HZ :
                             UI_MIN_FREQUENCY_HZ;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MULTIPLY)
        {
            if (s_known_target_peak_v <
                (UI_KNOWN_TARGET_MAX_VPP - UI_KNOWN_TARGET_STEP_VPP * 0.5f))
            {
                s_known_target_peak_v += UI_KNOWN_TARGET_STEP_VPP;
            }
            if (s_known_target_peak_v > UI_KNOWN_TARGET_MAX_VPP)
            {
                s_known_target_peak_v = UI_KNOWN_TARGET_MAX_VPP;
            }
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_DIVIDE)
        {
            if (s_known_target_peak_v >
                (UI_KNOWN_TARGET_MIN_VPP + UI_KNOWN_TARGET_STEP_VPP * 0.5f))
            {
                s_known_target_peak_v -= UI_KNOWN_TARGET_STEP_VPP;
            }
            if (s_known_target_peak_v < UI_KNOWN_TARGET_MIN_VPP)
            {
                s_known_target_peak_v = UI_KNOWN_TARGET_MIN_VPP;
            }
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
        {
            command->type = UI_COMMAND_RUN_KNOWN_MODEL;
        }
    }
    /* 菜单5：选择校准测试项目，所有结果通过USART6输出 */
    else if (s_page == PAGE_ADS_TEST)
    {
        if (key == PS2_KEY_VALUE_ADD)
        {
            s_calibration_item = (s_calibration_item < 6U) ?
                                 (uint8_t)(s_calibration_item + 1U) : 1U;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MINUS)
        {
            s_calibration_item = (s_calibration_item > 1U) ?
                                 (uint8_t)(s_calibration_item - 1U) : 6U;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
        {
            command->type = UI_COMMAND_RUN_CALIBRATION;
        }
    }
    /* 其余页面回车触发对应功能 */
    else if (key == PS2_KEY_VALUE_ENTER)
    {
        if (s_page == PAGE_KNOWN)
        {
            command->type = UI_COMMAND_RUN_KNOWN_MODEL;
        }
        else if (s_page == PAGE_UNKNOWN)
        {
            command->type = UI_COMMAND_START_UNKNOWN_MODEL;
        }
        else if (s_page == PAGE_OUTPUT)
        {
            command->type = UI_COMMAND_START_INFERENCE;
        }
    }

    return (command->type != UI_COMMAND_NONE);
}

/* 测量数据更新回调 自动刷新对应页面 */
void UI_UpdateMeasurement(const UI_Measurement_t *measurement)
{
    if (measurement == NULL)
    {
        return;
    }

    s_measurement = *measurement;
    /* 仅对应页面收到对应事件才刷新 */
    if (((measurement->event == UI_EVENT_MEASUREMENT) &&
         (s_page == PAGE_KNOWN)) ||
        ((measurement->event == UI_EVENT_SWEEP_PROGRESS) &&
         (s_page == PAGE_UNKNOWN)) ||
        ((measurement->event == UI_EVENT_MODEL_RESULT) &&
         (s_page == PAGE_UNKNOWN)) ||
        ((measurement->event == UI_EVENT_STATUS) &&
         (s_page == PAGE_OUTPUT)))
    {
        refresh_page_data();
    }
}

/* 主循环入口 PS2 按键扫描 */
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
        HAL_Delay(10U);
    }
}

/* 兼容旧版对外接口 */

void Init_All(void)
{
    UI_Init();
}

void Disp_Main(void)
{
    s_page = PAGE_MAIN;
    LCD_Clear(Black);
    draw_reference_frame();
    draw_page_content();
    draw_menu_selection();
}

/* 指定位置显示数值 兼容旧版接口 */
void Show_Val(uint8_t location, float value, char *format)
{
    uint16_t x;
    uint16_t y;

    if ((location > 0U) && (location <= 10U))
    {
        x = 314U;
        y = (uint16_t)(96U + 32U * (location - 1U));
    }
    else if ((location > 10U) && (location <= 20U))
    {
        x = 564U;
        y = (uint16_t)(96U + 32U * (location - 11U));
    }
    else
    {
        UI_String_Show(314U, 96U, 32U, 1U,
                       (const u8 *)"ERROR", "ERROR");
        return;
    }

    OS_Num_Show(x, y, 24U, 1U, (double)value, format, format);
}

/* 外部切换菜单接口 */
void Change_Menu(uint8_t menu_sign)
{
    if ((menu_sign >= 1U) && (menu_sign <= MenuChoiceNum))
    {
        switch_page((UiPage)menu_sign);
    }
    else
    {
        switch_page(PAGE_MAIN);
    }
}

/* 各菜单快捷跳转函数 */
void MenuHaddler_1(void)
{
    switch_page(PAGE_SINE);
}

void MenuHaddler_2(void)
{
    switch_page(PAGE_KNOWN);
}

void MenuHaddler_3(void)
{
    switch_page(PAGE_UNKNOWN);
}

void MenuHaddler_4(void)
{
    switch_page(PAGE_OUTPUT);
}

void MenuHaddler_5(void)
{
    switch_page(PAGE_ADS_TEST);
}

/* 预留数值读取接口 兼容旧代码 */
float PS2_ReadNum(float num)
{
    return num;
}
