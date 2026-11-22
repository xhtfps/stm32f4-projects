/**
 * @file    user.c
 * @brief   工程模板 LCD 显示与 UI 框架
 *
 * 提供基于 TFT LCD 的完整菜单框架，支持多页面切换、
 * GBK 中文和 ASCII 英文双模式显示。
 * 开发新项目时，在 draw_page_content() 中添加自定义页面绘制函数。
 */

#include "user.h"

#include <stdio.h>
#include <string.h>

#include "./BSP/TFT_LCD/TFT_LCD.h"
#include "./BSP/TFT_LCD/text.h"
#include "./BSP/AD7606/AD7606.h"
#include "./BSP/MPU6050/MPU6050.h"

/* ================================================================
 * 页面宏定义
 * ================================================================ */
#define PAGE_MAIN       0U            /* 主菜单首页 */
#define PAGE_SINE       1U            /* 预留页面 1 */
#define PAGE_KNOWN      2U            /* 预留页面 2 */
#define PAGE_UNKNOWN    3U            /* 预留页面 3 */
#define PAGE_OUTPUT     4U            /* 预留页面 4 */
#define PAGE_ADS_TEST   5U            /* 预留页面 5 */

#define UI_MENU_SIZE    24U           /* 左侧菜单字号 */
#define UI_PRIMARY_SIZE 24U           /* 正文字号 */
#define UI_HELP_SIZE    16U           /* 提示小字号 */

typedef uint8_t UiPage;               /* 页面类型 */

/* ================================================================
 * GBK 中文字符串（可替换为项目名称）
 * ================================================================ */
static const u8 gbk_title[]        = "\xB9\xA4\xB3\xCC\xC4\xA3\xB0\xE5";       /* 工程模板 */
static const u8 gbk_sine[]         = "1 MPU6050";
static const u8 gbk_known[]        = "2 AD7606 TEST";
static const u8 gbk_unknown[]      = "3 PAGE 3";
static const u8 gbk_output[]       = "4 PAGE 4";
static const u8 gbk_ads_test[]     = "5 PAGE 5";
static const u8 gbk_dash[]         = "-";
static const u8 gbk_selected[]     = ">";
static const u8 gbk_select[]       = "\xC7\xEB\xD1\xA1\xD4\xF1\xB9\xA6\xC4\xDC\xB2\xCB\xB5\xA5"; /* 请选择功能菜单 */
static const u8 gbk_run[]          = "\xBB\xD8\xB3\xB5\xC8\xB7\xC8\xCF\xD6\xB4\xD0\xD0";       /* 回车确认执行 */
static const u8 gbk_back[]         = "BACKSPACE \xB7\xB5\xBB\xD8\xD6\xF7\xB2\xCB\xB5\xA5";      /* BACKSPACE 返回主菜单 */

/* ================================================================
 * 全局状态变量
 * ================================================================ */
static UiPage s_page = PAGE_MAIN;                /* 当前页面 */
static uint8_t s_chinese_font_available;         /* 中文字库可用标志 */

/* ========== MPU6050 测试页状态 ========== */
static volatile uint8_t  s_mpu_initialized;      /* MPU6050 初始化成功标志 */
static Mpu6050Data        s_mpu_data;            /* 最近一次读取的物理量数据 */

/* ========== AD7606 测试页状态 ========== */
static uint8_t s_ad7606_initialized;
static int16_t s_ad7606_samples[AD7606_CHANNEL_COUNT];

/* ================================================================
 * ASCII 字符显示底层函数
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

    /* 计算字符占据的字节数 */
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
        /* 根据字号选择对应ASCII字库 */
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

        /* 逐位显示字符 */
        for (t1 = 0U; t1 < 8U; ++t1)
        {
            if ((temp & 0x80U) != 0U)
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(TextColor);      /* 前景色 */
            }
            else if (mode == 1U)               /* mode=1 背景填充 */
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

/* 中文/GBK 底层字库显示 */
static void OS_Font_Show(uint16_t x, uint16_t y, char *font,
                         uint8_t size, uint8_t mode)
{
    LCD_ShowGBKFont(x, y, (u8 *)font, size, mode);
}

/* 中英文字符串自动显示（自动识别 GBK） */
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
            /* 判断是否为 GBK 首字节（最高位为1即为中文） */
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
            str += 2;                         /* 中文占两字节 */
            x = (uint16_t)(x + size);
        }
    }
}

/* ASCII 英文字符串显示 */
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

/* 双模式字符串显示（GBK 中文 / ASCII 英文回退） */
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

/* 格式化数字显示 */
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

/* ================================================================
 * LCD 框架绘制函数
 * ================================================================ */

/* 清空右侧内容区域 */
static void clear_right_panel(void)
{
    LCD_Appoint_Clear(252U, 72U, 800U, 440U, Black);
}

/* 绘制参考框架（标题栏、左侧菜单栏、状态栏） */
static void draw_reference_frame(void)
{
    uint8_t count;

    LCD_SetColors(White, Black);
    UI_String_Show(272U, 16U, 32U, 0U, gbk_title, "PROJECT TEMPLATE");
    /* 绘制分隔线边框 */
    LCD_Appoint_Clear(0U, 64U, 800U, 72U, White);
    LCD_Appoint_Clear(0U, 440U, 800U, 448U, White);
    LCD_Appoint_Clear(250U, 72U, 252U, 440U, White);

    /* 底部版本信息 */
    UI_String_Show(32U, 456U, 16U, 0U,
                   (const u8 *)"STM32F4", "STM32F4");
    UI_String_Show(632U, 456U, 16U, 0U,
                   (const u8 *)"V1.0", "V1.0");

    /* 左侧菜单前缀 "-" 符号 */
    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 0U,
                       gbk_dash, "-");
    }
    /* 左侧菜单项名称 */
    UI_String_Show(80U, 96U, UI_MENU_SIZE, 0U, gbk_sine, "1 MPU6050");
    UI_String_Show(80U, 160U, UI_MENU_SIZE, 0U, gbk_known, "2 AD7606 TEST");
    UI_String_Show(80U, 224U, UI_MENU_SIZE, 0U, gbk_unknown, "3 PAGE 3");
    UI_String_Show(80U, 288U, UI_MENU_SIZE, 0U, gbk_output, "4 PAGE 4");
    UI_String_Show(80U, 352U, UI_MENU_SIZE, 0U, gbk_ads_test, "5 PAGE 5");
}

/* 绘制左侧菜单选中标识 ">" 箭头 */
static void draw_menu_selection(void)
{
    uint8_t count;

    /* 擦除所有 "-" */
    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 1U,
                       gbk_dash, "-");
    }
    /* 当前页面显示选中箭头 */
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

/* ========== MPU6050 测试页面布局常量 ========== */
#define MPU_LABEL_X     320U      /* 标签 X */
#define MPU_DATA_X      405U      /* 数值 X */
#define MPU_L1_Y        140U      /* 第 1 行 (ACCEL) */
#define MPU_L2_Y        168U      /* 第 2 行 (GYRO)  */
#define MPU_L3_Y        196U      /* 第 3 行 (TEMP)  */
#define MPU_L4_Y        224U      /* 第 4 行 (WHO_AM_I) */
#define MPU_L5_Y        260U      /* 第 5 行 (状态/提示) */

/* ---------- MPU6050 测试页面（菜单 1） ---------- */
static void draw_page1(void)
{
    uint8_t who_am_i;

    LCD_SetColors(White, Black);

    /* 标题 */
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   (const u8 *)"MPU6050 TEST", "MPU6050 TEST");

    /* 首次进入时初始化 MPU6050 */
    if (s_mpu_initialized == 0U)
    {
        if (MPU6050_Init() == HAL_OK)
        {
            s_mpu_initialized = 1U;
        }
    }

    if (s_mpu_initialized != 0U)
    {
        /* 标签（仅绘制一次） */
        OS_Ascii_Show(MPU_LABEL_X, MPU_L1_Y, 16U, 1U, "ACCEL(g):");
        OS_Ascii_Show(MPU_LABEL_X, MPU_L2_Y, 16U, 1U, "GYRO(dps):");
        OS_Ascii_Show(MPU_LABEL_X, MPU_L3_Y, 16U, 1U, "TEMP(C):");

        /* WHO_AM_I */
        if (MPU6050_ReadWhoAmI(&who_am_i) == HAL_OK)
        {
            char who_str[32];
            (void)sprintf(who_str, "WHO_AM_I:  0x%02X  OK", who_am_i);
            OS_Ascii_Show(MPU_LABEL_X, MPU_L4_Y, 16U, 1U, who_str);
        }

        /* 初次读取数据 */
        (void)MPU6050_ReadData(&s_mpu_data);
    }
    else
    {
        LCD_SetColors(Red, Black);
        OS_Ascii_Show(MPU_LABEL_X, MPU_L1_Y, 16U, 1U,
                      "MPU6050 INIT FAILED!");
        LCD_SetColors(White, Black);
        OS_Ascii_Show(MPU_LABEL_X, MPU_L2_Y, 16U, 1U,
                      "Check I2C wiring & address(0x68)");
    }

    /* 返回提示 */
    UI_String_Show(CONTENT_LEFT, 420U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

/* ---------- 刷新 MPU6050 数值（仅更新数值区域） ---------- */
static void mpu6050_refresh_values(void)
{
    char line_buf[56];

    if ((s_page != PAGE_SINE) || (s_mpu_initialized == 0U))
    {
        return;
    }

    /* 重新读取数据 */
    if (MPU6050_ReadData(&s_mpu_data) != HAL_OK)
    {
        return;
    }

    /* ---- 加速度: X:+0.023  Y:-0.015  Z:+1.002 ---- */
    LCD_SetColors(Yellow, Black);
    (void)sprintf(line_buf, "X:%+7.3f  Y:%+7.3f  Z:%+7.3f",
                  (double)s_mpu_data.accel_x_g,
                  (double)s_mpu_data.accel_y_g,
                  (double)s_mpu_data.accel_z_g);
    OS_Ascii_Show(MPU_DATA_X, MPU_L1_Y, 16U, 1U, line_buf);

    /* ---- 陀螺仪: X:-0.5  Y:+0.3  Z:-0.1 ---- */
    LCD_SetColors(Cyan, Black);
    (void)sprintf(line_buf, "X:%+6.1f  Y:%+6.1f  Z:%+6.1f",
                  (double)s_mpu_data.gyro_x_dps,
                  (double)s_mpu_data.gyro_y_dps,
                  (double)s_mpu_data.gyro_z_dps);
    OS_Ascii_Show(MPU_DATA_X, MPU_L2_Y, 16U, 1U, line_buf);

    /* ---- 温度 ---- */
    LCD_SetColors(Green, Black);
    (void)sprintf(line_buf, "%+6.1f C", (double)s_mpu_data.temperature_c);
    OS_Ascii_Show(MPU_DATA_X, MPU_L3_Y, 16U, 1U, line_buf);

    LCD_SetColors(White, Black);
}

#define AD7606_LABEL_X       320U
#define AD7606_VALUE_X       384U
#define AD7606_FIRST_Y       128U
#define AD7606_ROW_HEIGHT    32U

/* ---------- AD7606 测试页面（菜单 2） ---------- */
static void draw_page2(void)
{
    uint8_t channel;

    LCD_SetColors(White, Black);
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   (const u8 *)"AD7606 TEST", "AD7606 TEST");

    if (s_ad7606_initialized == 0U)
    {
        if (AD7606_Init() == HAL_OK)
        {
            s_ad7606_initialized = 1U;
        }
    }

    if (s_ad7606_initialized != 0U)
    {
        for (channel = 0U; channel < AD7606_CHANNEL_COUNT; ++channel)
        {
            char label[12];

            (void)sprintf(label, "CH%u:", (unsigned int)(channel + 1U));
            OS_Ascii_Show(AD7606_LABEL_X,
                          (uint16_t)(AD7606_FIRST_Y +
                                     AD7606_ROW_HEIGHT * channel),
                          16U, 1U, label);
        }
        OS_Ascii_Show(CONTENT_LEFT, 392U, UI_HELP_SIZE, 1U,
                      "RANGE: +/-10V  OS: 1x");
    }
    else
    {
        LCD_SetColors(Red, Black);
        OS_Ascii_Show(AD7606_LABEL_X, 144U, 16U, 1U,
                      "AD7606 INIT FAILED!");
        LCD_SetColors(White, Black);
        OS_Ascii_Show(AD7606_LABEL_X, 176U, 16U, 1U,
                      "CHECK POWER / WIRING");
    }

    UI_String_Show(CONTENT_LEFT, 420U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

static void ad7606_refresh_values(void)
{
    uint8_t channel;
    char line_buf[40];

    if ((s_page != PAGE_KNOWN) || (s_ad7606_initialized == 0U))
    {
        return;
    }

    if (AD7606_ReadFrame(s_ad7606_samples) != HAL_OK)
    {
        LCD_SetColors(Red, Black);
        OS_Ascii_Show(AD7606_VALUE_X, 392U, 16U, 1U,
                      "READ ERROR             ");
        LCD_SetColors(White, Black);
        return;
    }

    LCD_SetColors(Yellow, Black);
    for (channel = 0U; channel < AD7606_CHANNEL_COUNT; ++channel)
    {
        float voltage;

        voltage = AD7606_CodeToVoltage(s_ad7606_samples[channel],
                                       AD7606_INPUT_RANGE_VOLTS);
        (void)sprintf(line_buf, "%+7.3f V  %6d",
                      (double)voltage, (int)s_ad7606_samples[channel]);
        OS_Ascii_Show(AD7606_VALUE_X,
                      (uint16_t)(AD7606_FIRST_Y +
                                 AD7606_ROW_HEIGHT * channel),
                      16U, 1U, line_buf);
    }
    LCD_SetColors(White, Black);
}

static void draw_page3(void)
{
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   (const u8 *)"PAGE 3 CONTENT", "PAGE 3 CONTENT");
    UI_String_Show(CONTENT_LEFT, 160U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

static void draw_page4(void)
{
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   (const u8 *)"PAGE 4 CONTENT", "PAGE 4 CONTENT");
    UI_String_Show(CONTENT_LEFT, 160U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

static void draw_page5(void)
{
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   (const u8 *)"PAGE 5 CONTENT", "PAGE 5 CONTENT");
    UI_String_Show(CONTENT_LEFT, 160U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

/* 绘制页面内容（根据当前页面分发） */
static void draw_page_content(void)
{
    if (s_page == PAGE_MAIN)
    {
        draw_main_page();
    }
    else if (s_page == PAGE_SINE)
    {
        draw_page1();
    }
    else if (s_page == PAGE_KNOWN)
    {
        draw_page2();
    }
    else if (s_page == PAGE_UNKNOWN)
    {
        draw_page3();
    }
    else if (s_page == PAGE_ADS_TEST)
    {
        draw_page5();
    }
    else
    {
        draw_page4();            /* PAGE_OUTPUT */
    }
}

/* 完整渲染当前页面（清空右侧 + 绘制内容 + 菜单选中标识） */
static void render_page(void)
{
    clear_right_panel();
    draw_page_content();
    draw_menu_selection();
}

/* 仅刷新页面数据（不清空右侧，用于数据更新） */
static void refresh_page_data(void)
{
    draw_page_content();
}

/* 切换页面 */
static void switch_page(UiPage next_page)
{
    if (next_page <= PAGE_ADS_TEST)
    {
        s_page = next_page;
        render_page();
    }
}

/* ================================================================
 * 公共接口函数
 * ================================================================ */

/* 设置中文字库可用标志 */
void UI_SetChineseFontAvailable(uint8_t available)
{
    s_chinese_font_available = (available != 0U) ? 1U : 0U;
}

/* UI 初始化：清屏、绘制框架和页面内容 */
void UI_Init(void)
{
    s_page = PAGE_MAIN;
    LCD_Clear(Black);
    draw_reference_frame();
    draw_page_content();
    draw_menu_selection();
}

/* 显示主页面（全屏重绘） */
void Disp_Main(void)
{
    s_page = PAGE_MAIN;
    LCD_Clear(Black);
    draw_reference_frame();
    draw_page_content();
    draw_menu_selection();
}

/* 键盘事件处理（返回 true 表示产生有效命令） */
bool UI_HandleKey(PS2_KeyValue_t key, UI_Command_t *command)
{
    if (command == NULL)
    {
        return false;
    }

    command->type = UI_COMMAND_NONE;

    /* Backspace: 停止当前操作，返回主页 */
    if (key == PS2_KEY_VALUE_BACKSPACE)
    {
        switch_page(PAGE_MAIN);
        return false;
    }

    /* 主页下按 1~5 切换页面 */
    if ((s_page == PAGE_MAIN) &&
        (key >= PS2_KEY_VALUE_1) && (key <= PS2_KEY_VALUE_5))
    {
        switch_page((UiPage)key);
        return false;
    }

    return (command->type != UI_COMMAND_NONE);
}

/* 菜单页面切换函数（兼容旧版接口） */
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

/* 用户主循环 */
void User_main(void)
{
    UI_Init();
    for (;;)
    {
        /* 用户自定义循环逻辑 */
        HAL_Delay(10U);
    }
}

/* 全初始化（兼容旧版接口） */
void Init_All(void)
{
    UI_Init();
}

/* 指定位置显示数值 */
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

/* 外部切换菜单页面 */
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

/* FreeRTOS 演示页面显示（按键切换） */
void UI_ShowDemoPage(uint8_t page)
{
    clear_right_panel();

    switch (page)
    {
        case 0U:
            UI_String_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (const u8 *)"\xBB\xB6\xD3\xAD\xCA\xB9\xD3\xC3",
                "WELCOME TO USE");
            break;
        case 1U:
            UI_String_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (const u8 *)"FreeRTOS + STM32F4",
                "FreeRTOS + STM32F4");
            break;
        case 2U:
            UI_String_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (const u8 *)"\xC8\xAB\xB2\xBF\xC7\xFD\xB6\xAF\xB6\xBC\xD2\xD1\xB1\xA3\xC1\xF4",
                "ALL DRIVERS INCLUDED");
            break;
        case 3U:
            UI_String_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (const u8 *)"LCD TFT \xCF\xD4\xCA\xBE\xD5\xFD\xB3\xA3",
                "LCD TFT DISPLAY OK");
            break;
        default:
            break;
    }
}

/* 预置数值读取接口（兼容旧版） */
float PS2_ReadNum(float num)
{
    return num;
}

/* ================================================================
 * MPU6050 公共接口
 * ================================================================ */

/**
 * @brief 获取当前页面编号
 * @return 当前页面编号 (0=主页, 1~5=子页面)
 */
uint8_t UI_GetCurrentPage(void)
{
    return (uint8_t)s_page;
}

/**
 * @brief 刷新 MPU6050 显示数值
 *
 * 当页面为 MPU6050 测试页时，重新读取传感器数据并更新 LCD 数值。
 * 建议由 FreeRTOS 任务以 100~500ms 周期调用。
 */
void UI_Mpu6050_UpdateData(void)
{
    mpu6050_refresh_values();
}

/**
 * @brief 刷新 AD7606 测试页面数据。
 */
void UI_Ad7606_UpdateData(void)
{
    ad7606_refresh_values();
}
