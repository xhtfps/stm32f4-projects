/* 相位差测量仪 UI：保持“简易电路模型探究”原四页菜单框架。 */
#include "user.h"

#include <stdio.h>
#include <string.h>

#include "./BSP/TFT_LCD/TFT_LCD.h"
#include "./BSP/TFT_LCD/text.h"

#define CONTENT_LEFT    314U
#define DATA_VALUE_X    564U
#define PAGE_MAIN       0U
#define PAGE_SINE       1U
#define PAGE_KNOWN      2U
#define PAGE_UNKNOWN    3U
#define PAGE_OUTPUT     4U
#define PAGE_ADS_TEST   5U
#define UI_MENU_SIZE    24U
#define UI_PRIMARY_SIZE 24U
#define UI_HELP_SIZE    16U

typedef uint8_t UiPage;

static const uint8_t gbk_title[] =
    "\xCF\xE0\xCE\xBB\xB2\xEE\xB2\xE2\xC1\xBF\xD2\xC7";
static const uint8_t gbk_dash[] = "\xA1\xAA";
static const uint8_t gbk_selected[] = "\xA1\xF4";
static const uint8_t gbk_select[] = "\xC7\xEB\xD1\xA1\xD4\xF1\xB2\xCB\xB5\xA5";
static const uint8_t gbk_run[] = "\xB0\xB4\xCF\xC2\xBB\xD8\xB3\xB5\xC6\xF4\xB6\xAF";
static const uint8_t gbk_back[] = "\xB7\xB5\xBB\xD8\xD6\xF7\xB2\xCB\xB5\xA5";

static UiPage s_page = PAGE_MAIN;
static uint32_t s_frequency_hz = UI_DEFAULT_FREQUENCY_HZ;
static float s_amplitude_mv = UI_DEFAULT_AMPLITUDE_MV;
static uint32_t s_phase_deg = UI_DEFAULT_PHASE_DEG;
static float s_known_target_peak_v = UI_DEFAULT_KNOWN_TARGET_PEAK_V;
static uint8_t s_chinese_font_available;
static UI_Measurement_t s_measurement;
static uint8_t s_calibration_item = UI_CALIBRATION_ADS_REFERENCE;
static uint8_t s_pga_channel;
static float s_pga_gain_db[4];

static void OS_Char_Show(uint16_t x, uint16_t y, uint8_t num,
                         uint8_t size, uint8_t mode)
{
    uint8_t temp;
    uint8_t t1;
    uint8_t t;
    uint16_t y0 = y;
    uint8_t csize;

    csize = (uint8_t)((size / 8U + ((size % 8U) ? 1U : 0U)) *
                      (size / 2U));
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
        for (t1 = 0U; t1 < 8U; ++t1)
        {
            if ((temp & 0x80U) != 0U)
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(TextColor);
            }
            else if (mode == 1U)
            {
                LCD_SetCursor(x, y);
                LCD_WriteRAM_Prepare();
                LCD_WriteRAM(BackColor);
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

static void OS_Font_Show(uint16_t x, uint16_t y, char *font,
                         uint8_t size, uint8_t mode)
{
    LCD_ShowGBKFont(x, y, (uint8_t *)font, size, mode);
}

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
            if ((int8_t)*str < 0)
            {
                chinese = 1U;
            }
            else if (*str == '\n')
            {
                y = (uint16_t)(y + size);
                x = x0;
                ++str;
                continue;
            }
            else
            {
                OS_Char_Show(x, y, (uint8_t)*str, size, mode);
                ++str;
                x = (uint16_t)(x + size / 2U);
            }
        }
        else
        {
            chinese = 0U;
            OS_Font_Show(x, y, str, size, mode);
            str += 2;
            x = (uint16_t)(x + size);
        }
    }
}

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

static void clear_right_panel(void)
{
    LCD_Appoint_Clear(252U, 72U, 800U, 440U, Black);
}

static void draw_reference_frame(void)
{
    uint8_t count;

    LCD_SetColors(White, Black);
    UI_String_Show(272U, 16U, 32U, 0U, gbk_title, TitleStr);
    LCD_Appoint_Clear(0U, 64U, 800U, 72U, White);
    LCD_Appoint_Clear(0U, 440U, 800U, 448U, White);
    LCD_Appoint_Clear(250U, 72U, 252U, 440U, White);
    UI_String_Show(32U, 456U, 16U, 0U,
                   (const uint8_t *)ModelVerStr, ModelVerStr);
    UI_String_Show(632U, 456U, 16U, 0U,
                   (const uint8_t *)UserVerStr, UserVerStr);

    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 0U,
                       gbk_dash, "-");
    }
    UI_String_Show(80U, 96U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice1, "1 FREQUENCY");
    UI_String_Show(80U, 160U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice2, "2 PHASE");
    UI_String_Show(80U, 224U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice3, "3 ADC/FFT");
    UI_String_Show(80U, 288U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice4, "4 VOFA OUTPUT");
    UI_String_Show(80U, 352U, UI_MENU_SIZE, 0U,
                   (const uint8_t *)Menu1Choice5, "5 PGA4311 GAIN");
}

static void draw_menu_selection(void)
{
    uint8_t count;

    for (count = 1U; count <= MenuChoiceNum; ++count)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * count), 32U, 1U,
                       gbk_dash, "-");
    }
    if (s_page != PAGE_MAIN)
    {
        UI_String_Show(32U, (uint16_t)(32U + 64U * s_page), 32U, 1U,
                       gbk_selected, ">");
    }
}

static void draw_main_page(void)
{
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   gbk_select, "SELECT MENU");
    UI_String_Show(CONTENT_LEFT, 160U, UI_HELP_SIZE, 1U,
                   gbk_run, "ENTER RUN");
    UI_String_Show(CONTENT_LEFT, 184U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK");
}

static void refresh_sine_measurement_values(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.frequency_hz_precise,
                "\xB2\xE2\xC1\xBF\xC6\xB5\xC2\xCA %.3f Hz      ",
                "MEASURED FREQ %.3f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.period_ticks,
                "TIM2 CH3 %.0f ticks      ",
                "TIM2 CH3 CAPTURE %.0f ticks      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.capture_cycles,
                "\xB2\xB6\xBB\xF1\xD6\xDC\xC6\xDA %.0f      ",
                "CAPTURE PERIODS %.0f      ");
    OS_Num_Show(CONTENT_LEFT, 192U, 24U, 1U,
                (double)s_measurement.divider,
                "\xCD\xE2\xB2\xBF\xB7\xD6\xC6\xB5 1:%.0f      ",
                "EXTERNAL DIVIDER 1:%.0f      ");
    UI_String_Show(CONTENT_LEFT, 224U, 24U, 1U,
                   (const uint8_t *)((s_measurement.frequency_valid != 0U) ?
                       "STATUS SIGNAL VALID        " :
                       "STATUS WAIT SIGNAL         "),
                   (s_measurement.frequency_valid != 0U) ?
                       "STATUS SIGNAL VALID        " :
                       "STATUS WAIT SIGNAL         ");
}

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

static void refresh_known_measurement_values(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.phase_difference_deg,
                "\xCF\xE0\xCE\xBB\xB2\xEE%.3f deg      ",
                "PHASE %.3f deg      ");
    OS_Num_Show(DATA_VALUE_X, 96U, 24U, 1U,
                (double)s_measurement.frequency_hz,
                "\xC6\xB5\xC2\xCA%.0f Hz      ",
                "FREQ %.0f Hz      ");
    OS_Num_Show(CONTENT_LEFT, 128U, 24U, 1U,
                (double)s_measurement.channel1_amplitude,
                "CH1 \xB7\xF9\xD6\xB5%.4f      ",
                "CH1 AMP %.4f      ");
    OS_Num_Show(DATA_VALUE_X, 128U, 24U, 1U,
                (double)s_measurement.channel2_amplitude,
                "CH2 \xB7\xF9\xD6\xB5%.4f      ",
                "CH2 AMP %.4f      ");
    OS_Num_Show(CONTENT_LEFT, 160U, 24U, 1U,
                (double)s_measurement.selected_bin,
                "FFT\xC6\xB5\xC2\xCA\xB5\xA5\xD4\xAA %.0f      ",
                "FFT BIN %.0f      ");
    OS_Num_Show(DATA_VALUE_X, 160U, 24U, 1U,
                (double)s_measurement.phase_valid,
                "\xD3\xD0\xD0\xA7 %.0f      ", "VALID %.0f      ");
}

static void draw_known_page(void)
{
    refresh_known_measurement_values();
    UI_String_Show(CONTENT_LEFT, 220U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"ADC1+ADC2\xCD\xAC\xB2\xBD DMA",
                   "ADC1+ADC2 SYNCHRONIZED DMA");
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP");
}

static void refresh_unknown_measurement_values(void)
{
    OS_Num_Show(CONTENT_LEFT, 96U, 24U, 1U,
                (double)s_measurement.adc_overrun_count,
                "DMA\xD2\xE7\xB3\xF6 %.0f      ",
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
}

static void draw_unknown_page(void)
{
    refresh_unknown_measurement_values();
    UI_String_Show(CONTENT_LEFT, 220U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"256kHz ADC / 256 POINT FFT",
                   "256kHz ADC / 256 POINT FFT");
    UI_String_Show(CONTENT_LEFT, 244U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP");
}

static void draw_output_page(void)
{
    UI_String_Show(CONTENT_LEFT, 96U, UI_PRIMARY_SIZE, 1U,
                   (const uint8_t *)"VOFA FireWater", "VOFA FIREWATER");
    UI_String_Show(CONTENT_LEFT, 144U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"CH1,CH2", "CH1, CH2 WAVEFORM");
    UI_String_Show(CONTENT_LEFT, 168U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"TIM_FREQ,TIM_PERIOD",
                   "TIM_FREQ, TIM_PERIOD");
    UI_String_Show(CONTENT_LEFT, 192U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"TIM_WAVE", "TIM_WAVE RECONSTRUCTED");
    UI_String_Show(CONTENT_LEFT, 240U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"USART6 PC6/PC7 230400",
                   "USART6 PC6/PC7 230400");
    UI_String_Show(CONTENT_LEFT, 264U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK STOP/MAIN");
}

static void draw_ads_calibration_page(void)
{
    char buffer[64];
    uint8_t index;

    UI_String_Show(CONTENT_LEFT, 76U, UI_PRIMARY_SIZE, 1U,
                   (const uint8_t *)"PGA4311 4CH GAIN",
                   "PGA4311 4CH GAIN");
    for (index = 0U; index < 4U; index++)
    {
        (void)sprintf(buffer, "%c CH%u  %+5.1fdB  N=%3u      ",
                      (index == s_pga_channel) ? '>' : ' ',
                      (unsigned int)(index + 1U),
                      (double)s_pga_gain_db[index],
                      (unsigned int)(192.0f +
                                     2.0f * s_pga_gain_db[index]));
        UI_String_Show(CONTENT_LEFT,
                       (uint16_t)(120U + (uint16_t)index * 32U),
                       UI_HELP_SIZE, 1U,
                       (const uint8_t *)buffer, buffer);
    }
    UI_String_Show(CONTENT_LEFT, 264U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"/ CH   +/- 0.5dB",
                   "/ CH   +/- 0.5dB");
    UI_String_Show(CONTENT_LEFT, 288U, UI_HELP_SIZE, 1U,
                   (const uint8_t *)"* RESET ALL 0dB",
                   "* RESET ALL 0dB");
    UI_String_Show(CONTENT_LEFT, 312U, UI_HELP_SIZE, 1U,
                   gbk_back, "BACK MAIN");
}

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
        draw_output_page();
    }
}

static void render_page(void)
{
    clear_right_panel();
    draw_page_content();
    draw_menu_selection();
}

static void refresh_page_data(void)
{
    draw_page_content();
}

static void refresh_measurement_data(void)
{
    if (s_page == PAGE_SINE)
    {
        refresh_sine_measurement_values();
    }
    else if (s_page == PAGE_KNOWN)
    {
        refresh_known_measurement_values();
    }
    else if (s_page == PAGE_UNKNOWN)
    {
        refresh_unknown_measurement_values();
    }
}

static void switch_page(UiPage next_page)
{
    if (next_page <= PAGE_ADS_TEST)
    {
        if ((next_page == PAGE_KNOWN) || (next_page == PAGE_SINE))
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
        if (next_page == PAGE_ADS_TEST)
        {
            s_calibration_item = UI_CALIBRATION_ADS_REFERENCE;
        }
        s_page = next_page;
        render_page();
    }
}

void UI_SetChineseFontAvailable(uint8_t available)
{
    s_chinese_font_available = (available != 0U) ? 1U : 0U;
}

void UI_Init(void)
{
    s_page = PAGE_MAIN;
    s_frequency_hz = UI_DEFAULT_FREQUENCY_HZ;
    s_amplitude_mv = UI_DEFAULT_AMPLITUDE_MV;
    s_phase_deg = UI_DEFAULT_PHASE_DEG;
    s_known_target_peak_v = UI_DEFAULT_KNOWN_TARGET_PEAK_V;
    s_calibration_item = UI_CALIBRATION_ADS_REFERENCE;
    s_pga_channel = 0U;
    s_pga_gain_db[0] = 0.0f;
    s_pga_gain_db[1] = 0.0f;
    s_pga_gain_db[2] = 0.0f;
    s_pga_gain_db[3] = 0.0f;
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
    command->pga_channel = (uint8_t)(s_pga_channel + 1U);
    command->pga_gain_db = s_pga_gain_db[s_pga_channel];

    if (key == PS2_KEY_VALUE_BACKSPACE)
    {
        command->type = (s_page == PAGE_MAIN) ?
                        UI_COMMAND_NONE : UI_COMMAND_STOP_OUTPUT;
        switch_page(PAGE_MAIN);
        return (command->type != UI_COMMAND_NONE);
    }
    if ((s_page == PAGE_MAIN) &&
        (key >= PS2_KEY_VALUE_1) && (key <= PS2_KEY_VALUE_5))
    {
        switch_page((UiPage)key);
        if (key == PS2_KEY_VALUE_1)
        {
            command->type = UI_COMMAND_START_FREQUENCY_MEASUREMENT;
        }
        else if (key == PS2_KEY_VALUE_2)
        {
            command->type = UI_COMMAND_RUN_KNOWN_MODEL;
        }
        else if (key == PS2_KEY_VALUE_3)
        {
            command->type = UI_COMMAND_START_UNKNOWN_MODEL;
        }
        else if (key == PS2_KEY_VALUE_4)
        {
            command->type = UI_COMMAND_START_INFERENCE;
        }
        return (command->type != UI_COMMAND_NONE);
    }

    if (s_page == PAGE_SINE)
    {
        if (key == PS2_KEY_VALUE_ENTER)
        {
            command->type = UI_COMMAND_START_FREQUENCY_MEASUREMENT;
        }
    }
    else if (s_page == PAGE_KNOWN)
    {
        if (key == PS2_KEY_VALUE_ADD)
        {
            s_phase_deg = (s_phase_deg < 350U) ? s_phase_deg + 10U : 360U;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MINUS)
        {
            s_phase_deg = (s_phase_deg > 10U) ? s_phase_deg - 10U : 0U;
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_ENTER)
        {
            command->type = UI_COMMAND_RUN_KNOWN_MODEL;
        }
    }
    else if (s_page == PAGE_ADS_TEST)
    {
        if (key == PS2_KEY_VALUE_DIVIDE)
        {
            s_pga_channel = (uint8_t)((s_pga_channel + 1U) % 4U);
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_ADD)
        {
            if (s_pga_gain_db[s_pga_channel] < 31.5f)
            {
                s_pga_gain_db[s_pga_channel] += 0.5f;
            }
            command->type = UI_COMMAND_SET_PGA_GAIN;
            command->pga_channel = (uint8_t)(s_pga_channel + 1U);
            command->pga_gain_db = s_pga_gain_db[s_pga_channel];
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MINUS)
        {
            if (s_pga_gain_db[s_pga_channel] > -95.5f)
            {
                s_pga_gain_db[s_pga_channel] -= 0.5f;
            }
            command->type = UI_COMMAND_SET_PGA_GAIN;
            command->pga_channel = (uint8_t)(s_pga_channel + 1U);
            command->pga_gain_db = s_pga_gain_db[s_pga_channel];
            refresh_page_data();
        }
        else if (key == PS2_KEY_VALUE_MULTIPLY)
        {
            s_pga_gain_db[0] = 0.0f;
            s_pga_gain_db[1] = 0.0f;
            s_pga_gain_db[2] = 0.0f;
            s_pga_gain_db[3] = 0.0f;
            command->type = UI_COMMAND_RESET_PGA_GAIN;
            command->pga_gain_db = 0.0f;
            refresh_page_data();
        }
    }
    else if (key == PS2_KEY_VALUE_ENTER)
    {
        if (s_page == PAGE_UNKNOWN)
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

void UI_UpdateMeasurement(const UI_Measurement_t *measurement)
{
    if (measurement == NULL)
    {
        return;
    }
    s_measurement = *measurement;
    if ((measurement->event == UI_EVENT_MEASUREMENT) &&
        ((s_page == PAGE_SINE) || (s_page == PAGE_KNOWN) ||
         (s_page == PAGE_UNKNOWN)))
    {
        refresh_measurement_data();
    }
    else if ((measurement->event == UI_EVENT_STATUS) &&
             ((s_page == PAGE_MAIN) || (s_page == PAGE_OUTPUT) ||
              (s_page == PAGE_ADS_TEST)))
    {
        refresh_page_data();
    }
    else if ((measurement->event == UI_EVENT_STATUS) &&
             ((s_page == PAGE_SINE) || (s_page == PAGE_KNOWN) ||
              (s_page == PAGE_UNKNOWN)))
    {
        refresh_measurement_data();
    }
}

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

void Show_Val(uint8_t location, float value, char *format)
{
    uint16_t x;
    uint16_t y;

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

void MenuHaddler_1(void) { switch_page(PAGE_SINE); }
void MenuHaddler_2(void) { switch_page(PAGE_KNOWN); }
void MenuHaddler_3(void) { switch_page(PAGE_UNKNOWN); }
void MenuHaddler_4(void) { switch_page(PAGE_OUTPUT); }
void MenuHaddler_5(void) { switch_page(PAGE_ADS_TEST); }

float PS2_ReadNum(float num)
{
    return num;
}
