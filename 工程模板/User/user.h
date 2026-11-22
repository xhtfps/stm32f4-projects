#ifndef USER_H
#define USER_H

#include <stdbool.h>
#include <stdint.h>

#include "./BSP/PS2/ps2.h"

/* 参考工程固定的菜单、版本和参数宏；仅把题目内容改为相位差测量。 */
#define TitleLength   5U
#define TitleStr      "\xCF\xE0\xCE\xBB\xB2\xEE\xB2\xE2\xC1\xBF\xD2\xC7"
#define ModelVerStr   "F4 10Hz-5MHz"
#define UserVerStr    "V1.0"
#define MenuChoiceNum 5U
#define Menu1Choice1  "1 \xC6\xB5\xC2\xCA\xB2\xE2\xC1\xBF"
#define Menu1Choice2  "2 \xCF\xE0\xCE\xBB\xB2\xEE\xB2\xE2\xC1\xBF"
#define Menu1Choice3  "3 ADC/FFT\xB2\xE2\xC1\xBF"
#define Menu1Choice4  "4 VOFA\xCA\xFD\xBE\xDD\xCA\xE4\xB3\xF6"
#define Menu1Choice5  "5 PGA4311\xD4\xF6\xD2\xE6"

#define UI_MENU_COUNT              5U
#define UI_DEFAULT_FREQUENCY_HZ    1000U
#define UI_MIN_FREQUENCY_HZ        10U
#define UI_MAX_FREQUENCY_HZ        5000000U
#define UI_KNOWN_MAX_FREQUENCY_HZ  5000000U
#define UI_KNOWN_FREQUENCY_STEP_HZ 100U
#define UI_DEFAULT_AMPLITUDE_MV    250.0f
#define UI_MAX_AMPLITUDE_MV        500.0f
#define UI_DEFAULT_PHASE_DEG       0U
#define UI_DEFAULT_KNOWN_TARGET_PEAK_V 0.0f
#define UI_KNOWN_TARGET_MIN_VPP    0.0f
#define UI_KNOWN_TARGET_MAX_VPP    0.0f
#define UI_KNOWN_TARGET_STEP_VPP   0.1f

typedef enum
{
    UI_COMMAND_NONE = 0,
    UI_COMMAND_START_FREQUENCY_MEASUREMENT,
    UI_COMMAND_SET_SINE,
    UI_COMMAND_STOP_OUTPUT,
    UI_COMMAND_RUN_KNOWN_MODEL,
    UI_COMMAND_START_UNKNOWN_MODEL,
    UI_COMMAND_START_INFERENCE,
    UI_COMMAND_SET_PGA_GAIN,
    UI_COMMAND_RESET_PGA_GAIN,
    UI_COMMAND_TEST_ADS1256,
    UI_COMMAND_RUN_CALIBRATION
} UI_CommandType_t;

typedef enum
{
    UI_CALIBRATION_ADS_REFERENCE = 1U,
    UI_CALIBRATION_ZERO_OFFSET,
    UI_CALIBRATION_CHANNEL_GAIN,
    UI_CALIBRATION_SWEEP,
    UI_CALIBRATION_IIR_DEBUG,
    UI_CALIBRATION_DAC_TEST
} UI_CalibrationItem_t;

typedef struct
{
    UI_CommandType_t type;
    uint32_t frequency_hz;
    float amplitude_mv;
    uint32_t phase_deg;
    float target_output_peak_v;
    uint8_t calibration_item;
    uint8_t pga_channel;
    float pga_gain_db;
} UI_Command_t;

typedef enum
{
    UI_EVENT_NONE = 0U,
    UI_EVENT_MEASUREMENT,
    UI_EVENT_SWEEP_PROGRESS,
    UI_EVENT_MODEL_RESULT,
    UI_EVENT_STATUS,
    UI_EVENT_ADS_TEST
} UI_EventType_t;

/* 与参考工程相同的统一测量事件载体，追加本题相位测量字段。 */
typedef uint8_t CircuitModelType_t;

typedef struct
{
    UI_EventType_t event;
    uint32_t frequency_hz;
    float input_rms;
    float output_rms;
    float gain_db;
    float expected_output_rms;
    float source_amplitude_mv;
    float target_output_peak_v;
    float output_peak_v;
    float expected_output_peak_v;
    uint16_t sweep_index;
    uint16_t sweep_total;
    CircuitModelType_t model_type;
    float f1_hz;
    float f2_hz;
    float center_hz;
    float q;
    uint8_t valid;

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

    uint32_t divided_frequency_hz;
    float frequency_hz_precise;
    float divided_frequency_hz_precise;
    uint32_t period_ticks;
    uint32_t capture_cycles;
    uint32_t divider;
    uint32_t adc_overrun_count;
    uint16_t selected_bin;
    float channel1_amplitude;
    float channel2_amplitude;
    float phase_difference_deg;
    uint8_t frequency_valid;
    uint8_t phase_valid;
    uint8_t status_code;
    char status_text[32];
} UI_Measurement_t;

void User_main(void);
void Init_All(void);
void Disp_Main(void);
void Show_Val(uint8_t location, float value, char *format);
void Change_Menu(uint8_t menu_sign);
void MenuHaddler_1(void);
void MenuHaddler_2(void);
void MenuHaddler_3(void);
void MenuHaddler_4(void);
void MenuHaddler_5(void);
float PS2_ReadNum(float num);

void UI_Init(void);
void UI_SetChineseFontAvailable(uint8_t available);
bool UI_HandleKey(PS2_KeyValue_t key, UI_Command_t *command);
void UI_UpdateMeasurement(const UI_Measurement_t *measurement);

#endif /* USER_H */
