#include "pid_serial.h"

#include <stdio.h>
#include <string.h>

#include "./BSP/CONTROL/wired_control.h"
#include "./SYSTEM/usart/usart.h"

#define PID_SERIAL_LINE_SIZE          128U
#define PID_SERIAL_RESPONSE_SIZE      192U
#define PID_SERIAL_MAX_BYTES_PER_POLL 128U

static char s_pid_serial_line[PID_SERIAL_LINE_SIZE];
static uint16_t s_pid_serial_length;
static uint8_t s_pid_serial_discard_line;

static void PidSerial_SendText(const char *text)
{
    char response[PID_SERIAL_RESPONSE_SIZE];
    int length;

    if (text == NULL)
    {
        return;
    }

    length = snprintf(response, sizeof(response), "\r\n%s\r\n", text);
    if ((length > 0) && (length < (int)sizeof(response)))
    {
        (void)usart1_write_blocking(
            (const uint8_t *)response, (uint16_t)length);
    }
}

static void PidSerial_SendConfig(
    const char *name,
    WiredControlLoop_t loop)
{
    WiredPiConfig_t config;
    char response[PID_SERIAL_RESPONSE_SIZE];
    int length;

    if ((name == NULL) ||
        (wired_control_get_pid_config(loop, &config) == 0U))
    {
        PidSerial_SendText("@ERR,INTERNAL");
        return;
    }

    length = snprintf(
        response, sizeof(response),
        "\r\n@PID,%s,%.6f,%.6f,%.6f,%.6f\r\n",
        name, (double)config.kp, (double)config.ki,
        (double)config.kd,
        (double)config.derivative_filter_alpha);
    if ((length > 0) && (length < (int)sizeof(response)))
    {
        (void)usart1_write_blocking(
            (const uint8_t *)response, (uint16_t)length);
    }
}

static uint8_t PidSerial_ParseFloat(const char *text, float *value)
{
    float result = 0.0f;
    float fraction_scale = 0.1f;
    uint8_t negative = 0U;
    uint8_t has_digit = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }
    if ((*text == '+') || (*text == '-'))
    {
        negative = (*text == '-') ? 1U : 0U;
        ++text;
    }
    while ((*text >= '0') && (*text <= '9'))
    {
        has_digit = 1U;
        result = result * 10.0f + (float)(*text - '0');
        ++text;
    }
    if (*text == '.')
    {
        ++text;
        while ((*text >= '0') && (*text <= '9'))
        {
            has_digit = 1U;
            result += (float)(*text - '0') * fraction_scale;
            fraction_scale *= 0.1f;
            ++text;
        }
    }
    if ((has_digit == 0U) || (*text != '\0'))
    {
        return 0U;
    }

    *value = (negative != 0U) ? -result : result;
    return 1U;
}

static uint8_t PidSerial_ParseLoop(
    const char *text,
    WiredControlLoop_t *loop,
    const char **name)
{
    if ((text == NULL) || (loop == NULL) || (name == NULL))
    {
        return 0U;
    }
    if (strcmp(text, "FREQ") == 0)
    {
        *loop = WIRED_CONTROL_LOOP_FREQUENCY;
        *name = "FREQ";
    }
    else if (strcmp(text, "FREQ_LOW") == 0)
    {
        *loop = WIRED_CONTROL_LOOP_FREQUENCY_LOW;
        *name = "FREQ_LOW";
    }
    else if (strcmp(text, "PHASE") == 0)
    {
        *loop = WIRED_CONTROL_LOOP_PHASE;
        *name = "PHASE";
    }
    else if (strcmp(text, "PHASE_LOW") == 0)
    {
        *loop = WIRED_CONTROL_LOOP_PHASE_LOW;
        *name = "PHASE_LOW";
    }
    else if (strcmp(text, "AMP") == 0)
    {
        *loop = WIRED_CONTROL_LOOP_AMPLITUDE;
        *name = "AMP";
    }
    else
    {
        return 0U;
    }
    return 1U;
}

static void PidSerial_Uppercase(char *text)
{
    while ((text != NULL) && (*text != '\0'))
    {
        if ((*text >= 'a') && (*text <= 'z'))
        {
            *text = (char)(*text - ('a' - 'A'));
        }
        ++text;
    }
}

static void PidSerial_HandleLine(char *line)
{
    char *command;
    char *action;
    char *loop_text;
    char *kp_text;
    char *ki_text;
    char *kd_text;
    char *alpha_text;
    const char *loop_name;
    WiredControlLoop_t loop;
    float kp;
    float ki;
    float kd;
    float alpha;

    PidSerial_Uppercase(line);
    command = strtok(line, " \t");
    if ((command == NULL) || (strcmp(command, "PID") != 0))
    {
        return;
    }

    action = strtok(NULL, " \t");
    if (action == NULL)
    {
        PidSerial_SendText("@ERR,FORMAT");
        return;
    }
    if (strcmp(action, "GET") == 0)
    {
        PidSerial_SendConfig("FREQ", WIRED_CONTROL_LOOP_FREQUENCY);
        PidSerial_SendConfig("FREQ_LOW", WIRED_CONTROL_LOOP_FREQUENCY_LOW);
        PidSerial_SendConfig("PHASE", WIRED_CONTROL_LOOP_PHASE);
        PidSerial_SendConfig("PHASE_LOW", WIRED_CONTROL_LOOP_PHASE_LOW);
        PidSerial_SendConfig("AMP", WIRED_CONTROL_LOOP_AMPLITUDE);
        PidSerial_SendText("@END");
        return;
    }
    if (strcmp(action, "SET") != 0)
    {
        PidSerial_SendText("@ERR,ACTION");
        return;
    }

    loop_text = strtok(NULL, " \t");
    kp_text = strtok(NULL, " \t");
    ki_text = strtok(NULL, " \t");
    kd_text = strtok(NULL, " \t");
    alpha_text = strtok(NULL, " \t");
    if ((PidSerial_ParseLoop(loop_text, &loop, &loop_name) == 0U) ||
        (PidSerial_ParseFloat(kp_text, &kp) == 0U) ||
        (PidSerial_ParseFloat(ki_text, &ki) == 0U) ||
        (PidSerial_ParseFloat(kd_text, &kd) == 0U) ||
        (PidSerial_ParseFloat(alpha_text, &alpha) == 0U) ||
        (strtok(NULL, " \t") != NULL))
    {
        PidSerial_SendText("@ERR,FORMAT");
        return;
    }

    if (wired_control_set_pid_config(
            loop, kp, ki, kd, alpha) == 0U)
    {
        PidSerial_SendText("@ERR,RANGE");
        return;
    }
    PidSerial_SendConfig(loop_name, loop);
    PidSerial_SendText("@OK");
}

void PidSerial_Init(void)
{
    s_pid_serial_length = 0U;
    s_pid_serial_discard_line = 0U;
    (void)memset(s_pid_serial_line, 0, sizeof(s_pid_serial_line));
}

void PidSerial_Process(void)
{
    uint16_t processed = 0U;
    uint8_t data;

    while ((processed < PID_SERIAL_MAX_BYTES_PER_POLL) &&
           (usart1_read_byte(&data) != 0U))
    {
        ++processed;
        if ((data == '\r') || (data == '\n'))
        {
            if (s_pid_serial_discard_line != 0U)
            {
                PidSerial_SendText("@ERR,TOO_LONG");
            }
            else if (s_pid_serial_length != 0U)
            {
                s_pid_serial_line[s_pid_serial_length] = '\0';
                PidSerial_HandleLine(s_pid_serial_line);
            }
            s_pid_serial_length = 0U;
            s_pid_serial_discard_line = 0U;
        }
        else if (s_pid_serial_discard_line == 0U)
        {
            if ((data >= 32U) && (data <= 126U) &&
                (s_pid_serial_length < PID_SERIAL_LINE_SIZE - 1U))
            {
                s_pid_serial_line[s_pid_serial_length++] = (char)data;
            }
            else
            {
                s_pid_serial_discard_line = 1U;
            }
        }
    }
}
