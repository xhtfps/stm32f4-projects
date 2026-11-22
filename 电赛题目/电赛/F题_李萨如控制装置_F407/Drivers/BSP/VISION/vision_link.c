#include "vision_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "./BSP/DEBUG/debug_uart6.h"

#define VISION_LINK_LINE_SIZE          192U
#define VISION_LINK_ONLINE_TIMEOUT_MS  1000U
#define VISION_LINK_MAX_BYTES_PER_POLL 256U

static char s_line[VISION_LINK_LINE_SIZE];
static uint16_t s_line_length;
static uint8_t s_collecting;
static uint32_t s_last_good_ms;
static uint32_t s_good_frames;
static uint32_t s_bad_frames;

static uint16_t VisionLink_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 1U) != 0U) ?
                (uint16_t)((crc >> 1U) ^ 0xA001U) :
                (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static uint8_t VisionLink_ParseU32(const char *text, uint32_t *value)
{
    char *end;
    unsigned long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
        return 0U;
    parsed = strtoul(text, &end, 10);
    if ((*end != '\0') || (parsed > 0xFFFFFFFFUL))
        return 0U;
    *value = (uint32_t)parsed;
    return 1U;
}

static uint8_t VisionLink_ParseI32(const char *text, int32_t *value)
{
    char *end;
    long parsed;

    if ((text == NULL) || (value == NULL) || (*text == '\0'))
        return 0U;
    parsed = strtol(text, &end, 10);
    if (*end != '\0')
        return 0U;
    *value = (int32_t)parsed;
    return 1U;
}

static uint8_t VisionLink_ParseLine(VisionLinkMeasurement_t *measurement)
{
    uint32_t values[9];
    int32_t signed_values[4];
    char *tokens[14];
    char *token;
    char *star;
    char *end;
    uint16_t expected_crc;
    uint16_t actual_crc;
    uint8_t index = 0U;

    if ((measurement == NULL) || (s_line[0] != '$'))
        return 0U;
    star = strchr(s_line, '*');
    if ((star == NULL) || ((star - s_line) < 3) ||
        (strlen(star + 1) != 4U))
        return 0U;

    expected_crc = (uint16_t)strtoul(star + 1, &end, 16);
    if ((*end != '\0') || (end != star + 5))
        return 0U;
    actual_crc = VisionLink_Crc16(
        (const uint8_t *)&s_line[1],
        (uint16_t)(star - &s_line[1]));
    if (actual_crc != expected_crc)
        return 0U;

    *star = '\0';
    token = strtok(&s_line[1], ",");
    while ((token != NULL) && (index < 14U))
    {
        tokens[index++] = token;
        token = strtok(NULL, ",");
    }
    if ((index != 14U) || (token != NULL) ||
        (strcmp(tokens[0], "V") != 0))
        return 0U;

    if ((VisionLink_ParseU32(tokens[1], &values[0]) == 0U) ||
        (VisionLink_ParseU32(tokens[2], &values[1]) == 0U) ||
        (VisionLink_ParseU32(tokens[3], &values[2]) == 0U) ||
        (VisionLink_ParseU32(tokens[4], &values[3]) == 0U) ||
        (VisionLink_ParseU32(tokens[5], &values[4]) == 0U) ||
        (VisionLink_ParseU32(tokens[6], &values[5]) == 0U) ||
        (VisionLink_ParseU32(tokens[7], &values[6]) == 0U) ||
        (VisionLink_ParseU32(tokens[8], &values[7]) == 0U) ||
        (VisionLink_ParseI32(tokens[9], &signed_values[0]) == 0U) ||
        (VisionLink_ParseI32(tokens[10], &signed_values[1]) == 0U) ||
        (VisionLink_ParseI32(tokens[11], &signed_values[2]) == 0U) ||
        (VisionLink_ParseI32(tokens[12], &signed_values[3]) == 0U) ||
        (VisionLink_ParseU32(tokens[13], &values[8]) == 0U))
        return 0U;

    if ((values[0] > 65535U) || (values[1] > 65535U) ||
        (values[2] > 1U) || (values[3] > 255U) ||
        (values[4] > 255U) || (values[5] > 1000U) ||
        (values[6] > 65535U) || (values[7] > 65535U) ||
        (values[8] > 65535U) ||
        (signed_values[0] < -32768L) ||
        (signed_values[0] > 32767L) ||
        (signed_values[1] < -32768L) ||
        (signed_values[1] > 32767L))
        return 0U;

    measurement->sequence = (uint16_t)values[0];
    measurement->probe_id = (uint16_t)values[1];
    measurement->valid = (uint8_t)values[2];
    measurement->ratio_y = (uint8_t)values[3];
    measurement->ratio_x = (uint8_t)values[4];
    measurement->confidence_permille = (uint16_t)values[5];
    measurement->span_x_millidiv = (uint16_t)values[6];
    measurement->span_y_millidiv = (uint16_t)values[7];
    measurement->center_x_millidiv = (int16_t)signed_values[0];
    measurement->center_y_millidiv = (int16_t)signed_values[1];
    measurement->phase_error_millideg = signed_values[2];
    measurement->phase_rate_millideg_s = signed_values[3];
    measurement->quality_flags = (uint16_t)values[8];
    return 1U;
}

void VisionLink_Init(void)
{
    VisionLink_Reset();
    s_good_frames = 0U;
    s_bad_frames = 0U;
}

void VisionLink_Reset(void)
{
    uint8_t discarded;

    s_line_length = 0U;
    s_collecting = 0U;
    s_last_good_ms = 0U;
    while (DebugUart6_ReadByte(&discarded) != 0U)
    {
    }
}

uint8_t VisionLink_Poll(uint32_t now_ms,
                        VisionLinkMeasurement_t *measurement)
{
    VisionLinkMeasurement_t parsed;
    uint16_t processed = 0U;
    uint8_t data;
    uint8_t found = 0U;

    if (measurement == NULL)
        return 0U;

    while ((processed < VISION_LINK_MAX_BYTES_PER_POLL) &&
           (DebugUart6_ReadByte(&data) != 0U))
    {
        ++processed;
        if (data == '$')
        {
            s_line[0] = '$';
            s_line_length = 1U;
            s_collecting = 1U;
        }
        else if ((data == '\r') || (data == '\n'))
        {
            if ((s_collecting != 0U) && (s_line_length > 1U))
            {
                s_line[s_line_length] = '\0';
                if (VisionLink_ParseLine(&parsed) != 0U)
                {
                    *measurement = parsed;
                    s_last_good_ms = now_ms;
                    ++s_good_frames;
                    found = 1U;
                }
                else
                {
                    ++s_bad_frames;
                }
            }
            s_line_length = 0U;
            s_collecting = 0U;
        }
        else if (s_collecting != 0U)
        {
            if (s_line_length < (VISION_LINK_LINE_SIZE - 1U))
                s_line[s_line_length++] = (char)data;
            else
            {
                s_line_length = 0U;
                s_collecting = 0U;
                ++s_bad_frames;
            }
        }
    }
    return found;
}

uint8_t VisionLink_SendProbe(uint16_t sequence,
                             uint16_t probe_id,
                             uint32_t frequency_millihz,
                             uint32_t phase_millideg,
                             uint8_t target_shape)
{
    char body[96];
    char frame[112];
    int body_length;
    int frame_length;
    uint16_t crc;

    body_length = snprintf(body, sizeof(body), "P,%u,%u,%lu,%lu,%u",
                           (unsigned int)sequence,
                           (unsigned int)probe_id,
                           (unsigned long)frequency_millihz,
                           (unsigned long)phase_millideg,
                           (unsigned int)target_shape);
    if ((body_length <= 0) || (body_length >= (int)sizeof(body)))
        return 0U;
    crc = VisionLink_Crc16((const uint8_t *)body,
                           (uint16_t)body_length);
    frame_length = snprintf(frame, sizeof(frame), "$%s*%04X\r\n",
                            body, (unsigned int)crc);
    if ((frame_length <= 0) || (frame_length >= (int)sizeof(frame)))
        return 0U;
    return (DebugUart6_Write((const uint8_t *)frame,
                             (uint16_t)frame_length) == HAL_OK) ? 1U : 0U;
}

uint8_t VisionLink_IsOnline(uint32_t now_ms)
{
    return ((s_good_frames != 0U) &&
            ((uint32_t)(now_ms - s_last_good_ms) <=
             VISION_LINK_ONLINE_TIMEOUT_MS)) ? 1U : 0U;
}

uint32_t VisionLink_GetGoodFrameCount(void)
{
    return s_good_frames;
}

uint32_t VisionLink_GetBadFrameCount(void)
{
    return s_bad_frames;
}
