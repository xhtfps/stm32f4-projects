#include "vision_slave.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "./BSP/DEBUG/debug_uart6.h"

#define VISION_SLAVE_LINE_SIZE          128U
#define VISION_SLAVE_ONLINE_TIMEOUT_MS  1000U
#define VISION_SLAVE_MAX_BYTES_PER_POLL 256U
#define VISION_SLAVE_MIN_DDS_MILLIHZ     1000UL
#define VISION_SLAVE_MAX_DDS_MILLIHZ     200000000UL
#define VISION_SLAVE_MIN_INPUT_MILLIHZ   1000000UL
#define VISION_SLAVE_MAX_INPUT_MILLIHZ   100000000UL

static char s_line[VISION_SLAVE_LINE_SIZE];
static uint16_t s_length;
static uint8_t s_collecting;
static uint32_t s_last_rx_ms;

static uint8_t VisionSlave_ParseU32(const char *text, uint32_t *value)
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

static uint8_t VisionSlave_ParseLine(VisionSlaveCommand_t *command)
{
    char *token;
    char *tokens[8];
    uint32_t values[6];
    uint8_t count = 0U;

    if ((command == NULL) || (s_line[0] == '\0'))
        return 0U;
    (void)memset(command, 0, sizeof(*command));

    token = strtok(s_line, ",");
    while ((token != NULL) && (count < 8U))
    {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }
    if (token != NULL)
        return 0U;

    if ((count == 2U) && (strcmp(tokens[0], "STOP") == 0))
    {
        if (VisionSlave_ParseU32(tokens[1], &values[0]) == 0U)
            return 0U;
        if (values[0] > 65535U)
            return 0U;
        command->kind = VISION_SLAVE_COMMAND_STOP;
        command->sequence = (uint16_t)values[0];
        return 1U;
    }

    /* RUN,shape: PC can start one of the three target modes without a UI key. */
    if ((count == 2U) && (strcmp(tokens[0], "RUN") == 0))
    {
        if ((VisionSlave_ParseU32(tokens[1], &values[0]) == 0U) ||
            (values[0] < 1U) || (values[0] > 3U))
            return 0U;
        command->kind = VISION_SLAVE_COMMAND_RUN;
        command->target_shape = (uint8_t)values[0];
        return 1U;
    }

    /* LOCKED/DONE,seq,input_frequency_millihz */
    if ((count == 3U) &&
        ((strcmp(tokens[0], "LOCKED") == 0) ||
         (strcmp(tokens[0], "DONE") == 0)))
    {
        if ((VisionSlave_ParseU32(tokens[1], &values[0]) == 0U) ||
            (VisionSlave_ParseU32(tokens[2], &values[1]) == 0U) ||
            (values[0] > 65535U) ||
            (values[1] < VISION_SLAVE_MIN_INPUT_MILLIHZ) ||
            (values[1] > VISION_SLAVE_MAX_INPUT_MILLIHZ))
            return 0U;
        command->kind = (strcmp(tokens[0], "LOCKED") == 0) ?
            VISION_SLAVE_COMMAND_LOCKED : VISION_SLAVE_COMMAND_DONE;
        command->sequence = (uint16_t)values[0];
        command->input_frequency_millihz = values[1];
        return 1U;
    }

    /* FAILED,seq,error_code */
    if ((count == 3U) && (strcmp(tokens[0], "FAILED") == 0))
    {
        if ((VisionSlave_ParseU32(tokens[1], &values[0]) == 0U) ||
            (VisionSlave_ParseU32(tokens[2], &values[1]) == 0U) ||
            (values[0] > 65535U) ||
            (values[1] == 0U) || (values[1] > 255U))
            return 0U;
        command->kind = VISION_SLAVE_COMMAND_FAILED;
        command->sequence = (uint16_t)values[0];
        command->error_code = (uint8_t)values[1];
        return 1U;
    }

    /* SET,seq,probe_id,frequency_millihz,phase_millideg,amp_mv,shape */
    if ((count != 7U) || (strcmp(tokens[0], "SET") != 0))
        return 0U;
    if ((VisionSlave_ParseU32(tokens[1], &values[0]) == 0U) ||
        (VisionSlave_ParseU32(tokens[2], &values[1]) == 0U) ||
        (VisionSlave_ParseU32(tokens[3], &values[2]) == 0U) ||
        (VisionSlave_ParseU32(tokens[4], &values[3]) == 0U) ||
        (VisionSlave_ParseU32(tokens[5], &values[4]) == 0U) ||
        (VisionSlave_ParseU32(tokens[6], &values[5]) == 0U))
        return 0U;
    if ((values[0] > 65535U) || (values[1] > 65535U) ||
        (values[2] < VISION_SLAVE_MIN_DDS_MILLIHZ) ||
        (values[2] > VISION_SLAVE_MAX_DDS_MILLIHZ) ||
        (values[3] >= 360000U) || (values[4] > 1000U) ||
        (values[5] < 1U) || (values[5] > 3U))
        return 0U;

    command->kind = VISION_SLAVE_COMMAND_SET;
    command->sequence = (uint16_t)values[0];
    command->probe_id = (uint16_t)values[1];
    command->frequency_millihz = values[2];
    command->phase_millideg = values[3];
    command->amplitude_mv = (float)values[4];
    command->target_shape = (uint8_t)values[5];
    return 1U;
}

static uint8_t VisionSlave_SendLine(const char *line)
{
    uint16_t length;

    if (line == NULL)
        return 0U;
    length = (uint16_t)strlen(line);
    return (DebugUart6_Write((const uint8_t *)line, length) == HAL_OK) ?
        1U : 0U;
}

void VisionSlave_Init(void)
{
    uint8_t discarded;

    s_length = 0U;
    s_collecting = 0U;
    s_last_rx_ms = 0U;
    while (DebugUart6_ReadByte(&discarded) != 0U)
    {
    }
}

uint8_t VisionSlave_Poll(uint32_t now_ms,
                         VisionSlaveCommand_t *command)
{
    VisionSlaveCommand_t parsed;
    uint16_t processed = 0U;
    uint8_t data;
    uint8_t found = 0U;

    if (command == NULL)
        return 0U;
    while ((processed < VISION_SLAVE_MAX_BYTES_PER_POLL) &&
           (DebugUart6_ReadByte(&data) != 0U))
    {
        ++processed;
        if (data == '$')
        {
            s_length = 0U;
            s_collecting = 1U;
        }
        else if ((data == '\r') || (data == '\n'))
        {
            if ((s_collecting != 0U) && (s_length != 0U))
            {
                s_line[s_length] = '\0';
                if (VisionSlave_ParseLine(&parsed) != 0U)
                {
                    *command = parsed;
                    s_last_rx_ms = now_ms;
                    found = 1U;
                }
            }
            s_length = 0U;
            s_collecting = 0U;
        }
        else if (s_collecting != 0U)
        {
            if (s_length < (VISION_SLAVE_LINE_SIZE - 1U))
                s_line[s_length++] = (char)data;
            else
            {
                s_length = 0U;
                s_collecting = 0U;
            }
        }
    }
    return found;
}

uint8_t VisionSlave_SendStart(uint8_t target_shape)
{
    char line[32];
    int length = snprintf(line, sizeof(line), "$START,%u\r\n",
                          (unsigned int)target_shape);
    if ((length <= 0) || (length >= (int)sizeof(line)))
        return 0U;
    return VisionSlave_SendLine(line);
}

uint8_t VisionSlave_SendApplied(uint16_t sequence,
                                uint16_t probe_id)
{
    char line[48];
    int length = snprintf(line, sizeof(line), "$APPLIED,%u,%u\r\n",
                          (unsigned int)sequence,
                          (unsigned int)probe_id);
    if ((length <= 0) || (length >= (int)sizeof(line)))
        return 0U;
    return VisionSlave_SendLine(line);
}

uint8_t VisionSlave_SendError(uint16_t sequence,
                              uint16_t probe_id,
                              uint8_t error_code)
{
    char line[56];
    int length = snprintf(line, sizeof(line), "$ERROR,%u,%u,%u\r\n",
                          (unsigned int)sequence,
                          (unsigned int)probe_id,
                          (unsigned int)error_code);
    if ((length <= 0) || (length >= (int)sizeof(line)))
        return 0U;
    return VisionSlave_SendLine(line);
}

uint8_t VisionSlave_IsOnline(uint32_t now_ms)
{
    return ((s_last_rx_ms != 0U) &&
            ((uint32_t)(now_ms - s_last_rx_ms) <=
             VISION_SLAVE_ONLINE_TIMEOUT_MS)) ? 1U : 0U;
}
