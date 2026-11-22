#ifndef VISION_SLAVE_H
#define VISION_SLAVE_H

#include <stdint.h>

typedef enum
{
    VISION_SLAVE_COMMAND_NONE = 0U,
    VISION_SLAVE_COMMAND_SET,
    VISION_SLAVE_COMMAND_STOP,
    VISION_SLAVE_COMMAND_LOCKED,
    VISION_SLAVE_COMMAND_DONE,
    VISION_SLAVE_COMMAND_FAILED,
    VISION_SLAVE_COMMAND_RUN
} VisionSlaveCommandKind_t;

typedef struct
{
    VisionSlaveCommandKind_t kind;
    uint16_t sequence;
    uint16_t probe_id;
    uint32_t frequency_millihz;
    uint32_t input_frequency_millihz;
    uint32_t phase_millideg;
    float amplitude_mv;
    uint8_t target_shape;
    uint8_t error_code;
} VisionSlaveCommand_t;

void VisionSlave_Init(void);
uint8_t VisionSlave_Poll(uint32_t now_ms,
                         VisionSlaveCommand_t *command);
uint8_t VisionSlave_SendStart(uint8_t target_shape);
uint8_t VisionSlave_SendApplied(uint16_t sequence,
                                uint16_t probe_id);
uint8_t VisionSlave_SendError(uint16_t sequence,
                              uint16_t probe_id,
                              uint8_t error_code);
uint8_t VisionSlave_IsOnline(uint32_t now_ms);

#endif /* VISION_SLAVE_H */
