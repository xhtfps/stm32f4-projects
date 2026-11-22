#ifndef VISION_LINK_H
#define VISION_LINK_H

#include <stdint.h>

/* K230 -> STM32: all fixed-point fields avoid locale/float parsing issues. */
typedef struct
{
    uint16_t sequence;
    uint16_t probe_id;
    uint8_t valid;
    uint8_t ratio_y;
    uint8_t ratio_x;
    uint16_t confidence_permille;
    uint16_t span_x_millidiv;
    uint16_t span_y_millidiv;
    int16_t center_x_millidiv;
    int16_t center_y_millidiv;
    int32_t phase_error_millideg;
    int32_t phase_rate_millideg_s;
    uint16_t quality_flags;
} VisionLinkMeasurement_t;

void VisionLink_Init(void);
void VisionLink_Reset(void);

/* Drains USART6 and returns the newest CRC-valid V frame, if any. */
uint8_t VisionLink_Poll(uint32_t now_ms,
                        VisionLinkMeasurement_t *measurement);

/* STM32 -> K230 probe/feedback request. Returns 1 when queued. */
uint8_t VisionLink_SendProbe(uint16_t sequence,
                             uint16_t probe_id,
                             uint32_t frequency_millihz,
                             uint32_t phase_millideg,
                             uint8_t target_shape);

uint8_t VisionLink_IsOnline(uint32_t now_ms);
uint32_t VisionLink_GetGoodFrameCount(void);
uint32_t VisionLink_GetBadFrameCount(void);

#endif /* VISION_LINK_H */
