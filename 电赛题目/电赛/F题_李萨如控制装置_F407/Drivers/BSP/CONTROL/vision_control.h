#ifndef VISION_CONTROL_H
#define VISION_CONTROL_H

#include <stdint.h>

#include "./BSP/VISION/vision_link.h"

typedef enum
{
    VISION_CONTROL_IDLE = 0U,
    VISION_CONTROL_COARSE_SWEEP,
    VISION_CONTROL_LOCAL_CONFIRM,
    VISION_CONTROL_FREQUENCY_LOCK,
    VISION_CONTROL_AMPLITUDE_LOCK,
    VISION_CONTROL_PHASE_LOCK,
    VISION_CONTROL_VERIFY,
    VISION_CONTROL_HOLD,
    VISION_CONTROL_ERROR
} VisionControlStage_t;

typedef struct
{
    uint8_t apply_dds;
    uint8_t send_probe;
    uint32_t frequency_millihz;
    float amplitude_mv;
    uint32_t phase_millideg;
    uint16_t sequence;
    uint16_t probe_id;
    uint8_t target_shape;
} VisionControlAction_t;

typedef struct
{
    VisionControlStage_t stage;
    uint8_t active;
    uint8_t target_shape;
    uint8_t expected_ratio_y;
    uint8_t expected_ratio_x;
    uint8_t waiting_for_frame;
    uint8_t candidate_hits;
    uint8_t stable_count;
    uint8_t timeout_count;
    uint16_t sequence;
    uint16_t pending_probe_id;
    uint16_t probe_cursor;
    uint16_t current_probe_id;
    uint32_t start_ms;
    uint32_t deadline_ms;
    uint32_t candidate_input_millihz;
    uint32_t input_frequency_millihz;
    uint32_t output_frequency_millihz;
    float output_amplitude_mv;
    uint32_t output_phase_millideg;
    VisionLinkMeasurement_t latest;
} VisionControl_t;

void VisionControl_Init(VisionControl_t *control);
uint8_t VisionControl_Start(VisionControl_t *control,
                            uint8_t target_shape,
                            uint32_t now_ms);
void VisionControl_Stop(VisionControl_t *control);
void VisionControl_Abort(VisionControl_t *control);

/* Returns 1 when state/UI data changed or an action was emitted. */
uint8_t VisionControl_Process(VisionControl_t *control,
                              uint32_t now_ms,
                              const VisionLinkMeasurement_t *measurement,
                              VisionControlAction_t *action);

uint16_t VisionControl_GetProbeTotal(void);

#endif /* VISION_CONTROL_H */
