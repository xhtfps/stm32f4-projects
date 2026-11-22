#ifndef PHASE_APP_H
#define PHASE_APP_H

#include <stdint.h>

#include "user.h"

typedef void (*PhaseAppMeasurementPublisher_t)(
    const UI_Measurement_t *measurement);

void phase_app_set_measurement_publisher(
    PhaseAppMeasurementPublisher_t publisher);
uint8_t phase_app_init(void);
void phase_app_step(void);
void phase_app_handle_command(const UI_Command_t *command);

#endif /* PHASE_APP_H */
