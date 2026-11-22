#ifndef PHASE_CALIBRATION_H
#define PHASE_CALIBRATION_H

#include "phase_route.h"

#include <stdint.h>

#define PHASE_CALIBRATION_POINT_COUNT       13U
#define PHASE_CALIBRATION_SAMPLES_PER_POINT  8U

typedef enum
{
    PHASE_CALIBRATION_LEARN_NONE = 0U,
    PHASE_CALIBRATION_LEARN_NEXT_POINT = 1U,
    PHASE_CALIBRATION_LEARN_COMPLETE = 2U
} PhaseCalibrationLearnEvent_t;

typedef struct
{
    uint8_t calibration_valid;
    uint8_t learning_active;
    uint8_t save_ok;
    uint8_t point_index;
    uint8_t point_total;
    uint8_t sample_count;
    uint32_t target_frequency_hz;
    float last_phase_error_deg;
    float last_gain_error_db;
} PhaseCalibrationStatus_t;

/* Loads the last valid table from reserved internal Flash. */
uint8_t PhaseCalibration_Init(void);

/* Starts a new table while keeping the previous valid table active. */
void PhaseCalibration_StartLearning(void);
void PhaseCalibration_AbortLearning(void);
uint8_t PhaseCalibration_IsLearning(void);
uint32_t PhaseCalibration_GetTargetFrequency(void);

/*
 * Adds one raw, valid measurement.  A point is accepted only when the
 * measured frequency and ADC route match the current learning target.
 */
PhaseCalibrationLearnEvent_t PhaseCalibration_AddSample(
    uint32_t frequency_hz,
    PhaseAdcMode_t adc_mode,
    float raw_phase_deg,
    float raw_gain_db);

/* Saves the completed active table after acquisition has been stopped. */
uint8_t PhaseCalibration_Save(void);

/*
 * Corrects CH2-CH1 phase and gain using frequency interpolation within the
 * active direct or mixed calibration table.  Identity is used if no table
 * has been learned.
 */
void PhaseCalibration_Apply(uint32_t frequency_hz,
                            PhaseAdcMode_t adc_mode,
                            float *phase_deg,
                            float *gain_db);

void PhaseCalibration_GetStatus(PhaseCalibrationStatus_t *status);

#endif /* PHASE_CALIBRATION_H */
