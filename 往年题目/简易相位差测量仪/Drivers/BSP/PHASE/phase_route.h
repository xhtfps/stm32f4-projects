#ifndef PHASE_ROUTE_H
#define PHASE_ROUTE_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

/*
 * Board-level relay mapping.
 *
 * The handover document defines the logical OUT/IN relationship but does not
 * contain MCU pin numbers.  The original project report used PG10/PG12/PG11/
 * PG13 for the four direct/mixed relay controls.  The mapping below preserves
 * that order and uses the adjacent PG9/PG14/PG15 pins for the new OUT1 and
 * OUT3 one-hot controls.  Verify these seven macros against the final harness
 * before energising the relay board; no application code writes them directly.
 * On the standard F407 base-board schematic PG9/PG10/PG12 are also labelled
 * FSMC_NE2/NE3/NE4, so they are not safe assumptions when external memory is
 * populated or initialized.
 */
#define PHASE_ROUTE_GPIO_PORT             GPIOG
#define PHASE_ROUTE_OUT1_CTRL_PIN          GPIO_PIN_9
#define PHASE_ROUTE_OUT2_CTRL_PIN          GPIO_PIN_10
#define PHASE_ROUTE_OUT3_IN6_PIN           GPIO_PIN_12
#define PHASE_ROUTE_OUT4_CTRL_PIN          GPIO_PIN_11
#define PHASE_ROUTE_OUT5_CTRL_PIN          GPIO_PIN_13
#define PHASE_ROUTE_OUT3_IN5_PIN           GPIO_PIN_14
#define PHASE_ROUTE_OUT3_IN7_PIN           GPIO_PIN_15

/*
 * OUT1 is fixed by the handover document: low selects IN1 and high selects
 * IN2.  The old relay code used low for direct and high for mixed on
 * OUT2/OUT4/OUT5.  OUT3 uses independent active-high relay drivers.
 */
#define PHASE_ROUTE_OUT1_RAW_STATE         GPIO_PIN_RESET
#define PHASE_ROUTE_OUT1_DIV32_STATE       GPIO_PIN_SET
#define PHASE_ROUTE_ADC_DIRECT_STATE       GPIO_PIN_RESET
#define PHASE_ROUTE_ADC_MIXED_STATE        GPIO_PIN_SET
#define PHASE_ROUTE_OUT3_ACTIVE_STATE      GPIO_PIN_SET
#define PHASE_ROUTE_OUT3_INACTIVE_STATE    GPIO_PIN_RESET

/*
 * OUT1 is an analogue-path selection, so its 4.9/5.1 MHz crossover remains
 * independent of the divider ratio.  IN2 now divides by 32.  TIM2's separate
 * ICPSC_DIV8 setting limits the capture-event rate; it is not an OUT1 switch
 * threshold.
 */
#define PHASE_ROUTE_CAPTURE_UP_HZ          5100000U
#define PHASE_ROUTE_CAPTURE_DOWN_HZ        4900000U
#define PHASE_ROUTE_MIX_UP_HZ               210000U
#define PHASE_ROUTE_MIX_DOWN_HZ             190000U

typedef enum
{
    PHASE_CAPTURE_ROUTE_RAW = 0U,
    PHASE_CAPTURE_ROUTE_DIV32 = 1U
} PhaseCaptureRoute_t;

typedef enum
{
    PHASE_ADC_MODE_DIRECT = 0U,
    PHASE_ADC_MODE_MIXED = 1U
} PhaseAdcMode_t;

typedef enum
{
    PHASE_OUT3_NONE = 0U,
    PHASE_OUT3_IN5 = 5U,
    PHASE_OUT3_IN6 = 6U
} PhaseOut3Input_t;

typedef struct
{
    uint8_t initialized;
    uint8_t out1_ctrl;
    uint8_t out2_ctrl;
    uint8_t out3_in5;
    uint8_t out3_in6;
    uint8_t out3_in7;
    uint8_t out4_ctrl;
    uint8_t out5_ctrl;
    PhaseOut3Input_t out3_input;
} PhaseRouteDiagnostics_t;

HAL_StatusTypeDef PhaseRoute_Init(void);

PhaseCaptureRoute_t PhaseRoute_DecideCapture(
    PhaseCaptureRoute_t current_route, uint32_t frequency_hz);
PhaseAdcMode_t PhaseRoute_DecideAdc(PhaseAdcMode_t current_mode,
                                    uint32_t frequency_hz);

void PhaseRoute_SetMainRelays(PhaseCaptureRoute_t capture_route,
                              PhaseAdcMode_t adc_mode);
void PhaseRoute_Out3DisconnectAll(void);
HAL_StatusTypeDef PhaseRoute_Out3ConnectOnly(PhaseOut3Input_t input);
PhaseOut3Input_t PhaseRoute_GetOut3Input(void);
void PhaseRoute_GetDiagnostics(PhaseRouteDiagnostics_t *diagnostics);

uint32_t PhaseRoute_GetDivider(PhaseCaptureRoute_t capture_route);
PhaseOut3Input_t PhaseRoute_GetOut3Target(PhaseAdcMode_t adc_mode);

#endif /* PHASE_ROUTE_H */
