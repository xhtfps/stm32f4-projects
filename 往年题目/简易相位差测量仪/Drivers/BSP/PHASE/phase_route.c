#include "phase_route.h"

#include <stddef.h>

static PhaseOut3Input_t s_out3_input = PHASE_OUT3_NONE;
static uint8_t s_phase_route_initialized;

static void PhaseRoute_Write(GPIO_TypeDef *port, uint16_t pin,
                             GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

HAL_StatusTypeDef PhaseRoute_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint16_t all_pins;

    if (s_phase_route_initialized != 0U)
    {
        return HAL_OK;
    }

    __HAL_RCC_GPIOG_CLK_ENABLE();
    all_pins = PHASE_ROUTE_OUT1_CTRL_PIN |
               PHASE_ROUTE_OUT2_CTRL_PIN |
               PHASE_ROUTE_OUT3_IN5_PIN |
               PHASE_ROUTE_OUT3_IN6_PIN |
               PHASE_ROUTE_OUT3_IN7_PIN |
               PHASE_ROUTE_OUT4_CTRL_PIN |
               PHASE_ROUTE_OUT5_CTRL_PIN;

    /*
     * Preload safe levels before changing MODER.  This prevents a short relay
     * pulse while the pins transition from reset-state inputs to outputs.
     */
    PhaseRoute_Write(PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT1_CTRL_PIN,
                     PHASE_ROUTE_OUT1_RAW_STATE);
    PhaseRoute_Write(PHASE_ROUTE_GPIO_PORT,
                     PHASE_ROUTE_OUT2_CTRL_PIN |
                     PHASE_ROUTE_OUT4_CTRL_PIN |
                     PHASE_ROUTE_OUT5_CTRL_PIN,
                     PHASE_ROUTE_ADC_DIRECT_STATE);
    PhaseRoute_Write(PHASE_ROUTE_GPIO_PORT,
                     PHASE_ROUTE_OUT3_IN5_PIN |
                     PHASE_ROUTE_OUT3_IN6_PIN |
                     PHASE_ROUTE_OUT3_IN7_PIN,
                     PHASE_ROUTE_OUT3_INACTIVE_STATE);

    gpio.Pin = all_pins;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PHASE_ROUTE_GPIO_PORT, &gpio);

    s_out3_input = PHASE_OUT3_NONE;
    s_phase_route_initialized = 1U;
    return HAL_OK;
}

PhaseCaptureRoute_t PhaseRoute_DecideCapture(
    PhaseCaptureRoute_t current_route, uint32_t frequency_hz)
{
    if ((current_route == PHASE_CAPTURE_ROUTE_RAW) &&
        (frequency_hz > PHASE_ROUTE_CAPTURE_UP_HZ))
    {
        return PHASE_CAPTURE_ROUTE_DIV32;
    }
    if ((current_route == PHASE_CAPTURE_ROUTE_DIV32) &&
        (frequency_hz < PHASE_ROUTE_CAPTURE_DOWN_HZ))
    {
        return PHASE_CAPTURE_ROUTE_RAW;
    }
    return current_route;
}

PhaseAdcMode_t PhaseRoute_DecideAdc(PhaseAdcMode_t current_mode,
                                    uint32_t frequency_hz)
{
    if ((current_mode == PHASE_ADC_MODE_DIRECT) &&
        (frequency_hz > PHASE_ROUTE_MIX_UP_HZ))
    {
        return PHASE_ADC_MODE_MIXED;
    }
    if ((current_mode == PHASE_ADC_MODE_MIXED) &&
        (frequency_hz < PHASE_ROUTE_MIX_DOWN_HZ))
    {
        return PHASE_ADC_MODE_DIRECT;
    }
    return current_mode;
}

void PhaseRoute_SetMainRelays(PhaseCaptureRoute_t capture_route,
                              PhaseAdcMode_t adc_mode)
{
    GPIO_PinState capture_state;
    GPIO_PinState adc_state;

    if (s_phase_route_initialized == 0U)
    {
        return;
    }

    capture_state = (capture_route == PHASE_CAPTURE_ROUTE_DIV32) ?
                    PHASE_ROUTE_OUT1_DIV32_STATE :
                    PHASE_ROUTE_OUT1_RAW_STATE;
    adc_state = (adc_mode == PHASE_ADC_MODE_MIXED) ?
                PHASE_ROUTE_ADC_MIXED_STATE :
                PHASE_ROUTE_ADC_DIRECT_STATE;

    PhaseRoute_Write(PHASE_ROUTE_GPIO_PORT,
                     PHASE_ROUTE_OUT1_CTRL_PIN, capture_state);
    PhaseRoute_Write(PHASE_ROUTE_GPIO_PORT,
                     PHASE_ROUTE_OUT2_CTRL_PIN |
                     PHASE_ROUTE_OUT4_CTRL_PIN |
                     PHASE_ROUTE_OUT5_CTRL_PIN,
                     adc_state);
}

void PhaseRoute_Out3DisconnectAll(void)
{
    if (s_phase_route_initialized == 0U)
    {
        return;
    }

    PhaseRoute_Write(PHASE_ROUTE_GPIO_PORT,
                     PHASE_ROUTE_OUT3_IN5_PIN |
                     PHASE_ROUTE_OUT3_IN6_PIN |
                     PHASE_ROUTE_OUT3_IN7_PIN,
                     PHASE_ROUTE_OUT3_INACTIVE_STATE);
    s_out3_input = PHASE_OUT3_NONE;
}

HAL_StatusTypeDef PhaseRoute_Out3ConnectOnly(PhaseOut3Input_t input)
{
    uint16_t target_pin;

    if (s_phase_route_initialized == 0U)
    {
        return HAL_ERROR;
    }
    if (input == PHASE_OUT3_IN5)
    {
        target_pin = PHASE_ROUTE_OUT3_IN5_PIN;
    }
    else if (input == PHASE_OUT3_IN6)
    {
        target_pin = PHASE_ROUTE_OUT3_IN6_PIN;
    }
    else
    {
        PhaseRoute_Out3DisconnectAll();
        return HAL_ERROR;
    }

    /*
     * Always force every coil off first.  IN7 has no valid target and can
     * therefore never be enabled through this API.
     */
    PhaseRoute_Out3DisconnectAll();
    PhaseRoute_Write(PHASE_ROUTE_GPIO_PORT, target_pin,
                     PHASE_ROUTE_OUT3_ACTIVE_STATE);
    s_out3_input = input;
    return HAL_OK;
}

PhaseOut3Input_t PhaseRoute_GetOut3Input(void)
{
    return s_out3_input;
}

void PhaseRoute_GetDiagnostics(PhaseRouteDiagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }

    diagnostics->initialized = s_phase_route_initialized;
    diagnostics->out1_ctrl = (uint8_t)HAL_GPIO_ReadPin(
        PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT1_CTRL_PIN);
    diagnostics->out2_ctrl = (uint8_t)HAL_GPIO_ReadPin(
        PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT2_CTRL_PIN);
    diagnostics->out3_in5 = (uint8_t)HAL_GPIO_ReadPin(
        PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT3_IN5_PIN);
    diagnostics->out3_in6 = (uint8_t)HAL_GPIO_ReadPin(
        PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT3_IN6_PIN);
    diagnostics->out3_in7 = (uint8_t)HAL_GPIO_ReadPin(
        PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT3_IN7_PIN);
    diagnostics->out4_ctrl = (uint8_t)HAL_GPIO_ReadPin(
        PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT4_CTRL_PIN);
    diagnostics->out5_ctrl = (uint8_t)HAL_GPIO_ReadPin(
        PHASE_ROUTE_GPIO_PORT, PHASE_ROUTE_OUT5_CTRL_PIN);
    diagnostics->out3_input = s_out3_input;
}

uint32_t PhaseRoute_GetDivider(PhaseCaptureRoute_t capture_route)
{
    return (capture_route == PHASE_CAPTURE_ROUTE_DIV32) ? 32U : 1U;
}

PhaseOut3Input_t PhaseRoute_GetOut3Target(PhaseAdcMode_t adc_mode)
{
    return (adc_mode == PHASE_ADC_MODE_MIXED) ?
           PHASE_OUT3_IN6 : PHASE_OUT3_IN5;
}
