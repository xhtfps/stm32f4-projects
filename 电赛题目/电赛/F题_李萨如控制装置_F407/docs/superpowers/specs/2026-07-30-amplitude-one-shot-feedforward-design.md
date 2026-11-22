# Amplitude one-shot feedforward design

## Problem

Requirements 2 and 3 continuously compare the filtered input Vpp with the Vpp used to establish amplitude feedforward. A difference of 100 mV or 5 percent for five blocks recalculates feedforward. Retaining the existing PI correction while changing the feedforward term changes the total DDS amplitude command immediately, so ADC measurement movement near 100 mV can create an approximately quarter-division output step.

## Design

Amplitude feedforward is established once after entering a wired mode. After `amplitude_feedforward_ready` becomes true, target tracking is handled only by the existing filtered amplitude PI. The existing 30 mV PI deadband, clipping reduction, initial five-block stability check, amplitude calibration, and AD9959 conversion remain unchanged.

Requirement 4 already starts with a ready feedforward value and therefore also avoids runtime feedforward recalculation. Selecting another requirement-4 division restarts the controller and establishes the new feedforward normally.

## Acceptance

- Initial feedforward still becomes ready after stable measurements.
- A later 100 to 125 mV measurement change does not rewrite feedforward.
- Amplitude PI continues to correct errors larger than 30 mV.
- No AD9959, amplitude conversion, or calibration constant is changed.
