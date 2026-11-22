# Phase ADC Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make PA1/PA2 phase measurement trustworthy from 1 kHz to 200 kHz analysis frequency and prevent an unverified ADC phase value from steering the phase loop.

**Architecture:** Keep the existing simultaneous ADC1/ADC2 packed DMA and 1024-point single-frequency DFT. Replace the fixed power-of-two sample-rate table with the approved frequency-ratio table, select the longest ADC acquisition time that fits each trigger period, and calculate an independent rising-zero-crossing phase from the same DMA block. The control layer accepts analog phase only when DFT and zero-crossing estimates agree; amplitude control and AD9959 code are untouched.

**Tech Stack:** STM32F407 HAL, TIM8 TRGO, ADC1/ADC2 dual regular simultaneous mode, DMA2, C99 math, Keil MDK-ARM.

---

### Task 1: Frequency-proportional PA1/PA2 sampling

**Files:**
- Modify: `Drivers/BSP/MEASUREMENT/wired_measurement.c`
- Modify: `Drivers/BSP/PHASE/phase_adc.c`
- Modify: `Drivers/BSP/PHASE/phase_adc.h`

- [x] Replace `WiredMeasurement_SelectSampleRate()` with the approved 1024/256/64/16/8/4 multiplier table and clamp the result to the phase ADC hardware limits.
- [x] Keep requirement 2/4 selection based on `fin` and requirement 3 selection based on `2*fin`.
- [x] Select ADC sampling time from the requested trigger rate: longest HAL sampling-time option whose total conversion time remains shorter than one trigger period with margin.
- [x] Verify boundary calculations for 1 kHz, 10 kHz, 100 kHz, and 200 kHz analysis frequencies.

### Task 2: Independent phase estimate and quality gate

**Files:**
- Modify: `Drivers/BSP/PHASE/phase_fft.c`
- Modify: `Drivers/BSP/PHASE/phase_fft.h`
- Modify: `Drivers/BSP/MEASUREMENT/wired_measurement.c`
- Modify: `Drivers/BSP/MEASUREMENT/wired_measurement.h`

- [x] Add rising zero-crossing interpolation over the same 1024-point, DC-removed channel arrays.
- [x] Fit each channel's crossing sequence to its configured signal frequency and compute requirement 2 as `phase_y-phase_x`, requirement 3 as `phase_y-2*phase_x`.
- [x] Normalize phase to `(-180,180]` and report the circular difference between DFT and crossing estimates.
- [x] Mark analog phase valid only after several consecutive blocks agree within a bounded tolerance; disagreement remains visible as a diagnostic and cannot calibrate TIM2.

### Task 3: DMA block integrity

**Files:**
- Modify: `Drivers/BSP/PHASE/phase_adc.c`
- Modify: `Drivers/BSP/PHASE/phase_adc.h`

- [x] Claim a ready half-buffer atomically and record its generation before copying.
- [x] Recheck generation after copying; reject a block if DMA rewrote that half during the copy.
- [x] Keep a separate rejected-block counter from the existing pending-block overrun counter.

### Task 4: Diagnostics and control integration

**Files:**
- Modify: `User/f_app.c`
- Modify: `User/user.h`
- Modify: `User/user.c`

- [x] Display actual ADC sample rate, DFT phase, crossing phase, their difference, pending-block overruns, and copy-race rejects on the wired/debug display.
- [x] Feed the control state machine only the quality-gated analog phase validity and value.
- [x] Do not modify amplitude targets, amplitude PI parameters, amplitude measurement conversion, or any file below `Drivers/BSP/AD9959`.

### Task 5: Verification

**Files:**
- Verify: `Projects/MDK-ARM/atk_f407.uvprojx`

- [x] Run static searches proving the 1024-point block size remains unchanged and forbidden subsystems have no new diff.
- [ ] Rebuild the Keil target and require `0 Error(s), 0 Warning(s)`.
- [ ] On hardware, compare DFT phase, crossing phase, and oscilloscope time-domain phase at 1 kHz, 10 kHz, 39 kHz, 100 kHz, and requirement-3 200 kHz analysis.
- [x] Runtime gate requires three consecutive agreeing blocks before automatic capture-target calibration.
