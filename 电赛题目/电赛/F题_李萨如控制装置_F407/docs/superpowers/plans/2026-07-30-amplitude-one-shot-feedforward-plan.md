# Amplitude One-Shot Feedforward Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove quarter-division amplitude steps caused by repeated runtime feedforward recalculation.

**Architecture:** Preserve the existing amplitude measurement, filter, PI, clipping protection, and DDS interface. Add one guard in the feedforward update path so a successfully established feedforward remains fixed until the controller is restarted.

**Tech Stack:** STM32F407 HAL, C99, Keil MDK-ARM.

---

### Task 1: Make amplitude feedforward one-shot

**Files:**
- Modify: `Drivers/BSP/CONTROL/wired_control.c`

- [x] Add an early return after target validation when `amplitude_feedforward_ready` is already true.
- [x] Clear any pending refeed candidate before returning.
- [x] Keep the initial five-block feedforward establishment path unchanged.

### Task 2: Verify scope and behavior

**Files:**
- Verify: `Drivers/BSP/CONTROL/wired_control.c`
- Verify: `Drivers/BSP/AD9959/Drive_AD9959.c`

- [x] Confirm amplitude PI still runs for filtered errors greater than 30 mV.
- [x] Confirm only `wired_control.c` behavior changed and AD9959 code was not edited.
- [ ] Rebuild with Keil if the recorded `D:\keil5` toolchain is available.
- [ ] On hardware, hold a fixed input and verify that the output no longer moves in quarter-division steps.
