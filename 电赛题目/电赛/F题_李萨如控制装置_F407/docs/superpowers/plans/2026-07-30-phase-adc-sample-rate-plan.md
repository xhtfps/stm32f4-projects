# PA1/PA2 FFT Sample Rate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed power-of-two PA1/PA2 sampling-rate table with the approved frequency-proportional table while keeping every DMA/DFT block at 1024 synchronized sample pairs.

**Architecture:** Keep TIM8, ADC1/ADC2 dual simultaneous mode, DMA buffering, and the single-frequency DFT unchanged. Only `WiredMeasurement_SelectSampleRate()` computes a different requested rate; `PhaseAdc_InitTimer()` still quantizes it to hardware PSC/ARR values and the DFT still consumes the measured actual rate.

**Tech Stack:** STM32F407 HAL, TIM8 TRGO, ADC1/ADC2 dual mode, DMA2, C, Keil MDK-ARM.

---

### Task 1: Replace the sampling-rate selector

**Files:**
- Modify: `Drivers/BSP/MEASUREMENT/wired_measurement.c:21`
- Verify: `Drivers/BSP/PHASE/phase_adc.c`
- Verify: `Drivers/BSP/PHASE/phase_fft.h`

- [ ] **Step 1: Record the expected boundary table**

```text
analysis f =   1,000 Hz -> requested Fs =  16,000 Hz
analysis f =   9,999 Hz -> requested Fs = 159,984 Hz
analysis f =  10,000 Hz -> requested Fs =  80,000 Hz
analysis f =  99,999 Hz -> requested Fs = 799,992 Hz
analysis f = 100,000 Hz -> requested Fs = 400,000 Hz
analysis f = 200,000 Hz -> requested Fs = 800,000 Hz
```

- [ ] **Step 2: Replace only the selector implementation**

```c
/** Select the requested rate from the approved analysis-frequency bands. */
static uint32_t WiredMeasurement_SelectSampleRate(
    uint32_t analysis_frequency_hz)
{
    if (analysis_frequency_hz < 10U)
    {
        return analysis_frequency_hz * 1024U;
    }
    if (analysis_frequency_hz < 100U)
    {
        return analysis_frequency_hz * 256U;
    }
    if (analysis_frequency_hz < 1000U)
    {
        return analysis_frequency_hz * 64U;
    }
    if (analysis_frequency_hz < 10000U)
    {
        return analysis_frequency_hz * 16U;
    }
    if (analysis_frequency_hz < 100000U)
    {
        return analysis_frequency_hz * 8U;
    }
    return analysis_frequency_hz * 4U;
}
```

- [ ] **Step 3: Verify scope and invariants**

Run:

```powershell
git diff -- Drivers/BSP/MEASUREMENT/wired_measurement.c
rg -n "PHASE_ADC_BLOCK_COUNT|PHASE_FFT_MAX_POINTS|WiredMeasurement_SelectSampleRate" Drivers/BSP/MEASUREMENT Drivers/BSP/PHASE
```

Expected: `PHASE_ADC_BLOCK_COUNT` and `PHASE_FFT_MAX_POINTS` remain 1024; no amplitude, phase-control, or AD9959 source is changed.

- [ ] **Step 4: Rebuild the existing Keil project**

Build `Projects/MDK-ARM/atk_f407.uvprojx` with the installed Keil command-line executable and capture its log under `Output`.

Expected:

```text
0 Error(s)
```

- [ ] **Step 5: Do not commit**

The workspace contains pre-existing user changes and the `新代码` tree is untracked at the repository root. Leave the verified source edit unstaged and uncommitted.
