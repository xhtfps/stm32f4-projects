# PA1/PA2 ADC phase correction summary

## Root causes addressed

1. The old ADC acquisition interval was fixed at 3 ADC cycles. Different PA1 and PA2 source impedances could therefore create a frequency-dependent settling error.
2. One DFT result was accepted directly by the phase calibration state machine. There was no independent estimator or consecutive-block quality gate.
3. A DMA half-buffer could be copied while circular DMA started writing it again, producing a mixed block that still looked valid to the DFT.

## Implemented behavior

- The 1024-point block remains unchanged.
- Analysis sample rate uses the approved `1024/256/64/16/8/4` frequency multipliers. Requirements 2 and 4 use `fin`; requirement 3 uses `2*fin`.
- ADC acquisition time is selected at each start from 480, 144, 112, 84, 56, 28, 15, or 3 cycles, using the longest setting that leaves 20 percent conversion margin.
- A rising zero-crossing estimator runs on the same PA1/PA2 block independently from the Hann single-frequency DFT.
- Analog phase becomes valid only after DFT and zero-crossing relations differ by at most 12 degrees for three consecutive blocks.
- Phase validity uses raw ADC signal strength, clipping, and bias checks. It no longer depends on the calibrated Vpp conversion result, so amplitude conversion remains untouched.
- DMA generation and completion sequence are checked around every copy. A raced block is rejected and counted separately.

## Debug page fields

- `ADC FS`: actual TIM8 trigger sample rate.
- `DFT`: DFT phase relation (`Y-X` or `Y-2X`).
- `ZC`: rising-zero-crossing phase relation.
- `D-Z`: zero-crossing relation minus DFT relation. The phase gate requires `abs(D-Z) <= 12 deg` for three blocks.
- `OVR`: pending half-buffer overwritten before processing.
- `REJ`: block rejected because DMA advanced during copying.

## Required hardware verification

Test 1 kHz, 10 kHz, 39 kHz, 100 kHz, and requirement-3 100 kHz input/200 kHz output. At each point compare `DFT`, `ZC`, and oscilloscope time-domain phase. `OVR` and `REJ` should remain stable at zero. A nonzero stable `D-Z` indicates waveform distortion or ADC settling; equal DFT/ZC values that both disagree with the oscilloscope indicate a fixed analog path delay and require a measured per-frequency calibration table rather than another control-loop patch.

The current machine cannot run the Keil rebuild because the recorded toolchain path is `D:\keil5`, while only drive `C:` is mounted. The existing zero-error log predates these edits and is not accepted as verification of this firmware.
