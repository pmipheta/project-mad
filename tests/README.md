# Oscilloscope regression / build

Run from the project directory:

```
python3 tests/test_osc.py
python3 tests/test_dma.py
python3 tests/test_slow.py
python3 tests/test_autotb.py
python3 tests/build_arm.py
```

Host tests extract actual production C functions; only hardware input/HAL/cache operations are modeled. They are not board verification. Outputs go to `~/.hermes/cache/scratch/osc_regression` and `~/.hermes/cache/scratch/osc_arm_build`, never CubeIDE's existing Debug directory. ARM helper requires the CubeIDE toolchain path defined in the script.

## Measurement contract

- Periodic, adequately clean input: **40 Hz–100 kHz**, at least **50 mVpp**, input kept within **0–3.3 V**. Accuracy depends on HSE accuracy and the analog front end; use a low-impedance source and appropriate external anti-alias filtering. Arbitrary multi-tone/noisy or above-Nyquist inputs are not guaranteed.
- Frequency uses a separate contiguous fast ADC capture, not the 240 displayed points or UI smoothing. At 8 MHz HSE, ADC clock is 12 MHz, rate 800 kSa/s (3+12 cycles). Initial measurement is 3200 samples / 4 ms; if insufficient complete periods, grow on subsequent frames, at most 65520 samples / 81.9 ms. A valid estimate sets the next duration to about 3.2 periods (minimum 4 ms). Following abrupt frequency changes, allow up to seven successful frames to settle. Unknown/no-signal frequency is 0, not fabricated.
- Display captures remain contiguous. 6 ms/div selects 15-cycle sampling / 27 conversion cycles, approximately 444.444 kSa/s, retaining the requested time span while staying below 16-bit NDTR. No stitched samples or silent timebase truncation.
- Display Vmax/Vmin/Vpp/RMS and cursor voltages intentionally describe the calibrated, smoothed plotted signal; frequency and AutoScale raw Vpp are independent. Display boxcar/filtering can suppress high frequencies at slow T/div; that is not measurement-frequency aliasing.
- HOLD preserves captured rate/timebase; settings apply to future captures. AutoScale is ignored in HOLD. The AUTO button runs one state machine, using a fresh valid raw measurement only.
- Failed acquisition clears validity/measurements, does not advance AutoScale or draw stale data as new, displays `ADC ERR`, and retries on the next frame. Last trace remains on screen with the error status. DMA start/timeout/error/stop paths are covered by fault injection.
- DWT diagnostic elapsed time subtracts unsigned raw cycles before scaling. Cache ranges are 32-byte rounded within a 32-byte-aligned, whole-cache-line buffer, cleaned/invalidated before DMA and invalidated after stop.

## Recorded RED examples before fixes

```
frequency 100 -> 0.00
frequency 20000 -> 4000.00
DMA request 72202 limit 65535
captured rate retained: 4000.0  (expected 12345)
roll start 200                 (expected 0)
calibrated DC pixel 90         (expected 120)
DMA fault/cache/wrap failures=5
```

State-contract tests also failed for separate button AutoScale, decimated frequency selection, stale-capture processing, invalid-capture advancement, and missing explicit ADC-error display. Green tests include a 210-case frequency/timebase/filter sweep, raw square/DC/no-signal checks, display/cursor mapping, HOLD/AutoScale/recovery, and injected HAL/cache/DWT conditions. Real source compiled and linked with ARM GCC; no flash or commit is performed.

## SLOW mode (T/DIV 50-500 ms, signals ~0.2-60 Hz)

`test_slow.py` extracts the SLOW sampler from `main.c` (SLOW_BEGIN..SLOW_END) and runs it on a PC against a modelled ADC: frequency within 3 %, honest 0 Hz when fewer than 2 periods or below 50 mVpp, clipping visible, FAST<->SLOW switching. Not board verification. FAST measurement (40 Hz-100 kHz) is unchanged; below 40 Hz the frequency readout comes from the SLOW window.

## AUTO time base

`test_autotb.py` runs the FAST/SLOW switching logic (extracted from `main.c`, AUTO_TB_BEGIN..END) in a closed loop against a modelled measurement chain: 0.5 Hz..100 kHz settle on the right mode, 1 Hz<->1 kHz transitions, no switching without signal, bounded switching on pure noise, manual T/DIV and HOLD disable it. Manual T/DIV change pauses AUTO; `z` (or long SET) re-arms it. The readout shows `---` when the frequency is unknown. Not board verification.
