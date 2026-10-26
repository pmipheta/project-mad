# Developer Guide — STM32F767 Oscilloscope

This guide is for someone who will **continue developing** this firmware. It explains
what every file does, which function does what, how data moves from the input pin to
the LCD, and the rules you must keep so nothing breaks.

If you only have five minutes, read [§1 Quick start](#1-quick-start),
[§3 How a frame is made](#3-how-a-frame-is-made) and [§9 Rules and pitfalls](#9-rules-and-pitfalls).

---

## Contents

1. [Quick start](#1-quick-start)
2. [The hardware and who owns which peripheral](#2-the-hardware-and-who-owns-which-peripheral)
3. [How a frame is made](#3-how-a-frame-is-made)
4. [Repository map](#4-repository-map)
5. [Module reference (file by file)](#5-module-reference-file-by-file)
6. [Shared state reference](#6-shared-state-reference)
7. [User controls: buttons](#7-user-controls-buttons)
8. [How to… (common changes)](#8-how-to-common-changes)
9. [Rules and pitfalls](#9-rules-and-pitfalls)
10. [Tests](#10-tests)
11. [Debugging on the board](#11-debugging-on-the-board)
12. [Known limitations and open items](#12-known-limitations-and-open-items)
13. [Other documents](#13-other-documents)

---

## 1. Quick start

| What | How |
|---|---|
| Open | STM32CubeIDE → *File → Open Projects from File System* → this folder (it is the project root) |
| Build | *Project → Build* (configuration **Debug**). Expected result: 0 errors, 1 warning (`DrawLineBresenham` defined but not used) |
| Flash | *Run → Debug* in CubeIDE, or `STM32_Programmer_CLI -c port=SWD -w Debug/oscilloscope.elf -v -rst` |
| Host tests (no board) | `python3 tests/run_all.py` (needs Python 3 and `cc`/`gcc`/`clang`) |
| Input signal | **PA0** (Arduino A0), **0–3.3 V only** at the pin |

Expected test result today: 12 of 13 suites pass. `test_osc.py` reports `behavior failures=1`.
That failure is old and known (it was there before the RTOS split). Any **new** failure is yours.

---

## 2. The hardware and who owns which peripheral

| Part | Detail |
|---|---|
| Board | NUCLEO-F767ZI, Cortex-M7. SYSCLK **96 MHz** (HSE 8 MHz, PLL M=8 N=192 P=2). APB2 = 48 MHz, APB1 = 24 MHz |
| Display | ILI9341 320×240 on **SPI5** (SCK PF7, MISO PF8, MOSI PF9), CS = PC8, DC = PC9, RST = PC10. Landscape |
| Input | PA0 = ADC1/2/3 channel 0. The analog front end divides the input: `ADC_INPUT_GAIN = 1.25` input volts per pin volt |
| Buttons | PB0, PB1, PB2, PB3, PA3, PC0, PC3, PF3 (active LOW, internal pull-up) + PC13 blue user button (active HIGH) |

### Peripheral ownership

| Peripheral | Configured by | Used by | Notes |
|---|---|---|---|
| ADC1 + DMA2 Stream0 | CubeMX (`adc.c`, `dma.c`) | `adc_capture.c` (FAST burst capture) | Re-configured at run time by `AdcSetRateLevel()` (prescaler, sample time) |
| ADC2 | `AdcTripleInit()` (by hand) | 4.8 MSa/s triple-interleave mode | Copies `hadc1.Init` |
| ADC3 | `AdcTripleInit()` + `SlowAdcPrepare()` | **two users**: the 1 kHz SLOW sampler in SysTick *and* triple-interleave mode | Shared with the `slow_busy` / `slow_adc3_dirty` flags. See §9 |
| SysTick (1 kHz) | HAL + FreeRTOS | `SysTick_Handler` in `stm32f7xx_it.c` | Calls `HAL_IncTick()`, `OscSlowTick()`, then `xPortSysTickHandler()` once the scheduler runs |
| SPI5 | CubeMX | ILI9341 driver + `display.c` column renderer | Blocking `HAL_SPI_Transmit` |
| DWT cycle counter | `AdcInputInit()` | timing (`AdcElapsedUs`, `prof_us[]`) | Diagnostics only |
| Button GPIOs | `AppInit()` (by hand, **not** in the `.ioc`) | `buttons.c` | If you add them in CubeMX later, remove the manual init |

> **CubeMX safety.** You can open `oscilloscope.ioc` and press *Generate Code*: on the
> committed tree it changes no file. Hand-written code is only in `Core/Src/app`,
> `Core/Inc/app`, `ThirdParty/`, `FreeRTOSConfig.h`, `freertos_hooks.c` and inside
> `/* USER CODE BEGIN … */` blocks. **Do not enable FreeRTOS in the CubeMX middleware
> list.** The kernel lives in `ThirdParty/FreeRTOS`. If you enable it, CubeMX generates a second
> copy plus its own SVC/PendSV handlers and the build breaks.

---

## 3. How a frame is made

### 3.1 Start-up

```
main()                                   Core/Src/main.c (CubeMX)
 ├─ SCB_EnableICache/DCache, HAL_Init, SystemClock_Config
 ├─ MX_GPIO / DMA / RNG / SPI5 / TIM1 / I2C1 / ADC1 _Init
 ├─ AppInit()                            app/app_main.c
 │    ├─ AdcInputInit()                  DWT on, measure real ADC rate, init ADC2+ADC3
 │    ├─ button GPIO init + Button structs
 │    ├─ ILI9341_Init, rotation, clear screen
 │    └─ (SHOW_FPS) one-time SPI/pixel benchmark, DrawGrid
 └─ AppStart()                           app/app_main.c — never returns
      ├─ xOscMutex, xAppLock = xSemaphoreCreateMutex()
      ├─ xTaskCreate(OscTask,   prio idle+1, 2048 words)
      ├─ xTaskCreate(InputTask, prio idle+2, 2048 words)
      └─ vTaskStartScheduler()
```

### 3.2 Tasks and interrupts at run time

```
                ┌──────────────────────── InputTask (input_task.c) ─────────────────┐
  every 1 ms →  │ APP_LOCK; HandleButtons(now); APP_UNLOCK;                           │
                │ xTaskNotifyGive(OscTask)                                            │
                └──────────────────────────────────────────────────────────────────────┘
                ┌──────────────────────── OscTask (osc_task.c) ─────────────────────┐
                │ APP_LOCK                                                            │
                │  • abort a stale pre-started capture                                │
                │  • C1/C2 moved while frozen → CursorsMoveFast() (fast repaint)      │
                │  • every FRAME_MS (33 ms):  capture → process → measure → auto →    │
                │                             hold check → draw  (see 3.3)            │
                │  • every HEADER_MS (300 ms): DrawTopPanel(), FPS, stack marks       │
                │ APP_UNLOCK                                                          │
                │ sleep (ulTaskNotifyTake) until the next frame/header is due,        │
                │ or until InputTask has polled                                       │
                └──────────────────────────────────────────────────────────────────────┘
  ISRs (never take a mutex):
    SysTick 1 kHz ............ HAL_IncTick, OscSlowTick (ADC3 → slow history), FreeRTOS tick
    DMA2_Stream0 / ADC ....... HAL_ADC_ConvCpltCallback / HAL_ADC_ErrorCallback → adc_done / adc_error
```

The app lock makes the two tasks take turns. A button press is always
applied **between** two OscTask passes, never in the middle of a frame. That is how the
old single super-loop behaved, so behaviour is identical. The gain is that OscTask now
sleeps instead of spinning: it is ~98 % idle while frozen. When running, the frame rate is
limited by the SPI transfer to the LCD (~25 frames/s).

### 3.3 One frame, step by step (inside `OscTask`)

| # | Step | Function (file) | Input → Output |
|---|---|---|---|
| 1 | Choose mode | `SlowSync()` (`slow_mode.c`) | `osc_tdiv_i` → `slow_active`, `slow_period_us` |
| 2a | FAST capture (T/DIV 5 µs … 6 ms) | `AdcCaptureFrame()` (`adc_capture.c`) | ADC1 DMA burst in `adc_raw_buf[]` → trigger search → `raw_samples[240]`, `osc_raw_*`, `osc_fs_eff` |
| 2b | SLOW capture (T/DIV 10 … 500 ms) | `SlowCaptureFrame()` (`slow_mode.c`) | 1 ms history `slow_hist[]` → trigger search → `raw_samples[240]`, `osc_raw_*` |
| 3 | Calibrate + filter | `OscProcessFrame()` (`osc_process.c`) | `raw_samples` (ADC counts) → `osc_v[240]` (input volts) |
| 4 | Measure | `OscMeasureFrame()` (`osc_process.c`) | → `osc_meas` (Vmax, Vmin, Vpp, RMS, F, DC), `osc_det` (wave type) |
| 5 | One-shot AUTO scale | `OscAutoScaleStep()` (`osc_actions.c`) | only while `osc_auto_stage` → V/DIV, T/DIV |
| 6 | Auto time base | `OscAutoTimebase()` (`osc_timebase.c`) | only while `osc_auto_tb` → may change `osc_tdiv_i` (FAST↔SLOW) |
| 7 | Pre-start next capture | `AdcPrestart()` (`adc_capture.c`) | FAST only: starts the next DMA capture so it runs while we draw |
| 8 | Hold check | `OscFrameHold()` (`osc_timebase.c`) | 1 = do not draw this frame (stale or known-bad) |
| 9 | Map to pixels | `MapToScreen()` (`display.c`) | `osc_v` → `new_y[240]`, peak band `env_y_*` |
| 10 | Draw | `DrawWaveform()`, `DrawCursors()`, `DrawSettingBar()` (`display.c`) | changed screen columns only, one SPI burst each |

**Units, so you never mix them up:**

- `adc_raw_buf[]`, `raw_samples[]`, `osc_raw_*_ct`, `osc_env_*_ct`, `slow_hist[]`: ADC counts, 0…4095.
- `osc_v[]`, `osc_meas.*`: **input volts**. That is pin volts × `ADC_INPUT_GAIN` after offset removal.
- `osc_raw_vpp`: **pin volts** (no gain). The signal-detection thresholds 0.05 V / 0.30 V are pin volts on purpose.
- `new_y[]`, `env_y_*[]`: LCD rows. Point `i` is drawn at `x = i·320/240`.

---

## 4. Repository map

| Path | What it is | Edit? |
|---|---|---|
| `Core/Src/app/*.c`, `Core/Inc/app/*.h` | **All application code** (13 modules, §5) | Yes — this is where you work |
| `Core/Inc/app/app_config.h` | Build switches, pins, screen geometry, timing constants, shared types | Yes |
| `Core/Inc/app/app.h` | Includes every module header. Each module `.c` includes only `app/app.h` | When adding a module |
| `Core/Src/main.c` | CubeMX. User code: `#include "app/app.h"` + `AppInit(); AppStart();` | Only inside USER CODE |
| `Core/Src/adc.c`, `dma.c`, `gpio.c`, `spi.c`, `tim.c`, `i2c.c`, `rng.c`, `stm32f7xx_hal_msp.c` | CubeMX peripheral init | Via the `.ioc` only |
| `Core/Src/stm32f7xx_it.c` | Interrupt handlers (CubeMX). USER CODE in SysTick calls `OscSlowTick()` and the FreeRTOS tick | Only inside USER CODE |
| `Core/Src/freertos_hooks.c` | `vApplicationStackOverflowHook` (halts with interrupts off) | Rarely |
| `Core/Inc/FreeRTOSConfig.h` | Kernel config: 1 kHz tick, 24 KB heap (`heap_4`), mutexes, stack-overflow check 2, max syscall IRQ priority 5 | Yes, carefully |
| `Core/Src/ILI9341/` | Third-party LCD driver (martnak/STM32-ILI9341): pixels, rectangles, 5×5 font text | Rarely |
| `ThirdParty/FreeRTOS/` | FreeRTOS V11.1.0 kernel, vendored (port `GCC/ARM_CM7/r0p1`, `heap_4.c`) | No |
| `Drivers/` | ST HAL + CMSIS | No |
| `oscilloscope.ioc` | CubeMX project | Through CubeMX |
| `tests/` | Host tests that compile the real C from the modules and run it on a PC (§10) | Yes |

---

## 5. Module reference (file by file)

Each module is one `.c` in `Core/Src/app/` with a header of the same name in `Core/Inc/app/`.
Functions marked *static* are private to their file.

### 5.1 `app_main` — start-up

| Function | What it does |
|---|---|
| `AppInit()` | Everything after the CubeMX init: `AdcInputInit()`, the 8 button GPIOs (input + pull-up), the `Button` structs, LCD init + clear, `HdrInvalidate()`, optional SPI benchmark (`SHOW_FPS`), `old_y[]` = centre line |
| `AppStart()` | Creates `xOscMutex` and `xAppLock`, creates OscTask and InputTask, starts the scheduler. Never returns. Any failure → `Error_Handler()` |

Header `app_main.h` defines the task stacks and priorities, `INPUT_POLL_MS`, and `APP_LOCK()` / `APP_UNLOCK()`.

### 5.2 `osc_task` — the oscilloscope task

| Function | What it does |
|---|---|
| `OscTask()` | Start-up: `DrawGrid()`, `DrawCursor()`, `DrawTopPanel()`, then notifies InputTask. Loop: the frame pipeline of §3.3, the header every 300 ms, the FPS readout, stack high-water marks (`dbg_stack_free[]`), then sleeps until the next deadline |

Things to know:

- `should_draw` depends on `trigger_mode` (always `TRIG_AUTO` today; SINGLE/NORM are left over from the original mock-up) and on `hold_active`.
- `osc_dirty = 1` forces one redraw even while frozen. Every setting change sets it.

### 5.3 `input_task` — buttons

| Function | What it does |
|---|---|
| `InputTask()` | Waits for OscTask's start-up notify, then every 1 ms: `APP_LOCK; HandleButtons(HAL_GetTick()); APP_UNLOCK;` and wakes OscTask |

### 5.4 `adc_capture` — FAST mode acquisition (T/DIV 5 µs … 6 ms)

How FAST capture works:

1. One contiguous DMA burst of `n` samples (≤ 65 520) goes into `adc_raw_buf[]`.
2. A Schmitt trigger finds the edge.
3. The burst is resampled to 240 display points with a boxcar average.

**Rate levels** (`adc_rate_tab[]`, chosen by `AdcPlanFrame()`, applied by `AdcSetRateLevel()`):

| Level | Rate | How |
|---|---|---|
| L0 | 1.6 MSa/s | ADC 24 MHz, 3 + 12 cycles |
| L1 | 800 kSa/s | ADC 12 MHz, 3 + 12 cycles (the default) |
| L2 | 444 kSa/s | 15-cycle sampling (slow FAST bases, so the capture fits 65 520 samples) |
| L3 | 300 kSa/s | 28-cycle sampling |
| L4 | 4.8 MSa/s | ADC1+ADC2+ADC3 triple interleave, 5-cycle delay, DMA mode 2 |

| Function | What it does |
|---|---|
| `AdcInputInit()` | Enables the DWT, measures the real ADC rate with a 2000-sample capture (`dbg_adc_rate`), runs `AdcTripleInit()` |
| `HAL_ADC_ConvCpltCallback()` / `HAL_ADC_ErrorCallback()` | ISR callbacks: set `adc_done` / `adc_error`. A tail overrun after a completed DMA is not an error |
| `AdcTripleInit()` *static* | Sets up ADC2 and ADC3 as copies of ADC1 on channel 0 |
| `AdcCaptureTriple()` *static* | One 4.8 MSa/s interleaved capture (32-bit DMA), then restores single-ADC mode and marks ADC3 dirty |
| `AdcCaptureRawInner()` / `AdcCaptureRaw()` *static* | Blocking single-ADC capture of `n` samples with D-cache clean/invalidate. Sets `slow_busy` so SysTick leaves the ADCs alone |
| `AdcCaptureStart()` / `AdcCaptureCollect()` *static* | The same, split in two: start the DMA, collect later (pipelining) |
| `AdcRateHz()`, `AdcSetRateLevel()` *static* | Rate of a level; program the common prescaler and the sample time (turns ADC3 off while doing it) |
| `AdcRawFrequency()` *static, -O2* | Frequency from raw samples: Schmitt edges with 20 % hysteresis, rejects irregular spacing and too-fast/aliased content. On success it also fills the whole-cycle statistics `osc_raw_mean/min/max/rms/dcrms_ct` and `osc_raw_valid = 1`. Sets `adc_too_fast` and `adc_fast_content` |
| `AdcPlanFrame()` *static* | Pure planning: which level and step for the current T/DIV, and whether the frequency can be read from the display capture itself (`fast_meas`, saves a second capture) |
| `AdcPreFor()` *static* | How many display points to capture **before** the trigger search window (1.3 periods + margin, max `ADC_PRE_POINTS` = 200) |
| `AdcPrestart()` | Starts the next frame's capture asynchronously before drawing (FAST, single ADC, `fast_meas` path only) |
| `AdcAbortPending()` | Cancels a pre-started capture (time base changed, frozen, or nobody collected it within 400 ms) |
| `AdcCaptureFrame()` *-O2* | **The FAST frame.** Steps below |

`AdcCaptureFrame()` does five steps in order:

1. **Measurement.** If `fast_meas` is not possible, it makes a separate measurement capture and reads its frequency. This step includes the rate hunting, a 1 s alias probe at 4.8 MSa/s, and a 1.5 s hold of the last good reading.
2. **Display capture.** It uses the pre-started capture if one matches, otherwise it captures now.
3. **Trigger.** It finds the mid-level crossing with a 15 % Schmitt trigger, refines it on a boxcar-smoothed copy, then backs off by `lead` ≈ 2 display points. The edge lands at screen point ~2.
4. **Resample** into `raw_samples[240]`. When there are fewer than 8 points per period it also fills the **peak-detect band** (`osc_env_lo_ct/hi_ct`, `osc_env_on`).
5. **Keep the outside points.** `MvStoreExt()` stores the points just outside the window for the edge filter. Finally it sets `osc_fs_eff`, `osc_capture_tdiv`, `osc_frame_valid`, `new_frame_ready`.

### 5.5 `slow_mode` — SLOW mode (T/DIV 10 … 500 ms)

A 1 kHz sampler runs **all the time**, also in FAST mode. On every SysTick it reads a burst of 16 conversions on ADC3. The burst mean goes into `slow_hist[]` and the burst spread into `slow_hist_rg[]`. The ring holds 8192 ticks (8.2 s), so switching to SLOW shows a full picture at once.

| Function | What it does |
|---|---|
| `OscSlowTick()` | **Runs in the SysTick ISR.** One 16-conversion burst; missed ticks (while a burst capture owns the ADCs) are filled by linear interpolation. Writes `slow_last_t` last |
| `SlowAdcPrepare()` / `SlowAdcRead()` *static* | Configure ADC3 for single software-started conversions; one conversion with a short guard timeout |
| `SlowSync()` | T/DIV ≥ 10 ms → `slow_active = 1` and `slow_period_us` = time per display point |
| `SlowBuildPointsAt()` *static* | History → 240 display points ending at tick `t_view` (interpolation below 1 ms per point, averaging above). Also returns the alias-guard spread/roughness of the newest 100 ms |
| `SlowFindTrigger()` *static* | Finds the newest clean rising/falling edge in the last 1.5 windows (20 % hysteresis) and returns the view end so the edge sits **2 display points from the left** (v96, same as FAST) |
| `SlowFreqFromPts()` *static* | Frequency of a window from regularly spaced mid-level edges; with `store = 1` it also fills the `osc_raw_*` statistics. Never invents a number |
| `SlowLooksFast()` *static* | Alias guard: burst spread or ms-to-ms jumps that a slow signal cannot make |
| `SlowEnvAt()` *static* | Peak-detect band for SLOW when there are fewer than 12 points per period |
| `SlowStoreExt()`, `SlowExtHi()`, `SlowExtPoint()` | The real neighbours outside the SLOW window, for the edge filter (left: real history; right: one period back) |
| `SlowRecentFlat()` / `SlowSlowish()` / `SlowProbeFreq()` | History probes used by the auto time base: "signal just stopped", "a slow signal is moving", "what would time base *ti* read?" |
| `SlowCaptureFrame()` | **The SLOW frame.** Steps below |

`SlowCaptureFrame()` works like this:

1. It triggers the picture with `SlowFindTrigger()`, or free-runs (newest data at the right) when there is no edge.
2. It eases the drawn time base 15 % per frame after a T/DIV change.
3. It measures. **The readout is measured on a window 22 points earlier than the picture.** The first edge needs the slope before it to arm the hysteresis, otherwise a 3-cycle window loses an edge.
4. It sets `slow_trig_on`, which drives the header "AUTO:R/F" vs "AUTO".

`SlowCaptureFrame()` writes shared state, so OscTask calls it inside `OSC_LOCK()`.

### 5.6 `osc_process` — calibration, filter, measurements

| Function | What it does |
|---|---|
| `OscFiltWin()` | The display filter window actually used: `osc_fwin` (odd 1…21), narrowed until the boxcar keeps ≥ 98 % of the amplitude at the measured frequency (so the trace matches V/DIV) |
| `OscProcessFrame()` | Offset: AC mode → mean over whole cycles (`osc_raw_mean_ct`), else mid-range; DC mode → `ADC_DC_ZERO_CT`. Moving average → `osc_v[]` in input volts. Edge points use the real outside neighbours (`OscExtPoint`/`SlowExtPoint`) so the screen edges never spike; fallback rule when there are none. DC mode clips below 0 V |
| `OscMeasureFrame()` | Vmax/Vmin/RMS/Vpp from the **unfiltered** data (whole-cycle stats when valid, else the peak band, else the window). Frequency = `osc_raw_freq`. Applies the input-bandwidth correction above ~100 kHz (`ADC_INPUT_FC`, `ADC_INPUT_N`, capped at `ADC_INPUT_GMAX`). Crest factor → `osc_det` (SQR/SIN/TRI/SAW). Writes `osc_meas` under `OSC_LOCK` |

### 5.7 `osc_timebase` — frame hold and the automatic time base

| Function | What it does |
|---|---|
| `OscFrameHold(now)` | Returns 1 = do not draw. Three cases: captured under another T/DIV; FAST has a real signal it cannot read (keep the picture ≤ 0.5 s); amplitude collapsed after a good picture (keep ≤ 2 s, the generator blanks while you edit a digit) |
| `OscAutoTimebase()` | Every frame. Tracks `osc_lock_auto` (≥ 45 Hz lock, < 35 Hz release). If `osc_auto_tb` is on, it moves FAST→SLOW below **70 Hz** and SLOW→FAST above **90 Hz**. In FAST it picks a T/DIV showing ~6 cycles. When FAST can't read the signal, it probes the SLOW history every 250 ms. Every switch is logged (`dbg_atb_log[]`, reason codes in `dbg_atb_why`) |
| `AtbPickSlow(f)` | SLOW T/DIV for frequency f: < 0.9 Hz → 500 ms, < 12 Hz → 200 ms, < 45 Hz → 50 ms, else 10 ms/div |
| `AtbPickFast()`, `AtbSwitch()`, `AtbProbeSlow()`, `AtbFastFallback()` *static* | Helpers: nearest FAST base to ~6 cycles; perform and log a switch; probe SLOW bases in order 100/50/200/500/20/10 ms |
| `OscManualToggle()` | FAST↔SLOW by hand (PB3/PC13 short), turns `osc_auto_tb` off |
| `OscAutoRearm()` | Automatic FAST/SLOW again (PB3/PC13 long) |

`osc_timebase.h` exports `ATB_FAST_TO_SLOW_HZ` (70) and `ATB_SLOW_TO_FAST_HZ` (90).

### 5.8 `osc_settings` — the setting menu

The menu has two name lists in `osc_state.c`:

- **Running:** `osc_set_names` = FILTER, TRIG, V/DIV, T/DIV.
- **Frozen:** `osc_hold_names` = C1, C2, V/DIV.

`osc_sidx` is the selected entry.

| Function | What it does |
|---|---|
| `OscSetCount()`, `OscSetName()` | Number of entries / name of the selected entry for the current state (running or frozen) |
| `OscSetIndex(i)` | Select an entry (wraps) |
| `OscAdjust(dir)` | +1/−1 on the selected entry. FILTER ±2 (odd 1…21), TRIG toggles RISE/FALL, V/DIV and T/DIV step through their tables. **T/DIV stays inside the current mode's zone** (FAST 0…11, SLOW 12…17), and a manual T/DIV turns `osc_auto_tb` off and cancels a pending AUTO. C1/C2 move 1 px. Sets `osc_dirty` (or `osc_cur_dirty` for cursors) |
| `OscSetValueText()` | Text for the header field, e.g. `FILT:5`, `TRIG:RISE`, `V:1.0000` |
| `OscCursorSelected()` | 1 when frozen and C1/C2 is selected (buttons repeat faster) |
| `OscAutoScale()` | Starts the one-shot AUTO state machine (`osc_auto_stage = 1`). Ignored while frozen |
| `OscClampI()`, `OscClampF()` | Clamp helpers used everywhere |

### 5.9 `osc_actions` — every user action (`Act*`)

`HandleButtons()` calls these functions, one per button role.

| Function | Action |
|---|---|
| `ActFreeze()` | Freeze on/off (`hold_active`), selects the first menu entry |
| `ActAuto()` | Unfreeze, start one-shot AUTO scale |
| `ActToggleCoupling()` | AC ↔ DC (`osc_calib`) |
| `ActToggleStable()` | Trigger lock on/off (`osc_stable`; header AUTO vs ROLL) |
| `ActIncrease(n)` / `ActDecrease(n)` | +/− : `OscAdjust` on the selected entry, running or frozen (n steps for C1/C2) |
| `ActSelectSetting(name)` | Select a menu entry by name (no-op if it is not in the current list) |
| `ActToggleTrigEdge()` | RISE ↔ FALL directly (not wired to a button today) |
| `ActToggleFastSlow()`, `ActAutoMode()` | → `OscManualToggle()`, `OscAutoRearm()` |
| `OscAutoScaleStep()` | One-shot AUTO, per frame: V/DIV so Vpp fills ≤ 4 divisions (DC: peak ≤ 2 div), T/DIV by the same rules as the auto time base (≥ 2 cycles in FAST). Falls back to the 200 ms/div SLOW window when FAST reads nothing |
| `OscCursorV(x)`, `OscCursorDtS()` | Voltage under cursor pixel `x` (interpolated between the two neighbouring points); C2−C1 time in seconds |
| `MvFmtT()`, `MvFmtV()` | Header text of T/DIV and V/DIV (4 decimals) |

### 5.10 `ext_points` — the display points just outside the FAST window

| Function | What it does |
|---|---|
| `MvStoreExt()` *-O2* | Called by every FAST capture: up to 256 display points before and after the window (`mv_ext_lo/hi`), averaged like the window. Needed because the next capture overwrites `adc_raw_buf` |
| `OscExtPoint(k, &v)` | Display point `k` outside 0…239 for the edge filter (FAST: `mv_ext_*`; SLOW: `SlowExtPoint`) |

### 5.11 `display` — everything drawn on the LCD

| Function | What it does |
|---|---|
| `MapToScreen(off)` | `osc_v` → `new_y[]` rows with V/DIV. DC mode: shifts the trace so its middle sits on the centre line (display only, the numbers keep real volts). Also maps the peak band → `env_y_lo/hi[]` |
| `DrawWaveform()` | **Column renderer.** For each of the 320 columns, computes the y-run of the polyline (`TraceColumns`). It repaints only columns whose run changed, top to bottom in one SPI burst (`LcdWriteColumn`), with the background from `BackgroundColour()` (cursor > grid dot > black). No stale pixels, ~10× faster than per-pixel drawing |
| `DrawGrid()` | The dotted grid: rows every 40 px (1 V/DIV), columns every 32 px (1 T/DIV). The rule is the single macro `IS_GRID_DOT(x, y)` in `app_config.h` |
| `DrawCursors()` / `ClearCursors()` | Yellow C1/C2 lines while frozen; erase them (grid restored) |
| `CursorsMoveFast()` | Moves C1/C2 by repainting only the old and new cursor columns (~1 ms instead of a ~98 ms full redraw), then the changed characters of the setting bar |
| `DrawCursor()` | The fixed dark-cyan centre marker line (`cursor_x`) |
| `DrawSettingBar()` | Bottom bar: "SET VDIV=1.0V UP/DOWN STEP" etc. Drawn after the waveform |
| `DrawTopPanel()` | The 3-row header. Running: V/DIV, T/DIV, selected setting (blinks), RUN/HOLD/ADC ERR, AC/DC; Vp, F, Rs; Mx, Mn, trigger state (AUTO:R/F, ROLL), AUT/FAST*/SLOW*, CLIP. Frozen: U1, U2, dV, dt, FAST/SLOW |
| `HdrText()`, `HdrInvalidate()` *(HdrText static)* | Header text cache: only characters that changed are redrawn, so numbers don't blink. Every call site gets its own cache slot through `__COUNTER__` (macro `HDRT`) |
| `DrawFps()` | FPS at the bottom right (`SHOW_FPS`) |

### 5.12 `buttons` — reading the front panel

| Function | What it does |
|---|---|
| `HandleButtons(now)` | Called by InputTask every 1 ms. Reads every button and calls the `Act*` function for it (table in §7) |
| `ButtonJustPressed()` *static* | Edge with 200 ms debounce (PA3, PC0, PF3) |
| `ButtonRepeat()` *static* | Hold-repeat for +/−: 1 step on press, repeat every 60 ms after 300 ms, 3 steps per repeat after 1.5 s; 20 ms repeat when moving C1/C2. Release must be stable 30 ms |
| `ButtonRaw()` *static* | Pin state, active LOW |
| short/long pattern (inline) | PB2, PB3, PC3, PC13: short = act on release (≥ 40 ms), long = act once when the hold time is reached |

### 5.13 `osc_state` — shared variables

`osc_state.c` defines the shared variables. It also defines the tables: `osc_vdivs[14]` (0.0001…2 V), `osc_tdivs_ms[18]` (12 FAST + 6 SLOW) and the menu name lists. `osc_state.h` declares them all and provides `OSC_LOCK()`, `OSC_UNLOCK()`, `OSC_SET(var, value)` and `OscMeasGet()` (a locked snapshot of `osc_meas`). The full list is in §6.

---

## 6. Shared state reference

All in `osc_state.c` / `osc_state.h` unless noted. "Lock" = how writers must write it.

### Settings (what the user chose)

| Variable | Meaning | Lock |
|---|---|---|
| `osc_vdiv_i`, `volts_per_div` | V/DIV index into `osc_vdivs[]` (default 12 = 1 V) and its value | — |
| `osc_tdiv_i` | T/DIV index into `osc_tdivs_ms[]` (default 8 = 1 ms). 0…11 FAST, 12…17 SLOW | — |
| `osc_fwin` | Display filter window (odd 1…21, default 5) | — |
| `osc_trig_fall` | 0 = rising edge, 1 = falling | — |
| `osc_calib` | 1 = AC coupling (default), 0 = DC | — |
| `osc_stable` | 1 = trigger lock (AUTO), 0 = ROLL | `OSC_SET` |
| `osc_auto_tb` | 1 = firmware picks FAST/SLOW + T/DIV continuously. 0 at power-on | `OSC_SET` |
| `osc_sidx` | Selected menu entry | `OSC_SET` |
| `hold_active` | Frozen | `OSC_SET` |
| `osc_c1`, `osc_c2` | Cursor LCD x (0…319) | — |

### Flags between modules

| Variable | Set by → read by |
|---|---|
| `osc_dirty` (`OSC_SET`) | any setting change → OscTask redraws once even when frozen |
| `osc_cur_dirty` | cursor moved while frozen → OscTask fast cursor repaint |
| `osc_auto_stage` (`osc_settings.c`) | `OscAutoScale()` → `OscAutoScaleStep()` until done |
| `slow_active` (volatile) | `SlowSync()` → everything that differs between FAST and SLOW |
| `osc_lock_auto` | `OscAutoTimebase()` (≥ 45 Hz) → FAST header shows AUTO:R/F |
| `slow_trig_on` | `SlowCaptureFrame()` → header (SLOW locked or scrolling) |
| `osc_frame_valid`, `new_frame_ready` | capture → process/draw; 0 = "ADC ERR" |
| `slow_busy`, `slow_adc3_dirty` (`adc_capture.c`) | FAST capture ↔ SysTick sampler: who owns the ADCs |
| `adc_pend` + `adc_pd_*` | `AdcPrestart()` → `AdcCaptureFrame()` / `AdcAbortPending()` |

### Data buffers

| Buffer | Content |
|---|---|
| `adc_raw_buf[65520]` (`adc_capture.c`, 32-byte aligned) | Raw DMA capture |
| `slow_hist[8192]` (`slow_mode.c`) | 1 ms history (written in the SysTick ISR) |
| `raw_samples[240]` | Display points, ADC counts |
| `osc_v[240]` | Calibrated + filtered input volts |
| `osc_env_lo_ct/hi_ct[240]`, `osc_env_on` | Peak-detect band (counts) when a period has < 8 (FAST) / < 12 (SLOW) points |
| `new_y`, `old_y`, `env_y_lo/hi` | LCD rows |
| `osc_meas` (`OSC_LOCK`) | The numbers shown: vmax, vmin, vpp, vrms, freq, dc_offset. Read with `OscMeasGet()` |
| `osc_raw_freq`, `osc_raw_vpp`, `osc_raw_*_ct`, `osc_raw_valid` | Raw measurement of this frame (counts / pin volts) |

---

## 7. User controls: buttons

### Buttons (`buttons.c`)

> ⚠️ The `Button` struct **names** come from the original 4-button mock-up and no longer
> match their roles. Go by the pin, not the variable name.

| Pin | Variable | Short press | Long press |
|---|---|---|---|
| PB0 | `btn_up` | + on the selected setting (hold = repeat) | — |
| PB1 | `btn_down` | − on the selected setting (hold = repeat) | — |
| PB2 | `btn_hold` | one-shot AUTO scale (`ActAuto`) | 1.5 s: AC ↔ DC (`ActToggleCoupling`) |
| PB3 | `btn_set` | FAST ↔ SLOW by hand (`ActToggleFastSlow`) | 1 s: automatic FAST/SLOW (`ActAutoMode`) |
| PA3 | `btn_vdiv` | select V/DIV ↔ T/DIV | — |
| PC0 | `btn_tdiv` | select FILTER ↔ TRIG | — |
| PC3 | `btn_filt` | trigger lock AUTO ↔ ROLL (`ActToggleStable`) | 1.5 s while frozen: select C1 ↔ C2 |
| PF3 | `btn_trig` | FREEZE on/off (`ActFreeze`) | — |
| PC13 (blue) | — | FAST ↔ SLOW by hand | 1 s: automatic FAST/SLOW |

There is no serial console (removed in v97). Every control is a button.

---

## 8. How to… (common changes)

### Add or change a button
1. Pin macros go in `app_config.h`; add the GPIO init in `AppInit()` (or configure it in CubeMX and remove the manual init).
2. Add a `Button` in `osc_state.c/.h` and initialise it in `AppInit()`.
3. Handle it in `HandleButtons()` using one of the existing patterns (`ButtonJustPressed`, `ButtonRepeat`, or the inline short/long block) and call an `Act*` function.
4. Update `BUTTON_HARDWARE_MAP.md` and the tests in `tests/test_eightbtn.py` / `tests/test_buttons.py`.

### Add a setting to the menu
1. Add the name to `osc_set_names[]` (and/or `osc_hold_names[]`) in `osc_state.c`, and bump `OSC_SET_COUNT` / `OSC_HOLD_COUNT` in `app_config.h` and the array sizes in `osc_state.h`.
2. Handle it in `OscAdjust()`, `OscSetValueText()` (`osc_settings.c`) and `SettingBarDraw()` (`display.c`).

### Change the V/DIV or T/DIV table
- V/DIV: `osc_vdivs[]` in `osc_state.c`, its size in `osc_state.h`, `OSC_VDIV_DEF`.
- T/DIV: `osc_tdivs_ms[]`, `OSC_NT`, `OSC_NT_FAST`, `OSC_TDIV_DEF`. Check every place that uses fixed indices: `AtbPickSlow()`, `atb_probe_order[]`, `OscAutoScaleStep()` (`OSC_NT_FAST + 4`), `OscManualToggle()`, and the tests (`T20`, `T50` … in `tests/test_slow.py`).
- SLOW starts at `SLOW_MIN_TDIV_MS` (10 ms) in `slow_mode.c`.

### Add a header field
Use `HDRT(text, x, y, fg, 1, bg)` inside `DrawTopPanel()`. Each `HDRT` call site uses one of 31 cache slots (`HDR_FIELDS = 32`, slot 31 is the RUN/HOLD status). If you go past 31 call sites, raise `HDR_FIELDS`. X positions must be ≤ 255 (the driver takes `uint8_t`).

### Add a new module
1. Create `Core/Src/app/<name>.c` (first line `#include "app/app.h"`) and `Core/Inc/app/<name>.h`.
2. Include the header from `app/app.h`.
3. Add the name to `MODULES` in `tests/app_src.py`, in reading order, or the host tests won't see it.

### Add a task
Create it in `AppStart()` next to the others, keep **one** `APP_LOCK` per pass around any work on scope state or the LCD, and make sure `configTOTAL_HEAP_SIZE` (24 KB) still fits all stacks. Check `dbg_stack_free[]` on the board. `tests/test_rtos_step3a.py` checks the task contracts and needs updating.

### Regenerate CubeMX code
Open `oscilloscope.ioc`, change what you need, *Generate Code*, then run `git diff`. Only the files you expect should change. Then build and run the tests. Never enable FreeRTOS middleware (§2).

---

## 9. Rules and pitfalls

**Locks and tasks**
- Lock order is always `xAppLock` (`APP_LOCK`) first, then `xOscMutex` (`OSC_LOCK`) inside it. Never the reverse, never nested `OSC_LOCK`.
- Never take a mutex in an ISR (`OscSlowTick`, ADC callbacks) or before `vTaskStartScheduler()`. On this board, creating the mutex too early hung the board with a white screen (v66).
- No LCD or HAL calls inside an `OSC_LOCK` section.
- `HandleButtons()` runs only in InputTask. Drawing runs only in OscTask.
- These rules are checked by `tests/test_rtos_step3a.py`.

**ADC and DMA**
- `adc_raw_buf` is DMA memory with the D-cache on. Every capture cleans/invalidates the cache by 32-byte lines before and invalidates after. Keep that if you add a capture path.
- ADC3 is shared by the SysTick sampler and the triple mode. Any burst capture sets `slow_busy = 1` while it runs. Anything that reconfigures or switches off ADC3 sets `slow_adc3_dirty = 1` so the sampler re-prepares it.
- `AdcSetRateLevel()` changes the **common** ADC prescaler, so all three ADCs are affected.
- DMA count is limited to 65 535 (`ADC_RAW_MAX = 65520`, whole cache lines).

**Build**
- The Debug build is `-O0`. The heavy sample loops are marked `FAST_O2` (compiled at -O2) because at -O0 they were far too slow (51 ms for one frequency read). Keep the attribute on new hot loops.
- `ADC_DC_ZERO_CT = 415` is a **temporary** board calibration: diode D1 leaks into the input. Set it back to 0 once the diodes are replaced.

**Tests depend on text markers**
- The host tests cut code out of the joined source between markers: `SLOW_BEGIN`/`SLOW_END`, `AUTO_TB_BEGIN`/`AUTO_TB_END`, `HOLD_BEGIN`/`HOLD_END`, `ACTIONS_BEGIN`/`ACTIONS_END`, `EXT_POINTS_BEGIN`/`EXT_POINTS_END`, `DRAWWF_BEGIN`/`DRAWWF_END`. Keep each marker unique and never write a marker name in another comment.
- A macro used by another module lives in that module's header, with a comment `/* NAME: defined in X.h (used by other modules) */` where it used to be. `tests/app_src.py` uses that comment to put it back, so keep the exact wording.

**Display**
- Only redraw what changed. A full-screen clear makes the numbers blink. The header uses the character cache, the waveform the column renderer.
- Any new screen element that the trace can cross must also be part of `BackgroundColour()`, or the renderer will erase it.

---

## 10. Tests

Run all of them: `python3 tests/run_all.py`. Each suite compiles real functions cut out of the app modules (through `tests/app_src.py`) against a modelled ADC/HAL and checks behaviour. They are **not** a board test.

| Suite | Covers |
|---|---|
| `test_osc.py` | measurement, FAST capture model, trigger, parameters (has 1 known old failure) |
| `test_dma.py` | DMA start/timeout/error/stop paths, cache maintenance, buffer wrap |
| `test_slow.py` | SLOW sampler, frequency reader, alias guards, triggered SLOW view, edge position (v96) |
| `test_autotb.py` | automatic FAST/SLOW switching in a closed loop |
| `test_hold.py` | frame-hold rules |
| `test_draw.py` | column renderer pixel by pixel, grid |
| `test_edge_filter.py` | filter edges from 2.4 to 120 points per cycle, DC mode |
| `test_buttons.py`, `test_eightbtn.py`, `test_blink.py` | button roles, AUTO scale, hold-repeat, selection blink; no UART/MOVE/ZOOM/simulator left (v97) |
| `test_realpic.py` | default picture, one-shot AUTO |
| `test_vdiv_range.py` | V/DIV table |
| `test_rtos_step3a.py` | lock rules and the two-task contracts |
| `build_arm.py` | compiles and links everything with arm-none-eabi-gcc outside the IDE |

Some checks compare against archived old sources in `../builds/` (on the original developer's machine, not in git). Those checks print `SKIP` when the archive is missing; that is normal.

`tests/app_src.py` joins the modules in the order of `MODULES`. It puts moved macros back, restores `static` on functions that only lost it because of the split, and inlines `AppInit()`/`AppStart()` into `main()`. That way the tests still see the "one main.c" they were written for.

---

## 11. Debugging on the board

- **SWD live variables** (CubeIDE *Live Expressions*, or `STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -r <addr> <bytes> out.bin` with the address from `arm-none-eabi-nm Debug/oscilloscope.elf`). Useful ones:

| Variable | Meaning |
|---|---|
| `dbg_frames_total`, `dbg_fps_x10` | frames drawn, frames/s × 10 |
| `prof_us[8]` | µs per stage: 0 capture, 1 process+measure+auto, 2 draw, 3 whole frame, 4 header, 5 measurement capture, 6 display capture / cursor move, 7 fast measurement |
| `dbg_osc_sleep_ms` | total time OscTask slept |
| `dbg_stack_free[2]` | stack words never used: [0] OscTask, [1] InputTask (2048 each; ~1760/1820 measured) |
| `dbg_adc_rate`, `dbg_adc_lvl`, `dbg_adc_start`, `dbg_adc_pre` | ADC rate, rate level, trigger start index, pre-trigger points |
| `dbg_atb_log[8]`, `dbg_atb_why` | last auto time base switches: `(why<<16)\|(from<<8)\|to` |
| `dbg_slow_spread`, `dbg_slow_rough`, `slow_valid_cnt` | SLOW alias guard inputs, history fill |
| `dbg_lcd_set`, `dbg_lcd_bar` | the exact header/bar text on the screen |

- **Writing a variable over SWD** only works reliably for variables in DTCM (addresses below `0x20020000`). The D-cache is on, so a debugger write to a variable in SRAM1 (`0x2002xxxx` and up) can be overwritten by the CPU's cached copy. Check the address with `arm-none-eabi-nm` first.

- **Stack overflow** halts in `vApplicationStackOverflowHook()` with interrupts off. Attach the debugger and look at `pcTaskName`.

---

## 12. Known limitations and open items

- Input 0–3.3 V at the pin only. Above ~1 MHz the amplitude is approximate (2.4 samples per cycle at 2 MHz, corrected by the bandwidth model).
- Below ~5 mV/div the picture is ADC noise (one 12-bit step = 0.8 mV at the pin).
- The LCD SPI transfer (~25 ms per frame) limits the frame rate; another draw task would not help.
- `ADC_DC_ZERO_CT` is a temporary calibration (§9).
- Button GPIOs are initialised by hand, not in the `.ioc`.
- `trigger_mode` SINGLE/NORM and `cursor_x` are left over from the first mock-up and are never changed at run time.
- Not yet verified on the board after the v93–v97 changes:
  - pressing every physical button (FREEZE and the C1/C2 cursors included)
  - looking at the LCD (the frozen header without the MOVE/zoom fields)
  - the SLOW trigger edge position (v96)

---

## 13. Other documents

| File | Content | Status |
|---|---|---|
| `README.md` | One-page overview | current |
| `SOURCE_CODE_GUIDE.md` | Architecture overview and version history (v64–v97) | current |
| `BUTTON_HARDWARE_MAP.md` | Pin wiring of the 8 buttons | current |
| `tests/README.md` | Measurement contract and the original test notes | partly historical (mentions `main.c`) |
| `RTOS_REFACTOR_STATUS.md` | Notes from the FreeRTOS migration (single task era, up to v76) | historical |
| `SESSION_SUMMARY.md` | Thai session notes from Sept 2026 (pre-RTOS) | historical |
