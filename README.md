# STM32F767 Oscilloscope

A digital oscilloscope on a **NUCLEO-F767ZI** board with an **ILI9341 320×240 LCD**.
It measures a signal on pin PA0 and draws it live with Vpp, frequency, RMS, min/max and
cursors. It has eight front-panel buttons and runs on **FreeRTOS**. It is built with
**STM32CubeIDE**.

> New to the code? This README is the overview. For every function and how to change
> things safely, read **[`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md)**.

---

## Contents

1. [Features](#features)
2. [Hardware](#hardware)
3. [Quick start](#quick-start)
4. [Using the scope](#using-the-scope)
5. [How it works](#how-it-works)
6. [Source code map](#source-code-map)
7. [Tests](#tests)
8. [Debugging on the board](#debugging-on-the-board)
9. [Limitations](#limitations)
10. [Documents](#documents)

---

## Features

| | |
|---|---|
| **Time base** | 18 steps from 5 µs/div to 500 ms/div, 10 divisions across the screen |
| **FAST mode** (5 µs … 6 ms/div) | DMA burst capture at 300 kSa/s to 1.6 MSa/s, and **4.8 MSa/s** with three ADCs interleaved |
| **SLOW mode** (10 … 500 ms/div) | A 1 kHz sampler that always runs and keeps 8 s of history, so the picture is full as soon as you switch |
| **Automatic time base** | Switches FAST ↔ SLOW by itself (below 70 Hz → SLOW, above 90 Hz → FAST) |
| **One-shot AUTO scale** | Picks V/DIV and T/DIV once so the signal fits the screen |
| **V/DIV** | 0.1 mV … 2 V per division (1-2-5 steps) |
| **Trigger** | Rising or falling edge with a Schmitt trigger; free-running (ROLL) mode too |
| **Coupling** | AC (the trace is centred on the signal's mean) or DC (real volts from 0 V) |
| **Measurements** | Vpp, frequency, RMS, Vmax, Vmin, and wave type (sine/square/triangle) |
| **Freeze** | Stops capturing. Two cursors C1/C2 then show U1, U2, ΔV and Δt |
| **Display filter** | Moving average, 1 … 21 points |
| **Peak detect** | When a period has too few points, the min/max band is drawn so a fast signal is not lost |
| **Smooth display** | Only changed LCD columns and changed header characters are redrawn (~25 frames/s, no flicker) |

---

## Hardware

| Part | Detail |
|---|---|
| Board | NUCLEO-F767ZI (Cortex-M7, 96 MHz from the 8 MHz HSE) |
| Display | ILI9341 320×240 SPI LCD on **SPI5**: SCK PF7, MISO PF8, MOSI PF9, CS PC8, DC PC9, RST PC10 |
| Signal input | **PA0** (Arduino A0). **0 – 3.3 V at the pin only**. The front end gives 1.25 input volts per pin volt (`ADC_INPUT_GAIN`) |
| Buttons | PB0, PB1, PB2, PB3, PA3, PC0, PC3, PF3: wired to GND, internal pull-up. Plus the blue user button PC13 |

> ⚠️ Never put more than 3.3 V or less than 0 V on PA0. That can damage the MCU.

---

## Quick start

1. **Open:** in STM32CubeIDE, choose *File → Open Projects from File System* and pick this folder. It is the project root.
2. **Build:** *Project → Build*, Debug configuration. The expected result is 0 errors and 1 warning (`DrawLineBresenham` unused).
3. **Flash:** use *Run → Debug*, or run:
   ```bash
   STM32_Programmer_CLI -c port=SWD -w Debug/oscilloscope.elf -v -rst
   ```
4. **Connect** a signal generator to PA0 (keep it within 0–3.3 V) and GND.
5. **Run the host tests** (no board needed):
   ```bash
   python3 tests/run_all.py
   ```

---

## Using the scope

### Screen

```
┌──────────────────────────────────────────────────────────────┐
│ V:1.0000V  T:1.0000ms  FILT:5       RUN      AC              │  row 1: scales, selected setting, state, coupling
│ Vp:3.00V   F:2.03kHz   Rs:1.06                               │  row 2: measurements
│ Mx:1.52V   Mn:-1.48V   AUTO:R  CLIP  AUT                     │  row 3: min/max, trigger, clip warning, time base
├──────────────────────────────────────────────────────────────┤
│        · · · · · · · ·  waveform, dotted grid  · · · · · ·   │
└──────────────────────────────────────────────────────────────┘
```

When frozen, rows 2 and 3 show the cursor readouts instead: `U1`, `U2`, `dV`, `dt`, and FAST/SLOW.

### Buttons

| Button | Short press | Long press |
|---|---|---|
| **PB0** | + on the selected setting (hold to repeat) | — |
| **PB1** | − on the selected setting (hold to repeat) | — |
| **PB2** | One-shot AUTO scale | 1.5 s: AC ↔ DC coupling |
| **PB3** | FAST ↔ SLOW by hand | 1 s: automatic FAST/SLOW again |
| **PA3** | Select V/DIV ↔ T/DIV | — |
| **PC0** | Select FILTER ↔ TRIG | — |
| **PC3** | Trigger lock AUTO ↔ ROLL | 1.5 s while frozen: select C1 ↔ C2 |
| **PF3** | FREEZE on/off | — |
| **PC13** (blue) | FAST ↔ SLOW by hand | 1 s: automatic FAST/SLOW again |

The selected setting blinks in the header. PB0/PB1 change it. While frozen, the menu is
C1, C2 and V/DIV, and PB0/PB1 move the cursors one pixel per step. Details are in
[`BUTTON_HARDWARE_MAP.md`](BUTTON_HARDWARE_MAP.md).

---

## How it works

### Two tasks

```
main()  (CubeMX init) → AppInit() → AppStart() → FreeRTOS scheduler
                                                     │
          ┌──────────────────────────────────────────┴───────────────────────┐
  InputTask (every 1 ms, higher priority)               OscTask (the scope)
  APP_LOCK                                              APP_LOCK
    HandleButtons()  → Act*() actions                     every 33 ms: capture → process →
  APP_UNLOCK                                              measure → auto → draw
  wake OscTask                                            every 300 ms: header
                                                        APP_UNLOCK, sleep until the next deadline

  SysTick (1 kHz ISR): HAL tick + OscSlowTick() (SLOW history sampler) + FreeRTOS tick
```

The app lock (`xAppLock`) makes the two tasks take turns. A button press is always applied
between two frames, never in the middle of one. OscTask sleeps between deadlines, so it is
~98 % idle while frozen.

### One frame

| # | Step | Function | Result |
|---|---|---|---|
| 1 | Pick the mode | `SlowSync()` | FAST or SLOW from the T/DIV |
| 2 | Capture | `AdcCaptureFrame()` (FAST) / `SlowCaptureFrame()` (SLOW) | 240 display points in `raw_samples[]`, already aligned on the trigger edge |
| 3 | Calibrate + filter | `OscProcessFrame()` | `osc_v[240]` in input volts |
| 4 | Measure | `OscMeasureFrame()` | `osc_meas`: Vmax, Vmin, Vpp, RMS, frequency |
| 5 | Auto scale / auto time base | `OscAutoScaleStep()`, `OscAutoTimebase()` | may change V/DIV and T/DIV |
| 6 | Pre-start the next capture | `AdcPrestart()` | the next DMA burst runs while this frame is drawn |
| 7 | Draw | `MapToScreen()` → `DrawWaveform()` | only the LCD columns that changed |

**Frequency is not measured from the 240 drawn points.** It comes from the raw ADC capture
(regular edge spacing, with checks against noise and aliasing). That keeps the reading
correct even when the picture is heavily averaged.

---

## Source code map

All hand-written application code is in **`Core/Src/app/`** (`.c`) and **`Core/Inc/app/`**
(`.h`), one module per job. Each `.c` includes only `app/app.h`.

| Module | Lines | What it does |
|---|---:|---|
| `app_main` | 90 | `AppInit()`: ADC, button GPIOs, LCD. `AppStart()`: mutexes, the two tasks, the scheduler |
| `osc_task` | 165 | **OscTask**: the frame pipeline above, the header, FPS and stack monitoring |
| `input_task` | 19 | **InputTask**: polls the buttons every 1 ms under the app lock |
| `adc_capture` | 766 | FAST acquisition: DMA bursts, sample-rate levels, triple-ADC 4.8 MSa/s, trigger search, resampling to 240 points, peak-detect band, raw frequency reader |
| `slow_mode` | 495 | SLOW mode: the 1 kHz ADC3 sampler in SysTick, the 8 s history, the SLOW trigger, frequency and alias guards |
| `ext_points` | 45 | Display points just outside the FAST window, so the filter has real neighbours at the screen edges |
| `osc_process` | 216 | Offset removal (AC/DC), display filter, measurements, input-bandwidth correction, wave type |
| `osc_timebase` | 225 | Automatic FAST/SLOW switching and the frame-hold rules (do not draw a frame known to be wrong) |
| `osc_settings` | 109 | The setting menu (FILTER, TRIG, V/DIV, T/DIV; C1, C2, V/DIV while frozen) and `OscAdjust()` |
| `osc_actions` | 162 | One `Act*()` function per button role, the cursor readouts, the one-shot AUTO scale |
| `buttons` | 145 | Debounce, hold-repeat, long press, and which button calls which action |
| `display` | 520 | Everything on the LCD: trace mapping, column renderer, grid, cursors, setting bar, cached header text |
| `osc_state` | 150 | Shared variables and tables (V/DIV, T/DIV), the `OSC_LOCK` / `OSC_SET` helpers |
| `app_config.h` | — | Pins, screen geometry, timing constants, build switches, shared types |

Other folders:

| Path | What it is | Edit it? |
|---|---|---|
| `Core/Src/main.c`, `adc.c`, `dma.c`, `gpio.c`, `spi.c`, … | Generated by CubeMX from `oscilloscope.ioc` | Only inside `USER CODE` blocks |
| `Core/Src/stm32f7xx_it.c` | Interrupt handlers. SysTick calls `OscSlowTick()` | Only inside `USER CODE` |
| `Core/Src/ILI9341/` | LCD driver (martnak/STM32-ILI9341) | Rarely |
| `Core/Inc/FreeRTOSConfig.h`, `Core/Src/freertos_hooks.c` | Kernel settings, stack-overflow hook | Carefully |
| `ThirdParty/FreeRTOS/` | FreeRTOS V11.1.0 kernel, vendored | No |
| `Drivers/` | ST HAL + CMSIS | No |
| `tests/` | Host tests (below) | Yes |

> **CubeMX:** you can open `oscilloscope.ioc` and press *Generate Code* safely. On the
> committed tree it changes no file. **Do not enable FreeRTOS in the CubeMX middleware
> list.** The kernel lives in `ThirdParty/` on purpose.

---

## Tests

```bash
python3 tests/run_all.py
```

The tests cut the **real C functions** out of the app modules (`tests/app_src.py` joins
them), compile them on your PC with a modelled ADC and HAL, and check the behaviour. They
need Python 3 and `gcc` or `clang`. They are not a board test.

| Suite | Covers |
|---|---|
| `test_osc.py` | measurement, FAST capture, trigger |
| `test_dma.py` | DMA start/timeout/error paths, cache maintenance |
| `test_slow.py` | SLOW sampler, frequency, alias guards, SLOW trigger |
| `test_autotb.py` | automatic FAST/SLOW switching |
| `test_hold.py` | frame-hold rules |
| `test_draw.py` | column renderer, pixel by pixel |
| `test_edge_filter.py` | filter at the screen edges |
| `test_buttons.py`, `test_eightbtn.py`, `test_blink.py` | button roles, AUTO scale, hold-repeat, blink |
| `test_realpic.py` | default picture, one-shot AUTO |
| `test_vdiv_range.py` | V/DIV table |
| `test_rtos_step3a.py` | lock rules and the two-task contracts |

Expected today: **12 of 13 suites pass**. `test_osc.py` has one old known failure
(`behavior failures=1`). Any new failure comes from your change.

---

## Debugging on the board

Read variables live over SWD with CubeIDE *Live Expressions*, or with:

```bash
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -r <address> <bytes> out.bin
```

Take the address from `arm-none-eabi-nm Debug/oscilloscope.elf`. Useful variables:

- `dbg_frames_total`, `dbg_fps_x10`: frames drawn, frames per second × 10
- `osc_meas`: the measurements on the screen
- `prof_us[8]`: µs per pipeline stage
- `dbg_lcd_set`: the exact header text
- `dbg_stack_free[2]`: unused stack of OscTask and InputTask

The D-cache is on. A debugger **write** only sticks reliably for variables in DTCM
(addresses below `0x20020000`).

---

## Limitations

- Input 0–3.3 V at the pin only. Above ~1 MHz the amplitude is approximate.
- Below ~5 mV/div the picture is mostly ADC noise (one 12-bit step = 0.8 mV).
- The LCD SPI transfer limits the frame rate to ~25 frames/s.
- `ADC_DC_ZERO_CT = 415` is a temporary DC zero calibration for the current board (diode D1 leaks into the input).
- The button GPIOs are set up by hand in `AppInit()`, not in the `.ioc`.

---

## Documents

| File | Content |
|---|---|
| [`DEVELOPER_GUIDE.md`](DEVELOPER_GUIDE.md) | **Start here to change code:** every file and function, shared state, how-to recipes, rules and pitfalls |
| [`SOURCE_CODE_GUIDE.md`](SOURCE_CODE_GUIDE.md) | Architecture overview and the version history (v64–v97) |
| [`BUTTON_HARDWARE_MAP.md`](BUTTON_HARDWARE_MAP.md) | Button pins and wiring |
| [`tests/README.md`](tests/README.md) | Measurement contract and test notes |
| `RTOS_REFACTOR_STATUS.md`, `SESSION_SUMMARY.md` | Historical notes |
