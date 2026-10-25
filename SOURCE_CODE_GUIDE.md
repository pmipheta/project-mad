# Source code guide — STM32F767 oscilloscope (`oscilloscope`)

This explains how the firmware is organised and how a signal travels from the
input pin to the LCD. For a file-by-file and function-by-function reference, and
for how to make common changes, see **`DEVELOPER_GUIDE.md`**. Read this together with:

- `BUTTON_HARDWARE_MAP.md` — the 8 physical buttons, pins and wiring
- `UART_input_commands.md` — every serial command
- `RTOS_REFACTOR_STATUS.md` — FreeRTOS migration state and what is verified on hardware

---

## 1. Hardware at a glance

| Part | Detail |
|---|---|
| MCU board | NUCLEO-F767ZI (Cortex-M7, SYSCLK 96 MHz from 8 MHz HSE) |
| Display | ILI9341 320×240 SPI LCD (SPI5), landscape |
| Signal input | **PA0 = ADC1_IN0** (Arduino A0). Range **0–3.3 V only** |
| Serial | USART3 = ST-LINK virtual COM, 115200 baud |
| Buttons | PB0–PB3, PA3, PC0, PC3, PF3 (active-LOW, internal pull-up) + PC13 blue user button (active-HIGH) |

The input is AC-displayed: the trace is drawn around the signal's own mean, so a
sine of 1.65 V DC ± 1 V appears centred on the screen.

---

## 2. Folder layout

| Path | What it is |
|---|---|
| `Core/Src/app/*.c`, `Core/Inc/app/*.h` | **All application code**, one module per job (table below). Each `.c` includes `app/app.h` |
| `Core/Src/main.c` | CubeMX-generated. Only USER CODE: `#include "app/app.h"`, then `AppInit(); AppStart();` after the `MX_*` init |
| `Core/Src/adc.c`, `dma.c` | CubeMX-generated ADC1 (PA0 = IN0) + DMA2 Stream0 init, `hadc1`, `hdma_adc1`, `HAL_ADC_MspInit` |
| `Core/Src/stm32f7xx_it.c` | Interrupt handlers (CubeMX). `SysTick_Handler` USER CODE drives the SLOW sampler and the FreeRTOS tick |
| `Core/Src/freertos_hooks.c` | `vApplicationStackOverflowHook` (required by `configCHECK_FOR_STACK_OVERFLOW = 2`) |
| `Core/Src/ILI9341/` | LCD driver (pixels, rectangles, text) |
| `Core/Src/gpio.c, spi.c, usart.c, tim.c, i2c.c, rng.c` | CubeMX-generated peripheral init |
| `Core/Inc/FreeRTOSConfig.h` | Kernel configuration (1 kHz tick, 24 KB heap, mutexes on). Hand-written, CubeMX does not touch it |
| `ThirdParty/FreeRTOS/` | FreeRTOS V11.1.0 kernel, vendored by hand (outside `Middlewares/`, which CubeMX manages) |
| `Drivers/` | ST HAL + CMSIS |
| `tests/` | Host (PC) tests that extract real C from the app modules and run it — see §10 |

| Module (`Core/Src/app/`) | Job |
|---|---|
| `app_config.h` (Inc only) | build switches, button pins, screen geometry, timing, shared types |
| `osc_state` | shared settings, measurement results, display buffers, `xOscMutex` + `OSC_LOCK/OSC_SET` |
| `adc_capture` | FAST capture: DMA bursts, rate levels, triple-ADC interleave, `AdcInputInit` (rate measurement) |
| `slow_mode` | SLOW mode: 1 kHz ADC3 history sampler (`OscSlowTick`) and the SLOW frame |
| `osc_process` | offset/AC-DC, display filter, measurements (Vmax/Vmin/Vpp/RMS/F) |
| `osc_timebase` | frame hold and the automatic FAST/SLOW time base |
| `osc_settings` | setting menu (FILTER/TRIG/V/DIV/T/DIV, cursors), AutoScale request |
| `osc_actions` | user actions `Act*`, MOVE/ZOOM of a frozen capture, AutoScale steps |
| `move_view` | data behind MOVE/ZOOM (points outside the window, raw copy, panned view) |
| `display` | LCD drawing: trace, grid, columns, cursors, setting bar, header |
| `buttons` | debounce, hold-repeat, long press, mapping to actions |
| `uart_console` | USART3 RX ring, status panel, command parser |
| `osc_sim` | signal simulator (only with `ADC_INPUT_ENABLE = 0`) |
| `osc_task` | **OscTask**: capture → process → draw → header, sleeps between deadlines |
| `input_task` | **InputTask**: buttons + UART every 1 ms |
| `app_main` | `AppInit()` (buttons GPIO, UART, LCD) and `AppStart()` (mutexes, tasks, scheduler) |

> **CubeMX Generate is safe.** Everything hand-written is either in `Core/Src/app`,
> `ThirdParty/`, `FreeRTOSConfig.h`, `freertos_hooks.c` or inside `USER CODE` blocks.
> ADC1/DMA live in `oscilloscope.ioc` with the exact settings the code needs; SVC/PendSV
> handler generation is off (FreeRTOS's port provides them). Checked: Generate on the
> committed tree changes no file, and the result builds.

`Core/Src/main.c.bak*` files are old personal backups, not part of the build logic
(the build compiles `*.c` only).

---

## 3. Start-up and run-time structure

```
main()  (CubeMX)
 ├─ HAL_Init, SystemClock_Config (96 MHz)
 ├─ MX_GPIO/DMA/RNG/SPI5/TIM1/USART3/I2C1/ADC1_Init
 ├─ AppInit():  AdcInputInit (DWT, rate measurement, triple ADC), 8 button GPIOs,
 │              Button structs, UartInputInit, ILI9341_Init, clear screen
 └─ AppStart(): xOscMutex + xAppLock, xTaskCreate(OscTask), xTaskCreate(InputTask),
                vTaskStartScheduler()          <- never returns

InputTask  (prio idle+2, 2048 words)           OscTask  (prio idle+1, 2048 words)
 ├─ wait until OscTask drew the start screen    ├─ once: UartPanel, DrawGrid, DrawCursor, DrawTopPanel
 └─ every 1 ms:                                 └─ for (;;)
     APP_LOCK                                       APP_LOCK
     HandleButtons(now), UartPoll()                 abort stale capture, C1/C2 fast repaint
     APP_UNLOCK                                     every FRAME_MS (33 ms): capture → process → measure → auto → draw
     wake OscTask                                   every HEADER_MS (300 ms): DrawTopPanel
                                                    APP_UNLOCK
                                                    sleep until the next frame/header, or until InputTask wakes it
```

The app lock makes the two tasks take turns exactly like the old single loop did
(input between two passes, never in the middle of a frame), so the behaviour is the
same; the difference is that nothing spins any more. Frozen (HOLD), OscTask sleeps
~98 % of the time (the old loop spun at 100 %); running, the frame rate is unchanged
(the LCD/SPI transfer is the limit, ~25 frames/s).

Interrupts (never take the mutex):

| ISR | Job |
|---|---|
| `SysTick_Handler` (1 kHz) | `HAL_IncTick()` always; `OscSlowTick()` (SLOW sampler); `xPortSysTickHandler()` only once the scheduler runs |
| `USART3_IRQHandler` → `HAL_UART_RxCpltCallback` | puts each received byte in the 96-byte ring `uart_rx[]` |
| `DMA2_Stream0_IRQHandler`, `ADC_IRQHandler` (CubeMX) → `HAL_ADC_ConvCpltCallback` / `HAL_ADC_ErrorCallback` | FAST capture completion |

**Hardware lesson (v66–v71):** creating the mutex early in `main()` (before the
LCD/peripheral init) hung the board with a white screen. Creating it immediately
before `xTaskCreate` works. Never call `xSemaphoreTake/Give` before
`vTaskStartScheduler()` or from an ISR.

---

## 4. One frame, step by step

Inside `OscTask`, every 33 ms (if not frozen):

1. **Capture**
   - FAST (T/DIV 5 µs … 6 ms): `AdcCaptureFrame()`
   - SLOW (T/DIV 10 … 500 ms): `SlowCaptureFrame()` (inside the mutex)
   Both fill `raw_samples[240]` — the 240 display points in ADC counts.
2. **`OscProcessFrame()`** — removes the DC offset (mean over whole cycles) and
   applies the moving-average filter (window `osc_fwin`, odd 1…21) → `osc_v[240]` in volts.
3. **`OscMeasureFrame()`** — Vmax, Vmin, Vpp, RMS, DC, frequency → `osc_meas`;
   input-bandwidth gain correction at high frequency; shows `---` when unknown.
4. **`OscAutoScaleStep()`** — only while a one-shot AUTO scale is running.
5. **`OscAutoTimebase()`** — automatic FAST/SLOW and T/DIV choice (when enabled).
6. **`OscFrameHold()`** — decides whether a frame the board knows is wrong
   (stale time base, generator glitch) should not be drawn.
7. **`MapToScreen()` → `DrawWaveform()`** — volts → pixel rows → LCD.

---

## 5. Acquisition

### FAST mode — `AdcCaptureFrame()` and the `Adc*` functions
- One DMA burst of up to 65 520 samples into `adc_raw_buf`.
- Sample-rate levels (`AdcSetRateLevel`): 800 kSa/s, 1.6 MSa/s, and **4.8 MSa/s
  triple-interleaved** (ADC1+2+3, `AdcCaptureTriple`) for the fastest bases.
- Trigger: Schmitt-trigger search on the raw data (rising/falling = TRIG setting),
  level = mid-range, refined on a smoothed copy so noise does not move the edge.
- Resampling: 240 display points from `start + i·step`, each a boxcar of `w` raw samples.
- `MvStoreExt()` also keeps up to 256 points **just outside** the window
  (`mv_ext_lo/hi`): used for X-pan when frozen and, since v76, as the real
  neighbours of the filter at the screen edges.
- `AdcRawFrequency()` — frequency from regular edge spacing; refuses noise.
- Pipelining: `AdcPrestart()` starts the next DMA capture while the current frame is drawn.

### SLOW mode — the Slow block (`SLOW_BEGIN =` … `SLOW_END =`)
- `OscSlowTick()` runs in SysTick at 1 kHz on **ADC3**: a burst of 16 conversions
  per tick (mean = sample, max−min = "fast content" spread) into an 8192-point
  history (`SLOW_HIST_N`, ≈ 8 s). It runs **all the time**, so switching to SLOW
  shows a full picture at once.
- `SlowCaptureFrame()` decimates the newest window of that history into
  `raw_samples`, with its own frequency reader (`SlowFreqFromPts`) and alias guards.
- `SlowFindTrigger()` locks the picture to the newest edge (2 points from the left
  since v96); without an edge the picture scrolls. Changes here must keep
  `tests/test_slow.py` passing and be checked on the board with a slow signal.

### FAST ↔ SLOW (AUTO time base) — `AUTO_TB_BEGIN =` block
- FAST → SLOW below **70 Hz**, SLOW → FAST above **90 Hz** (hysteresis between).
- Manual T/DIV, freeze and AUTO-scale pause it; UART `A` / PB3 long / PC13 long re-arm it.
- Every switch is logged with a reason code (`dbg_atb_why`, `dbg_atb_log[]`).

---

## 6. Display

| Piece | Function | Notes |
|---|---|---|
| Waveform | `DrawWaveform` (`DRAWWF_BEGIN` block) | **Column renderer**: each changed screen column is repainted once, top to bottom, in one SPI burst. No stale pixels, ~10× faster than per-pixel drawing |
| Background | `BackgroundColour(x,y)` | what lies under the trace: cursors > grid dot > black |
| Grid | `IS_GRID_DOT(x,y)` macro | **one rule** used by `DrawGrid`, erase, cursor clearing and the renderer. Rows every 40 px = 1 V/DIV; **vertical lines every 32 px = 1 T/DIV** (v75) |
| X mapping | `x = i·320/240` | 240 points over 320 px, 10 divisions → 24 points = 32 px per T/DIV |
| Header | `DrawTopPanel` + `HdrText` cache | only changed characters are redrawn; V/DIV and T/DIV with 4 decimals (v72); the selected setting blinks (300 ms) |
| Filter edges | `OscProcessFrame` | v74–v76: edge points use the real outside samples (FAST) or a safe fallback (SLOW), so the left/right edges never spike |

Scales:
- **V/DIV** (`osc_vdivs[]`, v73): 0.0001 … 2 V in 1-2-5 steps; default 1 V (`OSC_VDIV_DEF` = 12).
  Below ~5 mV/div the picture is ADC noise (12-bit step = 0.8 mV).
- **T/DIV** (`osc_tdivs_ms[]`): 5 µs … 6 ms FAST, 10 … 500 ms SLOW; default 1 ms.

---

## 7. Freeze, stable, zoom, cursors (`MOVE_BEGIN =` and `MOVE_DATA_BEGIN =` blocks)

| Feature | Variable | What it does |
|---|---|---|
| FREEZE (PF3, UART `h`) | `hold_active` | stops capturing; enables cursors C1/C2, MOVE X/Y, zoom |
| STABLE / trigger-lock (PC3 short, UART `m`) | `osc_stable` | keeps capturing live but aligns each frame on the trigger edge. FAST locks from **45 Hz** up (`osc_lock_auto`); SLOW locks whenever its history has a clean edge (v87) |
| Zoom (`+`/`-` while frozen) | `mv_zoom_x/y` | X up to ×64, Y up to ×32, from the stored raw data (no invented detail) |
| Pan | `mv_x_pts`, `mv_y_steps` | over the stored outside points / frozen SLOW history |
| Cursors | `osc_c1`, `osc_c2` | U1, U2, dV, dt in the header while frozen |

---

## 8. Controls

Physical buttons are read in `HandleButtons()` and call the same `Act*()` functions
as the UART commands, so both always behave identically.

| Button | Action |
|---|---|
| PB0 / PB1 | + / − on the selected setting (frozen: same, never moves the trace) |
| PB2 | short: one-shot AUTO scale · hold 1.5 s: AC ↔ DC coupling |
| PB3 | short: FAST↔SLOW (manual) · long: automatic FAST/SLOW again |
| PA3 | select V/DIV ↔ T/DIV |
| PC0 | select FILTER ↔ TRIG |
| PC3 | short: STABLE on/off · hold 1.5 s while frozen: C1 ↔ C2 |
| PF3 | FREEZE on/off |
| PC13 (blue) | short: FAST↔SLOW · 1 s hold: AUTO again |

UART: `UartPoll()` reads the ring buffer, `UartCommand()` dispatches
(`s u d h x 0 + - F A z m v t f a o n w c g ?`), `UartPanel()` prints the status.
Full list: `UART_input_commands.md`.

---

## 9. Shared state and the locks

Two plain (non-recursive) mutexes:

- `xAppLock` (`APP_LOCK/APP_UNLOCK`, `app_main.h`): whoever holds it may touch any scope
  state and the LCD. OscTask holds it for one whole pass, InputTask for one poll.
- `xOscMutex` (`OSC_LOCK/OSC_SET`, `osc_state.h`): the older fine-grained lock (step 3a)
  for the variables below; taken only inside a pass/poll. Order: `xAppLock` first, then
  `xOscMutex`, never the reverse.

| Protected variable | Written by | How |
|---|---|---|
| `osc_meas` | `OscMeasureFrame`, Slow block | `OSC_LOCK()` … `OSC_UNLOCK()`; readers use `OscMeasGet()` snapshots |
| `osc_sidx`, `hold_active`, `osc_dirty`, `osc_stable`, `osc_auto_tb` | `Act*`, `OscAdjust`, AUTO time base | `OSC_SET(var, value)` |
| Slow block writes | `SlowCaptureFrame` | its single call site is wrapped in the lock |

Rules (checked by `tests/test_rtos_step3a.py`): no lock from any ISR, no nested
`OSC_LOCK`, no UART/LCD/HAL call inside an `OSC_LOCK` section, nothing locks before the
scheduler starts, `HandleButtons`/`UartPoll` only in InputTask, one `APP_LOCK` per pass.

---

## 10. Tests (run on a PC, no board needed)

```
cd oscilloscope/tests
python3 run_all.py
```

`run_all.py` judges each suite by its exit code **and** its printed result (some
older scripts print `FAIL` but exit 0). Needs `gcc`/`clang` and Python 3.

| Suite | Covers |
|---|---|
| `test_osc.py` | measurement, trigger, FAST capture model, parameters |
| `test_dma.py` | DMA/cache/wrap faults |
| `test_slow.py` | Slow block: frequency reader, alias guards, triggered SLOW view |
| `test_autotb.py` | AUTO FAST/SLOW switching (closed-loop model) |
| `test_hold.py` | frame-hold rules |
| `test_draw.py` | column renderer, pixel-exact, incl. the T/DIV grid |
| `test_edge_filter.py` | filter edges from 2.4 to 120 points/cycle |
| `test_buttons.py`, `test_eightbtn.py`, `test_blink.py` | buttons, UART mapping, selection blink |
| `test_realpic.py`, `test_zoom.py` | default picture, zoom/pan; plus "unchanged since vNN" contracts |
| `test_vdiv_range.py` | V/DIV table 0.0001–2 V |
| `test_rtos_step3a.py` | mutex rules (pre-scheduler, ISR, nesting) |

The tests read the application through `tests/app_src.py`, which joins the modules into
one text in reading order (and inlines `AppInit`/`AppStart` into `main()`), then extract
code between the block markers (`SLOW_BEGIN =`, `AUTO_TB_BEGIN =`, `DRAWWF_BEGIN`, …).
**Keep every marker unique** across the modules, and never mention a marker name in
another comment. A new module must be added to `MODULES` in `app_src.py`.

Some contracts compare against archived sources in `builds/` (kept on the
developer machine, not in git); those checks are skipped when the archive is absent.

---

## 11. Building and flashing

- **STM32CubeIDE:** open the project and build (Debug). `.cproject` already has the
  `ThirdParty` source folder and the include paths `ThirdParty/FreeRTOS/Source/include`
  and `.../portable/GCC/ARM_CM7/r0p1`.
- **Command line (arm-none-eabi-gcc):** `python3 tests/build_arm.py`, or compile `Core/Src/**/*.c`,
  `Drivers/STM32F7xx_HAL_Driver/Src/*.c` (skip `*template*`) and the FreeRTOS files
  above with `-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -DUSE_HAL_DRIVER -DSTM32F767xx`,
  assemble `Core/Startup/startup_stm32f767zitx.s`, link with `-TSTM32F767ZITX_FLASH.ld
  --specs=nano.specs --specs=nosys.specs -u _printf_float -lm`.
- **Flash:** `openocd -f board/st_nucleo_f7.cfg -c 'program fw.elf verify reset exit'`.
  Back up the flash first (`dump_image backup.bin 0x08000000 0x200000`). If a
  programming run times out mid-write, do a `stm32f7x mass_erase 0` before reflashing.

---

## 12. Version history (this branch)

| Ver | Change | Hardware |
|---|---|---|
| v64 | 8-button panel (commit `d5aadc6`) | verified |
| v65 | FreeRTOS kernel, old loop moved into one task | verified |
| v71 | Step 3a mutex around shared state (created right before the task) | verified |
| v72 | V/DIV and T/DIV shown with 4 decimals | verified |
| v73 | V/DIV down to 0.0001 V | verified |
| v74/v76 | filter edges: no spikes at 2 MHz / 500 kHz | verified (edge/middle 0.83–1.02 at 420–520 kHz) |
| v75 | vertical grid line every 1 T/DIV | flashed |
| v93 | `main.c` split into `Core/Src/app` modules (pure move; machine code identical per function) | verified (A/B) |
| v94 | CubeMX owns ADC1/DMA (`adc.c`, `dma.c`); kernel moved to `ThirdParty/`; Generate is safe | verified (register A/B) |
| v95 | OscTask + InputTask under the app lock; OscTask sleeps between deadlines | verified (frame rate A/B, HOLD idle 98 %) |
| v96 | SLOW trigger edge 2 points from the left like FAST (was 10 %); readout keeps the old window | host tests; board: FAST only (no slow signal on the input) |

## 13. Known limitations

- Input 0–3.3 V only; above ~1 MHz amplitude readings are approximate (2.4 samples/cycle at 2 MHz).
- V/DIV below a few mV/div shows ADC noise, not detail.
- One display task: the LCD (SPI, ~25 ms per frame) is the bottleneck, so a separate draw task would not raise the frame rate.
