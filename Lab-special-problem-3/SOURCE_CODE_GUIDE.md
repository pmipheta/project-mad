# Source code guide — STM32F767 oscilloscope (`Lab-special-problem-3`)

This explains how the firmware is organised and how a signal travels from the
input pin to the LCD. Read this together with:

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
| `Core/Src/main.c` | **Almost all application code** (~3800 lines): acquisition, measurement, auto modes, display, buttons, UART, RTOS task |
| `Core/Src/stm32f7xx_it.c` | Interrupt handlers. `SysTick_Handler` drives the HAL tick, the SLOW sampler and the FreeRTOS tick |
| `Core/Src/freertos_hooks.c` | `vApplicationStackOverflowHook` (required by `configCHECK_FOR_STACK_OVERFLOW = 2`) |
| `Core/Src/ILI9341/` | LCD driver (pixels, rectangles, text) |
| `Core/Src/gpio.c, spi.c, usart.c, tim.c, i2c.c, rng.c` | CubeMX-generated peripheral init |
| `Core/Inc/FreeRTOSConfig.h` | Kernel configuration (1 kHz tick, 15 KB heap, mutexes on) |
| `Middlewares/FreeRTOS/` | FreeRTOS kernel, **vendored by hand** (not CubeMX) |
| `Drivers/` | ST HAL + CMSIS |
| `tests/` | Host (PC) tests that extract real C from `main.c` and run it — see §10 |

> **Do not regenerate code with CubeMX.** The 8 buttons and FreeRTOS were added by
> hand in `main.c` / `Middlewares`; regeneration would delete them.

`Core/Src/main.c.bak*` files are old personal backups, not part of the build logic
(the build compiles `*.c` only).

---

## 3. Start-up and run-time structure

```
main()
 ├─ HAL_Init, SystemClock_Config (96 MHz)
 ├─ MX_* peripheral init, AdcInputInit (ADC1 + PA0 + DMA)
 ├─ manual GPIO init of the 8 buttons, Button structs
 ├─ UartInputInit, ILI9341_Init, clear screen
 ├─ xOscMutex = xSemaphoreCreateMutex()      <- created LAST, right before the task
 ├─ xTaskCreate(vMainTask, 2048 words, prio idle+1)
 └─ vTaskStartScheduler()                     <- never returns

vMainTask  (the old bare-metal while(1), one task)
 ├─ once: UartPanel, DrawGrid, DrawCursor, DrawTopPanel
 └─ for (;;)
     ├─ HandleButtons(now)          every pass
     ├─ UartPoll()                  every pass
     ├─ every FRAME_MS (33 ms):     capture → process → measure → auto → draw
     └─ every HEADER_MS (300 ms):   DrawTopPanel (text header)
```

Interrupts (never take the mutex):

| ISR | Job |
|---|---|
| `SysTick_Handler` (1 kHz) | `HAL_IncTick()` always; `OscSlowTick()` (SLOW sampler); `xPortSysTickHandler()` only once the scheduler runs |
| `USART3_IRQHandler` → `HAL_UART_RxCpltCallback` | puts each received byte in the 96-byte ring `uart_rx[]` |
| `DMA2_Stream0_IRQHandler`, `ADC_IRQHandler` | FAST capture completion |

**Hardware lesson (v66–v71):** creating the mutex early in `main()` (before the
LCD/peripheral init) hung the board with a white screen. Creating it immediately
before `xTaskCreate` works. Never call `xSemaphoreTake/Give` before
`vTaskStartScheduler()` or from an ISR.

---

## 4. One frame, step by step

Inside `vMainTask`, every 33 ms (if not frozen):

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
- **This block is frozen:** every change since v59 is checked to leave it
  byte-identical. Do not edit it without a separate decision.

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
| STABLE / trigger-lock (PC3 short, UART `m`) | `osc_stable` | keeps capturing live but aligns each frame on the trigger edge. Only effective from **45 Hz** up (`osc_lock_auto`); SLOW signals always roll |
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
| PB2 | one-shot AUTO scale |
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

## 9. Shared state and the mutex (RTOS step 3a)

Today there is still **one task**; the mutex prepares the split into input /
display tasks. `xOscMutex` is a plain (non-recursive) mutex.

| Protected variable | Written by | How |
|---|---|---|
| `osc_meas` | `OscMeasureFrame`, Slow block | `OSC_LOCK()` … `OSC_UNLOCK()`; readers use `OscMeasGet()` snapshots |
| `osc_sidx`, `hold_active`, `osc_dirty`, `osc_stable`, `osc_auto_tb` | `Act*`, `OscAdjust`, AUTO time base | `OSC_SET(var, value)` |
| Slow block writes | `SlowCaptureFrame` | its single call site is wrapped in the lock |

Rules (checked by `tests/test_rtos_step3a.py`): no lock from any ISR, no nested
locking, no UART/LCD/HAL call inside a critical section, nothing locks before the
scheduler starts.

---

## 10. Tests (run on a PC, no board needed)

```
cd Lab-special-problem-3/tests
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

The tests extract code between the block markers (`SLOW_BEGIN =`, `AUTO_TB_BEGIN =`,
`DRAWWF_BEGIN`, …). **Keep every marker unique** in `main.c`, and never mention a
marker name in another comment.

Some contracts compare against archived sources in `builds/` (kept on the
developer machine, not in git); those checks are skipped when the archive is absent.

---

## 11. Building and flashing

- **STM32CubeIDE:** open the project and build (Debug). FreeRTOS sources must be
  in the build: `Middlewares/FreeRTOS/Source/*.c`, `portable/GCC/ARM_CM7/r0p1/port.c`,
  `portable/MemMang/heap_4.c`, include paths `Middlewares/FreeRTOS/Source/include`
  and `.../portable/GCC/ARM_CM7/r0p1`.
- **Command line (arm-none-eabi-gcc):** compile `Core/Src/*.c`, `Core/Src/ILI9341/*.c`,
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

## 13. Known limitations

- Input 0–3.3 V only; above ~1 MHz amplitude readings are approximate (2.4 samples/cycle at 2 MHz).
- STABLE has no effect below ~45 Hz (SLOW always rolls) — changing that needs the Slow block.
- V/DIV below a few mV/div shows ADC noise, not detail.
- Still one RTOS task; the input-task / display-task split is the next step.
