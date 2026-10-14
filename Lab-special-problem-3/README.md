# STM32F767 oscilloscope

The firmware is compiled from ordinary `.c` translation units in `Core/Src`,
with explicit interfaces in `Core/Inc`. No implementation files are included.

## Module map

| Module | Ownership / extension point |
|---|---|
| `main.c` | Cache/HAL/peripheral startup, system clock, fatal handler. Calls `OscAppInit()` once and `OscAppStep()` repeatedly. |
| `osc_app.c/.h` | Foreground settings, frame/header scheduling, HOLD/SINGLE gates, unified AutoScale state machine. Add application policy here. |
| `osc_acquisition.c/.h` | ADC1/DMA2 init, unique ADC/DMA IRQ handlers and callbacks, cache ownership, DWT timing, adaptive raw frequency measurement, triggering and resampling. Raw DMA buffer and completion/error flags do not cross its interface. |
| `osc_signal.c/.h` | Frame exchange storage, captured timing/validity, calibration, display filter, measurements and simulator trigger search. Add signal processing here. |
| `osc_display.c/.h` | LCD initialization/benchmark, private old/new pixel buffers, grid, waveform, cursors, panels and rendering diagnostics. |
| `osc_input.c/.h` | Private button debounce state and UART ring/parser; unique USART3 IRQ/callbacks; menu adjustment and reporting. Both UART `z` and long SET call the app AutoScale implementation. |
| `osc_simulator.c/.h` | Generator settings and three modeled signal sources; private phases/noise state. |
| `osc_types.h`, `osc_config.h` | Common types and compile-time constants only (no all-globals header). |

### Data flow and invariants

`input → app settings → acquisition → frame exchange → signal processing → display`

The concern-specific headers expose only the foreground state used across those
boundaries. Internal scheduler, UART, button, cursor, pixel and DMA bookkeeping
stays file-local. Existing `dbg_*` symbols remain available to the debugger.
This is a single-instance, foreground application, not a reentrant library.

- Requested `osc_tdiv_i` is separate from captured `osc_capture_tdiv` and
  `osc_fs_eff`. HOLD/filter redraws must not retime or replace the frame.
- ADC frequency uses a separate adaptive raw capture (40 Hz–100 kHz), not the
  filtered/display-decimated samples. DMA requests remain at most 65520 samples.
- Plotting and cursor readings use the same calibrated/filtered `osc_v` and
  `osc_plot_offset`.
- Acquisition failure invalidates measurements and prevents waveform redraw;
  successful later frames recover. Roll bypasses trigger alignment.
- Preserve DWT unsigned subtraction, full cache-line ownership, and checked HAL
  start/stop/error handling when changing acquisition.
- Add commands in input, signal algorithms in signal/acquisition, and visual
  changes in display. Keep IRQ definitions unique; do not duplicate them in
  `stm32f7xx_it.c` or generated peripheral files.

## Build and regression

Run from this directory (Python 3, clang for host checks, ARM GCC for firmware):

```sh
python3 tests/test_modules.py
python3 tests/test_osc.py
python3 tests/test_dma.py
python3 tests/test_linked.py
python3 tests/build_arm.py
python3 tests/build_arm.py --simulator
git diff --check
```

`test_osc.py` extracts the actual production routines from the modules and retains
all 210 frequency/timebase/filter cases plus HOLD, AutoScale, calibration, failure,
roll, no-signal, square-wave and cursor checks. `test_dma.py` extracts the production
capture/error routines against HAL/cache/DWT fault injection. `test_linked.py`
separately compiles and links the **unmodified** signal and simulator source files
against a minimal clock mock. Modeled inputs are not board measurements.

`build_arm.py` discovers every source in `Core/Src`, compiles separate objects and
links the actual firmware. Set `ARM_GCC` to an executable path, put
`arm-none-eabi-gcc` on PATH, or use the automatically discovered macOS CubeIDE
installation. `--simulator` sets `ADC_INPUT_ENABLE=0`; default is real ADC.
All test binaries, ELF/map files and logs go under `~/.hermes/cache/scratch/`;
existing CubeIDE `Debug/` artifacts are untouched.

CubeIDE Debug and Release already include `Core` recursively as a source entry,
so the new `.c` files need no manual source list or project-setting changes.
Refresh the project, then Clean/Build to regenerate its managed makefiles; do not
reuse a stale pre-refactor `Debug/Core/Src/subdir.mk`. The portable ARM build
verifies compilation/linking without flashing. LCD behavior, physical ADC signal
accuracy, board timing and CubeIDE GUI execution still require hardware/manual
validation. Clock/peripheral configuration is deliberately unchanged.
