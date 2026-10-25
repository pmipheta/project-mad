# oscilloscope

STM32F767ZI oscilloscope firmware built with STM32CubeIDE and FreeRTOS.

## Project layout

- `Core/Src/app/`, `Core/Inc/app/` — the application, one module per job (see `SOURCE_CODE_GUIDE.md` §2)
- `Core/` (rest) — CubeMX-generated MCU initialization; hand-written code only inside `USER CODE` blocks
- `Drivers/` — STM32 HAL and CMSIS dependencies
- `ThirdParty/FreeRTOS/` — FreeRTOS kernel (kept outside `Middlewares/` so CubeMX Generate leaves it alone)
- `tests/` — host-side regression tests
- `oscilloscope.ioc` — STM32CubeMX project configuration

## Run regression tests

```bash
python3 tests/run_all.py
```

Import this directory directly as the STM32CubeIDE project root.
