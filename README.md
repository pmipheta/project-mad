# oscilloscope

STM32F767ZI oscilloscope firmware built with STM32CubeIDE and FreeRTOS.

## Project layout

- `Core/` — application and MCU initialization code
- `Drivers/` — STM32 HAL and CMSIS dependencies
- `Middlewares/` — FreeRTOS kernel
- `tests/` — host-side regression tests
- `oscilloscope.ioc` — STM32CubeMX project configuration

## Run regression tests

```bash
python3 tests/run_all.py
```

Import this directory directly as the STM32CubeIDE project root.
