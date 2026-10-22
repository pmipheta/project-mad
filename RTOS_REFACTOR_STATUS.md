# FreeRTOS refactor — status (2026-10-20)

## On the board now: v76 (this commit)
- **Step 1 (v65):** FreeRTOS kernel; the whole old bare-metal loop runs in one task
  (`vMainTask`). Hardware-verified: all 8 buttons, UART, measurements, Slow mode,
  stable-lock, freeze behave exactly like the pre-RTOS firmware.
- **Step 3a (v71): mutex around shared state — WORKING on hardware.**
  `xOscMutex` protects `osc_meas`, `osc_sidx`, `hold_active`, `osc_dirty`,
  `osc_stable`, `osc_auto_tb` (`OSC_LOCK/OSC_UNLOCK`, `OSC_SET`, `OscMeasGet`).
  Still one task; this prepares the split.
- v72–v76 on top: 4-decimal V/DIV and T/DIV, V/DIV down to 0.0001 V, no spikes at
  the filter edges, vertical grid every 1 T/DIV. See `SOURCE_CODE_GUIDE.md` §12.

## Step 3a: how the hang was found
Two earlier attempts (v66, v66b) froze the board (white screen, ADC capture count 0)
even though every PC test passed. It was isolated on the board one change at a time:

| Build | Change | Board |
|---|---|---|
| v67 | create the mutex only (no locking) | works |
| v68 | + one take/give at the start of the task | works |
| v69 | full locking, mutex created **early** in `main()` | **white screen** (reproduced) |
| v70/v71 | same full locking, mutex created **right before `xTaskCreate`** | works |

**Root cause:** creating the mutex early in `main()` (right after the clock setup,
before the peripheral/LCD init) hangs the board. The fix is the creation point.
Rules kept by `tests/test_rtos_step3a.py`: no take/give before
`vTaskStartScheduler()`, none in any ISR, no nested locking, no UART/LCD/HAL call
inside a critical section. Its negative control still flags the hanging v66 source.

## Next steps
1. Step 3b: move `HandleButtons` + `UartPoll` into `vInputTask`.
2. Step 3c: move drawing into `vDisplayTask`.
3. ADC / measurement / Slow block stay in the main task (timing-sensitive).
Each step: one change per flash, flash backup first, PC suites (`tests/run_all.py`)
plus a live check on the board.

## Known limitation: SLOW mode cannot "stop roll"
STABLE (PC3 short / UART `m`) only acts when `osc_stable && osc_lock_auto`, and
`osc_lock_auto` turns on at ≥ 45 Hz. SLOW signals are below that, so they always
roll. Changing this means editing the Slow block (frozen by project rule) — not done.

## Testing notes
- Button long-press tests over chat: round-trip delay can exceed the 1.5 s long-press
  of PC3 and make a working short press look broken. Verify by reading the variable
  over SWD around one quick tap.
- Judge test scripts by their printed results (`run_all.py`), not exit code alone:
  `test_eightbtn.py` used to print FAIL lines and still exit 0.

## Files
- `Core/Src/main.c` — application, task, mutex
- `Core/Src/stm32f7xx_it.c` — SysTick: `HAL_IncTick` always, kernel tick only after scheduler start
- `Core/Src/freertos_hooks.c` — stack-overflow hook (required by the config; was missing from the v65 commit)
- `Core/Inc/FreeRTOSConfig.h`, `Middlewares/FreeRTOS/` — kernel (vendored by hand, not CubeMX)
- `tests/test_rtos_step3a.py`, `tests/_unlock.py` — mutex rules; `_unlock` lets older text-based tests ignore the lock wrappers
