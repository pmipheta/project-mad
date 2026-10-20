# FreeRTOS refactor — status (2026-10-20)

## What's flashed and working right now
**v65 (this commit)** — Step 1 of the FreeRTOS refactor only: the kernel
is in, the entire old bare-metal main loop runs inside ONE task
(`vMainTask`), zero functional change from the pre-RTOS firmware. Hardware
confirmed by the user: all 8 physical buttons, UART commands, Vpp/F
measurement, Slow mode, stable-lock, freeze — all work exactly like the
pre-RTOS version.

## What's NOT in this commit (parked, broken)
**Step 3a (splitting into tasks, starting with a mutex)** was attempted
twice and hung the board both times — screen blank white, ADC stops
capturing (confirmed via SWD: capture count stuck at 0). Root cause of
attempt #1 (mutex `Take()` calls running before `vTaskStartScheduler()`)
was found and fixed in attempt #2, but attempt #2 **still hung** —
meaning there is a second, not-yet-found cause. All PC-side tests passed
both times; the bug only shows on real hardware, which is why nothing
broken was pushed. This work is **on hold**, not abandoned — the next
attempt needs a different diagnosis approach (step through on actual
silicon via breakpoints, not just text-pattern tests).

Recovery each time: full board backup before every flash, mass-erase +
clean reflash of v65 confirmed working again both times.

## Known limitation found this session: Slow mode can't "stop roll"
The STABLE/lock toggle (PC3 short-press / UART `m`) only has an effect
when `osc_stable && osc_lock_auto` — and `osc_lock_auto` only turns on
at signal frequency ≥45 Hz. Slow mode exists specifically for signals
*below* that range, so the picture always rolls/free-runs in Slow mode,
regardless of the STABLE toggle. This is not something v61-v66 broke —
it's how the lock mechanism has always worked.

Fixing this would mean giving Slow mode its own trigger-lock logic,
which means touching the Slow block — **explicitly off-limits** per
standing project rule, unless the user clears that specific exception.
Not done. Waiting on a decision.

## Testing gotcha learned this session
Short physical button presses tested by typing in chat ("press X, tell
me when done") can accidentally exceed a button's long-press threshold
(e.g. PC3's 1500 ms) just from round-trip message latency — making a
working short-press action look broken. Confirmed by reading
`osc_stable`/GPIO registers live over SWD before and after a press:
the hardware and short-press logic both work correctly; the apparent
failure was the user unintentionally holding past 1.5 s while waiting
for a response. Test button timing with quick taps, not hold-and-wait.

## Files
- `Lab-special-problem-3/Core/Src/main.c` — v65: kernel + one task, else unchanged
- `Lab-special-problem-3/Core/Src/stm32f7xx_it.c` — SysTick dual-handler fix (HAL_IncTick + guarded xPortSysTickHandler)
- `Lab-special-problem-3/Core/Inc/FreeRTOSConfig.h` — kernel config for this target
- `Lab-special-problem-3/Middlewares/FreeRTOS/` — vendored kernel source (hand-added, not CubeMX-generated)
- `Lab-special-problem-3/tests/test_rtos_step1.py` — proves vMainTask body == original loop body
- `builds/v65-rtos-step1/`, `builds/v66-rtos-step3a/`, `builds/v66b-rtos-step3a-fix/` — full snapshots of each attempt, including the two broken ones (kept for when step 3 is retried)
- `Designs/2026-10-20-freertos-refactor.md` — the approved plan this work follows
