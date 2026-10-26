# Physical button hardware map

8 physical buttons (plus the blue PC13 user button) are the only controls;
the UART console was removed in v97. All 8 are wired active-LOW with an
internal pull-up (`GPIO_MODE_INPUT`, `GPIO_PULLUP`) — press = pin reads LOW.

| Pin | Button | Press action |
|---|---|---|
| **PB0** | INCREASE (+) | Increases whichever setting is currently selected (V/DIV, T/DIV, FILTER, or TRIG). Hold to auto-repeat (1 step, then every 60 ms after 300 ms, 3 steps per repeat after 1.5 s). |
| **PB1** | DECREASE (−) | Same as PB0, decreases. |
| **PB2** | AUTO / AC-DC | Short press: one-shot auto-scale: picks V/DIV + T/DIV + FAST/SLOW once to fit ~2–4 cycles on screen, then stays fixed. Hold ≥1500 ms: toggle AC ↔ DC coupling. |
| **PB3** | FAST/SLOW | Short press: toggle forced FAST↔SLOW mode. Long press ≥1000 ms: return to automatic FAST/SLOW switching. |
| **PA3** | SELECT V/DIV ↔ T/DIV | Toggles which of these two is the active setting for PB0/PB1 to adjust. While the picture is frozen, T/DIV isn't a valid setting (the frozen menu is C1/C2/V-DIV only) — pressing PA3 while frozen just selects V/DIV every time, harmlessly. |
| **PC0** | SELECT FILTER ↔ TRIG | Toggles which of these two is active for PB0/PB1. Does nothing while frozen (neither exists in the frozen menu). |
| **PC3** | STABLE/LOCK (short) / CURSOR C1↔C2 (long ≥1500 ms) | Short press: toggle trigger-lock display mode — stops the waveform sliding/scrolling like an animation, **without** stopping live capture. Long press (held past 1500 ms): toggle between C1/C2 cursor selection — only has a visible effect once the picture is truly frozen (via PF3); harmless no-op otherwise. |
| **PF3** | FREEZE | True freeze/unfreeze — stops capture entirely so the picture becomes a static snapshot you can inspect with the C1/C2 cursors. |

## Design notes
- PB0–PB3 pins were originally reserved (unused) as `BTN_UP/DOWN/HOLD/SET`
  for an earlier SET-menu-cycling scheme; their original UP/DOWN roles are
  reused directly (natural fit), HOLD and SET pins were repurposed to AUTO
  and FAST/SLOW since the final 8-button scheme doesn't need a scrolling
  menu.
- All 8 button pins are configured by hand in `AppInit()` (`Core/Src/app/app_main.c`) (CubeMX
  `.ioc` has not been regenerated with these — if you ever reopen the
  project in CubeIDE and regenerate code, these 4 GPIO inits must be kept
  or re-added by hand).
- Zoom, pan/move and axis-toggle of a frozen picture were UART-only and were
  removed together with the UART console in v97.
- Full history of how this mapping was decided (including two revisions
  after hardware testing) is in `Designs/2026-10-20-eight-button-input.md`
  in the shared vault.
