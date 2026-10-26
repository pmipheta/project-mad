/* buttons.c - Front-panel buttons: debounce, hold-repeat, long press, mapping to actions. */
#include "app/app.h"

static uint8_t  ButtonJustPressed(Button *btn, uint32_t now);
static uint8_t  ButtonLongPress(Button *btn, uint32_t now);

/* --------------------------------------------------------------------------
 * ButtonJustPressed
 * Returns 1 on the first confirmed press (rising edge with debounce).
 * Active-LOW buttons: GPIO reads 0 when pressed.
 * -------------------------------------------------------------------------- */
static uint8_t ButtonJustPressed(Button *btn, uint32_t now)
{
    uint8_t raw = (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET) ? 1 : 0;

    if (raw) {
        if (!btn->is_held) {
            /* Transition to pressed */
            btn->is_held       = 1;
            btn->held_since_tick = now;
            btn->long_fired    = 0;
            if (now - btn->last_press_tick >= BTN_DEBOUNCE_MS) {
                btn->last_press_tick = now;
                return 1;   /* confirmed short press */
            }
        }
    } else {
        btn->is_held    = 0;
        btn->long_fired = 0;
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * ButtonLongPress
 * Returns 1 once when the button has been held for at least BTN_LONG_MS.
 * -------------------------------------------------------------------------- */
static uint8_t ButtonLongPress(Button *btn, uint32_t now)
{
    if (btn->is_held && !btn->long_fired) {
        if (now - btn->held_since_tick >= BTN_LONG_MS) {
            btn->long_fired = 1;
            return 1;
        }
    }
    return 0;
}

/* v54: UP/DOWN are held, not toggled. Returns the number of steps due now: 1 on the press, then 1 every BTN_REPEAT_MS after BTN_REPEAT_FIRST_MS,
 * BTN_ACCEL_STEPS per repeat once held for BTN_ACCEL_MS. raw = 1 while the pin reads pressed. */
static uint32_t btn_rep_ms = BTN_REPEAT_MS;   /* v85: repeat period, BTN_CUR_REPEAT_MS while C1/C2 is adjusted */
static int ButtonRepeat(Button *b, uint8_t raw, uint32_t now)
{
    if (!raw) {
        if (b->is_held) {
            if (!b->low_pending) { b->low_pending = 1; b->low_tick = now; }
            else if ((now - b->low_tick) >= BTN_REL_MS) { b->is_held = 0; b->low_pending = 0; }
        }
        return 0;
    }
    b->low_pending = 0;
    if (!b->is_held) {
        b->is_held = 1; b->held_since_tick = now; b->next_rep_tick = now + BTN_REPEAT_FIRST_MS;
        return 1;
    }
    if ((int32_t)(now - b->next_rep_tick) >= 0) {
        b->next_rep_tick = now + btn_rep_ms;
        return ((now - b->held_since_tick) >= BTN_ACCEL_MS) ? BTN_ACCEL_STEPS : 1;
    }
    return 0;
}

static uint8_t ButtonRaw(const Button *b)
{
    return (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET) ? 1u : 0u;   /* active LOW */
}

/* --------------------------------------------------------------------------
 * HandleButtons
 * Call this every loop iteration (before frame logic).
 * Roles: HOLD pin = FREEZE | SET pin = AUTO | UP pin = INCREASE | DOWN pin = DECREASE | PC13 = FAST<>SLOW (short) / AUTO FAST/SLOW (long)
 * -------------------------------------------------------------------------- */
void HandleButtons(uint32_t now)
{
    int n;

    /* v61: PB0 = INCREASE, PB1 = DECREASE (hold-repeat; OscAdjust only) */
    btn_rep_ms = OscCursorSelected() ? BTN_CUR_REPEAT_MS : BTN_REPEAT_MS;   /* v85: C1/C2 glide; other settings keep 60 ms */
    if ((n = ButtonRepeat(&btn_up,   ButtonRaw(&btn_up),   now)) > 0) ActIncrease(n);
    if ((n = ButtonRepeat(&btn_down, ButtonRaw(&btn_down), now)) > 0) ActDecrease(n);

    /* PB2 = short (on release) AUTO one-shot scale | v90: held BTN_ACDC_LONG_MS (fires once) toggle AC <-> DC coupling.
     * Replaces the v89 PE0 switch input: PE0 sat on the analog node through SW1 and could not be read reliably. Same pattern as PB3. */
    { static uint8_t au_prev = 0, au_long = 0; static uint32_t au_t0 = 0;
      uint8_t pressed = ButtonRaw(&btn_hold);
      if (pressed && !au_prev) { au_t0 = now; au_long = 0; }
      if (pressed && !au_long && (now - au_t0) >= BTN_ACDC_LONG_MS) { au_long = 1; ActToggleCoupling(); }
      if (!pressed && au_prev && !au_long && (now - au_t0) >= 40u) ActAuto();
      au_prev = pressed; }
    (void)ButtonLongPress;

    /* v62: PA3 = toggle V/DIV <-> T/DIV select; PC0 = toggle FILTER <-> TRIG select (no-op where the target does not exist, e.g. frozen) */
    if (ButtonJustPressed(&btn_vdiv, now)) ActSelectSetting(strcmp(OscSetName(), "V/DIV") == 0 ? "T/DIV" : "V/DIV");
    if (ButtonJustPressed(&btn_tdiv, now)) ActSelectSetting(strcmp(OscSetName(), "FILTER") == 0 ? "TRIG" : "FILTER");
    (void)ActToggleTrigEdge;

    /* v64: PF3 = true FREEZE toggle (ActFreeze) */
    if (ButtonJustPressed(&btn_trig, now)) ActFreeze();

    /* v62: PC3 = short (on release) FREEZE | held BTN_CURSOR_LONG_MS (fires once) toggle C1<->C2 cursor (only while frozen). Same pattern as PB3 below. */
    {
        static uint8_t  cu_prev = 0, cu_long = 0;
        static uint32_t cu_t0 = 0;
        uint8_t pressed = ButtonRaw(&btn_filt);
        if (pressed && !cu_prev) { cu_t0 = now; cu_long = 0; }
        if (pressed && !cu_long && (now - cu_t0) >= BTN_CURSOR_LONG_MS) {
            cu_long = 1;
            if (hold_active) ActSelectSetting(strcmp(OscSetName(), "C1") == 0 ? "C2" : "C1");
        }
        if (!pressed && cu_prev && !cu_long && (now - cu_t0) >= 40u) ActToggleStable();
        cu_prev = pressed;
    }

    /* PB3 = FAST/SLOW: short = toggle (on release), held BTN_MAN_LONG_MS = back to automatic. Same pattern as PC13 below. */
    {
        static uint8_t  fs_prev = 0, fs_long = 0;
        static uint32_t fs_t0 = 0;
        uint8_t pressed = ButtonRaw(&btn_set);
        if (pressed && !fs_prev) { fs_t0 = now; fs_long = 0; }
        if (pressed && !fs_long && (now - fs_t0) >= BTN_MAN_LONG_MS) { fs_long = 1; ActAutoMode(); }
        if (!pressed && fs_prev && !fs_long && (now - fs_t0) >= 40u) ActToggleFastSlow();
        fs_prev = pressed;
    }

    /* PC13: manual FAST/SLOW. Acts on release (short) or after 1 s held (back to AUTO). */
    {
        static uint8_t  man_prev = 0, man_long = 0;
        static uint32_t man_t0 = 0;
        uint8_t pressed = (HAL_GPIO_ReadPin(BTN_MAN_PORT, BTN_MAN_PIN) == (BTN_MAN_ACTIVE_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET)) ? 1u : 0u;
        if (pressed && !man_prev) { man_t0 = now; man_long = 0; }
        if (pressed && !man_long && (now - man_t0) >= BTN_MAN_LONG_MS) { man_long = 1; ActAutoMode(); }
        if (!pressed && man_prev && !man_long && (now - man_t0) >= 40u) ActToggleFastSlow();
        man_prev = pressed;
    }
}
