#include "app/app.h"

static uint8_t  ButtonJustPressed(Button *btn, uint32_t now);
static uint8_t  ButtonLongPress(Button *btn, uint32_t now);

static uint8_t ButtonJustPressed(Button *btn, uint32_t now)
{
    uint8_t raw = (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET) ? 1 : 0;

    if (raw) {
        if (!btn->is_held) {
            btn->is_held       = 1;
            btn->held_since_tick = now;
            btn->long_fired    = 0;
            if (now - btn->last_press_tick >= BTN_DEBOUNCE_MS) {
                btn->last_press_tick = now;
                return 1;
            }
        }
    } else {
        btn->is_held    = 0;
        btn->long_fired = 0;
    }
    return 0;
}

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

static uint32_t btn_rep_ms = BTN_REPEAT_MS;
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
    return (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

void HandleButtons(uint32_t now)
{
    int n;

    btn_rep_ms = OscCursorSelected() ? BTN_CUR_REPEAT_MS : BTN_REPEAT_MS;
    if ((n = ButtonRepeat(&btn_up,   ButtonRaw(&btn_up),   now)) > 0) ActIncrease(n);
    if ((n = ButtonRepeat(&btn_down, ButtonRaw(&btn_down), now)) > 0) ActDecrease(n);

    { static uint8_t au_prev = 0, au_long = 0; static uint32_t au_t0 = 0;
      uint8_t pressed = ButtonRaw(&btn_hold);
      if (pressed && !au_prev) { au_t0 = now; au_long = 0; }
      if (pressed && !au_long && (now - au_t0) >= BTN_ACDC_LONG_MS) { au_long = 1; ActToggleCoupling(); }
      if (!pressed && au_prev && !au_long && (now - au_t0) >= 40u) ActAuto();
      au_prev = pressed; }
    (void)ButtonLongPress;

    if (ButtonJustPressed(&btn_vdiv, now)) ActSelectSetting(strcmp(OscSetName(), "V/DIV") == 0 ? "T/DIV" : "V/DIV");
    if (ButtonJustPressed(&btn_tdiv, now)) ActSelectSetting(strcmp(OscSetName(), "FILTER") == 0 ? "TRIG" : "FILTER");
    (void)ActToggleTrigEdge;

    if (ButtonJustPressed(&btn_trig, now)) ActFreeze();

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

    {
        static uint8_t  fs_prev = 0, fs_long = 0;
        static uint32_t fs_t0 = 0;
        uint8_t pressed = ButtonRaw(&btn_set);
        if (pressed && !fs_prev) { fs_t0 = now; fs_long = 0; }
        if (pressed && !fs_long && (now - fs_t0) >= BTN_MAN_LONG_MS) { fs_long = 1; ActAutoMode(); }
        if (!pressed && fs_prev && !fs_long && (now - fs_t0) >= 40u) ActToggleFastSlow();
        fs_prev = pressed;
    }

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
