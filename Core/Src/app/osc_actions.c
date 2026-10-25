/* osc_actions.c - User actions (Act*): freeze, auto, up/down, MOVE/ZOOM of a frozen capture, AutoScale steps. */
#include "app/app.h"

static void     AutoScale(void);

/* MOVE_BEGIN ================================================================
 * v54: the four button roles (FREEZE, AUTO, UP/LEFT, DOWN/RIGHT) and the extra keys. One Act*() per role; UART keys and buttons call the same function.
 *   MOVE acts ONLY while frozen (hold_active). Not frozen: UP/DOWN keep their v53 job (adjust the setting chosen with 's').
 *   Y move: vertical offset added at draw time (0.1 div per step, +-4 div).  X move: pan of the frozen capture (1/40 screen per step) inside the data that was captured.
 *   Numbers (Vpp / F) are never touched by moving. Unfreeze, AUTO and '0' reset both offsets.
 *   F = force FAST<->SLOW (turns the auto time base off, = the PC13 short press), A = automatic switching again (= the PC13 long press).
 * ========================================================================== */
#define MV_Y_MAX_STEPS   40                  /* +-4 div */
#define MV_X_STEP_PTS    (N_SAMPLES / 40)    /* 1/40 screen = 6 display points */
#define AS_MIN_VPP       0.05f               /* below this Vpp there is no signal */
#define AS_MIN_CYC       1.95f               /* v56: one-shot AUTO, real-scope style: the fastest FAST base that shows >= 2 cycles (2-5 on screen) */

static float MvShownV(int idx)               /* the voltage of display point idx as drawn (panned view while X-moved) */
{
    return (mv_view_on ? mv_view_v : osc_v)[((idx % N_SAMPLES) + N_SAMPLES) % N_SAMPLES];
}

/* v85: cursor readouts. C1/C2 are LCD x positions; point i is drawn at x = i*SCREEN_W/N_SAMPLES and joined by straight
 * lines, so the voltage under pixel x is the line between its two neighbouring points. */
float OscCursorV(int x)
{
    float s = (float)x * (float)N_SAMPLES / (float)SCREEN_W;
    int   i = (int)s;
    float f = s - (float)i;
    if (i >= N_SAMPLES - 1) return MvShownV(N_SAMPLES - 1 + osc_plot_offset);
    return MvShownV(i + osc_plot_offset) * (1.0f - f) + MvShownV(i + 1 + osc_plot_offset) * f;
}
float OscCursorDtS(void)               /* C2 - C1 in seconds (time zoom included) */
{
    return (float)(osc_c2 - osc_c1) * (float)N_SAMPLES / (float)SCREEN_W / osc_fs_eff / (float)mv_zoom_x;
}

static void MvReset(void)                    /* offsets back to zero: the picture is the captured one again */
{
    mv_y_steps = 0; mv_x_pts = 0; mv_view_on = 0;
    mv_zoom_x = 1; mv_zoom_y = 1; mv_zoom_step = 0; mv_x_q = 0;      /* v57: zoom back to x1 */
    OSC_SET(osc_dirty, 1);
}

void MvSync(void)                     /* freeze / unfreeze edge: called from every Act*() and once per main-loop pass */
{
    if (hold_active && !mv_prev_hold)      { mv_y_steps = 0; mv_x_pts = 0; mv_view_on = 0; mv_zoom_x = 1; mv_zoom_y = 1; mv_zoom_step = 0; mv_x_q = 0; MvFreezeCapture(); }
    else if (!hold_active && mv_prev_hold) { MvReset(); mv_x_min = mv_x_max = 0; }
    mv_prev_hold = hold_active;
}

/* v57 ZOOM (frozen only). X: the view is 1/N of the screen around the screen centre; the pan (1/16 display point) may go as far as the stored data allows.
 * Left edge of the view = 120 - 120/N + p >= -nlo, right edge = 120 + 119/N + p <= 239 + nhi (N = 1 gives the v54 range). */
static void MvSetPanQ(int q)
{
    int zx = mv_zoom_x;
    q = OscClampI(q, mv_x_min * 16 - (1920 - 1920 / zx), mv_x_max * 16 + (1904 - 1904 / zx));
    mv_x_q = q;
    mv_x_pts = (q >= 0) ? (q / 16) : -((-q + 15) / 16);
}

void ActZoom(int dir)                 /* '+' (dir +1) / '-' (dir -1): only while frozen; v59: BOTH axes together, MOVE axis not involved */
{
    MvSync();
    if (!hold_active) return;
    int st = OscClampI((int)mv_zoom_step + (dir > 0 ? 1 : -1), 0, 6);   /* time x1..x64 (step 0..6), volts x1..x32 (capped at step 5) */
    if (st != mv_zoom_step) {
        int zx = 1 << st, zy = 1 << ((st > 5) ? 5 : st);
        int q = (mv_zoom_x > 1) ? mv_x_q : mv_x_pts * 16;
        int xchg = (zx != mv_zoom_x);
        mv_zoom_step = (uint8_t)st; mv_zoom_y = (uint8_t)zy; mv_zoom_x = (uint8_t)zx;
        if (xchg) {
            if (zx > 1) MvSetPanQ(q);
            else { int r = q + 8; mv_x_pts = OscClampI((r >= 0) ? (r / 16) : -((-r + 15) / 16), mv_x_min, mv_x_max); mv_x_q = 0; }
        }
        MvBuildView();
    }
    OSC_SET(osc_dirty, 1);
}

void MvFmtT(char *b, int n, float t_ms)       /* header T field: effective T/DIV (T/DIV / zoom), 4 decimals */
{
    if (t_ms < 0.1f) snprintf(b, (size_t)n, "T:%.4fus", (double)(t_ms * 1000.0f));
    else             snprintf(b, (size_t)n, "T:%.4fms", (double)t_ms);
}

void MvFmtV(char *b, int n, float v)          /* header V field: effective V/DIV (V/DIV / zoom), 4 decimals */
{
    snprintf(b, (size_t)n, "V:%.4fV", (double)v);
}

void MvFmtZ(char *b, int n)                   /* header Z field */
{
    if (mv_zoom_x > 1 && mv_zoom_y > 1 && mv_zoom_x != mv_zoom_y) snprintf(b, (size_t)n, "Z:%d/%d", mv_zoom_x, mv_zoom_y);   /* v59: time/volt only when they differ (x64/x32) */
    else if (mv_zoom_x > 1 && mv_zoom_x == mv_zoom_y) snprintf(b, (size_t)n, "Z:x%d   ", mv_zoom_x);
    else if (mv_zoom_x > 1)             snprintf(b, (size_t)n, "Z:x%d   ", mv_zoom_x);
    else if (mv_zoom_y > 1)             snprintf(b, (size_t)n, "Z:x%d   ", mv_zoom_y);
    else                                snprintf(b, (size_t)n, "        ");
}

static void MvMove(int dir, int n)           /* dir +1 = UP / LEFT, -1 = DOWN / RIGHT; only called while frozen */
{
    if (n < 1) n = 1;
    if (!mv_axis_x) {
        mv_y_steps = OscClampI(mv_y_steps + dir * n, -MV_Y_MAX_STEPS, MV_Y_MAX_STEPS);
    } else if (mv_zoom_x > 1) {                          /* v57: zoomed: finer steps (1/40 of the VIEW), range = the stored data */
        MvSetPanQ(mv_x_q - dir * n * ((MV_X_STEP_PTS * 16 + mv_zoom_x / 2) / mv_zoom_x));
        MvBuildView();
    } else {
        int p = OscClampI(mv_x_pts - dir * n * MV_X_STEP_PTS, mv_x_min, mv_x_max);
        if (p != mv_x_pts) { mv_x_pts = p; MvBuildView(); }
    }
    OSC_SET(osc_dirty, 1);
}

void ActToggleStable(void)            /* v63: PC3 short / PF3 = same as UART 'm' bare toggle */
{
    OSC_SET(osc_stable, (uint8_t)!osc_stable);
    OSC_SET(osc_dirty , 1);
}

void ActFreeze(void)                  /* FREEZE button / 'h' */
{
    OSC_SET(hold_active, !hold_active);
    OSC_SET(osc_sidx   , 0);
    OSC_SET(osc_dirty  , 1);
    MvSync();
}

void ActAuto(void)                    /* AUTO button / 'z': unfreeze, offsets to zero, auto scale */
{
    if (hold_active) { OSC_SET(hold_active, 0); OSC_SET(osc_sidx, 0); }
    MvSync();
    MvReset();
    AutoScale();                             /* = OscAutoScale() */
}

void ActToggleCoupling(void)          /* v90: AUTO (PB2) held BTN_ACDC_LONG_MS / UART "cal": AC <-> DC */
{
    osc_calib = !osc_calib;
    OSC_SET(osc_dirty, 1);
}

void ActUp(int n)                     /* UP/LEFT button / 'u' (n = steps, >1 only for a long hold) */
{
    MvSync();
    if (hold_active) MvMove(+1, n); else OscAdjust(+1);
}

void ActDown(int n)                   /* DOWN/RIGHT button / 'd' */
{
    MvSync();
    if (hold_active) MvMove(-1, n); else OscAdjust(-1);
}

/* v61 physical + / - buttons: ALWAYS OscAdjust (setting adjust), never MvMove/zoom, even while frozen.
 * Move/zoom/freeze stay UART-only (8-button design); UART 'u'/'d' still use ActUp/ActDown which branch on hold_active. */
void ActIncrease(int n)
{
    MvSync();
    for (int k = OscCursorSelected() ? n : 1; k > 0; k--) OscAdjust(+1);   /* v85: a long hold moves C1/C2 n samples per repeat */
}

void ActDecrease(int n)
{
    MvSync();
    for (int k = OscCursorSelected() ? n : 1; k > 0; k--) OscAdjust(-1);
}

/* v61/v62: select a setting by name in the CURRENTLY ACTIVE name array (osc_hold_names while frozen, osc_set_names otherwise).
 * Name not in the active array = silent no-op (osc_sidx untouched). */
void ActSelectSetting(const char *name)
{
    const char * const *names = hold_active ? osc_hold_names : osc_set_names;
    int n = OscSetCount();
    int i;
    for (i = 0; i < n; i++) {
        if (!strcmp(names[i], name)) { OscSetIndex(i); return; }
    }
}

/* v61 TRIGGER button: flip the edge directly, no need to select TRIG first. Works frozen or not. */
void ActToggleTrigEdge(void)
{
    osc_trig_fall = !osc_trig_fall;
    OSC_SET(osc_dirty, 1);
}

void ActToggleMoveAxis(void)          /* 'x' */
{
    mv_axis_x = (uint8_t)!mv_axis_x;
    OSC_SET(osc_dirty, 1);
}

void ActMoveCenter(void)              /* '0' */
{
    MvSync();
    MvReset();
}

void ActToggleFastSlow(void)          /* 'F' / PC13 short: force FAST <-> SLOW (opposite of the current one); auto time base off. Ignored while frozen. */
{
    if (hold_active) return;
    OscManualToggle();
}

void ActAutoMode(void)                /* 'A' / PC13 long: automatic FAST/SLOW switching again */
{
    OscAutoRearm();
}

/* AUTO scale, real-scope style.
 * V/DIV : the smallest step that keeps Vpp within OSC_V_FILL_DIVS divisions (the screen shows ~5.3 divisions).
 * T/DIV : chosen with the SAME rules as the auto time base (AtbPickSlow below ATB_FAST_TO_SLOW_HZ, FAST with >= AS_MIN_CYC cycles),
 *         so the auto time base agrees with it and does not move it a moment later.
 * No signal (Vpp < AS_MIN_VPP): 1 V/div and the default FAST time base. */
static int OscAutoPickTdiv(float f)
{
    int want_slow = slow_active ? (f <= ATB_SLOW_TO_FAST_HZ) : (f < ATB_FAST_TO_SLOW_HZ);
    if (want_slow) return AtbPickSlow(f);
    for (int k = 0; k < OSC_NT_FAST; k++)
        if (f * osc_tdivs_ms[k] * (float)OSC_H_DIVS / 1000.0f >= AS_MIN_CYC) return k;
    return OSC_NT_FAST - 1;
}

static void OscAutoApply(float vpp, int ti)
{
    int vi = OSC_VDIV_DEF;                               /* default 1 V/div */
    if (vpp >= AS_MIN_VPP) {
        vi = (int)(sizeof(osc_vdivs) / sizeof(osc_vdivs[0])) - 1;
        for (int i = 0; i <= vi; i++) { if (osc_vdivs[i] * OSC_V_FILL_DIVS >= vpp) { vi = i; break; } }
    }
    osc_vdiv_i = vi;
    volts_per_div = osc_vdivs[vi];
    osc_tdiv_i = ti; /* captured time metadata changes only on acquisition */
    osc_auto_stage = 0;
    OSC_SET(osc_dirty, 1);
}

void OscAutoScaleStep(void)
{
    if (!osc_auto_stage || hold_active || !osc_frame_valid || !new_frame_ready) return;
    float f = osc_meas.freq;
    float vpp = ADC_INPUT_ENABLE ? osc_raw_vpp * ADC_INPUT_GAIN : osc_meas.vpp;   /* v84: input volts, like the trace */
    if (!osc_calib) {   /* v90: DC: size V/DIV by the larger absolute peak (from 0 V), as the span OscAutoApply fills in OSC_V_FILL_DIVS */
        float pk = fmaxf(fabsf(osc_meas.vmax), fabsf(osc_meas.vmin));
        vpp = pk * (OSC_V_FILL_DIVS / OSC_V_DC_DIVS);
    }
    if (f > 0.0f) {
        dbg_auto_fsel = f;
        OscAutoApply(vpp, OscAutoPickTdiv(f));
        return;
    }
    /* no periodic reading (yet) */
    if (osc_auto_tries < 250u) osc_auto_tries++;
    if (slow_active && vpp < AS_MIN_VPP) { OscAutoApply(0.0f, OSC_TDIV_DEF); return; }    /* the SLOW window is flat: no signal */
    if (!slow_active && osc_auto_tries == 15u) { osc_tdiv_i = OSC_NT_FAST + 4; OSC_SET(osc_dirty, 1); }   /* FAST sees only >= 40 Hz: look at the SLOW 200 ms/div window (2 s) */
    else if (slow_active && osc_auto_tries > 100u) OscAutoApply(vpp, OSC_TDIV_DEF);        /* not periodic: scale V/DIV, default FAST time base */
}
/* MOVE_END ================================================================= */
/* --------------------------------------------------------------------------
 * AutoScale
 * Adjust V/div so the waveform Vpp spans approximately 4 vertical divisions.
 * Triggered by BTN_SET held for BTN_LONG_MS.
 * -------------------------------------------------------------------------- */
static void AutoScale(void)
{
    OscAutoScale();
}
