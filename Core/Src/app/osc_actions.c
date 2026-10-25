/* osc_actions.c - User actions (Act*): freeze, auto, +/-, setting select, FAST/SLOW, cursor readouts, AutoScale steps. */
#include "app/app.h"

static void     AutoScale(void);

/* ACTIONS_BEGIN =============================================================
 * One Act*() per button role (buttons.c calls them). Numbers (Vpp / F) are never touched by an action.
 * ========================================================================== */
#define AS_MIN_VPP       0.05f               /* below this Vpp there is no signal */
#define AS_MIN_CYC       1.95f               /* v56: one-shot AUTO, real-scope style: the fastest FAST base that shows >= 2 cycles (2-5 on screen) */


/* v85: cursor readouts. C1/C2 are LCD x positions; point i is drawn at x = i*SCREEN_W/N_SAMPLES and joined by straight
 * lines, so the voltage under pixel x is the line between its two neighbouring points. */
float OscCursorV(int x)
{
    float s = (float)x * (float)N_SAMPLES / (float)SCREEN_W;
    int   i = (int)s;
    float f = s - (float)i;
    if (i >= N_SAMPLES - 1) return osc_v[N_SAMPLES - 1];
    return osc_v[i] * (1.0f - f) + osc_v[i + 1] * f;
}
float OscCursorDtS(void)               /* C2 - C1 in seconds */
{
    return (float)(osc_c2 - osc_c1) * (float)N_SAMPLES / (float)SCREEN_W / osc_fs_eff;
}

void MvFmtT(char *b, int n, float t_ms)       /* header T field: T/DIV, 4 decimals */
{
    if (t_ms < 0.1f) snprintf(b, (size_t)n, "T:%.4fus", (double)(t_ms * 1000.0f));
    else             snprintf(b, (size_t)n, "T:%.4fms", (double)t_ms);
}

void MvFmtV(char *b, int n, float v)          /* header V field: V/DIV, 4 decimals */
{
    snprintf(b, (size_t)n, "V:%.4fV", (double)v);
}

void ActToggleStable(void)            /* v63: PC3 short */
{
    OSC_SET(osc_stable, (uint8_t)!osc_stable);
    OSC_SET(osc_dirty , 1);
}

void ActFreeze(void)                  /* FREEZE button (PF3) */
{
    OSC_SET(hold_active, !hold_active);
    OSC_SET(osc_sidx   , 0);
    OSC_SET(osc_dirty  , 1);
}

void ActAuto(void)                    /* AUTO button: unfreeze, auto scale */
{
    if (hold_active) { OSC_SET(hold_active, 0); OSC_SET(osc_sidx, 0); }
    AutoScale();                             /* = OscAutoScale() */
}

void ActToggleCoupling(void)          /* v90: AUTO (PB2) held BTN_ACDC_LONG_MS: AC <-> DC */
{
    osc_calib = !osc_calib;
    OSC_SET(osc_dirty, 1);
}

/* v61 physical + / - buttons: OscAdjust on the selected setting, running or frozen. */
void ActIncrease(int n)
{
    for (int k = OscCursorSelected() ? n : 1; k > 0; k--) OscAdjust(+1);   /* v85: a long hold moves C1/C2 n samples per repeat */
}

void ActDecrease(int n)
{
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

void ActToggleFastSlow(void)          /* PB3 / PC13 short: force FAST <-> SLOW (opposite of the current one); auto time base off. Ignored while frozen. */
{
    if (hold_active) return;
    OscManualToggle();
}

void ActAutoMode(void)                /* PB3 / PC13 long: automatic FAST/SLOW switching again */
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
    float vpp = osc_raw_vpp * ADC_INPUT_GAIN;   /* v84: input volts, like the trace */
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
/* ACTIONS_END ============================================================== */
/* --------------------------------------------------------------------------
 * AutoScale
 * Adjust V/div so the waveform Vpp spans approximately 4 vertical divisions.
 * Triggered by BTN_SET held for BTN_LONG_MS.
 * -------------------------------------------------------------------------- */
static void AutoScale(void)
{
    OscAutoScale();
}
