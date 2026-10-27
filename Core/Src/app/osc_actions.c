#include "app/app.h"

static void     AutoScale(void);

#define AS_MIN_VPP       0.05f
#define AS_MIN_CYC       1.95f

float OscCursorV(int x)
{
    float s = (float)x * (float)N_SAMPLES / (float)SCREEN_W;
    int   i = (int)s;
    float f = s - (float)i;
    if (i >= N_SAMPLES - 1) return osc_v[N_SAMPLES - 1];
    return osc_v[i] * (1.0f - f) + osc_v[i + 1] * f;
}
float OscCursorDtS(void)
{
    return (float)(osc_c2 - osc_c1) * (float)N_SAMPLES / (float)SCREEN_W / osc_fs_eff;
}

void MvFmtT(char *b, int n, float t_ms)
{
    if (t_ms < 0.1f) snprintf(b, (size_t)n, "T:%.4fus", (double)(t_ms * 1000.0f));
    else             snprintf(b, (size_t)n, "T:%.4fms", (double)t_ms);
}

void MvFmtV(char *b, int n, float v)
{
    snprintf(b, (size_t)n, "V:%.4fV", (double)v);
}

void ActToggleStable(void)
{
    OSC_SET(osc_stable, (uint8_t)!osc_stable);
    OSC_SET(osc_dirty , 1);
}

void ActFreeze(void)
{
    OSC_SET(hold_active, !hold_active);
    OSC_SET(osc_sidx   , 0);
    OSC_SET(osc_dirty  , 1);
}

void ActAuto(void)
{
    if (hold_active) { OSC_SET(hold_active, 0); OSC_SET(osc_sidx, 0); }
    AutoScale();
}

void ActToggleCoupling(void)
{
    osc_calib = !osc_calib;
    OSC_SET(osc_dirty, 1);
}

void ActIncrease(int n)
{
    for (int k = OscCursorSelected() ? n : 1; k > 0; k--) OscAdjust(+1);
}

void ActDecrease(int n)
{
    for (int k = OscCursorSelected() ? n : 1; k > 0; k--) OscAdjust(-1);
}

void ActSelectSetting(const char *name)
{
    const char * const *names = hold_active ? osc_hold_names : osc_set_names;
    int n = OscSetCount();
    int i;
    for (i = 0; i < n; i++) {
        if (!strcmp(names[i], name)) { OscSetIndex(i); return; }
    }
}

void ActToggleTrigEdge(void)
{
    osc_trig_fall = !osc_trig_fall;
    OSC_SET(osc_dirty, 1);
}

void ActToggleFastSlow(void)
{
    if (hold_active) return;
    OscManualToggle();
}

void ActAutoMode(void)
{
    OscAutoRearm();
}

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
    int vi = OSC_VDIV_DEF;
    if (vpp >= AS_MIN_VPP) {
        vi = (int)(sizeof(osc_vdivs) / sizeof(osc_vdivs[0])) - 1;
        for (int i = 0; i <= vi; i++) { if (osc_vdivs[i] * OSC_V_FILL_DIVS >= vpp) { vi = i; break; } }
    }
    osc_vdiv_i = vi;
    volts_per_div = osc_vdivs[vi];
    osc_tdiv_i = ti;
    osc_auto_stage = 0;
    OSC_SET(osc_dirty, 1);
}

void OscAutoScaleStep(void)
{
    if (!osc_auto_stage || hold_active || !osc_frame_valid || !new_frame_ready) return;
    float f = osc_meas.freq;
    float vpp = osc_raw_vpp * ADC_INPUT_GAIN;
    if (!osc_calib) {
        float pk = fmaxf(fabsf(osc_meas.vmax), fabsf(osc_meas.vmin));
        vpp = pk * (OSC_V_FILL_DIVS / OSC_V_DC_DIVS);
    }
    if (f > 0.0f) {
        dbg_auto_fsel = f;
        OscAutoApply(vpp, OscAutoPickTdiv(f));
        return;
    }
    if (osc_auto_tries < 250u) osc_auto_tries++;
    if (slow_active && vpp < AS_MIN_VPP) { OscAutoApply(0.0f, OSC_TDIV_DEF); return; }
    if (!slow_active && osc_auto_tries == 15u) { osc_tdiv_i = OSC_NT_FAST + 4; OSC_SET(osc_dirty, 1); }
    else if (slow_active && osc_auto_tries > 100u) OscAutoApply(vpp, OSC_TDIV_DEF);
}
static void AutoScale(void)
{
    OscAutoScale();
}
