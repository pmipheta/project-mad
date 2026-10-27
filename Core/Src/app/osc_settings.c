#include "app/app.h"

int OscSetCount(void)
{
    return hold_active ? OSC_HOLD_COUNT : OSC_SET_COUNT;
}

const char *OscSetName(void)
{
    return hold_active ? osc_hold_names[osc_sidx % OSC_HOLD_COUNT]
                       : osc_set_names[osc_sidx % OSC_SET_COUNT];
}

int OscCursorSelected(void)
{
    return hold_active && (!strcmp(OscSetName(), "C1") || !strcmp(OscSetName(), "C2"));
}

int OscClampI(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

float OscClampF(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void OscSetValueText(char *out, int n)
{
    const char *name = OscSetName();
    if      (!strcmp(name, "FILTER")) snprintf(out, n, "FILT:%d", osc_fwin);
    else if (!strcmp(name, "TRIG"))   snprintf(out, n, "TRIG:%s", osc_trig_fall ? "FALL" : "RISE");
    else if (!strcmp(name, "V/DIV"))  snprintf(out, n, "V:%.4f", (double)osc_vdivs[osc_vdiv_i]);
    else if (!strcmp(name, "T/DIV"))  snprintf(out, n, "T:%.4f", (double)osc_tdivs_ms[osc_tdiv_i]);
    else if (!strcmp(name, "C1"))     snprintf(out, n, "C1:%d", osc_c1);
    else if (!strcmp(name, "C2"))     snprintf(out, n, "C2:%d", osc_c2);
    else                              snprintf(out, n, "--");
}

void OscSetIndex(int idx)
{
    OSC_SET(osc_sidx, idx % OscSetCount());
    if (osc_sidx < 0) OSC_SET(osc_sidx, 0);
}

uint8_t osc_auto_stage;

void OscAdjust(int dir)
{
    const char *name = OscSetName();

    if (!strcmp(name, "FILTER")) {
        int v = osc_fwin + dir * 2;
        if ((v % 2) == 0) v += dir;
        osc_fwin = OscClampI(v, 1, 21);
    } else if (!strcmp(name, "TRIG")) {
        osc_trig_fall = !osc_trig_fall;
    } else if (!strcmp(name, "V/DIV")) {
        osc_vdiv_i = OscClampI(osc_vdiv_i + dir, 0, (int)(sizeof(osc_vdivs) / sizeof(float)) - 1);
        volts_per_div = osc_vdivs[osc_vdiv_i];
    } else if (!strcmp(name, "T/DIV")) {
        int in_slow = (osc_tdiv_i >= OSC_NT_FAST);
        int lo = in_slow ? OSC_NT_FAST : 0;
        int hi = in_slow ? (OSC_NT - 1) : (OSC_NT_FAST - 1);
        osc_tdiv_i = OscClampI(osc_tdiv_i + dir, lo, hi);
        OSC_SET(osc_auto_tb, 0);
        osc_auto_stage = 0;
    } else if (!strcmp(name, "C1")) {
        osc_c1 = OscClampI(osc_c1 + dir, 0, SCREEN_W - 1);
    } else if (!strcmp(name, "C2")) {
        osc_c2 = OscClampI(osc_c2 + dir, 0, SCREEN_W - 1);
    }
    if (hold_active && (!strcmp(name, "C1") || !strcmp(name, "C2"))) osc_cur_dirty = 1;
    else OSC_SET(osc_dirty, 1);
}

uint8_t osc_auto_stage = 0;
uint8_t osc_auto_tries = 0;
volatile float dbg_auto_fsel = 0.0f;

void OscAutoScale(void)
{
    if (hold_active) return;
    osc_auto_stage = 1;
    osc_auto_tries = 0;
    OSC_SET(osc_dirty, 1);
}

float SlowProbeFreq(int ti);
uint8_t SlowSlowish(void);

uint8_t SlowRecentFlat(void);
