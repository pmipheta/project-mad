/* osc_settings.c - Setting menu (FILTER/TRIG/V/DIV/T/DIV, cursors) and the one-shot AutoScale request. */
#include "app/app.h"

/* --------------------------------------------------------------------------
 * MENU: SET วน setting, UP/DOWN ปรับค่า (เหมือน active_settings/osc_adjust)
 * -------------------------------------------------------------------------- */
int OscSetCount(void)
{
    return hold_active ? OSC_HOLD_COUNT : OSC_SET_COUNT;
}

const char *OscSetName(void)
{
    return hold_active ? osc_hold_names[osc_sidx % OSC_HOLD_COUNT]
                       : osc_set_names[osc_sidx % OSC_SET_COUNT];
}

int OscCursorSelected(void)   /* v85: frozen and the setting being adjusted is C1 or C2 */
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

void OscSetValueText(char *out, int n)   /* "FILTER:5" / "TRIG:RISE" ... */
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

uint8_t osc_auto_stage;      /* defined with the AutoScale state machine below */

void OscAdjust(int dir)      /* dir = +1 / -1 */
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
        /* v77: T/DIV หมุนได้เฉพาะในโซนของโหมดปัจจุบัน (FAST = 0..OSC_NT_FAST-1, SLOW = OSC_NT_FAST..OSC_NT-1)
         * -> ปรับ T/DIV สูง<->ต่ำ ไม่ข้ามเส้น 10 ms เอง โหมด FAST/SLOW จึงคงเดิมตามที่ผู้ใช้ตั้งไว้
         *    (เปลี่ยนโหมดได้ด้วยคีย์ 'F' / ปุ่ม PB3 / ปุ่มน้ำเงิน PC13 เท่านั้น ซึ่งจะกระโดด T/DIV ไปโซนใหม่ให้) */
        int in_slow = (osc_tdiv_i >= OSC_NT_FAST);
        int lo = in_slow ? OSC_NT_FAST : 0;
        int hi = in_slow ? (OSC_NT - 1) : (OSC_NT_FAST - 1);
        osc_tdiv_i = OscClampI(osc_tdiv_i + dir, lo, hi);
        /* New timebase takes effect only on the next capture. */
        OSC_SET(osc_auto_tb, 0);            /* manual choice wins until 'z' (AutoScale) re-arms AUTO */
        osc_auto_stage = 0;                 /* a pending one-shot AutoScale would otherwise overwrite this T/DIV (and flip FAST/SLOW) a few frames later */
    } else if (!strcmp(name, "C1")) {
        osc_c1 = OscClampI(osc_c1 + dir, 0, SCREEN_W - 1);   /* v85: 1 LCD pixel (was 5 samples = 6.7 px), holding repeats every 20 ms */
    } else if (!strcmp(name, "C2")) {
        osc_c2 = OscClampI(osc_c2 + dir, 0, SCREEN_W - 1);
    }
    if (hold_active && (!strcmp(name, "C1") || !strcmp(name, "C2"))) osc_cur_dirty = 1;   /* v85: only the cursor columns change */
    else OSC_SET(osc_dirty, 1);             /* ให้วาดใหม่แม้อยู่ใน HOLD */
}


/* One AutoScale state machine for UART z and long SET. Consume a fresh,
 * valid capture only; frequency and amplitude come from the raw acquisition. */
uint8_t osc_auto_stage = 0;
uint8_t osc_auto_tries = 0;   /* frames without a valid frequency */
volatile float dbg_auto_fsel = 0.0f;

void OscAutoScale(void)
{
    if (hold_active) return; /* HOLD never replaces or retimes its capture. */
    osc_auto_stage = 1;
    /* v56: one-shot. 'z' does not touch osc_auto_tb (continuous auto time base only via 'A') */
    osc_auto_tries = 0;
    OSC_SET(osc_dirty, 1);
    UartPrint("\r\n AutoScale...\r\n");
}

float SlowProbeFreq(int ti);          /* defined in the SLOW block */
uint8_t SlowSlowish(void);            /* defined in the SLOW block: 1 = the newest history is a clearly moving, clearly slow signal */

uint8_t SlowRecentFlat(void);   /* 1 = the newest 1.5 s of the slow history is flat (the signal has stopped) */
