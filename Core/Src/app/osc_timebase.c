/* osc_timebase.c - Frame hold (do not draw a known-bad frame) and the automatic FAST/SLOW time base. */
#include "app/app.h"

/* HOLD_BEGIN =============================================================
 * Do not draw a frame the board knows is wrong:
 *  1. a frame captured under another time base than the one now selected (the auto time base switched while it was being processed): stale, skip it;
 *  2. FAST has a real signal (>= 0.30 V) it cannot read (a slow wave looks like a straight line in its 60 ms window, e.g. 101 -> 1 Hz): keep the last good
 *     picture for at most 0.5 s while the board switches;
 *  3. the amplitude collapsed (< 0.30 V) within 2 s of a good picture (the generator blanks its output while a digit is edited): keep the picture up to 2 s;
 *     fast signals (adc_fast_content), SLOW frames, a changed time base and a collapse that lasts longer than 2 s are drawn.
 * Returns 1 = skip the waveform of this frame. Readable FAST frames record the time/base of the last good picture. */
/* The frequency of a FAST frame is only trusted when it was measured in this frame, or when the picture agrees with it. A held reading (1.5 s memory that keeps F steady while a
 * 100 Hz signal is read on every other frame) used to keep "F: 101 Hz" on the screen for up to 1.5 s after the signal changed to 1 Hz, while the window showed a sloping straight line:
 * the board believed it could read the signal, so it neither switched nor held the picture. A held F that promises >= 3 cycles in the window but finds < 2 crossings is not trusted. */
static uint8_t OscFreqTrusted(void)
{
    float cyc;
    if (osc_meas.freq <= 0.0f) return 0;
    if (!adc_f_held) return 1;
    cyc = osc_meas.freq * osc_capture_tdiv * (float)OSC_H_DIVS / 1000.0f;
    return (cyc >= 3.0f && osc_disp_cross < 2) ? 0 : 1;
}
#define HOLD_MAX_MS 500u
#define HOLD_COLLAPSE_MS 2000u   /* the generator blanks its output while a digit is edited (board log: 1.8 s of 0.06-0.5 V wiggles between 101 Hz and 1 Hz) */
static uint32_t osc_good_t    = 0;      /* tick of the last drawn readable FAST frame (0 = none) */
static int      osc_good_tdiv = -1;     /* its time base */
uint8_t OscFrameHold(uint32_t now)
{
    if (osc_capture_tdiv != osc_tdivs_ms[osc_tdiv_i]) return 1;                              /* 1: captured under another base */
    if (slow_active) { osc_good_t = 0; return 0; }
    if (OscFreqTrusted()) { osc_good_t = now ? now : 1u; osc_good_tdiv = osc_tdiv_i; return 0; }
    if (osc_good_t && osc_good_tdiv == osc_tdiv_i) {
        uint32_t age = now - osc_good_t;
        if (osc_raw_vpp >= 0.30f) { if (age < HOLD_MAX_MS && !adc_fast_content) return 1; }  /* 2: unreadable real signal: keep the picture */
        else if (age < HOLD_COLLAPSE_MS) return 1;                                           /* 3: the signal collapsed to a flat sloping line after a good picture: keep it for up to 2 s */
    }
    return 0;
}
/* HOLD_END ================================================================ */


/* AUTO_TB_BEGIN =============================================================
 * AUTO time base: เลือก FAST/SLOW และ T/DIV เองตามความถี่ที่วัดได้ (ปิดเมื่อผู้ใช้ปรับ T/DIV เอง; 'z' เปิดใหม่)
 *   FAST (burst DMA)  วัดความถี่ได้ >= ~40 Hz       SLOW (SysTick ring) วัดได้ ~0.2-60 Hz
 *   กฎ:  FAST + f<45 Hz  -> SLOW (เลือก T/DIV ให้เห็น >= 3 คาบ และ f <= Fs/8)
 *        SLOW + f>55 Hz  -> FAST (เลือก T/DIV ให้เห็น ~2.5 คาบ)      (45-55 Hz = hysteresis กันสลับไปมา)
 *        FAST อ่านไม่ได้ -> ถามประวัติ SLOW (ADC3 เก็บตลอดเวลา) ทุก 250 ms: ถ้าอ่านได้ -> สลับทันที ไม่ต้องรอจอเต็ม
 *        SLOW อ่านไม่ได้ -> ลอง base อื่นจากประวัติ ถ้าไม่ได้ 1.5 s -> FAST
 *   ไม่มีสัญญาณ = ไม่เปลี่ยนอะไร  |  HOLD / AutoScale กำลังทำงาน = ไม่ทำงาน
 * ========================================================================== */
/* ATB_FAST_TO_SLOW_HZ: defined in osc_timebase.h (used by other modules) */
/* ATB_SLOW_TO_FAST_HZ: defined in osc_timebase.h (used by other modules) */
#define ATB_BAD_MS           1500u
#define ATB_FAST_CYC         6.0f     /* FAST: the window should show about this many cycles (SLOW shows 5-9 at the hand-off) */

static uint32_t atb_t_switch  = 0;      /* tick of the last switch */
static uint32_t atb_t_bad     = 0;      /* tick when SLOW first had a signal it could not read (0 = no) */
static uint32_t atb_t_probe   = 0;      /* tick of the last history probe */
static float    atb_cand_f    = 0.0f;   /* SLOW repick candidate: a new base is picked only after 3 consistent readings (a single bogus reading used to park a 1 Hz signal on 10 ms/div for 4 s) */
static uint8_t  atb_cand_n    = 0;
static uint32_t atb_t_unread  = 0;      /* tick when FAST first saw a real signal it could not read (a slow wave looks like a straight line in its 60 ms window) */
static uint32_t atb_t_hold    = 0;      /* tick of the last SLOW -> FAST switch caused by a fast signal: FAST needs ~1 s for its first reading and the history still holds the OLD slow signal */
static uint32_t atb_t_flat    = 0;      /* tick when SLOW first saw a flat window (no signal) */
static uint8_t  atb_slowish_n = 0;      /* consecutive probes that saw a slow moving signal while FAST could read nothing */
static float    atb_last_f    = 0.0f;   /* last frequency that was read in either mode (used by the manual toggle) */
static uint8_t  atb_last_fast = 0xFFu;  /* last FAST time base that gave a reading (0xFF = none yet) */
static uint32_t atb_last_fast_t = 0;    /* tick when it was set */
static int AtbFastFallback(uint32_t now)   /* recent FAST base, else 1 ms/div (a flat line at 5 us/div hides everything below 100 kHz) */
{ return (atb_last_fast != 0xFFu && (now - atb_last_fast_t) < 15000u) ? (int)atb_last_fast : OSC_TDIV_DEF; }
volatile uint8_t dbg_atb_switches = 0;  /* number of switches (SWD) */

static int AtbPickFast(float f)         /* FAST base ที่ได้ ~2.5 คาบ */
{
    int ti = 0; float best = 1.0e30f;
    for (int k = 0; k < OSC_NT_FAST; k++) {
        float d = fabsf(f * osc_tdivs_ms[k] * (float)OSC_H_DIVS / 1000.0f - ATB_FAST_CYC);
        if (d < best) { best = d; ti = k; }
    }
    return ti;
}

int AtbPickSlow(float f)         /* SLOW base for frequency f. One base covers a WIDE range, so neighbouring frequencies (4, 5, 6 Hz; 11 -> 1 Hz; 20 -> 10 Hz) keep the same
                                         * time base and the picture just stretches like the signal does. The frequency is measured on whichever window can read it. */
{
    if (f <  0.9f)  return OSC_NT - 1;     /* 0.2 - 0.9 Hz : 500 ms/div */
    if (f < 12.0f)  return OSC_NT - 2;     /* 0.9 - 12 Hz  : 200 ms/div (1 Hz = 2 cycles, 12 Hz = 24 cycles) */
    if (f < 45.0f)  return OSC_NT - 4;     /* 12 - 45 Hz   :  50 ms/div */
    return OSC_NT - 6;                     /* 45 Hz and up :  10 ms/div (4.5 cycles at 45 Hz, 9 at 90 Hz: close to what FAST shows) */
}

volatile uint8_t dbg_atb_why = 0, dbg_atb_from = 0, dbg_atb_to = 0;   /* last switch: reason code, old base, new base */
volatile float   dbg_atb_arg = 0.0f;                                  /* the frequency that decided it */
volatile uint32_t dbg_atb_log[8];                                      /* ring of the last 8 switches: (why<<16)|(from<<8)|to */
volatile uint8_t  dbg_atb_log_i = 0;
volatile float    dbg_atb_arg_log[8];                                  /* the frequency that decided each logged switch */
static void AtbSwitch(int ti)
{
    if (ti == osc_tdiv_i) return;
    dbg_atb_from = (uint8_t)osc_tdiv_i; dbg_atb_to = (uint8_t)ti;
    dbg_atb_arg_log[dbg_atb_log_i & 7u] = dbg_atb_arg;
    dbg_atb_log[dbg_atb_log_i & 7u] = ((uint32_t)dbg_atb_why << 16) | ((uint32_t)osc_tdiv_i << 8) | (uint32_t)ti; dbg_atb_log_i++;
    osc_tdiv_i   = ti;
    atb_t_switch = HAL_GetTick();
    atb_t_bad    = 0;
    atb_t_flat   = 0;
    atb_t_unread = 0;
    atb_cand_n   = 0;
    OSC_SET(osc_dirty   , 1);
    dbg_atb_switches++;
}

/* time bases the history probe tries, best first: 100, 50, 200, 500 ms/div */
static const uint8_t atb_probe_order[6] = { OSC_NT_FAST + 3, OSC_NT_FAST + 2, OSC_NT_FAST + 4, OSC_NT_FAST + 5, OSC_NT_FAST + 1, OSC_NT_FAST + 0 };

/* first SLOW time base whose history window shows a clean frequency (-1 = none); *f_out = that frequency */
static int AtbProbeSlow(int skip, float *f_out)
{
    for (uint8_t k = 0; k < 6u; k++) {
        int ti = atb_probe_order[k];
        if (ti == skip) continue;
        float pf = SlowProbeFreq(ti);
        if (pf > 0.0f) { *f_out = pf; return ti; }
    }
    return -1;
}

void OscAutoTimebase(void)       /* called every frame after measuring */
{
    uint32_t now;
    float f, cyc, pf;
    int ti;

    if (osc_meas.freq > 0.0f && osc_frame_valid) { atb_last_f = osc_meas.freq; if (osc_meas.freq >= 45.0f) osc_lock_auto = 1; else if (osc_meas.freq < 35.0f) osc_lock_auto = 0; if (!slow_active) { atb_last_fast = (uint8_t)osc_tdiv_i; atb_last_fast_t = HAL_GetTick(); } }
    if (!osc_auto_tb || hold_active || osc_auto_stage || !osc_frame_valid) return;

    now = HAL_GetTick();
    if ((now - atb_t_switch) < 300u) return;                  /* let the new setting produce a frame first */

    /* SLOW saw a fast signal: back to FAST at once */
    if (slow_active && slow_fast_flag) { atb_t_bad = 0; dbg_atb_why = 1; dbg_atb_arg = 0.0f; AtbSwitch(AtbFastFallback(now)); atb_t_hold = now; return; }

    f = osc_meas.freq;
    if (!slow_active && !OscFreqTrusted()) f = 0.0f;           /* FAST: a held F whose picture shows no cycles counts as unreadable */
    if (f > 0.0f) {                                           /* readable */
        atb_t_bad = 0; atb_t_flat = 0; atb_t_unread = 0;
        cyc = f * osc_tdivs_ms[osc_tdiv_i] * (float)OSC_H_DIVS / 1000.0f;
        if (!slow_active) {
            if (f < ATB_FAST_TO_SLOW_HZ)        { dbg_atb_why = 4; dbg_atb_arg = f; AtbSwitch(AtbPickSlow(f)); }
            else if (cyc < 3.0f || cyc > 14.0f) { dbg_atb_why = 5; dbg_atb_arg = f; AtbSwitch(AtbPickFast(f)); }
        } else {
            if (f > ATB_SLOW_TO_FAST_HZ)        { dbg_atb_why = 2; dbg_atb_arg = f; AtbSwitch(AtbPickFast(f)); atb_t_hold = now; }
            else if (cyc < 1.6f || f > (24000.0f / osc_tdivs_ms[osc_tdiv_i]) / 9.0f || (osc_tdiv_i == OSC_NT - 1 && f > 1.2f)) {
                if (atb_cand_n && fabsf(f - atb_cand_f) <= 0.10f * atb_cand_f) atb_cand_n++; else { atb_cand_f = f; atb_cand_n = 1; }
                if (atb_cand_n >= 3u) { atb_cand_n = 0; dbg_atb_why = 3; dbg_atb_arg = f; AtbSwitch(AtbPickSlow(f)); }
            } else atb_cand_n = 0;
        }
        return;
    }

    atb_cand_n = 0;
    /* not readable in the current mode */
    if (!slow_active) {
        /* FAST cannot see signals below ~40 Hz (they look flat in its 4-80 ms window). The history can: ask it, no waiting. */
        if ((now - atb_t_hold) < 1500u) return;                                        /* just came here because of a fast signal: give FAST time to read it; the history is stale */
        if (adc_fast_content) { atb_t_unread = 0; return; }                            /* clearly a fast signal right now */
        if (osc_raw_vpp < 0.30f) { atb_t_unread = 0; }                                 /* silence or a 0.15-0.2 Vpp noise band: stay (the FAST guards use the same 0.30 V) */
        else if (!atb_t_unread) atb_t_unread = now;
        else if ((now - atb_t_unread) >= 700u) {                                       /* 0.7 s of a signal FAST cannot read (101 -> 1 Hz: a straight line): show the history view now,
                                                                                        * the number follows when two periods exist. Readable frames reset the timer, so a 100 Hz signal that FAST reads on every other frame never gets here. */
            atb_t_unread = 0; dbg_atb_why = 12; dbg_atb_arg = 0.0f; AtbSwitch(OSC_NT - 2); return;
        }
        if ((now - atb_t_probe) < 250u) return;
        atb_t_probe = now;
        ti = AtbProbeSlow(-1, &pf);
        if (ti >= 0 && pf < ATB_FAST_TO_SLOW_HZ) { dbg_atb_why = 6; dbg_atb_arg = pf; atb_slowish_n = 0; AtbSwitch(AtbPickSlow(pf)); }
        else if (ti < 0 && SlowSlowish()) {                    /* nothing readable yet, but the history shows a slow moving signal (1 Hz after 0 Hz / after a fast signal):
                                                                * show it NOW in SLOW so the new wave rolls in from the right; the number follows when two periods exist */
            if (++atb_slowish_n >= 2u) { dbg_atb_why = 10; dbg_atb_arg = 0.0f; atb_slowish_n = 0; AtbSwitch(OSC_NT - 2); }
        } else atb_slowish_n = 0;
        return;
    }

    /* SLOW, current base shows nothing clean */
    if (osc_raw_vpp < 0.05f) {                                /* flat window = no signal */
        atb_t_bad = 0;
        /* PARK: after 2.4 s of silence (longer than the 2 s window of 200 ms/div, so the screen is already flat and nothing visibly changes) move to the 200 ms/div base.
         * The next slow signal then rolls in from the right at a base that fits 1-12 Hz, instead of filling a 0.5 s window in half a second and zooming 3 s later. */
        if (!atb_t_flat) atb_t_flat = now;
        else if ((now - atb_t_flat) >= 2400u && osc_tdiv_i < (OSC_NT - 2)) { atb_t_flat = 0; dbg_atb_why = 11; dbg_atb_arg = 0.0f; AtbSwitch(OSC_NT - 2); }
        return;
    }
    atb_t_flat = 0;
    if (SlowRecentFlat()) { atb_t_bad = 0; return; }          /* the signal just stopped (1 Hz -> 0 Hz): stay, let the old trace scroll out of the window by itself - no jump to FAST */
    if (!atb_t_bad) atb_t_bad = now;
    {   /* a window of 5 s at 500 ms/div needs ~2-3 s after a change (3 Hz -> 1 Hz) before two clean new periods exist: wait that long */
        uint32_t lim = 8000u;                                     /* the longest window (5 s) needs ~2.2 periods of the slowest signal we read (0.4 Hz) */
        if ((now - atb_t_bad) >= lim) { atb_t_bad = 0; dbg_atb_why = 9; dbg_atb_arg = 0.0f; AtbSwitch(AtbFastFallback(now)); }   /* SLOW cannot read it: FAST will */
    }
}

/* PC13 short press: switch FAST <-> SLOW by hand and pause AUTO. Long press re-arms AUTO. */
void OscManualToggle(void)
{
    int ti;
    float pf = 0.0f;
    OSC_SET(osc_auto_tb, 0);
    if (!slow_active) {                                       /* FAST -> SLOW */
        ti = AtbProbeSlow(-1, &pf);
        if (ti >= 0)                                  ti = AtbPickSlow(pf);
        else if (atb_last_f > 0.0f && atb_last_f < ATB_SLOW_TO_FAST_HZ) ti = AtbPickSlow(atb_last_f);
        else                                          ti = OSC_NT_FAST + 4;      /* 200 ms/div */
    } else {                                                  /* SLOW -> FAST */
        ti = (atb_last_f >= ATB_SLOW_TO_FAST_HZ) ? AtbPickFast(atb_last_f) : ((atb_last_fast != 0xFFu) ? (int)atb_last_fast : OSC_TDIV_DEF);
    }
    AtbSwitch(ti);
}

void OscAutoRearm(void)
{
    OSC_SET(osc_auto_tb , 1);
    atb_t_switch = HAL_GetTick();
    atb_t_bad    = 0;
    OSC_SET(osc_dirty   , 1);
}
/* AUTO_TB_END ============================================================== */
