#include "app/app.h"

static uint8_t OscFreqTrusted(void)
{
    float cyc;
    if (osc_meas.freq <= 0.0f) return 0;
    if (!adc_f_held) return 1;
    cyc = osc_meas.freq * osc_capture_tdiv * (float)OSC_H_DIVS / 1000.0f;
    return (cyc >= 3.0f && osc_disp_cross < 2) ? 0 : 1;
}
#define HOLD_MAX_MS 500u
#define HOLD_COLLAPSE_MS 2000u
static uint32_t osc_good_t    = 0;
static int      osc_good_tdiv = -1;
uint8_t OscFrameHold(uint32_t now)
{
    if (osc_capture_tdiv != osc_tdivs_ms[osc_tdiv_i]) return 1;
    if (slow_active) { osc_good_t = 0; return 0; }
    if (OscFreqTrusted()) { osc_good_t = now ? now : 1u; osc_good_tdiv = osc_tdiv_i; return 0; }
    if (osc_good_t && osc_good_tdiv == osc_tdiv_i) {
        uint32_t age = now - osc_good_t;
        if (osc_raw_vpp >= 0.30f) { if (age < HOLD_MAX_MS && !adc_fast_content) return 1; }
        else if (age < HOLD_COLLAPSE_MS) return 1;
    }
    return 0;
}

#define ATB_BAD_MS           1500u
#define ATB_FAST_CYC         6.0f

static uint32_t atb_t_switch  = 0;
static uint32_t atb_t_bad     = 0;
static uint32_t atb_t_probe   = 0;
static float    atb_cand_f    = 0.0f;
static uint8_t  atb_cand_n    = 0;
static uint32_t atb_t_unread  = 0;
static uint32_t atb_t_hold    = 0;
static uint32_t atb_t_flat    = 0;
static uint8_t  atb_slowish_n = 0;
static float    atb_last_f    = 0.0f;
static uint8_t  atb_last_fast = 0xFFu;
static uint32_t atb_last_fast_t = 0;
static int AtbFastFallback(uint32_t now)
{ return (atb_last_fast != 0xFFu && (now - atb_last_fast_t) < 15000u) ? (int)atb_last_fast : OSC_TDIV_DEF; }
volatile uint8_t dbg_atb_switches = 0;

static int AtbPickFast(float f)
{
    int ti = 0; float best = 1.0e30f;
    for (int k = 0; k < OSC_NT_FAST; k++) {
        float d = fabsf(f * osc_tdivs_ms[k] * (float)OSC_H_DIVS / 1000.0f - ATB_FAST_CYC);
        if (d < best) { best = d; ti = k; }
    }
    return ti;
}

int AtbPickSlow(float f)
{
    if (f <  0.9f)  return OSC_NT - 1;
    if (f < 12.0f)  return OSC_NT - 2;
    if (f < 45.0f)  return OSC_NT - 4;
    return OSC_NT - 6;
}

volatile uint8_t dbg_atb_why = 0, dbg_atb_from = 0, dbg_atb_to = 0;
volatile float   dbg_atb_arg = 0.0f;
volatile uint32_t dbg_atb_log[8];
volatile uint8_t  dbg_atb_log_i = 0;
volatile float    dbg_atb_arg_log[8];
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

static const uint8_t atb_probe_order[6] = { OSC_NT_FAST + 3, OSC_NT_FAST + 2, OSC_NT_FAST + 4, OSC_NT_FAST + 5, OSC_NT_FAST + 1, OSC_NT_FAST + 0 };

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

void OscAutoTimebase(void)
{
    uint32_t now;
    float f, cyc, pf;
    int ti;

    if (osc_meas.freq > 0.0f && osc_frame_valid) { atb_last_f = osc_meas.freq; if (osc_meas.freq >= 45.0f) osc_lock_auto = 1; else if (osc_meas.freq < 35.0f) osc_lock_auto = 0; if (!slow_active) { atb_last_fast = (uint8_t)osc_tdiv_i; atb_last_fast_t = HAL_GetTick(); } }
    if (!osc_auto_tb || hold_active || osc_auto_stage || !osc_frame_valid) return;

    now = HAL_GetTick();
    if ((now - atb_t_switch) < 300u) return;

    if (slow_active && slow_fast_flag) { atb_t_bad = 0; dbg_atb_why = 1; dbg_atb_arg = 0.0f; AtbSwitch(AtbFastFallback(now)); atb_t_hold = now; return; }

    f = osc_meas.freq;
    if (!slow_active && !OscFreqTrusted()) f = 0.0f;
    if (f > 0.0f) {
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
    if (!slow_active) {
        if ((now - atb_t_hold) < 1500u) return;
        if (adc_fast_content) { atb_t_unread = 0; return; }
        if (osc_raw_vpp < 0.30f) { atb_t_unread = 0; }
        else if (!atb_t_unread) atb_t_unread = now;
        else if ((now - atb_t_unread) >= 700u) {
            atb_t_unread = 0; dbg_atb_why = 12; dbg_atb_arg = 0.0f; AtbSwitch(OSC_NT - 2); return;
        }
        if ((now - atb_t_probe) < 250u) return;
        atb_t_probe = now;
        ti = AtbProbeSlow(-1, &pf);
        if (ti >= 0 && pf < ATB_FAST_TO_SLOW_HZ) { dbg_atb_why = 6; dbg_atb_arg = pf; atb_slowish_n = 0; AtbSwitch(AtbPickSlow(pf)); }
        else if (ti < 0 && SlowSlowish()) {
            if (++atb_slowish_n >= 2u) { dbg_atb_why = 10; dbg_atb_arg = 0.0f; atb_slowish_n = 0; AtbSwitch(OSC_NT - 2); }
        } else atb_slowish_n = 0;
        return;
    }

    if (osc_raw_vpp < 0.05f) {
        atb_t_bad = 0;
        if (!atb_t_flat) atb_t_flat = now;
        else if ((now - atb_t_flat) >= 2400u && osc_tdiv_i < (OSC_NT - 2)) { atb_t_flat = 0; dbg_atb_why = 11; dbg_atb_arg = 0.0f; AtbSwitch(OSC_NT - 2); }
        return;
    }
    atb_t_flat = 0;
    if (SlowRecentFlat()) { atb_t_bad = 0; return; }
    if (!atb_t_bad) atb_t_bad = now;
    {
        uint32_t lim = 8000u;
        if ((now - atb_t_bad) >= lim) { atb_t_bad = 0; dbg_atb_why = 9; dbg_atb_arg = 0.0f; AtbSwitch(AtbFastFallback(now)); }
    }
}

void OscManualToggle(void)
{
    int ti;
    float pf = 0.0f;
    OSC_SET(osc_auto_tb, 0);
    if (!slow_active) {
        ti = AtbProbeSlow(-1, &pf);
        if (ti >= 0)                                  ti = AtbPickSlow(pf);
        else if (atb_last_f > 0.0f && atb_last_f < ATB_SLOW_TO_FAST_HZ) ti = AtbPickSlow(atb_last_f);
        else                                          ti = OSC_NT_FAST + 4;
    } else {
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
