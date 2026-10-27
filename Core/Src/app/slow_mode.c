#include "app/app.h"

#define SLOW_MIN_TDIV_MS  10.0f
#define SLOW_SPREAD_MAX   600u
#define SLOW_ROUGH_FAST   100u
#define SLOW_BURST        16u
#define SLOW_ROUGH_TICKS  100u

volatile uint16_t slow_hist[SLOW_HIST_N];
static volatile uint16_t slow_hist_rg[SLOW_HIST_N];
static volatile uint32_t slow_tick       = 0;
volatile uint32_t slow_last_t     = 0;
static volatile uint16_t slow_last_v     = 0;
volatile uint32_t slow_valid_cnt  = 0;
volatile uint32_t slow_period_us  = 2083;
volatile uint16_t        dbg_slow_spread = 0;
volatile uint16_t        dbg_slow_rough  = 0;
volatile uint32_t        dbg_slow_pts    = 0;
volatile uint16_t        dbg_slow_last   = 0;

#ifndef OSC_SLOW_HOST_TEST
static void SlowAdcPrepare(void)
{
    ADC3->CR2 &= ~ADC_CR2_ADON;
    ADC3->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS);
    ADC3->SQR1 = 0u;
    ADC3->SQR3 = 0u;
    ADC3->SMPR2 = (ADC3->SMPR2 & ~7u) | 1u;
    ADC3->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t d = 0; d < 2000u; d++) { }
}

static volatile uint8_t slow_adc_fail = 0;
static inline uint16_t SlowAdcRead(void)
{
    uint32_t guard = 600u;
    ADC3->SR = 0;
    ADC3->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC3->SR & ADC_SR_EOC)) { if (!--guard) { slow_adc_fail = 1; break; } }
    return (uint16_t)(ADC3->DR & 0x0FFFu);
}
#endif

void OscSlowTick(void)
{
    uint32_t t = slow_tick++;
    uint32_t acc = 0;
    uint16_t mn = 0xFFFF, mx = 0, v, rg;

    if (slow_busy) return;
#ifndef OSC_SLOW_HOST_TEST
    if (!adc_tri_ready) return;
    if (!(ADC3->CR2 & ADC_CR2_ADON)) slow_adc3_dirty = 1;
    if (slow_adc3_dirty) { SlowAdcPrepare(); slow_adc3_dirty = 0; }
    slow_adc_fail = 0;
#endif
    for (uint32_t k = 0; k < SLOW_BURST; k++) {
        for (volatile uint32_t d = (k * 3u) % 7u; d > 0u; d--) { }
        uint16_t a = SlowAdcRead();
#ifndef OSC_SLOW_HOST_TEST
        if (slow_adc_fail) { slow_adc3_dirty = 1; return; }
#endif
        acc += a; if (a < mn) mn = a; if (a > mx) mx = a;
    }
    v  = (uint16_t)(acc / SLOW_BURST);
    rg = (uint16_t)(mx - mn);

    if (slow_valid_cnt == 0) {
        for (uint32_t i = 0; i < SLOW_HIST_N; i++) { slow_hist[i] = v; slow_hist_rg[i] = rg; }
    } else {
        uint32_t gap = t - slow_last_t;
        if (gap > 1u) {
            uint32_t k0 = (gap > SLOW_HIST_N) ? (gap - SLOW_HIST_N + 1u) : 1u;
            for (uint32_t k = k0; k < gap; k++) {
                int32_t iv = (int32_t)slow_last_v + (((int32_t)v - (int32_t)slow_last_v) * (int32_t)k) / (int32_t)gap;
                slow_hist[(slow_last_t + k) & (SLOW_HIST_N - 1u)]    = (uint16_t)iv;
                slow_hist_rg[(slow_last_t + k) & (SLOW_HIST_N - 1u)] = rg;
            }
        }
    }
    slow_hist[t & (SLOW_HIST_N - 1u)]    = v;
    slow_hist_rg[t & (SLOW_HIST_N - 1u)] = rg;
    slow_last_v = v;
    slow_last_t = t;
    slow_valid_cnt++;
    dbg_slow_pts = slow_valid_cnt;
    dbg_slow_last = v;
}

void SlowSync(void)
{
    uint8_t want = (osc_tdivs_ms[osc_tdiv_i] >= SLOW_MIN_TDIV_MS);
    if (want) slow_period_us = (uint32_t)(osc_tdivs_ms[osc_tdiv_i] * (float)OSC_H_DIVS * 1000.0f / (float)N_SAMPLES);
    slow_active = want;
}

static uint16_t SlowBuildPointsAt(uint16_t *out, uint32_t period_us, uint32_t t_view, uint32_t back_us, uint16_t *rough_out)
{
    uint32_t t_end = slow_last_t + 1u;
    uint32_t rs = 0, rn = 0, rj = 0;
    if (period_us < 1000u) {
        for (uint32_t j = 0; j < N_SAMPLES; j++) {
            uint32_t off_us = (N_SAMPLES - 1u - j) * period_us + back_us;
            t_end = t_view;
            uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
            uint32_t t_low = (t_end - 1u) - off_ms - (rem ? 1u : 0u);
            uint32_t a = slow_hist[t_low & (SLOW_HIST_N - 1u)];
            uint32_t b = rem ? slow_hist[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
            out[j] = (uint16_t)((a * rem + b * (1000u - rem)) / 1000u);
        }
        t_end = slow_last_t + 1u;
    } else
    for (uint32_t j = 0; j < N_SAMPLES; j++) {
        uint32_t off_a = ((N_SAMPLES - j) * period_us + 999u) / 1000u;
        t_end = t_view;
        uint32_t off_b = ((N_SAMPLES - 1u - j) * period_us + 999u) / 1000u;
        uint32_t sum = 0, cnt = 0;
        for (uint32_t tk = t_end - off_a; tk != t_end - off_b; tk++) { sum += slow_hist[tk & (SLOW_HIST_N - 1u)]; cnt++; }
        out[j] = (uint16_t)(cnt ? (sum / cnt) : slow_hist[(t_end - 1u) & (SLOW_HIST_N - 1u)]);
    }
    t_end = slow_last_t + 1u;
    for (uint32_t k = 1u; k <= SLOW_ROUGH_TICKS; k++) {
        uint16_t a = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)], b = slow_hist[(t_end - k - 1u) & (SLOW_HIST_N - 1u)];
        rs += slow_hist_rg[(t_end - k) & (SLOW_HIST_N - 1u)]; rn++;
        rj += (a > b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
    }
    *rough_out = (uint16_t)(rj / rn);
    return (uint16_t)(rs / rn);
}

static void SlowEnvAt(uint32_t period_us, uint32_t t_view)
{
    if (period_us < 1000u) return;
    for (uint32_t j = 0; j < N_SAMPLES; j++) {
        uint32_t off_a = ((N_SAMPLES - j) * period_us + 999u) / 1000u;
        uint32_t off_b = ((N_SAMPLES - 1u - j) * period_us + 999u) / 1000u;
        uint16_t lo = 0xFFFF, hi = 0;
        for (uint32_t tk = t_view - off_a; tk != t_view - off_b; tk++) {
            uint16_t v = slow_hist[tk & (SLOW_HIST_N - 1u)];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (lo > hi) lo = hi = slow_hist[(t_view - 1u) & (SLOW_HIST_N - 1u)];
        osc_env_lo_ct[j] = lo; osc_env_hi_ct[j] = hi;
    }
    osc_env_on = 1;
}

static uint16_t SlowBuildPoints(uint16_t *out, uint32_t period_us, uint16_t *rough_out)
{
    return SlowBuildPointsAt(out, period_us, slow_last_t + 1u, 0u, rough_out);
}

#define SLOW_EXT_N 10
static uint16_t slow_ext_lo[SLOW_EXT_N];
static int      slow_nlo = 0;
static void SlowStoreExt(uint32_t period_us, uint32_t t_view, uint32_t back_us)
{
    uint32_t have = (slow_valid_cnt < SLOW_HIST_N) ? slow_valid_cnt : SLOW_HIST_N;
    uint32_t newest = slow_last_t;
    slow_nlo = 0;
    for (uint32_t c = 0; c < SLOW_EXT_N; c++) {
        uint32_t k = N_SAMPLES + c;
        if (period_us < 1000u) {
            uint32_t off_us = k * period_us + back_us;
            uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
            uint32_t t_low = (t_view - 1u) - off_ms - (rem ? 1u : 0u);
            if (newest - t_low + 1u > have) break;
            uint32_t a = slow_hist[t_low & (SLOW_HIST_N - 1u)];
            uint32_t b = rem ? slow_hist[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
            slow_ext_lo[c] = (uint16_t)((a * rem + b * (1000u - rem)) / 1000u);
        } else {
            uint32_t off_a = ((k + 1u) * period_us + 999u) / 1000u;
            uint32_t off_b = (k * period_us + 999u) / 1000u;
            uint32_t sum = 0, cnt = 0;
            if (newest + 1u - (t_view - off_a) > have) break;
            for (uint32_t tk = t_view - off_a; tk != t_view - off_b; tk++) { sum += slow_hist[tk & (SLOW_HIST_N - 1u)]; cnt++; }
            if (!cnt) break;
            slow_ext_lo[c] = (uint16_t)(sum / cnt);
        }
        slow_nlo = (int)c + 1;
    }
}

static float slow_ext_P = 0.0f;
static int SlowExtHi(int c, float *v)
{
    if (slow_ext_P < 2.0f || c < 0 || c >= SLOW_EXT_N) return 0;
    float m = ceilf((float)(c + 1) / slow_ext_P);
    float x = (float)(N_SAMPLES + c) - m * slow_ext_P;
    if (x < 0.0f) return 0;
    int   i0 = (int)x;
    float fr = x - (float)i0;
    if (i0 >= N_SAMPLES - 1) { *v = (float)raw_samples[N_SAMPLES - 1]; return 1; }
    *v = (float)raw_samples[i0] * (1.0f - fr) + (float)raw_samples[i0 + 1] * fr;
    return 1;
}

int SlowExtPoint(int k, float *v)
{
    if (k < 0 && (-k - 1) < slow_nlo) { *v = (float)slow_ext_lo[-k - 1]; return 1; }
    if (k >= N_SAMPLES) return SlowExtHi(k - N_SAMPLES, v);
    return 0;
}

static volatile uint8_t dbg_slow_trig = 0;
static float slow_pv_cur = 0.0f; static uint32_t slow_pv_t = 0u;
static uint8_t SlowFindTrigger(uint32_t period_us, uint32_t *t_view, uint32_t *back_us)
{
    uint32_t w_ms = (uint32_t)(((uint64_t)period_us * N_SAMPLES) / 1000u);
    uint32_t last = slow_last_t;
    float    pre  = 2.0f * (float)period_us / 1000.0f;
    uint32_t reach = w_ms + w_ms / 2u;
    if (!osc_stable || slow_valid_cnt < reach + 8u) return 0;
    uint32_t newest_ok = last - (uint32_t)ceilf((float)w_ms - pre) - 1u;
    uint32_t s0 = last - reach;
    uint16_t mn = 0xFFFF, mx = 0;
    for (uint32_t t = s0; t <= last; t++) {
        uint16_t v = slow_hist[t & (SLOW_HIST_N - 1u)];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    if ((uint32_t)(mx - mn) < 100u) return 0;
    float mid = 0.5f * (float)(mx + mn), hy = 0.2f * (float)(mx - mn);
    int found = 0; float tc = 0.0f;
    uint8_t armed = 0;
    uint16_t pv = slow_hist[s0 & (SLOW_HIST_N - 1u)];
    armed = osc_trig_fall ? ((float)pv > mid) : ((float)pv < mid);
    for (uint32_t t = s0 + 1u; t <= newest_ok; t++) {
        float v = (float)slow_hist[t & (SLOW_HIST_N - 1u)], p = (float)slow_hist[(t - 1u) & (SLOW_HIST_N - 1u)];
        if (osc_trig_fall) {
            if (v > mid + hy) armed = 1;
            else if (armed && p >= mid && v < mid) { tc = (float)(t - 1u) + (p - mid) / (p - v); found = 1; armed = 0; }
        } else {
            if (v < mid - hy) armed = 1;
            else if (armed && p < mid && v >= mid) { tc = (float)(t - 1u) + (mid - p) / (v - p); found = 1; armed = 0; }
        }
    }
    if (!found) return 0;
    {
        float t_new = tc + ((float)w_ms - pre) - (float)period_us / 1000.0f;
        uint32_t tick = (uint32_t)ceilf(t_new);
        if (tick > last) return 0;
        *t_view = tick + 1u;
        *back_us = (uint32_t)(((float)tick - t_new) * 1000.0f);
        if (*back_us > 999u) *back_us = 999u;
    }
    return 1;
}

static uint8_t SlowLooksFast(float f, float vpp_counts, uint16_t spread, uint16_t rough)
{
    if (spread > SLOW_SPREAD_MAX) return 1;
    if (f > 0.0f) return (rough > (uint32_t)(0.004f * f * vpp_counts) + 60u) ? 1u : 0u;
    {   uint32_t thr = (uint32_t)(0.26f * vpp_counts);
        if (thr < SLOW_ROUGH_FAST) thr = SLOW_ROUGH_FAST;
        return (rough > thr) ? 1u : 0u;
    }
}

static float SlowFreqFromPts(const uint16_t *p, float fs, float *vpp_out, int store)
{
    uint16_t mn = 4095, mx = 0;
    float at_[40];
    uint32_t edges = 0;
    int armed = 0;
    for (uint32_t i = 0; i < N_SAMPLES; i++) { if (p[i] < mn) mn = p[i]; if (p[i] > mx) mx = p[i]; }
    float vpp = (float)(mx - mn) * ADC_VREF / ADC_FULL;
    if (vpp_out) *vpp_out = vpp;
    if (vpp < 0.05f) return 0.0f;
    float mid = (float)(mx + mn) * 0.5f, hyst = (float)(mx - mn) * 0.2f;
    for (uint32_t i = 1; i < N_SAMPLES; i++) {
        float prev = (float)p[i - 1], v = (float)p[i];
        if (v < mid - hyst) armed = 1;
        if (armed && prev < mid && v >= mid) {
            if (edges < 40u) at_[edges] = (float)(i - 1) + (mid - prev) / (v - prev);
            edges++;
            armed = 0;
        }
    }
    if (edges < 3u || edges > 40u) return 0.0f;
    {
        uint32_t k0 = edges - 1u;
        float ref = at_[edges - 1u] - at_[edges - 2u];
        if (ref <= 0.0f) return 0.0f;
        while (k0 > 0u) {
            float dt = at_[k0] - at_[k0 - 1u];
            if (dt > 1.15f * ref || dt < 0.85f * ref) break;
            k0--;
        }
        if (edges - k0 < 3u) return 0.0f;
        {
            uint32_t m = edges - k0;
            for (uint32_t k = 0; k < m; k++) at_[k] = at_[k0 + k];
            edges = m;
        }
    }
    float first = at_[0], last = at_[edges - 1u];
    if (last <= first) return 0.0f;
    float mean = (last - first) / (float)(edges - 1u);
    uint32_t bad = 0;
    for (uint32_t k = 1; k < edges; k++) {
        float dt = at_[k] - at_[k - 1u];
        if (dt > 1.3f * mean || dt < 0.7f * mean) bad++;
    }
    if (bad > 0u) return 0.0f;
    float f = (float)(edges - 1u) * fs / (last - first);
    if (!(f >= 0.2f && f <= fs / 8.0f)) return 0.0f;
    if (store) {
        uint32_t i0 = (uint32_t)(first + 1.0f), i1 = (uint32_t)last;
        if (i1 > i0 + 1u && i1 < N_SAMPLES) {
            uint32_t sm = 0;
            float cnt = (float)(i1 - i0 + 1u), mean, sq = 0.0f;
            for (uint32_t i = i0; i <= i1; i++) sm += p[i];
            mean = (float)sm / cnt;
            float sqd = 0.0f;
            for (uint32_t i = i0; i <= i1; i++) { float d = (float)p[i] - mean; sq += d * d;
                float z = (float)p[i] - ADC_DC_ZERO_CT; if (z > 0.0f) sqd += z * z; }
            osc_raw_mean_ct = mean; osc_raw_rms_ct = sqrtf(sq / cnt); osc_raw_dcrms_ct = sqrtf(sqd / cnt);
            osc_raw_min_ct = (float)mn; osc_raw_max_ct = (float)mx; osc_raw_valid = 1;
        }
    }
    return f;
}

uint8_t SlowRecentFlat(void)
{
    uint32_t t_end = slow_last_t + 1u;
    uint16_t mn = 0xFFFF, mx = 0;
    if (slow_valid_cnt < 1500u) return 0;
    for (uint32_t k = 1u; k <= 1500u; k++) {
        uint16_t v = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    return ((float)(mx - mn) * ADC_VREF / ADC_FULL < 0.20f) ? 1u : 0u;
}

uint8_t SlowSlowish(void)
{
    uint32_t t_end = slow_last_t + 1u, rs = 0;
    uint16_t mn = 0xFFFF, mx = 0;
    if (slow_valid_cnt < 1500u) return 0;
    for (uint32_t k = 1u; k <= 1500u; k++) {
        uint16_t v = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    if ((float)(mx - mn) * ADC_VREF / ADC_FULL < 0.30f) return 0;
    for (uint32_t k = 1u; k <= SLOW_ROUGH_TICKS; k++) {
        uint16_t a = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)], b = slow_hist[(t_end - k - 1u) & (SLOW_HIST_N - 1u)];
        rs += (a > b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
    }
    return ((float)(rs / SLOW_ROUGH_TICKS) < 0.08f * (float)(mx - mn) + 12.0f) ? 1u : 0u;
}

float SlowProbeFreq(int ti)
{
    uint16_t pts[N_SAMPLES], rough, spread;
    float vpp = 0.0f, f;
    uint32_t pus = (uint32_t)(osc_tdivs_ms[ti] * (float)OSC_H_DIVS * 1000.0f / (float)N_SAMPLES);
    if (slow_valid_cnt < 1000u) return 0.0f;
    spread = SlowBuildPoints(pts, pus, &rough);
    f = SlowFreqFromPts(pts, 1.0e6f / (float)pus, &vpp, 0);
    if (f <= 0.0f) return 0.0f;
    return SlowLooksFast(f, vpp * (float)ADC_FULL / ADC_VREF, spread, rough) ? 0.0f : f;
}

void SlowCaptureFrame(void)
{
    uint16_t pts[N_SAMPLES], mpts[N_SAMPLES];
    const uint16_t *meas = pts;
    uint16_t vmin = 0xFFFF, vmax = 0, spread, rough;

    new_frame_ready = 0;
    osc_frame_valid = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_meas = (OscMeasure){0};
    strcpy(osc_det, "INVALID");
    slow_ext_P = 0.0f;
    osc_env_on = 0;

    if (slow_valid_cnt == 0) { dbg_adc_ok = 0; osc_dirty = 1; return; }
    uint32_t ext_tv, ext_bk = 0, ext_pus = slow_period_us;
    {   uint32_t tv = 0, bk = 0;
        if (SlowFindTrigger(slow_period_us, &tv, &bk)) { spread = SlowBuildPointsAt(pts, slow_period_us, tv, bk, &rough); dbg_slow_trig = 1; ext_tv = tv; ext_bk = bk;
            uint32_t mb = bk + (N_SAMPLES / 10u - 2u) * slow_period_us; uint16_t r3;
            (void)SlowBuildPointsAt(mpts, slow_period_us, tv - mb / 1000u, mb % 1000u, &r3); meas = mpts; }
        else { ext_tv = slow_last_t + 1u; spread = SlowBuildPointsAt(pts, slow_period_us, ext_tv, 0u, &rough); dbg_slow_trig = 0; }
    }
    for (uint32_t i = 0; i < N_SAMPLES; i++) {
        if (pts[i] < vmin) vmin = pts[i];
        if (pts[i] > vmax) vmax = pts[i];
    }
    {
        uint16_t dpts[N_SAMPLES], r2;
        const uint16_t *src = pts;
        uint32_t nowt = HAL_GetTick();
        float tgt = (float)slow_period_us;
        if (slow_pv_cur < 1.0f || (nowt - slow_pv_t) > 1500u) slow_pv_cur = tgt;
        slow_pv_t = nowt;
        if (slow_pv_cur < tgt)      { slow_pv_cur *= 1.15f; if (slow_pv_cur > tgt) slow_pv_cur = tgt; }
        else if (slow_pv_cur > tgt) { slow_pv_cur /= 1.15f; if (slow_pv_cur < tgt) slow_pv_cur = tgt; }
        if (!dbg_slow_trig && slow_pv_cur != tgt) { (void)SlowBuildPointsAt(dpts, (uint32_t)slow_pv_cur, ext_tv, 0u, &r2); src = dpts; }
        for (uint32_t i = 0; i < N_SAMPLES; i++) raw_samples[i] = (int16_t)src[i];
        ext_pus = (src == dpts) ? (uint32_t)slow_pv_cur : slow_period_us;
        SlowStoreExt(ext_pus, ext_tv, ext_bk);
    }
    osc_fs_eff   = 1000000.0f / (float)slow_period_us;
    osc_raw_valid = 0;
    osc_raw_freq = SlowFreqFromPts(meas, osc_fs_eff, &osc_raw_vpp, 1);
    if (osc_raw_freq <= 0.0f) {
        for (uint32_t k = 0; k < 6u; k++) {
            int ti = (int)(OSC_NT - 1u - k);
            uint16_t mp[N_SAMPLES], mr = 0, msp;
            float mv = 0.0f, mf;
            uint32_t mus;
            if (ti == osc_tdiv_i) continue;
            mus = (uint32_t)(osc_tdivs_ms[ti] * (float)OSC_H_DIVS * 1000.0f / (float)N_SAMPLES);
            msp = SlowBuildPoints(mp, mus, &mr);
            mf  = SlowFreqFromPts(mp, 1.0e6f / (float)mus, &mv, 1);
            if (mf <= 0.0f || mf > 130.0f) continue;
            if (SlowLooksFast(mf, mv * (float)ADC_FULL / ADC_VREF, msp, mr)) { osc_raw_valid = 0; continue; }
            osc_raw_freq = mf; osc_raw_vpp = mv; spread = msp; rough = mr;
            break;
        }
    }
    dbg_slow_spread = spread;
    dbg_slow_rough  = rough;
    slow_fast_flag  = 0;
    if (SlowLooksFast(osc_raw_freq, osc_raw_vpp * (float)ADC_FULL / ADC_VREF, spread, rough)) {
        slow_fast_flag = 1;
        osc_raw_freq   = 0.0f;
        osc_raw_valid  = 0;
    }
    if (osc_raw_freq > 0.0f && ext_pus > 0u) slow_ext_P = (1.0e6f / (float)ext_pus) / osc_raw_freq;
    slow_trig_on = dbg_slow_trig;
    if (slow_ext_P > 0.0f && slow_ext_P < 12.0f) SlowEnvAt(ext_pus, ext_tv);
    osc_capture_tdiv = osc_tdivs_ms[osc_tdiv_i];
    dbg_adc_ok       = 1;
    dbg_adc_min      = vmin;
    dbg_adc_max      = vmax;
    dbg_adc_frames++;
    osc_frame_valid  = 1;
    new_frame_ready  = 1;
    osc_dirty        = 1;
}
