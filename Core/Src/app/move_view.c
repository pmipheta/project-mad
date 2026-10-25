/* move_view.c - Data behind MOVE/ZOOM of a frozen capture (points outside the window, raw copy, panned view). */
#include "app/app.h"

#if ADC_INPUT_ENABLE

/* MOVE_DATA_BEGIN ============================================================
 * The data that X move pans over. FAST: the display points just outside the window (before it / after it), computed from the raw capture of THIS frame
 * (the next capture overwrites adc_raw_buf while the picture is drawn, so it cannot be read later). SLOW: a copy of the 8 s history taken at the moment of
 * the freeze (the live history keeps running). Nothing is invented: outside these ranges X move stops. */
#define MV_EXT_MAX 256
static uint16_t mv_ext_lo[MV_EXT_MAX], mv_ext_hi[MV_EXT_MAX];   /* lo[j] = display point -1-j, hi[j] = display point N_SAMPLES+j */
static int      mv_nlo = 0, mv_nhi = 0;                          /* how many of each exist */
static uint16_t mv_snap[SLOW_HIST_N];
static uint32_t mv_snap_end = 0, mv_snap_period = 1000u;
static uint8_t  mv_slow_mode = 0;
/* v57 ZOOM data: a copy of the raw capture region behind the display window (the next DMA capture overwrites adc_raw_buf), taken together with the ext points. */
static uint16_t mv_raw[ADC_RAW_MAX];
static uint32_t mv_raw_off = 0, mv_raw_n = 0, mv_z_w = 1;     /* mv_raw[j] = adc_raw_buf[mv_raw_off + j]; mv_raw_n = 0: no raw data */
static float    mv_z_step = 1.0f;
static uint32_t mv_z_start = 0;

int OscExtPoint(int k, float *v)       /* v76: the filter's real neighbours just outside the screen (FAST only) */
{
    if (slow_active) {                                           /* SLOW frames do not refresh mv_ext_*: never use stale points */
        return SlowExtPoint(k, v);                               /* v80: SLOW neighbours of THIS frame (defined in SLOW) */
    }
    if (k < 0 && (-k - 1) < mv_nlo)            { *v = (float)mv_ext_lo[-k - 1]; return 1; }
    if (k >= N_SAMPLES && (k - N_SAMPLES) < mv_nhi) { *v = (float)mv_ext_hi[k - N_SAMPLES]; return 1; }
    return 0;
}

FAST_O2 void MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n)
{
    int k, cnt;
    mv_nlo = 0; mv_nhi = 0;
    for (cnt = 0, k = -1; cnt < MV_EXT_MAX; cnt++, k--) {
        int32_t c = (int32_t)start + (int32_t)floorf((float)k * step + 0.5f);
        int32_t lo = c - (int32_t)((w - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0 || lo + (int32_t)w > (int32_t)n) break;
        for (uint32_t q = 0; q < w; q++) acc += adc_raw_buf[(uint32_t)lo + q];
        mv_ext_lo[cnt] = (uint16_t)(acc / w);
        mv_nlo = cnt + 1;
    }
    for (cnt = 0, k = N_SAMPLES; cnt < MV_EXT_MAX; cnt++, k++) {
        int32_t c = (int32_t)start + (int32_t)((float)k * step + 0.5f);
        int32_t lo = c - (int32_t)((w - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0 || lo + (int32_t)w > (int32_t)n) break;
        for (uint32_t q = 0; q < w; q++) acc += adc_raw_buf[(uint32_t)lo + q];
        mv_ext_hi[cnt] = (uint16_t)(acc / w);
        mv_nhi = cnt + 1;
    }
    {   /* v57: keep the raw samples of everything the pan can show (FAST_O2 memcpy of at most ADC_RAW_MAX samples) */
        float lo_f = (float)start - ((float)mv_nlo + 1.0f) * step - (float)w;
        float hi_f = (float)start + ((float)(N_SAMPLES + mv_nhi) + 1.0f) * step + 2.0f * (float)w + 2.0f;
        uint32_t a = (lo_f > 0.0f) ? (uint32_t)lo_f : 0u, b = (hi_f < (float)n) ? (uint32_t)hi_f : n;
        if (b > ADC_RAW_MAX) b = ADC_RAW_MAX;
        if (b > a) { memcpy(mv_raw, &adc_raw_buf[a], (size_t)(b - a) * sizeof(uint16_t)); mv_raw_off = a; mv_raw_n = b - a; }
        else mv_raw_n = 0;
        mv_z_start = start; mv_z_step = step; mv_z_w = w;
    }
}

void MvFreezeCapture(void)            /* called once, at the moment of the freeze: fixes the pan range */
{
    mv_x_min = 0; mv_x_max = 0; mv_slow_mode = 0;
    if (slow_active) {
        uint32_t valid = slow_valid_cnt;
        if (valid > SLOW_HIST_N) valid = SLOW_HIST_N;
        if (valid < 64u) return;
        for (uint32_t i = 0; i < SLOW_HIST_N; i++) mv_snap[i] = slow_hist[i];
        mv_snap_end    = slow_last_t + 1u;
        mv_snap_period = slow_period_us ? slow_period_us : 1000u;
        {   uint32_t win   = (uint32_t)(((uint64_t)N_SAMPLES * mv_snap_period + 999u) / 1000u) + 2u;     /* ticks the window needs */
            uint32_t avail = valid - 4u;                                                                  /* ticks of real history behind the newest */
            if (avail > win) {
                uint32_t pts = (uint32_t)(((uint64_t)(avail - win) * 1000u) / mv_snap_period);
                mv_x_min = -(int)((pts > 30000u) ? 30000u : pts);
            }
        }
        mv_slow_mode = 1;
    } else {
        mv_x_min = -mv_nlo;
        mv_x_max = mv_nhi;
    }
}

static uint16_t MvSlowPoint(int j, int pan)  /* display point j of the SLOW window moved |pan| points earlier (same averaging as SlowBuildPointsAt) */
{
    uint32_t period = mv_snap_period;
    uint32_t back_us = (uint32_t)(-pan) * period;
    uint32_t t_end = mv_snap_end;
    if (period < 1000u) {
        uint32_t off_us = (N_SAMPLES - 1u - (uint32_t)j) * period + back_us;
        uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
        uint32_t t_low = (t_end - 1u) - off_ms - (rem ? 1u : 0u);
        uint32_t a = mv_snap[t_low & (SLOW_HIST_N - 1u)];
        uint32_t b = rem ? mv_snap[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
        return (uint16_t)((a * rem + b * (1000u - rem)) / 1000u);
    } else {
        uint32_t tb = back_us / 1000u;
        uint32_t off_a = (((N_SAMPLES - (uint32_t)j) * period + 999u) / 1000u) + tb;
        uint32_t off_b = (((N_SAMPLES - 1u - (uint32_t)j) * period + 999u) / 1000u) + tb;
        uint32_t sum = 0, cnt = 0;
        for (uint32_t tk = t_end - off_a; tk != t_end - off_b; tk++) { sum += mv_snap[tk & (SLOW_HIST_N - 1u)]; cnt++; }
        return (uint16_t)(cnt ? (sum / cnt) : mv_snap[(t_end - off_b) & (SLOW_HIST_N - 1u)]);
    }
}

/* v57: time zoom. Output pixel i shows display position u = (1920 + (i - 120) * 16 / N + mv_x_q) / 16 of the frozen picture (N = mv_zoom_x).
 * FAST: raw position r = start + u * step. Pixels are step/N raw samples apart: >= 1 -> boxcar of round(w/N) raw samples (real detail),
 *       < 1 (or no raw copy) -> linear interpolation between neighbours. SLOW: the frozen 1 ms history, same rule (interpolate below 1 ms, average above).
 * The display filter shrinks with the zoom (window / N) so the extra detail is not smeared away again. */
static float MvZoomCount(int32_t U)
{
    const int N = mv_zoom_x;
    if (mv_slow_mode) {
        uint32_t period = mv_snap_period, t_end = mv_snap_end;
        int64_t off64 = ((int64_t)((N_SAMPLES - 1) * 16) - (int64_t)U) * (int64_t)period / 16;
        uint32_t off_us = (off64 > 0) ? (uint32_t)off64 : 0u, span = period / (uint32_t)N;
        if (span < 1000u) {
            uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
            uint32_t t_low = (t_end - 1u) - off_ms - (rem ? 1u : 0u);
            uint32_t a = mv_snap[t_low & (SLOW_HIST_N - 1u)];
            uint32_t b = rem ? mv_snap[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
            return (float)((a * rem + b * (1000u - rem)) / 1000u);
        } else {
            uint32_t off_a = (off_us + span + 999u) / 1000u, off_b = (off_us + 999u) / 1000u, sum = 0, c = 0;
            for (uint32_t tk = t_end - off_a; tk != t_end - off_b; tk++) { sum += mv_snap[tk & (SLOW_HIST_N - 1u)]; c++; }
            return (float)(c ? (sum / c) : mv_snap[(t_end - off_b) & (SLOW_HIST_N - 1u)]);
        }
    }
    if (mv_raw_n > 0) {
        float r = (float)mv_z_start + ((float)U / 16.0f) * mv_z_step - (float)mv_raw_off;     /* index inside mv_raw */
        float sp = mv_z_step / (float)N;
        int last = (int)mv_raw_n - 1;
        if (sp >= 1.0f) {
            int w2 = ((int)mv_z_w + N / 2) / N, lo; if (w2 < 1) w2 = 1;
            lo = (int)floorf(r + 0.5f) - (w2 - 1) / 2;
            if (lo + w2 > last + 1) lo = last + 1 - w2;
            if (lo < 0) lo = 0;
            if (lo + w2 > last + 1) w2 = last + 1 - lo;
            { uint32_t acc = 0; for (int q = 0; q < w2; q++) acc += mv_raw[lo + q]; return (float)acc / (float)w2; }
        } else {
            int i0 = (int)floorf(r); float fr = r - (float)i0; int i1 = i0 + 1;
            i0 = OscClampI(i0, 0, last); i1 = OscClampI(i1, 0, last);
            return (float)mv_raw[i0] + ((float)mv_raw[i1] - (float)mv_raw[i0]) * fr;
        }
    }
    {   /* no stored samples: straight lines between the 240 display points */
        float u = (float)U / 16.0f; int i0; float fr;
        if (u < 0.0f) u = 0.0f;
        if (u > (float)(N_SAMPLES - 1)) u = (float)(N_SAMPLES - 1);
        i0 = (int)floorf(u); fr = u - (float)i0;
        return (float)raw_samples[i0] + (float)(raw_samples[OscClampI(i0 + 1, 0, N_SAMPLES - 1)] - raw_samples[i0]) * fr;
    }
}

static void MvBuildZoom(void)
{
    float cnt[N_SAMPLES];
    const float kv = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;   /* v84 */
    int win = OscFiltWin(), half, zw;   /* v82: same window as OscProcessFrame */
    zw = win / mv_zoom_x; if (zw < 1) zw = 1; if ((zw % 2) == 0) zw++;
    half = zw / 2;
    for (int i = 0; i < N_SAMPLES; i++) cnt[i] = MvZoomCount(1920 + ((i - N_SAMPLES / 2) * 16) / mv_zoom_x + mv_x_q);
    for (int i = 0; i < N_SAMPLES; i++) {
        float acc = 0.0f;
        for (int j = -half; j <= half; j++) acc += cnt[OscClampI(i + j, 0, N_SAMPLES - 1)];
        mv_view_v[i] = (acc / (float)zw - osc_offset_ct) * kv;
        if (!osc_calib && mv_view_v[i] < 0.0f) mv_view_v[i] = 0.0f;   /* v90: DC */
    }
    mv_view_on = 1;
}

void MvBuildView(void)                /* mv_view_v[] = the frozen capture panned by mv_x_pts (same calibration + filter as OscProcessFrame) */
{
    float cnt[N_SAMPLES];
    if (mv_zoom_x > 1) { MvBuildZoom(); return; }       /* v57 */
    const float kv = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;   /* v84 */
    int win = OscFiltWin(), half;       /* v82: same window as OscProcessFrame */
    half = win / 2;
    for (int i = 0; i < N_SAMPLES; i++) {
        int k = i + mv_x_pts;
        if (mv_slow_mode)  cnt[i] = (float)MvSlowPoint(i, mv_x_pts);
        else if (k < 0)    cnt[i] = (float)mv_ext_lo[OscClampI(-k - 1, 0, MV_EXT_MAX - 1)];
        else if (k < N_SAMPLES) cnt[i] = (float)raw_samples[k];
        else               cnt[i] = (float)mv_ext_hi[OscClampI(k - N_SAMPLES, 0, MV_EXT_MAX - 1)];
    }
    for (int i = 0; i < N_SAMPLES; i++) {
        float acc = 0.0f;
        for (int j = -half; j <= half; j++) acc += cnt[OscClampI(i + j, 0, N_SAMPLES - 1)];
        mv_view_v[i] = (acc / (float)win - osc_offset_ct) * kv;
        if (!osc_calib && mv_view_v[i] < 0.0f) mv_view_v[i] = 0.0f;   /* v90: DC */
    }
    mv_view_on = (mv_x_pts != 0);
}
/* MOVE_DATA_END ============================================================= */

#else  /* !ADC_INPUT_ENABLE */
void MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n) { (void)start; (void)step; (void)w; (void)n; }
int OscExtPoint(int k, float *v) { (void)k; (void)v; return 0; }
void MvFreezeCapture(void) { mv_x_min = mv_x_max = 0; }
void MvBuildView(void) { mv_view_on = 0; }
void OscSlowTick(void) { }
float SlowProbeFreq(int ti) { (void)ti; return 0.0f; }
uint8_t SlowSlowish(void) { return 0u; }
#endif /* ADC_INPUT_ENABLE */
