/* ext_points.c - Display points just outside the FAST window: the real neighbours of the display filter at the screen edges. */
#include "app/app.h"

/* EXT_POINTS_BEGIN ===========================================================
 * The display points just outside the window (before it / after it), computed from the raw capture of THIS frame (the next capture
 * overwrites adc_raw_buf while the picture is drawn, so it cannot be read later). OscProcessFrame uses them so the first / last `half`
 * points get the same centred filter as the middle (v76). SLOW frames have their own (SlowExtPoint). Nothing is invented. */
#define MV_EXT_MAX 256
static uint16_t mv_ext_lo[MV_EXT_MAX], mv_ext_hi[MV_EXT_MAX];   /* lo[j] = display point -1-j, hi[j] = display point N_SAMPLES+j */
static int      mv_nlo = 0, mv_nhi = 0;                          /* how many of each exist */

int OscExtPoint(int k, float *v)       /* v76: the filter's real neighbours just outside the screen */
{
    if (slow_active) {                                           /* SLOW frames do not refresh mv_ext_*: never use stale points */
        return SlowExtPoint(k, v);                               /* v80: SLOW neighbours of THIS frame (slow_mode.c) */
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
}
/* EXT_POINTS_END ============================================================= */
