#include "app/app.h"

#define MV_EXT_MAX 256
static uint16_t mv_ext_lo[MV_EXT_MAX], mv_ext_hi[MV_EXT_MAX];
static int      mv_nlo = 0, mv_nhi = 0;

int OscExtPoint(int k, float *v)
{
    if (slow_active) {
        return SlowExtPoint(k, v);
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
