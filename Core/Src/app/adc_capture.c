#include "app/app.h"

ALIGN_32BYTES(uint16_t adc_raw_buf[ADC_RAW_MAX]);
static volatile uint8_t  adc_done = 0, adc_error = 0;
static volatile uint8_t  adc_async = 0;

volatile float    dbg_adc_rate   = 0.0f;
volatile uint8_t  dbg_adc_lvl    = 0;
volatile uint32_t dbg_adc_decim  = 1;
volatile uint32_t dbg_adc_n      = 0;
volatile uint32_t dbg_adc_us     = 0;
volatile uint16_t dbg_adc_min    = 0;
volatile uint16_t dbg_adc_max    = 0;
volatile uint32_t dbg_adc_frames = 0;
volatile uint8_t  dbg_adc_ok     = 0;
volatile float    dbg_adc_step   = 1.0f;
volatile uint32_t dbg_adc_start  = 0;

uint32_t AdcElapsedUs(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start) / (HAL_RCC_GetHCLKFreq() / 1000000UL);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_done = 1;
        if (adc_async) {
            ADC1->CR2 &= ~ADC_CR2_CONT;
            ADC1->CR1 &= ~ADC_CR1_OVRIE;
        }
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 && !(adc_done && hadc->ErrorCode == HAL_ADC_ERROR_OVR))
        adc_error = 1;
}

static ADC_HandleTypeDef hadc2, hadc3;
volatile uint8_t slow_busy = 0;
volatile uint8_t slow_adc3_dirty = 1;
uint8_t adc_tri_ready = 0;
static uint8_t adc_tri_on    = 0;
volatile uint32_t dbg_tri[6];

static uint8_t AdcTripleInit(void)
{
    ADC_ChannelConfTypeDef c = {0};
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    hadc2.Instance = ADC2; hadc2.Init = hadc1.Init; hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc3.Instance = ADC3; hadc3.Init = hadc1.Init; hadc3.Init.DMAContinuousRequests = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK || HAL_ADC_Init(&hadc3) != HAL_OK) return 0;
    c.Channel = ADC_CHANNEL_0; c.Rank = ADC_REGULAR_RANK_1; c.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &c) != HAL_OK || HAL_ADC_ConfigChannel(&hadc3, &c) != HAL_OK) return 0;
    adc_tri_ready = 1;
    return 1;
}

static uint8_t AdcCaptureTriple(uint32_t n, uint32_t *t_us)
{
    uint32_t t0, to, words = (n + 1u) / 2u;
    uint8_t ok;
    if (n == 0 || n > ADC_RAW_MAX || !t_us) return 0;
    if (!adc_tri_ready && !AdcTripleInit()) return 0;
    uint32_t bytes = (words * 4u + 31u) & ~31u;
    adc_done = adc_error = 0;
    ADC3->CR2 &= ~ADC_CR2_ADON;
    ADC3->CR2 |= ADC_CR2_CONT; ADC2->CR2 |= ADC_CR2_CONT;
    ADC3->SMPR2 &= ~7u;
    ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~(ADC_CCR_MULTI | ADC_CCR_DMA | ADC_CCR_DDS | ADC_CCR_DELAY))
                       | ADC_TRIPLEMODE_INTERL | ADC_DMAACCESSMODE_2 | ADC_TWOSAMPLINGDELAY_5CYCLES;
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE)) | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1;
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start(&hadc2);
    t0 = DWT->CYCCNT;
    dbg_tri[0] = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_raw_buf, words);
    if (dbg_tri[0] != HAL_OK) {
        ok = 0;
    } else {
        to = HAL_GetTick();
        while (!adc_done && !adc_error && (HAL_GetTick() - to) < 200u) { }
        ok = adc_done && !adc_error;
    }
    dbg_tri[2] = adc_done; dbg_tri[3] = adc_error; dbg_tri[4] = hadc1.ErrorCode;
    dbg_tri[1] = HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    dbg_tri[5] = ok;
    HAL_ADC_Stop(&hadc2);
    HAL_ADC_Stop(&hadc3);
    slow_adc3_dirty = 1;
    *t_us = AdcElapsedUs(t0);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    ADC123_COMMON->CCR &= ~(ADC_CCR_MULTI | ADC_CCR_DMA | ADC_CCR_DDS | ADC_CCR_DELAY);
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE)) | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0;
    return ok;
}

static uint8_t AdcCaptureRawInner(uint32_t n, uint32_t *t_us)
{
    uint32_t t0, to;
    if (adc_tri_on) return AdcCaptureTriple(n, t_us);
    if (n == 0 || n > ADC_RAW_MAX || n > 65535u || !t_us) return 0;
    uint32_t bytes = (n * 2u + 31u) & ~31u;
    adc_done = adc_error = 0;
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    t0 = DWT->CYCCNT;
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_buf, n) != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc1);
        return 0;
    }
    to = HAL_GetTick();
    while (!adc_done && !adc_error && (HAL_GetTick() - to) < 200u) { }
    uint8_t ok = adc_done && !adc_error;
    if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) ok = 0;
    *t_us = AdcElapsedUs(t0);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    return ok;
}

static uint8_t AdcCaptureRaw(uint32_t n, uint32_t *t_us)
{
    uint8_t ok;
    slow_busy = 1;
    ok = AdcCaptureRawInner(n, t_us);
    slow_busy = 0;
    return ok;
}

static uint8_t AdcCaptureStart(uint32_t n, uint32_t *t0)
{
    if (adc_tri_on || n == 0 || n > ADC_RAW_MAX || n > 65535u || !t0) return 0;
    uint32_t bytes = (n * 2u + 31u) & ~31u;
    adc_done = adc_error = 0;
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    *t0 = DWT->CYCCNT;
    ADC1->CR2 |= ADC_CR2_CONT;
    slow_busy = 1;
    adc_async = 1;
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_buf, n) != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc1);
        ADC1->CR2 |= ADC_CR2_CONT;
        adc_async = 0; slow_busy = 0;
        return 0;
    }
    return 1;
}

static uint8_t AdcCaptureCollect(uint32_t n, uint32_t t0, uint32_t *t_us)
{
    uint32_t bytes = (n * 2u + 31u) & ~31u, to = HAL_GetTick();
    uint8_t ok;
    while (!adc_done && !adc_error && (HAL_GetTick() - to) < 200u) { }
    ok = adc_done && !adc_error;
    if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) ok = 0;
    ADC1->CR2 |= ADC_CR2_CONT;
    adc_async = 0; slow_busy = 0;
    if (t_us) *t_us = AdcElapsedUs(t0);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    return ok;
}

void AdcInputInit(void)
{
    uint32_t t_us = 0;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *((volatile uint32_t *)0xE0001FB0) = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    adc_done = 0;
    if (AdcCaptureRaw(2000u, &t_us) && t_us > 0u) {
        dbg_adc_rate = 2000.0f * 1000000.0f / (float)t_us;
        dbg_adc_ok   = 1;
    }
    (void)AdcTripleInit();
}

#define ADC_RATE_LEVELS  5
#define ADC_MEAS_LEVELS  3
static const struct { uint32_t div; uint32_t smp; float cyc; float mult; } adc_rate_tab[ADC_RATE_LEVELS] = {
    { 2u, ADC_SAMPLETIME_3CYCLES,  15.0f, 1.0f },
    { 4u, ADC_SAMPLETIME_3CYCLES,  15.0f, 1.0f },
    { 4u, ADC_SAMPLETIME_15CYCLES, 27.0f, 1.0f },
    { 4u, ADC_SAMPLETIME_28CYCLES, 40.0f, 1.0f },
    { 2u, ADC_SAMPLETIME_3CYCLES,  15.0f, 3.0f },
};

static float AdcRateHz(int lvl)
{
    return (float)HAL_RCC_GetPCLK2Freq() / (float)adc_rate_tab[lvl].div / adc_rate_tab[lvl].cyc * adc_rate_tab[lvl].mult;
}

static uint8_t AdcSetRateLevel(int lvl)
{
    ADC_ChannelConfTypeDef c = {0};
    if (lvl < 0 || lvl >= ADC_RATE_LEVELS) return 0;
    ADC3->CR2 &= ~ADC_CR2_ADON;
    slow_adc3_dirty = 1;
    ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~ADC_CCR_ADCPRE)
                       | ((adc_rate_tab[lvl].div == 2u) ? ADC_CLOCK_SYNC_PCLK_DIV2 : ADC_CLOCK_SYNC_PCLK_DIV4);
    c.Channel = ADC_CHANNEL_0;
    c.Rank = ADC_REGULAR_RANK_1;
    c.SamplingTime = adc_rate_tab[lvl].smp;
    if (HAL_ADC_ConfigChannel(&hadc1, &c) != HAL_OK) return 0;
    adc_tri_on = (adc_rate_tab[lvl].mult > 1.5f) ? 1u : 0u;
    dbg_adc_rate = AdcRateHz(lvl);
    return 1;
}

static float FAST_O2 AdcRawFrequency(const uint16_t *buf, uint32_t n, float fs)
{
    uint16_t mn = 4095, mx = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (buf[i] < mn) mn = buf[i];
        if (buf[i] > mx) mx = buf[i];
    }
    osc_raw_vpp = (mx - mn) * ADC_VREF / ADC_FULL;
    adc_fast_content = 0;
    adc_rough_ratio = 0.0f;
    osc_raw_valid = 0;
    if (osc_raw_vpp < 0.05f) return 0.0f;
    if (adc_tri_on && osc_raw_vpp < 0.30f) return 0.0f;
    {
        uint32_t acc = 0;
        for (uint32_t i = 1; i < n; i++) acc += (buf[i] > buf[i - 1]) ? (uint32_t)(buf[i] - buf[i - 1]) : (uint32_t)(buf[i - 1] - buf[i]);
        adc_rough_ratio = (float)acc / (float)(n - 1u) / (float)(mx - mn);
        if (osc_raw_vpp >= 0.30f && adc_rough_ratio > 0.25f && (float)acc / (float)(n - 1u) > 120.0f) adc_fast_content = 1;
    }
    float mid = (mx + mn) * 0.5f;
    for (int pass = 0; pass < (adc_tri_on ? 2 : 1); pass++) {
        float hyst = (mx - mn) * (pass ? 0.08f : 0.2f);
        float first = 0.0f, last = 0.0f;
        uint32_t edges = 0;
        int armed = 0;
        for (uint32_t i = 1; i < n; i++) {
            float prev = buf[i - 1], v = buf[i];
            if (v < mid - hyst) armed = 1;
            if (armed && prev < mid && v >= mid) {
                float at = (float)(i - 1) + (mid - prev) / (v - prev);
                if (!edges) first = at;
                last = at;
                edges++;
                armed = 0;
            }
        }
        if (edges < 3 || last <= first) continue;
        float f = (edges - 1) * fs / (last - first);
        float fmax = adc_tri_on ? fs * 0.45f : fs / 5.0f;
        if (f > fmax) { if (osc_raw_vpp >= 0.30f) adc_too_fast = 1; continue; }
        if (f < 39.9f) continue;
        if (adc_rough_ratio > 3.0f * 0.64f * sinf(3.14159265f * f / fs) + 0.10f + 12.0f / (float)(mx - mn)) continue;
        if (adc_tri_on && f < fs / 9.0f && (adc_fast_content || (adc_over_t && (HAL_GetTick() - adc_over_t) < 3000u))) continue;
        {
            float mean = (last - first) / (float)(edges - 1), lastat = 0.0f;
            uint32_t bad = 0, cnt = 0, good = 0;
            float gsum = 0.0f;
            armed = 0;
            for (uint32_t i = 1; i < n; i++) {
                float prev = buf[i - 1], v = buf[i];
                if (v < mid - hyst) armed = 1;
                if (armed && prev < mid && v >= mid) {
                    float at = (float)(i - 1) + (mid - prev) / (v - prev);
                    if (cnt) { float dt = at - lastat; if (dt > 1.3f * mean || dt < 0.7f * mean) bad++; else { gsum += dt; good++; } }
                    lastat = at; cnt++; armed = 0;
                }
            }
            if (bad * 20u > cnt || good < 2u) continue;
            f = fs * (float)good / gsum;
        }
        {
            uint32_t i0 = (uint32_t)(first + 1.0f), i1 = (uint32_t)last;
            if (i1 > i0 + 1u && i1 < n) {
                uint32_t sm = 0;
                float cnt = (float)(i1 - i0 + 1u), mean, sq = 0.0f;
                for (uint32_t i = i0; i <= i1; i++) sm += buf[i];
                mean = (float)sm / cnt;
                float sqd = 0.0f;
                for (uint32_t i = i0; i <= i1; i++) { float d = (float)buf[i] - mean; sq += d * d;
                    float z = (float)buf[i] - ADC_DC_ZERO_CT; if (z > 0.0f) sqd += z * z; }
                osc_raw_mean_ct = mean; osc_raw_rms_ct = sqrtf(sq / cnt); osc_raw_dcrms_ct = sqrtf(sqd / cnt);
                osc_raw_min_ct = (float)mn; osc_raw_max_ct = (float)mx; osc_raw_valid = 1;
                {
                    uint32_t k = (uint32_t)(fs / f / 32.0f);
                    if (k > 64u) k = 64u;
                    if (k >= 3u && i1 > i0 + k) {
                        uint32_t run = 0;
                        float smn = 1e9f, smx = -1e9f;
                        for (uint32_t i = i0; i < i0 + k; i++) run += buf[i];
                        for (uint32_t i = i0 + k; ; i++) {
                            float a = (float)run / (float)k;
                            if (a < smn) smn = a;
                            if (a > smx) smx = a;
                            if (i > i1) break;
                            run += buf[i]; run -= buf[i - k];
                        }
                        osc_raw_min_ct = smn; osc_raw_max_ct = smx;
                    }
                }
            }
        }
        return f;
    }
    return 0.0f;
}

static uint8_t AdcPlanFrame(float fs_want, int *lvl_o, float *step_o)
{
    int lvl = 1;
    float step = AdcRateHz(1) / fs_want;
    uint8_t fast_meas = 0;
    if (step < 1.0f) {
        lvl = 0; step = AdcRateHz(0) / fs_want;
        if (step < 1.0f) { lvl = 4; step = AdcRateHz(4) / fs_want; }
    } else if ((N_SAMPLES + ADC_PRE_POINTS + 1) * step + 3 > ADC_RAW_MAX) {
        lvl = 2; step = AdcRateHz(2) / fs_want;
        if ((N_SAMPLES + ADC_PRE_POINTS + 1) * step + 3 > ADC_RAW_MAX) { lvl = 3; step = AdcRateHz(3) / fs_want; }
    }
    {   uint32_t t0n = HAL_GetTick();
        if (adc_last_good_f > 0.0f && (t0n - adc_last_good_t) < 1200u && (t0n - adc_legacy_t) < 1000u && lvl != 4) {
            float R = AdcRateHz(lvl), f0 = adc_last_good_f;
            if (f0 <= R * (1.0f / 16.0f)) {
                uint32_t pp0 = (uint32_t)(1.3f * fs_want / f0) + 12u;
                float n0;
                if (pp0 > ADC_PRE_POINTS) pp0 = ADC_PRE_POINTS;
                if (pp0 < 40u) pp0 = 40u;
                n0 = ((float)(N_SAMPLES + pp0) * step + step + 2.0f);
                if (n0 * f0 / R >= 3.4f) fast_meas = 1;
            }
        }
    }
    *lvl_o = lvl; *step_o = step;
    return fast_meas;
}

static uint32_t AdcPreFor(float fs_want, float f)
{
    uint32_t pre = ADC_PRE_POINTS;
    if (f > 0.0f && fs_want > 0.0f) {
        uint32_t pp = (uint32_t)(1.3f * fs_want / f) + 12u;
        if (pp < pre) pre = pp;
        if (pre < 40u) pre = 40u;
    }
    return pre;
}

void AdcAbortPending(void)
{
    if (!adc_pend) return;
    HAL_ADC_Stop_DMA(&hadc1);
    ADC1->CR2 |= ADC_CR2_CONT;
    adc_async = 0; slow_busy = 0; adc_pend = 0;
}

void AdcPrestart(void)
{
    float fs_want, step;
    int lvl;
    uint32_t w, pre, n;
    if (adc_pend || osc_tdiv_i >= OSC_NT_FAST || !osc_frame_valid) return;
    fs_want = (float)N_SAMPLES / (osc_tdivs_ms[osc_tdiv_i] * OSC_H_DIVS / 1000.0f);
    if (!AdcPlanFrame(fs_want, &lvl, &step) || lvl == 4) return;
    if (!AdcSetRateLevel(lvl)) return;
    step = dbg_adc_rate / fs_want;
    if (step < 1.0f) step = 1.0f;
    w = (uint32_t)(step + 0.5f);
    pre = AdcPreFor(fs_want, adc_last_good_f);
    n = (uint32_t)((N_SAMPLES + pre) * step) + w + 2u;
    if (n > ADC_RAW_MAX) return;
    if (!AdcCaptureStart(n, &adc_pd_t0)) return;
    adc_pd_tdiv = osc_tdiv_i; adc_pd_lvl = lvl; adc_pd_n = n; adc_pd_w = w; adc_pd_pre = pre; adc_pd_step = step; adc_pd_tick = HAL_GetTick();
    adc_pend = 1;
    dbg_pipe_cnt++;
}

#define PROF_START()   (DWT->CYCCNT)
#define PROF_END(i,t0)  (prof_us[i] = AdcElapsedUs(t0))
void FAST_O2 AdcCaptureFrame(void)
{
    float fs_want = (float)N_SAMPLES / (osc_tdivs_ms[osc_tdiv_i] * OSC_H_DIVS / 1000.0f);
    uint32_t n, t_us = 0;
    uint16_t vmin = 0xFFFF, vmax = 0;
    new_frame_ready = 0;
    osc_frame_valid = 0;
    adc_f_held = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_raw_valid = 0;
    OSC_LOCK(); osc_meas = (OscMeasure){0}; OSC_UNLOCK();
    strcpy(osc_det, "INVALID");

    int lvl; float step;
    uint8_t fast_meas = AdcPlanFrame(fs_want, &lvl, &step);

    uint8_t use_pend = 0;
    if (adc_pend) {
        if (fast_meas && adc_pd_tdiv == osc_tdiv_i && adc_pd_lvl == lvl) use_pend = 1;
        else AdcAbortPending();
    }

    uint32_t pB0 = PROF_START(); prof_us[5] = 0;
    if (!fast_meas) {
    if (!AdcSetRateLevel(adc_meas_seq[adc_meas_lvl]) || !AdcCaptureRaw(adc_measure_n, &t_us)) goto failed;
    adc_too_fast = 0;
    osc_raw_freq = AdcRawFrequency(adc_raw_buf, adc_measure_n, dbg_adc_rate);
    adc_over_range = adc_too_fast;
    uint8_t meas_on_tri = adc_tri_on;
    uint8_t probe_due = ((HAL_GetTick() - adc_probe_t) >= 1000u);
    if (adc_tri_on && (adc_too_fast || (adc_fast_content && osc_raw_freq == 0.0f))) adc_over_t = HAL_GetTick() | 1u;
    if (osc_raw_freq > 0.0f) {
        if (adc_meas_lvl < ADC_MEAS_LEVELS - 1u && osc_raw_freq > dbg_adc_rate * (1.0f / 16.0f)) {
            adc_meas_lvl  = ADC_MEAS_LEVELS - 1u;
            adc_measure_n = 3200u;
            osc_raw_freq  = 0.0f;
            adc_hold_f    = 0.0f;
        } else {
            float want = 3.2f * dbg_adc_rate / osc_raw_freq;
            adc_measure_n = (uint32_t)fminf(ADC_RAW_MAX, fmaxf(3200.0f, want));
        }
    } else if (adc_too_fast && adc_meas_lvl < ADC_MEAS_LEVELS - 1u) {
        adc_meas_lvl++;
        adc_measure_n = 3200u;
    } else if (adc_measure_n >= ADC_RAW_MAX) {
        adc_meas_lvl  = (uint8_t)((adc_meas_lvl + 1u) % ADC_MEAS_LEVELS);
        adc_measure_n = 3200u;
    } else {
        adc_measure_n = (adc_measure_n > ADC_RAW_MAX / 2u) ? ADC_RAW_MAX : adc_measure_n * 2u;
    }

    if (osc_raw_freq > 0.0f && !meas_on_tri && probe_due) {
        adc_probe_t = HAL_GetTick();
        uint32_t tp = 0;
        if (AdcSetRateLevel(4) && AdcCaptureRaw(1600u, &tp)) {
            uint16_t pmn = 0xFFFF, pmx = 0; uint32_t pacc = 0;
            for (uint32_t i = 0; i < 1600u; i++) {
                if (adc_raw_buf[i] < pmn) pmn = adc_raw_buf[i];
                if (adc_raw_buf[i] > pmx) pmx = adc_raw_buf[i];
                if (i) pacc += (adc_raw_buf[i] > adc_raw_buf[i - 1]) ? (uint32_t)(adc_raw_buf[i] - adc_raw_buf[i - 1]) : (uint32_t)(adc_raw_buf[i - 1] - adc_raw_buf[i]);
            }
            if ((float)(pmx - pmn) * ADC_VREF / ADC_FULL >= 0.30f && (float)pacc / 1599.0f > 0.25f * (float)(pmx - pmn)
                && (float)pacc / 1599.0f > 120.0f) {
                osc_raw_freq  = 0.0f;
                adc_hold_f    = 0.0f;
                adc_over_t    = HAL_GetTick() | 1u;
                adc_meas_lvl  = ADC_MEAS_LEVELS - 1u;
                adc_measure_n = 3200u;
            }
        }
    }

    if (osc_raw_freq <= 0.0f) osc_raw_valid = 0;
    {
        uint32_t tnow = HAL_GetTick();
        if (osc_raw_freq > 0.0f)                                { adc_hold_f = osc_raw_freq; adc_hold_t = tnow; adc_last_good_f = osc_raw_freq; adc_last_good_t = tnow; adc_last_good_vpp = osc_raw_vpp; }
        else if (osc_raw_vpp < 0.05f || adc_over_range)         { adc_hold_f = 0.0f; }
        else if (adc_hold_f > 0.0f && (tnow - adc_hold_t) < 1500u) { osc_raw_freq = adc_hold_f; adc_f_held = 1; }
    }
    adc_legacy_t = HAL_GetTick();
    PROF_END(5, pB0);
    }

    uint32_t pD0 = PROF_START();
    uint32_t w, dec, pre;
    if (use_pend) {
        step = adc_pd_step; w = adc_pd_w; pre = adc_pd_pre; n = adc_pd_n;
        dec = w;
        dbg_adc_lvl = (uint8_t)lvl;
        dbg_adc_pre = pre;
        if (!AdcCaptureCollect(n, adc_pd_t0, &t_us)) { adc_pend = 0; goto failed; }
        adc_pend = 0;
    } else {
        if (!AdcSetRateLevel(lvl)) goto failed;
        dbg_adc_lvl = (uint8_t)lvl;
        step = dbg_adc_rate / fs_want;
        if (step < 1.0f) step = 1.0f;
        w = (uint32_t)(step + 0.5f);
        dec = w;
        pre = AdcPreFor(fs_want, fast_meas ? adc_last_good_f : osc_raw_freq);
        dbg_adc_pre = pre;
        n = (uint32_t)((N_SAMPLES + pre) * step) + w + 2u;
        if (n > ADC_RAW_MAX || !AdcCaptureRaw(n, &t_us)) goto failed;
    }
    PROF_END(6, pD0);
    dbg_adc_ok = 1;
    dbg_adc_step = step;
    prof_us[7] = 0;
    if (fast_meas) {
        uint32_t pF0 = PROF_START();
        float f2;
        uint32_t tnow;
        adc_too_fast = 0;
        {   uint32_t m = (uint32_t)(3.8f * dbg_adc_rate / adc_last_good_f) + 64u;
            if (m > n) m = n;
            f2 = AdcRawFrequency(adc_raw_buf + (n - m), m, dbg_adc_rate);
        }
        adc_over_range = adc_too_fast;
        tnow = HAL_GetTick();
        dbg_fm_cnt++;
        PROF_END(7, pF0);
        if (f2 > 0.0f && !adc_too_fast && fabsf(f2 - adc_last_good_f) <= 0.10f * adc_last_good_f
            && osc_raw_vpp > 0.80f * adc_last_good_vpp && osc_raw_vpp < 1.25f * adc_last_good_vpp) {
            if (osc_raw_valid && osc_raw_vpp > 0.0f) {
                float kk = adc_last_good_vpp / osc_raw_vpp;
                osc_raw_max_ct = osc_raw_mean_ct + (osc_raw_max_ct - osc_raw_mean_ct) * kk;
                osc_raw_min_ct = osc_raw_mean_ct + (osc_raw_min_ct - osc_raw_mean_ct) * kk;
                osc_raw_rms_ct *= kk;
            }
            osc_raw_vpp = adc_last_good_vpp;
            osc_raw_freq = f2; adc_last_good_f = f2; adc_last_good_t = tnow; adc_hold_f = f2; adc_hold_t = tnow;
        } else {
            if (f2 <= 0.0f) { if (adc_too_fast) dbg_fm_fail[2]++; else dbg_fm_fail[0]++; } else dbg_fm_fail[1]++;
            osc_raw_freq = 0.0f; osc_raw_valid = 0; adc_last_good_t = 0u;
            if (osc_raw_vpp < 0.05f || adc_over_range)                 adc_hold_f = 0.0f;
            else if (adc_hold_f > 0.0f && (tnow - adc_hold_t) < 1500u) { osc_raw_freq = adc_hold_f; adc_f_held = 1; }
        }
    }

    uint32_t edge_m = (uint32_t)(10.0f * step + 0.5f) + w + 2u;
    uint8_t  edge_fit = ((float)edge_m + (float)N_SAMPLES * step + (float)w <= (float)n);
    uint32_t trig_h = (uint32_t)(step * 1.0f) + 1u; if (trig_h > 64u) trig_h = 64u;
    {   float fp = fast_meas ? adc_last_good_f : osc_raw_freq;
        if (fp > 0.0f) { uint32_t hp = (uint32_t)(dbg_adc_rate / fp / 8.0f); if (hp < 1u) hp = 1u; if (trig_h > hp) trig_h = hp; }
    }
    uint32_t t_from = edge_fit ? (edge_m + (uint32_t)(2.0f * step + 1.0f) + 3u * trig_h + 1u) : 1u;
    uint32_t start = 0;
    if (osc_stable && osc_lock_auto) {
        float sum = 0.0f, mid;
        uint32_t lim = (uint32_t)((float)pre * step) + w;
        if (lim < 4u) lim = 4u;
        if (lim > n) lim = n;
        uint16_t tmn = 0xFFFF, tmx = 0;
        for (uint32_t i = 0; i < n; i++) {
            uint16_t v = adc_raw_buf[i];
            sum += (float)v;
            if (v < tmn) tmn = v;
            if (v > tmx) tmx = v;
        }
        (void)sum;
        mid = 0.5f * (float)(tmx + tmn);
        {
            float hy = 0.15f * (float)(tmx - tmn);
            uint8_t armed = 0;
            if (osc_trig_fall) {
                for (uint32_t i = 1; i < lim; i++) {
                    float v = (float)adc_raw_buf[i];
                    if (v > mid + hy) armed = 1;
                    else if (armed && v < mid && i >= t_from) { start = i; break; }
                    else if (armed && v < mid) armed = 0;
                }
            } else {
                for (uint32_t i = 1; i < lim; i++) {
                    float v = (float)adc_raw_buf[i];
                    if (v < mid - hy) armed = 1;
                    else if (armed && v >= mid && i >= t_from) { start = i; break; }
                    else if (armed && v >= mid) armed = 0;
                }
            }
        }
        if (start > 0u) {
            uint32_t H = trig_h;
            uint32_t j0 = (start > 3u * H + H + 1u) ? (start - 3u * H) : (H + 1u), j1 = start + 3u * H;
            if (j1 + H + 1u > n) j1 = n - H - 1u;
            float prev = -1.0f; uint8_t got = 0; uint32_t found = start;
            for (uint32_t j = j0; j <= j1; j++) {
                float a = 0.0f; for (uint32_t q = j - H; q <= j + H; q++) a += (float)adc_raw_buf[q];
                a /= (float)(2u * H + 1u);
                if (prev >= 0.0f) {
                    if (!osc_trig_fall && prev < mid && a >= mid) { found = j; got = 1; break; }
                    if ( osc_trig_fall && prev >= mid && a < mid) { found = j; got = 1; break; }
                }
                prev = a;
            }
            if (got) start = found;
        }
        if ((float)start + (float)N_SAMPLES * step + (float)w > (float)n) start = 0;

        uint32_t lead = (uint32_t)(2.0f * step + 1.0f);
        start = (start > lead) ? (start - lead) : 0u;
    }
    if (edge_fit && start < edge_m) start = edge_m;
    dbg_adc_start = start;

    uint32_t wa = w;
    {   float fp = fast_meas ? adc_last_good_f : osc_raw_freq;
        if (fp > 0.0f) {
            float P = dbg_adc_rate / fp;
            if (2.0f * step >= P) {
                float np = floorf((float)w / P + 0.5f);
                if (np < 1.0f) np = 1.0f;
                uint32_t wp = (uint32_t)(np * P + 0.5f);
                if (wp >= 1u && wp <= n) wa = wp;
            }
        }
    }
    for (int i = 0; i < N_SAMPLES; i++) {
        int32_t c = (int32_t)start + (int32_t)((float)i * step + 0.5f);
        int32_t lo = c - (int32_t)((wa - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0) lo = 0;
        if (lo + (int32_t)wa > (int32_t)n) lo = (int32_t)n - (int32_t)wa;
        if (lo < 0) lo = 0;
        for (uint32_t k = 0; k < wa; k++) acc += adc_raw_buf[(uint32_t)lo + k];
        uint16_t v = (uint16_t)(acc / wa);
        raw_samples[i] = (int16_t)v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    dec = wa;
    osc_env_on = 0;
    {   float fp = fast_meas ? adc_last_good_f : osc_raw_freq;
        if (fp > 0.0f && dbg_adc_rate / fp < 8.0f * step) {
            uint32_t k = (uint32_t)(dbg_adc_rate / fp / 16.0f);
            if (k < 1u) k = 1u;
            if (k > 64u) k = 64u;
            if (n > 2u * k + 2u) {
                {
                    uint32_t q0 = start, q1 = start + (uint32_t)((float)N_SAMPLES * step);
                    float sm = 0.0f, sq = 0.0f, mu;
                    if (q1 > n) q1 = n;
                    if (q1 <= q0) q1 = q0 + 1u;
                    for (uint32_t q = q0; q < q1; q++) sm += (float)adc_raw_buf[q];
                    mu = sm / (float)(q1 - q0);
                    float sqd = 0.0f;
                    for (uint32_t q = q0; q < q1; q++) { float dq = (float)adc_raw_buf[q] - mu; sq += dq * dq;
                        float z = (float)adc_raw_buf[q] - ADC_DC_ZERO_CT; if (z > 0.0f) sqd += z * z; }
                    osc_env_mean_ct = mu; osc_env_rms_ct = sqrtf(sq / (float)(q1 - q0)); osc_env_dcrms_ct = sqrtf(sqd / (float)(q1 - q0));
                }
                for (int i = 0; i < N_SAMPLES; i++) {
                    int32_t c = (int32_t)start + (int32_t)((float)i * step + 0.5f);
                    int32_t a = c - (int32_t)(step * 0.5f) - (int32_t)(k / 2u);
                    int32_t b = c + (int32_t)(step * 0.5f + 0.5f) - (int32_t)(k / 2u);
                    if (b <= a) b = a + 1;
                    if (a < 0) a = 0;
                    if (b > (int32_t)(n - k)) b = (int32_t)(n - k);
                    if (b <= a) a = b - 1;
                    if (a < 0) a = 0;
                    uint32_t run = 0, lo = 0xFFFFFFFFu, hi = 0;
                    for (uint32_t q = 0; q < k; q++) run += adc_raw_buf[(uint32_t)a + q];
                    for (int32_t s = a; ; s++) {
                        if (run < lo) lo = run;
                        if (run > hi) hi = run;
                        if (s + 1 >= b) break;
                        run += adc_raw_buf[(uint32_t)s + k]; run -= adc_raw_buf[(uint32_t)s];
                    }
                    osc_env_lo_ct[i] = (uint16_t)(lo / k);
                    osc_env_hi_ct[i] = (uint16_t)(hi / k);
                }
                osc_env_on = 1;
            }
        }
    }
    MvStoreExt(start, step, wa, n);

    osc_fs_eff = dbg_adc_rate / step;

    dbg_adc_decim  = dec;
    dbg_adc_n      = n;
    dbg_adc_us     = t_us;
    dbg_adc_min    = vmin;
    dbg_adc_max    = vmax;
    dbg_adc_frames++;
    osc_capture_tdiv = osc_tdivs_ms[osc_tdiv_i];
    osc_frame_valid = 1;
    new_frame_ready = 1;
    OSC_SET(osc_dirty      , 1);
    return;
failed:
    dbg_adc_ok = 0;
    osc_raw_valid = 0;
    adc_over_range = 0;
    adc_over_t = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    OSC_SET(osc_dirty, 1);
}
