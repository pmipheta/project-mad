/* Acquisition concern; see README.md for ownership and extension points. */
#include "osc_acquisition.h"
#include "osc_signal.h"
#include "osc_app.h"
#include "main.h"
#include <math.h>
#include <string.h>

#if ADC_INPUT_ENABLE

static uint32_t AdcElapsedUs(uint32_t start);
void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
void ADC_IRQHandler(void);
void DMA2_Stream0_IRQHandler(void);
static uint8_t AdcCaptureRaw(uint32_t n, uint32_t *t_us);
void AdcInputInit(void);
static uint8_t AdcSetRate(int slow);
static float AdcRawFrequency(uint32_t n, float fs);
void AdcCaptureFrame(void);

static uint32_t adc_measure_n = 3200u; /* adaptive, independent of display */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

ALIGN_32BYTES(uint16_t adc_raw_buf[ADC_RAW_MAX]);
static volatile uint8_t  adc_done = 0, adc_error = 0;

/* ค่าที่อ่านผ่าน SWD เพื่อตรวจสอบ */
volatile float    dbg_adc_rate   = 0.0f;   /* ADC clock / conversion cycles */
volatile uint32_t dbg_adc_decim  = 1;      /* ตัวหารที่ใช้เฟรมล่าสุด              */
volatile uint32_t dbg_adc_n      = 0;      /* จำนวนจุดดิบที่จับ                     */
volatile uint32_t dbg_adc_us     = 0;      /* เวลาจับข้อมูลจริง (µs)               */
volatile uint16_t dbg_adc_min    = 0;      /* ค่าต่ำสุด/สูงสุดในบัฟเฟอร์ดิบ         */
volatile uint16_t dbg_adc_max    = 0;
volatile uint32_t dbg_adc_frames = 0;
volatile uint8_t  dbg_adc_ok     = 0;      /* 0 = ยังวัดอัตราสุ่มไม่สำเร็จ         */
volatile float    dbg_adc_step   = 1.0f;   /* จุดดิบต่อ 1 จุดแสดงผล (step จริง)     */
volatile uint32_t dbg_adc_start  = 0;      /* index ที่เริ่มวาด (จุด trigger, p38)   */

static uint32_t AdcElapsedUs(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start) / (HAL_RCC_GetHCLKFreq() / 1000000UL);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1) {
        GPIO_InitTypeDef g = {0};
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* PA0 = ADC1_IN0 (ขาว่างของโปรเจกต์นี้: ปุ่มใช้ PB0-PB3) */
        g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &g);

        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_NORMAL;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) adc_done = 1;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    /* Continuous ADC may overrun DR after normal-mode DMA has filled every
     * requested sample. That tail overrun cannot corrupt the completed buffer. */
    if (hadc->Instance == ADC1 && !(adc_done && hadc->ErrorCode == HAL_ADC_ERROR_OVR))
        adc_error = 1;
}

void ADC_IRQHandler(void) { HAL_ADC_IRQHandler(&hadc1); }
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

/* จับข้อมูลดิบ n จุด (blocking สั้น ๆ) -> 1 = สำเร็จ */
static uint8_t AdcCaptureRaw(uint32_t n, uint32_t *t_us)
{
    uint32_t t0, to;
    if (n == 0 || n > ADC_RAW_MAX || n > 65535u || !t_us) return 0;
    uint32_t bytes = (n * 2u + 31u) & ~31u;
    adc_done = adc_error = 0;
    /* Aligned buffer and padded capacity own every invalidated cache line. */
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

void AdcInputInit(void)
{
    ADC_ChannelConfTypeDef c = {0};
    uint32_t t_us = 0;


    /* DWT ไมโครวินาที (โปรเจกต์นี้ยังไม่เปิด) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *((volatile uint32_t *)0xE0001FB0) = 0xC5ACCE55;        /* ปลดล็อก DWT (Cortex-M7) */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;   /* PCLK2/4 = 12 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    c.Channel = ADC_CHANNEL_0; c.Rank = ADC_REGULAR_RANK_1; c.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &c) != HAL_OK) Error_Handler();

    /* วัดอัตราสุ่มจริง (ใช้ 2000 จุด -> ไม่กี่ ms) แล้วเก็บเป็นแหล่งความจริง */
    adc_done = 0;
    if (AdcCaptureRaw(2000u, &t_us) && t_us > 0u) {
        dbg_adc_rate = 2000.0f * 1000000.0f / (float)t_us;
        dbg_adc_ok   = 1;
    }
}

/* Use ADC clock timing, not DMA start/stop software latency, for sample spacing. */
static uint8_t AdcSetRate(int slow)
{
    ADC_ChannelConfTypeDef c = {0};
    c.Channel = ADC_CHANNEL_0;
    c.Rank = ADC_REGULAR_RANK_1;
    c.SamplingTime = slow ? ADC_SAMPLETIME_15CYCLES : ADC_SAMPLETIME_FAST;
    if (HAL_ADC_ConfigChannel(&hadc1, &c) != HAL_OK) return 0;
    dbg_adc_rate = (float)HAL_RCC_GetPCLK2Freq() / 4.0f / (slow ? 27.0f : 15.0f);
    return 1;
}

static float AdcRawFrequency(uint32_t n, float fs)
{
    uint16_t mn = 4095, mx = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (adc_raw_buf[i] < mn) mn = adc_raw_buf[i];
        if (adc_raw_buf[i] > mx) mx = adc_raw_buf[i];
    }
    osc_raw_vpp = (mx - mn) * ADC_VREF / ADC_FULL;
    if (osc_raw_vpp < 0.05f) return 0.0f;
    float mid = (mx + mn) * 0.5f, hyst = (mx - mn) * 0.2f;
    float first = 0.0f, last = 0.0f;
    uint32_t edges = 0;
    int armed = 0;
    for (uint32_t i = 1; i < n; i++) {
        float prev = adc_raw_buf[i - 1], v = adc_raw_buf[i];
        if (v < mid - hyst) armed = 1;
        if (armed && prev < mid && v >= mid) {
            float at = (float)(i - 1) + (mid - prev) / (v - prev);
            if (!edges) first = at;
            last = at;
            edges++;
            armed = 0;
        }
    }
    /* Need repeated complete periods; partial windows must not invent a frequency. */
    if (edges < 3 || last <= first) return 0.0f;
    float f = (edges - 1) * fs / (last - first);
    return (f >= 39.9f && f <= 100100.0f && f <= fs / 8.0f) ? f : 0.0f;
}

/* จับ 1 เฟรม: เลือกตัวหารจาก T/DIV -> เฉลี่ย -> raw_samples[N_SAMPLES] */
void AdcCaptureFrame(void)
{
    float fs_want = (float)N_SAMPLES / (osc_tdivs_ms[osc_tdiv_i] * OSC_H_DIVS / 1000.0f);
    uint32_t n, t_us = 0;
    uint16_t vmin = 0xFFFF, vmax = 0;
    new_frame_ready = 0;
    osc_frame_valid = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_meas = (OscMeasure){0};
    strcpy(osc_det, "INVALID");

    /* Fast, independent measurement: 4 ms initially, grow on later frames
     * until >=3 periods are available. Max 81.9 ms only for slow/no signal.
     * Every acquisition is contiguous; never stitch separate DMA blocks.
     * Supported periodic input: 40 Hz..100 kHz, >=50 mVpp, within 0..3.3 V.
     * External anti-alias filtering/source impedance remain hardware duties. */
    if (!AdcSetRate(0) || !AdcCaptureRaw(adc_measure_n, &t_us)) goto failed;
    osc_raw_freq = AdcRawFrequency(adc_measure_n, dbg_adc_rate);
    if (osc_raw_freq > 0.0f) {
        float want = 3.2f * dbg_adc_rate / osc_raw_freq;
        adc_measure_n = (uint32_t)fminf(ADC_RAW_MAX, fmaxf(3200.0f, want));
    } else {
        adc_measure_n = (adc_measure_n > ADC_RAW_MAX / 2u) ? ADC_RAW_MAX : adc_measure_n * 2u;
    }

    /* 6 ms/div needs >65535 samples at 800 kSa/s. Use 15-cycle
     * sampling (27 cycles/conversion) for that display capture, not truncation. */
    float step = dbg_adc_rate / fs_want;
    if ((N_SAMPLES + ADC_PRE_POINTS + 1) * step + 3 > ADC_RAW_MAX) {
        if (!AdcSetRate(1)) goto failed;
        step = dbg_adc_rate / fs_want;
    }
    if (step < 1.0f) step = 1.0f;
    uint32_t w = (uint32_t)(step + 0.5f);
    uint32_t dec = w;
    n = (uint32_t)((N_SAMPLES + ADC_PRE_POINTS) * step) + w + 2u;
    if (n > ADC_RAW_MAX || !AdcCaptureRaw(n, &t_us)) goto failed;
    dbg_adc_ok = 1;
    dbg_adc_step = step;

    /* --- หาจุด trigger ในข้อมูลดิบ (ระดับ = ค่ากลางของบัฟเฟอร์) แล้วเริ่มวาดที่จุดนั้น --- */
    uint32_t start = 0;
    if (osc_stable) {
        float sum = 0.0f, mid;
        uint32_t lim = (uint32_t)((float)ADC_PRE_POINTS * step) + w;   /* ค้นเฉพาะส่วนก่อน trigger */
        if (lim < 4u) lim = 4u;
        if (lim > n) lim = n;
        for (uint32_t i = 0; i < n; i++) sum += (float)adc_raw_buf[i];
        mid = sum / (float)n;
        if (osc_trig_fall) {
            for (uint32_t i = 1; i < lim; i++)
                if ((float)adc_raw_buf[i - 1u] >= mid && (float)adc_raw_buf[i] < mid) { start = i; break; }
        } else {
            for (uint32_t i = 1; i < lim; i++)
                if ((float)adc_raw_buf[i - 1u] < mid && (float)adc_raw_buf[i] >= mid) { start = i; break; }
        }
        if ((float)start + (float)N_SAMPLES * step + (float)w > (float)n) start = 0;   /* กันหลุดขอบท้าย */

        /* เหลื่อมไปข้างหน้า ~2 จุดแสดงผล: ไม่ให้หน้าต่าง "เริ่มตรงจุดตัดศูนย์พอดี"
         * (ถ้าเริ่มพอดี ตัววัดความถี่ของ firmware จะนับขอบแรกพลาดเป็นครั้งคราว -> f กระพริบเป็น 0) */
        uint32_t lead = (uint32_t)(2.0f * step + 1.0f);
        start = (start > lead) ? (start - lead) : 0u;
    }
    dbg_adc_start = start;

    for (int i = 0; i < N_SAMPLES; i++) {   /* resample แบบเศษ + boxcar เฉลี่ย w จุด (เริ่มที่จุด trigger) */
        int32_t c = (int32_t)start + (int32_t)((float)i * step + 0.5f);
        int32_t lo = c - (int32_t)((w - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0) lo = 0;
        if (lo + (int32_t)w > (int32_t)n) lo = (int32_t)n - (int32_t)w;
        if (lo < 0) lo = 0;
        for (uint32_t k = 0; k < w; k++) acc += adc_raw_buf[(uint32_t)lo + k];
        uint16_t v = (uint16_t)(acc / w);
        raw_samples[i] = (int16_t)v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }

    /* แกนเวลา: อัตราสุ่มจริงหลัง resample (ไม่ใช้ค่าที่ตั้งไว้) */
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
    osc_dirty       = 1;
    return;
failed:
    dbg_adc_ok = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_dirty = 1;
}

#endif /* ADC_INPUT_ENABLE */
