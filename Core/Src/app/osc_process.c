/* osc_process.c - Frame processing: offset calibration, display filter, measurements (Vmax/Vmin/Vpp/RMS/F). */
#include "app/app.h"

/* --------------------------------------------------------------------------
 * ค่าโวลต์หลัง calibrate (AC coupling) + moving average filter
 * ทำงานเหมือน calibrate_offset() + moving_average_filter() ของ simulator
 * -------------------------------------------------------------------------- */
int OscExtPoint(int k, float *v);   /* v76: real display point k (<0 or >=N_SAMPLES) of THIS FAST frame; defined in MOVE_DATA */
int SlowExtPoint(int k, float *v);  /* v80: the same for a SLOW frame; defined in SLOW */

/* v82: display filter window actually used. The FILTER setting (osc_fwin) is a boxcar; at the measured frequency it keeps
 * H = sin(win*pi/ppc) / (win*sin(pi/ppc)) of the amplitude (ppc = display points per period). 5 points at 23.6 ppc (2 kHz, 0.5 ms/div)
 * drew the wave 7 % too small, so it no longer matched V/DIV on the grid. The window is narrowed (by 2) until H >= 0.98.
 * Unknown frequency, or < 2 points per period (not drawable as a wave, see v81): the setting as it is. */
int OscFiltWin(void)
{
    int win = (osc_fwin < 1) ? 1 : osc_fwin;
    if ((win % 2) == 0) win++;
    if (osc_raw_freq > 0.0f && osc_fs_eff > 0.0f) {
        float ppc = osc_fs_eff / osc_raw_freq;
        if (ppc >= 2.0f) {
            float th = 3.14159265f / ppc;
            while (win > 1 && sinf((float)win * th) / ((float)win * sinf(th)) < 0.98f) win -= 2;
        }
    }
    return win;
}

void OscProcessFrame(void)
{
    const float k  = ADC_VREF / (float)ADC_FULL;                  /* pin volts (DC readout) */
    const float kg = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN; /* v84: input volts (trace) */
    float       s = 0.0f;
    int         win, half;

    /* osc_fs_eff belongs to the captured frame, including HOLD redraws. */

    /* Calibration (AC coupling): the offset is the mean over WHOLE cycles of the raw capture (a partial cycle biases a
     * window mean by up to ~13 % of the amplitude). No clean period -> mid-range (exact for symmetric waves). */
    if (osc_calib) {
#if ADC_INPUT_ENABLE
        (void)s;
        if (osc_raw_valid && osc_frame_valid) {
            osc_offset_ct = osc_raw_mean_ct;
        } else {
            int16_t lo = raw_samples[0], hi = raw_samples[0];
            for (int i = 1; i < N_SAMPLES; i++) { if (raw_samples[i] < lo) lo = raw_samples[i]; if (raw_samples[i] > hi) hi = raw_samples[i]; }
            osc_offset_ct = ((float)lo + (float)hi) * 0.5f;
        }
#else
        for (int i = 0; i < N_SAMPLES; i++) s += (float)raw_samples[i];
        osc_offset_ct = s / (float)N_SAMPLES;
#endif
    } else {
        osc_offset_ct = ADC_DC_ZERO_CT;                 /* v89: DC: real 0 V on the centre line (was ADC_MID) */
    }
    osc_offset_v = osc_calib ? (osc_offset_ct - (float)ADC_MID) * k : 0.0f;   /* v89: DC removes nothing */

    /* Moving average filter (window 1..21, คี่) */
    win  = OscFiltWin();   /* v82: odd, never shrinks the wave by > 2 % */
    half = win / 2;

    for (int i = 0; i < N_SAMPLES; i++) {
        if (win <= 1) {
            osc_v[i] = ((float)raw_samples[i] - osc_offset_ct) * kg;
        }
    }
    if (win > 1) {
        /* Middle: the same centred moving average as before. */
        float flo = 1e9f, fhi = -1e9f, rlo = 1e9f, rhi = -1e9f;
        for (int i = half; i < N_SAMPLES - half; i++) {
            float acc = 0.0f;
            for (int j = -half; j <= half; j++) acc += (float)raw_samples[i + j];
            acc /= (float)win;
            osc_v[i] = (acc - osc_offset_ct) * kg;
            if (acc < flo) flo = acc;
            if (acc > fhi) fhi = acc;
            if (raw_samples[i] < rlo) rlo = (float)raw_samples[i];
            if (raw_samples[i] > rhi) rhi = (float)raw_samples[i];
        }
        /* v74/v76 edges (the first/last `half` points have no on-screen neighbour on one side).  The old clamp
         * repeated sample 0 / 239, so the edge points stayed almost unfiltered and stood out as spikes whenever the
         * filter visibly smooths the middle (2 MHz, 500 kHz, ...).
         *  1. FAST: the capture keeps the real display points just outside the window (mv_ext_lo/hi, same
         *     resampling as raw_samples) -> the edge point gets the SAME centred filter as the middle. Exact.
         *  2. no outside data (SLOW, sim): if the filter changes the middle by > 10 % slide the window inward
         *     (edge averages `win` real samples like the middle -> no spike), else shrink the centred window. */
        uint8_t crushed = (fhi - flo) < 0.9f * (rhi - rlo);
        for (int e = 0; e < 2 * half; e++) {
            int i = (e < half) ? e : (N_SAMPLES - 2 * half + e);
            float acc = 0.0f, ev;
            int j, ok = 1;
            for (j = -half; j <= half; j++) {
                int m = i + j;
                if (m >= 0 && m < N_SAMPLES) acc += (float)raw_samples[m];
                else if (OscExtPoint(m, &ev)) acc += ev;
                else { ok = 0; break; }
            }
            if (ok) { osc_v[i] = (acc / (float)win - osc_offset_ct) * kg; continue; }   /* v88: kg like the middle (v84 left k: edges 20 % short = spikes) */
            int a, n;
            if (crushed) { a = (i < half) ? 0 : (N_SAMPLES - win); n = win; }
            else         { int hh = (i < half) ? i : (N_SAMPLES - 1 - i); a = i - hh; n = 2 * hh + 1; }
            acc = 0.0f;
            for (int m = a; m < a + n; m++) acc += (float)raw_samples[m];
            osc_v[i] = (acc / (float)n - osc_offset_ct) * kg;
        }
    }
    if (!osc_calib)   /* v90: DC: the front end passes no negative half -> nothing below 0 V (counts under the DC zero are the clamp floor) */
        for (int i = 0; i < N_SAMPLES; i++) if (osc_v[i] < 0.0f) osc_v[i] = 0.0f;
}

/* --------------------------------------------------------------------------
 * วัดค่า Vmax/Vmin/Vpp/RMS/Freq + ตรวจชนิดคลื่นจาก crest factor
 * เหมือน extract_features() + detect_waveform_type() ของ simulator
 * -------------------------------------------------------------------------- */
void OscMeasureFrame(void)
{
    float vmin = 1e9f, vmax = -1e9f, s2 = 0.0f, prev = 0.0f, peak = 0.0f;
    float cross[8];
    int   ncross = 0;

    for (int i = 0; i < N_SAMPLES; i++) {          /* รอบ 1: min / max / RMS / peak */
        float v = osc_v[i];
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
        s2 += v * v;
        if (fabsf(v) > peak) peak = fabsf(v);
    }

    /* รอบ 2 (p42): นับขอบตัดศูนย์แบบ "มี hysteresis" — กัน noise สร้างขอบปลอม
     * เดิมนับทุกครั้งที่ข้าม 0: ที่ความถี่ต่ำ (หน้าต่างมี 1-2 คาบ) noise ทำให้ได้ขอบเกิน
     * -> คาบสั้นผิด (อ่านได้ 2 เท่า) หรือได้ขอบไม่ครบ -> อ่านได้ 0
     * ใหม่: หลังนับขอบหนึ่ง ต้องรอให้สัญญาณแกว่งพ้นครึ่งตรงข้ามก่อน จึงนับขอบถัดไป */
    {
        float hyst = 0.2f * (vmax - vmin);          /* ±20% ของ Vpp */
        float vc = osc_calib ? 0.0f : 0.5f * (vmax + vmin);   /* v89: DC trace: count around its middle */
        int   armed = 1;
        if (hyst < 1e-6f) hyst = 1e-6f;
        prev = osc_v[0] - vc;
        for (int i = 1; i < N_SAMPLES; i++) {
            float v = osc_v[i] - vc;
            int up = (!osc_trig_fall && prev < 0.0f && v >= 0.0f);
            int dn = ( osc_trig_fall && prev >= 0.0f && v < 0.0f);
            if (armed && (up || dn)) {
                float f = (v != prev) ? (-prev / (v - prev)) : 0.0f;
                if (ncross < 8) cross[ncross++] = ((float)i - 1.0f + f) / osc_fs_eff;
                armed = 0;
            } else if (!armed) {
                if (osc_trig_fall ? (v > hyst) : (v < -hyst)) armed = 1;
            }
            prev = v;
        }
    }
    osc_disp_cross = ncross;

#if ADC_INPUT_ENABLE
    {   /* Mx / Mn / Rs / Vp from the UNFILTERED data: the display smoothing filter and the averaging used to draw fast
         * signals must not shrink the numbers (a 5-point filter alone costs ~6 % at 1 kHz). */
        const float kv = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;   /* v84: input volts, same scale as the trace */
        if (osc_raw_valid && osc_frame_valid) {            /* whole cycles of the raw capture */
            float ref = osc_calib ? osc_raw_mean_ct : osc_offset_ct;   /* v89: DC = absolute volts */
            vmax = (osc_raw_max_ct - ref) * kv;
            vmin = (osc_raw_min_ct - ref) * kv;
            s2   = osc_raw_rms_ct * kv;  s2 = s2 * s2 + ((osc_raw_mean_ct - ref) * kv) * ((osc_raw_mean_ct - ref) * kv);
            if (!osc_calib) { s2 = osc_raw_dcrms_ct * kv; s2 = s2 * s2; }   /* v90: DC: rms of the clipped (>= 0 V) signal, includes the mean */
            s2  *= (float)N_SAMPLES;
        } else if (osc_env_on && !slow_active && osc_frame_valid) {   /* v83: no clean period, peak-detect frame: the band (the drawn points are only the mean) */
            float hi = 0.0f, lo = 1e9f;
            for (int i = 0; i < N_SAMPLES; i++) {
                if ((float)osc_env_hi_ct[i] > hi) hi = (float)osc_env_hi_ct[i];
                if ((float)osc_env_lo_ct[i] < lo) lo = (float)osc_env_lo_ct[i];
            }
            float ref = osc_calib ? osc_env_mean_ct : osc_offset_ct;   /* v89 */
            vmax = (hi - ref) * kv;
            vmin = (lo - ref) * kv;
            s2   = osc_env_rms_ct * kv;  s2 = s2 * s2 + ((osc_env_mean_ct - ref) * kv) * ((osc_env_mean_ct - ref) * kv);
            if (!osc_calib) { s2 = osc_env_dcrms_ct * kv; s2 = s2 * s2; }   /* v90: DC */
            s2  *= (float)N_SAMPLES;
        } else {                                            /* no clean period: whole unfiltered window */
            vmax = -1e9f; vmin = 1e9f; s2 = 0.0f;
            for (int i = 0; i < N_SAMPLES; i++) {
                float v = ((float)raw_samples[i] - osc_offset_ct) * kv;
                if (!osc_calib && v < 0.0f) v = 0.0f;   /* v90: DC clipped at 0 V (Mn and Rs) */
                if (v > vmax) vmax = v;
                if (v < vmin) vmin = v;
                s2 += v * v;
            }
        }
        if (!osc_calib) { if (vmin < 0.0f) vmin = 0.0f; if (vmax < 0.0f) vmax = 0.0f; }   /* v90: DC: Mn/Mx never below 0 V, like the trace */
        peak = (fabsf(vmax) > fabsf(vmin)) ? fabsf(vmax) : fabsf(vmin);
    }
#endif
    OSC_LOCK();
    osc_meas.vmax = vmax;
    osc_meas.vmin = vmin;
    osc_meas.vpp  = vmax - vmin;
    osc_meas.vrms = sqrtf(s2 / (float)N_SAMPLES);
    osc_meas.dc_offset = osc_offset_v;              /* DC ก่อนทำ AC coupling */
    osc_meas.freq = 0.0f;
    if ((vmax - vmin) < 0.05f) {                    /* p43: สัญญาณเล็กเกิน (เช่นยังไม่ต่อสาย)
                                                     * -> ไม่รายงานความถี่ (เดิมโชว์ค่า noise) */
        osc_meas.freq = 0.0f;
    } else if (ncross >= 2) {                       /* วัดคาบจากจุดตัดศูนย์ */
        float span = cross[ncross - 1] - cross[0];
        if (span > 0.0f) osc_meas.freq = (float)(ncross - 1) / span;
    }

#if ADC_INPUT_ENABLE
    osc_meas.freq = osc_frame_valid ? osc_raw_freq : 0.0f;
#endif
    OSC_UNLOCK();
#if ADC_INPUT_ENABLE
    /* v52: analog input bandwidth compensation.  The ADC input (source resistance + sample-and-hold capacitor) acts as
     * a one-pole low-pass at about ADC_INPUT_FC: measured on the board, 2.1 MHz read 0.87 V for a ~1.93 V signal.
     * The raw samples are already that small, so the software cannot "see" more; instead the DISPLAYED numbers are
     * scaled back by the known filter gain at the measured frequency.  Below ~100 kHz the factor is < 1.005 (no change);
     * SLOW mode frequencies (< 100 Hz) give exactly 1.  Only osc_meas (screen/UART) is scaled - the raw values used by
     * auto time base, hold and trigger logic are untouched. */
    OSC_LOCK();
    {
        float g = 1.0f;
        if (osc_meas.freq > 0.0f) {
            float r = osc_meas.freq / ADC_INPUT_FC;
            g = sqrtf(1.0f + expf(ADC_INPUT_N * logf(r)));
            if (g > ADC_INPUT_GMAX) g = ADC_INPUT_GMAX;
        }
        osc_meas.vmax *= g; osc_meas.vmin *= g; osc_meas.vpp *= g; osc_meas.vrms *= g;
        peak *= g;
    }
    OSC_UNLOCK();
#endif
    if (peak < 0.001f) {
        strcpy(osc_det, "---");
    } else {
        float crest = osc_meas.vrms / peak;          /* sine .707 sq 1.0 tri .577 */
        if      (crest > 0.85f) strcpy(osc_det, "SQR");
        else if (crest > 0.65f) strcpy(osc_det, "SIN");
        else                    strcpy(osc_det, "TRI/SAW");
    }
}
