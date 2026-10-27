#include "app/app.h"

int OscExtPoint(int k, float *v);
int SlowExtPoint(int k, float *v);

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
    const float k  = ADC_VREF / (float)ADC_FULL;
    const float kg = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;
    int         win, half;

    if (osc_calib) {
        if (osc_raw_valid && osc_frame_valid) {
            osc_offset_ct = osc_raw_mean_ct;
        } else {
            int16_t lo = raw_samples[0], hi = raw_samples[0];
            for (int i = 1; i < N_SAMPLES; i++) { if (raw_samples[i] < lo) lo = raw_samples[i]; if (raw_samples[i] > hi) hi = raw_samples[i]; }
            osc_offset_ct = ((float)lo + (float)hi) * 0.5f;
        }
    } else {
        osc_offset_ct = ADC_DC_ZERO_CT;
    }
    osc_offset_v = osc_calib ? (osc_offset_ct - (float)ADC_MID) * k : 0.0f;

    win  = OscFiltWin();
    half = win / 2;

    for (int i = 0; i < N_SAMPLES; i++) {
        if (win <= 1) {
            osc_v[i] = ((float)raw_samples[i] - osc_offset_ct) * kg;
        }
    }
    if (win > 1) {
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
            if (ok) { osc_v[i] = (acc / (float)win - osc_offset_ct) * kg; continue; }
            int a, n;
            if (crushed) { a = (i < half) ? 0 : (N_SAMPLES - win); n = win; }
            else         { int hh = (i < half) ? i : (N_SAMPLES - 1 - i); a = i - hh; n = 2 * hh + 1; }
            acc = 0.0f;
            for (int m = a; m < a + n; m++) acc += (float)raw_samples[m];
            osc_v[i] = (acc / (float)n - osc_offset_ct) * kg;
        }
    }
    if (!osc_calib)
        for (int i = 0; i < N_SAMPLES; i++) if (osc_v[i] < 0.0f) osc_v[i] = 0.0f;
}

void OscMeasureFrame(void)
{
    float vmin = 1e9f, vmax = -1e9f, s2 = 0.0f, prev = 0.0f, peak = 0.0f;
    int   ncross = 0;

    for (int i = 0; i < N_SAMPLES; i++) {
        float v = osc_v[i];
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
        s2 += v * v;
        if (fabsf(v) > peak) peak = fabsf(v);
    }

    {
        float hyst = 0.2f * (vmax - vmin);
        float vc = osc_calib ? 0.0f : 0.5f * (vmax + vmin);
        int   armed = 1;
        if (hyst < 1e-6f) hyst = 1e-6f;
        prev = osc_v[0] - vc;
        for (int i = 1; i < N_SAMPLES; i++) {
            float v = osc_v[i] - vc;
            int up = (!osc_trig_fall && prev < 0.0f && v >= 0.0f);
            int dn = ( osc_trig_fall && prev >= 0.0f && v < 0.0f);
            if (armed && (up || dn)) {
                if (ncross < 8) ncross++;
                armed = 0;
            } else if (!armed) {
                if (osc_trig_fall ? (v > hyst) : (v < -hyst)) armed = 1;
            }
            prev = v;
        }
    }
    osc_disp_cross = ncross;

    {
        const float kv = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;
        if (osc_raw_valid && osc_frame_valid) {
            float ref = osc_calib ? osc_raw_mean_ct : osc_offset_ct;
            vmax = (osc_raw_max_ct - ref) * kv;
            vmin = (osc_raw_min_ct - ref) * kv;
            s2   = osc_raw_rms_ct * kv;  s2 = s2 * s2 + ((osc_raw_mean_ct - ref) * kv) * ((osc_raw_mean_ct - ref) * kv);
            if (!osc_calib) { s2 = osc_raw_dcrms_ct * kv; s2 = s2 * s2; }
            s2  *= (float)N_SAMPLES;
        } else if (osc_env_on && !slow_active && osc_frame_valid) {
            float hi = 0.0f, lo = 1e9f;
            for (int i = 0; i < N_SAMPLES; i++) {
                if ((float)osc_env_hi_ct[i] > hi) hi = (float)osc_env_hi_ct[i];
                if ((float)osc_env_lo_ct[i] < lo) lo = (float)osc_env_lo_ct[i];
            }
            float ref = osc_calib ? osc_env_mean_ct : osc_offset_ct;
            vmax = (hi - ref) * kv;
            vmin = (lo - ref) * kv;
            s2   = osc_env_rms_ct * kv;  s2 = s2 * s2 + ((osc_env_mean_ct - ref) * kv) * ((osc_env_mean_ct - ref) * kv);
            if (!osc_calib) { s2 = osc_env_dcrms_ct * kv; s2 = s2 * s2; }
            s2  *= (float)N_SAMPLES;
        } else {
            vmax = -1e9f; vmin = 1e9f; s2 = 0.0f;
            for (int i = 0; i < N_SAMPLES; i++) {
                float v = ((float)raw_samples[i] - osc_offset_ct) * kv;
                if (!osc_calib && v < 0.0f) v = 0.0f;
                if (v > vmax) vmax = v;
                if (v < vmin) vmin = v;
                s2 += v * v;
            }
        }
        if (!osc_calib) { if (vmin < 0.0f) vmin = 0.0f; if (vmax < 0.0f) vmax = 0.0f; }
        peak = (fabsf(vmax) > fabsf(vmin)) ? fabsf(vmax) : fabsf(vmin);
    }
    OSC_LOCK();
    osc_meas.vmax = vmax;
    osc_meas.vmin = vmin;
    osc_meas.vpp  = vmax - vmin;
    osc_meas.vrms = sqrtf(s2 / (float)N_SAMPLES);
    osc_meas.dc_offset = osc_offset_v;
    osc_meas.freq = osc_frame_valid ? osc_raw_freq : 0.0f;
    OSC_UNLOCK();
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
    if (peak < 0.001f) {
        strcpy(osc_det, "---");
    } else {
        float crest = osc_meas.vrms / peak;
        if      (crest > 0.85f) strcpy(osc_det, "SQR");
        else if (crest > 0.65f) strcpy(osc_det, "SIN");
        else                    strcpy(osc_det, "TRI/SAW");
    }
}
