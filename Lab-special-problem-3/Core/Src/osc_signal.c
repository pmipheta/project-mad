/* Signal concern; see README.md for ownership and extension points. */
#include "osc_signal.h"
#include "osc_app.h"
#include <math.h>
#include <string.h>

void OscProcessFrame(void);
void OscMeasureFrame(void);
int FindTrigger(void);

/* --- Sample buffer (teammate writes here) ---------------------------------- */
volatile int16_t raw_samples[N_SAMPLES];  /* ADC counts 0-4095                */
volatile uint8_t new_frame_ready = 0;     /* teammate sets 1 when buffer ready */

/* --- Measurement results (teammate writes here) ---------------------------- */
volatile OscMeasure osc_meas = {0};
float    osc_offset_ct = (float)ADC_MID;
static float    osc_offset_v  = 0.0f;
float    osc_fs_eff    = 100000.0f; /* sample rate สุทธิของหน้าต่างที่แสดง */
char     osc_det[8]    = "---";     /* ชนิดคลื่นที่ตรวจได้               */
float    osc_v[N_SAMPLES];          /* ข้อมูลหลัง calibrate + filter    */
uint8_t  osc_frame_valid = 0;
float    osc_raw_freq = 0.0f, osc_raw_vpp = 0.0f;
float    osc_capture_tdiv = 1.0f;

void OscProcessFrame(void)
{
    const float k = ADC_VREF / (float)ADC_FULL;
    float       s = 0.0f;
    int         win, half;

    /* osc_fs_eff belongs to the captured frame, including HOLD redraws. */

    /* Calibration: หา DC offset จริงในบัฟเฟอร์แล้วทำ AC coupling */
    if (osc_calib) {
        for (int i = 0; i < N_SAMPLES; i++) s += (float)raw_samples[i];
        osc_offset_ct = s / (float)N_SAMPLES;
    } else {
        osc_offset_ct = (float)ADC_MID;
    }
    osc_offset_v = (osc_offset_ct - (float)ADC_MID) * k;

    /* Moving average filter (window 1..21, คี่) */
    win  = (osc_fwin < 1) ? 1 : osc_fwin;
    if ((win % 2) == 0) win++;
    half = win / 2;

    for (int i = 0; i < N_SAMPLES; i++) {
        if (win <= 1) {
            osc_v[i] = ((float)raw_samples[i] - osc_offset_ct) * k;
        } else {
            float acc = 0.0f;
            for (int j = -half; j <= half; j++) {
                int m = i + j;
                if (m < 0) m = 0;
                if (m >= N_SAMPLES) m = N_SAMPLES - 1;
                acc += (float)raw_samples[m];
            }
            osc_v[i] = (acc / (float)win - osc_offset_ct) * k;
        }
    }
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
        int   armed = 1;
        if (hyst < 1e-6f) hyst = 1e-6f;
        prev = osc_v[0];
        for (int i = 1; i < N_SAMPLES; i++) {
            float v = osc_v[i];
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
    if (peak < 0.001f) {
        strcpy(osc_det, "---");
    } else {
        float crest = osc_meas.vrms / peak;          /* sine .707 sq 1.0 tri .577 */
        if      (crest > 0.85f) strcpy(osc_det, "SQR");
        else if (crest > 0.65f) strcpy(osc_det, "SIN");
        else                    strcpy(osc_det, "TRI/SAW");
    }
}
int FindTrigger(void)
{
    /* หาจุด trigger จากข้อมูลที่สอบเทียบ/กรองแล้ว (osc_v แกว่งรอบ 0 = auto level)
     * ใช้ขอบตามที่ตั้งไว้ใน TRIG: RISE = ผ่าน 0 ขาขึ้น | FALL = ผ่าน 0 ขาลง
     * ค้นแค่ 3/4 ของบัฟเฟอร์ เพื่อให้เหลือข้อมูลพอวาดเต็มจอหลังจุด trigger */
    int limit = (N_SAMPLES * 3) / 4;
    if (osc_trig_fall) {
        for (int i = 1; i < limit; i++)
            if (osc_v[i - 1] >= 0.0f && osc_v[i] < 0.0f) return i;
    } else {
        for (int i = 1; i < limit; i++)
            if (osc_v[i - 1] < 0.0f && osc_v[i] >= 0.0f) return i;
    }
    return -1;   /* หน้าต่างนี้ไม่เจอขอบ -> ใช้ค่าเดิม (ภาพไม่กระโดด) */
}
