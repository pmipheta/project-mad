/* Simulator concern; see README.md for ownership and extension points. */
#include "osc_simulator.h"
#include "osc_signal.h"
#include "osc_display.h"
#include "main.h"
#include <math.h>

void GenFrame(void);
static float SimNoise(void);
static void SimUpdateMeasure(void);
void SimSamplingStream(void);
void GenerateFakeSineWave(void);

static float mock_phase = 0.0f;
const char  *gen_wave_names[]  = {"SINE", "SQUARE", "TRI", "SAW"};
/* --- generator (ส่วน input ของ simulator) --- */
int      gen_wave      = 0;         /* 0=SINE 1=SQUARE 2=TRI 3=SAW      */
float    gen_freq      = 1000.0f;   /* 100 .. 5000 Hz                   */
float    gen_amp       = 0.5f;      /* 0.1 .. 1.5 V                     */
float    gen_dc        = 1.65f;     /* 0.5 .. 2.5 V                     */
float    gen_noise     = 0.05f;     /* 0 .. 0.3 V                       */
float    gen_ph        = 0.0f;      /* เฟสสะสม (roll)                    */
static float    sim_t    = 0.0f;    /* เวลาจริงสะสม (วินาที)                     */
static float    sim_ph1  = 0.0f;    /* เฟสสะสมโทนหลัก (rad, 0..2pi)              */
static float    sim_ph2  = 0.0f;    /* เฟสสะสมโทนที่ 2                            */
static uint32_t sim_prev = 0;       /* tick ครั้งก่อน                             */
static uint32_t sim_seed = 0x2545F491u;

void GenFrame(void)
{
    const float w = 6.28318531f;
    uint32_t    sum = 2166136261u;
    float       dtwin = (float)OSC_PUSH / osc_fs_eff;    /* เวลาต่อเฟรม (คุมการไถล) */

    gen_ph = fmodf(gen_ph + w * gen_freq * dtwin, w);

    for (int i = 0; i < N_SAMPLES; i++) {
        float ph  = gen_ph + w * gen_freq * (float)i / osc_fs_eff;
        float cyc = ph / w;
        float frac = cyc - floorf(cyc);
        float sig;

        switch (gen_wave) {
            case 1:  sig = (sinf(ph) >= 0.0f) ? 1.0f : -1.0f; break;   /* SQUARE  */
            case 2:  sig = 4.0f * fabsf(frac - 0.5f) - 1.0f;   break;   /* TRI     */
            case 3:  sig = 2.0f * frac - 1.0f;                 break;   /* SAW     */
            default: sig = sinf(ph);                           break;   /* SINE    */
        }
        /* noise แบบ gaussian โดยประมาณ (ผลรวม 3 ตัวแปรสุ่มสม่ำเสมอ, sd = 1) */
        float nz  = SimNoise() + SimNoise() + SimNoise();
        float volt = gen_dc + gen_amp * sig + gen_noise * nz;          /* โวลต์ */
        float raw  = volt / ADC_VREF * (float)ADC_FULL;

        if (raw < 0.0f)            raw = 0.0f;
        if (raw > (float)ADC_FULL) raw = (float)ADC_FULL;
        raw_samples[i] = (int16_t)(raw + 0.5f);
        sum = (sum ^ (uint32_t)(uint16_t)raw_samples[i]) * 16777619u;
    }

#if SHOW_FPS
    dbg_sig_hash = sum;
    dbg_t0_ms    = HAL_GetTick();
#endif
}
static float SimNoise(void)
{
    sim_seed ^= sim_seed << 13;
    sim_seed ^= sim_seed >> 17;
    sim_seed ^= sim_seed << 5;
    return ((float)((sim_seed >> 9) & 0xFFFFu) / 32768.0f) - 1.0f;
}

/* ค่าวัดทุกตัวคำนวณจากข้อมูลจริงในบัฟเฟอร์ -> ตัวเลขบนหัวจอตรงกับที่เห็น */
static void SimUpdateMeasure(void)
{
    float vmin = 1e9f, vmax = -1e9f, s1 = 0.0f, s2 = 0.0f, prev = 0.0f;
    float cross[8];
    int   ncross = 0;
    const float k = ADC_VREF / (float)ADC_FULL;

    for (int i = 0; i < N_SAMPLES; i++) {
        float v = ((float)raw_samples[i] - (float)ADC_MID) * k;
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
        s1 += v;
        s2 += v * v;
        if (i > 0 && prev < 0.0f && v >= 0.0f && ncross < 8) {   /* ตัดศูนย์ขึ้น */
            float f = (v != prev) ? (-prev / (v - prev)) : 0.0f;
            cross[ncross++] = ((float)i - 1.0f + f) / SIM_FS_HZ;
        }
        prev = v;
    }

    osc_meas.vmax = vmax;
    osc_meas.vmin = vmin;
    osc_meas.vpp  = vmax - vmin;
    osc_meas.vrms = sqrtf(s2 / (float)N_SAMPLES);
    osc_meas.dc_offset = s1 / (float)N_SAMPLES;
    if (ncross >= 2) {                                     /* วัดคาบจากจุดตัดศูนย์ */
        float span = cross[ncross - 1] - cross[0];
        if (span > 0.0f) osc_meas.freq = (float)(ncross - 1) / span;
    }
}

void SimSamplingStream(void)
{
    const float w    = 6.28318531f;
    const float step = 1.0f / SIM_FS_HZ;                   /* คาบสุ่ม = 10 us */
    uint32_t    now  = HAL_GetTick();
    uint32_t    sum  = 2166136261u;                        /* FNV-1a: ตรวจว่าข้อมูลไม่ซ้ำ */

    if (sim_prev == 0) sim_prev = now;
    float dt = (float)(now - sim_prev) / 1000.0f;   /* เวลาจริงที่ผ่านไป (ใช้กับ drift) */
    float dtwin = (float)SIM_PUSH / SIM_FS_HZ;      /* เวลาหน้าต่างต่อเฟรม = ตัวคุมความเร็วไถล */
    sim_prev = now;

    /* เดินเวลาจริง + เฟสสะสม (แบบ fmod กัน float เสียความละเอียด) */
    sim_t   = fmodf(sim_t + dt, SIM_T_WRAP);
    float f1 = SIM_FREQ_HZ + 60.0f * sinf(w * 0.23f * sim_t);   /* ความถี่ลอยช้า ๆ */
    sim_ph1 = fmodf(sim_ph1 + w * f1 * dtwin, w);   /* เฟสเดินตามหน้าต่างที่เลื่อน */
    sim_ph2 = fmodf(sim_ph2 + w * 1370.0f * dtwin, w);

    float amp = 1.00f + 0.20f * sinf(w * 0.11f * sim_t + 0.7f);  /* แอมพลิจูดหายใจช้า ๆ */
    float off = 0.15f * sinf(w * 0.07f * sim_t + 1.3f);          /* DC ลอยช้า ๆ        */

    for (int i = 0; i < N_SAMPLES; i++) {
        float ti = (float)i * step;                        /* เวลาภายในหน้าต่าง */
        float v  = amp * sinf(sim_ph1 + w * f1 * ti)
                 + 0.22f * sinf(sim_ph2 + w * 1370.0f * ti) /* โทน 2 ไม่ลงตัวกับ 1 kHz */
                 + off
                 + SIM_NOISE * SimNoise();
        float raw = (float)ADC_MID * (1.0f + SIM_SCALE * v);
        if (raw < 0.0f)            raw = 0.0f;
        if (raw > (float)ADC_FULL) raw = (float)ADC_FULL;
        raw_samples[i] = (int16_t)(raw + 0.5f);
        sum = (sum ^ (uint32_t)(uint16_t)raw_samples[i]) * 16777619u;
    }

#if SHOW_FPS
    dbg_sig_hash = sum;          /* ลายนิ้วมือของเฟรมนี้ (ใช้ตรวจว่าไม่ซ้ำ) */
    dbg_t0_ms    = now;
#endif
    SimUpdateMeasure();
}
/* --------------------------------------------------------------------------
 * GenerateFakeSineWave
 * TODO: DELETE after merging with sampling teammate.
 *       Also fills osc_meas with mock values for display testing.
 * -------------------------------------------------------------------------- */
void GenerateFakeSineWave(void)
{
    for (int i = 0; i < N_SAMPLES; i++) {
        float angle = (2.0f * 3.14159265f * (float)i / (float)N_SAMPLES) + mock_phase;
        raw_samples[i] = (int16_t)((float)ADC_MID + 1800.0f * sinf(angle));
    }
    mock_phase += 0.15f;
    if (mock_phase >= 2.0f * 3.14159265f) mock_phase -= 2.0f * 3.14159265f;

    /* Mock measurements — teammate replaces these with real calculations */
    float peak_v = 1800.0f * ADC_VREF / (float)ADC_FULL;  /* ~1.45 V */
    ((OscMeasure *)&osc_meas)->vmax     =  peak_v;
    ((OscMeasure *)&osc_meas)->vmin     = -peak_v;
    ((OscMeasure *)&osc_meas)->vpp      =  peak_v * 2.0f;
    ((OscMeasure *)&osc_meas)->vrms     =  peak_v * 0.7071f;
    ((OscMeasure *)&osc_meas)->freq     =  1000.0f;
    ((OscMeasure *)&osc_meas)->dc_offset = 0.0f;
}
