/* App concern; see README.md for ownership and extension points. */
#include "osc_app.h"
#include "osc_signal.h"
#include "osc_simulator.h"
#include "osc_input.h"
#include "osc_display.h"
#include "osc_acquisition.h"
#include "main.h"
#include <math.h>

void OscAutoScale(void);
static void OscAutoScaleStep(void);
void OscAppInit(void);
void OscAppStep(void);

/* --- Display state --------------------------------------------------------- */
float volts_per_div = 1.0f;

/* Time/div labels — shown in header; actual timebase is set by teammate's timer */
#define TDIV_COUNT  7

TriggerMode trigger_mode    = TRIG_AUTO;
uint8_t     hold_active     = 0;
uint8_t     single_captured = 0;  /* 1 = SINGLE mode has captured frame */
static uint32_t fps_frames = 0, last_fps_tick = 0;
const float  osc_vdivs[]       = {0.1f, 0.2f, 0.5f, 1.0f, 2.0f};
const float  osc_tdivs_ms[]    = {0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 6.0f};
/* --- oscilloscope settings --- */
int      osc_vdiv_i    = 3;         /* -> 1.0 V/div                     */
int      osc_tdiv_i    = 3;         /* -> 1.0 ms/div                    */
int      osc_fwin      = 5;         /* moving average window (1..21 คี่) */
int      osc_trig_fall = 0;         /* 0 = rising, 1 = falling          */
int      osc_sidx      = 0;         /* setting ที่กำลังปรับ              */
int      osc_c1        = 40, osc_c2 = 160;
uint8_t  osc_calib     = 1;         /* AC coupling (calibrate offset)   */
uint8_t  osc_dirty     = 1;         /* สั่งวาดใหม่แม้อยู่โหมด HOLD         */

/* โหมดจอ (ดูโหมดที่ใช้อยู่ได้ที่บรรทัด Trig: ในหน้ารายงาน / บนจอ LCD) */
uint8_t  osc_stable    = 1;   /* 1 = นิ่ง (trigger) | 0 = ไถล (roll)  */
static int      trig_hold     = 0;   /* trigger offset ล่าสุดที่ใช้          */
static int      dbg_trig_off  = 0;   /* เก็บไว้ตรวจย้อนหลังผ่าน SWD          */
int      osc_plot_offset = 0;
static uint8_t osc_auto_stage = 0;
volatile float dbg_auto_fsel = 0.0f;
static uint32_t last_frame_tick = 0, last_header_tick = 0;

void OscAutoScale(void)
{
    if (hold_active) return; /* HOLD never replaces or retimes its capture. */
    osc_auto_stage = 1;
    osc_dirty = 1;
    UartPrint("\r\n AutoScale...\r\n");
}

static void OscAutoScaleStep(void)
{
    if (!osc_auto_stage || hold_active || !osc_frame_valid || !new_frame_ready) return;
    float f = osc_meas.freq;
    if (!(f > 0.0f)) return; /* adaptive measurement still acquiring periods */
    float vpp = ADC_INPUT_ENABLE ? osc_raw_vpp : osc_meas.vpp;
    int vi = (int)(sizeof(osc_vdivs) / sizeof(osc_vdivs[0])) - 1;
    for (int i = 0; i <= vi; i++) {
        if (osc_vdivs[i] * OSC_V_FILL_DIVS >= vpp) { vi = i; break; }
    }
    int ti = 0;
    float best = 1.0e30f;
    for (int i = 0; i < OSC_NT; i++) {
        float d = fabsf(f * osc_tdivs_ms[i] * OSC_H_DIVS / 1000.0f - OSC_FIT_CYCLES);
        if (d < best) { best = d; ti = i; }
    }
    osc_vdiv_i = vi;
    volts_per_div = osc_vdivs[vi];
    osc_tdiv_i = ti; /* captured time metadata changes only on acquisition */
    dbg_auto_fsel = f;
    osc_auto_stage = 0;
    osc_dirty = 1;
}

void OscAppInit(void)
{
#if ADC_INPUT_ENABLE
    AdcInputInit();
#endif
    OscInputInit();
    OscDisplayInit();
}

void OscAppStep(void)
{
        uint32_t now = HAL_GetTick();

        /* --- Button handling (every loop) --------------------------------- */
        HandleButtons(now);
        UartPoll();               /* รับคำสั่งจาก USART3 (แทนปุ่มจริง) */


        /* --- Waveform frame update (~30 FPS) ------------------------------ */
        if (now - last_frame_tick >= FRAME_MS)
        {
            last_frame_tick = now;

#if SHOW_FPS
            {
                static uint32_t t_prev = 0;
                dbg_loop_ms = (t_prev == 0) ? 0 : (now - t_prev);
                t_prev = now;
            }
#endif

            /* Determine if we should redraw this frame */
            uint8_t should_draw = 0;
            if      (trigger_mode == TRIG_AUTO)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_NORM)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_SINGLE) should_draw = (!hold_active && !single_captured);
            if (osc_dirty) should_draw = 1;      /* ปรับ setting ขณะ HOLD -> วาดใหม่ 1 เฟรม */

            if (should_draw)
            {
                /* SIM_STREAM_MODE: 2 = function generator (ตาม simulator, ค่าเริ่มต้น)
                 *                  1 = สตรีมต่อเนื่องเดิม   0 = sine เดิมที่วนลูป
                 * ทีม sampling ต่อข้อมูลจริงตรงนี้ (แทน GenFrame)                    */
                if (!hold_active) {               /* HOLD = freeze ข้อมูลไว้ ไม่ generate ใหม่ */
#if ADC_INPUT_ENABLE
                    AdcCaptureFrame();            /* p30: อ่านข้อมูลจริงจาก ADC (PA0) */
                    if (!osc_frame_valid) goto skip_draw;
#else
                    osc_capture_tdiv = osc_tdivs_ms[osc_tdiv_i];
                    osc_fs_eff = N_SAMPLES / (osc_capture_tdiv * OSC_H_DIVS / 1000.0f);
                    if      (SIM_STREAM_MODE == 2) GenFrame();
                    else if (SIM_STREAM_MODE == 1) SimSamplingStream();
                    else                           GenerateFakeSineWave();
                    osc_frame_valid = 1;
                    new_frame_ready = 1;
#endif
                }
                if (!osc_frame_valid) goto skip_draw;
                OscProcessFrame();      /* calibrate offset (AC) + moving average filter */
                OscMeasureFrame();      /* วัดค่า Vmax/Vmin/Vpp/RMS/Freq + ชนิดคลื่น */
                /* AutoScale ขั้น 2: พอวัดใหม่ที่ time base เร็วสุดแล้ว -> เลือกสเกล */
                if (osc_auto_stage) OscAutoScaleStep();
                new_frame_ready = 0;

#if SHOW_FPS
                dbg_px_frame = 0;
                uint32_t t_wf0 = HAL_GetTick();
#endif

                /* osc_stable = 1: จัดแนว trigger (ใช้ขอบ RISE/FALL ตามค่า TRIG) -> กราฟนิ่ง
                 * osc_stable = 0: ไม่จัดแนว -> ข้อมูลใหม่เข้าขวา เส้นไถลซ้าย (roll) */
                int trig_offset = 0;
                if (osc_stable) {
#if ADC_INPUT_ENABLE
                    /* p38: จัดแนว trigger ตั้งแต่ตอนจับข้อมูล (AdcCaptureFrame) แล้ว
                     * -> หน้าต่างเริ่มที่จุด trigger จริง ไม่ต้องหาใหม่/ไม่ต้องวนกลับบัฟเฟอร์
                     *    (เดิม FindTrigger หาใหม่แล้วได้คนละจุด ทำให้จุดวนกลับเลื่อนกลางจอ) */
                    trig_offset = 0;
                    trig_hold   = 0;
                    (void)FindTrigger;   /* p39: โหมด ADC จัดแนวตอนจับข้อมูลแล้ว */
#else
                    int t = FindTrigger();
                    if (t >= 0) {
                        trig_hold   = t;
                        trig_offset = t;
                    } else {
                        /* หน้าต่างสั้นกว่าหนึ่งคาบ (ความถี่ต่ำกว่า time base) -> หาขอบไม่ได้
                         * เดิมใช้ค่า trig_hold เก่า => รูปวาดไม่ตรงจุด -> กระโดด/ขาด
                         * ใหม่: วาดแบบไถล (offset 0) ต่อเนื่องแทน */
                        trig_offset = 0;
                    }
#endif
                }
                dbg_trig_off = trig_offset;
                ClearCursors();         /* ล้างเส้น cursor เก่าก่อนวาดเฟรมใหม่ */
                MapToScreen(trig_offset);
                DrawWaveform();
                DrawCursors();          /* HOLD: วาดเส้น C1/C2 ทับหลังวาดรูป */
                DrawSettingBar();       /* แถบ "กำลังตั้งค่าอะไร" ทับหลังวาดรูป */
                osc_dirty = 0;

#if SHOW_FPS
                dbg_wf_ms = HAL_GetTick() - t_wf0;
                dbg_px_last = dbg_px_frame;
#endif

#if SHOW_FPS
                fps_frames++;          /* นับเฉพาะเฟรมที่วาดจริง */
                dbg_frames_total++;
#endif

                /* SINGLE mode: lock after first captured frame */
                if (trigger_mode == TRIG_SINGLE) {
                    single_captured = 1;
                    hold_active     = 1;
                }
            }
            skip_draw: ;
        }

        /* --- Header update (every 300 ms) --------------------------------- */
        if (now - last_header_tick >= HEADER_MS)
        {
            last_header_tick = now;

#if SHOW_FPS
            uint32_t t_h0 = HAL_GetTick();
#endif
            DrawTopPanel();

#if SHOW_FPS
            dbg_hdr_ms = HAL_GetTick() - t_h0;
#endif

#if SHOW_FPS
            /* FPS จริงทุก ~500 ms (ผูกกับรอบ refresh header) */
            if (now - last_fps_tick >= 500)
            {
                DrawFps((float)fps_frames * 1000.0f / (float)(now - last_fps_tick));
                dbg_fps_x10   = (uint32_t)(fps_frames * 10000u / (now - last_fps_tick));
                dbg_uptime_ms = now;
                fps_frames    = 0;

                last_fps_tick = now;
            }
#endif
        }
}
