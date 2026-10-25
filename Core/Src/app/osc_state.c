/* osc_state.c - Shared oscilloscope state: settings, measurement results, display buffers, the state mutex. */
#include "app/app.h"


/* --- Sample buffer (teammate writes here) ---------------------------------- */
volatile int16_t raw_samples[N_SAMPLES];  /* ADC counts 0-4095                */
volatile uint8_t new_frame_ready = 0;     /* teammate sets 1 when buffer ready */

/* --- Measurement results (teammate writes here) ---------------------------- */
volatile OscMeasure osc_meas = {0};

/* --- v66 step 3a: mutex around the oscilloscope state shared between future tasks -------------------------------
 * Protected: osc_meas, osc_sidx, hold_active, osc_dirty, osc_stable, osc_auto_tb.  Plain (non-recursive) mutex: every
 * critical section is a strict leaf (no lock is ever taken while held).  NEVER use from an ISR (OscSlowTick etc.). */
SemaphoreHandle_t xOscMutex;
/* OSC_LOCK: defined in osc_state.h (used by other modules) */
/* OSC_UNLOCK: defined in osc_state.h (used by other modules) */
/* OSC_SET: defined in osc_state.h (used by other modules) */
void OscMeasGet(OscMeasure *o)
{
    OSC_LOCK();
    o->vmax = osc_meas.vmax; o->vmin = osc_meas.vmin; o->vpp = osc_meas.vpp;
    o->vrms = osc_meas.vrms; o->freq = osc_meas.freq; o->dc_offset = osc_meas.dc_offset;
    OSC_UNLOCK();
}

/* --- Display double-buffer ------------------------------------------------- */
int16_t old_y[N_SAMPLES];
int16_t new_y[N_SAMPLES];
/* v83 PEAK DETECT: min / max (ADC counts) of the raw samples behind each display point, valid when osc_env_on (FAST frame whose signal has
 * < 8 display points per period). MapToScreen turns them into env_y_*, TraceColumns paints the band between them. */
uint16_t osc_env_lo_ct[N_SAMPLES], osc_env_hi_ct[N_SAMPLES];
uint8_t  osc_env_on = 0;
float    osc_env_mean_ct = 0.0f, osc_env_rms_ct = 0.0f;   /* mean / AC rms of the raw samples behind the window (readout fallback) */
float    osc_env_dcrms_ct = 0.0f;   /* v90: rms of max(0, count - ADC_DC_ZERO_CT) over the same samples (DC Rs) */
int16_t  env_y_lo[N_SAMPLES], env_y_hi[N_SAMPLES];   /* screen y of the max (top) and the min (bottom) */
uint8_t  env_y_on = 0;

/* --- Display state --------------------------------------------------------- */
float volts_per_div = 1.0f;

/* Time/div labels — shown in header; actual timebase is set by teammate's timer */
#define TDIV_COUNT  7

TriggerMode trigger_mode    = TRIG_AUTO;
uint8_t     hold_active     = 0;
uint8_t     single_captured = 0;  /* 1 = SINGLE mode has captured frame */

/* --- Cursor ---------------------------------------------------------------- */
uint16_t cursor_x = SCREEN_W / 2;  /* vertical cursor at screen centre  */
/* TODO: add BTN logic to move cursor_x left/right if needed                   */

/* --- Buttons --------------------------------------------------------------- */
Button btn_up, btn_down, btn_hold, btn_set;
Button btn_vdiv, btn_tdiv, btn_filt, btn_trig;   /* v61 */

#if SHOW_FPS
uint32_t fps_frames = 0, last_fps_tick = 0;

/* globals สำหรับวัดผลจากภายนอก (อ่านผ่าน SWD) — dbg_frames_total เป็น monotonic */
volatile uint32_t dbg_frames_total = 0;
/* ข้อความที่เพิ่งส่งไปจอ LCD (อ่านผ่าน SWD เพื่อยืนยันว่า indicator ขึ้นจริง) */
volatile char     dbg_lcd_set[24] = {0};    /* ช่อง setting มุมขวาบน */
volatile char     dbg_lcd_bar[40] = {0};    /* แถบ "กำลังตั้งค่าอะไร" บรรทัดล่าง */
volatile uint32_t dbg_fps_x10      = 0;
volatile uint32_t dbg_uptime_ms    = 0;
volatile uint32_t dbg_sig_hash     = 0;   /* FNV-1a ของ raw_samples[] เฟรมล่าสุด */
volatile uint32_t dbg_t0_ms        = 0;   /* tick ของเฟรมล่าสุด */

/* ตัววัดเพื่อหาคอขวด: พิกเซลต่อเฟรม + เวลาที่ใช้ในแต่ละส่วน */
volatile uint32_t dbg_px_frame = 0;
volatile uint32_t dbg_wf_ms    = 0;
volatile uint32_t dbg_hdr_ms   = 0;
volatile uint32_t dbg_loop_ms  = 0;
volatile uint32_t dbg_px_last   = 0;    /* px/frame ที่วัดได้เต็มเฟรม */
volatile uint32_t dbg_b_spi_ms  = 0;    /* 2000 x HAL_SPI_Transmit 2 bytes */
volatile uint32_t dbg_b_px_ms   = 0;    /* 2000 x ILI9341_Draw_Pixel */
volatile uint32_t dbg_b_burst_ms= 0;    /* 20 x Draw_Rectangle 300x1 (burst) */
#endif

volatile uint32_t dbg_adc_pre = 0;   /* pre-trigger display points used by the last FAST capture */
volatile uint32_t prof_us[8];   /* PROFILING (us): 0 capture, 1 process+measure+atb, 2 draw, 3 whole frame, 4 header, 5 loop period, 6 C1/C2 move (v85) */


const float  osc_vdivs[]       = {0.0001f, 0.0002f, 0.0005f,
                                          0.001f,  0.002f,  0.005f,
                                          0.01f,   0.02f,   0.05f,
                                          0.1f,    0.2f,    0.5f, 1.0f, 2.0f};
const float  osc_tdivs_ms[]    = {0.005f, 0.01f, 0.015f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 6.0f,
                                         10.0f, 20.0f, 50.0f, 100.0f, 200.0f, 500.0f};   /* 0.015 ms/div = raw ADC samples (1.6 MSa/s); 10..500 ms/div = SLOW mode (10/20 ms/div: 30-100 Hz with 3-6 cycles instead of 15) */
const char  *osc_set_names[]   = {"FILTER", "TRIG", "V/DIV", "T/DIV"};
const char  *osc_hold_names[]  = {"C1", "C2", "V/DIV"};

/* --- oscilloscope settings --- */
int      osc_vdiv_i    = OSC_VDIV_DEF; /* -> 1.0 V/div; table reaches 0.0001 V/div */
int      osc_tdiv_i    = OSC_TDIV_DEF;  /* -> 1.0 ms/div                */
int      osc_fwin      = 5;         /* moving average window (1..21 คี่) */
int      osc_trig_fall = 0;         /* 0 = rising, 1 = falling          */
int      osc_sidx      = 0;         /* setting ที่กำลังปรับ              */
int      osc_c1        = 53, osc_c2 = 213;   /* v85: LCD x of the C1/C2 lines (0..SCREEN_W-1, 1 px per step; were sample indices 40 / 160) */
uint8_t  osc_cur_dirty = 0;         /* v85: C1/C2 moved while frozen: repaint only the cursor columns */
uint8_t  osc_calib     = 1;         /* 1 = AC coupling (calibrate offset), 0 = DC; v90: toggled by holding AUTO (PB2) */
uint8_t  osc_dirty     = 1;         /* สั่งวาดใหม่แม้อยู่โหมด HOLD         */

/* โหมดจอ (ดูโหมดที่ใช้อยู่ได้ที่บรรทัด Trig: ในหน้ารายงาน / บนจอ LCD) */
uint8_t  osc_stable    = 1;   /* v56: 1 = AUTO (default): lock to the trigger edge from 45 Hz up, free-running below / without an edge | 0 = ROLL: never lock (PC3 short) */
uint8_t  slow_trig_on  = 0;   /* v86: 1 = this SLOW frame is locked to an edge (header AUTO:R/F), 0 = it scrolls (ROLL) */
uint8_t  osc_lock_auto = 0;   /* set by the auto time base: 1 while the signal is >= 45 Hz (hysteresis 35 / 45 Hz) */   /* 1 = นิ่ง (trigger) | 0 = ไถล (roll)  */
int      dbg_trig_off  = 0;   /* เก็บไว้ตรวจย้อนหลังผ่าน SWD          */
float    osc_offset_ct = (float)ADC_MID;
float    osc_offset_v  = 0.0f;
float    osc_dc_shift_v = 0.0f;     /* v91: DC display shift (V) MapToScreen used last; 0 in AC */
float    osc_fs_eff    = 100000.0f; /* sample rate สุทธิของหน้าต่างที่แสดง */
char     osc_det[8]    = "---";     /* ชนิดคลื่นที่ตรวจได้               */
float    osc_v[N_SAMPLES];          /* ข้อมูลหลัง calibrate + filter    */
uint8_t  osc_frame_valid = 0;
volatile uint8_t slow_active = 0;  /* 1 = SLOW mode (T/DIV >= 50 ms) */
volatile uint8_t slow_fast_flag = 0; /* SLOW sees fast content inside each 1 ms tick -> signal too fast for SLOW (aliased) */
uint8_t  osc_auto_tb = 0;           /* v56: 0 = T/DIV stays where the user put it (power-on, default); 1 = firmware picks FAST/SLOW + T/DIV continuously (only the 'A' key / PC13 long sets it). 'z' is one-shot and never sets it. */
float    osc_raw_freq = 0.0f, osc_raw_vpp = 0.0f;
uint8_t  adc_f_held = 0;         /* 1 = the frequency shown is the 1.5 s HOLD of an earlier reading, not measured in this frame */
int      osc_disp_cross = 0;     /* rising zero crossings in the displayed window (0..8) */
float    osc_raw_mean_ct = 0.0f, osc_raw_rms_ct = 0.0f, osc_raw_min_ct = 0.0f, osc_raw_max_ct = 0.0f;   /* statistics over whole cycles of the raw capture (ADC counts) */
float    osc_raw_dcrms_ct = 0.0f;   /* v90: rms of max(0, count - ADC_DC_ZERO_CT) over the same whole cycles = Rs of the clipped DC trace */
uint8_t  osc_raw_valid = 0;     /* 1 = the values above belong to this frame (a clean periodic signal was found) */
float    osc_capture_tdiv = 1.0f;
uint32_t adc_measure_n = 3200u; /* adaptive, independent of display */
uint8_t  adc_meas_lvl = 0;      /* position in adc_meas_seq[] used for the frequency measurement capture */
const uint8_t adc_meas_seq[3] = { 1u, 0u, 4u };   /* 800 kSa/s (original), 1.6 MSa/s, 4.8 MSa/s (3 ADCs interleaved) */
uint8_t  adc_too_fast = 0;      /* last measurement saw a signal above the usable range of its rate */
uint8_t  adc_fast_content = 0;  /* the 800 kSa/s-or-faster capture is full of sample-to-sample change: signal is fast, not slow */
uint8_t  adc_over_range = 0;    /* frequency above the readout limit (shown as >2.1MHz) */
float    adc_last_good_vpp = 0.0f; /* Vpp of the last old-style measurement */
float    adc_last_good_f = 0.0f;   /* last confirmed reading (old measurement or display capture) */
uint32_t adc_last_good_t = 0u, adc_legacy_t = 0u;   /* its tick; tick of the last old two-capture measurement */
volatile uint32_t dbg_hold_cnt = 0;   /* frames whose waveform was not drawn (stale or unreadable FAST): SWD */
volatile uint32_t dbg_fm_cnt = 0;
volatile uint32_t dbg_fm_fail[3];     /* fast-path failures: unreadable / changed or amplitude moved / too fast (SWD) */
volatile uint32_t dbg_pipe_cnt = 0;   /* frames whose capture was started during the previous draw (SWD) */
volatile uint8_t adc_pend = 0;  /* a display capture is running (started by AdcPrestart) */
int adc_pd_tdiv, adc_pd_lvl;    /* settings it was started with */
uint32_t adc_pd_n, adc_pd_w, adc_pd_pre, adc_pd_t0, adc_pd_tick;
float adc_pd_step;      /* frames measured from the display capture (SWD) */
float    adc_hold_f = 0.0f;     /* last good reading, held for a short time so one failed frame does not blank the display */
uint32_t adc_hold_t = 0;
uint32_t adc_probe_t = 0;       /* tick of the last alias probe (at most one per second) */
float    adc_rough_ratio = 0.0f; /* mean |sample-to-sample change| / Vpp of the last measurement capture */
uint32_t adc_over_t = 0;        /* tick of the last 'above readout limit' frame (display keeps >2.1MHz for 1.5 s: no blinking) */
int      osc_plot_offset = 0;

