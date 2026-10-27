#include "app/app.h"

volatile int16_t raw_samples[N_SAMPLES];
volatile uint8_t new_frame_ready = 0;

volatile OscMeasure osc_meas = {0};

SemaphoreHandle_t xOscMutex;
void OscMeasGet(OscMeasure *o)
{
    OSC_LOCK();
    o->vmax = osc_meas.vmax; o->vmin = osc_meas.vmin; o->vpp = osc_meas.vpp;
    o->vrms = osc_meas.vrms; o->freq = osc_meas.freq; o->dc_offset = osc_meas.dc_offset;
    OSC_UNLOCK();
}

int16_t old_y[N_SAMPLES];
int16_t new_y[N_SAMPLES];
uint16_t osc_env_lo_ct[N_SAMPLES], osc_env_hi_ct[N_SAMPLES];
uint8_t  osc_env_on = 0;
float    osc_env_mean_ct = 0.0f, osc_env_rms_ct = 0.0f;
float    osc_env_dcrms_ct = 0.0f;
int16_t  env_y_lo[N_SAMPLES], env_y_hi[N_SAMPLES];
uint8_t  env_y_on = 0;

float volts_per_div = 1.0f;

#define TDIV_COUNT  7

TriggerMode trigger_mode    = TRIG_AUTO;
uint8_t     hold_active     = 0;
uint8_t     single_captured = 0;

uint16_t cursor_x = SCREEN_W / 2;

Button btn_up, btn_down, btn_hold, btn_set;
Button btn_vdiv, btn_tdiv, btn_filt, btn_trig;

#if SHOW_FPS
uint32_t fps_frames = 0, last_fps_tick = 0;

volatile uint32_t dbg_frames_total = 0;
volatile char     dbg_lcd_set[24] = {0};
volatile char     dbg_lcd_bar[40] = {0};
volatile uint32_t dbg_fps_x10      = 0;
volatile uint32_t dbg_uptime_ms    = 0;
volatile uint32_t dbg_sig_hash     = 0;
volatile uint32_t dbg_t0_ms        = 0;

volatile uint32_t dbg_px_frame = 0;
volatile uint32_t dbg_wf_ms    = 0;
volatile uint32_t dbg_hdr_ms   = 0;
volatile uint32_t dbg_loop_ms  = 0;
volatile uint32_t dbg_px_last   = 0;
volatile uint32_t dbg_b_spi_ms  = 0;
volatile uint32_t dbg_b_px_ms   = 0;
volatile uint32_t dbg_b_burst_ms= 0;
#endif

volatile uint32_t dbg_adc_pre = 0;
volatile uint32_t prof_us[8];

const float  osc_vdivs[]       = {0.0001f, 0.0002f, 0.0005f,
                                          0.001f,  0.002f,  0.005f,
                                          0.01f,   0.02f,   0.05f,
                                          0.1f,    0.2f,    0.5f, 1.0f, 2.0f};
const float  osc_tdivs_ms[]    = {0.005f, 0.01f, 0.015f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 6.0f,
                                         10.0f, 20.0f, 50.0f, 100.0f, 200.0f, 500.0f};
const char  *osc_set_names[]   = {"FILTER", "TRIG", "V/DIV", "T/DIV"};
const char  *osc_hold_names[]  = {"C1", "C2", "V/DIV"};

int      osc_vdiv_i    = OSC_VDIV_DEF;
int      osc_tdiv_i    = OSC_TDIV_DEF;
int      osc_fwin      = 5;
int      osc_trig_fall = 0;
int      osc_sidx      = 0;
int      osc_c1        = 53, osc_c2 = 213;
uint8_t  osc_cur_dirty = 0;
uint8_t  osc_calib     = 1;
uint8_t  osc_dirty     = 1;

uint8_t  osc_stable    = 1;
uint8_t  slow_trig_on  = 0;
uint8_t  osc_lock_auto = 0;
int      dbg_trig_off  = 0;
float    osc_offset_ct = (float)ADC_MID;
float    osc_offset_v  = 0.0f;
float    osc_dc_shift_v = 0.0f;
float    osc_fs_eff    = 100000.0f;
char     osc_det[8]    = "---";
float    osc_v[N_SAMPLES];
uint8_t  osc_frame_valid = 0;
volatile uint8_t slow_active = 0;
volatile uint8_t slow_fast_flag = 0;
uint8_t  osc_auto_tb = 0;
float    osc_raw_freq = 0.0f, osc_raw_vpp = 0.0f;
uint8_t  adc_f_held = 0;
int      osc_disp_cross = 0;
float    osc_raw_mean_ct = 0.0f, osc_raw_rms_ct = 0.0f, osc_raw_min_ct = 0.0f, osc_raw_max_ct = 0.0f;
float    osc_raw_dcrms_ct = 0.0f;
uint8_t  osc_raw_valid = 0;
float    osc_capture_tdiv = 1.0f;
uint32_t adc_measure_n = 3200u;
uint8_t  adc_meas_lvl = 0;
const uint8_t adc_meas_seq[3] = { 1u, 0u, 4u };
uint8_t  adc_too_fast = 0;
uint8_t  adc_fast_content = 0;
uint8_t  adc_over_range = 0;
float    adc_last_good_vpp = 0.0f;
float    adc_last_good_f = 0.0f;
uint32_t adc_last_good_t = 0u, adc_legacy_t = 0u;
volatile uint32_t dbg_hold_cnt = 0;
volatile uint32_t dbg_fm_cnt = 0;
volatile uint32_t dbg_fm_fail[3];
volatile uint32_t dbg_pipe_cnt = 0;
volatile uint8_t adc_pend = 0;
int adc_pd_tdiv, adc_pd_lvl;
uint32_t adc_pd_n, adc_pd_w, adc_pd_pre, adc_pd_t0, adc_pd_tick;
float adc_pd_step;
float    adc_hold_f = 0.0f;
uint32_t adc_hold_t = 0;
uint32_t adc_probe_t = 0;
float    adc_rough_ratio = 0.0f;
uint32_t adc_over_t = 0;
int      osc_plot_offset = 0;
