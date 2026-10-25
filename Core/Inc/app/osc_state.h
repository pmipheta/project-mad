/* osc_state.h - Shared oscilloscope state: settings, measurement results, display buffers, the state mutex. */
#ifndef APP_OSC_STATE_H
#define APP_OSC_STATE_H

#include "app/app_config.h"

#define OSC_LOCK()    ((void)xSemaphoreTake(xOscMutex, portMAX_DELAY))
#define OSC_UNLOCK()  ((void)xSemaphoreGive(xOscMutex))
#define OSC_SET(v, x) do { OSC_LOCK(); (v) = (x); OSC_UNLOCK(); } while (0)

extern volatile int16_t raw_samples[N_SAMPLES];
extern volatile uint8_t new_frame_ready;
extern volatile OscMeasure osc_meas;
extern SemaphoreHandle_t xOscMutex;
void OscMeasGet(OscMeasure *o);
extern int16_t old_y[N_SAMPLES];
extern int16_t new_y[N_SAMPLES];
extern uint16_t osc_env_lo_ct[N_SAMPLES], osc_env_hi_ct[N_SAMPLES];
extern uint8_t osc_env_on;
extern float osc_env_mean_ct, osc_env_rms_ct;
extern float osc_env_dcrms_ct;
extern int16_t env_y_lo[N_SAMPLES], env_y_hi[N_SAMPLES];
extern uint8_t env_y_on;
extern float volts_per_div;
extern TriggerMode trigger_mode;
extern uint8_t hold_active;
extern uint8_t single_captured;
extern uint16_t cursor_x;
extern Button btn_up, btn_down, btn_hold, btn_set;
extern Button btn_vdiv, btn_tdiv, btn_filt, btn_trig;
extern float mock_phase;
extern uint32_t fps_frames, last_fps_tick;
extern volatile uint32_t dbg_frames_total;
extern volatile char dbg_lcd_set[24];
extern volatile char dbg_lcd_bar[40];
extern volatile uint32_t dbg_fps_x10;
extern volatile uint32_t dbg_uptime_ms;
extern volatile uint32_t dbg_sig_hash;
extern volatile uint32_t dbg_t0_ms;
extern volatile uint32_t dbg_px_frame;
extern volatile uint32_t dbg_wf_ms;
extern volatile uint32_t dbg_hdr_ms;
extern volatile uint32_t dbg_loop_ms;
extern volatile uint32_t dbg_px_last;
extern volatile uint32_t dbg_b_spi_ms;
extern volatile uint32_t dbg_b_px_ms;
extern volatile uint32_t dbg_b_burst_ms;
extern volatile uint32_t dbg_adc_pre;
extern volatile uint32_t prof_us[8];
extern const char *gen_wave_names[4];
extern const float osc_vdivs[14];
extern const float osc_tdivs_ms[18];
extern const char *osc_set_names[4];
extern const char *osc_hold_names[3];
extern int gen_wave;
extern float gen_freq;
extern float gen_amp;
extern float gen_dc;
extern float gen_noise;
extern float gen_ph;
extern int osc_vdiv_i;
extern int osc_tdiv_i;
extern int osc_fwin;
extern int osc_trig_fall;
extern int osc_sidx;
extern int osc_c1, osc_c2;
extern uint8_t osc_cur_dirty;
extern uint8_t osc_calib;
extern uint8_t osc_dirty;
extern uint8_t osc_stable;
extern uint8_t slow_trig_on;
extern uint8_t osc_lock_auto;
extern int trig_hold;
extern int dbg_trig_off;
extern float osc_offset_ct;
extern float osc_offset_v;
extern float osc_dc_shift_v;
extern float osc_fs_eff;
extern char osc_det[8];
extern float osc_v[N_SAMPLES];
extern uint8_t osc_frame_valid;
extern volatile uint8_t slow_active;
extern volatile uint8_t slow_fast_flag;
extern uint8_t osc_auto_tb;
extern float osc_raw_freq, osc_raw_vpp;
extern uint8_t adc_f_held;
extern int osc_disp_cross;
extern float osc_raw_mean_ct, osc_raw_rms_ct, osc_raw_min_ct, osc_raw_max_ct;
extern float osc_raw_dcrms_ct;
extern uint8_t osc_raw_valid;
extern float osc_capture_tdiv;
extern uint32_t adc_measure_n;
extern uint8_t adc_meas_lvl;
extern const uint8_t adc_meas_seq[3];
extern uint8_t adc_too_fast;
extern uint8_t adc_fast_content;
extern uint8_t adc_over_range;
extern float adc_last_good_vpp;
extern float adc_last_good_f;
extern uint32_t adc_last_good_t, adc_legacy_t;
extern volatile uint32_t dbg_hold_cnt;
extern volatile uint32_t dbg_fm_cnt;
extern volatile uint32_t dbg_fm_fail[3];
extern volatile uint32_t dbg_pipe_cnt;
extern volatile uint8_t adc_pend;
extern int adc_pd_tdiv, adc_pd_lvl;
extern uint32_t adc_pd_n, adc_pd_w, adc_pd_pre, adc_pd_t0, adc_pd_tick;
extern float adc_pd_step;
extern float adc_hold_f;
extern uint32_t adc_hold_t;
extern uint32_t adc_probe_t;
extern float adc_rough_ratio;
extern uint32_t adc_over_t;
extern int osc_plot_offset;
extern int mv_y_steps;
extern int mv_x_pts;
extern int mv_x_min, mv_x_max;
extern uint8_t mv_axis_x;
extern uint8_t mv_prev_hold;
extern uint8_t mv_view_on;
extern uint8_t mv_zoom_x;
extern uint8_t mv_zoom_step;
extern uint8_t mv_zoom_y;
extern int mv_x_q;
extern float mv_view_v[N_SAMPLES];

#endif /* APP_OSC_STATE_H */
