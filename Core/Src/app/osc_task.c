#include "app/app.h"

volatile uint32_t dbg_osc_sleep_ms = 0;
volatile uint32_t dbg_stack_free[2];

void OscTask(void *argument)
{
    (void)argument;
    static uint32_t last_frame_tick  = 0;
    static uint32_t last_header_tick = 0;

    DrawGrid();
    DrawCursor();
    DrawTopPanel();
    xTaskNotifyGive(input_task_handle);

    for (;;)
    {
        APP_LOCK();
        uint32_t now = HAL_GetTick();

        if (adc_pend && (hold_active || osc_auto_stage || (now - adc_pd_tick) > 400u)) AdcAbortPending();
        if (osc_cur_dirty) {
            if (hold_active && cursors_on && !osc_dirty) {
                { uint32_t pq0 = DWT->CYCCNT; CursorsMoveFast(); prof_us[6] = AdcElapsedUs(pq0); }
                if ((now - last_header_tick) >= 50u && (now - last_header_tick) < HEADER_MS) last_header_tick = now - HEADER_MS;
            } else if (!hold_active) osc_cur_dirty = 0;
        }

        if (now - last_frame_tick >= FRAME_MS)
        {
            last_frame_tick = now;
            uint32_t pf0 = DWT->CYCCNT;

#if SHOW_FPS
            {
                static uint32_t t_prev = 0;
                dbg_loop_ms = (t_prev == 0) ? 0 : (now - t_prev);
                t_prev = now;
            }
#endif

            uint8_t should_draw = 0;
            if      (trigger_mode == TRIG_AUTO)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_NORM)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_SINGLE) should_draw = (!hold_active && !single_captured);
            if (osc_dirty) should_draw = 1;

            if (should_draw)
            {
                if (!hold_active) {
                    SlowSync();
                    { uint32_t pc0 = DWT->CYCCNT;
                    if (slow_active) { AdcAbortPending(); OSC_LOCK(); SlowCaptureFrame(); OSC_UNLOCK(); }
                    else             AdcCaptureFrame();
                    prof_us[0] = AdcElapsedUs(pc0); }
                    if (!osc_frame_valid) goto skip_draw;
                }
                if (!osc_frame_valid) goto skip_draw;
                uint32_t pm0 = DWT->CYCCNT;
                OscProcessFrame();
                OscMeasureFrame();
                if (osc_auto_stage) OscAutoScaleStep();
                OscAutoTimebase();
                prof_us[1] = AdcElapsedUs(pm0);
                new_frame_ready = 0;
                if (!hold_active && !osc_auto_stage && trigger_mode != TRIG_SINGLE && !slow_active) AdcPrestart();

#if SHOW_FPS
                dbg_px_frame = 0;
                uint32_t t_wf0 = HAL_GetTick();
#endif

                int trig_offset = 0;
                dbg_trig_off = trig_offset;
                uint32_t pd0 = DWT->CYCCNT;
                uint8_t hold_frame = OscFrameHold(HAL_GetTick());
                dbg_hold_cnt += hold_frame;
                ClearCursors();
                if (!hold_frame) { MapToScreen(trig_offset);
                DrawWaveform(); }
                DrawCursors();
                DrawSettingBar();
                prof_us[2] = AdcElapsedUs(pd0);
                OSC_SET(osc_dirty, 0);
                osc_cur_dirty = 0;

#if SHOW_FPS
                dbg_wf_ms = HAL_GetTick() - t_wf0;
                dbg_px_last = dbg_px_frame;
#endif

#if SHOW_FPS
                fps_frames++;
                dbg_frames_total++;
#endif

                if (trigger_mode == TRIG_SINGLE) {
                    single_captured = 1;
                    OSC_SET(hold_active    , 1);
                }
            }
            skip_draw: ;
            prof_us[3] = AdcElapsedUs(pf0);
        }

        if (now - last_header_tick >= HEADER_MS)
        {
            last_header_tick = now;

#if SHOW_FPS
            uint32_t t_h0 = HAL_GetTick();
#endif
            { uint32_t ph0 = DWT->CYCCNT; DrawTopPanel(); prof_us[4] = AdcElapsedUs(ph0); }

#if SHOW_FPS
            dbg_hdr_ms = HAL_GetTick() - t_h0;
#endif

#if SHOW_FPS
            if (now - last_fps_tick >= 500)
            {
                DrawFps((float)fps_frames * 1000.0f / (float)(now - last_fps_tick));
                dbg_fps_x10   = (uint32_t)(fps_frames * 10000u / (now - last_fps_tick));
                dbg_uptime_ms = now;
                fps_frames    = 0;

                last_fps_tick = now;
            }
#endif
            dbg_stack_free[0] = uxTaskGetStackHighWaterMark(NULL);
            dbg_stack_free[1] = uxTaskGetStackHighWaterMark(input_task_handle);
        }
        APP_UNLOCK();

        {
            uint32_t t = HAL_GetTick();
            uint32_t to_frame  = (t - last_frame_tick  >= FRAME_MS)  ? 0u : FRAME_MS  - (t - last_frame_tick);
            uint32_t to_header = (t - last_header_tick >= HEADER_MS) ? 0u : HEADER_MS - (t - last_header_tick);
            uint32_t wait = (to_frame < to_header) ? to_frame : to_header;
            if (wait) {
                (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait));
                dbg_osc_sleep_ms += HAL_GetTick() - t;
            }
        }
    }
}
