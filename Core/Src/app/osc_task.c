/* osc_task.c - The oscilloscope task: capture -> process -> draw loop. */
#include "app/app.h"

volatile uint32_t dbg_osc_sleep_ms = 0;      /* total time OscTask slept (SWD): the CPU time the old busy loop burnt */
volatile uint32_t dbg_stack_free[2];         /* stack high-water marks in words: [0] OscTask, [1] InputTask */

/* The former main-loop body. Each pass runs under the app lock (InputTask applies buttons/commands only between passes,
 * as the single loop did); between passes the task sleeps until the next frame or header is due, or until InputTask
 * has polled the inputs (then the pass sees the new state at once, as the old loop did on its next spin). */
void OscTask(void *argument)
{
    (void)argument;
    static uint32_t last_frame_tick  = 0;
    static uint32_t last_header_tick = 0;

    /* v66b: one-shot start-up, runs AFTER vTaskStartScheduler() so xSemaphoreTake/Give are legal
     * (xOscMutex was created in main() before xTaskCreate; creation does not block, only Take/Give must wait for the scheduler). */
    DrawGrid();
    DrawCursor();
    DrawTopPanel();
    xTaskNotifyGive(input_task_handle);   /* start-up screen drawn: InputTask may start polling */

    for (;;)
    {
        APP_LOCK();
        uint32_t now = HAL_GetTick();

        if (adc_pend && (hold_active || osc_auto_stage || (now - adc_pd_tick) > 400u)) AdcAbortPending();   /* nobody collected it: give the ADC and the 1 kHz sampler back */
        /* --- Buttons: InputTask (input_task.c), between two passes --- */
        if (osc_cur_dirty) {      /* v85: C1/C2 moved while frozen: repaint the cursor columns now, not on the 33 ms frame */
            if (hold_active && cursors_on && !osc_dirty) {
                { uint32_t pq0 = DWT->CYCCNT; CursorsMoveFast(); prof_us[6] = AdcElapsedUs(pq0); }
                if ((now - last_header_tick) >= 50u && (now - last_header_tick) < HEADER_MS) last_header_tick = now - HEADER_MS;   /* U1/U2/dV/dt follow */
            } else if (!hold_active) osc_cur_dirty = 0;
        }


        /* --- Waveform frame update (~30 FPS) ------------------------------ */
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

            /* Determine if we should redraw this frame */
            uint8_t should_draw = 0;
            if      (trigger_mode == TRIG_AUTO)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_NORM)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_SINGLE) should_draw = (!hold_active && !single_captured);
            if (osc_dirty) should_draw = 1;      /* ปรับ setting ขณะ HOLD -> วาดใหม่ 1 เฟรม */

            if (should_draw)
            {
                if (!hold_active) {               /* HOLD = freeze: no new capture */
                    SlowSync();                   /* T/DIV >= 50 ms -> SLOW (ring), ไม่งั้น FAST (burst) */
                    { uint32_t pc0 = DWT->CYCCNT;
                    if (slow_active) { AdcAbortPending(); OSC_LOCK(); SlowCaptureFrame(); OSC_UNLOCK(); }
                    else             AdcCaptureFrame();   /* p30: อ่านข้อมูลจริงจาก ADC (PA0) */
                    prof_us[0] = AdcElapsedUs(pc0); }
                    if (!osc_frame_valid) goto skip_draw;
                }
                if (!osc_frame_valid) goto skip_draw;
                uint32_t pm0 = DWT->CYCCNT;
                OscProcessFrame();      /* calibrate offset (AC) + moving average filter */
                OscMeasureFrame();      /* วัดค่า Vmax/Vmin/Vpp/RMS/Freq + ชนิดคลื่น */
                /* AutoScale ขั้น 2: พอวัดใหม่ที่ time base เร็วสุดแล้ว -> เลือกสเกล */
                if (osc_auto_stage) OscAutoScaleStep();
                OscAutoTimebase();      /* AUTO: สลับ FAST/SLOW + T/DIV ตามความถี่ (ถ้าไม่ได้ปรับเอง) */
                prof_us[1] = AdcElapsedUs(pm0);
                new_frame_ready = 0;
                if (!hold_active && !osc_auto_stage && trigger_mode != TRIG_SINGLE && !slow_active) AdcPrestart();   /* the next capture runs by DMA while we draw */

#if SHOW_FPS
                dbg_px_frame = 0;
                uint32_t t_wf0 = HAL_GetTick();
#endif
                /* ============================================================ */

                /* osc_stable = 1: จัดแนว trigger (ใช้ขอบ RISE/FALL ตามค่า TRIG) -> กราฟนิ่ง
                 * osc_stable = 0: ไม่จัดแนว -> ข้อมูลใหม่เข้าขวา เส้นไถลซ้าย (roll) */
                int trig_offset = 0;    /* p38: the capture (AdcCaptureFrame / SlowCaptureFrame) already starts at the trigger edge */
                dbg_trig_off = trig_offset;
                uint32_t pd0 = DWT->CYCCNT;
                uint8_t hold_frame = OscFrameHold(HAL_GetTick());
                dbg_hold_cnt += hold_frame;
                ClearCursors();         /* ล้างเส้น cursor เก่าก่อนวาดเฟรมใหม่ */
                if (!hold_frame) { MapToScreen(trig_offset);
                DrawWaveform(); }
                DrawCursors();          /* HOLD: วาดเส้น C1/C2 ทับหลังวาดรูป */
                DrawSettingBar();       /* แถบ "กำลังตั้งค่าอะไร" ทับหลังวาดรูป */
                prof_us[2] = AdcElapsedUs(pd0);
                OSC_SET(osc_dirty, 0);
                osc_cur_dirty = 0;      /* v85: the full redraw drew the cursors too */

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
                    OSC_SET(hold_active    , 1);
                }
            }
            skip_draw: ;
            prof_us[3] = AdcElapsedUs(pf0);
        }

        /* --- Header update (every 300 ms) --------------------------------- */
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
            dbg_stack_free[0] = uxTaskGetStackHighWaterMark(NULL);
            dbg_stack_free[1] = uxTaskGetStackHighWaterMark(input_task_handle);
        }
        APP_UNLOCK();

        /* --- Sleep until the next frame / header is due (InputTask wakes us after every poll) --- */
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
