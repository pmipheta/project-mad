#ifndef OSC_DISPLAY_H
#define OSC_DISPLAY_H
#include "osc_types.h"

/* Owned by osc_display.c; private buffers/ISR state stay in that file. */
void MapToScreen(int trig_offset);
void ClearCursors(void);
void DrawCursors(void);
void DrawWaveform(void);
void DrawSettingBar(void);
void DrawTopPanel(void);
void DrawFps(float fps);
void OscDisplayInit(void);
extern volatile uint32_t dbg_frames_total;
extern volatile uint32_t dbg_fps_x10;
extern volatile uint32_t dbg_uptime_ms;
extern volatile uint32_t dbg_sig_hash;
extern volatile uint32_t dbg_t0_ms;
extern volatile uint32_t dbg_px_frame;
extern volatile uint32_t dbg_wf_ms;
extern volatile uint32_t dbg_hdr_ms;
extern volatile uint32_t dbg_loop_ms;
extern volatile uint32_t dbg_px_last;
#endif
