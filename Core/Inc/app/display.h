/* display.h - LCD drawing: trace mapping, grid, waveform columns, cursors, setting bar, header panel. */
#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

#include "app/app_config.h"

void MapToScreen(int trig_offset);
void DrawGrid(void);
extern uint8_t cursors_on;
void ClearCursors(void);
void DrawCursors(void);
void DrawCursor(void);
void DrawWaveform(void);
void DrawSettingBar(void);
void CursorsMoveFast(void);
void HdrInvalidate(void);
void DrawTopPanel(void);
void DrawFps(float fps);

#endif /* APP_DISPLAY_H */
