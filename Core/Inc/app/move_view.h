/* move_view.h - Data behind MOVE/ZOOM of a frozen capture (points outside the window, raw copy, panned view). */
#ifndef APP_MOVE_VIEW_H
#define APP_MOVE_VIEW_H

#include "app/app_config.h"

int OscExtPoint(int k, float *v);
void MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n);
void MvFreezeCapture(void);
void MvBuildView(void);

#endif /* APP_MOVE_VIEW_H */
