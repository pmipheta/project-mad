/* ext_points.h - Display points just outside the FAST window (real neighbours for the display filter at the screen edges). */
#ifndef APP_EXT_POINTS_H
#define APP_EXT_POINTS_H

#include "app/app_config.h"

int OscExtPoint(int k, float *v);
void MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n);

#endif /* APP_EXT_POINTS_H */
