#ifndef APP_SLOW_MODE_H
#define APP_SLOW_MODE_H

#include "app/app_config.h"

#define SLOW_HIST_N       8192u

extern volatile uint16_t slow_hist[SLOW_HIST_N];
extern volatile uint32_t slow_last_t;
extern volatile uint32_t slow_valid_cnt;
extern volatile uint32_t slow_period_us;
void OscSlowTick(void);
void SlowSync(void);
int SlowExtPoint(int k, float *v);
uint8_t SlowRecentFlat(void);
uint8_t SlowSlowish(void);
float SlowProbeFreq(int ti);
void SlowCaptureFrame(void);

#endif
