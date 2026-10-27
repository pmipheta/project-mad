#ifndef APP_OSC_TIMEBASE_H
#define APP_OSC_TIMEBASE_H

#include "app/app_config.h"

#define ATB_FAST_TO_SLOW_HZ  70.0f
#define ATB_SLOW_TO_FAST_HZ  90.0f

uint8_t OscFrameHold(uint32_t now);
int AtbPickSlow(float f);
void OscAutoTimebase(void);
void OscManualToggle(void);
void OscAutoRearm(void);

#endif
