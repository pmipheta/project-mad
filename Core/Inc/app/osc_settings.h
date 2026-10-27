#ifndef APP_OSC_SETTINGS_H
#define APP_OSC_SETTINGS_H

#include "app/app_config.h"

int OscSetCount(void);
const char *OscSetName(void);
int OscCursorSelected(void);
int OscClampI(int v, int lo, int hi);
float OscClampF(float v, float lo, float hi);
void OscSetValueText(char *out, int n);
void OscSetIndex(int idx);
extern uint8_t osc_auto_stage;
void OscAdjust(int dir);
extern uint8_t osc_auto_tries;
extern volatile float dbg_auto_fsel;
void OscAutoScale(void);

#endif
