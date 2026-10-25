/* osc_actions.h - User actions (Act*): freeze, auto, +/-, setting select, FAST/SLOW, cursor readouts, AutoScale steps. */
#ifndef APP_OSC_ACTIONS_H
#define APP_OSC_ACTIONS_H

#include "app/app_config.h"

float OscCursorV(int x);
float OscCursorDtS(void);
void MvFmtT(char *b, int n, float t_ms);
void MvFmtV(char *b, int n, float v);
void ActToggleStable(void);
void ActFreeze(void);
void ActAuto(void);
void ActToggleCoupling(void);
void ActIncrease(int n);
void ActDecrease(int n);
void ActSelectSetting(const char *name);
void ActToggleTrigEdge(void);
void ActToggleFastSlow(void);
void ActAutoMode(void);
void OscAutoScaleStep(void);

#endif /* APP_OSC_ACTIONS_H */
