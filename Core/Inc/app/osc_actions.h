/* osc_actions.h - User actions (Act*): freeze, auto, up/down, MOVE/ZOOM of a frozen capture, AutoScale steps. */
#ifndef APP_OSC_ACTIONS_H
#define APP_OSC_ACTIONS_H

#include "app/app_config.h"

float OscCursorV(int x);
float OscCursorDtS(void);
void MvSync(void);
void ActZoom(int dir);
void MvFmtT(char *b, int n, float t_ms);
void MvFmtV(char *b, int n, float v);
void MvFmtZ(char *b, int n);
void ActToggleStable(void);
void ActFreeze(void);
void ActAuto(void);
void ActToggleCoupling(void);
void ActUp(int n);
void ActDown(int n);
void ActIncrease(int n);
void ActDecrease(int n);
void ActSelectSetting(const char *name);
void ActToggleTrigEdge(void);
void ActToggleMoveAxis(void);
void ActMoveCenter(void);
void ActToggleFastSlow(void);
void ActAutoMode(void);
void OscAutoScaleStep(void);

#endif /* APP_OSC_ACTIONS_H */
