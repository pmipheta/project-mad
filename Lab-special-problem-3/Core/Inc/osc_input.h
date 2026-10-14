#ifndef OSC_INPUT_H
#define OSC_INPUT_H
#include "osc_types.h"

/* Owned by osc_input.c; private buffers/ISR state stay in that file. */
void UartPrint(const char *s);
const char *OscSetName(void);
void OscSetValueText(char *out, int n);
void UartPoll(void);
void AutoScale(void);
void HandleButtons(uint32_t now);
void OscInputInit(void);
#endif
