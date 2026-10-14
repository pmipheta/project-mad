#ifndef OSC_ACQUISITION_H
#define OSC_ACQUISITION_H
#include "osc_types.h"

/* Owned by osc_acquisition.c; private buffers/ISR state stay in that file. */
void AdcInputInit(void);
void AdcCaptureFrame(void);
#endif
