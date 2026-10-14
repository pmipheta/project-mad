#ifndef OSC_TYPES_H
#define OSC_TYPES_H
#include <stdint.h>
#include "osc_config.h"
/* Trigger modes */
typedef enum { TRIG_AUTO = 0, TRIG_SINGLE, TRIG_NORM } TriggerMode;

/* Measurement values — teammate populates this after each ADC frame */
typedef struct {
    float vmax;
    float vmin;
    float vpp;
    float vrms;
    float freq;
    float dc_offset;
} OscMeasure;

#endif
