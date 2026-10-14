#ifndef OSC_SIGNAL_H
#define OSC_SIGNAL_H
#include "osc_types.h"

/* Frame exchange: acquisition/simulator write raw data and captured timing.
 * Processing writes calibrated volts/measurements; display/input only read.
 * Called in the foreground; ISR ownership remains in acquisition/input. */
void OscProcessFrame(void);
void OscMeasureFrame(void);
int FindTrigger(void);
extern volatile int16_t raw_samples[N_SAMPLES];
extern volatile uint8_t new_frame_ready;
extern volatile OscMeasure osc_meas;
extern float    osc_fs_eff;
extern float    osc_offset_ct;
extern char     osc_det[8];
extern float    osc_v[N_SAMPLES];
extern uint8_t  osc_frame_valid;
extern float    osc_raw_freq , osc_raw_vpp;
extern float    osc_capture_tdiv;
#endif
