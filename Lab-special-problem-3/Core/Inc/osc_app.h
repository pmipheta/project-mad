#ifndef OSC_APP_H
#define OSC_APP_H
#include "osc_types.h"

/* Foreground UI settings owned by the app, edited by input/AutoScale.
 * Requested timebase is NOT the captured timebase in osc_signal.h.
 * Frame scheduler and AutoScale pending state remain private. */
void OscAutoScale(void);
void OscAppInit(void);
void OscAppStep(void);
extern float volts_per_div;
extern TriggerMode trigger_mode;
extern uint8_t     hold_active;
extern uint8_t     single_captured;
extern const float  osc_vdivs[5];
extern const float  osc_tdivs_ms[7];
extern int      osc_vdiv_i;
extern int      osc_tdiv_i;
extern int      osc_fwin;
extern int      osc_trig_fall;
extern int      osc_sidx;
extern int      osc_c1        , osc_c2;
extern uint8_t  osc_calib;
extern uint8_t  osc_dirty;
extern uint8_t  osc_stable;
extern int      osc_plot_offset;
#endif
