/* osc_process.h - Frame processing: offset calibration, display filter, measurements (Vmax/Vmin/Vpp/RMS/F). */
#ifndef APP_OSC_PROCESS_H
#define APP_OSC_PROCESS_H

#include "app/app_config.h"

int OscFiltWin(void);
void OscProcessFrame(void);
void OscMeasureFrame(void);

#endif /* APP_OSC_PROCESS_H */
