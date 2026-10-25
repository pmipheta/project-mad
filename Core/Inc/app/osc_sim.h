/* osc_sim.h - Signal simulator (used only with ADC_INPUT_ENABLE = 0). */
#ifndef APP_OSC_SIM_H
#define APP_OSC_SIM_H

#include "app/app_config.h"

void GenFrame(void);
void SimSamplingStream(void);
void GenerateFakeSineWave(void);

#endif /* APP_OSC_SIM_H */
