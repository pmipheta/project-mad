#ifndef OSC_SIMULATOR_H
#define OSC_SIMULATOR_H
#include "osc_types.h"

/* Owned by osc_simulator.c; private buffers/ISR state stay in that file. */
void GenFrame(void);
void SimSamplingStream(void);
void GenerateFakeSineWave(void);
extern const char  *gen_wave_names[4];
extern int      gen_wave;
extern float    gen_freq;
extern float    gen_amp;
extern float    gen_dc;
extern float    gen_noise;
extern float    gen_ph;
#endif
