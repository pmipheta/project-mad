"""Link unmodified signal/simulator translation units against a minimal clock mock."""
from pathlib import Path
import subprocess
root = Path(__file__).resolve().parents[1]
out = Path.home()/'.hermes/cache/scratch/osc_linked'
out.mkdir(parents=True, exist_ok=True)
(out/'main.h').write_text('#include <stdint.h>\nuint32_t HAL_GetTick(void);\n')
(out/'test.c').write_text(r'''
#include "osc_signal.h"
#include "osc_simulator.h"
#include "osc_app.h"
#include <math.h>
#include <stdio.h>
int osc_fwin=1, osc_trig_fall=0;
uint8_t osc_calib=1;
volatile uint32_t dbg_sig_hash, dbg_t0_ms;
uint32_t HAL_GetTick(void) { return 100; }
int main(void) {
    int failures=0;
    gen_noise=0; gen_freq=1000; osc_fs_eff=24000;
    gen_amp=.5f; gen_dc=1.65f;
    for(int wave=0;wave<4;wave++) {
        gen_wave=wave; GenFrame(); OscProcessFrame(); OscMeasureFrame();
        if(!isfinite(osc_meas.vrms)||osc_meas.vpp<.8f) failures++;
        for(int i=0;i<N_SAMPLES;i++) if(raw_samples[i]<0||raw_samples[i]>4095) failures++;
    }
    gen_wave=0; GenFrame(); OscProcessFrame(); OscMeasureFrame();
    if(fabsf(osc_meas.freq/1000-1)>.02f) failures++;
    float fs=osc_fs_eff;
    osc_fwin=21; OscProcessFrame(); if(osc_fs_eff!=fs) failures++;
    for(int i=0;i<N_SAMPLES;i++)raw_samples[i]=3000;
    OscProcessFrame(); OscMeasureFrame();
    if(fabsf(osc_meas.vpp)>.0001f||fabsf(osc_v[0])>.0001f) failures++;
    SimSamplingStream(); GenerateFakeSineWave();
    printf("linked signal/simulator failures=%d\n",failures);
    return failures?1:0;
}
''')
objects=[]
for source in [root/'Core/Src/osc_signal.c', root/'Core/Src/osc_simulator.c',out/'test.c']:
    obj=out/(source.stem+'.o')
    subprocess.run(['clang','-std=c11','-Wall','-Wextra','-Werror','-DADC_INPUT_ENABLE=0','-I'+str(out),'-I'+str(root/'Core/Inc'),'-c',str(source),'-o',str(obj)],check=True)
    objects.append(str(obj))
subprocess.run(['clang',*objects,'-o',str(out/'test')],check=True)
subprocess.run([str(out/'test')],check=True)
