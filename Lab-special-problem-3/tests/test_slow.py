"""Host test of the SLOW mode sampler (T/DIV >= 50 ms) extracted from Core/Src/main.c.
The ADC is modelled as an ideal 12-bit converter; SysTick is called once per simulated ms. Not a board test."""
from pathlib import Path
import subprocess
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__import__('tempfile').gettempdir()) / 'osc_slow'
OUT.mkdir(parents=True, exist_ok=True)
src = (ROOT/'Core/Src/main.c').read_text()
a, b = src.index('SLOW_BEGIN'), src.index('SLOW_END')
block = src[src.rfind('/*', 0, a):src.index('\n', b)+1]
(OUT/'slow_extract.c').write_text(block)
harness = r'''
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#define OSC_SLOW_HOST_TEST 1
#define N_SAMPLES 240
#define OSC_H_DIVS 10
#define ADC_VREF 3.3f
#define ADC_FULL 4095
typedef struct {float vmax,vmin,vpp,vrms,dc_offset,freq;} OscMeasure;
static OscMeasure osc_meas; static char osc_det[8];
static const float osc_tdivs_ms[] = {0.1f,0.2f,0.5f,1.0f,2.0f,5.0f,6.0f,50.0f,100.0f,200.0f,500.0f};
static int osc_tdiv_i = 3;
static volatile int16_t raw_samples[N_SAMPLES];
static float osc_fs_eff, osc_raw_freq, osc_raw_vpp, osc_capture_tdiv;
static volatile uint16_t dbg_adc_min, dbg_adc_max; static volatile uint32_t dbg_adc_frames; static volatile uint8_t dbg_adc_ok;
static volatile uint8_t new_frame_ready, osc_dirty, osc_frame_valid, slow_active, slow_fast_flag;
static uint32_t __get_PRIMASK(void){return 0;} static void __disable_irq(void){} static void __set_PRIMASK(uint32_t x){(void)x;}
static double now_s, F_HZ=1.0, AMP=0.3, DC=1.65, NOISE=0.0;
static unsigned rs=12345; static double rnd(void){rs=rs*1103515245u+12345u; return ((rs>>8)&0xFFFF)/65535.0-0.5;}
static int burst_k;   /* consecutive conversions inside one tick are 1.3 us apart */
static inline uint16_t SlowAdcRead(void){ double v=DC+AMP*sin(2*M_PI*F_HZ*(now_s+1.3e-6*burst_k++))+NOISE*rnd(); if(v<0)v=0; if(v>3.3)v=3.3; return (uint16_t)(v/3.3*4095.0+0.5);}
#include "slow_extract.c"
static int failures;
static void run(double secs){ for(double t=0;t<secs*1000;t+=1){ now_s+=0.001; burst_k=0; OscSlowTick(); if(((int)t)%33==0){ SlowSync(); if(slow_active) SlowCaptureFrame(); } } }
static void expect_f(const char*n,double f_true,int ti,double secs,double tol){
  osc_tdiv_i=ti; F_HZ=f_true; run(secs);
  int ok = osc_frame_valid && fabs(osc_raw_freq-f_true) <= tol*f_true;
  printf("%s %-26s tdiv=%3.0fms Fs=%6.1f Vpp=%.3f measured=%.3f Hz (true %.2f)\n", ok?"PASS":"FAIL", n, osc_tdivs_ms[ti], osc_fs_eff, osc_raw_vpp, osc_raw_freq, f_true);
  if(!ok) failures++;
}
static void expect_zero(const char*n,int ti,double secs){
  osc_tdiv_i=ti; run(secs); int ok = osc_frame_valid && osc_raw_freq==0.0f;
  printf("%s %-26s measured=%.3f Hz (must be 0, never invented)\n", ok?"PASS":"FAIL", n, osc_raw_freq); if(!ok) failures++;
}
static void expect_zero_flag(const char*n,double f_true,int ti,double secs,int want_flag){
  osc_tdiv_i=ti; F_HZ=f_true; run(secs); int ok = osc_frame_valid && osc_raw_freq==0.0f && slow_fast_flag==want_flag;
  printf("%s %-36s measured=%.3f Hz flag=%d spread=%u (must read 0 Hz; flag expected=%d)\n", ok?"PASS":"FAIL", n, osc_raw_freq, slow_fast_flag, (unsigned)dbg_slow_spread, want_flag); if(!ok) failures++;
}
int main(void){
  osc_tdiv_i=9; run(0.01);
  expect_f("1 Hz @200 ms/div",1.0,9,2.8,0.03);
  expect_f("1 Hz @500 ms/div",1.0,10,6.0,0.03);
  expect_f("0.5 Hz @500 ms/div",0.5,10,7.0,0.03);
  expect_f("11 Hz @50 ms/div",11.0,7,1.5,0.03);
  expect_f("30 Hz @50 ms/div",30.0,7,1.5,0.03);
  NOISE=0.02; expect_f("1 Hz + 20 mV noise",1.0,9,3.0,0.03); NOISE=0;
  F_HZ=1.0; expect_zero("1 Hz @100 ms/div (1 cycle)",8,1.6);
  AMP=0.01; expect_zero("20 mVpp < 50 mVpp limit",9,3.0); AMP=0.3;
  DC=0.2; osc_tdiv_i=9; run(3.0); if(!(dbg_adc_min<=8)){printf("FAIL clip min\n");failures++;} else printf("PASS clipping at 0 V visible (min=%u)\n",dbg_adc_min); DC=1.65;
  AMP=0.3; DC=1.65; NOISE=0.0;
  expect_zero_flag("300.00114 kHz aliases to 1.14 Hz",300001.14,10,8.0,1);
  expect_zero_flag("300 kHz exact (aliases to DC)",300000.0,9,3.0,1);
  expect_zero_flag("1 kHz exact at 200 ms/div",1000.0,9,3.0,1);
  expect_zero_flag("150 Hz at 200 ms/div (too fast)",150.0,9,3.0,0);
  AMP=0.05; expect_zero_flag("300 kHz at 100 mVpp",300001.14,10,8.0,1); AMP=0.3;
  F_HZ=1.0; expect_f("1 Hz again after alias cases",1.0,10,6.0,0.03); if(slow_fast_flag){printf("FAIL flag stuck\n");failures++;}
  AMP=1.6; F_HZ=60.0; expect_f("60 Hz 3.2 Vpp not flagged",60.0,7,1.5,0.04); if(slow_fast_flag){printf("FAIL 60 Hz flagged\n");failures++;} AMP=0.3;
  NOISE=0.015; F_HZ=1.0; expect_f("1 Hz + 15 mV noise not flagged",1.0,10,6.0,0.05); if(slow_fast_flag){printf("FAIL noise flagged\n");failures++;} NOISE=0;
  osc_tdiv_i=3; run(0.1); if(slow_active||slow_cfg_tdiv!=-1){printf("FAIL back to FAST\n");failures++;} else printf("PASS back to FAST restores ADC mode\n");
  osc_tdiv_i=9; run(0.1); if(!slow_active){printf("FAIL re-enter SLOW\n");failures++;} else printf("PASS re-enter SLOW\n");
  printf("slow failures=%d\n", failures); return failures; }
'''
(OUT/'slow_test.c').write_text(harness)
subprocess.run(['cc','-O1','-Wall','-o',str(OUT/'slow_test'),str(OUT/'slow_test.c'),'-lm'],check=True,cwd=OUT)
r = subprocess.run([str(OUT/'slow_test')],capture_output=True,text=True); print(r.stdout, end='')
raise SystemExit(r.returncode)
