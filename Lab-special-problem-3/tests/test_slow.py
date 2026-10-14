"""Host test of the SLOW history sampler + SLOW display (T/DIV >= 50 ms) extracted from Core/Src/main.c.
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
#include <stddef.h>
#define OSC_SLOW_HOST_TEST 1
#define N_SAMPLES 240
#define OSC_H_DIVS 10
#define ADC_VREF 3.3f
#define ADC_FULL 4095
typedef struct {float vmax,vmin,vpp,vrms,dc_offset,freq;} OscMeasure;
static OscMeasure osc_meas; static char osc_det[8];
static const float osc_tdivs_ms[] = {0.005f,0.01f,0.015f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f,5.0f,6.0f,10.0f,20.0f,50.0f,100.0f,200.0f,500.0f};
#define T10 12
#define T20 13
#define T50 14
#define T100 15
#define T200 16
#define T500 17
#define OSC_NT 18
static uint32_t HAL_GetTick(void);
static int osc_tdiv_i = 8; static int osc_trig_fall = 0; static uint8_t osc_stable = 1; static uint8_t osc_lock_auto = 1;
static volatile int16_t raw_samples[N_SAMPLES];
static float osc_fs_eff, osc_raw_freq, osc_raw_vpp, osc_capture_tdiv;
static float osc_raw_mean_ct, osc_raw_rms_ct, osc_raw_min_ct, osc_raw_max_ct; static uint8_t osc_raw_valid;
static volatile uint16_t dbg_adc_min, dbg_adc_max; static volatile uint32_t dbg_adc_frames; static volatile uint8_t dbg_adc_ok;
static volatile uint8_t new_frame_ready, osc_dirty, osc_frame_valid, slow_active, slow_fast_flag, slow_busy;
static double now_s, F_HZ=1.0, AMP=0.3, DC=1.65, NOISE=0.0, F2_HZ=0.0, AMP2=0.0;
static unsigned rs=12345; static double rnd(void){rs=rs*1103515245u+12345u; return ((rs>>8)&0xFFFF)/65535.0-0.5;}
static int burst_k; static double burst_t;   /* conversions 1.25 us apart + the firmware's dither delay ((k*3)%7 loops of ~62 ns) */
static inline uint16_t SlowAdcRead(void){ burst_t += 1.25e-6 + 62e-9*((burst_k*3)%7); burst_k++; double tt=now_s+burst_t;
  double v=DC+AMP*sin(2*M_PI*F_HZ*tt)+AMP2*sin(2*M_PI*F2_HZ*tt)+NOISE*rnd(); if(v<0)v=0; if(v>3.3)v=3.3; return (uint16_t)(v/3.3*4095.0+0.5);}
static uint32_t HAL_GetTick(void){ return (uint32_t)(now_s*1000.0+0.5); }
#include "slow_extract.c"
static int failures;
static int busy_period=0, busy_len=0;       /* simulate burst captures: slow_busy=1 for busy_len ms every busy_period ms */
static void run(double secs){ for(double t=0;t<secs*1000;t+=1){ now_s+=0.001; burst_k=0; burst_t=0;
    slow_busy = (busy_period && ((int)t % busy_period) < busy_len) ? 1 : 0;
    OscSlowTick();
    if(((int)t)%33==0){ SlowSync(); if(slow_active) SlowCaptureFrame(); } } slow_busy=0; }
static void expect_f(const char*n,double f_true,int ti,double secs,double tol){
  osc_tdiv_i=ti; F_HZ=f_true; run(secs);
  int ok = osc_frame_valid && fabs(osc_raw_freq-f_true) <= tol*f_true;
  printf("%s %-34s tdiv=%3.0fms Fs=%6.1f Vpp=%.3f measured=%.3f Hz (true %.2f)\n", ok?"PASS":"FAIL", n, osc_tdivs_ms[ti], osc_fs_eff, osc_raw_vpp, osc_raw_freq, f_true);
  if(!ok) failures++;
}
static void expect_zero(const char*n,int ti,double secs){
  osc_tdiv_i=ti; run(secs); int ok = osc_frame_valid && osc_raw_freq==0.0f;
  printf("%s %-34s measured=%.3f Hz (must be 0, never invented)\n", ok?"PASS":"FAIL", n, osc_raw_freq); if(!ok) failures++;
}
static void expect_zero_flag(const char*n,double f_true,int ti,double secs,int want_flag){
  osc_tdiv_i=ti; F_HZ=f_true; run(secs); int ok = osc_frame_valid && osc_raw_freq==0.0f && slow_fast_flag==want_flag;
  printf("%s %-34s measured=%.3f Hz flag=%d spread=%u (must read 0 Hz; flag expected=%d)\n", ok?"PASS":"FAIL", n, osc_raw_freq, slow_fast_flag, (unsigned)dbg_slow_spread, want_flag); if(!ok) failures++;
}
static double max_step_ratio(void){ double m=0; int lo=4095,hi=0; for(int i=0;i<N_SAMPLES;i++){ if(raw_samples[i]<lo)lo=raw_samples[i]; if(raw_samples[i]>hi)hi=raw_samples[i]; }
  for(int i=1;i<N_SAMPLES;i++){ double d=fabs((double)raw_samples[i]-raw_samples[i-1]); if(d>m)m=d; } return hi>lo ? m/(hi-lo) : 0; }
int main(void){
  osc_tdiv_i=T200; run(0.01);
  expect_f("1 Hz @200 ms/div (measured on 5 s)",1.0,T200,6.0,0.03);   /* the picture shows 2 cycles, the number comes from the 5 s window */
  expect_f("1 Hz @500 ms/div",1.0,T500,6.0,0.03);
  expect_f("0.6 Hz @500 ms/div (3 cycles)",0.6,T500,7.0,0.03);
  expect_f("11 Hz @50 ms/div",11.0,T50,1.5,0.03);
  expect_f("30 Hz @50 ms/div",30.0,T50,1.5,0.03);
  expect_f("30 Hz @20 ms/div (6 cycles)",30.0,T20,1.5,0.03);
  expect_f("45 Hz @10 ms/div (4.5 cycles)",45.0,T10,1.5,0.03);
  expect_f("20 Hz @20 ms/div (4 cycles)",20.0,T20,1.5,0.03);
  expect_f("20 Hz @10 ms/div (measured on 20/50 ms)",20.0,T10,1.5,0.03);
  NOISE=0.02; expect_f("1 Hz + 20 mV noise",1.0,T500,6.0,0.03); NOISE=0;
  expect_f("1 Hz @100 ms/div (1 cycle shown)",1.0,T100,6.0,0.03);
  AMP=0.3; DC=1.65; NOISE=0.0;
  expect_zero_flag("300.00114 kHz aliases to 1.14 Hz",300001.14,T500,8.0,1);
  expect_zero_flag("300 kHz exact (aliases to DC)",300000.0,T200,3.0,1);
  expect_zero_flag("150 Hz at 200 ms/div (too fast)",150.0,T200,3.0,1);
  F_HZ=1.0; expect_f("1 Hz again after alias cases",1.0,T500,6.0,0.03); if(slow_fast_flag){printf("FAIL flag stuck\n");failures++;}
  AMP=1.6; F_HZ=55.0; expect_f("55 Hz 3.2 Vpp not flagged",55.0,T50,1.5,0.04); if(slow_fast_flag){printf("FAIL 55 Hz flagged\n");failures++;} AMP=0.3;
  NOISE=0.015; F_HZ=1.0; expect_f("1 Hz + 15 mV noise not flagged",1.0,T500,6.0,0.05); if(slow_fast_flag){printf("FAIL noise flagged\n");failures++;} NOISE=0;

  /* --- statistics over whole cycles: 1 Hz window with 5.5 cycles must still give exact DC / RMS / peaks --- */
  F_HZ=1.1; AMP=0.3; DC=1.65; osc_tdiv_i=T500; run(7.0);
  { float k=3.3f/4095.0f; float dc=osc_raw_mean_ct*k, rms=osc_raw_rms_ct*k, mx=(osc_raw_max_ct-osc_raw_mean_ct)*k, mn=(osc_raw_min_ct-osc_raw_mean_ct)*k;
    int ok = osc_raw_valid && fabsf(dc-1.65f)<0.01f && fabsf(rms-0.3f*0.7071f)<0.006f && fabsf(mx-0.3f)<0.01f && fabsf(mn+0.3f)<0.01f;
    printf("%s SLOW stats 1.1 Hz (5.5 cycles): DC=%.3f rms=%.3f Mx=%.3f Mn=%.3f (true 1.650 0.212 +-0.300)\n", ok?"PASS":"FAIL", dc, rms, mx, mn); if(!ok)failures++; }
  F_HZ=1.0;
  /* --- noisy generator rail: 170 mV of noise inside every burst must NOT make a slow signal look fast --- */
  F_HZ=1.0; AMP=0.3; NOISE=0.17; osc_tdiv_i=T500; run(7.0);
  { int ok = osc_frame_valid && fabs(osc_raw_freq-1.0)<0.03 && !slow_fast_flag; printf("%s 1 Hz with 170 mV burst noise: %.3f Hz fast_flag=%d spread=%u rough=%u\n", ok?"PASS":"FAIL", osc_raw_freq, slow_fast_flag, (unsigned)dbg_slow_spread, (unsigned)dbg_slow_rough); if(!ok)failures++; }
  { float pr=SlowProbeFreq(T500); int ok = fabs(pr-1.0)<0.03; printf("%s probe with the same noise: %.3f\n", ok?"PASS":"FAIL", pr); if(!ok)failures++; }
  NOISE=0.0;
  /* --- history is always running: FAST display (sampler still on) then SLOW at once, no refill --- */
  F_HZ=1.0; AMP=0.3; osc_tdiv_i=8; run(8.0);                       /* FAST (1 ms/div): nothing captured by SLOW */
  osc_tdiv_i=T500; SlowSync(); SlowCaptureFrame();                 /* switch NOW */
  { int ok = osc_frame_valid && fabs(osc_raw_freq-1.0)<0.03; printf("%s instant FAST->SLOW 500 ms/div: %.3f Hz (no wait)\n", ok?"PASS":"FAIL", osc_raw_freq); if(!ok)failures++; }
  osc_tdiv_i=T200; SlowSync(); SlowCaptureFrame();
  { int ok = osc_frame_valid && fabs(osc_raw_freq-1.0)<0.03; printf("%s instant 500->200 ms/div: window has 2 cycles, number from the 5 s window -> %.3f Hz\n", ok?"PASS":"FAIL", osc_raw_freq); if(!ok)failures++; }
  /* trace continuity: switching bases must not leave steps/holes (sine of 1 Hz, 20 ms points: step <= 20 % of Vpp) */
  { double r=max_step_ratio(); int ok = r<0.20; printf("%s trace smooth after switch: biggest step %.1f %% of Vpp\n", ok?"PASS":"FAIL", r*100); if(!ok)failures++; }

  /* --- burst captures steal ticks (80 ms every 200 ms): history is interpolated, result unchanged --- */
  busy_period=200; busy_len=80; F_HZ=1.0; osc_tdiv_i=T500; run(7.0);
  { int ok = osc_frame_valid && fabs(osc_raw_freq-1.0)<0.03; double r=max_step_ratio(); ok = ok && r<0.20;
    printf("%s 1 Hz with 40 %% of ticks lost to captures: %.3f Hz, biggest step %.1f %%\n", ok?"PASS":"FAIL", osc_raw_freq, r*100); if(!ok)failures++; }
  busy_period=0;

  /* --- probe: what each base would show, display untouched --- */
  F_HZ=1.0; AMP=0.3; osc_tdiv_i=8; run(8.0);
  { float p5=SlowProbeFreq(T500), p2=SlowProbeFreq(T200), p1=SlowProbeFreq(T100);
    int ok = fabs(p5-1.0)<0.03 && p2==0.0f && p1==0.0f;
    printf("%s probe 1 Hz: 500ms=%.3f 200ms(2 cycles)=%.3f 100ms(1 cycle)=%.3f\n", ok?"PASS":"FAIL", p5,p2,p1); if(!ok)failures++; }
  F_HZ=30.0; run(3.0);
  { float p0=SlowProbeFreq(T50), p2=SlowProbeFreq(T200);
    int ok = fabs(p0-30.0)<1.0 && p2==0.0f;
    printf("%s probe 30 Hz: 50ms=%.3f 200ms(too fast)=%.3f\n", ok?"PASS":"FAIL", p0,p2); if(!ok)failures++; }
  F_HZ=300000.0; run(3.0);
  { float p5=SlowProbeFreq(T500), p0=SlowProbeFreq(T50), p2=SlowProbeFreq(T200);
    int ok = p5==0.0f && p0==0.0f && p2==0.0f; printf("%s probe 300 kHz (aliased): %.3f %.3f %.3f (all must be 0)\n", ok?"PASS":"FAIL", p5,p0,p2); if(!ok)failures++; }

  /* --- signal CHANGE 300 kHz -> 1 Hz: no intermediate wrong numbers, first correct value within 4 s --- */
  AMP=0.3; F_HZ=300000.0; run(8.0); F_HZ=1.0;
  { int wrong=0; double first=-1; for(int k=0;k<70;k++){ run(0.1); float pr=SlowProbeFreq(T500); float f2=SlowProbeFreq(T100);
      if(pr>0){ if(fabs(pr-1.0)>0.03) wrong++; if(first<0) first=(k+1)*0.1; } if(f2>0 && fabs(f2-1.0)>0.03) wrong++; }
    int ok = wrong==0 && first>0 && first<=4.0; printf("%s 300 kHz -> 1 Hz: wrong intermediate readings=%d, first correct reading after %.1f s\n", ok?"PASS":"FAIL", wrong, first); if(!ok)failures++; }
  /* --- frequency STEPS 1 -> 2 -> 3 Hz: only values near the true frequency (never 2.14 / 2.67 in between) --- */
  { AMP=0.3; F_HZ=1.0; run(8.0); int wrong=0, seen[4]={0,0,0,0}; float steps[3]={2.0f,3.0f,1.0f};
    for(int st=0; st<3; st++){ F_HZ=steps[st]; for(int k=0;k<80;k++){ run(0.1); float pr=SlowProbeFreq(T500);
        if(pr>0){ int hit=0; for(int h=1;h<=3;h++) if(fabs(pr-h)<0.04*h){ hit=1; seen[h]++; } if(!hit) wrong++; } } }
    int ok = wrong==0 && seen[2]>5 && seen[3]>5 && seen[1]>5; printf("%s 1->2->3->1 Hz steps: in-between readings=%d (readings at 1/2/3 Hz: %d/%d/%d)\n", ok?"PASS":"FAIL", wrong, seen[1], seen[2], seen[3]); if(!ok) failures++; }
  /* --- 3 Vpp slow sines the history cannot read right now (1-2 cycles in the window): NOT a fast signal (was flagged at 13+ Hz) --- */
  AMP=1.5;
  expect_f("13 Hz 3 Vpp @10 ms/div reads 13",13.0,T10,1.5,0.04); if(slow_fast_flag){printf("FAIL 13 Hz flagged\n");failures++;}
  expect_f("15 Hz 3 Vpp @10 ms/div reads 15",15.0,T10,1.5,0.04); if(slow_fast_flag){printf("FAIL 15 Hz flagged\n");failures++;}
  expect_zero_flag("150 Hz 3 Vpp is still FAST",150.0,T50,1.5,1);
  AMP=0.3;
  /* --- TRIGGERED VIEW (10..50 ms/div): the rising edge must sit at the same screen position in every frame (a still picture) --- */
  { struct { double f; int ti; const char *n; } cs[] = {{30.0,T20,"30 Hz @20 ms/div"},{45.0,T10,"45 Hz @10 ms/div"},{12.0,T50,"12 Hz @50 ms/div"},{25.0,T50,"25 Hz @50 ms/div"}};
    for(int q=0;q<4;q++){ AMP=0.9; NOISE=0.04; F_HZ=cs[q].f; osc_tdiv_i=cs[q].ti; run(3.0);
      double lo=1e9,hi=-1e9; int fr=0, ntrig=0;
      for(int k=0;k<60;k++){ run(0.0113+0.0007*(k%5)); SlowSync(); SlowCaptureFrame(); if(!osc_frame_valid) continue; fr++; ntrig+=dbg_slow_trig;
        int mn=4095,mx=0; for(int i=0;i<N_SAMPLES;i++){ if(raw_samples[i]<mn)mn=raw_samples[i]; if(raw_samples[i]>mx)mx=raw_samples[i]; }
        double mid=2048.0; double pos=-1; for(int i=1;i<N_SAMPLES;i++) if(raw_samples[i-1]<mid && raw_samples[i]>=mid){ pos=i-1+(mid-raw_samples[i-1])/(double)(raw_samples[i]-raw_samples[i-1]); break; }
        if(pos<0) continue; if(pos<lo)lo=pos; if(pos>hi)hi=pos; }
      int ok=(fr>50 && ntrig==fr && (hi-lo)<1.5);
      printf("%s triggered view %-18s edge position spread %.2f points over %d frames (limit 1.5), triggered %d/%d, measured %.3f Hz\n", ok?"PASS":"FAIL", cs[q].n, hi-lo, fr, ntrig, fr, osc_raw_freq); if(!ok) failures++; }
    NOISE=0.0; AMP=0.3; }
  /* 11 Hz -> 1 Hz on ONE time base (200 ms/div): the picture keeps scrolling, no empty screen, and the number follows within ~3 s */
  { AMP=0.9; F_HZ=11.0; osc_stable=0; osc_tdiv_i=T200; run(6.0); F_HZ=1.0; int ok=1, got=0; double t_read=-1; int minspan=99999;
    for(int k=0;k<60;k++){ run(0.1); if(!osc_frame_valid) ok=0;
      int lo=4095,hi=0; for(int i=0;i<N_SAMPLES;i++){ if(raw_samples[i]<lo)lo=raw_samples[i]; if(raw_samples[i]>hi)hi=raw_samples[i]; } if(hi-lo<minspan) minspan=hi-lo;
      if(!got && fabs(osc_raw_freq-1.0)<0.03){ got=1; t_read=(k+1)*0.1; } }
    ok = ok && got && t_read<=3.6 && minspan>1200;
    printf("%s 11 Hz -> 1 Hz @200 ms/div: reads 1.00 Hz after %.1f s, smallest on-screen span %d counts (must stay > 1200)\n", ok?"PASS":"FAIL", t_read, minspan); if(!ok) failures++; AMP=0.3; }
  /* smooth zoom: after a change of time base the DRAWN time base moves by <= 16 % per frame and then reaches the selected one */
  { AMP=0.9; F_HZ=5.0; osc_stable=0; osc_tdiv_i=T100; run(3.0); double prev=slow_pv_cur, worst=1.0; osc_tdiv_i=T500; int n=0;
    for(int k=0;k<80;k++){ run(0.033); double r=slow_pv_cur/prev; if(r<1)r=1/r; if(r>worst)worst=r; prev=slow_pv_cur; if(fabs(slow_pv_cur-20833.0)<1.0){ n=k; break; } n=k; }
    int ok=(worst<=1.16 && fabs(slow_pv_cur-20833.0)<1.0); printf("%s zoom 100 -> 500 ms/div: biggest step %.3f x per frame, arrived after %d frames\n", ok?"PASS":"FAIL", worst, n); if(!ok) failures++; osc_stable=1; AMP=0.3; }
  /* lock switched off: free-run even at 20 ms/div */
  { osc_stable=0; AMP=0.9; F_HZ=30.0; osc_tdiv_i=T20; run(3.0); SlowSync(); SlowCaptureFrame(); int ok=(dbg_slow_trig==0); printf("%s lock off: 30 Hz @20 ms/div free-runs\n", ok?"PASS":"FAIL"); if(!ok) failures++; osc_stable=1; AMP=0.3; }
  /* --- roll mode (>= 100 ms/div) and no-signal stay free-running --- */
  { AMP=0.3; F_HZ=1.0; osc_tdiv_i=T500; run(8.0); SlowSync(); SlowCaptureFrame(); int ok=(dbg_slow_trig==0); printf("%s 1 Hz @500 ms/div stays in roll mode (no trigger)\n", ok?"PASS":"FAIL"); if(!ok) failures++; }
  /* --- noise only must not give an invented number --- */
  AMP=0.0; NOISE=0.6; F_HZ=1.0; osc_tdiv_i=T200; run(4.0);
  { int ok = osc_raw_freq==0.0f; printf("%s noise only: %.3f Hz (must be 0)\n", ok?"PASS":"FAIL", osc_raw_freq); if(!ok)failures++; }
  NOISE=0.0; AMP=0.3;

  /* --- sampler resumes cleanly after a very long busy period (> history length) --- */
  F_HZ=1.0; slow_busy=1; for(int i=0;i<9000;i++){ now_s+=0.001; OscSlowTick(); } slow_busy=0; osc_tdiv_i=T500; run(6.0);
  { int ok = osc_frame_valid && fabs(osc_raw_freq-1.0)<0.03; printf("%s after a 9 s busy gap: %.3f Hz\n", ok?"PASS":"FAIL", osc_raw_freq); if(!ok)failures++; }

  printf("slow failures=%d\n", failures);
  return failures?1:0;
}
'''
(OUT/'slow_test.c').write_text(harness)
subprocess.run(['cc', '-O1', '-Wall', '-o', str(OUT/'slow_test'), str(OUT/'slow_test.c'), '-lm'], check=True)
r = subprocess.run([str(OUT/'slow_test')], text=True, capture_output=True)
print(r.stdout, end='')
raise SystemExit(r.returncode)
