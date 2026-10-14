"""Host regression of extracted production C; ADC sine inputs are modeled, not hardware."""
from pathlib import Path
import re, subprocess, os
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__import__('tempfile').gettempdir()) / 'osc_regression'
OUT.mkdir(parents=True, exist_ok=True)
s = re.sub(r'/\*.*?\*/', '', (ROOT/'Core/Src/main.c').read_text(), flags=re.S)
def function(name):
    m = re.search(r'^(?:static\s+)?(?:inline\s+)?(?:void|int|float|uint8_t|uint32_t)\s+'+name+r'\([^;]*?\)\s*\{', s, re.M)
    assert m, name
    p = s.index('{', m.start()); depth=1; i=p+1
    while depth:
        depth += (s[i]=='{')-(s[i]=='}'); i+=1
    return s[m.start():i]
def macro(name):
    return re.search(r'^#define '+name+r'\s+([^/\n]+)',s,re.M)[1].strip()
pre = '''#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define ADC_INPUT_ENABLE 1
#define N_SAMPLES 240
#define ADC_PRE_POINTS 200
#define ADC_VREF 3.3f
#define ADC_FULL 4095
#define ADC_MID 2048
#define OSC_H_DIVS 10
#define CENTER_Y 120
#define GRID_Y_STEP 40
#define WAVEFORM_TOP 29
#define SCREEN_H 240
static float volts_per_div=1;
static int16_t new_y[240];
static int osc_plot_offset;
static const float osc_tdivs_ms[]={.005f,.01f,.015f,.02f,.05f,.1f,.2f,.5f,1,2,5,6,10,20,50,100,200,500};
static int osc_tdiv_i,osc_fwin=5,osc_calib=1,osc_trig_fall=0,osc_stable=1,osc_lock_auto=1;
static float osc_fs_eff,osc_offset_ct,osc_offset_v,osc_v[240];
static char osc_det[8];
static int16_t raw_samples[240];
typedef struct {float vmax,vmin,vpp,vrms,dc_offset,freq;} OscMeasure;
static OscMeasure osc_meas;
static uint8_t osc_frame_valid;
static float osc_raw_freq,osc_raw_vpp,osc_capture_tdiv;static float osc_raw_mean_ct,osc_raw_rms_ct,osc_raw_min_ct,osc_raw_max_ct;static uint8_t osc_raw_valid;
static uint32_t adc_measure_n=3200;
static float dbg_adc_rate=800000,dbg_adc_step;
static uint32_t dbg_adc_start,dbg_adc_decim,dbg_adc_n,dbg_adc_us,dbg_adc_frames;
static uint16_t dbg_adc_min,dbg_adc_max;
static int dbg_adc_ok=1,osc_dirty,new_frame_ready;
static float input_f=1000,phase=.31f,input_amp=620,input_dc=2048;static float input_noise=0;
static int wave;
static uint32_t request_n,max_n;
static int fail_capture;
#define OSC_NT 18
#define OSC_NT_FAST 12
#define OSC_TDIV_DEF 8
#define ADC_RATE_LEVELS 5
#define ADC_MEAS_LEVELS 3
static uint8_t adc_meas_lvl,dbg_adc_lvl,adc_too_fast,adc_tri_on,adc_fast_content,adc_over_range;static float adc_hold_f;static uint32_t adc_hold_t,adc_over_t,adc_probe_t;static float adc_rough_ratio;static uint32_t tick_ms;static uint32_t HAL_GetTick(void){tick_ms+=40;return tick_ms;}static const uint8_t adc_meas_seq[3]={1,0,4};
static float AdcRateHz(int l);static uint8_t AdcSetRateLevel(int l);
#define OSC_V_FILL_DIVS 5.2f
#define OSC_FIT_CYCLES 2.5f
static uint8_t osc_auto_stage,osc_auto_tries,hold_active,osc_auto_tb=1;
static volatile uint8_t slow_active;
static float dbg_auto_fsel;
static int osc_vdiv_i;
static const float osc_vdivs[]={.1f,.2f,.5f,1,2};
static void UartPrint(const char*s){(void)s;}
static uint8_t AdcCaptureRaw(uint32_t n,uint32_t *us);
'''
pre += '#define ADC_RAW_MAX '+macro('ADC_RAW_MAX')+'\nstatic uint16_t adc_raw_buf[ADC_RAW_MAX];\n'
pre += '''static float AdcRateHz(int l){static const float t[5]={1600000.f,800000.f,12000000.f/27,300000.f,4800000.f};return t[l];}
static uint8_t AdcSetRateLevel(int l){dbg_adc_rate=AdcRateHz(l);adc_tri_on=(l==4);return 1;}
static uint8_t AdcCaptureRaw(uint32_t n,uint32_t *us){
 if(fail_capture)return 0;
 request_n=n;if(n>max_n)max_n=n;
 for(uint32_t i=0;i<n;i++) adc_raw_buf[i]=(uint16_t)fminf(4095.0f,fmaxf(0.0f,roundf(input_dc+input_noise*((rand()%2001)/1000.0f-1.0f)+input_amp*(wave ? (sinf(6.28318530718f*input_f*i/dbg_adc_rate+phase)>=0?1:-1) : sinf(6.28318530718f*input_f*i/dbg_adc_rate+phase)))));
 *us=(uint32_t)(n/0.8f);return 1;
}
'''
body='\n'.join(function(x) for x in ['OscProcessFrame','OscMeasureFrame','AdcRawFrequency','AdcCaptureFrame','MapToScreen','OscAutoScale','OscAutoScaleStep','AutoScale'])
# State-machine contract: both entry points must use the same implementation,
# never consume held/failed captures, and never derive frequency from pixels.
auto = function('OscAutoScaleStep')
contracts = [
 ('button shares autoscale', 'OscAutoScale();' in function('AutoScale')),
 ('autoscale rejects invalid/held frames', 'osc_frame_valid' in auto and 'hold_active' in auto),
 ('autoscale uses raw measurement', 'OscFreqFromWindow()' not in auto),
 ('main gates failed capture', 'if (!osc_frame_valid) goto skip_draw;' in s),
 ('invalid panel explicit', 'ADC ERR' in s),
]
contract_failures = sum(not ok for _,ok in contracts)
for name,ok in contracts: print(('PASS ' if ok else 'FAIL ')+name)
main='''int main(void){int failures=0;
 osc_fs_eff=12345;osc_tdiv_i=6;OscProcessFrame();
 printf("captured rate retained: %.1f\\n",osc_fs_eff);if(osc_fs_eff!=12345)failures++;
 osc_stable=0;input_f=1000;AdcCaptureFrame();
 printf("roll start %u\\n",dbg_adc_start);if(dbg_adc_start!=0)failures++;
 for(int i=0;i<240;i++)raw_samples[i]=3000;osc_raw_valid=0;OscProcessFrame();MapToScreen(0);
 printf("calibrated DC pixel %d expected 120\\n",new_y[0]);if(new_y[0]!=120)failures++;
 float freq[]={100,20000};
 for(int k=0;k<2;k++) {osc_tdiv_i=k?8:5; input_f=freq[k];for(int settle=0;settle<6;settle++) AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();
 printf("frequency %.0f -> %.2f\\n",input_f,osc_meas.freq);
 if(fabsf(osc_meas.freq/input_f-1)>.02f) failures++;
 }
 osc_tdiv_i=11;AdcCaptureFrame();printf("DMA request %u limit 65535\\n",request_n);if(request_n>65535) failures++;
 int cases=0;
 float sweep[]={40,50,100,500,999,1000,10001,20000,48000,100000};
 for(int ti=0;ti<12;ti++)for(int fi=0;fi<10;fi++)for(int filter=1;filter<=21;filter+=10){
  osc_tdiv_i=ti;osc_fwin=filter;input_f=sweep[fi];phase=.17f+fi*.29f;adc_measure_n=3200;
  for(int settle=0;settle<16;settle++)AdcCaptureFrame();
  OscProcessFrame();OscMeasureFrame();cases++;
  if(!osc_frame_valid||fabsf(osc_meas.freq/input_f-1)>.02f){printf("FAIL sweep %d %.0f filter%d ->%.3f\\n",ti,input_f,filter,osc_meas.freq);failures++;}
  float expected=240/(osc_tdivs_ms[ti]*.01f);
  if(fabsf(osc_fs_eff/expected-1)>.0001f)failures++;
 }
 float hf[]={150000,200000,300000,500000,800000,1000000,1150000,1500000,1800000,2000000,2100000};osc_tdiv_i=0;
 for(int fi=0;fi<11;fi++){input_f=hf[fi];adc_measure_n=3200;adc_meas_lvl=0;osc_fwin=1;for(int s=0;s<12;s++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();cases++;
  printf("HF %.0f -> %.1f\\n",input_f,osc_meas.freq);if(!osc_frame_valid||fabsf(osc_meas.freq/input_f-1)>.02f){printf("FAIL HF %.0f\\n",input_f);failures++;}}
 input_f=900000;adc_measure_n=3200;adc_meas_lvl=0;osc_tdiv_i=0;for(int s=0;s<12;s++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();
 printf("900 kHz alias check -> %.1f\\n",osc_meas.freq);if(fabsf(osc_meas.freq/900000-1)>.02f){printf("FAIL alias 900k\\n");failures++;}
 input_f=2300000;adc_measure_n=3200;adc_meas_lvl=0;for(int s=0;s<12;s++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();
 printf("HF above limit 2300000 -> %.1f over_range=%d (must be 0, 1)\\n",osc_meas.freq,adc_over_range);if(!adc_over_range)failures++;
 {int bad=0; float nq[]={2400000,2395000,2405000,2450000,2600000,3000000};
  for(int k=0;k<6;k++){input_f=nq[k];for(int ph=0;ph<20;ph++){phase=.1f+ph*.31f;adc_measure_n=3200;adc_meas_lvl=2;adc_hold_f=0;for(int s=0;s<3;s++)AdcCaptureFrame();
   if(osc_raw_freq>0&&fabsf(osc_raw_freq/input_f-1)>.02f&&fabsf(osc_raw_freq/(4800000-input_f)-1)>.02f){bad++;}}}
  printf("Nyquist-region false readings: %d (must be 0)\\n",bad);if(bad)failures++;}
 {   /* 2.4 MHz = 3 x 800 kSa/s aliases to DC; a 33 kHz clock-offset beat must be rejected from the 800 kSa/s level */
  input_f=2400000*1.0137;adc_measure_n=3200;adc_meas_lvl=0;adc_hold_f=0;int shown=0;
  for(int s=0;s<8;s++){AdcCaptureFrame();if(osc_raw_freq>0&&fabsf(osc_raw_freq/input_f-1)>.02f)shown++;}
  printf("alias-to-low (2.43 MHz seen at 800k): wrong numbers %d (must be 0)\\n",shown);if(shown)failures++;}
 /* hold: a good 1 kHz reading survives <1.5 s of failed frames, then goes away; no hold without signal */
 input_f=1000;adc_measure_n=3200;adc_meas_lvl=0;for(int s=0;s<8;s++)AdcCaptureFrame();float good=osc_raw_freq;fail_capture=0;
 if(good<900||good>1100){printf("FAIL hold setup %.1f\\n",good);failures++;}
 input_amp=0;AdcCaptureFrame();if(osc_raw_freq!=0){printf("FAIL hold with no signal\\n");failures++;}input_amp=620;if(osc_meas.freq!=0)failures++;
 {   /* parameters: Mx/Mn/Rs/Vp must match the true signal whatever the time base, filter and number of cycles in the window */
  float Atrue=620.0f*3.3f/4095.0f; int nbad=0;
  float cfg[][2]={{250,8},{1000,8},{20000,6},{100000,5},{150000,3}};
  for(int k=0;k<5;k++){ input_f=cfg[k][0]; osc_tdiv_i=(int)cfg[k][1]; osc_fwin=5; input_amp=620; input_dc=2048; wave=0; phase=0.4f+k*0.7f; adc_measure_n=3200; adc_meas_lvl=0;
   for(int s=0;s<14;s++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();
   int ok = fabsf(osc_meas.vmax-Atrue)<0.04f*Atrue && fabsf(osc_meas.vmin+Atrue)<0.04f*Atrue && fabsf(osc_meas.vrms-Atrue*0.7071f)<0.04f*Atrue && fabsf(osc_meas.vpp-2*Atrue)<0.06f*Atrue;
   printf("%s params %.0f Hz tdiv_i=%d fwin=5: Mx=%.3f Mn=%.3f Rs=%.3f Vpp=%.3f (true +-%.3f, rms %.3f)\\n",ok?"PASS":"FAIL",input_f,osc_tdiv_i,osc_meas.vmax,osc_meas.vmin,osc_meas.vrms,osc_meas.vpp,Atrue,Atrue*0.7071f);
   if(!ok){nbad++;failures++;} }
  /* DC offset of the signal must not leak into Mx/Mn (AC coupling) */
  input_f=1000; osc_tdiv_i=8; input_dc=2600; adc_measure_n=3200; adc_meas_lvl=0; for(int s=0;s<14;s++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();
  { int ok = fabsf(osc_meas.vmax-Atrue)<0.04f*Atrue && fabsf(osc_meas.vmin+Atrue)<0.04f*Atrue; printf("%s params with DC offset 2600: Mx=%.3f Mn=%.3f\\n",ok?"PASS":"FAIL",osc_meas.vmax,osc_meas.vmin); if(!ok)failures++; }
  (void)nbad; input_dc=2048; osc_fwin=1; }
 {   /* a NOISY RAIL (flat level + +-85 counts of noise = 0.21 Vpp) must never become a frequency or a "too fast" flag */
  int nbad=0;
  for(int lv=0; lv<3; lv++){ input_amp=0; input_dc=3800; input_noise=85; wave=0; adc_measure_n=3200; adc_meas_lvl=lv; adc_too_fast=0; adc_fast_content=0; adc_over_t=0; adc_hold_f=0;
   for(int s=0;s<6;s++)AdcCaptureFrame();
   int ok = osc_raw_freq==0.0f && adc_fast_content==0 && adc_too_fast==0 && adc_over_t==0;
   printf("%s noisy rail, meas level %d: raw_freq=%.1f fast_content=%d too_fast=%d over=%u (all must be 0)\\n", ok?"PASS":"FAIL", lv, osc_raw_freq, adc_fast_content, adc_too_fast, adc_over_t);
   if(!ok){nbad++;failures++;} }
  /* 151 Hz sine + the noisy generator rail: the alias probe must not fire, the reading must not blink to 2 MHz */
  { int bad2=0, miss=0, n151=0;
    input_amp=620; input_dc=2048; input_noise=85; input_f=151; wave=0; adc_meas_lvl=0; adc_measure_n=3200; adc_over_t=0; adc_hold_f=0; adc_probe_t=0;
    for(int s=0;s<120;s++){ AdcCaptureFrame(); if(s<12) continue; n151++;
      if(osc_raw_freq==0.0f) miss++; else if(fabsf(osc_raw_freq/151.0f-1)>0.03f) bad2++;
      if(adc_over_t) bad2++; }
    int ok = bad2==0 && miss<=3;
    printf("%s 151 Hz + noise: wrong readings=%d, missing readings=%d of %d, over-range flag set=%d\\n", ok?"PASS":"FAIL", bad2, miss, n151, adc_over_t?1:0);
    if(!ok){failures++;} }
  input_noise=0; input_amp=620; input_dc=2048; (void)nbad; }

 {   /* TRIGGER STABILITY: 200 Hz at 1 ms/div (2 cycles on screen, search window ~ 1 period) must start on the same edge every frame,
      * otherwise the picture jumps by half a cycle from frame to frame ("not smooth") */
  int unstable=0, frames=0; input_f=200; input_amp=620; input_dc=2048; input_noise=30; wave=0; osc_tdiv_i=8; osc_fwin=5; osc_stable=1;
  for(int ph=0; ph<150; ph++){ phase=0.05f+ph*0.37f; adc_measure_n=3200; adc_meas_lvl=0; AdcCaptureFrame(); if(!osc_frame_valid) continue; frames++;
    int mn=4095,mx=0; for(int i=0;i<240;i++){ if(raw_samples[i]<mn)mn=raw_samples[i]; if(raw_samples[i]>mx)mx=raw_samples[i]; }
    float mid=(mn+mx)/2.0f; int first=-1; for(int i=1;i<240;i++) if(raw_samples[i-1]<mid && raw_samples[i]>=mid){ first=i; break; }
    if(first<0 || first>12) unstable++; }
  int ok=(unstable==0 && frames>100);
  printf("%s trigger stability 200 Hz: %d of %d frames did not start on the rising edge\\n", ok?"PASS":"FAIL", unstable, frames); if(!ok) failures++;
  input_noise=0; input_f=1000; }

 {   /* EDGE JITTER: the rising edge must land on the same screen position frame after frame ("shudder" < 1 point peak-to-peak) */
  float lo=1e9,hi=-1e9; int fr=0; input_f=200; input_amp=620; input_dc=2048; input_noise=85; wave=0; osc_tdiv_i=8; osc_fwin=5; osc_stable=1;
  for(int ph=0; ph<150; ph++){ phase=0.05f+ph*0.37f; adc_measure_n=3200; adc_meas_lvl=0; AdcCaptureFrame(); if(!osc_frame_valid) continue; fr++;
    float mid=2048.0f; float pos=-1; for(int i=1;i<240;i++) if(raw_samples[i-1]<mid && raw_samples[i]>=mid){ pos=i+(mid-raw_samples[i-1])/(float)(raw_samples[i]-raw_samples[i-1]); break; }
    if(pos<0) continue; if(pos<lo)lo=pos; if(pos>hi)hi=pos; }
  int ok=(fr>100 && (hi-lo)<1.0f);
  printf("%s edge jitter 200 Hz + 85 counts noise: %.2f display points peak-to-peak over %d frames (limit 1.0)\\n", ok?"PASS":"FAIL", hi-lo, fr); if(!ok) failures++;
  input_noise=0; input_f=1000; }

 {   /* GENERATOR OFF / 0 Hz: the output sits at ~0.1 V with 0.2 Vpp noise, clipped at 0 -> must not read as a frequency (was 2.00 MHz) */
  int bad=0;
  for(int lv=0; lv<3; lv++) for(int dcv=40; dcv<=240; dcv+=100){ input_amp=0; input_dc=dcv; input_noise=100; wave=0; adc_measure_n=3200; adc_meas_lvl=lv; adc_too_fast=0; adc_fast_content=0; adc_over_t=0; adc_hold_f=0; adc_probe_t=0;
    for(int s=0;s<8;s++){ AdcCaptureFrame(); if(osc_raw_freq!=0.0f) bad++; } }
  int ok=(bad==0); printf("%s generator at 0 Hz (clipped noise at 0 V): false frequency readings=%d (must be 0)\\n", ok?"PASS":"FAIL", bad); if(!ok) failures++;
  input_noise=0; input_amp=620; input_dc=2048; }
 printf("sweep %d cases, max DMA %u\\n",cases,max_n);if(max_n>65535)failures++;
 float saved_fs=osc_fs_eff,saved_td=osc_capture_tdiv;
 hold_active=1;osc_auto_stage=0;AutoScale();if(osc_auto_stage||osc_fs_eff!=saved_fs)failures++;
 hold_active=0;AutoScale();fail_capture=1;AdcCaptureFrame();OscAutoScaleStep();
 if(osc_frame_valid||new_frame_ready||osc_meas.freq||dbg_adc_ok||osc_auto_stage!=1)failures++;
 if(osc_fs_eff!=saved_fs||osc_capture_tdiv!=saved_td)failures++;
 fail_capture=0;input_f=20000;AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();OscAutoScaleStep();
 if(!osc_frame_valid||!dbg_adc_ok||osc_auto_stage||(osc_tdiv_i!=1&&osc_tdiv_i!=2)||volts_per_div!=osc_vdivs[osc_vdiv_i])failures++;
 input_amp=0;for(int i=0;i<7;i++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();if(osc_meas.freq!=0)failures++;
 input_amp=10;for(int i=0;i<7;i++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();if(osc_meas.freq!=0)failures++;
 input_amp=620;input_dc=2600;wave=1;
 for(int i=0;i<7;i++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();if(fabsf(osc_meas.freq/20000-1)>.02f)failures++;
 for(int i=0;i<240;i++)raw_samples[i]=(i%2)?3000:1000;osc_raw_valid=0;
 osc_calib=1;osc_fwin=21;OscProcessFrame();MapToScreen(7);
 for(int i=0;i<240;i++){int y=120-(int)(osc_v[(i+7)%240]/volts_per_div*40);if(y<29)y=29;if(y>239)y=239;if(new_y[i]!=y)failures++;}
 if(osc_plot_offset!=7)failures++;
 printf("DC/no-signal/square/filter/cursor checks complete\\n");
 printf("behavior failures=%d\\n",failures);
 return failures?1:0;}
'''
(OUT/'regression.c').write_text(pre+body+main)
subprocess.run(['clang','-O0',str(OUT/'regression.c'),'-o',str(OUT/'regression'),'-lm'],check=True)
r=subprocess.run([str(OUT/'regression')],text=True,capture_output=True)
print(r.stdout,end='');raise SystemExit(r.returncode or bool(contract_failures))
