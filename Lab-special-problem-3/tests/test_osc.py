"""Host regression of extracted production C; ADC sine inputs are modeled, not hardware."""
from pathlib import Path
import re, subprocess, os
ROOT = Path(__file__).resolve().parents[1]
OUT = (Path.home()/'.hermes/cache/scratch') / 'osc_regression'
OUT.mkdir(parents=True, exist_ok=True)
s = re.sub(r'/\*.*?\*/', '', '\n'.join(p.read_text() for p in sorted((ROOT/'Core/Src').glob('osc_*.c'))) + (ROOT/'Core/Inc/osc_config.h').read_text(), flags=re.S)
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
#define ADC_PRE_POINTS 120
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
static const float osc_tdivs_ms[]={.1f,.2f,.5f,1,2,5,6};
static int osc_tdiv_i,osc_fwin=5,osc_calib=1,osc_trig_fall=0,osc_stable=1;
static float osc_fs_eff,osc_offset_ct,osc_offset_v,osc_v[240];
static char osc_det[8];
static int16_t raw_samples[240];
typedef struct {float vmax,vmin,vpp,vrms,dc_offset,freq;} OscMeasure;
static OscMeasure osc_meas;
static uint8_t osc_frame_valid;
static float osc_raw_freq,osc_raw_vpp,osc_capture_tdiv;
static uint32_t adc_measure_n=3200;
static uint8_t AdcSetRate(int slow);
static float dbg_adc_rate=800000,dbg_adc_step;
static uint32_t dbg_adc_start,dbg_adc_decim,dbg_adc_n,dbg_adc_us,dbg_adc_frames;
static uint16_t dbg_adc_min,dbg_adc_max;
static int dbg_adc_ok=1,osc_dirty,new_frame_ready;
static float input_f=1000,phase=.31f,input_amp=620,input_dc=2048;
static int wave;
static uint32_t request_n,max_n;
static int fail_capture;
#define OSC_NT 7
#define OSC_V_FILL_DIVS 5.2f
#define OSC_FIT_CYCLES 2.5f
static uint8_t osc_auto_stage,hold_active;
static float dbg_auto_fsel;
static int osc_vdiv_i;
static const float osc_vdivs[]={.1f,.2f,.5f,1,2};
static void UartPrint(const char*s){(void)s;}
static uint8_t AdcCaptureRaw(uint32_t n,uint32_t *us);
'''
pre += '#define ADC_RAW_MAX '+macro('ADC_RAW_MAX')+'\nstatic uint16_t adc_raw_buf[ADC_RAW_MAX];\n'
pre += '''static uint8_t AdcSetRate(int slow){dbg_adc_rate=12000000.0f/(slow?27:15);return 1;}
static uint8_t AdcCaptureRaw(uint32_t n,uint32_t *us){
 if(fail_capture)return 0;
 request_n=n;if(n>max_n)max_n=n;
 for(uint32_t i=0;i<n;i++) adc_raw_buf[i]=(uint16_t)roundf(input_dc+input_amp*(wave ? (sinf(6.28318530718f*input_f*i/dbg_adc_rate+phase)>=0?1:-1) : sinf(6.28318530718f*input_f*i/dbg_adc_rate+phase)));
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
 for(int i=0;i<240;i++)raw_samples[i]=3000;OscProcessFrame();MapToScreen(0);
 printf("calibrated DC pixel %d expected 120\\n",new_y[0]);if(new_y[0]!=120)failures++;
 float freq[]={100,20000};
 for(int k=0;k<2;k++) {osc_tdiv_i=k?3:0; input_f=freq[k];for(int settle=0;settle<6;settle++) AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();
 printf("frequency %.0f -> %.2f\\n",input_f,osc_meas.freq);
 if(fabsf(osc_meas.freq/input_f-1)>.02f) failures++;
 }
 osc_tdiv_i=6;AdcCaptureFrame();printf("DMA request %u limit 65535\\n",request_n);if(request_n>65535) failures++;
 int cases=0;
 float sweep[]={40,50,100,500,999,1000,10001,20000,48000,100000};
 for(int ti=0;ti<7;ti++)for(int fi=0;fi<10;fi++)for(int filter=1;filter<=21;filter+=10){
  osc_tdiv_i=ti;osc_fwin=filter;input_f=sweep[fi];phase=.17f+fi*.29f;adc_measure_n=3200;
  for(int settle=0;settle<7;settle++)AdcCaptureFrame();
  OscProcessFrame();OscMeasureFrame();cases++;
  if(!osc_frame_valid||fabsf(osc_meas.freq/input_f-1)>.02f){printf("FAIL sweep %d %.0f filter%d ->%.3f\\n",ti,input_f,filter,osc_meas.freq);failures++;}
  float expected=240/(osc_tdivs_ms[ti]*.01f);
  if(fabsf(osc_fs_eff/expected-1)>.0001f)failures++;
 }
 printf("sweep %d cases, max DMA %u\\n",cases,max_n);if(max_n>65535)failures++;
 float saved_fs=osc_fs_eff,saved_td=osc_capture_tdiv;
 hold_active=1;osc_auto_stage=0;AutoScale();if(osc_auto_stage||osc_fs_eff!=saved_fs)failures++;
 hold_active=0;AutoScale();fail_capture=1;AdcCaptureFrame();OscAutoScaleStep();
 if(osc_frame_valid||new_frame_ready||osc_meas.freq||dbg_adc_ok||osc_auto_stage!=1)failures++;
 if(osc_fs_eff!=saved_fs||osc_capture_tdiv!=saved_td)failures++;
 fail_capture=0;input_f=20000;AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();OscAutoScaleStep();
 if(!osc_frame_valid||!dbg_adc_ok||osc_auto_stage||osc_tdiv_i!=0||volts_per_div!=osc_vdivs[osc_vdiv_i])failures++;
 input_amp=0;for(int i=0;i<7;i++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();if(osc_meas.freq!=0)failures++;
 input_amp=10;for(int i=0;i<7;i++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();if(osc_meas.freq!=0)failures++;
 input_amp=620;input_dc=2600;wave=1;
 for(int i=0;i<7;i++)AdcCaptureFrame();OscProcessFrame();OscMeasureFrame();if(fabsf(osc_meas.freq/20000-1)>.02f)failures++;
 for(int i=0;i<240;i++)raw_samples[i]=(i%2)?3000:1000;
 osc_calib=1;osc_fwin=21;OscProcessFrame();MapToScreen(7);
 for(int i=0;i<240;i++){int y=120-(int)(osc_v[(i+7)%240]/volts_per_div*40);if(y<29)y=29;if(y>239)y=239;if(new_y[i]!=y)failures++;}
 if(osc_plot_offset!=7)failures++;
 printf("DC/no-signal/square/filter/cursor checks complete\\n");
 printf("behavior failures=%d\\n",failures);
 return failures?1:0;}
'''
(OUT/'regression.c').write_text(pre+body+main)
subprocess.run(['clang','-O0',str(OUT/'regression.c'),'-o',str(OUT/'regression')],check=True)
r=subprocess.run([str(OUT/'regression')],text=True,capture_output=True)
print(r.stdout,end='');raise SystemExit(r.returncode or bool(contract_failures))
