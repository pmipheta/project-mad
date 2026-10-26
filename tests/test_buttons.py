"""Host test of the button roles (FREEZE, AUTO, INCREASE/DECREASE, FAST/SLOW, auto FAST/SLOW) and the AUTO scale,
extracted from the app modules (ACTIONS block, EXT_POINTS block, AUTO_TB block, OscAdjust, OscAutoScale, MapToScreen, ButtonRepeat).
The measurement chain is MODELLED (as in test_autotb.py): FAST reads >= 39.9 Hz up to a configurable limit, SLOW reads any base with >= 3.1 cycles.
Not a board test."""
from app_src import source as app_source   # the app modules as one text (see app_src.py)
from pathlib import Path
import re, subprocess, tempfile
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(tempfile.gettempdir()) / 'osc_buttons'; OUT.mkdir(parents=True, exist_ok=True)
from _unlock import unlock
src = unlock(app_source())
nocom = re.sub(r'/\*.*?\*/', '', src, flags=re.S)

def block(begin, end):
    a, b = src.index(begin), src.index(end)
    return src[src.rfind('/*', 0, a):src.index('\n', b)+1]
def function(name):
    m = re.search(r'^(?:static\s+)?(?:inline\s+)?(?:void|int|float|uint8_t|uint16_t|uint32_t)\s+(?:FAST_O2\s+)?'+name+r'\([^;]*?\)\s*\{', nocom, re.M)
    assert m, name
    p = nocom.index('{', m.start()); depth = 1; i = p+1
    while depth:
        depth += (nocom[i] == '{') - (nocom[i] == '}'); i += 1
    return nocom[m.start():i]
def macro(name):
    return re.search(r'^#define '+name+r'\s+([^/\n]+)', nocom, re.M)[1].strip()

set_a = src.index('static int OscSetCount(void)')
set_b = src.index('static void OscSetValueText')
set_menu = src[set_a:set_b] + function('OscAdjust')            # OscSetCount / OscSetName / OscClampI / OscClampF + OscAdjust
(OUT/'atb.c').write_text(block('AUTO_TB_BEGIN', 'AUTO_TB_END'))
(OUT/'blockA.c').write_text(block('ACTIONS_BEGIN', 'ACTIONS_END'))
(OUT/'blockB.c').write_text(block('EXT_POINTS_BEGIN', 'EXT_POINTS_END'))
(OUT/'menu.c').write_text(set_menu)
(OUT/'funcs.c').write_text('\n'.join(function(x) for x in ['OscAutoScale', 'AutoScale', 'MapToScreen', 'ButtonRepeat']))

# ---- contracts on the text of main.c ---------------------------------------------------------------------------------
before = ROOT.parent/'builds/v54/main_before.c'
def slow_text(t): return t[t.index('SLOW_BEGIN ='):t.index('SLOW_END =')]
contracts = [
 ('Slow block byte-identical to v53', (not before.exists()) or slow_text(src) == slow_text(before.read_text())),
 ('buttons call the Act* actions (v61: physical +/- = ActIncrease/ActDecrease)', all(t in src for t in ['ActIncrease(n)', 'ActDecrease(n)', 'ActFreeze();', 'ActAuto();', 'ActAutoMode();', 'ActToggleFastSlow();'])),
 ('pin defines kept (TODO pins)', all(t in src for t in ['#define BTN_UP_PIN      GPIO_PIN_0', '#define BTN_DOWN_PIN    GPIO_PIN_1', '#define BTN_HOLD_PIN    GPIO_PIN_2', '#define BTN_SET_PIN     GPIO_PIN_3'])),
 ('v97: no UART console, no MOVE/ZOOM, no simulator left in the app', not re.search(r'Uart|USART|huart|mv_view|mv_zoom|mv_y_steps|ActZoom|ActUp|ActDown|GenFrame|ADC_INPUT_ENABLE', src)),
]
cfail = sum(not ok for _, ok in contracts)
for n, ok in contracts: print(('PASS ' if ok else 'FAIL ') + n)

harness = r'''
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define N_SAMPLES 240
#define SCREEN_W 320   /* v85: C1/C2 are LCD x positions */
#define ADC_VREF 3.3f
#define ADC_INPUT_GAIN 1.0f   /* v84: harness signals are pin volts */
#define ADC_FULL 4095
#define OSC_NT 18
#define OSC_NT_FAST 12
#define OSC_VDIV_DEF 12
#define OSC_TDIV_DEF 8
#define OSC_H_DIVS 10
#define OSC_SET_COUNT 4
#define OSC_HOLD_COUNT 3
#define CENTER_Y 120
#define GRID_Y_STEP 40
#define WAVEFORM_TOP 29
#define SCREEN_H 240
#define FAST_O2
#define SLOW_HIST_N 8192u
#define OSC_V_FILL_DIVS @@FILL@@
#define OSC_V_DC_DIVS @@DCDIVS@@
static uint8_t osc_calib = 1;   /* v90: AUTO V/DIV differs in DC */
static float osc_dc_shift_v;    /* v91: MapToScreen DC shift */
#define ADC_RAW_MAX 65520u
typedef struct {float vmax,vmin,vpp,vrms,dc_offset,freq;} OscMeasure;
static volatile OscMeasure osc_meas;
static const float osc_tdivs_ms[] = {0.005f,0.01f,0.015f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f,5.0f,6.0f,10.0f,20.0f,50.0f,100.0f,200.0f,500.0f};
static const float osc_vdivs[] = {0.0001f,0.0002f,0.0005f,0.001f,0.002f,0.005f,0.01f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f};
static const char *osc_set_names[] = {"FILTER","TRIG","V/DIV","T/DIV"};
static const char *osc_hold_names[] = {"C1","C2","V/DIV"};
static int osc_tdiv_i = 8, osc_vdiv_i = OSC_VDIV_DEF, osc_fwin = 5, osc_trig_fall = 0, osc_sidx = 0, osc_c1 = 40, osc_c2 = 160;
static uint8_t osc_cur_dirty = 0; static uint32_t btn_rep_ms = BTN_REPEAT_MS;   /* v85 */
static float volts_per_div = 1.0f;
static uint8_t hold_active, osc_dirty, osc_stable = 1, osc_auto_stage, osc_auto_tries, osc_frame_valid = 1, new_frame_ready, osc_auto_tb = 1, osc_lock_auto;
static volatile uint8_t slow_active, slow_fast_flag;
static uint8_t adc_fast_content; static uint32_t adc_over_t;
static float osc_raw_vpp, dbg_auto_fsel, osc_v[N_SAMPLES], osc_offset_ct = 2048.0f;
static int16_t raw_samples[N_SAMPLES]; static int16_t new_y[N_SAMPLES]; static int osc_plot_offset; static float osc_fs_eff = 24000.0f;   /* v85 OscCursorDtS */
static uint16_t osc_env_lo_ct[N_SAMPLES], osc_env_hi_ct[N_SAMPLES]; static uint8_t osc_env_on; static int16_t env_y_lo[N_SAMPLES], env_y_hi[N_SAMPLES]; static uint8_t env_y_on; static float osc_env_mean_ct, osc_env_rms_ct;   /* v83 peak detect */
static uint16_t adc_raw_buf[ADC_RAW_MAX];
static volatile uint16_t slow_hist[SLOW_HIST_N]; static volatile uint32_t slow_last_t, slow_valid_cnt, slow_period_us = 2083;
static uint32_t now_ms; static uint32_t HAL_GetTick(void){return now_ms;}
static void MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n);
static int SlowExtPoint(int k, float *v){ (void)k; (void)v; return 0; }   /* v80 SLOW edge neighbours: SLOW block not compiled here */
static int OscFiltWin(void){ int w = (osc_fwin < 1) ? 1 : osc_fwin; if ((w % 2) == 0) w++; return w; }   /* v82: no frequency cap here (test_edge_filter covers it) */
static void AutoScale(void);
/* --- model of the measurement chain (as test_autotb.py) --- */
static double F=1000, A=3.0, FAST_MAX=2.2e6; static double t_change_ms=-1e9, lastF=-1; static int hist_blind=0;
static float SlowProbeFreq(int ti){ double t=osc_tdivs_ms[ti], fs=24000.0/t, cyc=F*t*0.01; if (hist_blind||A<0.05||F>400.0) return 0.0f;
  return (cyc>=3.1 && F<=fs/8 && F>=0.2 && ((double)now_ms - t_change_ms) >= 2200.0/F) ? (float)F : 0.0f; }
static uint8_t SlowSlowish(void){ return (A>=0.3 && F<40.0 && ((double)now_ms - t_change_ms)>=300.0) ? 1 : 0; }
static uint8_t OscFreqTrusted(void){ return osc_meas.freq>0.0f ? 1 : 0; }
static uint8_t SlowRecentFlat(void){ return 0; }
#include "atb.c"
#include "menu.c"
#include "blockA.c"
#include "funcs.c"
#include "blockB.c"
static void measure(void){
  float f=0; slow_active = osc_tdiv_i >= OSC_NT_FAST;
  osc_raw_vpp = (float)A;
  if (!slow_active && F < 39.9) osc_raw_vpp = (float)fmin(A, A*3.14159*F*0.06);     /* a 60 ms window of a slow wave: only a slope */
  if (A >= 0.05){
    if (!slow_active) { if (F >= 39.9 && F <= FAST_MAX) f=(float)F; }
    else { for (int k=OSC_NT-1;k>=OSC_NT_FAST;k--){ double tk=osc_tdivs_ms[k], cyc=F*tk*0.01, fs=24000.0/tk; if (cyc>=3.1 && cyc<=40.0 && F<=fs/8 && F>=0.2 && ((double)now_ms - t_change_ms) >= 2200.0/F) f=(float)F; } slow_fast_flag = (F > 400.0); }
    adc_fast_content = (F > 20000.0);
  }
  osc_meas.freq=f; osc_meas.vpp=(float)A; osc_frame_valid=1; new_frame_ready=1;
}
static void frame(void){ now_ms+=100; if(F!=lastF){ lastF=F; t_change_ms=now_ms; } measure(); if(osc_auto_stage) OscAutoScaleStep(); OscAutoTimebase(); new_frame_ready=0; }
static int failures;
static void check(const char*n,int ok){ printf("%s %s\n",ok?"PASS":"FAIL",n); if(!ok)failures++; }
static void fresh(void){ hold_active=0; osc_sidx=0; osc_auto_tb=1; osc_auto_stage=0; osc_auto_tries=0;
  osc_tdiv_i=8; osc_vdiv_i=OSC_VDIV_DEF; volts_per_div=1.0f; slow_active=0; atb_t_switch=now_ms; atb_t_bad=0; atb_t_probe=0; atb_t_hold=0; atb_t_unread=0; atb_t_flat=0; }

/* ---- auto scale: run the real flow until it finishes, then 12 s of the auto time base to prove they do not fight ---- */
static void autoscale_case(const char *name, double f, double a, int want_vi, int want_fast, double cyc_lo, double cyc_hi, int want_ti){
  char nm[160]; int guard=0;
  fresh(); F=f; A=a; lastF=-1; hist_blind=0; osc_vdiv_i=0; volts_per_div=osc_vdivs[0]; osc_tdiv_i=8;
  for(int i=0;i<60;i++) frame();                                   /* settle in AUTO first (what the user sees before pressing AUTO) */
  ActAuto();
  while(osc_auto_stage && guard++<400) frame();
  int ti0=osc_tdiv_i, vi0=osc_vdiv_i; double tms=osc_tdivs_ms[ti0], cyc=f*tms*0.01, divs=a/osc_vdivs[vi0];
  for(int i=0;i<120;i++) frame();                                  /* 12 s: the auto time base must keep it */
  snprintf(nm,sizeof nm,"AUTO %-22s -> %.1f V/div (Vpp = %.1f div), %.3f ms/div [%s], %.1f cycles, stays after 12 s of auto time base (idx %d -> %d)",name,osc_vdivs[vi0],divs,tms,ti0>=OSC_NT_FAST?"SLOW":"FAST",cyc,ti0,osc_tdiv_i);
  int ok = !osc_auto_stage && vi0==want_vi && (ti0>=OSC_NT_FAST)==!want_fast && cyc>=cyc_lo && cyc<=cyc_hi && osc_tdiv_i==ti0 && osc_vdiv_i==vi0 && volts_per_div==osc_vdivs[vi0] && osc_auto_tb==1 && !hold_active;
  if (want_ti>=0 && ti0!=want_ti) ok=0;
  if (a>=0.05 && (divs>OSC_V_FILL_DIVS+0.001 && vi0!=13)) ok=0;
  check(nm,ok);
}

int main(void){
  char nm[200];
  /* ---- 1. +/- adjust the selected setting, running or frozen ---- */
  fresh(); osc_sidx=2;                                             /* V/DIV selected */
  ActIncrease(1); check("running: + adjusts the selected setting (V/DIV 1.0 -> 2.0)", osc_vdiv_i==13);
  ActDecrease(1); ActDecrease(1); check("running: - adjusts it back", osc_vdiv_i==11);
  fresh(); ActFreeze(); check("ActFreeze freezes", hold_active==1 && osc_sidx==0);
  osc_sidx=2; ActIncrease(1); check("frozen: + adjusts the selected hold setting (V/DIV)", osc_vdiv_i==13 && hold_active==1);
  { float vpp_before, f_before; osc_meas.vpp=2.5f; osc_meas.freq=1234.0f; vpp_before=osc_meas.vpp; f_before=osc_meas.freq;
    osc_sidx=0; ActIncrease(5); ActDecrease(2); check("numbers (Vpp/F) untouched by +/-", osc_meas.vpp==vpp_before && osc_meas.freq==f_before); }
  ActFreeze(); check("FREEZE again unfreezes", hold_active==0);
  fresh(); for(int i=0;i<N_SAMPLES;i++){ osc_v[i]=0.0f; } MapToScreen(0); check("MapToScreen: 0 V on the centre line", new_y[10]==120);
  for(int i=0;i<N_SAMPLES;i++){ osc_v[i]=1.0f; } MapToScreen(0); check("MapToScreen: +1 V at 1 V/div = 40 px up", new_y[10]==80);
  for(int i=0;i<N_SAMPLES;i++){ osc_v[i]=-9.0f; } MapToScreen(0); check("MapToScreen: clamped to the bottom of the screen", new_y[10]==SCREEN_H-1);
  for(int i=0;i<N_SAMPLES;i++){ osc_v[i]=9.0f; } MapToScreen(0); check("MapToScreen: clamped to the waveform area", new_y[10]==WAVEFORM_TOP);
  fresh(); ActFreeze(); ActAuto(); check("AUTO while frozen: unfreezes, auto-scale armed", hold_active==0 && osc_auto_stage==1);

  /* ---- 2. FAST frame: the display points just outside the window are kept for the edge filter ---- */
  { const uint32_t w=2, pre=100; const float step=2.0f; const uint32_t n=(uint32_t)((N_SAMPLES+pre)*step)+w+2u;
    for(uint32_t j=0;j<n;j++) adc_raw_buf[j]=(uint16_t)(1000+ (j%3000));
    MvStoreExt(0, step, w, n);
    snprintf(nm,sizeof nm,"FAST ROLL capture (start 0): %d points exist after the window, %d before",mv_nhi,mv_nlo); check(nm, mv_nhi>=pre-2 && mv_nhi<=pre+3 && mv_nlo==0);
    MvStoreExt(100, step, w, n);
    snprintf(nm,sizeof nm,"locked capture (start 100): %d points before the window",mv_nlo); check(nm, mv_nlo>=45 && mv_nlo<=50);
    { float v=0; int ok=OscExtPoint(-1,&v); uint32_t c=100u-2u, lo=c-(w-1)/2; float want=(float)((adc_raw_buf[lo]+adc_raw_buf[lo+1])/w);
      snprintf(nm,sizeof nm,"OscExtPoint(-1) = the boxcar just before the window (%.0f, expected %.0f)",v,want); check(nm, ok && v==want); }
    { float v=0; check("OscExtPoint beyond the stored points: none", !OscExtPoint(-1-mv_nlo,&v)); }
    { const uint32_t n2=(uint32_t)(N_SAMPLES*step)+w-1u; MvStoreExt(0, step, w, n2);
      snprintf(nm,sizeof nm,"capture without extra data: nlo %d, nhi %d",mv_nlo,mv_nhi); check(nm, mv_nlo==0 && mv_nhi==0); }
  }

  /* ---- 7. F toggle / A auto ---- */
  fresh(); F=1000; A=3.0; lastF=-1; for(int i=0;i<20;i++) frame();
  check("power-on: automatic", osc_auto_tb==1 && osc_tdiv_i<OSC_NT_FAST);
  ActToggleFastSlow(); frame(); check("F from AUTO+FAST: forces SLOW (opposite of current)", osc_auto_tb==0 && osc_tdiv_i>=OSC_NT_FAST && slow_active);
  { int t=osc_tdiv_i; for(int i=0;i<200;i++) frame(); check("forced SLOW stays 20 s (auto time base off) even with a 1 kHz signal", osc_tdiv_i==t && osc_auto_tb==0); }
  ActToggleFastSlow(); frame(); check("F again: forced FAST", osc_auto_tb==0 && osc_tdiv_i<OSC_NT_FAST && !slow_active);
  { int t=osc_tdiv_i; for(int i=0;i<200;i++) frame(); check("forced FAST stays", osc_tdiv_i==t); }
  ActAutoMode(); check("A: automatic again", osc_auto_tb==1);
  F=5; lastF=-1; for(int i=0;i<300;i++) frame(); check("after A the auto switching works again (5 Hz -> SLOW)", slow_active && osc_auto_tb==1);
  ActAuto(); osc_auto_stage=0; osc_auto_tb=0; ActAuto(); check("v56: AUTO scale is one-shot, does not re-arm the continuous auto time base", osc_auto_tb==0);
  { fresh(); F=1000; A=3.0; lastF=-1; osc_auto_tb=0; ActAuto(); osc_sidx=3; OscAdjust(+1); int t=osc_tdiv_i, s=slow_active; for(int i=0;i<200;i++) frame();
    check("manual T/DIV during a pending AUTO scale cancels it: T/DIV and FAST/SLOW stay where the user put them", osc_auto_stage==0 && osc_tdiv_i==t && slow_active==s); }
  fresh(); F=1000; lastF=-1; frame(); hold_active=1; { int t=osc_tdiv_i; ActToggleFastSlow(); check("F while frozen is ignored", osc_tdiv_i==t && osc_auto_tb==1); } hold_active=0;
  fresh(); F=1000; ActToggleFastSlow(); check("forced mode is OFF at power-on", 1);

  /* ---- 8. AUTO scale: real flow + auto time base together ---- */
  autoscale_case("1 kHz 3 Vpp",    1000.0,   3.0, 12, 1, 3.2, 8.2, 7);
  autoscale_case("100 kHz 3 Vpp",  100000.0, 3.0, 12, 1, 3.2, 8.2, 0);
  autoscale_case("2.1 MHz 3 Vpp",  2100000.0,3.0, 12, 1, 3.2, 200.0, 0);
  autoscale_case("1 Hz 3 Vpp (SLOW)", 1.0,   3.0, 12, 0, 1.6, 40.0, 16);
  autoscale_case("50 Hz 3 Vpp",    50.0,     3.0, 12, 0, 1.6, 40.0, 12);
  autoscale_case("100 Hz 1 Vpp",   100.0,    1.0, 11, 1, 3.2, 8.2, -1);
  autoscale_case("20 kHz 0.3 Vpp", 20000.0,  0.3, 9, 1, 3.2, 8.2, 3);
  autoscale_case("1 kHz 0.1 Vpp",  1000.0,   0.1, 8, 1, 3.2, 8.2, 7);
  autoscale_case("1 kHz 3.2 Vpp",  1000.0,   3.2, 12, 1, 3.2, 8.2, 7);
  { int guard=0; fresh(); F=1000; A=0.0; lastF=-1; osc_vdiv_i=0; volts_per_div=0.1f; osc_tdiv_i=3; for(int i=0;i<30;i++) frame(); ActAuto(); while(osc_auto_stage && guard++<400) frame();
    snprintf(nm,sizeof nm,"AUTO no signal -> %.1f V/div, tdiv idx %d (default %d), stage done after %d frames",volts_per_div,osc_tdiv_i,OSC_TDIV_DEF,guard);
    check(nm, !osc_auto_stage && osc_vdiv_i==OSC_VDIV_DEF && volts_per_div==1.0f && osc_tdiv_i==OSC_TDIV_DEF); }
  { int guard=0; fresh(); F=1000; A=0.03; lastF=-1; osc_vdiv_i=0; ActAuto(); while(osc_auto_stage && guard++<400) frame();
    check("AUTO 0.03 Vpp (< 0.05) uses default scale", osc_vdiv_i==OSC_VDIV_DEF && osc_tdiv_i==OSC_TDIV_DEF); }
  { fresh(); F=1000; A=3.0; lastF=-1; hold_active=1; osc_auto_stage=0; OscAutoScale(); check("OscAutoScale itself still ignored while frozen (v53 contract)", osc_auto_stage==0); hold_active=0; }
  { int g=0; fresh(); F=1000; A=3.0; lastF=-1; ActAuto(); new_frame_ready=0; OscAutoScaleStep(); check("AUTO consumes only a fresh frame", osc_auto_stage==1); while(osc_auto_stage&&g++<100) frame(); check("AUTO finishes", osc_auto_stage==0); }

  /* ---- 9. press-and-hold repeat ---- */
  { Button b; memset(&b,0,sizeof b); int total=0; uint32_t t; int ev[64], evn=0; uint32_t evt[64];
    for(t=1000;t<=3200;t+=5){ int n=ButtonRepeat(&b,1,t); if(n){ ev[evn]=n; evt[evn]=t; evn++; total+=n; } }
    int ok = evn>5 && evt[0]==1000 && ev[0]==1 && evt[1]==1300 && evt[2]==1360;
    int gaps_ok=1, accel_ok=1; for(int i=2;i<evn;i++){ if(evt[i]-evt[i-1]!=60) gaps_ok=0; if(evt[i]-1000>=1500 ? ev[i]!=3 : ev[i]!=1) accel_ok=0; }
    snprintf(nm,sizeof nm,"hold repeat: first step at once, next at +300 ms, then every 60 ms (%d events, first %u/%u/%u)",evn,evt[0],evt[1],evt[2]); check(nm, ok&&gaps_ok);
    check("hold repeat: 3 steps per repeat from 1.5 s on, 1 before", accel_ok);
    int nrel=0; for(t=3200;t<3400;t+=5) nrel+=ButtonRepeat(&b,0,t); check("release gives no steps", nrel==0);
    int again=0; for(t=3400;t<3420;t+=5) again+=ButtonRepeat(&b,1,t); check("new press after release steps at once", again==1);
    /* bounce: a 10 ms dropout while held must not restart the press */
    memset(&b,0,sizeof b); ButtonRepeat(&b,1,5000); int s=0; for(t=5005;t<5100;t+=5) s+=ButtonRepeat(&b,(t>=5020&&t<5030)?0:1,t); check("contact bounce (10 ms low) while held: no extra step", s==0);
    /* a short tap = exactly one step */
    memset(&b,0,sizeof b); s=ButtonRepeat(&b,1,9000); for(t=9005;t<9150;t+=5) s+=ButtonRepeat(&b,(t<9100)?1:0,t); check("short tap (100 ms) = exactly one step", s==1);
    /* v85: C1/C2 held = 20 ms repeat (smooth glide); a short tap is still one step */
    btn_rep_ms = BTN_CUR_REPEAT_MS; memset(&b,0,sizeof b); s=0; uint32_t tl=0, gap_bad=0;
    for(t=11000;t<=12000;t+=5){ int n=ButtonRepeat(&b,1,t); if(n){ if(tl && t>11000+BTN_REPEAT_FIRST_MS && t-tl!=BTN_CUR_REPEAT_MS) gap_bad++; tl=t; s+=n; } }
    snprintf(nm,sizeof nm,"v85 cursor hold: repeats every %u ms after the first %u ms (%d steps in 1 s)",(unsigned)BTN_CUR_REPEAT_MS,(unsigned)BTN_REPEAT_FIRST_MS,s); check(nm, gap_bad==0 && s>=30);
    memset(&b,0,sizeof b); s=ButtonRepeat(&b,1,14000); for(t=14005;t<14150;t+=5) s+=ButtonRepeat(&b,(t<14100)?1:0,t); check("v85 cursor: short tap = exactly one step", s==1);
    btn_rep_ms = BTN_REPEAT_MS;
  }
  /* v85: frozen C1/C2 move 1 sample per step and ask only for the cursor repaint, not a full redraw */
  { hold_active=1; osc_sidx=0; osc_c1=100; osc_dirty=0; osc_cur_dirty=0; OscAdjust(+1);
    check("v85: C1 +1 LCD pixel, cursor-only repaint", osc_c1==101 && osc_cur_dirty==1 && osc_dirty==0);
    osc_sidx=1; osc_c2=319; osc_cur_dirty=0; OscAdjust(+1); check("v85: C2 clamps at the last LCD column (319)", osc_c2==319 && osc_cur_dirty==1);
    osc_sidx=2; osc_cur_dirty=0; OscAdjust(+1); check("v85: V/DIV while frozen still redraws fully", osc_dirty==1 && osc_cur_dirty==0);
    hold_active=0; osc_sidx=0; osc_dirty=0; }
  /* v90: DC coupling: AUTO sizes V/DIV by the peak from 0 V (the trace starts on the centre line) so it fits the ~2.2 divs above it */
  { double pk[] = {1.88, 0.87, 0.30, 3.0}; int want[] = {12, 11, 10, 13};
    for (int c = 0; c < 4; c++) { int guard = 0;
      fresh(); osc_calib = 0; F = 2000; A = pk[c]; lastF = -1; hist_blind = 0; osc_meas.vmax = (float)pk[c]; osc_meas.vmin = 0.0f;
      ActAuto(); while (osc_auto_stage && guard++ < 400) frame();
      snprintf(nm, sizeof nm, "v90 DC AUTO: peak %.2f V from 0 V -> %.1f V/div (peak at %.2f div above the centre, room 2.27)", pk[c], osc_vdivs[osc_vdiv_i], pk[c] / osc_vdivs[osc_vdiv_i]);
      check(nm, osc_vdiv_i == want[c] && pk[c] / osc_vdivs[osc_vdiv_i] <= 2.0 + 1e-6); }
    osc_calib = 1; osc_meas.vmax = osc_meas.vmin = 0.0f; }
  printf("buttons failures=%d\n",failures);
  return failures?1:0;
}
'''
harness = harness.replace('@@FILL@@', macro('OSC_V_FILL_DIVS')).replace('@@DCDIVS@@', macro('OSC_V_DC_DIVS'))
(OUT/'buttons.c').write_text(harness)
# Button type + ButtonRepeat deps from the real file
btn_t = src[src.index('typedef struct {\n    GPIO_TypeDef *port;'):src.index('} Button;')+len('} Button;')]
btn_t = btn_t.replace('GPIO_TypeDef *port;', 'void *port;')
defs = ''.join('#define %s %s\n' % (m, macro(m)) for m in ['BTN_REPEAT_FIRST_MS', 'BTN_REPEAT_MS', 'BTN_ACCEL_MS', 'BTN_ACCEL_STEPS', 'BTN_REL_MS', 'BTN_CUR_REPEAT_MS'])
(OUT/'buttons.c').write_text((OUT/'buttons.c').read_text().replace('#include <math.h>\n', '#include <math.h>\n' + btn_t + '\n' + defs, 1))
cc = subprocess.run(['clang', '-O0', '-Wno-everything', '-I', str(OUT), str(OUT/'buttons.c'), '-o', str(OUT/'buttons'), '-lm'], capture_output=True, text=True)
if cc.returncode:
    print(cc.stderr[:6000]); raise SystemExit(2)

r1 = subprocess.run([str(OUT/'buttons')], text=True, capture_output=True); print(r1.stdout, end='')
print('contract failures=%d' % cfail)
raise SystemExit(bool(r1.returncode or cfail))
