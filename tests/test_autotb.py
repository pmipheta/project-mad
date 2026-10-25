"""Closed-loop host test of the AUTO time base (FAST/SLOW switching), extracted from Core/Src/main.c.
The measurement chain is MODELLED (FAST reads >=39.9 Hz, SLOW needs >=2.1 cycles and f<=Fs/8); not a board test."""
from app_src import source as app_source   # the app modules as one text (see app_src.py)
from pathlib import Path
import subprocess
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__import__('tempfile').gettempdir()) / 'osc_autotb'
OUT.mkdir(parents=True, exist_ok=True)
src = app_source()
# Host harness is single-threaded: remove Step-3 locking wrappers while
# retaining the production expressions under test.
try:
    import _unlock
    src = _unlock.unlock(src)
except ImportError:
    pass
a, b = src.index('AUTO_TB_BEGIN'), src.index('AUTO_TB_END')
block = src[src.rfind('/*', 0, a):src.index('\n', b)+1]
(OUT/'atb_extract.c').write_text(block)
harness = r'''
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#define OSC_NT 18
#define OSC_NT_FAST 12
#define OSC_TDIV_DEF 8
#define OSC_H_DIVS 10
#define OSC_FIT_CYCLES 2.5f
static const float osc_tdivs_ms[] = {0.005f,0.01f,0.015f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f,5.0f,6.0f,10.0f,20.0f,50.0f,100.0f,200.0f,500.0f};
static int osc_tdiv_i = 8; static uint8_t osc_dirty, hold_active, osc_auto_stage, osc_frame_valid = 1; static volatile uint8_t slow_active;
static uint8_t osc_auto_tb = 1; static float osc_raw_vpp; static volatile uint8_t slow_fast_flag; static uint8_t adc_fast_content; static uint32_t adc_over_t;
typedef struct {float freq;} M; static volatile M osc_meas;
static uint32_t now_ms; static uint32_t HAL_GetTick(void){return now_ms;}
static double F=1000, A=2.5; static int fixed_zero=0, flat_fast=0, fast_blind_ms=0, stale_hist=0, hist_blind=0, flicker=0, glitch_t0=-1, glitch_ms=0; static double glitch_f=0;
static double lastF=-1, t_change_ms=-1e9; static uint32_t now_ms_dummy;
/* model of the history probe: a SLOW base reads F when >= 2.1 cycles fit, F <= Fs/8 and the signal is not fast */
static float SlowProbeFreq(int ti){ double t=osc_tdivs_ms[ti], fs=24000.0/t, cyc=F*t*0.01; if (hist_blind) return 0.0f; if (stale_hist && ((double)now_ms - t_change_ms) < 1500.0) return 1.0f; if (A<0.05 || fixed_zero || F>400.0) return 0.0f;
  return (cyc>=3.1 && F<=fs/8 && F>=0.2 && ((double)now_ms - t_change_ms) >= 2200.0/F) ? (float)F : 0.0f; }
static int slowish_model(void){ if (hist_blind) return 0; if (stale_hist && ((double)now_ms - t_change_ms) < 1500.0) return 1; return (A>=0.3 && !fixed_zero && F<40.0 && ((double)now_ms - t_change_ms)>=300.0) ? 1 : 0; }
static uint8_t SlowSlowish(void){ return (uint8_t)slowish_model(); }
static uint8_t osc_lock_auto=0; static int held_model=0; static double held_f=0; static uint8_t OscFreqTrusted(void){ return (osc_meas.freq>0.0f && !held_model) ? 1 : 0; }
static int recent_flat=0; static uint8_t SlowRecentFlat(void){ return recent_flat; }
#include "atb_extract.c"
static void measure(void){                       /* model of what the firmware reads in the current mode */
  osc_raw_vpp = (float)A; if (!slow_active && flat_fast) osc_raw_vpp = 0.0f; float f=0; double t=osc_tdivs_ms[osc_tdiv_i];
  slow_active = osc_tdiv_i >= OSC_NT_FAST;
  if (A >= 0.05 && !fixed_zero){
    if (!slow_active) { if (F >= 39.9 && F <= 100100 && ((double)now_ms - t_change_ms) >= fast_blind_ms && !(flicker && ((now_ms/100)&1))) f=(float)F; }
    else { for (int k=OSC_NT-1;k>=OSC_NT_FAST;k--){ double tk=osc_tdivs_ms[k], cyc=F*tk*0.01, fs=24000.0/tk; if (cyc>=3.1 && cyc<=40.0 && F<=fs/8 && F>=0.2 && ((double)now_ms - t_change_ms) >= 2200.0/F) f=(float)F; } slow_fast_flag = (F > 400.0); }   /* the firmware measures on ANY SLOW window */
  adc_fast_content = (F > 20000.0) ? 1 : 0;  /* burst spread trips for fast signals */
  }
  if (glitch_t0>=0 && (double)now_ms>=glitch_t0 && (double)now_ms<glitch_t0+glitch_ms && slow_active) f=(float)glitch_f;
  held_model=0; if (!slow_active && held_f>0 && f==0 && A>=0.3){ f=(float)held_f; held_model=1; }   /* the 1.5 s memory of the old reading */
  osc_meas.freq=f;
}
static void run(double secs){ for(double s=0;s<secs;s+=0.1){ now_ms+=100; if(F!=lastF){ lastF=F; t_change_ms=now_ms; } measure(); OscAutoTimebase(); } }
static int failures;
static void check(const char*n,int ok,const char*extra){ printf("%s %-36s tdiv_idx=%2d (%s) freq_read=%.2f %s\n",ok?"PASS":"FAIL",n,osc_tdiv_i,slow_active?"SLOW":"FAST",osc_meas.freq,extra); if(!ok)failures++; }
static void reset(int ti){ osc_tdiv_i=ti; atb_t_switch=now_ms; atb_t_bad=0; atb_t_probe=0; osc_auto_tb=1; hold_active=0; osc_auto_stage=0; slow_active=ti>=OSC_NT_FAST; measure(); }
int main(void){
  uint8_t s0=dbg_atb_switches;
  double fs_list[]={0.7,1,5,11,20,30,39,41,44,50,100,1000,100000};
  for(unsigned i=0;i<sizeof fs_list/sizeof *fs_list;i++){
    F=fs_list[i]; A=2.5; reset(8); run(40); char nm[48]; snprintf(nm,sizeof nm,"%.1f Hz from FAST start",F);
    check(nm, osc_meas.freq>0.0f && fabs(osc_meas.freq-F)<1e-3, "");
  }
  { double fl[]={0.7,5,11,30,44,60}; int want[]={17,16,16,14,14,12};   /* one wide base per range: <0.9 Hz 500 ms, <12 Hz 200 ms, <45 Hz 50 ms, above 10 ms */
    for(int q=0;q<6;q++){ F=fl[q]; A=2.5; reset(8); run(40); char nm[64]; snprintf(nm,sizeof nm,"%.0f Hz picks SLOW base %d",F,want[q]); check(nm, slow_active && osc_tdiv_i==want[q] && osc_meas.freq>0, ""); } }
  F=300000; A=2.1; reset(8); s0=dbg_atb_switches; run(120); check("300 kHz: ends FAST, <=3 switches", !slow_active && dbg_atb_switches-s0<=3, ""); A=2.5;
  F=300000; reset(17); run(30); check("300 kHz seen in SLOW: back to FAST", !slow_active, "");
  /* the friend's sequence: 1 Hz -> 2 MHz -> 1 Hz, each held 20 s (model: FAST cannot read 2 MHz or 1 Hz) */
  F=1; A=2.5; reset(8); run(40); check("1 Hz first", slow_active && osc_meas.freq>0, "");
  F=2000000; run(20); check("2 MHz: back in FAST within 20 s", !slow_active, "");
  F=1; run(30); check("2 MHz -> 1 Hz: SLOW and reading within 30 s", slow_active && osc_meas.freq>0, "");
  F=1; reset(8); run(40); F=2000000; run(15); F=1; run(12); check("2 MHz -> 1 Hz: SLOW reading within 12 s (remembers 500 ms/div)", slow_active && osc_meas.freq>0, "");
  F=20000; run(10); F=1; run(30); check("20 kHz -> 1 Hz: SLOW and reading within 30 s", slow_active && osc_meas.freq>0, "");
  F=1; reset(8); run(40); F=1000; run(60); check("1 Hz -> 1 kHz (back to FAST)", !slow_active && osc_meas.freq>0, "");
  F=1000; reset(8); run(10); F=1; run(60); check("1 kHz -> 1 Hz (to SLOW)", slow_active && osc_meas.freq>0, "");
  F=20; reset(16); run(60); check("20 Hz started on 200 ms/div", osc_meas.freq>0, "");
  A=0.0; F=100; reset(8); s0=dbg_atb_switches; run(60); check("no signal: no switching", dbg_atb_switches==s0 && osc_tdiv_i==8, "");
  A=0.1; fixed_zero=1; reset(8); s0=dbg_atb_switches; run(120); printf("     noise-only: %d switches in 120 s\n", dbg_atb_switches-s0);
  check("noise-only: bounded, ends FAST", dbg_atb_switches-s0<=6 && !slow_active, "");
  /* 11 Hz -> 1 Hz: the 50 ms window shows a flat line; within ~1.2 s the board must be on the 5 s window, reading within 5 s */
  fixed_zero=0; F=11; A=2.5; reset(8); run(40); s0=dbg_atb_switches; check("11 Hz first (SLOW, reading)", slow_active && osc_meas.freq>0, "");
  F=1; run(1.3); check("11 Hz -> 1 Hz: same 200 ms/div, no jump", slow_active && osc_tdiv_i==OSC_NT-2 && dbg_atb_switches==s0, "");
  run(4.0); check("11 Hz -> 1 Hz: reading 1 Hz within 5.3 s, still no jump", slow_active && osc_meas.freq>0 && fabs(osc_meas.freq-1.0)<0.01 && dbg_atb_switches==s0, "");
  /* 4 -> 5 -> 6 -> 7 Hz and 20 -> 10 Hz: not one switch */
  F=4; reset(16); run(20); s0=dbg_atb_switches; F=5; run(10); F=6; run(10); F=7; run(10); check("4 -> 5 -> 6 -> 7 Hz: no time base change", dbg_atb_switches==s0 && osc_tdiv_i==OSC_NT-2 && osc_meas.freq>0, "");
  F=20; reset(14); run(20); s0=dbg_atb_switches; F=15; run(10); F=10; run(10); check("20 -> 15 -> 10 Hz: no time base change", dbg_atb_switches==s0 && osc_tdiv_i==OSC_NT-4 && osc_meas.freq>0, "");
  F=1; reset(16); run(20); s0=dbg_atb_switches; F=2; run(10); F=3; run(10); F=4; run(10); check("1 -> 2 -> 3 -> 4 Hz: no time base change", dbg_atb_switches==s0 && osc_meas.freq>0, "");
  /* 1 Hz stops (generator set to 0 Hz): the old trace is still in the window (raw Vpp large) but the newest 1.5 s are flat: stay SLOW, no jump to FAST */
  fixed_zero=0; F=1; A=2.5; reset(8); run(40); recent_flat=1; fixed_zero=1; s0=dbg_atb_switches; run(20);
  check("1 Hz stops: stays SLOW, no switch for 20 s", slow_active && dbg_atb_switches==s0, ""); recent_flat=0; fixed_zero=0;
  fixed_zero=0; A=0.0; run(1); A=2.5; F=2; run(60); check("signal lost then 2 Hz: re-arms at once", slow_active && osc_meas.freq>0, "");
  /* a 4 Hz square wave looks flat to FAST (Vpp 0 in its 4-80 ms window): the history must still find it */
  flat_fast=1; A=2.5; F=4; reset(8); run(10); check("4 Hz, FAST sees only a flat line: SLOW within 10 s", slow_active && osc_meas.freq>0, ""); flat_fast=0;
  /* manual button: toggles, pauses AUTO, long press re-arms */
  F=20; A=2.5; reset(8); run(10); check("20 Hz auto -> SLOW", slow_active && osc_meas.freq>0, "");
  OscManualToggle(); measure(); check("PC13 short: SLOW -> FAST, AUTO paused", !slow_active && osc_auto_tb==0, "");
  s0=dbg_atb_switches; run(30); check("manual FAST stays (no auto switch for 30 s)", !slow_active && dbg_atb_switches==s0, "");
  OscManualToggle(); measure(); check("PC13 short: FAST -> SLOW on the right base", slow_active && osc_meas.freq>0 && osc_auto_tb==0, "");
  OscAutoRearm(); F=1000; run(15); check("PC13 long: AUTO again -> FAST at 1 kHz", osc_auto_tb==1 && !slow_active && osc_meas.freq>0, "");
  A=2.5; F=1; reset(8); osc_auto_tb=0; s0=dbg_atb_switches; run(60); check("manual: never switches", dbg_atb_switches==s0 && osc_tdiv_i==8, "");
  F=1; reset(8); hold_active=1; s0=dbg_atb_switches; run(60); check("HOLD: never switches", dbg_atb_switches==s0, "");
  /* a signal appears after silence (0 Hz -> 1 Hz, or after a fast signal): SLOW takes over at once, no 2 s of analysis in FAST */
  flat_fast=1; A=2.5; F=1000; reset(8); run(10); F=1; s0=dbg_atb_switches; run(0.9); check("1 kHz -> 1 Hz: SLOW within 0.9 s (rolls in)", slow_active && osc_tdiv_i==OSC_NT-2, ""); flat_fast=0;
  A=2.5; F=2000000; reset(8); run(10); A=0.0; run(5); A=2.5; F=1; flat_fast=1; run(0.9); check("2 MHz -> 0 -> 1 Hz: SLOW within 0.9 s", slow_active, ""); flat_fast=0;
  /* the lock (still picture) follows the frequency with hysteresis 35 / 45 Hz */
  F=50; A=2.5; reset(8); run(10); check("50 Hz: picture locked", osc_lock_auto==1, ""); F=40; run(10); check("40 Hz: still locked (hysteresis)", osc_lock_auto==1, ""); F=30; run(10); check("30 Hz: rolling", osc_lock_auto==0, ""); F=40; run(10); check("40 Hz from below: still rolling", osc_lock_auto==0, ""); F=60; run(10); check("60 Hz: locked again", osc_lock_auto==1, "");
  /* PARK after silence: SLOW at a faster base moves to 200 ms/div after 2.4 s of flat window, never earlier, never from other bases, and the next signal still gets its own base */
  { F=30; A=2.5; reset(14); run(20); check("30 Hz is on 50 ms/div", osc_tdiv_i==14 && osc_meas.freq>0, "");
    A=0.01; run(1.5); check("1.5 s silent: not parked yet", osc_tdiv_i==14, "");
    run(1.6); check("3.1 s silent: parked at 200 ms/div", osc_tdiv_i==16 && slow_active, "");
    { uint8_t s1=dbg_atb_switches; run(10); check("long silence: no more switching", osc_tdiv_i==16 && dbg_atb_switches==s1, ""); }
    F=1; A=2.5; run(8); check("1 Hz after silence: stays on 200 ms/div", osc_tdiv_i==16 && osc_meas.freq>0, "");
    F=30; A=0.01; reset(14); run(1.0); A=2.5; run(20); check("30 Hz back after 1 s silence: no park", osc_tdiv_i==14 && osc_meas.freq>0, "");
    F=30; A=0.01; reset(14); run(2.0); A=2.5; run(20); check("30 Hz back after 2 s silence: no park", osc_tdiv_i==14 && osc_meas.freq>0, "");
    A=0.01; reset(17); run(6); check("silence on 500 ms/div: stays", osc_tdiv_i==17, "");
    A=0.01; reset(8); run(10); check("silence in FAST: stays (no park)", !slow_active && osc_tdiv_i==8, "");
    F=30; A=2.5; reset(14); run(20); A=0.01; run(6); F=100; A=2.5; run(10); check("100 Hz after park: FAST", !slow_active && osc_meas.freq>0, ""); }
  /* 1 Hz -> 101 Hz: SLOW reads >90 Hz and goes to FAST; FAST needs ~1 s for its first reading while the history still holds the old 1 Hz. It must NOT go back to SLOW
   * (board log: 5 FAST<->SLOW switches in 2.3 s, a different picture every time). */
  { F=1; A=2.5; fast_blind_ms=0; stale_hist=0; reset(16); run(20); check("1 Hz settled in SLOW", slow_active && osc_meas.freq>0, "");
    F=101; fast_blind_ms=900; stale_hist=1; { uint8_t s1=dbg_atb_switches; run(6); int n=dbg_atb_switches-s1;
      char ex[40]; snprintf(ex,sizeof ex,"switches=%d",n); check("1 -> 101 Hz: no FAST<->SLOW ping-pong", n<=2 && !slow_active && osc_meas.freq>0, ex); }
    F=101; fast_blind_ms=1200; stale_hist=1; F=1; reset(16); run(20); F=101; { uint8_t s1=dbg_atb_switches; run(6); check("1 -> 101 Hz, FAST blind 1.2 s: still no ping-pong", dbg_atb_switches-s1<=2 && !slow_active && osc_meas.freq>0, ""); }
    fast_blind_ms=0; stale_hist=0;
    /* the hold-off must not trap a genuinely slow signal in FAST: 100 Hz -> 5 Hz (FAST readable <70 -> SLOW at once) and an unreadable FAST with a slow signal still returns to SLOW */
    F=100; A=2.5; reset(11); run(10); F=5; run(10); check("100 -> 5 Hz: SLOW", slow_active && osc_meas.freq>0, "");
    F=5; A=2.5; fixed_zero=0; reset(11); run(10); check("slow signal found while in FAST (no hold-off pending): SLOW", slow_active, ""); }
  /* 101 -> 1 Hz: FAST sees a straight line. The history is not ready (hist_blind), yet the board must show the slow view within ~1.2 s instead of waiting for the probes. */
  { hist_blind=1; F=101; A=2.5; reset(11); run(10); check("101 Hz in FAST", !slow_active && osc_meas.freq>0, "");
    F=1; run(0.6); check("0.6 s after 101 -> 1 Hz: not switched yet", !slow_active, "");
    run(1.0); check("1.6 s after 101 -> 1 Hz: SLOW 200 ms/div", slow_active && osc_tdiv_i==16, "");
    /* a 100 Hz signal that FAST reads on every other frame must never trigger it */
    hist_blind=1; flicker=1; F=100; A=2.5; reset(11); { uint8_t s1=dbg_atb_switches; run(20); check("100 Hz read every other frame: no switch", !slow_active && dbg_atb_switches==s1, ""); } flicker=0;
    /* silence in FAST and a clearly fast unreadable signal: stay */
    hist_blind=1; A=0.01; F=100; reset(11); run(10); check("silence in FAST: stay", !slow_active, ""); A=2.5;
    F=300000; reset(8); run(10); check("fast unreadable signal (adc_fast_content): stay in FAST", !slow_active, ""); F=100;
    /* the hold-off after SLOW -> FAST still protects the first second */
    hist_blind=1; F=1; A=2.5; reset(16); run(10); F=101; fast_blind_ms=1200; run(6); check("1 -> 101 Hz with blind history: ends in FAST", !slow_active && osc_meas.freq>0, ""); fast_blind_ms=0;
    hist_blind=0; }
  /* one bogus reading (board log: 68.5 Hz while the signal was 1 Hz) must not move the base; a real change still does */
  { F=1; A=2.5; hist_blind=0; reset(16); run(20); check("1 Hz settled", slow_active && osc_tdiv_i==16, "");
    { uint8_t g0=dbg_atb_switches; glitch_f=68.5; glitch_t0=(int)now_ms+100; glitch_ms=100; run(3); glitch_t0=-1; check("one bogus 68.5 Hz reading (100 ms): base never moves", osc_tdiv_i==16 && dbg_atb_switches==g0, ""); }
    { uint8_t g0=dbg_atb_switches; glitch_f=68.5; glitch_t0=(int)now_ms+100; glitch_ms=200; run(3); glitch_t0=-1; check("two bogus readings (200 ms): base never moves", osc_tdiv_i==16 && dbg_atb_switches==g0, ""); }
    F=30; run(10); check("real change 1 -> 30 Hz: base follows (50 ms/div)", osc_tdiv_i==14 && osc_meas.freq>0, "");
    F=3; run(10); check("real change 30 -> 3 Hz: base follows (200 ms/div)", osc_tdiv_i==16 && osc_meas.freq>0, ""); glitch_t0=-1; }
  /* the photo: 101 Hz -> 1 Hz, FAST still shows F=101 (held) over a sloping line. The held number must not count as readable. */
  { hist_blind=1; held_f=101; F=1; A=2.5; reset(11); run(0.5); check("held F=101 over a 1 Hz line: not switched in 0.5 s", !slow_active, "");
    run(1.5); check("held F=101 over a 1 Hz line: SLOW within 2 s", slow_active && osc_tdiv_i==16, ""); held_f=0; hist_blind=0; held_model=0; }
  printf("autotb failures=%d\n",failures); return failures; }
'''
(OUT/'atb_test.c').write_text(harness)
subprocess.run(['cc','-O1','-Wall','-o',str(OUT/'atb_test'),str(OUT/'atb_test.c'),'-lm'],check=True,cwd=OUT)
r = subprocess.run([str(OUT/'atb_test')],capture_output=True,text=True); print(r.stdout, end='')
raise SystemExit(r.returncode)
