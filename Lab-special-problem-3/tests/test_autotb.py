"""Closed-loop host test of the AUTO time base (FAST/SLOW switching), extracted from Core/Src/main.c.
The measurement chain is MODELLED (FAST reads >=39.9 Hz, SLOW needs >=2.1 cycles and f<=Fs/8); not a board test."""
from pathlib import Path
import subprocess
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__import__('tempfile').gettempdir()) / 'osc_autotb'
OUT.mkdir(parents=True, exist_ok=True)
src = (ROOT/'Core/Src/main.c').read_text()
a, b = src.index('AUTO_TB_BEGIN'), src.index('AUTO_TB_END')
block = src[src.rfind('/*', 0, a):src.index('\n', b)+1]
(OUT/'atb_extract.c').write_text(block)
harness = r'''
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#define OSC_NT 16
#define OSC_NT_FAST 12
#define OSC_TDIV_DEF 8
#define OSC_H_DIVS 10
#define OSC_FIT_CYCLES 2.5f
static const float osc_tdivs_ms[] = {0.005f,0.01f,0.015f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f,5.0f,6.0f,50.0f,100.0f,200.0f,500.0f};
static int osc_tdiv_i = 8; static uint8_t osc_dirty, hold_active, osc_auto_stage, osc_frame_valid = 1; static volatile uint8_t slow_active;
static uint8_t osc_auto_tb = 1; static float osc_raw_vpp; static volatile uint8_t slow_fast_flag; static uint8_t adc_fast_content; static uint32_t adc_over_t;
typedef struct {float freq;} M; static volatile M osc_meas;
static uint32_t now_ms; static uint32_t HAL_GetTick(void){return now_ms;}
#include "atb_extract.c"
static double F=1000, A=2.5; static int fixed_zero=0;
static void measure(void){                       /* model of what the firmware reads in the current mode */
  osc_raw_vpp = (float)A; float f=0; double t=osc_tdivs_ms[osc_tdiv_i];
  slow_active = osc_tdiv_i >= OSC_NT_FAST;
  if (A >= 0.05 && !fixed_zero){
    if (!slow_active) { if (F >= 39.9 && F <= 100100) f=(float)F; }
    else { double cyc=F*t*0.01, fs=24000.0/t; if (cyc>=2.1 && F<=fs/8 && F>=0.2) f=(float)F; slow_fast_flag = (F > 400.0); }
  adc_fast_content = (F > 20000.0) ? 1 : 0;  /* burst spread trips for fast signals */
  }
  osc_meas.freq=f;
}
static void run(double secs){ for(double s=0;s<secs;s+=0.1){ now_ms+=100; measure(); OscAutoTimebase(); } }
static int failures;
static void check(const char*n,int ok,const char*extra){ printf("%s %-36s tdiv_idx=%2d (%s) freq_read=%.2f %s\n",ok?"PASS":"FAIL",n,osc_tdiv_i,slow_active?"SLOW":"FAST",osc_meas.freq,extra); if(!ok)failures++; }
static void reset(int ti){ osc_tdiv_i=ti; atb_t_switch=now_ms; atb_t_bad=0; atb_t_backoff=0; atb_try=0; osc_auto_tb=1; hold_active=0; osc_auto_stage=0; slow_active=ti>=OSC_NT_FAST; measure(); }
int main(void){
  uint8_t s0=dbg_atb_switches;
  double fs_list[]={0.5,1,5,11,20,30,39,41,44,50,100,1000,100000};
  for(unsigned i=0;i<sizeof fs_list/sizeof *fs_list;i++){
    F=fs_list[i]; A=2.5; reset(8); run(40); char nm[48]; snprintf(nm,sizeof nm,"%.1f Hz from FAST start",F);
    check(nm, osc_meas.freq>0.0f && fabs(osc_meas.freq-F)<1e-3, "");
  }
  F=300000; A=2.1; reset(8); s0=dbg_atb_switches; run(120); check("300 kHz: ends FAST, <=3 switches", !slow_active && dbg_atb_switches-s0<=3, ""); A=2.5;
  F=300000; reset(15); run(30); check("300 kHz seen in SLOW: back to FAST", !slow_active, "");
  /* the friend's sequence: 1 Hz -> 2 MHz -> 1 Hz, each held 20 s (model: FAST cannot read 2 MHz or 1 Hz) */
  F=1; A=2.5; reset(8); run(40); check("1 Hz first", slow_active && osc_meas.freq>0, "");
  F=2000000; run(20); check("2 MHz: back in FAST within 20 s", !slow_active, "");
  F=1; run(30); check("2 MHz -> 1 Hz: SLOW and reading within 30 s", slow_active && osc_meas.freq>0, "");
  F=1; reset(8); run(40); F=2000000; run(15); F=1; run(12); check("2 MHz -> 1 Hz: SLOW reading within 12 s (remembers 500 ms/div)", slow_active && osc_meas.freq>0, "");
  F=20000; run(10); F=1; run(30); check("20 kHz -> 1 Hz: SLOW and reading within 30 s", slow_active && osc_meas.freq>0, "");
  F=1; reset(8); run(40); F=1000; run(60); check("1 Hz -> 1 kHz (back to FAST)", !slow_active && osc_meas.freq>0, "");
  F=1000; reset(8); run(10); F=1; run(60); check("1 kHz -> 1 Hz (to SLOW)", slow_active && osc_meas.freq>0, "");
  F=20; reset(14); run(60); check("20 Hz started on 200 ms/div", osc_meas.freq>0, "");
  A=0.0; F=100; reset(8); s0=dbg_atb_switches; run(60); check("no signal: no switching", dbg_atb_switches==s0 && osc_tdiv_i==8, "");
  A=0.1; fixed_zero=1; reset(8); s0=dbg_atb_switches; run(120); printf("     noise-only: %d switches in 120 s\n", dbg_atb_switches-s0);
  check("noise-only: bounded, ends FAST", dbg_atb_switches-s0<=6 && !slow_active, "");
  fixed_zero=0; A=0.0; run(1); A=2.5; F=2; run(60); check("signal lost then 2 Hz: re-arms at once", slow_active && osc_meas.freq>0, "");
  A=2.5; F=1; reset(8); osc_auto_tb=0; s0=dbg_atb_switches; run(60); check("manual: never switches", dbg_atb_switches==s0 && osc_tdiv_i==8, "");
  F=1; reset(8); hold_active=1; s0=dbg_atb_switches; run(60); check("HOLD: never switches", dbg_atb_switches==s0, "");
  printf("autotb failures=%d\n",failures); return failures; }
'''
(OUT/'atb_test.c').write_text(harness)
subprocess.run(['cc','-O1','-Wall','-o',str(OUT/'atb_test'),str(OUT/'atb_test.c'),'-lm'],check=True,cwd=OUT)
r = subprocess.run([str(OUT/'atb_test')],capture_output=True,text=True); print(r.stdout, end='')
raise SystemExit(r.returncode)
