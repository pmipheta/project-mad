"""v56 'real oscilloscope picture' host test.
Re-uses the harness of test_buttons.py (real C of the AUTO_TB / MOVE blocks, OscAutoScale, OscAdjust from Core/Src/main.c, with the
MODELLED measurement chain: FAST reads >= 39.9 Hz, SLOW reads any base with >= 3.1 cycles) and replaces its main().
Source contracts on main.c itself (boot state, osc_stable default, Slow block, unchanged measurement code) are checked too.
Not a board test."""
from pathlib import Path
import re, subprocess, tempfile, difflib
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(tempfile.gettempdir()) / 'osc_realpic'; OUT.mkdir(parents=True, exist_ok=True)
tb = (ROOT/'tests/test_buttons.py').read_text()

# --- take the preamble + harness of test_buttons.py, drop its contracts / UART part / main ---
i_main = tb.index("int main(void){\n  char nm[200];")
pre = tb[:i_main]
tail_start = tb.index("'''\nharness = harness.replace('@@FILL@@'")
tail_end = tb.index("# ---- UartCommand compiled for real")
tail = tb[tail_start:tail_end]
main_c = r'''int main(void){
  char nm[200];
  /* ---- (a) boot state (the model's fresh() is the old v55 boot; the real boot values are checked in the source contracts) ---- */
  fresh(); osc_auto_tb = 0;                                         /* v56 boot */
  check("(a) boot: 1 ms/div FAST, 1 V/div, continuous auto time base OFF", osc_tdiv_i==8 && osc_tdivs_ms[osc_tdiv_i]==1.0f && !slow_active && osc_vdiv_i==3 && osc_vdivs[osc_vdiv_i]==1.0f && osc_auto_tb==0);

  /* ---- (b) nothing re-zooms for 30 s simulated ---- */
  { double fl[] = {1000.0, 100000.0, 1.0};
    for (int q=0;q<3;q++){
      fresh(); osc_auto_tb=0; F=fl[q]; A=3.0; lastF=-1; hist_blind=0; uint8_t s0=dbg_atb_switches;
      for(int k=0;k<300;k++) frame();                                /* 30 s */
      snprintf(nm,sizeof nm,"(b) %.0f Hz, 30 s: T/DIV idx %d (1 ms), V/DIV idx %d (1 V), FAST, no switch, osc_auto_tb 0",F,osc_tdiv_i,osc_vdiv_i);
      check(nm, osc_tdiv_i==8 && osc_vdiv_i==3 && !slow_active && dbg_atb_switches==s0 && osc_auto_tb==0 && volts_per_div==1.0f);
    }
    fresh(); osc_auto_tb=0; F=1000; A=3.0; lastF=-1; for(int k=0;k<150;k++) frame(); F=50000; for(int k=0;k<150;k++) frame();
    check("(b) 1 kHz -> 50 kHz after 15 s, 30 s total: T/DIV and V/DIV unchanged", osc_tdiv_i==8 && osc_vdiv_i==3 && !slow_active && osc_auto_tb==0);
    fresh(); osc_auto_tb=0; F=1000; A=3.0; lastF=-1; for(int k=0;k<100;k++) frame(); F=1; for(int k=0;k<200;k++) frame();
    check("(b) 1 kHz -> 1 Hz: no automatic rescue (stays 1 ms/div FAST)", osc_tdiv_i==8 && !slow_active && osc_auto_tb==0);
    fresh(); osc_auto_tb=0; F=2000000; A=3.0; lastF=-1; for(int k=0;k<300;k++) frame();
    check("(b) 2.1 MHz-class signal at 1 ms/div: no rescue", osc_tdiv_i==8 && osc_auto_tb==0);
  }

  /* ---- (c) 'z' = ONE SHOT: pick once, then stay for 30 s, osc_auto_tb stays 0 ---- */
  { struct { double f; double a; int want_fast; int ti_lo, ti_hi; double cyc_lo, cyc_hi; const char *name; } c[] = {
      {1000.0,   3.0, 1,  0, 11, 1.9, 5.0,  "1 kHz  (2-4 cycles visible)"},
      {100000.0, 3.0, 1,  0, 11, 1.9, 99.0, "100 kHz"},
      {1.0,      3.0, 0, 12, 17, 1.5, 99.0, "1 Hz   (SLOW)"},
      {2100000.0,3.0, 1,  0,  0, 1.9, 999999.0, "2.1 MHz (fastest base)"},
      {50.0,     3.0, 0, 12, 12, 1.9, 5.5,  "50 Hz  (10 ms/div is SLOW in the firmware: SLOW_MIN_TDIV_MS = 10)"},
    };
    for (int q=0;q<5;q++){
      fresh(); osc_auto_tb=0; F=c[q].f; A=c[q].a; lastF=-1; hist_blind=0; int guard=0;
      for(int k=0;k<30;k++) frame();
      uint8_t s0=dbg_atb_switches;
      ActAuto();
      while(osc_auto_stage && guard++<600) frame();
      int ti0=osc_tdiv_i, vi0=osc_vdiv_i; double cyc=c[q].f*osc_tdivs_ms[ti0]*0.01;
      int fast0 = (ti0 < OSC_NT_FAST);
      for(int k=0;k<300;k++) frame();                                /* 30 s after: fixed */
      snprintf(nm,sizeof nm,"(c) z at %s: T/DIV idx %d (%.3f ms/div) %s, %.2f cycles, V/DIV idx %d; unchanged after 30 s; osc_auto_tb=%d",c[q].name,ti0,osc_tdivs_ms[ti0],fast0?"FAST":"SLOW",cyc,vi0,osc_auto_tb);
      check(nm, !osc_auto_stage && osc_auto_tb==0 && osc_tdiv_i==ti0 && osc_vdiv_i==vi0 && dbg_atb_switches==s0 && (fast0==c[q].want_fast) && ti0>=c[q].ti_lo && ti0<=c[q].ti_hi && cyc>=c[q].cyc_lo && cyc<=c[q].cyc_hi);
    }
    /* z from a SLOW start, and from an old A-mode state: z itself must not turn the continuous mode on */
    fresh(); osc_auto_tb=0; F=1000; A=3.0; lastF=-1; for(int k=0;k<20;k++) frame(); ActAuto(); check("(c) pressing z does not set osc_auto_tb", osc_auto_tb==0);
    for(int g=0; osc_auto_stage && g<400; g++) frame();
    ActToggleFastSlow(); frame(); check("(c) F after z: manual toggle works and osc_auto_tb stays 0", osc_auto_tb==0 && slow_active);
  }

  /* ---- (d) 'A' brings the old continuous behaviour back ---- */
  fresh(); osc_auto_tb=0; F=1000; A=3.0; lastF=-1; for(int k=0;k<50;k++) frame();
  ActAutoMode(); check("(d) A: continuous auto time base on", osc_auto_tb==1);
  F=5; lastF=-1; for(int k=0;k<300;k++) frame(); check("(d) A mode: 5 Hz -> SLOW by itself", slow_active && osc_auto_tb==1);
  F=1000; lastF=-1; for(int k=0;k<300;k++) frame(); check("(d) A mode: 1 kHz -> back to FAST with ~6 cycles by itself", !slow_active && osc_tdiv_i<OSC_NT_FAST && osc_meas.freq>0);
  F=100000; lastF=-1; for(int k=0;k<300;k++) frame(); { double cyc=100000*osc_tdivs_ms[osc_tdiv_i]*0.01; check("(d) A mode: 100 kHz re-fitted to ~3-14 cycles", cyc>=3.0 && cyc<=14.0); }

  /* ---- (e) numbers at the fixed 1 ms/div (model of the v55 numbers: F = signal, Vpp = amplitude) ---- */
  { double fl[] = {1000.0, 100000.0};
    for (int q=0;q<2;q++){ fresh(); osc_auto_tb=0; F=fl[q]; A=3.0; lastF=-1; for(int k=0;k<50;k++) frame();
      snprintf(nm,sizeof nm,"(e) 1 ms/div fixed: %.0f Hz reads F=%.0f, Vpp=%.2f",F,osc_meas.freq,osc_meas.vpp);
      check(nm, osc_tdiv_i==8 && fabsf(osc_meas.freq-(float)F)<0.5f && fabsf(osc_meas.vpp-3.0f)<1e-4f); }
    fresh(); osc_auto_tb=0; F=1; A=3.0; lastF=-1; for(int k=0;k<50;k++) frame();
    snprintf(nm,sizeof nm,"(e) 1 ms/div fixed, 1 Hz: F=%.2f (FAST cannot read it: 0, not a false value); F via z/SLOW reads 1 Hz below",osc_meas.freq);
    check(nm, osc_meas.freq==0.0f);
    ActAuto(); for(int g=0; osc_auto_stage && g<600; g++) frame(); for(int k=0;k<80;k++) frame();
    check("(e) after z the 1 Hz signal reads 1.000 Hz in SLOW", slow_active && fabsf(osc_meas.freq-1.0f)<0.01f);
  }

  /* ---- (f) trigger lock: the lock follows the reading even with the fixed time base (osc_lock_auto is set before the osc_auto_tb gate) ---- */
  fresh(); osc_auto_tb=0; osc_lock_auto=0; F=1000; A=3.0; lastF=-1; for(int k=0;k<20;k++) frame();
  check("(f) fixed T/DIV, 1 kHz: trigger lock engaged (osc_lock_auto=1)", osc_lock_auto==1 && osc_auto_tb==0);
  F=100; lastF=-1; for(int k=0;k<20;k++) frame();
  check("(f) fixed T/DIV, 100 Hz: locked again", osc_lock_auto==1);

  printf("realpic model failures=%d\n",failures);
  return failures?1:0;
}
'''
harness_py = pre + main_c + tail
# the 'tail' defines `harness = harness.replace(...)` and writes/compiles buttons.c; reuse as is, but with our own output name
harness_py = harness_py.replace("OUT = Path(tempfile.gettempdir()) / 'osc_buttons'", "OUT = Path(tempfile.gettempdir()) / 'osc_realpic_build'")
# strip the contracts section of the preamble (it is test_buttons' own) but keep the harness text
k0 = harness_py.index("# ---- contracts on the text of main.c")
k1 = harness_py.index("harness = r'''")
harness_py = harness_py[:k0] + harness_py[k1:]
harness_py += "r1 = subprocess.run([str(OUT/'buttons')], text=True, capture_output=True); print(r1.stdout, end='')\nMODEL_RC = r1.returncode\n"
ns = {'__file__': str(ROOT/'tests/test_realpic.py'), '__name__': 'realpic_inner'}
exec(compile(harness_py, 'realpic_harness', 'exec'), ns)

# ---- contracts on the text of main.c ------------------------------------------------------------------------------------
src = (ROOT/'Core/Src/main.c').read_text()
before = ROOT.parent/'builds/v57/main_before.c'
def slow_text(t): return t[t.index('SLOW_BEGIN ='):t.index('SLOW_END =')]
bt = before.read_text() if before.exists() else None
def has(p): return re.search(p, src) is not None
def set_sites():
    return [m.start() for m in re.finditer(r'osc_auto_tb\s*=\s*1\s*;', src)]
contracts = [
 ('(a) boot: osc_auto_tb = 0', has(r'static uint8_t\s+osc_auto_tb\s*=\s*0;')),
 ('(a) boot: T/DIV index = 1 ms/div (OSC_TDIV_DEF 8), V/DIV index 3 = 1 V', has(r'#define OSC_TDIV_DEF\s+8\b') and has(r'osc_tdiv_i\s*=\s*OSC_TDIV_DEF') and has(r'osc_vdiv_i\s*=\s*3;') and 'static const float osc_tdivs_ms[] = {0.005f,0.01f,0.015f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f' in src.replace(' ,', ',') or has(r'osc_tdivs_ms\[\][^;]*0\.5f,\s*1\.0f')),
 ('(f) default picture: osc_stable = 1 (trigger-locked)', has(r'static uint8_t\s+osc_stable\s*=\s*1;')),
 ("(f) UART 'm' still toggles roll/stable", 'osc_stable = (uint8_t)!osc_stable;' in src and 'osc_stable = 0;' in src and 'osc_stable = 1;' in src),
 ("only OscAutoRearm ('A') sets osc_auto_tb = 1", len(set_sites()) == 1 and 'static void OscAutoRearm(void)' in src[src.rfind('static void', 0, set_sites()[0]):set_sites()[0]+40]),
 ("'z' path (OscAutoScale) does not touch osc_auto_tb", 'osc_auto_tb' not in src[src.index('static void OscAutoScale(void)'):src.index('static float SlowProbeFreq')].replace('/* v56: one-shot. \'z\' does not touch osc_auto_tb (continuous auto time base only via \'A\') */','')),
 ('boot state saved: main_before.c exists', bt is not None),
 ('Slow block byte-identical to main_before.c (v56)', bt is not None and slow_text(src) == slow_text(bt)),
]
if bt is not None:
    def fn(t, name):
        m = re.search(r'^static [^\n;]*\b'+name+r'\(', t, re.M); a = m.start(); p = t.index('{', a); d = 1; q = p+1
        while d: d += (t[q]=='{') - (t[q]=='}'); q += 1
        return t[a:q]
    for name in ['OscMeasureFrame', 'OscProcessFrame', 'AdcCaptureFrame', 'OscAutoTimebase', 'OscManualToggle', 'OscAutoApply', 'OscAutoScaleStep', 'OscAutoPickTdiv', 'ActToggleFastSlow', 'ActAutoMode', 'OscAutoRearm', 'OscAdjust']:
        contracts.append(('function unchanged vs v56: '+name, fn(src, name) == fn(bt, name)))
    diff = [l for l in difflib.unified_diff(bt.splitlines(), src.splitlines(), lineterm='', n=0) if l[:1] in '+-' and l[:3] not in ('+++', '---')]
    contracts.append(('whole-file diff vs v56 is small (<= 300 changed lines; the tight v57 contract is in test_zoom): %d' % len(diff), len(diff) <= 300))
cfail = sum(not ok for _, ok in contracts)
for n, ok in contracts: print(('PASS ' if ok else 'FAIL ') + n)
print('contract failures=%d' % cfail)
fails = cfail + (1 if ns.get('MODEL_RC') else 0)
print('realpic failures=%d' % fails)
raise SystemExit(bool(fails))
