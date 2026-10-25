"""Host test of OscFrameHold (extracted from Core/Src/main.c): which frames are not drawn."""
from app_src import source as app_source   # the app modules as one text (see app_src.py)
from pathlib import Path
import subprocess, tempfile
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(tempfile.gettempdir()) / 'osc_hold'; OUT.mkdir(parents=True, exist_ok=True)
src = app_source()
a, b = src.index('HOLD_BEGIN'), src.index('HOLD_END')
(OUT/'hold_extract.c').write_text(src[src.rfind('/*', 0, a):src.index('\n', b)+1])
(OUT/'h.c').write_text(r"""
#include <stdio.h>
#include <stdint.h>
static const float osc_tdivs_ms[] = {0.005f,0.01f,0.015f,0.02f,0.05f,0.1f,0.2f,0.5f,1.0f,2.0f,5.0f,6.0f,10.0f,20.0f,50.0f,100.0f,200.0f,500.0f};
static uint8_t adc_f_held; static int osc_disp_cross = 6;
#define OSC_H_DIVS 10
static int osc_tdiv_i = 11; static float osc_capture_tdiv = 6.0f; static volatile uint8_t slow_active; static float osc_raw_vpp = 3.0f; static uint8_t adc_fast_content;
typedef struct { float freq; } M; static volatile M osc_meas;
#include "hold_extract.c"
static int failures;
static void chk(const char *n, int got, int want){ printf("%s %s (hold=%d)\n", got==want?"PASS":"FAIL", n, got); if(got!=want) failures++; }
int main(void){
  uint32_t t = 10000;
  osc_meas.freq = 0; chk("never had a good frame: draw", OscFrameHold(t), 0);
  osc_meas.freq = 101; chk("readable FAST frame: draw", OscFrameHold(t), 0);
  osc_meas.freq = 0; osc_raw_vpp = 3.0f; chk("unreadable real signal 100 ms later: hold", OscFrameHold(t+100), 1);
  chk("... 400 ms later: still hold", OscFrameHold(t+400), 1);
  chk("... 600 ms later: draw (the limit)", OscFrameHold(t+600), 0);
  osc_meas.freq = 101; OscFrameHold(t+700); osc_meas.freq = 0; osc_raw_vpp = 0.1f; chk("signal collapsed to 0.1 V 50 ms after a good frame: hold", OscFrameHold(t+750), 1);
  chk("... 1.5 s later: still hold", OscFrameHold(t+700+1500), 1);
  chk("... 2.1 s later: draw (a real silence is shown)", OscFrameHold(t+700+2100), 0);
  osc_meas.freq = 101; OscFrameHold(t+800); osc_meas.freq = 0; osc_raw_vpp = 3.0f; adc_fast_content = 1; chk("clearly fast unreadable signal: draw", OscFrameHold(t+850), 0); adc_fast_content = 0;
  osc_meas.freq = 101; OscFrameHold(t+900); osc_meas.freq = 0; slow_active = 1; chk("SLOW frame: always draw", OscFrameHold(t+950), 0); slow_active = 0;
  osc_meas.freq = 101; OscFrameHold(t+1000); osc_meas.freq = 0; osc_tdiv_i = 9; osc_capture_tdiv = 2.0f; chk("time base changed since the good frame: draw", OscFrameHold(t+1050), 0);
  osc_tdiv_i = 16; osc_capture_tdiv = 6.0f; chk("frame captured under another base than selected: skip", OscFrameHold(t+1100), 1);
  osc_capture_tdiv = 200.0f; chk("next frame under the selected base: draw", OscFrameHold(t+1200), 0);
  osc_tdiv_i = 11; osc_capture_tdiv = 6.0f; osc_meas.freq = 101; OscFrameHold(t+2000); osc_meas.freq = 0; osc_raw_vpp = 3.0f;
  { int h = 0; for (int k = 1; k <= 20; k++) h += OscFrameHold(t+2000+k*100); printf("%s at most 5 consecutive frames held at 100 ms per frame (held %d of 20)\n", (h>=4 && h<=5)?"PASS":"FAIL", h); if(!(h>=4&&h<=5)) failures++; }
  /* held frequency: the memory of an earlier reading. 101 Hz -> 1 Hz: F still says 101 (held) but the 60 ms window is a sloping line (0 crossings) */
  osc_tdiv_i = 11; osc_capture_tdiv = 6.0f; slow_active = 0; adc_f_held = 0; osc_disp_cross = 6; osc_meas.freq = 101; osc_raw_vpp = 3.0f; adc_fast_content = 0;
  chk("measured F with 6 crossings: trusted", OscFreqTrusted(), 1);
  adc_f_held = 1; chk("held F, picture has 6 crossings (100 Hz read on every other frame): trusted", OscFreqTrusted(), 1);
  osc_disp_cross = 0; chk("held F=101 but the window shows no crossing: NOT trusted", OscFreqTrusted(), 0);
  osc_disp_cross = 1; chk("held F=101, 1 crossing: NOT trusted", OscFreqTrusted(), 0);
  osc_meas.freq = 2; osc_disp_cross = 0; chk("held F=2 Hz (0.12 cycles in 60 ms) is not judged by crossings: trusted", OscFreqTrusted(), 1);
  osc_meas.freq = 101; adc_f_held = 0; osc_disp_cross = 6; OscFrameHold(t+5000);                       /* good frame */
  adc_f_held = 1; osc_disp_cross = 0; osc_raw_vpp = 0.52f; chk("held-F sloping line with Vpp 0.52 V (the photo): vpp < 0.30? no, 0.52 -> hold picture", OscFrameHold(t+5100), 1);
  chk("... still held 400 ms later", OscFrameHold(t+5400), 1);
  chk("... drawn after 0.5 s", OscFrameHold(t+5600), 0);
  adc_f_held = 1; osc_disp_cross = 6; osc_raw_vpp = 3.0f; osc_meas.freq = 100; { int h = 0; for (int k = 0; k < 20; k++) { adc_f_held = (k & 1); h += OscFrameHold(t+9000+k*100); } printf("%s 100 Hz read on every other frame: never held (held %d of 20)\n", h==0?"PASS":"FAIL", h); if (h) failures++; }
  printf("hold failures=%d\n", failures); return failures?1:0; }
""")
r = subprocess.run(['gcc','-O0','-Wall','-I',str(OUT),'-o',str(OUT/'h'),str(OUT/'h.c'),'-lm'],capture_output=True,text=True)
print(r.stderr[:600])
print(subprocess.run([str(OUT/'h')],capture_output=True,text=True).stdout, end='')
