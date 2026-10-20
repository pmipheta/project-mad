"""v57 ZOOM host test (frozen picture, '+'/'-' on the MOVE axis).
Re-uses the harness of test_buttons.py (real C of the MOVE / MOVE_DATA blocks, MapToScreen, OscAdjust ... from Core/Src/main.c) like test_realpic.py,
adds real C of UartPoll, and checks source contracts against builds/v57/main_before.c (= v56): Slow block byte-identical, nothing outside
move/zoom/draw/uart code changed. Not a board test."""
from pathlib import Path
import re, subprocess, tempfile
ROOT = Path(__file__).resolve().parents[1]
tb = (ROOT/'tests/test_buttons.py').read_text()

i_main = tb.index("int main(void){\n  char nm[200];")
pre = tb[:i_main]
tail_start = tb.index("'''\nharness = harness.replace('@@FILL@@'")
tail_end = tb.index("# ---- UartCommand compiled for real")
tail = tb[tail_start:tail_end]

main_c = r'''
#define KV (3.3f/4095.0f)
static void zfresh(void){ fresh(); mv_zoom_x=1; mv_zoom_y=1; mv_x_q=0; mv_raw_n=0; mv_nlo=mv_nhi=0; mv_slow_mode=0; slow_active=0; osc_fwin=1; osc_offset_ct=2048.0f; mv_axis_x=0; }
static void to_osc_v(void){ for(int i=0;i<N_SAMPLES;i++) osc_v[i]=((float)raw_samples[i]-osc_offset_ct)*KV; }
/* FAST capture model: 2 sine cycles over the screen (+ optional period-4 square ripple that a boxcar of w=8 removes) */
static double SINP(double j,double step){ return 2048.0 + 600.0*sin(6.283185307179586*j/(240.0*step/2.0)); }
static void fast_setup(float step, uint32_t pre_pts, uint32_t start, double hf){
  uint32_t w=(uint32_t)(step+0.5f), n=(uint32_t)((N_SAMPLES+2*pre_pts)*step)+w+2u;
  for(uint32_t j=0;j<n;j++) adc_raw_buf[j]=(uint16_t)(SINP(j,step) + (((j%4)<2)?hf:-hf));
  for(int i=0;i<N_SAMPLES;i++){ int32_t c=(int32_t)start+(int32_t)((float)i*step+0.5f); int32_t lo=c-(int32_t)((w-1)/2); uint32_t acc=0; if(lo<0)lo=0;
    for(uint32_t q=0;q<w;q++) acc+=adc_raw_buf[(uint32_t)lo+q]; raw_samples[i]=(int16_t)(acc/w); }
  to_osc_v(); MvStoreExt(start, step, w, n);
}
static float maxdiff(const float*v){ float m=0; for(int i=1;i<N_SAMPLES;i++){ float d=fabsf(v[i]-v[i-1]); if(d>m)m=d; } return m; }
static void slow_setup(double ripple, uint32_t period_us){
  slow_active=1; slow_period_us=period_us; slow_valid_cnt=8192; slow_last_t=20000;
  for(uint32_t t=0;t<SLOW_HIST_N;t++){ uint32_t tt=19000u+((t+SLOW_HIST_N-(19000u&8191u))&8191u); (void)tt; }
  for(uint32_t t=11000;t<=20000;t++) slow_hist[t&8191]=(uint16_t)(2048.0+800.0*sin(6.283185307179586*(double)t/2000.0)+((t&1)?ripple:-ripple));
}
static void slow_freeze(void){ ActFreeze(); for(int j=0;j<N_SAMPLES;j++) raw_samples[j]=(int16_t)MvSlowPoint(j,0); to_osc_v(); }

int main(void){
  char nm[240];
  /* ---- 1. zoom ignored when not frozen ---- */
  zfresh(); ActZoom(+1); mv_axis_x=1; ActZoom(+1); ActZoom(+1);
  check("not frozen: '+' changes nothing (X and Y axis)", mv_zoom_x==1 && mv_zoom_y==1 && mv_view_on==0 && hold_active==0);
  { volts_per_div=1.0f; for(int i=0;i<N_SAMPLES;i++) osc_v[i]=0.5f; MapToScreen(0); check("not frozen: picture unchanged (0.5 V at 1 V/div = 20 px above centre)", new_y[7]==100); }

  /* ---- 2. limits: X max 16, Y max 8, min 1 ---- */
  zfresh(); ActFreeze(); mv_axis_x=1; { int seq[8], ok=1; for(int i=0;i<7;i++){ ActZoom(+1); seq[i]=mv_zoom_x; }
    snprintf(nm,sizeof nm,"X zoom in x7: %d %d %d %d %d %d %d (2 4 8 16 16 16 16)",seq[0],seq[1],seq[2],seq[3],seq[4],seq[5],seq[6]);
    check(nm, seq[0]==2&&seq[1]==4&&seq[2]==8&&seq[3]==16&&seq[4]==16&&seq[5]==16&&seq[6]==16&&mv_zoom_y==1); (void)ok; }
  for(int i=0;i<8;i++) ActZoom(-1); check("X zoom out below 1 stays 1", mv_zoom_x==1);
  mv_axis_x=0; { int s[6]; for(int i=0;i<6;i++){ ActZoom(+1); s[i]=mv_zoom_y; }
    snprintf(nm,sizeof nm,"Y zoom in x6: %d %d %d %d %d %d (2 4 8 8 8 8)",s[0],s[1],s[2],s[3],s[4],s[5]);
    check(nm, s[0]==2&&s[1]==4&&s[2]==8&&s[3]==8&&s[5]==8&&mv_zoom_x==1); }
  for(int i=0;i<6;i++) ActZoom(-1); check("Y zoom out below 1 stays 1", mv_zoom_y==1);
  mv_axis_x=0; ActZoom(+1); mv_axis_x=1; ActZoom(+1); ActZoom(+1); check("zoom follows the MOVE axis: Y x2 and X x4 independent", mv_zoom_y==2 && mv_zoom_x==4);

  /* ---- 3. reset on unfreeze / '0' / AUTO ---- */
  zfresh(); ActFreeze(); mv_axis_x=1; ActZoom(+1); ActZoom(+1); mv_axis_x=0; ActZoom(+1); ActFreeze();
  check("unfreeze: zoom back to x1/x1", hold_active==0 && mv_zoom_x==1 && mv_zoom_y==1 && mv_view_on==0 && mv_x_q==0);
  zfresh(); ActFreeze(); mv_axis_x=1; ActZoom(+1); mv_axis_x=0; ActZoom(+1); ActMoveCenter();
  check("'0': zoom back to x1/x1 and stays frozen", hold_active==1 && mv_zoom_x==1 && mv_zoom_y==1 && mv_view_on==0);
  zfresh(); ActFreeze(); mv_axis_x=1; ActZoom(+1); ActZoom(+1); mv_axis_x=0; ActZoom(+1); ActAuto();
  check("AUTO: unfrozen, zoom x1/x1", hold_active==0 && mv_zoom_x==1 && mv_zoom_y==1 && osc_auto_stage==1);
  zfresh(); hold_active=1; MvSync(); mv_zoom_x=4; mv_zoom_y=2; hold_active=0; MvSync(); check("freeze made elsewhere: unfreeze still resets zoom", mv_zoom_x==1 && mv_zoom_y==1);

  /* ---- 4. header text: effective T/DIV and V/DIV ---- */
  { char b[40];
    MvFmtT(b,sizeof b,1.0f/4.0f);   check("T 1 ms/div x4 -> 'T:0.250ms/d'", strcmp(b,"T:0.250ms/d")==0);
    MvFmtT(b,sizeof b,0.005f/16.0f); snprintf(nm,sizeof nm,"T 5 us/div x16 -> '%s' (T:0.31us/d)",b); check(nm, strcmp(b,"T:0.31us/d")==0);
    MvFmtT(b,sizeof b,200.0f/2.0f); check("T 200 ms/div x2 -> 'T:100.0ms/d'", strcmp(b,"T:100.0ms/d")==0);
    MvFmtT(b,sizeof b,2.0f/2.0f);   check("T 2 ms/div x2 -> 'T:1.00ms/d'", strcmp(b,"T:1.00ms/d")==0);
    MvFmtV(b,sizeof b,1.0f/4.0f);   check("V 1 V/div x4 -> 'V:250.0mV/d'", strcmp(b,"V:250.0mV/d")==0);
    MvFmtV(b,sizeof b,2.0f/2.0f);   check("V 2 V/div x2 -> 'V:1.00V/d'", strcmp(b,"V:1.00V/d")==0);
    MvFmtV(b,sizeof b,0.5f/8.0f);   check("V 0.5 V/div x8 -> 'V:62.5mV/d'", strcmp(b,"V:62.5mV/d")==0);
    zfresh(); mv_zoom_x=4; MvFmtZ(b,sizeof b);     check("Z text X only", strncmp(b,"Z:x4",4)==0);
    mv_zoom_x=1; mv_zoom_y=8; MvFmtZ(b,sizeof b);  check("Z text Y only", strncmp(b,"Z:x8",4)==0);
    mv_zoom_x=16; mv_zoom_y=2; MvFmtZ(b,sizeof b); check("Z text both", strcmp(b,"Z:x16/x2")==0);
    mv_zoom_x=1; mv_zoom_y=1; MvFmtZ(b,sizeof b);  check("Z text x1: blank", b[0]==' ' && b[1]==' '); }

  /* ---- 5. Y zoom pixels + clipping ---- */
  zfresh(); volts_per_div=1.0f; ActFreeze();
  for(int i=0;i<N_SAMPLES;i++) osc_v[i]=(i<100)?0.5f:(i<200?-0.5f:0.0f);
  MapToScreen(0); check("Y x1: +0.5 V = 100 px, -0.5 V = 140 px, 0 V = 120", new_y[10]==100 && new_y[150]==140 && new_y[210]==120);
  mv_axis_x=0; ActZoom(+1); MapToScreen(0); check("Y x2: +0.5 V = 80 px, -0.5 V = 160 px, 0 V stays on the centre", new_y[10]==80 && new_y[150]==160 && new_y[210]==120);
  ActZoom(+1); MapToScreen(0); check("Y x4: +-0.5 V = +-80 px (40 / 200)", new_y[10]==40 && new_y[150]==200);
  ActZoom(+1); MapToScreen(0); check("Y x8: clipped to the waveform area top / screen bottom", new_y[10]==WAVEFORM_TOP && new_y[150]==SCREEN_H-1 && new_y[210]==120);
  { int ok=1; for(int i=0;i<N_SAMPLES;i++) if(new_y[i]<WAVEFORM_TOP||new_y[i]>=SCREEN_H) ok=0; check("Y x8: every pixel inside the waveform area", ok); }
  mv_y_steps=10; ActZoom(-1); ActZoom(-1); MapToScreen(0); check("Y move offset still 1 div (40 px) at x2: 0.5 V -> 80-40 = 40", new_y[10]==40);

  /* ---- 6. FAST time zoom ---- */
  { const float step=8.0f; const uint32_t pre_pts=100, start=800;
    zfresh(); fast_setup(step, pre_pts, start, 0.0);
    snprintf(nm,sizeof nm,"FAST capture: %d points before, %d after the window, raw copy %u samples",mv_nlo,mv_nhi,(unsigned)mv_raw_n); check(nm, mv_nlo>=95 && mv_nhi>=95 && mv_raw_n>=2000);
    ActFreeze(); mv_axis_x=1;
    for(int N=2; N<=16; N*=2){
      ActZoom(+1);
      float err=0; for(int i=0;i<N_SAMPLES;i++){ double u=120.0+(i-120.0)/N; float exp_v=(float)((SINP(start+u*step, step)-2048.0)*KV); float e=fabsf(mv_view_v[i]-exp_v); if(e>err)err=e; }
      snprintf(nm,sizeof nm,"FAST x%d: view = the true wave around the screen centre (max error %.4f V)",N,err); check(nm, mv_zoom_x==N && mv_view_on==1 && err<0.02f);
    }
    /* panned zoom: still the true wave at the shifted position */
    ActDown(3); { double p=mv_x_q/16.0; float err=0; for(int i=0;i<N_SAMPLES;i++){ double u=120.0+(i-120.0)/16.0+p; float exp_v=(float)((SINP(start+u*step, step)-2048.0)*KV); float e=fabsf(mv_view_v[i]-exp_v); if(e>err)err=e; }
      snprintf(nm,sizeof nm,"FAST x16 panned right by %.3f points: still the true wave (max error %.4f V)",p,err); check(nm, p>0.5 && err<0.02f); }
    /* pan range at zoom */
    zfresh(); fast_setup(step, pre_pts, start, 0.0); ActFreeze(); mv_axis_x=1;
    { int x1max=mv_x_max, x1min=mv_x_min;
      for(int i=0;i<300;i++) ActDown(1); snprintf(nm,sizeof nm,"x1 pan clamp is the v54 range: right %d = nhi %d",mv_x_pts,mv_nhi); check(nm, mv_x_pts==x1max && x1max==mv_nhi);
      ActZoom(+1); ActZoom(+1);   /* x4 */
      int qmax=x1max*16+(1904-1904/4), qmin=x1min*16-(1920-1920/4);
      for(int i=0;i<2000;i++) ActDown(1);
      snprintf(nm,sizeof nm,"x4 pan right clamp q=%d (expected %d = nhi*16 + 1428), pan range grew by %d points",mv_x_q,qmax,(qmax-x1max*16)/16); check(nm, mv_x_q==qmax && mv_x_pts==qmax/16);
      check("x4 right edge of the view stays inside the stored data", 1920+119*(16/4)+mv_x_q <= 16*(239+mv_nhi));
      for(int i=0;i<4000;i++) ActUp(1);
      snprintf(nm,sizeof nm,"x4 pan left clamp q=%d (expected %d)",mv_x_q,qmin); check(nm, mv_x_q==qmin);
      check("x4 left edge of the view stays inside the stored data", 1920-120*(16/4)+mv_x_q >= -16*mv_nlo);
      { int bad=0; for(int i=0;i<N_SAMPLES;i++){ float c=mv_view_v[i]/KV+2048.0f; if(c<1380.0f||c>2720.0f) bad++; } check("x4 at the left clamp: every pixel is real data (no garbage)", bad==0); }
      for(int i=0;i<2000;i++) ActDown(1); ActZoom(-1); ActZoom(-1);
      snprintf(nm,sizeof nm,"zoom back to x1 from the far right: pan re-clamped to the v54 range (%d, nhi %d)",mv_x_pts,mv_nhi); check(nm, mv_zoom_x==1 && mv_x_pts<=mv_nhi && mv_x_pts>=mv_nlo*-1);
      zfresh(); fast_setup(step, pre_pts, start, 0.0); ActFreeze(); mv_axis_x=1; ActZoom(+1); ActDown(1); int q1=mv_x_q; ActZoom(+1); ActDown(1); int q2=mv_x_q-q1;
      snprintf(nm,sizeof nm,"pan step shrinks with the zoom (x2: %d, then x4 step %d, in 1/16 point)",q1,q2); check(nm, q1==48 && q2==24+0*q2 && q2>0); }
    /* real raw detail: a period-4 ripple of +-300 counts that the w=8 boxcar removes completely at x1 */
    zfresh(); fast_setup(step, pre_pts, start, 300.0);
    { float d1=maxdiff(osc_v); ActFreeze(); mv_axis_x=1; ActZoom(+1); ActZoom(+1); ActZoom(+1); float d8=maxdiff(mv_view_v);
      snprintf(nm,sizeof nm,"FAST x8 shows raw detail: max step between pixels %.3f V at x8 against %.3f V at x1 (ripple averaged out)",d8,d1); check(nm, d1<0.05f && d8>0.35f); }
    /* beyond the raw resolution: linear interpolation (step 2 -> x8 puts 4 pixels between two raw samples) */
    zfresh(); { uint32_t n=(uint32_t)((N_SAMPLES+100)*2.0f)+4u; for(uint32_t j=0;j<n;j++) adc_raw_buf[j]=(uint16_t)((j%2)?2548:1548); /* alternating */
      for(int i=0;i<N_SAMPLES;i++) raw_samples[i]=2048; to_osc_v(); MvStoreExt(300, 2.0f, 2, n); }
    ActFreeze(); mv_axis_x=1; ActZoom(+1); ActZoom(+1); ActZoom(+1);
    { int mono=1, same=0; for(int i=0;i<N_SAMPLES-1;i++){ float d=mv_view_v[i+1]-mv_view_v[i]; if (fabsf(fabsf(d)-1000.0f*KV/4.0f)<0.004f) same++; }
      snprintf(nm,sizeof nm,"step 2 at x8: points joined by straight lines (%d of 239 steps equal 1/4 of the raw jump)",same); check(nm, mono && same>200); }
  }
  /* no raw copy at all (frozen before any capture): straight lines between the display points */
  zfresh(); for(int i=0;i<N_SAMPLES;i++) raw_samples[i]=(int16_t)((i%2)?2548:1548); to_osc_v(); mv_raw_n=0; ActFreeze(); mv_axis_x=1; ActZoom(+1); ActZoom(+1);
  { int ok=1; for(int i=0;i<N_SAMPLES-1;i++){ float d=fabsf(mv_view_v[i+1]-mv_view_v[i]); if(d>1000.0f*KV/4.0f*1.01f) ok=0; }
    check("no stored raw samples: x4 joins the display points by straight lines (no invented detail)", ok && mv_view_on==1); }

  /* ---- 7. SLOW time zoom (frozen 1 ms history) ---- */
  slow_setup(0.0, 8333u); zfresh(); slow_active=1; mv_slow_mode=0;
  slow_setup(0.0, 8333u); slow_freeze(); mv_axis_x=1;
  snprintf(nm,sizeof nm,"SLOW freeze: history copy taken, pan range left %d points",mv_x_min); check(nm, mv_slow_mode==1 && mv_x_min<-700 && mv_x_max==0);
  for(int N=2; N<=16; N*=2){
    ActZoom(+1);
    float err=0; double tend=20001.0;
    for(int i=0;i<N_SAMPLES;i++){ double u=120.0+(i-120.0)/N; double tpos=tend-1.0-(239.0-u)*8.333; double e_v=800.0*sin(6.283185307179586*tpos/2000.0); float exp_v=(float)(e_v*KV); float e=fabsf(mv_view_v[i]-exp_v); if(e>err)err=e; }
    snprintf(nm,sizeof nm,"SLOW x%d: view follows the frozen history around the centre (max error %.4f V)",N,err); check(nm, mv_zoom_x==N && mv_view_on==1 && err<0.05f);
  }
  { int qmin=mv_x_min*16-(1920-1920/16), qmax=mv_x_max*16+(1904-1904/16);
    for(int i=0;i<100000;i++) ActDown(1); snprintf(nm,sizeof nm,"SLOW x16 pan right clamp %d (expected %d: newest data at the right edge)",mv_x_q,qmax); check(nm, mv_x_q==qmax);
    for(int i=0;i<100000;i++) ActUp(1);   snprintf(nm,sizeof nm,"SLOW x16 pan left clamp %d (expected %d)",mv_x_q,qmin); check(nm, mv_x_q==qmin); }
  /* 1 ms ripple: averaged out at x1 (200 ms/div: 8 ticks per point), real at x16 (0.5 ms per pixel) */
  slow_setup(250.0, 8333u); zfresh(); slow_active=1; slow_setup(250.0, 8333u); slow_freeze(); mv_axis_x=1;
  { float d1=maxdiff(osc_v); for(int i=0;i<4;i++) ActZoom(+1); float d16=maxdiff(mv_view_v);
    snprintf(nm,sizeof nm,"SLOW x16 shows 1 ms detail: max step %.3f V at x16 against %.3f V at x1",d16,d1); check(nm, d16>0.1f && d1<0.08f); }
  /* no history (fewer than 64 ticks): stretched points, never garbage */
  zfresh(); slow_active=1; slow_valid_cnt=10; for(int i=0;i<N_SAMPLES;i++) raw_samples[i]=(int16_t)(2048+i*5); to_osc_v(); ActFreeze(); mv_axis_x=1; ActZoom(+1); ActZoom(+1);
  check("SLOW without history: x4 joins the display points (no garbage), pan range stays tight", mv_slow_mode==0 && fabsf(mv_view_v[120]-osc_v[120])<0.01f && mv_x_min==0);

  /* ---- 8. numbers untouched ---- */
  zfresh(); ActFreeze(); osc_meas.vpp=2.5f; osc_meas.freq=1234.0f; mv_axis_x=1; ActZoom(+1); mv_axis_x=0; ActZoom(+1); ActUp(2);
  check("Vpp / F untouched by zoom and move", osc_meas.vpp==2.5f && osc_meas.freq==1234.0f);

  printf("zoom model failures=%d\n",failures);
  return failures?1:0;
}
'''
harness_py = pre + main_c + tail
harness_py = harness_py.replace("OUT = Path(tempfile.gettempdir()) / 'osc_buttons'", "OUT = Path(tempfile.gettempdir()) / 'osc_zoom_build'")
k0 = harness_py.index("# ---- contracts on the text of main.c")
k1 = harness_py.index("harness = r'''")
harness_py = harness_py[:k0] + harness_py[k1:]
harness_py += "r1 = subprocess.run([str(OUT/'buttons')], text=True, capture_output=True); print(r1.stdout, end='')\nMODEL_RC = r1.returncode\n"
ns = {'__file__': str(ROOT/'tests/test_zoom.py'), '__name__': 'zoom_inner'}
exec(compile(harness_py, 'zoom_harness', 'exec'), ns)

# ---- real UartPoll: '+' / '-' are immediate keys, "v +" / "t -" stay line commands --------------------------------------
src = (ROOT/'Core/Src/main.c').read_text()
nocom = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
def func(t, name):
    m = re.search(r'^static\s+\w+\s+' + name + r'\([^;]*?\)\s*\{', t, re.M); assert m, name
    p = t.index('{', m.start()); d = 1; i = p + 1
    while d: d += (t[i] == '{') - (t[i] == '}'); i += 1
    return t[m.start():i]
OUT = ns['OUT']
(OUT/'poll.c').write_text(r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#define UART_RX_SIZE 96
#define UART_IDLE_MS 30
#define UART_DROP_MS 500
static uint8_t uart_rx[UART_RX_SIZE]; static volatile uint16_t uart_head, uart_tail; static char uart_line[24]; static uint8_t uart_len; static uint32_t uart_last_tick, now;
static uint32_t HAL_GetTick(void){ return now; }
static char got[200];
static void UartCommand(char*l){ strcat(got,"["); strcat(got,l); strcat(got,"]"); }
''' + func(nocom, 'UartPoll') + r'''
static void feed(const char*s){ got[0]=0; for(;*s;s++){ uart_rx[uart_head]=(uint8_t)*s; uart_head=(uint16_t)((uart_head+1)%UART_RX_SIZE); } now+=1000; UartPoll(); }
int main(void){ int f=0;
  feed("+"); if(strcmp(got,"[+]")){ printf("FAIL '+' immediate: %s\n",got); f++; } else printf("PASS '+' is an immediate key\n");
  feed("-"); if(strcmp(got,"[-]")){ printf("FAIL '-' immediate: %s\n",got); f++; } else printf("PASS '-' is an immediate key\n");
  feed("++-"); if(strcmp(got,"[+][+][-]")){ printf("FAIL '++-': %s\n",got); f++; } else printf("PASS three zoom keys in a row\n");
  feed("v +\r"); if(strcmp(got,"[v +]")){ printf("FAIL 'v +': %s\n",got); f++; } else printf("PASS 'v +' is still ONE line command (v, arg +)\n");
  feed("t -\r"); if(strcmp(got,"[t -]")){ printf("FAIL 't -': %s\n",got); f++; } else printf("PASS 't -' is still ONE line command\n");
  feed("f 100\r"); if(strcmp(got,"[f 100]")){ printf("FAIL 'f 100': %s\n",got); f++; } else printf("PASS 'f 100' unchanged\n");
  feed("h"); if(strcmp(got,"[h]")){ printf("FAIL 'h': %s\n",got); f++; } else printf("PASS 'h' still immediate\n");
  printf("uart poll failures=%d\n",f); return f?1:0; }
''')
r2 = subprocess.run(['clang', '-O0', str(OUT/'poll.c'), '-o', str(OUT/'poll')], text=True, capture_output=True)
if r2.returncode: print(r2.stderr)
r3 = subprocess.run([str(OUT/'poll')], text=True, capture_output=True); print(r3.stdout, end='')

# ---- contracts on the text of main.c ----------------------------------------------------------------------------------------
before = ROOT.parent/'builds/v57/main_before.c'
bt = before.read_text() if before.exists() else None
def slow_text(t): return t[t.index('SLOW_BEGIN ='):t.index('SLOW_END =')]
def block(t, b, e):
    a = t.index(b); a = t.rfind('/*', 0, a); z = t.index('\n', t.index(e)); return t[a:z+1]
def cut_func(t, name):
    m = re.search(r'^static [^\n;]*\b' + name + r'\([^;{]*\)[^;{]*\{', t, re.M); a = m.start(); p = t.index('{', a); d = 1; q = p + 1
    while d: d += (t[q] == '{') - (t[q] == '}'); q += 1
    return t[:a] + t[q:]
def strip_allowed(t):
    for b, e in [('MOVE_BEGIN =', 'MOVE_END ='), ('MOVE_DATA_BEGIN =', 'MOVE_DATA_END =')]:
        t = t.replace(block(t, b, e), '')
    a = t.index('/* --- v54: MOVE (only acts while frozen)'); z = t.index('static float    mv_view_v[N_SAMPLES];\n') + len('static float    mv_view_v[N_SAMPLES];\n')
    t = t[:a] + t[z:]
    for name in ['MapToScreen', 'DrawTopPanel', 'UartPanel', 'UartCommand', 'UartPoll']:
        t = cut_func(t, name)
    return t
contracts = [('baseline exists: builds/v57/main_before.c', bt is not None)]
if bt is not None:
    contracts.append(('SLOW block (SLOW_BEGIN = .. SLOW_END =) byte-identical to main_before.c', slow_text(src) == slow_text(bt)))
    rest_a, rest_b = strip_allowed(bt), strip_allowed(src)
    outside = [l for l in __import__('difflib').unified_diff(rest_a.splitlines(), rest_b.splitlines(), lineterm='', n=0) if l[:1] in '+-' and l[:3] not in ('+++', '---')]
    contracts.append(('nothing outside move/zoom/draw/uart code changed (MOVE + MOVE_DATA blocks, mv_ state, MapToScreen, DrawTopPanel, UartPanel/Command/Poll excluded): %d changed lines (budget 6)' % len(outside), len(outside) <= 6))
    for l in outside[:10]: print('   outside:', l)
    for name in ['OscMeasureFrame', 'OscProcessFrame', 'AdcCaptureFrame', 'OscAutoTimebase', 'OscAutoScale', 'SlowCaptureFrame', 'OscAdjust', 'OscFrameHold', 'DrawWaveform', 'main']:
        def fn(t):
            m = re.search(r'^(?:static )?[^\n;]*\b' + name + r'\([^;{]*\)[^;{]*\{', t, re.M); a = m.start(); p = t.index('{', a); d = 1; q = p + 1
            while d: d += (t[q] == '{') - (t[q] == '}'); q += 1
            return t[a:q]
        contracts.append(('function unchanged vs v56: ' + name, fn(src) == fn(bt)))
    diff = [l for l in __import__('difflib').unified_diff(bt.splitlines(), src.splitlines(), lineterm='', n=0) if l[:1] in '+-' and l[:3] not in ('+++', '---')]
    contracts.append(('whole-file diff budget (<= 260 changed lines): %d' % len(diff), len(diff) <= 260))
    # header: zoomed text only while frozen
    contracts.append(('header uses the effective T/V only when frozen and zoomed', 'if (hold_active && mv_zoom_y > 1) MvFmtV(' in src and 'if (hold_active && mv_zoom_x > 1) MvFmtT(' in src))
    contracts.append(("UART: '+' / '-' map to ActZoom, strchr key list of v56 kept", "case '+': ActZoom(+1);" in src and "case '-': ActZoom(-1);" in src and 'strchr("sdhg?zxFA0", c)' in src))
    contracts.append(('MapToScreen at zoom 1 is the v56 expression (GRID_Y_STEP * 1)', '(float)(GRID_Y_STEP * mv_zoom_y)' in src))
    contracts.append(('MvStoreExt not called by anything new (still the single v54 call in AdcCaptureFrame)', src.count('MvStoreExt(start, step, w, n);') == 1))
cfail = sum(not ok for _, ok in contracts)
for n, ok in contracts: print(('PASS ' if ok else 'FAIL ') + n)
print('contract failures=%d' % cfail)
fails = cfail + (1 if ns.get('MODEL_RC') else 0) + (1 if r3.returncode else 0)
print('zoom failures=%d' % fails)
raise SystemExit(bool(fails))
