"""Edge behaviour of the display moving-average filter (OscProcessFrame).
Bug history: the old clamp-to-edge padding left the first/last points almost unfiltered, so whenever the filter
smooths the middle (2 MHz = 2.4 points/cycle, 500 kHz = 9.6 points/cycle at 5 us/div) both screen edges stood out.
v74 fixed 2.4 pts/cycle but missed ~9.6 (edges 1.8x on the board) -> v76:
  FAST: real display points just outside the window (OscExtPoint) -> edge = exact centred filter;
  no outside data (SLOW): fallback rule, edges never noticeably taller than the middle."""
import pathlib, re, subprocess, sys, tempfile
ROOT = pathlib.Path(__file__).resolve().parents[1]
src = (ROOT/'Core/Src/main.c').read_text()
m = re.search(r'^static void OscProcessFrame\(void\)\n\{', src, re.M)
assert m, 'OscProcessFrame not found'
p = src.index('{', m.start()); d = 1; i = p+1
while d: d += (src[i] == '{') - (src[i] == '}'); i += 1
fn = src[m.start():i]
m2 = re.search(r'^static int OscFiltWin\(void\)\n\{', src, re.M)   # v82: the window OscProcessFrame uses
assert m2, 'OscFiltWin not found'
p = src.index('{', m2.start()); d = 1; i = p+1
while d: d += (src[i] == '{') - (src[i] == '}'); i += 1
fn = src[m2.start():i] + '\n' + fn
OUT = pathlib.Path(tempfile.gettempdir())/'osc_edge'; OUT.mkdir(parents=True, exist_ok=True)
harness = r'''
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#define ADC_INPUT_ENABLE 1
#define N_SAMPLES 240
#define ADC_VREF 3.3f
#define ADC_INPUT_GAIN 1.25f  /* v88: the board value, so a path that forgets the gain (v84 edge points) shows up */
#define ADC_FULL 4095
#define ADC_MID 2048
#define ADC_DC_ZERO_CT 0.0f   /* v89: DC mode zero */
static int16_t raw_samples[N_SAMPLES]; static float osc_v[N_SAMPLES];
static int osc_fwin = 5, osc_calib = 1; static float osc_offset_ct, osc_offset_v;
static float osc_raw_freq = 0.0f, osc_fs_eff = 0.0f;                  /* v82: 0 = unknown frequency -> OscFiltWin keeps osc_fwin */
static uint8_t osc_raw_valid = 0, osc_frame_valid = 1; static float osc_raw_mean_ct;
static double g_ppc, g_ph, g_amp; static int g_ext = 0;               /* g_ext: FAST frame with real outside points */
static double sig(double x){ return 2048 + g_amp*sin(2*M_PI*x/g_ppc + g_ph); }
static int OscExtPoint(int k, float *v){ if (!g_ext || k < -256 || k >= N_SAMPLES + 256) return 0; *v = (float)lround(sig(k)); return 1; }
''' + fn + r'''
static int fails;
static void check(const char *n, int ok){ printf("%s %s\n", ok ? "PASS" : "FAIL", n); if (!ok) fails++; }
static void fill(double ppc, double ph, double amp){ g_ppc=ppc; g_ph=ph; g_amp=amp; for (int i=0;i<N_SAMPLES;i++) raw_samples[i]=(int16_t)lround(sig(i)); }
static double edge_ratio(void){ double mean=0; for(int i=12;i<228;i++) mean+=osc_v[i]; mean/=216;
  double mid=0, edge=0; for(int i=12;i<228;i++) if (fabs(osc_v[i]-mean)>mid) mid=fabs(osc_v[i]-mean);
  for(int i=0;i<3;i++){ if(fabs(osc_v[i]-mean)>edge) edge=fabs(osc_v[i]-mean); if(fabs(osc_v[239-i]-mean)>edge) edge=fabs(osc_v[239-i]-mean); }
  return edge / (mid > 1e-3 ? mid : 1e-3); }
int main(void){
  char nm[200]; int wins[] = {5, 13, 21};
  double ppcs[] = {2.4, 3, 4, 5, 6, 7, 8, 9, 9.6, 10, 11, 12, 14, 16, 20, 25, 30, 40, 60, 120};
  int NP = (int)(sizeof ppcs / sizeof ppcs[0]);
  for (int w=0; w<3; w++){ osc_fwin = wins[w]; int h = wins[w]/2;
    /* FAST: real outside points -> every point (edges included) equals the ideal centred filter */
    double worst_err = 0; g_ext = 1;
    for (int q=0;q<NP;q++) for (int k=0;k<12;k++){ fill(ppcs[q], k*0.5, 1000); OscProcessFrame();
      for (int i=0;i<N_SAMPLES;i++){ double acc=0; for(int j=-h;j<=h;j++) acc+=lround(sig(i+j));
        double ideal=(acc/wins[w]-osc_offset_ct)*3.3/4095*ADC_INPUT_GAIN; if (fabs(osc_v[i]-ideal)>worst_err) worst_err=fabs(osc_v[i]-ideal); } }
    snprintf(nm,sizeof nm,"FAST with real outside points, window %d, 2.4..120 pts/cycle: edges = ideal filter (max error %.4f V, limit 0.002)", wins[w], worst_err);
    check(nm, worst_err < 0.002);
    /* SLOW / no outside data: fallback, no spike anywhere incl. 9.6 pts/cycle (the v74 miss) */
    double worst = 0, wppc = 0; g_ext = 0;
    for (int q=0;q<NP;q++) for (int k=0;k<12;k++){ fill(ppcs[q], k*0.5, 1000); OscProcessFrame(); double r=edge_ratio(); if (r>worst){ worst=r; wppc=ppcs[q]; } }
    snprintf(nm,sizeof nm,"no outside data, window %d, 2.4..120 pts/cycle: edges never taller than the middle (worst %.2f at %.1f pts/cycle, limit 1.15)", wins[w], worst, wppc);
    check(nm, worst <= 1.15);
    /* slow signal without outside data: edges follow the wave */
    double err = 0; fill(120.0, 0.7, 1200); OscProcessFrame();
    for (int i=0;i<N_SAMPLES;i++){ double t = 1200*sin(2*M_PI*i/120.0+0.7)*3.3/4095*ADC_INPUT_GAIN; if (fabs(osc_v[i]-t) > err) err = fabs(osc_v[i]-t); }
    snprintf(nm,sizeof nm,"no outside data, slow sine 120 pts/cycle, window %d: max error %.3f V incl. edges (limit 0.12 V of 1.93 Vpp)", wins[w], err);
    check(nm, err < 0.12);
  }
  /* v82: a known frequency narrows the window so the drawn wave keeps >= 98 % of its height (it must match V/DIV on the grid) */
  { double ppcs2[] = {16, 20, 23.6, 30, 40, 48, 60, 120}; int wins2[] = {5, 13, 21}; double worst = 1e9; char wn[120] = "";
    g_ext = 1; osc_fs_eff = 48000.0f;
    for (int w=0; w<3; w++) for (int q=0; q<8; q++) for (int k=0; k<6; k++) {
      osc_fwin = wins2[w]; osc_raw_freq = (float)(48000.0 / ppcs2[q]); fill(ppcs2[q], k*0.5, 1000); OscProcessFrame();
      double lo=1e9, hi=-1e9, rlo=1e9, rhi=-1e9; for (int i=0;i<N_SAMPLES;i++){ if(osc_v[i]<lo)lo=osc_v[i]; if(osc_v[i]>hi)hi=osc_v[i]; double r=(raw_samples[i]-osc_offset_ct)*3.3/4095*ADC_INPUT_GAIN; if(r<rlo)rlo=r; if(r>rhi)rhi=r; }
      double ratio = (hi-lo)/(rhi-rlo); if (ratio < worst){ worst = ratio; snprintf(wn, sizeof wn, "window %d at %.1f pts/cycle", wins2[w], ppcs2[q]); } }
    snprintf(nm,sizeof nm,"known frequency: drawn height >= 0.97 of the raw points (worst %.3f, %s)", worst, wn); check(nm, worst >= 0.97);
    osc_fwin = 5; osc_raw_freq = 400.0f; check("known frequency, 120 pts/cycle: FILTER 5 is kept", OscFiltWin() == 5);
    osc_fwin = 5; osc_raw_freq = (float)(48000.0/23.6); check("known frequency, 23.6 pts/cycle: FILTER 5 is narrowed", OscFiltWin() < 5);
    osc_raw_freq = 0.0f; osc_fs_eff = 0.0f; g_ext = 0; }
  osc_fwin = 1; g_ext = 0; fill(2.4, 0.3, 300); OscProcessFrame();
  check("window 1: raw samples unchanged", fabs(osc_v[0] - (raw_samples[0]-osc_offset_ct)*3.3/4095*ADC_INPUT_GAIN) < 1e-6);
  /* v89: DC coupling (PE0 = 0): no offset removed, 0 counts = 0 V, a 0..1900-count sine never goes negative */
  osc_calib = 0; osc_fwin = 1; g_ext = 0; g_ppc = 40; g_ph = 0; g_amp = 950;
  for (int i=0;i<N_SAMPLES;i++) raw_samples[i]=(int16_t)lround(950 + 950*sin(2*M_PI*i/40.0));
  OscProcessFrame(); { double lo=1e9, hi=-1e9; for (int i=0;i<N_SAMPLES;i++){ if(osc_v[i]<lo)lo=osc_v[i]; if(osc_v[i]>hi)hi=osc_v[i]; }
    snprintf(nm,sizeof nm,"DC mode: trace 0..%.3f V, min %.4f V (>= 0), offset_v %.3f (0)", hi, lo, osc_offset_v);
    check(nm, lo >= -1e-6 && fabs(hi - 1900*3.3/4095*ADC_INPUT_GAIN) < 0.01 && osc_offset_v == 0.0f); }
  osc_calib = 1;
  printf("edge_filter failures=%d\n", fails); return fails ? 1 : 0;
}'''
(OUT/'edge.c').write_text(harness)
subprocess.run(['cc', '-O1', '-o', str(OUT/'edge'), str(OUT/'edge.c'), '-lm'], check=True)
sys.exit(subprocess.run([str(OUT/'edge')]).returncode)
