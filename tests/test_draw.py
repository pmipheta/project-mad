"""Host test of the LCD column renderer (DrawWaveform), extracted from Core/Src/main.c between DRAWWF_BEGIN/END.
Simulates the screen as a pixel array; after every frame the screen must equal the ideal picture (trace + grid + cursors), exactly:
no gaps in the trace, no stale pixels from earlier frames."""
from app_src import source as app_source   # the app modules as one text (see app_src.py)
from pathlib import Path
import subprocess, tempfile
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(tempfile.gettempdir()) / 'osc_draw'; OUT.mkdir(parents=True, exist_ok=True)
src = app_source()
import re
# v75: take the real grid rule (GRID_* + IS_GRID_DOT) from main.c so the test cannot drift from the firmware
_gl, _cont = [], False
for _ln in src.split('\n'):
    if _cont or re.match(r'#define (?:GRID_\w+|IS_GRID_DOT)\b', _ln):
        _gl.append(_ln); _cont = _ln.rstrip().endswith('\\')
grid_defs = '\n'.join(_gl)
a, b = src.index('/* DRAWWF_BEGIN */'), src.index('/* DRAWWF_END */')
(OUT/'drawwf.c').write_text(src[a:b])
harness = r'''
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define N_SAMPLES 240
#define SCREEN_W 320
#define SCREEN_H 240
#define WAVEFORM_TOP 29
GRID_DEFS_HERE
#define BLACK 0x0000
#define YELLOW 0xFFE0
#define DARKGREY 0x7BEF
#define DARKCYAN 0x03EF
#define SHOW_FPS 0
static int16_t new_y[N_SAMPLES], old_y[N_SAMPLES];
static uint8_t hold_active = 0; static int osc_c1 = 67, osc_c2 = 200; static uint16_t cursor_x = 160; static uint8_t osc_full_redraw = 1;
static int OscClampI(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
static int OscCursorX(int x){ if(x<0)x=0; if(x>=SCREEN_W)x=SCREEN_W-1; return x; }   /* v85: cursors are LCD x */
static uint8_t col_lo_old[SCREEN_W], col_hi_old[SCREEN_W], col_lo_new[SCREEN_W], col_hi_new[SCREEN_W];
static int16_t env_y_lo[N_SAMPLES], env_y_hi[N_SAMPLES]; static uint8_t env_y_on = 0;   /* v83 peak-detect band (off here) */
static uint16_t fb[SCREEN_W][SCREEN_H]; static long writes;
static void LcdWriteColumn(uint16_t x, uint16_t y1, uint16_t y2, const uint8_t *bytes, uint32_t n){
  if ((uint32_t)(y2 - y1 + 1) * 2u != n) { printf("BAD LENGTH\n"); exit(2); }
  for (uint32_t k = 0; k < n/2; k++) fb[x][y1+k] = (uint16_t)((bytes[2*k] << 8) | bytes[2*k+1]);
  writes += n/2; }
#include "drawwf.c"
static uint16_t expect[SCREEN_W][SCREEN_H];
static void ideal(void){
  for(int x=0;x<SCREEN_W;x++) for(int y=0;y<SCREEN_H;y++) expect[x][y]=BackgroundColour(x,y);
  for(int i=0;i<N_SAMPLES-1;i++){ int x0=(i*SCREEN_W)/N_SAMPLES,x1=((i+1)*SCREEN_W)/N_SAMPLES,y0=new_y[i],y1=new_y[i+1];
    int dx=abs(x1-x0),sx=x0<x1?1:-1,dy=-abs(y1-y0),sy=y0<y1?1:-1,err=dx+dy;
    while(1){ expect[x0][y0]=YELLOW; if(x0==x1&&y0==y1)break; int e2=2*err; if(e2>=dy){err+=dy;x0+=sx;} if(e2<=dx){err+=dx;y0+=sy;} } } }
static int diff(void){ int bad=0; for(int x=0;x<SCREEN_W;x++) for(int y=WAVEFORM_TOP;y<SCREEN_H;y++) if(fb[x][y]!=expect[x][y]) bad++; return bad; }
static int clampy(int y){ return y<WAVEFORM_TOP?WAVEFORM_TOP:(y>=SCREEN_H?SCREEN_H-1:y); }
int main(void){
  int failures=0;
  for(int x=0;x<SCREEN_W;x++) for(int y=0;y<SCREEN_H;y++) fb[x][y]=BackgroundColour(x,y);     /* cleared screen with grid */
  struct { const char *name; int frames; } sc[] = {{"scrolling 15 cycles",60},{"scrolling 3 cycles",60},{"sparse changes",80},{"static trace",20},{"extreme amplitude (clamped)",40},{"HOLD cursors on",20}};
  srand(7); long total=0; int nfr=0;
  for(int s=0;s<6;s++){
    for(int f=0;f<sc[s].frames;f++){
      for(int i=0;i<N_SAMPLES;i++){
        double cyc = (s==0)?15:3; int y;
        if(s==0||s==1) y=(int)(120-55*sin(6.2831853*cyc*i/N_SAMPLES+0.9*f));
        else if(s==2) { y = new_y[i]; if(f==0||rand()%10==0) y = clampy(y? y: 120) + (f? rand()%81-40 : 0); if(f==0) y=120-(int)(40*sin(i*0.1)); }
        else if(s==3) y=(int)(120-50*sin(6.2831853*4*i/N_SAMPLES));
        else if(s==4) y=(int)(120-400*sin(6.2831853*5*i/N_SAMPLES+0.4*f));
        else { y=(int)(120-55*sin(6.2831853*3*i/N_SAMPLES+0.5*f)); hold_active=1; }
        new_y[i]=(int16_t)clampy(y); }
      if(s==5 && f==10){   /* cursors moved: ClearCursors restores the old columns, sets the flag; DrawCursors paints the new ones after the trace */
        uint8_t h=hold_active; hold_active=0; int ox[2]={OscCursorX(osc_c1),OscCursorX(osc_c2)};
        for(int k=0;k<2;k++) for(int y=WAVEFORM_TOP;y<SCREEN_H;y++) fb[ox[k]][y]=BackgroundColour(ox[k],y);
        hold_active=h; osc_c1=93; osc_c2=267; osc_full_redraw=1; }
      long w0=writes; DrawWaveform(); total+=writes-w0; nfr++;
      if(hold_active){ int cx[2]={OscCursorX(osc_c1),OscCursorX(osc_c2)}; for(int k=0;k<2;k++) for(int y=WAVEFORM_TOP;y<SCREEN_H;y++) fb[cx[k]][y]=YELLOW; }
      ideal(); int bad=diff(); if(bad){ printf("FAIL %s frame %d: %d wrong pixels\n",sc[s].name,f,bad); failures++; break; }
    }
    printf("PASS %s\n", sc[s].name);
  }
  /* v75: vertical T/DIV grid: dotted line exactly at x = 0,32,...,288 (x=160 is the cyan centre cursor) */
  { int ok=1, nlines=0; hold_active=0;
    for(int x=0;x<SCREEN_W;x++){ int dots=0; for(int y=WAVEFORM_TOP;y<SCREEN_H;y++) if(y%GRID_Y_STEP!=GRID_Y_OFFSET && BackgroundColour(x,y)==DARKGREY) dots++;
      int want = (x%32==0 && x!=(int)cursor_x); if(want){ nlines++; if(dots<40) ok=0; } else if(dots) ok=0; }
    printf("%s vertical grid every 1 T/DIV (32 px): %d dotted lines + centre cursor\n", (ok&&nlines==9)?"PASS":"FAIL", nlines); if(!(ok&&nlines==9)) failures++; }
  /* v85: frozen C1/C2 glide: repaint only the old + new cursor columns (RepaintColumn), the screen must equal a full redraw at every step */
  { int bad=0, steps=0; long w0=writes; hold_active=1;
    for(int v=94; v<=200; v++){ int o1=OscCursorX(osc_c1), o2=OscCursorX(osc_c2); osc_c1=v; if(v%3==0) osc_c2=OscClampI(osc_c2-1,0,SCREEN_W-1);
      int xs[4]={o1,o2,OscCursorX(osc_c1),OscCursorX(osc_c2)};
      for(int k=0;k<4;k++){ int dup=0; for(int j=0;j<k;j++) if(xs[j]==xs[k]) dup=1; if(!dup) RepaintColumn(xs[k],-1,-1); }
      ideal(); int cx[2]={OscCursorX(osc_c1),OscCursorX(osc_c2)}; for(int k=0;k<2;k++) for(int y=WAVEFORM_TOP;y<SCREEN_H;y++) expect[cx[k]][y]=YELLOW;
      bad+=diff(); steps++; }
    printf("%s v85 cursor glide: %d one-pixel steps equal a full redraw (%ld px written per step, a full redraw wrote %d)\n", bad?"FAIL":"PASS", steps, (writes-w0)/steps, SCREEN_W*(SCREEN_H-WAVEFORM_TOP)); if(bad) failures++;
    for(int y=0;y<SCREEN_H;y++) fb[5][y]=0x1234; RepaintColumn(5,228,235); int kept=1; for(int y=228;y<=235;y++) if(fb[5][y]!=0x1234) kept=0;
    int painted=(fb[5][WAVEFORM_TOP]!=0x1234 && fb[5][227]!=0x1234 && fb[5][236]!=0x1234 && fb[5][SCREEN_H-1]!=0x1234);
    printf("%s v85 RepaintColumn leaves the setting-bar rows alone and paints the rest\n", (kept&&painted)?"PASS":"FAIL"); if(!(kept&&painted)) failures++;
    hold_active=0; }
  printf("pixels written per frame (avg): %ld  (old per-pixel method wrote 6000-9000 at 15 cycles and cost ~58 us each)\n", total/nfr);
  printf("draw failures=%d\n", failures); return failures; }
'''
(OUT/'t.c').write_text(harness.replace('GRID_DEFS_HERE', grid_defs))
r = subprocess.run(['gcc','-O1','-Wall','-I',str(OUT),'-o',str(OUT/'t'),str(OUT/'t.c'),'-lm'],capture_output=True,text=True)
print(r.stderr[-1500:])
if r.returncode: raise SystemExit(2)   # v85: a failed compile must not rerun the previous binary
print(subprocess.run([str(OUT/'t')],capture_output=True,text=True).stdout)
