"""v60 SET-selection blink: contracts on DrawTopPanel + C model of HdrText/colour expressions extracted from main.c. Host test, not a board test."""
from app_src import source as app_source   # the app modules as one text (see app_src.py)
from pathlib import Path
import re, subprocess, tempfile
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(tempfile.gettempdir())/'osc_blink'; OUT.mkdir(parents=True, exist_ok=True)
src = app_source()
fails = 0
def chk(c, m):
    global fails
    print(('ok   ' if c else 'FAIL ')+m); fails += (not c)
a = src.index('static void DrawTopPanel(void)\n{'); b = src.index('\n}\n', a)
dtp = src[a:b]
period = re.search(r'#define BLINK_PERIOD_MS\s+(\d+)u?', src)[1]
blink_line = re.search(r'uint8_t blink_on = [^\n]+;', dtp)[0]
vd = re.search(r'HDRT\(buf, 2, 1, (.+?), 1, BLACK\);', dtp)[1]
td = re.search(r'HDRT\(buf, 72, 1, (.+?), 1, BLACK\);', dtp)[1]
sel = re.findall(r'(if \(blink_on\)|else\s)\s*HDRT\(buf, 150, 1, (\w+), 1, (\w+)\);', dtp)
chk(len(sel) == 2, 'x=150 field has exactly two HDRT calls (ON/OFF)')
# every other HDRT call must be unchanged vs v59 (colours untouched)
_old = Path(ROOT.parent/'builds/v60/main_before.c')   # local build archive, not in git
if _old.exists():
    old = _old.read_text()
    oa = old.index('static void DrawTopPanel(void)\n{'); odt = old[oa:old.index('\n}\n', oa)]
    norm = lambda t: [l.strip() for l in t.splitlines() if 'HDRT(' in l and not re.search(r'HDRT\(buf, (2, 1|72, 1|150, 1),', l)]
    chk(norm(dtp) == norm(odt) and len(norm(dtp)) > 10, 'all other HDRT calls (Vp:/F:/Rs/Mx/Mn/status/HOLD...) byte-identical to v59: %d lines' % len(norm(dtp)))
else:
    print('SKIP HDRT-unchanged-vs-v59 check (builds/ archive not present in this checkout)')
c = r'''
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#define BLACK 0u
#define ORANGE 1u
#define MAGENTA 2u
#define YELLOW 3u
#define WHITE 4u
#define BLINK_PERIOD_MS %s
static int nchar;
static void ILI9341_Draw_Char(char c,uint8_t x,uint8_t y,uint16_t f,uint16_t s,uint16_t b){(void)c;(void)x;(void)y;(void)f;(void)s;(void)b;nchar++;}
static void ILI9341_Draw_Text(const char*t,uint8_t x,uint8_t y,uint16_t f,uint16_t s,uint16_t b){(void)t;(void)x;(void)y;(void)f;(void)s;(void)b;}
#define HDR_FIELDS 32
static char hdr_txt[HDR_FIELDS][24]; static uint16_t hdr_fg[HDR_FIELDS], hdr_bg[HDR_FIELDS]; static uint8_t hdr_ok[HDR_FIELDS];
#define HDR_CHAR_W 6
%s
static uint32_t tick; static uint32_t HAL_GetTick(void){return tick;}
int main(void){
  int fails=0;
  /* (a) blink_on vs tick */
  for(uint32_t t=0;t<5000;t+=7){ tick=t; %s
    if(blink_on != ((t/BLINK_PERIOD_MS)&1u)){fails++;} 
    int ph=(t/BLINK_PERIOD_MS)&1; if(blink_on!=ph) fails++; }
  tick=0; { %s if(blink_on!=0) fails++; } tick=BLINK_PERIOD_MS-1; { %s if(blink_on!=0) fails++; }
  tick=BLINK_PERIOD_MS; { %s if(blink_on!=1) fails++; } tick=2*BLINK_PERIOD_MS; { %s if(blink_on!=0) fails++; }
  printf("a_fails=%%d\n",fails); int f2=0;
  for(int blink=0;blink<2;blink++){ uint8_t blink_on=(uint8_t)blink;
    for(int s=0;s<2;s++){ int sel_vdiv=s, sel_tdiv=s;
      unsigned v=(%s), t=(%s);
      printf("v blink=%%d sel=%%d -> %%u\n",blink,s,v); printf("t blink=%%d sel=%%d -> %%u\n",blink,s,t);
      if(s==0 && (v!=YELLOW||t!=WHITE)) f2++;                 /* (c) unselected constant */
      if(s==1 && (v!=(blink?MAGENTA:YELLOW)||t!=(blink?MAGENTA:WHITE))) f2++; /* (b) selected alternates */
    }}
  printf("bc_fails=%%d\n",f2);
  /* (d) x=150 combos */
  int f3=0; unsigned combo[2][2]={{ORANGE,BLACK},{BLACK,ORANGE}};
  %s
  printf("d_fails=%%d\n",f3);
  /* no-regression: identical text + identical colours => zero redraw; text change => only changed chars */
  hdr_ok[5]=0; nchar=0; HdrText(5,"Vp:1.00V",2,10,3,1,0); int first=nchar;
  nchar=0; HdrText(5,"Vp:1.00V",2,10,3,1,0); int same=nchar;
  nchar=0; HdrText(5,"Vp:1.01V",2,10,3,1,0); int one=nchar;
  nchar=0; HdrText(5,"Vp:1.01V",2,10,4,1,0); int recol=nchar;
  printf("first=%%d same=%%d one=%%d recolor=%%d\n",first,same,one,recol);
  if(first!=8||same!=0||one!=1||recol!=8) f3++;
  /* blink of field A must not touch field B */
  HdrText(6,"F:1.0kHz ",72,10,5,1,0); nchar=0;
  for(int i=0;i<10;i++){ HdrText(7,"SEL",150,1,i&1,1,(i&1)?0:1); HdrText(6,"F:1.0kHz ",72,10,5,1,0);} 
  printf("f_redraws_during_blink=%%d\n",nchar-30); if(nchar-30!=0) f3++;
  printf("TOTAL_FAILS=%%d\n",fails+f2+f3); return fails+f2+f3; }
''' 
d_code = '''for(int k=0;k<2;k++){ unsigned fg=%s,bg=%s; /*placeholder*/ (void)fg;(void)bg; }'''
sel_c = []
for kw, fg, bg in sel:
    sel_c.append((fg, bg))
d = 'for(int i=0;i<%d;i++){}' % 0
dd = ''.join('{unsigned fg=%s,bg=%s; int ok=0; for(int q=0;q<2;q++) if(fg==combo[q][0]&&bg==combo[q][1]) ok=1; if(!ok||fg==bg) f3++;}' % (fg, bg) for fg, bg in sel_c)
hs = src[src.index('static void HdrText('):src.index('#define HDRT(')]
code = c % (period, hs, blink_line, blink_line, blink_line, blink_line, blink_line, vd, td, dd)
code = code.replace('uint8_t blink_on', 'uint8_t blink_on', 1)
(OUT/'t.c').write_text(code)
r = subprocess.run(['gcc','-O0','-Wall','-o',str(OUT/'t'),str(OUT/'t.c')],capture_output=True,text=True)
print(r.stderr[:1500]); assert r.returncode == 0
r = subprocess.run([str(OUT/'t')],capture_output=True,text=True); print(r.stdout)
chk(r.returncode == 0, 'C model: blink phase, colours, x=150 combos, no-flicker of other fields')
print('failures=%d' % fails)
raise SystemExit(fails)
