"""v78 fast LCD path: the register-level LcdWriteColumn / LcdFastPixel must emit the SAME (DC, byte) stream as the old HAL path
(ILI9341_Set_Address + HAL_SPI_Transmit, ILI9341_Draw_Pixel), with CS low for every byte, and only change DC / CS while the SPI is idle.
Old side: LcdWriteColumn from builds/v78-fast-lcd/main_before.c (v77) + the real driver functions, against a fake HAL.
New side: the LCDFAST block of Core/Src/main.c against a fake SPI5/GPIOC register model (seams LCD_DR_WR/LCD_SR_RD/LCD_BSRR...)."""
import pathlib, re, subprocess, sys, tempfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
BEFORE = ROOT.parent / 'builds/v78-fast-lcd/main_before.c'
fails = 0
def chk(name, ok, extra=''):
    global fails
    print(('ok   ' if ok else 'FAIL ') + name + ((' :: ' + extra) if (extra and not ok) else ''))
    if not ok: fails += 1

new_src = (ROOT / 'Core/Src/main.c').read_text(encoding='utf-8')
hdr_src = (ROOT / 'Core/Inc/lcd_fast.h').read_text(encoding='utf-8')
drv = (ROOT / 'Core/Src/ILI9341/ILI9341_STM32_Driver.c').read_text(encoding='utf-8', errors='replace')
a, b = hdr_src.index('/* LCDFAST_BEGIN'), hdr_src.index('/* LCDFAST_END')
fast_block = hdr_src[a:b]
def c_func(text, name):          # text of a top-level C function `... name(...) {...}` (brace matched)
    m = re.search(r'^[A-Za-z_][^\n;{]*\b' + name + r'\s*\([^)]*\)\s*\{', text, re.M)
    assert m, name
    i = m.end(); depth = 1
    while depth:
        depth += {'{': 1, '}': -1}.get(text[i], 0); i += 1
    return text[m.start():i]

_mc = c_func(new_src, 'LcdWriteColumn')
_dl = [l for l in _mc.split('\n') if 'LcdWriteColumnFast' in l]
chk('main.c includes lcd_fast.h; LcdWriteColumn = one dispatch line on LCD_FAST_SPI + the v77 HAL body', '#include "lcd_fast.h"' in new_src and len(_dl) == 1 and _dl[0].lstrip().startswith('if (LCD_FAST_SPI) {'))
_old_col = _mc.replace(_dl[0] + '\n', '')        # the kept HAL body (= LCD_FAST_SPI 0 path)
if BEFORE.exists():
    chk('main.c LcdWriteColumn minus the dispatch line is byte-identical to the v77 function', _old_col == c_func(BEFORE.read_text(encoding='utf-8'), 'LcdWriteColumn'))
else:
    print('SKIP byte-identical-to-v77 check: builds/v78-fast-lcd/main_before.c not in this checkout (old side = the kept HAL body in main.c)')

# ---------------------------------------------------------------- common test driver (prints the recorded stream)
CASES = r'''
int main(void){
  static uint8_t buf[480];
  int yr[][2] = {{29,29},{29,30},{30,239},{29,239},{100,105},{0,239},{120,120},{200,239},{57,58},{238,239}};
  int nyr = sizeof(yr)/sizeof(yr[0]);
  for (int x = 0; x < 320; x++) for (int r = 0; r < nyr; r++) {
    int y1 = yr[r][0], y2 = yr[r][1]; uint32_t n = 2u*(y2-y1+1);
    for (uint32_t i = 0; i < n; i++) buf[i] = (uint8_t)(i*37u + x*11u + 5u);
    printf("# col %d %d %d\n", x, y1, y2); BEGIN_CASE(); LcdWriteColumn(x, y1, y2, buf, n); END_CASE();
  }
  int ys[] = {0,1,28,29,100,127,128,239,240,255,256,300};
  uint16_t cols[] = {0x0000,0xFFE0,0x7BEF,0x03EF,0x1234,0xFEDC};
  int xs[] = {0,1,2,127,128,254,255,256,319,320,400};
  for (unsigned xi = 0; xi < sizeof(xs)/sizeof(xs[0]); xi++) for (unsigned yi = 0; yi < sizeof(ys)/sizeof(ys[0]); yi++)
    for (unsigned ci = 0; ci < 6; ci++) {
      printf("# pix %d %d %04x\n", xs[xi], ys[yi], cols[ci]); BEGIN_CASE(); LcdFastPixel(xs[xi], ys[yi], cols[ci]); END_CASE(); }
  for (int x = 0; x < 320; x++) { printf("# pix %d 29 7bef\n", x); BEGIN_CASE(); LcdFastPixel(x, 29, 0x7BEF); END_CASE(); }
  return 0;
}
'''
HEAD = r'''
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#define FAST_O2
#define CS_Pin 0x0100
#define DC_Pin 0x0200
#define LCD_CS_PORT GPIOC
#define LCD_CS_PIN  CS_Pin
#define LCD_DC_PORT GPIOC
#define LCD_DC_PIN  DC_Pin
volatile uint16_t LCD_WIDTH = 320, LCD_HEIGHT = 240;
'''

OLD_C = HEAD + r'''
typedef struct { int dummy; } GPIO_TypeDef; static GPIO_TypeDef gpioc_obj; 
#define GPIOC (&gpioc_obj)
typedef struct { int dummy; } SPI_HandleTypeDef; static SPI_HandleTypeDef hspi5;
#define HSPI_INSTANCE (&hspi5)
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
typedef int GPIO_PinState;
static int dc = 0, cs = 1, started = 0;
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, GPIO_PinState s){ (void)p; if (pin == CS_Pin) cs = s; else if (pin == DC_Pin) dc = s; else exit(3); }
int HAL_SPI_Transmit(SPI_HandleTypeDef *h, const uint8_t *d, uint16_t n, uint32_t t){ (void)h; (void)t;
  for (uint16_t i = 0; i < n; i++) printf("%d %02x %d\n", dc, d[i], cs); return 0; }
#define BEGIN_CASE() do{}while(0)
#define END_CASE()   do{}while(0)
void ILI9341_Set_Address(uint16_t,uint16_t,uint16_t,uint16_t);
void ILI9341_Draw_Pixel(uint16_t,uint16_t,uint16_t);
#define LcdFastPixel(x,y,c) ILI9341_Draw_Pixel((x),(y),(c))
''' 

def old_source():
    old_col = c_func(BEFORE.read_text(encoding='utf-8'), 'LcdWriteColumn') if BEFORE.exists() else _old_col
    fns = [c_func(drv, n) for n in ('ILI9341_SPI_Send', 'ILI9341_Write_Command', 'ILI9341_Write_Data', 'ILI9341_Set_Address', 'ILI9341_Draw_Pixel')]
    # prototype for each so order does not matter
    return OLD_C + '\n'.join(fns) + '\n' + old_col + CASES

NEW_C = HEAD + r'''
typedef struct { volatile uint32_t CR1, SR, DR; } SPI_TypeDef;
typedef struct { volatile uint32_t BSRR; } GPIO_TypeDef;
static SPI_TypeDef spi_obj = { 0x40, 0, 0 };            /* SPE already set */
static GPIO_TypeDef gpioc_obj;
#define SPI5  (&spi_obj)
#define GPIOC (&gpioc_obj)
#define SPI_CR1_SPE  0x40u
#define SPI_SR_TXE   0x02u
#define SPI_SR_BSY   0x80u
#define SPI_SR_FTLVL 0x1800u
#define SPI_SR_FRLVL 0x0600u
#define HAL_SPI_Transmit(...) (abort(),0)                /* the fast path must never call these */
#define HAL_GPIO_WritePin(...) abort()
#define ILI9341_Set_Address(...) abort()
#define ILI9341_Draw_Pixel(...) abort()
/* register model: after a DR write TXE reads 0 once and BSY/FTLVL stay set for 3 SR reads; every DC/CS change and CS must be seen idle */
static int dc = 0, cs = 1, busy_reads = 0, txe_wait = 0, in_case = 0, viol_idle = 0, viol_cs = 0, viol_ovr = 0, evt_dr = 0, evt_sr = 0, last_ev = 0, spe_clear = 0;
static uint32_t fake_sr(void){ evt_sr++; last_ev = 2; uint32_t v = 0;
  if (txe_wait) { txe_wait--; } else v |= SPI_SR_TXE;
  if (busy_reads) { if (busy_reads > 4) v |= SPI_SR_FTLVL; busy_reads--; v |= SPI_SR_BSY; }
  return v; }
static uint32_t fake_dr_rd(void){ evt_dr++; last_ev = 1; return 0; }
static void fake_dr_wr(uint8_t b){ if (cs) viol_cs++; if (!(spi_obj.CR1 & SPI_CR1_SPE)) spe_clear++; printf("%d %02x %d\n", dc, b, cs); txe_wait = 1; busy_reads = 6; last_ev = 0; }
static void fake_bsrr(uint32_t port_is_c, uint32_t v){ (void)port_is_c; if (busy_reads) viol_idle++;
  if (v & 0xFFFF0000u) { uint32_t m = v >> 16; if (m & CS_Pin) cs = 0; if (m & DC_Pin) dc = 0; }
  if (v & 0xFFFFu) { if (v & CS_Pin) cs = 1; if (v & DC_Pin) dc = 1; } }
#define LCD_DR_WR(b)      fake_dr_wr(b)
#define LCD_DR_RD()       fake_dr_rd()
#define LCD_SR_RD()       fake_sr()
#define LCD_BSRR(port, v) fake_bsrr(1, (v))
#define BEGIN_CASE() do{ in_case = 1; }while(0)
#define END_CASE()   do{ if (cs != 1 || busy_reads) viol_idle++; in_case = 0; }while(0)
''' 

def new_source(block, pre=''):
    tail = CASES.replace('return 0;\n}', 'fprintf(stderr, "viol_idle=%d viol_cs=%d spe_clear=%d\\n", viol_idle, viol_cs, spe_clear);\n  return 0;\n}')
    return pre + NEW_C + block + '#define LcdWriteColumn LcdWriteColumnFast\n' + tail

def build_run(src, tag, extra=()):
    d = pathlib.Path(tempfile.mkdtemp(prefix='fastlcd_' + tag))
    (d / 'h.c').write_text(src)
    r = subprocess.run(['gcc', '-O1', '-w', *extra, '-o', str(d / 'h'), str(d / 'h.c')], capture_output=True, text=True)
    if r.returncode: return None, r.stderr[:3000]
    p = subprocess.run([str(d / 'h')], capture_output=True, text=True)
    return p, ''

def parse(out):
    cases, cur = [], None
    for l in out.splitlines():
        if l.startswith('#'): cur = [l, []]; cases.append(cur)
        else:
            dc_, by, cs_ = l.split(); cur[1].append((int(dc_), by, int(cs_)))
    return cases

old_p, err = build_run(old_source(), 'old')
chk('old HAL path (v77 LcdWriteColumn + real driver functions) compiles and runs', old_p is not None and old_p.returncode == 0, err)
old = parse(old_p.stdout) if old_p else []
chk('old reference is rich (%d cases)' % len(old), len(old) > 3500)
chk('old reference: every byte is sent with CS low (sanity of the harness)', all(c == 0 for _, bs in old for (_, _, c) in bs))

def compare(label, block, expect_equal=True):
    p, err = build_run(new_source(block), label)
    if p is None: return False, 'compile: ' + err, None
    if p.returncode: return False, 'run rc=%d %s' % (p.returncode, p.stderr[-300:]), p.stderr
    new = parse(p.stdout)
    if len(new) != len(old): return False, 'case count %d vs %d' % (len(new), len(old)), p.stderr
    for (h1, s1), (h2, s2) in zip(old, new):
        if h1 != h2: return False, 'case order', p.stderr
        if [(d, b) for d, b, _ in s1] != [(d, b) for d, b, _ in s2]:
            return False, '%s: old=%s new=%s' % (h1, s1[:14], s2[:14]), p.stderr
        if any(c != 0 for _, _, c in s2): return False, '%s: byte sent with CS high' % h1, p.stderr
    m = re.search(r'viol_idle=(\d+) viol_cs=(\d+) spe_clear=(\d+)', p.stderr)
    if not m or any(int(g) for g in m.groups()): return False, 'register-model violations ' + p.stderr[-200:], p.stderr
    return True, '', p.stderr

ok, why, _ = compare('new', fast_block)
chk('new register path == old HAL path: identical (DC, byte) stream for %d columns/pixels, CS low on every byte, DC/CS only changed while idle, SPE kept' % len(old), ok, why)

# ---- the test must be able to fail: deliberately broken variants of the new code
def broken(name, old_s, new_s, count=1):
    assert fast_block.count(old_s) >= 1, name
    blk = fast_block.replace(old_s, new_s, count)
    ok, why, _ = compare('brk_' + name, blk)
    chk('negative control "%s" is detected (%s)' % (name, why[:70] if why else 'NOT DETECTED'), not ok)
broken('swapped DC for one byte block (0x2B parameters sent as command)', 'LcdSpiCmd(0x2B);\n    a[0] = (uint8_t)(y1 >> 8); a[1] = (uint8_t)y1; a[2] = (uint8_t)(y2 >> 8); a[3] = (uint8_t)y2;\n    LcdSpiData(a, 4);',
       'LcdSpiCmd(0x2B);\n    a[0] = (uint8_t)(y1 >> 8); a[1] = (uint8_t)y1; a[2] = (uint8_t)(y2 >> 8); a[3] = (uint8_t)y2;\n    LcdSpiCmd(a[0]); LcdSpiData(a + 1, 3);')
broken('pixel X+1 high byte wrong', '(uint8_t)((X + 1) >> 8)', '(uint8_t)(X >> 8)')
broken('DC flipped before SPI idle', 'while (LCD_SR_RD() & SPI_SR_BSY)   { }', '')
broken('CS raised early (last data byte sent with CS high)', 'LcdSpiData(bytes, (uint16_t)n_bytes);\n    LCD_CS_HIGH();', 'LCD_CS_HIGH(); LcdSpiData(bytes, (uint16_t)n_bytes);')

# ---- old path is selectable and still identical
ok, why, _ = compare('off', fast_block.replace('#define LCD_FAST_SPI 1', '#define LCD_FAST_SPI 0'), True) if False else (None, '', None)
off_src = OLD_C + '\n'.join(c_func(drv, n) for n in ('ILI9341_SPI_Send', 'ILI9341_Write_Command', 'ILI9341_Write_Data', 'ILI9341_Set_Address', 'ILI9341_Draw_Pixel')) + '\n'
off_blk = fast_block.replace('#define LCD_FAST_SPI 1', '#define LCD_FAST_SPI 0')
off_src = off_src.replace('#define LcdFastPixel(x,y,c) ILI9341_Draw_Pixel((x),(y),(c))\n', '') + off_blk + c_func(new_src, 'LcdWriteColumn') + CASES
p, err = build_run(off_src, 'off')
chk('LCD_FAST_SPI=0 compiles (old HAL path + LcdFastPixel -> ILI9341_Draw_Pixel)', p is not None and p.returncode == 0, err)
if p and p.returncode == 0:
    chk('LCD_FAST_SPI=0 emits exactly the old stream', parse(p.stdout) == old)

# ---- static checks on the real source
fn = c_func(fast_block, 'LcdSpiBytes')
i_ftl, i_bsy = fn.index('SPI_SR_FTLVL'), fn.index('SPI_SR_BSY')
i_ovr_dr, i_ovr_sr = fn.index('(void)LCD_DR_RD()'), fn.index('(void)LCD_SR_RD()')
i_loop = fn.index('LCD_DR_WR(')
chk('static: LcdSpiBytes waits FTLVL==0 then !BSY after the last byte, before returning', i_loop < i_ftl < i_bsy)
chk('static: OVR cleared by reading DR then SR after the idle wait (as __HAL_SPI_CLEAR_OVRFLAG)', i_bsy < i_ovr_dr < i_ovr_sr)
chk('static: SPE enabled only if not set (as HAL_SPI_Transmit)', re.search(r'if \(!\(SPI5->CR1 & SPI_CR1_SPE\)\) SPI5->CR1 \|= SPI_CR1_SPE;', fn) is not None)
chk('static: TXE polled before each byte write', fn.index('SPI_SR_TXE') < i_loop)
chk('static: DR written as 8-bit access, BSRR used for CS/DC', '*(volatile uint8_t *)&SPI5->DR' in fast_block and '(port)->BSRR' in fast_block)
chk('static: every DC flip goes through LcdSpiCmd/LcdSpiData (the only callers of LCD_DC_*), each following an idle-waiting LcdSpiBytes',
    len(re.findall(r'LCD_DC_(?:LOW|HIGH)\(\);', fast_block)) == 2 and 'LCD_DC_LOW();  LcdSpiBytes' in fast_block and 'LCD_DC_HIGH(); LcdSpiBytes' in fast_block)
col = c_func(fast_block.split('#else')[0], 'LcdWriteColumnFast')
chk('static: CS low once, high once in LcdWriteColumn', col.count('LCD_CS_LOW()') == 1 and col.count('LCD_CS_HIGH()') == 1)
_nc = re.sub(r'/\*.*?\*/', '', fast_block.split('#else')[0], flags=re.S)
chk('static: no HAL call / hspi5 access in the fast code (comments stripped)', 'HAL_' not in _nc and 'hspi5' not in _nc)
chk('static: LCD_FAST_SPI defined as 1 in main.c', '#define LCD_FAST_SPI 1' in fast_block)
# real CMSIS bit positions behind the fake constants
cm = (ROOT / 'Drivers/CMSIS/Device/ST/STM32F7xx/Include/stm32f767xx.h').read_text(errors='replace')
def pos(n): return int(re.search(r'#define %s_Pos\s+\((\d+)U\)' % n, cm).group(1))
chk('CMSIS bit positions: SPE=6 TXE=1 BSY=7 FTLVL=11..12', (pos('SPI_CR1_SPE'), pos('SPI_SR_TXE'), pos('SPI_SR_BSY'), pos('SPI_SR_FTLVL')) == (6, 1, 7, 11))
# cursor functions use the fast pixel only
cur_a = new_src.index('static void ClearCursors(void)'); cur_b = new_src.index('static void DrawCursor(void)')
cur_c = new_src.index('#undef ILI9341_Draw_Pixel', cur_b)
redir = new_src.index('#define ILI9341_Draw_Pixel LcdFastPixel')
cur = new_src[cur_a:cur_c]
chk('cursor pixels: ILI9341_Draw_Pixel is redirected to LcdFastPixel only from before ClearCursors to after DrawCursor (4 call sites)',
    redir < cur_a and cur.count('ILI9341_Draw_Pixel(') == 4 and new_src.count('#define ILI9341_Draw_Pixel') == 1 and new_src.count('#undef ILI9341_Draw_Pixel') == 1
    and 'DrawWaveform' not in new_src[redir:cur_c] and 'void DrawGrid' not in new_src[redir:cur_c])
print('fast_lcd failures=%d' % fails)
sys.exit(1 if fails else 0)
