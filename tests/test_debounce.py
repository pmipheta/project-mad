"""(v79) Uniform input debounce for all 9 inputs (8 Button structs + PC13). Compiles the REAL ButtonSample + Debounce + ButtonRepeat/ButtonJustPressed/ButtonRaw + BtnEmit +
BtnDispatchEv + HandleButtons from main.c with fake GPIO / time / FreeRTOS queue (tests/btn_harness.py) and drives them with 1 ms-resolution pin timelines.
Usage: python3 test_debounce.py [MAIN_C]     (pass builds/v79-debounce/main_before.c as the negative control: it must FAIL)
Cases: (a) clean short / long press = exact action counts; (b) 10 ms random 0/1 chatter (1-15 ms runs) at press AND release edge, 200 seeded trials per button/kind
= same counts as clean; (c) chatter (released dips < 20 ms) in the middle of a held press = no extra action; (d) single 5 ms noise spike while released = no action;
(e) UP/DOWN hold-repeat call/step counts identical to the clean case; (f) the ~80 ms fallback cadence (HandleButtons samples itself) gives the same results; (g) source contracts."""
import random, re, subprocess, sys, tempfile, pathlib
import btn_harness as bh
ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / 'Core/Src/main.c'
OLDREF = ROOT.parent / 'builds/v79-debounce/main_before.c'
raw = MAIN.read_text(encoding='utf-8')
fails = 0
def chk(name, ok, extra=''):
    global fails
    print(('PASS ' if ok else 'FAIL ') + name + (' ' + extra if extra else '')); fails += (not ok)

C = r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
typedef struct { int dummy; } GPIO_TypeDef;
static GPIO_TypeDef PA, PB, PC, PF;
#define GPIOA (&PA)
#define GPIOB (&PB)
#define GPIOC (&PC)
#define GPIOF (&PF)
#define User_Blue_GPIO_Port GPIOC
#define User_Blue_Pin GPIO_PIN_13
#define GPIO_PIN_0 1
#define GPIO_PIN_1 2
#define GPIO_PIN_2 4
#define GPIO_PIN_3 8
#define GPIO_PIN_13 8192
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
%(defs)s
%(debouncer_t)s
%(button_t)s
static Button btn_up, btn_down, btn_hold, btn_set, btn_vdiv, btn_tdiv, btn_filt, btn_trig;
static uint8_t hold_active = 1;
static long c_inc, c_incs, c_dec, c_decs, c_auto, c_sel, c_freeze, c_stable, c_automode, c_fs;
static const char *OscSetName(void){ return "C1"; }
static void ActIncrease(int n){ c_inc++; c_incs += n; }
static void ActDecrease(int n){ c_dec++; c_decs += n; }
static void ActAuto(void){ c_auto++; }
static void ActFreeze(void){ c_freeze++; hold_active = !hold_active; }
static void ActToggleStable(void){ c_stable++; }
static void ActAutoMode(void){ c_automode++; }
static void ActToggleFastSlow(void){ c_fs++; }
static void ActToggleTrigEdge(void){ }
static void ActSelectSetting(const char *name){ (void)name; c_sel++; }
static void MvSync(void){ }
static uint32_t pins = 0;   /* bit0 up 1 down 2 hold 3 vdiv 4 tdiv 5 filt 6 trig 7 set 8 man (1 = pressed) */
static int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin){
    int bit = -1;
    if (p==&PB && pin==1) bit=0; else if (p==&PB && pin==2) bit=1; else if (p==&PB && pin==4) bit=2; else if (p==&PA && pin==8) bit=3;
    else if (p==&PC && pin==1) bit=4; else if (p==&PC && pin==8) bit=5; else if (p==&PF && pin==8) bit=6; else if (p==&PB && pin==8) bit=7; else if (p==&PC && pin==8192) bit=8;
    if (bit < 0) { printf("BAD PIN\n"); exit(3); }
    int pr = (pins >> bit) & 1;
    return (bit == 8) ? pr : !pr;      /* PC13 active high, others active low */
}
%(prelude)s
%(funcs)s
static void init(void){
    btn_up=(Button){BTN_UP_PORT,BTN_UP_PIN,0,0,0,0}; btn_down=(Button){BTN_DOWN_PORT,BTN_DOWN_PIN,0,0,0,0};
    btn_hold=(Button){BTN_HOLD_PORT,BTN_HOLD_PIN,0,0,0,0}; btn_set=(Button){BTN_SET_PORT,BTN_SET_PIN,0,0,0,0};
    btn_vdiv=(Button){BTN_VDIV_PORT,BTN_VDIV_PIN,0,0,0,0}; btn_tdiv=(Button){BTN_TDIV_PORT,BTN_TDIV_PIN,0,0,0,0};
    btn_filt=(Button){BTN_FILT_PORT,BTN_FILT_PIN,0,0,0,0}; btn_trig=(Button){BTN_TRIG_PORT,BTN_TRIG_PIN,0,0,0,0};
}
/* stdin protocol:  C cad | O phase | S mask ms (hold the pins for ms 1-ms steps; HandleButtons runs whenever T %% cad == 0) | R (zero counters, hold_active = 1) | P (print counters) */
int main(void){
    char cmd; long a, b; uint32_t T = 100000, cad = 5;
    init();
    while (scanf(" %%c", &cmd) == 1) {
        if (cmd == 'C') { scanf("%%ld", &a); cad = (uint32_t)a; }
        else if (cmd == 'O') { scanf("%%ld", &a); T = 100000 + (uint32_t)a; }
        else if (cmd == 'S') { scanf("%%ld %%ld", &a, &b); pins = (uint32_t)a; for (long i = 0; i < b; i++) { T++; if (T %% cad == 0) HandleButtons(T); } }
        else if (cmd == 'R') { c_inc=c_incs=c_dec=c_decs=c_auto=c_sel=c_freeze=c_stable=c_automode=c_fs=0; hold_active = 1; }
        else if (cmd == 'P') { printf("%%ld %%ld %%ld %%ld %%ld %%ld %%ld %%ld %%ld %%ld\n", c_inc, c_incs, c_dec, c_decs, c_auto, c_sel, c_freeze, c_stable, c_automode, c_fs); }
    }
    return 0;
}
'''
FIELDS = ['inc', 'incs', 'dec', 'decs', 'auto', 'sel', 'freeze', 'stable', 'automode', 'fs']

def build(text):
    ncx = re.sub(r'//[^\n]*', '', bh.strip_comments(text))
    names = (['Debounce'] if 'Debounce(' in ncx else []) + ['ButtonJustPressed', 'ButtonRepeat', 'ButtonRaw', 'ButtonLongPress'] + bh.split_names(text) + ['HandleButtons']
    defs = '\n'.join(re.search(r'^#define ' + n + r'\b[^\n]*', ncx, re.M)[0] for n in sorted(set(re.findall(r'^#define (BTN_\w+)', ncx, re.M))))
    dm = re.search(r'typedef struct \{[^}]*\} Debouncer;', ncx)
    btn_t = ncx[ncx.index('typedef struct {\n    GPIO_TypeDef *port;'):ncx.index('} Button;') + len('} Button;')]
    src = C % dict(defs=defs, debouncer_t=dm[0] if dm else '', button_t=btn_t, prelude=bh.prelude(text), funcs='\n'.join(bh.func_text(ncx, n) for n in names))
    d = pathlib.Path(tempfile.mkdtemp(prefix='osc_deb_'))
    (d / 'h.c').write_text(src)
    r = subprocess.run(['gcc', '-O1', '-Wall', '-Wno-unused-function', '-Wno-unused-variable', '-o', str(d / 'h'), str(d / 'h.c')], capture_output=True, text=True)
    if r.returncode: print(r.stderr[:2000]); return None
    return d / 'h'

exe = None
try: exe = build(raw)
except (AssertionError, ValueError, AttributeError) as ex: print('harness extraction failed:', repr(ex))
chk('harness compiles (real ButtonSample + dispatch from %s)' % MAIN.name, exe is not None)

def run(prog):
    """prog = list of command lines; returns one counter dict per 'P'."""
    r = subprocess.run([str(exe)], input='\n'.join(prog) + '\n', capture_output=True, text=True)
    return [dict(zip(FIELDS, map(int, l.split()))) for l in r.stdout.splitlines()]

UP, DN, HLD, VD, TD, FT, TR, ST, MAN = 1, 2, 4, 8, 16, 32, 64, 128, 256
BUT = [('PB0 UP', UP), ('PB1 DOWN', DN), ('PB2 AUTO', HLD), ('PA3 V/DIV', VD), ('PC0 T/DIV', TD), ('PF3 FREEZE', TR), ('PC3 FILT', FT), ('PB3 SET', ST), ('PC13 MAN', MAN)]
Z = dict.fromkeys(FIELDS, 0)
def exp(**kw): d = dict(Z); d.update(kw); return d
# (name, mask, short-press expectation, (long hold ms, long expectation) or None, mid-hold toggle hold ms)
CASES = [
    ('PB0 UP',     UP,  exp(inc=1, incs=1), None),
    ('PB1 DOWN',   DN,  exp(dec=1, decs=1), None),
    ('PB2 AUTO',   HLD, exp(auto=1), (2000, exp(auto=1))),
    ('PA3 V/DIV',  VD,  exp(sel=1), (2000, exp(sel=1))),
    ('PC0 T/DIV',  TD,  exp(sel=1), (2000, exp(sel=1))),
    ('PF3 FREEZE', TR,  exp(freeze=1), (2000, exp(freeze=1))),
    ('PC3 FILT',   FT,  exp(stable=1), (1700, exp(sel=1))),           # long (>= 1500 ms) = cursor toggle (hold_active = 1), no STABLE
    ('PB3 SET',    ST,  exp(fs=1), (1200, exp(automode=1))),          # long (>= 1000 ms) = AUTO mode, no toggle
    ('PC13 MAN',   MAN, exp(fs=1), (1200, exp(automode=1))),
]
SHORT_MS = 100
FB_SHORT_MS = 250   # at the 80 ms fallback cadence a press must span >= 2 samples to be seen at all (a 20 ms-debounce needs 2 reads >= 20 ms apart)
def idle(ms=500): return ['S 0 %d' % ms]
def chatter_window(rng, level, other, window=10):
    """10 ms of random 0/1 chatter made of runs of 1-15 ms (clipped to the window), as (mask, ms) list; 'level' = mask of the pressed state."""
    out, left, cur = [], window, rng.randint(0, 1)
    while left > 0:
        n = min(rng.randint(1, 15), left); out.append((level if cur else other, n)); left -= n; cur ^= 1
    return out
def segs(lst): return ['S %d %d' % (m, n) for m, n in lst if n > 0]

def trial_clean(m, hold): return idle() + segs([(m, hold)]) + idle()
def trial_bounce(rng, m, hold):
    return idle() + segs(chatter_window(rng, m, 0) + [(m, hold)] + chatter_window(rng, m, 0)) + idle()
def trial_mid(rng, m, hold, nb=None):
    """held press with 3-6 released bursts (random 0/1 chatter, every released run < 20 ms), none in the first 250 ms (past the 200 ms ButtonJustPressed lockout) nor the last 100 ms."""
    t, out = 0, []
    t += 250; out.append((m, 250))
    for _ in range(nb or rng.randint(3, 6)):
        gap = rng.randint(40, max(41, (hold - 350) // 6))
        if t + gap > hold - 100: break
        out.append((m, gap)); t += gap
        burst = chatter_window(rng, m, 0, rng.randint(2, 15))
        if burst[-1][0] == 0 and rng.random() < 0.5: burst.append((0, 0))
        if not any(x[0] == 0 for x in burst): burst = [(0, rng.randint(1, 15))]
        out += burst; t += sum(n for _, n in burst)
    out.append((m, max(1, hold - t)))
    return idle() + segs(out) + idle()
def trial_spike(rng, m): return idle(rng.randint(100, 300)) + segs([(m, 5)]) + idle()

def one(prog_body, phase=0, cad=5):
    """run a list of trial bodies (each is a list of lines) -> counters per trial."""
    prog = ['C %d' % cad]
    for body in prog_body:
        prog += ['R'] + body + ['P']
    return run(prog)

def check_group(name, results, want, extra=''):
    bad = [r for r in results if r != want]
    chk('%s: %d/%d trials give exactly %s' % (name, len(results) - len(bad), len(results), {k: v for k, v in want.items() if v}),
        not bad, ('first bad: %s' % {k: v for k, v in bad[0].items() if v}) if bad else '')

if exe:
    N = 200
    # ---------------- (a) clean ----------------
    for nm, m, short, lng in CASES:
        r = one([trial_clean(m, SHORT_MS)]); check_group('(a) %s clean short press' % nm, r, short)
        if lng: r = one([trial_clean(m, lng[0])]); check_group('(a) %s clean long press (%d ms)' % (nm, lng[0]), r, lng[1])
    # UP/DOWN: long press repeats; remember the clean reference for (e)
    ref = {}
    for nm, m, short, lng in CASES[:2]:
        r = one([trial_clean(m, 2310)])[0]; ref[nm] = r
        chk('(a) %s clean 2310 ms hold repeats (%d calls, %d steps; first step at once, accel after 1.5 s)' % (nm, r['inc'] + r['dec'], r['incs'] + r['decs']), (r['inc'] + r['dec']) > 20 and (r['incs'] + r['decs']) > (r['inc'] + r['dec']))
    # ---------------- (b) bounce at both edges ----------------
    for nm, m, short, lng in CASES:
        rng = random.Random(1000 + m)
        kinds = [('short', SHORT_MS, short)] + ([('long', lng[0], lng[1])] if lng else [])
        for kn, hold, want in kinds:
            check_group('(b) %s %s press, 10 ms chatter at press + release edge (seeded)' % (nm, kn), one([trial_bounce(rng, m, hold) for _ in range(N)]), want)
    # ---------------- (c) chatter in the middle of a held press ----------------
    for nm, m, short, lng in CASES[2:]:
        rng = random.Random(2000 + m)
        if m in (FT, ST, MAN):
            check_group('(c) %s 800 ms hold with released dips < 20 ms: toggle fires once' % nm, one([trial_mid(rng, m, 800) for _ in range(N)]), short)
            check_group('(c) %s %d ms hold with released dips < 20 ms: long action fires once, no short' % (nm, lng[0]), one([trial_mid(rng, m, lng[0]) for _ in range(N)]), lng[1])
        else:
            check_group('(c) %s 800 ms hold with released dips < 20 ms: one action' % nm, one([trial_mid(rng, m, 800) for _ in range(N)]), short)
    # ---------------- (d) single 5 ms spike while released ----------------
    for nm, m, short, lng in CASES:
        rng = random.Random(3000 + m)
        check_group('(d) %s single 5 ms noise spike while released: no action' % nm, one([trial_spike(rng, m) for _ in range(N)]), Z)
    # ---------------- (e) UP/DOWN hold-repeat counts identical to clean ----------------
    for nm, m, short, lng in CASES[:2]:
        rng = random.Random(4000 + m)
        check_group('(e) %s 2310 ms hold, edge bounce + mid-hold dips: repeat calls/steps identical to clean' % nm, one([trial_bounce(rng, m, 2310) for _ in range(N)]), ref[nm])
        rng = random.Random(4500 + m)
        check_group('(e) %s 2310 ms hold, released dips < 20 ms only: repeat calls/steps identical to clean' % nm, one([trial_mid(rng, m, 2310) for _ in range(N)]), ref[nm])
    # ---------------- (f) fallback cadence ~80 ms ----------------
    # NOTE: at 80 ms sampling only isolated dips can be rejected (two consecutive samples both landing in dips look like a real release); so mid-hold uses ONE dip per trial.
    for nm, m, short, lng in CASES:
        check_group('(f) %s @80 ms cadence clean short press (%d ms)' % (nm, FB_SHORT_MS), one([trial_clean(m, FB_SHORT_MS)], cad=80), short)
        if lng: check_group('(f) %s @80 ms cadence clean long press' % nm, one([trial_clean(m, lng[0])], cad=80), lng[1])
        rng = random.Random(5000 + m)
        check_group('(f) %s @80 ms cadence, edge bounce: one action' % nm, one([trial_bounce(rng, m, FB_SHORT_MS) for _ in range(N)], cad=80), short)
        if m not in (UP, DN):
            check_group('(f) %s @80 ms cadence, one released dip < 20 ms mid-hold: one action' % nm, one([trial_mid(rng, m, 800, nb=1) for _ in range(N)], cad=80), short)
        check_group('(f) %s @80 ms cadence, 5 ms spike while released: no action' % nm, one([trial_spike(rng, m) for _ in range(N)], cad=80), Z)
    r = one([trial_clean(UP, 2310)], cad=80)[0]
    chk('(f) UP hold-repeat at 80 ms cadence still repeats (%d calls)' % r['inc'], r['inc'] >= 15)

# ---------------- (g) source contracts ----------------
nc = re.sub(r'//[^\n]*', '', bh.strip_comments(raw))
def fb(n):
    try: return bh.func_text(nc, n)
    except AssertionError: return ''
chk('(g) BTN_STABLE_MS defined as 20 ms', re.search(r'#define BTN_STABLE_MS\s+20u?\b', nc) is not None)
chk('(g) Debounce() exists and uses BTN_STABLE_MS', 'Debounce(' in fb('Debounce') and 'BTN_STABLE_MS' in fb('Debounce'))
bs = fb('ButtonSample')
reads = re.findall(r'HAL_GPIO_ReadPin', bs)
chk('(g) ButtonSample: the only GPIO read (PC13) goes through Debounce()', len(reads) == 1 and re.search(r'Debounce\(&man_db,\s*\(HAL_GPIO_ReadPin', bs) is not None)
chk('(g) ButtonRaw reads through Debounce()', re.search(r'Debounce\(&b->db,\s*\(HAL_GPIO_ReadPin', fb('ButtonRaw')) is not None)
chk('(g) ButtonJustPressed no longer reads the pin (takes the debounced level)', 'HAL_GPIO_ReadPin' not in fb('ButtonJustPressed') and 'uint8_t raw' in fb('ButtonJustPressed'))
chk('(g) all 8 Button structs read via ButtonRaw in ButtonSample', all(('ButtonRaw(&btn_%s, now)' % n) in bs for n in ['up', 'down', 'hold', 'vdiv', 'tdiv', 'trig', 'filt', 'set']))
if OLDREF.exists() and MAIN.resolve() != OLDREF.resolve():
    o = OLDREF.read_text(encoding='utf-8')
    sl = lambda t: t[t.index('SLOW_BEGIN ='):t.index('SLOW_END =')]
    chk('(g) SLOW block byte-identical to builds/v79-debounce/main_before.c', sl(raw) == sl(o))
else:
    print('SKIP (g) SLOW-block identity: builds/v79-debounce/main_before.c not present (or is the file under test)')
print('debounce failures=%d' % fails)
sys.exit(1 if fails else 0)
