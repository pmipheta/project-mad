"""Host test of the v62 eight-button panel (PB0-3 as v61; PA3/PC0/PC3/PF3 revised). Real HandleButtons/ButtonJustPressed/ButtonRepeat/ButtonRaw/ActIncrease/ActDecrease/ActUp/ActDown/
ActSelectSetting/ActToggleTrigEdge/OscSetIndex extracted from Core/Src/main.c; GPIO, OscAdjust, MvMove, ActAuto, ActToggleFastSlow, ActAutoMode are recording stubs.
Not a board test."""
from pathlib import Path
import re, subprocess, tempfile
ROOT = Path(__file__).resolve().parents[1]
OUT = Path(tempfile.gettempdir()) / 'osc_eightbtn'; OUT.mkdir(parents=True, exist_ok=True)
from rtos_norm import unrtos
from _unlock import unlock
import btn_harness as bh
src = unrtos(unlock((ROOT/'Core/Src/main.c').read_text()))   # v71: undo the step-3a lock wrappers before the 'unchanged' checks
nocom = re.sub(r'/\*.*?\*/', '', src, flags=re.S)
def function(name):
    m = re.search(r'^(?:static\s+)?(?:void|int|float|uint8_t|const char\s*\*)\s*'+name+r'\([^;]*?\)\s*\{', nocom, re.M)
    assert m, name
    p = nocom.index('{', m.start()); d = 1; i = p+1
    while d:
        d += (nocom[i] == '{') - (nocom[i] == '}'); i += 1
    return nocom[m.start():i]
names = ['Debounce','ButtonJustPressed','ButtonRepeat','ButtonRaw','HandleButtons','ActIncrease','ActDecrease','ActUp','ActDown','ActSelectSetting','ActToggleTrigEdge','OscSetIndex','OscSetName','ActToggleStable']
raw3 = unlock((ROOT/'Core/Src/main.c').read_text())      # v77: HandleButtons is compiled as the REAL sampler+dispatch (fake queue), not the reconstructed combined text
nocom3 = bh.strip_comments(raw3)
funcs = '\n'.join(function(n) for n in names if n != 'HandleButtons') + '\n' + '\n'.join(bh.func_text(nocom3, n) for n in bh.split_names(raw3) + ['HandleButtons'])
macros = '\n'.join(re.search(r'^#define '+n+r'\s+[^/\n]+', nocom, re.M)[0] for n in
  ['BTN_CURSOR_LONG_MS','BTN_UP_PIN','BTN_DOWN_PIN','BTN_HOLD_PIN','BTN_SET_PIN','BTN_VDIV_PIN','BTN_TDIV_PIN','BTN_FILT_PIN','BTN_TRIG_PIN','BTN_MAN_LONG_MS','BTN_DEBOUNCE_MS','BTN_LONG_MS',
   'BTN_REPEAT_FIRST_MS','BTN_REPEAT_MS','BTN_ACCEL_MS','BTN_ACCEL_STEPS','BTN_REL_MS','BTN_STABLE_MS','BTN_MAN_ACTIVE_HIGH'])
names_line = re.search(r'static const char\s+\*osc_set_names\[\][^;]*;', nocom)[0] + re.search(r'static const char\s+\*osc_hold_names\[\][^;]*;', nocom)[0]
c = r'''
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
#define GPIO_PIN_0 1
#define GPIO_PIN_1 2
#define GPIO_PIN_2 4
#define GPIO_PIN_3 8
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0
#define GPIO_PinState int
#define BTN_UP_PORT GPIOB
#define BTN_DOWN_PORT GPIOB
#define BTN_HOLD_PORT GPIOB
#define BTN_SET_PORT GPIOB
#define BTN_VDIV_PORT GPIOA
#define BTN_TDIV_PORT GPIOC
#define BTN_FILT_PORT GPIOC
#define BTN_TRIG_PORT GPIOF
#define BTN_MAN_PORT GPIOC
#define BTN_MAN_PIN 0x2000
''' + macros + r'''
typedef struct { uint8_t level, pend; uint32_t since; } Debouncer;   /* v79 */
typedef struct { GPIO_TypeDef *port; uint16_t pin; uint32_t last_press_tick, held_since_tick; uint8_t is_held, long_fired; uint32_t next_rep_tick, low_tick; uint8_t low_pending; Debouncer db; } Button;
static Button btn_up, btn_down, btn_hold, btn_set, btn_vdiv, btn_tdiv, btn_filt, btn_trig;
''' + names_line + r'''
#define OSC_SET_COUNT 4
#define OSC_HOLD_COUNT 3
static int osc_sidx = 0, osc_trig_fall = 0, osc_dirty = 0, hold_active = 0;
static int n_freeze = 0; static uint8_t osc_stable = 1;
static int OscSetCount(void){ return hold_active ? OSC_HOLD_COUNT : OSC_SET_COUNT; }
static void ActFreeze(void){ n_freeze++; hold_active = !hold_active; osc_sidx = 0; }
/* pin state: pressed[port-id][pin] */
static int pressed_mask[4];   /* 0=A 1=B 2=C 3=F, bit per pin number; man (PC13) bit 13 of C */
static int pid(GPIO_TypeDef *p){ return p==&PA?0: p==&PB?1: p==&PC?2:3; }
static int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin){
    int low = (pressed_mask[pid(p)] & pin) != 0;
    if (p==&PC && pin==BTN_MAN_PIN) return low ? 1 : 0;     /* active high */
    return low ? 0 : 1;                                       /* active low */
}
static int n_adj_p, n_adj_m, n_move, n_auto, n_fs, n_am, n_sync;
static void OscAdjust(int d){ if (d>0) n_adj_p++; else n_adj_m++; }
static void MvMove(int d, int n){ (void)d;(void)n; n_move++; }
static void MvSync(void){ n_sync++; }
static void ActAuto(void){ n_auto++; }
static void ActToggleFastSlow(void){ n_fs++; }
static void ActAutoMode(void){ n_am++; }
static void ActIncrease(int n); static void ActDecrease(int n); static void ActSelectSetting(const char *name); static void ActToggleTrigEdge(void);
static uint8_t ButtonLongPress(Button *b, uint32_t now){ (void)b;(void)now; return 0; }
static void ActUp(int n); static void ActDown(int n); static void OscSetIndex(int idx); static const char *OscSetName(void); static void ActFreeze(void); static void ActToggleStable(void);
''' + bh.prelude(raw3) + funcs + r'''
static int fails = 0;
#define CHECK(name, cond) do { int ok_ = !!(cond); printf("%s %s\n", ok_?"PASS":"FAIL", name); if(!ok_) fails++; } while(0)
static void clr(void){ osc_dirty=0; n_adj_p=n_adj_m=n_move=n_auto=n_fs=n_am=n_freeze=0; }
static void press(int port, int pin, int on){ if(on) pressed_mask[port] |= pin; else pressed_mask[port] &= ~pin; }
static uint32_t T = 1000;
static void init(void){
    btn_up=(Button){BTN_UP_PORT,BTN_UP_PIN,0,0,0,0}; btn_down=(Button){BTN_DOWN_PORT,BTN_DOWN_PIN,0,0,0,0};
    btn_hold=(Button){BTN_HOLD_PORT,BTN_HOLD_PIN,0,0,0,0}; btn_set=(Button){BTN_SET_PORT,BTN_SET_PIN,0,0,0,0};
    btn_vdiv=(Button){BTN_VDIV_PORT,BTN_VDIV_PIN,0,0,0,0}; btn_tdiv=(Button){BTN_TDIV_PORT,BTN_TDIV_PIN,0,0,0,0};
    btn_filt=(Button){BTN_FILT_PORT,BTN_FILT_PIN,0,0,0,0}; btn_trig=(Button){BTN_TRIG_PORT,BTN_TRIG_PIN,0,0,0,0};
    memset(pressed_mask,0,sizeof pressed_mask); clr();
}
static void run(uint32_t ms){ for (uint32_t i=0;i<ms;i+=5){ T+=5; HandleButtons(T);} }
static void tap(int port,int pin,uint32_t hold){ press(port,pin,1); run(hold); press(port,pin,0); run(300); }
int main(void){
    int hold;
    /* (a) ActSelectSetting by name: live array */
    hold_active = 0;
    for (int prior=0; prior<4; prior++) {
        osc_sidx = prior; ActSelectSetting("V/DIV"); CHECK("live: select V/DIV from any state", osc_sidx==2);
        osc_sidx = prior; ActSelectSetting("T/DIV"); CHECK("live: select T/DIV from any state", osc_sidx==3);
        osc_sidx = prior; ActSelectSetting("FILTER"); CHECK("live: select FILTER from any state", osc_sidx==0);
        osc_sidx = prior; ActSelectSetting("TRIG"); CHECK("live: select TRIG from any state", osc_sidx==1);
        osc_sidx = prior; ActSelectSetting("NOPE"); CHECK("(i) unknown name leaves selection", osc_sidx==prior);
        osc_sidx = prior; ActSelectSetting("C1"); CHECK("(i) live: C1 not in array -> no-op", osc_sidx==prior);
    }
    hold_active = 1;
    for (int prior=0; prior<3; prior++) {
        osc_sidx = prior; ActSelectSetting("C1"); CHECK("frozen: select C1", osc_sidx==0);
        osc_sidx = prior; ActSelectSetting("C2"); CHECK("frozen: select C2", osc_sidx==1);
        osc_sidx = prior; ActSelectSetting("V/DIV"); CHECK("frozen: select V/DIV", osc_sidx==2);
        osc_sidx = prior; ActSelectSetting("T/DIV"); CHECK("(i) frozen: T/DIV not in array -> no-op", osc_sidx==prior);
        osc_sidx = prior; ActSelectSetting("FILTER"); CHECK("(i) frozen: FILTER no-op", osc_sidx==prior);
        osc_sidx = prior; ActSelectSetting("TRIG"); CHECK("(i) frozen: TRIG no-op", osc_sidx==prior);
        osc_sidx = prior; ActSelectSetting("NOPE"); CHECK("(i) frozen: unknown no-op", osc_sidx==prior);
    }
    /* (a) PA3 toggles V/DIV <-> T/DIV (live) */
    hold_active = 0; init(); osc_sidx = 0;
    tap(0,8,100); CHECK("(a) PA3 from FILTER -> V/DIV", osc_sidx==2);
    tap(0,8,100); CHECK("(a) PA3 V/DIV -> T/DIV", osc_sidx==3);
    tap(0,8,100); CHECK("(a) PA3 T/DIV -> V/DIV", osc_sidx==2);
    tap(0,8,100); CHECK("(a) PA3 V/DIV -> T/DIV again", osc_sidx==3);
    /* (b) PA3 while frozen always ends on V/DIV */
    hold_active = 1; init(); osc_sidx = 0;
    for (int k=0;k<4;k++) { tap(0,8,100); CHECK("(b) PA3 frozen -> V/DIV (index 2) every press", osc_sidx==2 && hold_active==1); }
    osc_sidx = 1; tap(0,8,100); CHECK("(b) PA3 frozen from C2 -> V/DIV", osc_sidx==2);
    /* (c) PC0 toggles FILTER <-> TRIG (live) */
    hold_active = 0; init(); osc_sidx = 2;
    tap(2,1,100); CHECK("(c) PC0 from V/DIV -> FILTER", osc_sidx==0);
    tap(2,1,100); CHECK("(c) PC0 FILTER -> TRIG", osc_sidx==1);
    tap(2,1,100); CHECK("(c) PC0 TRIG -> FILTER", osc_sidx==0);
    /* (d) PC0 no-op while frozen */
    hold_active = 1; init();
    for (int s0=0;s0<3;s0++) { osc_sidx = s0; tap(2,1,100); CHECK("(d) PC0 frozen: selection unchanged", osc_sidx==s0 && hold_active==1); }
    /* v63 (e) PC3 short release -> ActToggleStable (osc_stable flip + osc_dirty), NEVER ActFreeze/hold_active */
    hold_active = 0; init(); osc_stable = 1; osc_sidx = 3; press(2,8,1); run(500); CHECK("(e) PC3 down 500ms: nothing yet", osc_stable==1 && osc_dirty==0);
    press(2,8,0); run(300); CHECK("(e) PC3 release@500ms -> osc_stable 1->0, dirty set, no freeze", osc_stable==0 && osc_dirty==1 && n_freeze==0 && hold_active==0 && osc_sidx==3);
    clr(); press(2,8,1); run(500); press(2,8,0); run(300); CHECK("(e) PC3 short again -> osc_stable 0->1", osc_stable==1 && hold_active==0 && n_freeze==0);
    hold_active = 1; init(); osc_stable = 1; osc_sidx = 1; press(2,8,1); run(500); press(2,8,0); run(300);
    CHECK("(e) frozen on C2: short PC3 toggles stable only, no unfreeze, no C1/C2 toggle", osc_stable==0 && hold_active==1 && osc_sidx==1 && n_freeze==0);
    hold_active = 0; init(); osc_stable = 1; tap(2,8,20); CHECK("(e) PC3 <40ms glitch ignored", osc_stable==1 && n_freeze==0);
    /* (f) PC3 long hold while frozen: C1<->C2, once, release does not freeze */
    hold_active = 1; init(); osc_sidx = 0; press(2,8,1); run(1400); CHECK("(f) PC3 1400ms: not yet", osc_sidx==0 && n_freeze==0);
    run(200); CHECK("(f) PC3 >=1500ms C1 -> C2", osc_sidx==1 && n_freeze==0);
    run(3000); CHECK("(f) PC3 keeps holding: fires once only", osc_sidx==1);
    press(2,8,0); run(300); CHECK("(f) release after long: no freeze, osc_stable untouched (v63 regression)", n_freeze==0 && hold_active==1 && osc_sidx==1 && osc_stable==1 && osc_dirty==0);
    press(2,8,1); run(1600); press(2,8,0); run(300); CHECK("(f) second long: C2 -> C1, no freeze", osc_sidx==0 && n_freeze==0);
    osc_sidx = 2; press(2,8,1); run(1600); press(2,8,0); run(300); CHECK("(f) long from V/DIV -> C1", osc_sidx==0 && n_freeze==0);
    /* (g) PC3 long while live: nothing, no selection change, release does not freeze */
    hold_active = 0; init(); osc_sidx = 3; press(2,8,1); run(1700); CHECK("(g) live long: no selection change", osc_sidx==3 && n_freeze==0);
    press(2,8,0); run(300); CHECK("(g) live long release: short action suppressed (no stable toggle, no freeze)", n_freeze==0 && hold_active==0 && osc_sidx==3 && osc_stable==1);
    /* (h) v64: PF3 = ActFreeze (hold_active toggle), never touches osc_stable. */
    hold_active = 0; init(); osc_stable = 1; osc_sidx = 2; tap(3,8,100); CHECK("(h) PF3 press -> hold_active 0->1 via ActFreeze, osc_stable untouched", hold_active==1 && n_freeze==1 && osc_stable==1 && osc_sidx==0);
    tap(3,8,100); CHECK("(h) PF3 again -> unfreeze", hold_active==0 && n_freeze==2 && osc_stable==1);
    hold_active = 0; init(); osc_stable = 1; tap(2,8,100); CHECK("(h) PC3 short still toggles stable only, hold_active untouched (regression)", osc_stable==0 && n_freeze==0 && hold_active==0);
    /* (h2) real-world flow: PF3 freeze -> PC3 long 1500ms toggles C1<->C2 */
    hold_active = 0; init(); osc_stable = 1; osc_sidx = 3; tap(3,8,100); CHECK("(h2) PF3 freeze", hold_active==1 && osc_sidx==0);
    press(2,8,1); run(1600); CHECK("(h2) PC3 long after PF3 freeze: C1->C2", osc_sidx==1); press(2,8,0); run(300);
    CHECK("(h2) release: still frozen, stable untouched", hold_active==1 && osc_stable==1);
    press(2,8,1); run(1600); press(2,8,0); run(300); CHECK("(h2) second long: C2->C1", osc_sidx==0 && hold_active==1);
    CHECK("(h) none of the 4 pins call move/adjust", n_move==0 && n_adj_p==0 && n_adj_m==0);
    /* PF3 debounce. v79: bounce pulses are 30 ms (was 20): a pulse shorter than BTN_STABLE_MS is now rejected by the input debouncer itself (covered in test_debounce.py);
       30 ms pulses pass it, so the 200 ms ButtonJustPressed lockout is still what these cases exercise. */
    hold_active = 0; init(); osc_stable = 1; press(3,8,1); run(30); press(3,8,0); run(30); press(3,8,1); run(30); press(3,8,0); run(30);
    CHECK("PF3: bounce within 200ms = one freeze toggle", hold_active==1 && n_freeze==1 && osc_stable==1);
    run(300); press(3,8,1); run(2000); CHECK("PF3: held press toggles freeze once only", hold_active==0 && n_freeze==2 && osc_stable==1); press(3,8,0); run(300);
    /* PA3/PC0 debounce */
    hold_active = 0; init(); osc_sidx=0; press(0,8,1); run(30); press(0,8,0); run(30); press(0,8,1); run(30); press(0,8,0); run(30);
    CHECK("PA3: bounce = one toggle (FILTER->V/DIV, not back)", osc_sidx==2);
    init(); osc_sidx=2; press(2,1,1); run(30); press(2,1,0); run(30); press(2,1,1); run(30); press(2,1,0); run(30);
    CHECK("PC0: bounce = one toggle (->FILTER)", osc_sidx==0);
    /* (d) + / - never move while frozen; UART u/d still do */
    hold_active = 1; init();
    tap(1,1,100); tap(1,2,100);
    CHECK("frozen: PB0 = OscAdjust(+1)", n_adj_p==1);
    CHECK("frozen: PB1 = OscAdjust(-1)", n_adj_m==1);
    CHECK("frozen: physical +/- never MvMove", n_move==0);
    clr(); ActIncrease(3); ActDecrease(3);
    CHECK("ActIncrease/ActDecrease frozen: adjust, no move", n_adj_p==1 && n_adj_m==1 && n_move==0);
    clr(); ActUp(1); ActDown(1);
    CHECK("UART u/d frozen still MvMove, no adjust", n_move==2 && n_adj_p==0 && n_adj_m==0);
    hold_active = 0; clr(); ActUp(1); ActDown(1);
    CHECK("UART u/d live still OscAdjust", n_adj_p==1 && n_adj_m==1 && n_move==0);
    /* hold-repeat on PB0 */
    hold_active = 0; init(); press(1,1,1); run(250); CHECK("PB0 first step immediate, no repeat before 300ms", n_adj_p==1);
    run(200); CHECK("PB0 repeats after 300ms hold", n_adj_p>=3);
    press(1,1,0); run(300);
    /* (f) AUTO */
    init(); tap(1,4,100); CHECK("PB2 -> ActAuto once", n_auto==1 && n_fs==0 && n_am==0);
    run(300); press(1,4,1); run(30); press(1,4,0); run(30); press(1,4,1); run(30); press(1,4,0); run(30);
    CHECK("PB2 debounced", n_auto==2);
    /* (g) FAST/SLOW */
    init(); tap(1,8,200); CHECK("PB3 short = ActToggleFastSlow", n_fs==1 && n_am==0);
    clr(); press(1,8,1); run(900); CHECK("PB3 not yet long at 900ms", n_am==0 && n_fs==0);
    run(200); CHECK("PB3 long >=1000ms = ActAutoMode", n_am==1);
    press(1,8,0); run(300); CHECK("PB3 long release does not toggle", n_fs==0 && n_am==1);
    clr(); tap(1,8,20); CHECK("PB3 <40ms glitch ignored", n_fs==0);
    clr(); press(2,0x2000,1); run(200); press(2,0x2000,0); run(100); CHECK("PC13 unchanged: short = toggle", n_fs==1);
    clr(); press(2,0x2000,1); run(1100); press(2,0x2000,0); run(100); CHECK("PC13 unchanged: long = auto mode", n_am==1 && n_fs==0);
    printf("eightbtn failures=%d\n", fails); return fails;
}
'''
(OUT/'t.c').write_text(c)
r = subprocess.run(['gcc','-O0','-Wall','-o',str(OUT/'t'),str(OUT/'t.c')],capture_output=True,text=True)
if r.returncode: print(r.stderr[:3000])
rr = subprocess.run([str(OUT/'t')],capture_output=True,text=True); print(rr.stdout)
cfail = 0
before = ROOT.parent/'builds/v63/main_before.c'
def slow(t): return t[t.index('SLOW_BEGIN ='):t.index('SLOW_END =')]
def body(t, n):
    t = re.sub(r'/\*.*?\*/', '', t, flags=re.S)
    m = re.search(r'^static\s+\w+\s+'+n+r'\([^;]*?\)\s*\{', t, re.M); p=t.index('{',m.start()); d=1; i=p+1
    while d: d += (t[i]=='{')-(t[i]=='}'); i+=1
    return t[m.start():i]
bt = before.read_text() if before.exists() else None
checks = [
 ('Slow block identical to v61', bt is None or slow(src)==slow(bt)),
 ('UART dispatch (UartCommand) unchanged vs v62', bt is None or body(src,'UartCommand')==body(bt,'UartCommand')),
]
if bt:
    for f in ['ActAuto','ActUp','ActDown','ActFreeze','ActToggleFastSlow','ActAutoMode','ActToggleMoveAxis','ActMoveCenter','OscAdjust','OscSetIndex','ButtonRepeat']:
        checks.append((f+' body unchanged', body(src,f)==body(bt,f)))
    # v79: ButtonJustPressed now takes the debounced level as an argument instead of reading the pin; everything else is unchanged
    ws = lambda x: re.sub(r'\s+', '', x)
    ob = body(bt,'ButtonJustPressed').replace('ButtonJustPressed(Button *btn, uint32_t now)', 'ButtonJustPressed(Button *btn, uint8_t raw, uint32_t now)').replace('uint8_t raw = (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET) ? 1 : 0;', '')
    checks.append(('ButtonJustPressed body unchanged except raw is now a parameter (v79)', ws(ob) == ws(body(src,'ButtonJustPressed'))))
    added = [l for l in __import__('difflib').unified_diff(bt.splitlines(), src.splitlines(), lineterm='', n=0) if l.startswith(('+','-')) and not l.startswith(('+++','---'))]
    checks.append(('diff vs v61 bounded (<=200 changed lines; grew with v72 header format, v73 V/DIV table, v74-v76 filter/grid, v79 debounce +~12): %d' % len(added), len(added)<=200))
    print('diff changed lines vs v61:', len(added))
def hb(t): return body(t,'HandleButtons')
mb = re.search(r"case 'm': \{.*?\n        \}", re.sub(r'/\*.*?\*/','',body(src,'UartCommand'),flags=re.S), re.S)[0] if False else None
um = re.sub(r'/\*.*?\*/','',body(src,'UartCommand'),flags=re.S)
mcase = um[um.index("case 'm':"):um.index("case 'z'")]
checks += [
 ('(d) PF3 handler calls ActFreeze (same fn as UART h), exactly once in HandleButtons', hb(src).count('ActFreeze()') == 1 and 'if (ButtonJustPressed(&btn_trig, now)) ActFreeze();' in hb(src)),
 ('(d) ActFreeze still defined and reachable from UART h', 'static void ActFreeze(void)' in src and "case 'h': case 'r': ActFreeze(); break;" in src),
 ('(d) ActFreeze callers: UART h + PF3 only (def+2 calls)', len(re.findall(r'\bActFreeze\s*\(', re.sub(r'/\*.*?\*/','',src,flags=re.S))) == 3),
 ('(d) only PC3 short calls ActToggleStable in HandleButtons', hb(src).count('ActToggleStable()') == 1),
 ('(e) ActToggleStable body == UART m bare toggle (flip + osc_dirty=1)', re.sub(r'\s+','',body(src,'ActToggleStable')) == 'staticvoidActToggleStable(void){osc_stable=(uint8_t)!osc_stable;osc_dirty=1;}' and 'else{osc_stable=(uint8_t)!osc_stable;}osc_dirty=1;break;' in re.sub(r'\s+','',mcase)),
 ('UartCommand m case unchanged vs v62', mcase == (lambda b: b[b.index("case 'm':"):b.index("case 'z'")])(re.sub(r'/\*.*?\*/','',body(bt,'UartCommand'),flags=re.S)) if bt else True),
 ('PC3 long-press block unchanged', 'if (hold_active) ActSelectSetting(strcmp(OscSetName(), "C1") == 0 ? "C2" : "C1");' in src and 'BTN_CURSOR_LONG_MS' in hb(src)),
]
checks += [('PB2 AUTO -> ActAuto', 'ButtonJustPressed(&btn_hold, now)) ActAuto();' in src),
 ('PB0/PB1 -> ActIncrease/ActDecrease', 'ActIncrease(n);' in src and 'ActDecrease(n);' in src),
 ('ActIncrease/Decrease have no hold_active branch', 'hold_active' not in body(src,'ActIncrease')+body(src,'ActDecrease') and 'MvMove' not in body(src,'ActIncrease')+body(src,'ActDecrease')),
 ('pins PA3/PC0/PC3/PF3 defined', all(t in src for t in ['BTN_VDIV_PORT   GPIOA','BTN_VDIV_PIN    GPIO_PIN_3','BTN_TDIV_PIN    GPIO_PIN_0','BTN_FILT_PIN    GPIO_PIN_3','BTN_TRIG_PORT   GPIOF','BTN_TRIG_PIN    GPIO_PIN_3'])),
 ('new pins init pull-up input; GPIOA/C/F clocks', 'btn_init.Pin = BTN_VDIV_PIN' in src and '__HAL_RCC_GPIOF_CLK_ENABLE()' in src and 'GPIO_PULLUP' in src),
 ('UART s/v/t/h/z/F/A keys unchanged', all(t in src for t in ["case 's': OscSetIndex(osc_sidx + 1); break;","case 'F': ActToggleFastSlow(); break;","case 'A': ActAutoMode(); break;","case 'z': case 'Z': ActAuto(); break;","case 'v': OscSetIndex(2);","case 't': OscSetIndex(3);"])),
]
for n, ok in checks:
    print(('PASS ' if ok else 'FAIL ')+n); cfail += (not ok)
print('contract failures=%d' % cfail)
print('eightbtn total failures=%d' % (rr.returncode + cfail))
import sys
sys.exit(1 if (rr.returncode + cfail) else 0)
