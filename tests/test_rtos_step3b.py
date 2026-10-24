"""(v77) FreeRTOS step 3b contracts: button SAMPLER task (vInputTask -> ButtonSample) + event queue + DISPATCH in vMainTask (HandleButtons).
Usage: python3 test_rtos_step3b.py [MAIN_C]      (default Core/Src/main.c; pass builds/.../main_before.c as a negative control: it must FAIL)
(a) queue/task creation order in main(), (b) sampler purity, (c) static action mapping, (d) behaviour harness: scripted pin sequences give the same
Act* call sequence as the OLD combined HandleButtons from builds/v77-rtos-step3b/main_before.c."""
import re, subprocess, sys, tempfile, pathlib
import btn_harness as bh
ROOT = pathlib.Path(__file__).resolve().parents[1]
MAIN = pathlib.Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / 'Core/Src/main.c'
OLD = ROOT.parent / 'builds/v77-rtos-step3b/main_before.c'
raw = MAIN.read_text(encoding='utf-8')
nc = bh.strip_comments(raw)
nc = re.sub(r'//[^\n]*', '', nc)
fails = 0
def chk(name, ok, extra=''):
    global fails
    print(('PASS ' if ok else 'FAIL ') + name + (' ' + extra if extra else '')); fails += (not ok)
def fbody(name, text=None):
    try: return bh.func_text(text or nc, name)
    except AssertionError: return None

mn = fbody('main') or ''
def ix(s): return mn.index(s) if s in mn else -1

# ---------------- (a) creation order ----------------
a = [ix('xOscMutex = xSemaphoreCreateMutex();'), ix('xBtnQ = xQueueCreate('), ix('xTaskCreate(vInputTask'), ix('xTaskCreate(vMainTask'), ix('vTaskStartScheduler();')]
chk('(a) main(): xOscMutex < xQueueCreate < xTaskCreate(vInputTask) < xTaskCreate(vMainTask) < vTaskStartScheduler', all(x >= 0 for x in a) and a == sorted(a) and len(set(a)) == 5, str(a))
chk('(a) queue / vInputTask created nowhere else (never earlier in main())', nc.count('xQueueCreate(') == 1 and nc.count('xTaskCreate(vInputTask') == 1)
chk('(a) queue depth 16 of BtnEv', 'xQueueCreate(16, sizeof(BtnEv))' in mn)
chk('(a) vInputTask: priority idle+2, stack 256 words', re.search(r'xTaskCreate\(vInputTask, "Input", 256, NULL, tskIDLE_PRIORITY \+ 2, NULL\)', mn) is not None)
chk('(a) vMainTask stays idle+1 / 2048 words with a checked result', re.search(r'xTaskCreate\(vMainTask, "Main", 2048, NULL, tskIDLE_PRIORITY \+ 1, NULL\) != pdPASS\) Error_Handler\(\)', mn) is not None)
chk('(a) fallback: queue NULL / task creation failure handled (btn_task_ok flag, NULL queue -> direct dispatch)',
    'btn_task_ok = (xTaskCreate(vInputTask' in mn and 'if (xBtnQ != NULL)' in mn and 'if (xBtnQ == NULL)' in (fbody('BtnEmit') or '') and 'if (!btn_task_ok) ButtonSample(now);' in (fbody('HandleButtons') or ''))
chk('(a) queue.h included', '#include "queue.h"' in raw)
cfg = (ROOT / 'Core/Inc/FreeRTOSConfig.h').read_text()
hs = re.search(r'configTOTAL_HEAP_SIZE\s+\(\s*\(\s*size_t\s*\)\s*\(\s*(\d+)\s*\*\s*1024', cfg)
chk('(a) configTOTAL_HEAP_SIZE >= 24 KB', hs is not None and int(hs.group(1)) >= 24)
chk('(a) INCLUDE_vTaskDelayUntil enabled', re.search(r'#define INCLUDE_vTaskDelayUntil\s+1', cfg) is not None)
it = fbody('vInputTask') or ''
chk('(a) vInputTask = vTaskDelayUntil(5 ms) loop calling only ButtonSample(HAL_GetTick())',
    'vTaskDelayUntil(&last, pdMS_TO_TICKS(5))' in it and 'ButtonSample(HAL_GetTick())' in it and 'xTaskGetTickCount()' in it
    and set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', it)) - {'vInputTask', 'for', 'void'} == {'xTaskGetTickCount', 'ButtonSample', 'HAL_GetTick', 'vTaskDelayUntil', 'pdMS_TO_TICKS'}, str(sorted(set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', it)))))

# ---------------- (b) sampler purity ----------------
bs = fbody('ButtonSample')
chk('(b) ButtonSample exists', bs is not None)
bs = bs or ''
calls = set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', bs)) - {'if', 'ButtonSample', 'for', 'while', 'sizeof'}
allowed = {'ButtonRepeat', 'ButtonRaw', 'ButtonJustPressed', 'BtnEmit', 'HAL_GPIO_ReadPin', 'Debounce'}   # v79: Debounce added
chk('(b) sampler calls only %s' % sorted(allowed), bool(calls) and calls <= allowed, str(sorted(calls - allowed)))
bad = re.findall(r'\b(Act\w*|ILI9341\w*|HAL_SPI\w*|HAL_ADC\w*|Adc\w*|OSC_\w+|Osc\w*|Uart\w*|printf|snprintf|Draw\w*|Hdr\w*|xSemaphore\w*|hold_active|osc_\w+|Mv\w*)\b', bs)
chk('(b) sampler has no Act*/LCD/SPI/ADC/OSC_LOCK/UART/Draw/osc state references', not bad, str(bad))
be = fbody('BtnEmit') or ''
chk('(b) BtnEmit only touches the queue (xQueueSend, drop counter) + the NULL-queue fallback BtnDispatchEv',
    set(re.findall(r'\b([A-Za-z_]\w*)\s*\(', be)) - {'if', 'BtnEmit'} <= {'xQueueSend', 'BtnDispatchEv'} and 'xQueueSend(xBtnQ, &e, 0)' in be and 'dbg_btn_q_drop++' in be)
# button state is private to the sampler (+ the helper functions + init in main)
own = {'ButtonSample', 'ButtonRepeat', 'ButtonRaw', 'ButtonJustPressed', 'ButtonLongPress', 'Debounce', 'main'}
toks = r'\b(man_db|btn_up|btn_down|btn_hold|btn_set|btn_vdiv|btn_tdiv|btn_filt|btn_trig|cu_prev|cu_long|cu_t0|fs_prev|fs_long|fs_t0|man_prev|man_long|man_t0)\b'
outside = []
for m in re.finditer(r'^(?:static\s+)?[A-Za-z_][\w ]*?[\s\*](\w+)\([^;{}]*\)\s*\{', nc, re.M):
    q = m.end() - 1; dd = 1; k = q + 1
    while dd:
        dd += (nc[k] == '{') - (nc[k] == '}'); k += 1
    body = nc[q:k]
    if m.group(1) not in own and re.search(toks, body): outside.append(m.group(1))
chk('(b) btn_* structs and long-press statics are used by no function except sampler helpers/main-init', not outside, str(outside))
chk('(b) HandleButtons (dispatch) does not read the pins', 'HAL_GPIO_ReadPin' not in (fbody('HandleButtons') or '') and 'ButtonRaw' not in (fbody('HandleButtons') or ''))

# ---------------- (c) action mapping ----------------
if OLD.exists():
    old_nc = re.sub(r'//[^\n]*', '', bh.strip_comments(OLD.read_text(encoding='utf-8')))
    old_hb = bh.func_text(old_nc, 'HandleButtons')
    old_acts = set(re.findall(r'\b(Act\w+)\(', old_hb)) - {'ActToggleTrigEdge'}
    disp = fbody('BtnDispatchEv') or ''
    new_acts = set(re.findall(r'\b(Act\w+)\(', disp))
    chk('(c) dispatch calls exactly the Act* set the old HandleButtons called', old_acts == new_acts and len(new_acts) >= 8, str(sorted(old_acts ^ new_acts)))
    exp = {'BE_UP': 'ActIncrease(e->arg)', 'BE_DOWN': 'ActDecrease(e->arg)', 'BE_AUTO': 'ActAuto()', 'BE_FREEZE': 'ActFreeze()', 'BE_STABLE': 'ActToggleStable()',
           'BE_AUTOMODE': 'ActAutoMode()', 'BE_FASTSLOW': 'ActToggleFastSlow()'}
    chk('(c) dispatch case -> Act mapping', all(re.search(r'case %s:\s*%s;' % (k, re.escape(v)), disp) for k, v in exp.items()))
    chk('(c) V/DIV, FILTER and C1/C2 selections keep their name toggles and are evaluated at dispatch (OscSetName/hold_active in dispatch, not in sampler)',
        'ActSelectSetting(strcmp(OscSetName(), "V/DIV") == 0 ? "T/DIV" : "V/DIV")' in disp and 'ActSelectSetting(strcmp(OscSetName(), "FILTER") == 0 ? "TRIG" : "FILTER")' in disp
        and 'case BE_CURSOR_LONG: if (hold_active) ActSelectSetting(strcmp(OscSetName(), "C1") == 0 ? "C2" : "C1");' in disp)
    emits = re.findall(r'BtnEmit\((BE_\w+)', bs)
    want = ['BE_UP', 'BE_DOWN', 'BE_AUTO', 'BE_VDIV', 'BE_TDIV', 'BE_FREEZE', 'BE_CURSOR_LONG', 'BE_STABLE', 'BE_AUTOMODE', 'BE_FASTSLOW', 'BE_AUTOMODE', 'BE_FASTSLOW']
    chk('(c) sampler emits each event once per old Act* call site (PB3 and PC13 both emit AUTOMODE/FASTSLOW)', sorted(emits) == sorted(want), str(emits))
    chk('(c) old and new SLOW blocks byte-identical', raw[raw.index('SLOW_BEGIN ='):raw.index('SLOW_END =')] == OLD.read_text(encoding='utf-8')[OLD.read_text(encoding='utf-8').index('SLOW_BEGIN ='):OLD.read_text(encoding='utf-8').index('SLOW_END =')])
else:
    print('SKIP (c) and (d): builds/v77-rtos-step3b/main_before.c not present in this checkout')

# ---------------- (d) behaviour harness ----------------
C_COMMON = r'''
#include <stdio.h>
#include <stdint.h>
#include <string.h>
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
%(button_t)s
static Button btn_up, btn_down, btn_hold, btn_set, btn_vdiv, btn_tdiv, btn_filt, btn_trig;
static uint8_t hold_active = 0; static int osc_sidx = 0;
static const char *nm_live[] = {"FILTER","TRIG","V/DIV","T/DIV"}; static const char *nm_hold[] = {"C1","C2","V/DIV"};
static const char *OscSetName(void){ return hold_active ? nm_hold[osc_sidx %% 3] : nm_live[osc_sidx %% 4]; }
static uint32_t T = 0;
static void L(const char *s, int n){ printf("%%lu %%s %%d | s%%d h%%d\n", (unsigned long)T, s, n, osc_sidx, hold_active); }
static void ActIncrease(int n){ L("INC", n); }
static void ActDecrease(int n){ L("DEC", n); }
static void ActAuto(void){ L("AUTO", 0); }
static void ActFreeze(void){ L("FREEZE", 0); hold_active = !hold_active; osc_sidx = 0; }
static void ActToggleStable(void){ L("STABLE", 0); }
static void ActAutoMode(void){ L("AUTOMODE", 0); }
static void ActToggleFastSlow(void){ L("FASTSLOW", 0); }
static void ActToggleTrigEdge(void){ L("EDGE", 0); }
static void ActSelectSetting(const char *name){
    int n = hold_active ? 3 : 4; const char **a = hold_active ? nm_hold : nm_live;
    L(name, 0);
    for (int i = 0; i < n; i++) if (strcmp(a[i], name) == 0) osc_sidx = i;
}
static void MvSync(void){ }
static uint32_t pins = 0;   /* bit0 up 1 down 2 hold 3 vdiv 4 tdiv 5 filt 6 trig 7 set 8 man (1 = pressed) */
static int HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t pin){
    int bit = -1;
    if (p==&PB && pin==1) bit=0; else if (p==&PB && pin==2) bit=1; else if (p==&PB && pin==4) bit=2; else if (p==&PA && pin==8) bit=3;
    else if (p==&PC && pin==1) bit=4; else if (p==&PC && pin==8) bit=5; else if (p==&PF && pin==8) bit=6; else if (p==&PB && pin==8) bit=7; else if (p==&PC && pin==8192) bit=8;
    if (bit < 0) { printf("BAD PIN\n"); return 0; }
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
    pins = 0; osc_sidx = 0;
}
static uint32_t CAD = 5; static int MODE = 0;   /* v79: MODE 0 = bounce-free script (pulses < BTN_STABLE_MS and random chatter are filtered now: see test_debounce.py) */
static void hold(uint32_t mask, uint32_t ms){ pins = mask; for (uint32_t i = 0; i < ms; i += CAD){ T += CAD; HandleButtons(T); } }
static void tap(uint32_t mask, uint32_t press_ms){ hold(mask, press_ms); hold(0, 400); }
enum { UP=1, DN=2, HLD=4, VD=8, TD=16, FT=32, TR=64, ST=128, MAN=256 };
static uint32_t rng = 12345;
static uint32_t rnd(void){ rng = rng * 1664525u + 1013904223u; return rng >> 8; }
static void scenarios(int hold0){
    static const uint32_t all[] = {UP,DN,HLD,VD,TD,FT,TR,ST,MAN};
    printf("== cad %%lu hold0 %%d\n", (unsigned long)CAD, hold0);
    init(); hold_active = hold0; T = 100000;
    for (int i = 0; i < 9; i++) { tap(all[i], 100); if (MODE) tap(all[i], 30); tap(all[i], 1100); tap(all[i], 1700); }   /* short, bounce-short (MODE 1 only), ST/MAN long, PC3 long */
    hold(UP, 2300); hold(0, 300); hold(DN, 2300); hold(0, 300);                                              /* hold-repeat + acceleration */
    hold(UP|DN, 700); hold(0, 300);
    hold(FT|TR, 1700); hold(0, 300);                                                                         /* FREEZE then cursor-long in one pass */
    hold(FT, 1700); hold(0, 300); hold(FT, 1700); hold(0, 300);                                              /* cursor long with hold_active as left by the above */
    hold(0x1FF, 3000); hold(0, 500);
    if (MODE) for (int i = 0; i < 4000; i++) hold(rnd() & 0x1FF, 5 * (1 + (rnd() %% 12)));                  /* chatter (MODE 1 only) */
    hold(0, 500);
}
int main(int argc, char **argv){
    (void)argv; MODE = argc > 1;
    uint32_t cads[] = {5, 33, 80};
    for (int c = 0; c < 3; c++) for (int h = 0; h < 2; h++) { CAD = cads[c]; rng = 12345; scenarios(h); }
    return 0;
}
'''

def harness_source(text):
    ncx = re.sub(r'//[^\n]*', '', bh.strip_comments(text))
    names = (['Debounce'] if 'Debounce(' in ncx else []) + ['ButtonJustPressed', 'ButtonRepeat', 'ButtonRaw', 'ButtonLongPress'] + bh.split_names(text) + ['HandleButtons']
    defs = '\n'.join(re.search(r'^#define ' + n + r'\b[^\n]*', ncx, re.M)[0] for n in sorted(set(re.findall(r'^#define (BTN_\w+)', ncx, re.M))))
    dm = re.search(r'typedef struct \{[^}]*\} Debouncer;', ncx)
    btn_t = (dm[0] + '\n' if dm else '') + ncx[ncx.index('typedef struct {\n    GPIO_TypeDef *port;'):ncx.index('} Button;') + len('} Button;')]
    return C_COMMON % dict(defs=defs, button_t=btn_t, prelude=bh.prelude(text), funcs='\n'.join(bh.func_text(ncx, n) for n in names))

def run_variant(text, tag, mode=0):
    d = pathlib.Path(tempfile.mkdtemp(prefix='osc3b_' + tag))
    (d / 'h.c').write_text(harness_source(text))
    r = subprocess.run(['gcc', '-O1', '-Wall', '-Wno-unused-function', '-Wno-unused-variable', '-o', str(d / 'h'), str(d / 'h.c')], capture_output=True, text=True)
    if r.returncode: print(r.stderr[:2000]); return None
    return subprocess.run([str(d / 'h')] + (['x'] if mode else []), capture_output=True, text=True).stdout

if OLD.exists():
    try:
        old_out = run_variant(OLD.read_text(encoding='utf-8'), 'old')
        new_out = run_variant(raw, 'new')
    except (AssertionError, ValueError, AttributeError) as ex:
        old_out = new_out = None; print('harness extraction failed:', repr(ex))
    ok = old_out is not None and new_out is not None
    chk('(d) harness compiles for old and new sources', ok)
    if ok:
        ol, nl = old_out.splitlines(), new_out.splitlines()
        kinds = {l.split()[1] for l in ol if l and l[0].isdigit()}
        chk('(d) reference run is rich (%d events, kinds: %s)' % (len(ol), ','.join(sorted(kinds))), len(ol) > 300 and {'INC', 'DEC', 'AUTO', 'FREEZE', 'STABLE', 'AUTOMODE', 'FASTSLOW', 'C1', 'C2', 'V/DIV', 'T/DIV', 'FILTER', 'TRIG'} <= kinds)
        # v79: the input debouncer adds up to BTN_STABLE_MS (+ one sampling period) of latency, so for a bounce-free script compare the Act* SEQUENCE (name, arg, selection state)
        # exactly and each event time with a tolerance of 25 ms + the cadence. Random-chatter equivalence (expectation 'clean count') lives in test_debounce.py.
        def parse(lines):
            ev, cad = [], 0
            for l in lines:
                if l.startswith('== cad'): cad = int(l.split()[2]); continue
                t, rest = l.split(' ', 1); ev.append((int(t), rest, cad))
            return ev
        po, pn = parse(ol), parse(nl)
        seq_ok = [(r, c) for _, r, c in po] == [(r, c) for _, r, c in pn]
        first = next((i for i, (x, y) in enumerate(zip(po, pn)) if x[1:] != y[1:]), None)
        chk('(d) bounce-free script: new sampler+dispatch gives the SAME Act* call SEQUENCE as the old HandleButtons (5/33/80 ms cadence, hold_active 0/1, %d events)' % len(po),
            seq_ok, '' if seq_ok else 'first diff at %s: old=%r new=%r (len %d vs %d)' % (first, po[first] if first is not None and first < len(po) else None, pn[first] if first is not None and first < len(pn) else None, len(po), len(pn)))
        if seq_ok:
            worst = max(n[0] - o[0] for o, n in zip(po, pn)); best = min(n[0] - o[0] for o, n in zip(po, pn))
            bad = [(o, n) for o, n in zip(po, pn) if not (0 <= n[0] - o[0] <= 25 + o[2])]
            chk('(d) every event is delayed by 0..25 ms + cadence (debounce latency; observed %d..%d ms)' % (best, worst), not bad, str(bad[:2]))
print('rtos_step3b failures=%d' % fails)
sys.exit(1 if fails else 0)
