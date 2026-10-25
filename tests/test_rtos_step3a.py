"""(v66b) FreeRTOS refactor step 3a contracts: mutex around shared osc state; (v95) the OscTask + InputTask split under the app lock.
Pure text checks on the app source (app_src.py: modules + main() with AppInit/AppStart inlined)."""
from app_src import source as app_source   # the app modules as one text (see app_src.py)
import re, pathlib, sys
import _unlock
ROOT = pathlib.Path(__file__).resolve().parents[1]
raw = app_source()
_bp = ROOT.parent/'builds/v66-rtos-step3a/main_before.c'   # local build archive (not in the git repo)
bef = _bp.read_text(encoding='utf-8') if _bp.exists() else None
it  = (ROOT/'Core/Src/stm32f7xx_it.c').read_text()
amh = (ROOT/'Core/Inc/app/app_main.h').read_text()        # task stacks/priorities + the APP_LOCK macros
fails = 0
def chk(name, ok, extra=''):
    global fails
    print(('PASS ' if ok else 'FAIL ') + name + (' ' + extra if extra else '')); fails += (not ok)

def strip(t):   # comments/strings -> spaces (keeps offsets)
    t = re.sub(r'/\*.*?\*/', lambda m: re.sub(r'[^\n]', ' ', m.group(0)), t, flags=re.S)
    t = re.sub(r'//[^\n]*', lambda m: ' '*len(m.group(0)), t)
    return re.sub(r'"(?:\\.|[^"\\\n])*"', lambda m: '"'+' '*(len(m.group(0))-2)+'"', t)
src = strip(raw)
VARS = ['osc_meas', 'osc_sidx', 'hold_active', 'osc_dirty', 'osc_stable', 'osc_auto_tb']
SCALARS = VARS[1:]
KW = {'if', 'for', 'while', 'switch', 'return', 'sizeof'}

def functions(t):
    out = {}
    for m in re.finditer(r'^[A-Za-z_][\w \*]*?\b(\w+)\s*\([^;{}]*\)\s*\{', t, flags=re.M):
        if m.group(1) in KW: continue
        p = m.end()-1; d = 1; i = p+1
        while d:
            d += {'{': 1, '}': -1}.get(t[i], 0); i += 1
        out[m.group(1)] = (p, i)   # keeps the LAST definition with that name (alt #else stubs)
    return out
fn = functions(src)
def body(name): p, e = fn[name]; return src[p:e]
def lineno(pos): return src.count('\n', 0, pos)+1


# (f) PRE-SCHEDULER SAFETY: nothing that can block on the mutex may run before vTaskStartScheduler().
# Strict text-order check on main(): no Take/Give/OSC_* token AND no call (direct or transitive) to any function that locks.
LOCKTOK_F = re.compile(r'\b(OSC_LOCK|OSC_UNLOCK|OSC_SET|OscMeasGet|xSemaphoreTake|xSemaphoreGive|xQueue\w*|ulTaskNotifyTake)\b')
def presched_violations(text):
    st = strip(text); f = functions(st)
    if 'main' not in f or 'vTaskStartScheduler' not in st[f['main'][0]:f['main'][1]]:
        return [('main', 'vTaskStartScheduler() not found in main()')]
    bd = lambda n: st[f[n][0]:f[n][1]]
    L = {n for n in f if n != 'main' and LOCKTOK_F.search(bd(n))}
    while True:
        add = {n for n in f if n not in L and n != 'main' and any(re.search(r'\b%s\s*\(' % l, bd(n)) for l in L)}
        if not add: break
        L |= add
    m = bd('main'); pre = m[:m.index('vTaskStartScheduler')]; base = f['main'][0]
    v = [(st.count('\n', 0, base+x.start())+1, x.group(0)) for x in LOCKTOK_F.finditer(pre)]
    v += [(st.count('\n', 0, base+x.start())+1, 'call to locking fn ' + x.group(1)) for x in re.finditer(r'\b(\w+)\s*\(', pre) if x.group(1) in L]
    return v
if len(sys.argv) == 3 and sys.argv[1] == '--presched':      # negative-control mode: python3 test_rtos_step3a.py --presched FILE
    vv = presched_violations(pathlib.Path(sys.argv[2]).read_text(encoding='utf-8'))
    print('presched violations in %s: %s' % (sys.argv[2], vv)); sys.exit(1 if vv else 0)

# (0) Unlocking must preserve the non-locking behavior; display-only formatting
# changes made after Step 3a are intentionally allowed.
unlocked = _unlock.unlock(raw)
unlocked_src = strip(unlocked)
unlocked_fn = functions(unlocked_src)
for fn_name in ('OscMeasureFrame', 'OscProcessFrame', 'AdcCaptureFrame', 'OscAutoTimebase', 'SlowCaptureFrame', 'HandleButtons'):
    p, e = unlocked_fn[fn_name]
    chk('(0) unlock keeps behavior function parseable: ' + fn_name, e > p and len(unlocked_src[p:e]) > 10)

# (a) mutex creation
mn = src[src.index('int main(void)'):]
chk('(a) static SemaphoreHandle_t xOscMutex declared', re.search(r'^static SemaphoreHandle_t xOscMutex;', src, re.M) is not None)
chk('(a) xOscMutex = xSemaphoreCreateMutex() in main() (AppStart)', 'xOscMutex = xSemaphoreCreateMutex();' in mn)
chk('(a) plain mutex (no recursive variant used anywhere)', 'Recursive' not in src)
chk('(a) NULL check -> Error_Handler() (both mutexes)', re.search(r'xOscMutex = xSemaphoreCreateMutex\(\);[^\n]*\n\s*xAppLock  = xSemaphoreCreateMutex\(\);[^\n]*\n\s*if \(xOscMutex == NULL \|\| xAppLock == NULL\) Error_Handler\(\);', mn) is not None)
ix = lambda s: mn.index(s)
chk('(a) created before xTaskCreate', ix('xSemaphoreCreateMutex') < ix('xTaskCreate(OscTask'))
chk('(a) created after all init that must remain single-threaded and immediately before xTaskCreate',
    ix('xSemaphoreCreateMutex') < ix('xTaskCreate(OscTask') and ix('xSemaphoreCreateMutex') > ix('HdrInvalidate();'))
chk('(a) created after HAL_Init + SystemClock_Config (heap/clock ready)', ix('SystemClock_Config();') < ix('xSemaphoreCreateMutex'))
chk('(a) semphr.h included', '#include "semphr.h"' in raw)
chk('(a) xTaskCreate called exactly twice (OscTask, InputTask)', src.count('xTaskCreate(') == 2)

# (b) every write to a protected variable is locked
slow_a, slow_b = raw.index('SLOW_BEGIN ='), raw.index('SLOW_END =')
def in_slow(pos): return slow_a <= pos < slow_b
owner_of = lambda pos: next((n for n, (p, e) in fn.items() if p <= pos < e), None)
wr_scalar = re.compile(r'(?<![\w.>])(%s)\s*(?:=(?!=)|\+\+|--|[-+*/|&^]=)|(?:\+\+|--)\s*(%s)\b' % ('|'.join(SCALARS), '|'.join(SCALARS)))
unprot, n_ok, n_slow = [], 0, []
for m in wr_scalar.finditer(src):
    pos = m.start()
    ln_start = src.rfind('\n', 0, pos)+1; line = src[ln_start:src.index('\n', pos)]
    if owner_of(pos) is None: continue                                   # file-scope declarations/initialisers
    if in_slow(pos): n_slow.append((lineno(pos), owner_of(pos))); continue
    unprot.append((lineno(pos), line.strip()[:80]))   # a raw assignment survived outside OSC_SET
n_ok = len([m for m in re.finditer(r'\bOSC_SET\(\s*(?:%s)\b' % '|'.join(SCALARS), src) if owner_of(m.start()) and not in_slow(m.start())])
chk('(b) every non-Slow scalar write (osc_sidx/hold_active/osc_dirty/osc_stable/osc_auto_tb) goes through OSC_SET and no raw assignment survives (%d OSC_SET sites)' % n_ok, not unprot and n_ok >= 40, str(unprot))
# osc_meas field/struct writes must sit between OSC_LOCK and OSC_UNLOCK in the same function
wr_meas = re.compile(r'(?:osc_meas\s*\.\s*\w+|\(\(OscMeasure \*\)&osc_meas\)->\w+|(?<![\w.])osc_meas)\s*(?:=(?!=)|[-+*/]=)')
bad_meas, n_meas = [], 0
for m in wr_meas.finditer(src):
    pos = m.start(); o = owner_of(pos)
    if o is None or re.match(r'\s*volatile', src[src.rfind('\n', 0, pos)+1:pos]): continue
    if in_slow(pos): n_slow.append((lineno(pos), o)); continue
    b0 = fn[o][0]; seg = src[b0:pos]
    held = len(re.findall(r'\bOSC_LOCK\(\)', seg)) - len(re.findall(r'\bOSC_UNLOCK\(\)', seg))
    n_meas += 1
    if held != 1: bad_meas.append((lineno(pos), o))
chk('(b) every non-Slow osc_meas write is inside OSC_LOCK..OSC_UNLOCK (%d sites)' % n_meas, not bad_meas and n_meas >= 15, str(bad_meas))
# osc_meas reads that need a consistent multi-field snapshot use OscMeasGet (UartPanel + DrawTopPanel)
for f in ('UartPanel', 'DrawTopPanel'):
    chk('(b) %s reads osc_meas only through a OscMeasGet snapshot' % f, 'OscMeasGet(' in body(f) and 'osc_meas' not in body(f))
# Slow block: its writes live in SlowCaptureFrame (task context) and the only call site is locked
slow_owners = {o for _, o in n_slow}
chk('(b) Slow-block writes to protected vars are only in SlowCaptureFrame: ' + str(sorted(slow_owners)), slow_owners <= {'SlowCaptureFrame'})
calls = [m.start() for m in re.finditer(r'\bSlowCaptureFrame\s*\(\s*\)\s*;', src) if not in_slow(m.start()) and owner_of(m.start()) is not None]
chk('(b) SlowCaptureFrame has exactly 1 call site and it is wrapped OSC_LOCK(); ..; OSC_UNLOCK();',
    len(calls) == 1 and re.search(r'OSC_LOCK\(\);\s*SlowCaptureFrame\(\);\s*OSC_UNLOCK\(\);', src[calls[0]-20:calls[0]+60]) is not None)

# (c) no nested lock (plain mutex would deadlock even in one task)
LOCKTOK = re.compile(r'\b(OSC_LOCK|OSC_UNLOCK|OSC_SET|OscMeasGet|xSemaphoreTake|xSemaphoreGive)\b')
def lockers():
    s = {n for n in fn if n not in ('OscMeasGet',) and LOCKTOK.search(body(n))} | {'OscMeasGet'}
    s -= {'main'}
    while True:                                   # transitive
        add = {n for n in fn if n not in s and n != 'main' and any(re.search(r'\b%s\s*\(' % l, body(n)) for l in s)}
        if not add: return s
        s |= add
L = lockers()
nest = []; unbalanced = []; nlock_fn = 0
for n in fn:
    if n in ('main', 'OscMeasGet') : pass
    b = body(n); toks = list(LOCKTOK.finditer(b))
    if not toks: continue
    nlock_fn += 1; held = False; heldpos = 0
    # walk statements in order: lock tokens and calls to locker functions
    ev = sorted([(m.start(), m.group(1)) for m in toks] + [(m.start(), 'CALL:'+m.group(1)) for m in re.finditer(r'\b(\w+)\s*\(', b) if m.group(1) in L and m.group(1) not in ('OscMeasGet',) and not LOCKTOK.fullmatch(m.group(1))])
    for pos, t in ev:
        if t == 'OSC_LOCK' or t == 'xSemaphoreTake':
            if held: nest.append((n, 'lock while held'))
            held = True
        elif t == 'OSC_UNLOCK' or t == 'xSemaphoreGive':
            if not held: unbalanced.append((n, 'unlock without lock'))
            held = False
        elif t in ('OSC_SET', 'OscMeasGet'):
            if held: nest.append((n, t + ' while held'))
        elif t.startswith('CALL:'):
            if held: nest.append((n, 'calls locking function %s while held' % t[5:]))
    if held: unbalanced.append((n, 'returns/ends holding lock'))
    # OSC_SET argument must not contain locks/calls to lockers
for m in re.finditer(r'OSC_SET\(([^;]*?)\);', src):
    if owner_of(m.start()) and re.search(r'\b(%s)\b' % '|'.join(map(re.escape, L | {'OSC_LOCK', 'OSC_SET'})), m.group(1)): nest.append((owner_of(m.start()), 'OSC_SET arg calls locker'))
# no return/goto between a lock and its unlock
for n in fn:
    b = body(n)
    for m in re.finditer(r'OSC_LOCK\(\);(.*?)OSC_UNLOCK\(\);', b, flags=re.S):
        if re.search(r'\b(return|goto|break|continue)\b', m.group(1)): nest.append((n, 'early exit inside critical section'))
chk('(c) no nested lock / no call to a locking function while holding the lock / no early exit in a critical section (%d functions use the lock)' % nlock_fn, not nest, str(nest))
chk('(c) every function releases what it takes', not unbalanced, str(unbalanced))
chk('(c) OSC_LOCK/OSC_UNLOCK counts equal (excluding macro definitions)', len(re.findall(r'\bOSC_LOCK\(\)', src)) == len(re.findall(r'\bOSC_UNLOCK\(\)', src)))
sec = [m.group(1) for m in re.finditer(r'OSC_LOCK\(\);(.*?)OSC_UNLOCK\(\);', src, flags=re.S)]
chk('(c) critical sections are short: no UART/LCD/HAL/delay call inside any of them', not any(re.search(r'\b(Uart\w*|ILI9341_\w+|HAL_\w+|vTaskDelay|snprintf|Draw\w+)\s*\(', s_) for s_ in sec))
chk('(c) a non-recursive mutex does not need to be recursive: SlowCaptureFrame callees (inside its locked call) never lock',
    not any(re.search(r'\b%s\s*\(' % l, src[slow_a:slow_b]) for l in L) and not LOCKTOK.search(src[slow_a:slow_b]))

# (d) ISR safety
# ADC_IRQHandler / DMA2_Stream0_IRQHandler are CubeMX code (stm32f7xx_it.c) that only call HAL; HAL calls back into these from the ISR:
isr_roots = ['OscSlowTick', 'HAL_UART_RxCpltCallback', 'HAL_UART_ErrorCallback', 'USART3_IRQHandler', 'HAL_ADC_ConvCpltCallback', 'HAL_ADC_ErrorCallback']
seen, todo = set(), list(isr_roots)
while todo:
    n = todo.pop()
    if n in seen or n not in fn: continue
    seen.add(n)
    for m in re.finditer(r'\b(\w+)\s*\(', body(n)):
        if m.group(1) in fn: todo.append(m.group(1))
bad = [n for n in seen if LOCKTOK.search(body(n)) or re.search(r'\bxOscMutex\b|xSemaphore', body(n))]
chk('(d) no lock/semaphore token in OscSlowTick or any ISR-reachable function in main.c (checked %d: %s)' % (len(seen), ','.join(sorted(seen))), not bad, str(bad))
cross = [(n, v) for n in seen for v in VARS if re.search(r'\b%s\b' % v, body(n))]
chk('(d) OscSlowTick / ISR callees do not touch ANY protected variable (so nothing needs ISR-safe locking)', not cross, str(cross))
chk('(d) OscSlowTick defined in the Slow block, has no semaphore text', 'OscSlowTick' in src[slow_a:slow_b])
st = it[it.index('void SysTick_Handler(void)'):]; st = st[:st.index('\n}\n')]
chk('(d) SysTick_Handler has no semaphore/mutex call', 'xSemaphore' not in st and 'OSC_' not in st and 'xOscMutex' not in st)
chk('(d) no semaphore call anywhere in stm32f7xx_it.c', 'xSemaphore' not in it and 'xOscMutex' not in it)
chk('(d) no FromISR variants and no semaphore use outside the OSC_* / APP_* macros', 'FromISR' not in src + amh and len(re.findall(r'xSemaphore(?:Take|Give)', src + strip(amh))) == 4)

# (e) Slow block untouched, unrelated checks
sl = lambda t: t[t.index('SLOW_BEGIN ='):t.index('SLOW_END =')]
if bef is not None: chk('(e) Slow block byte-identical to v65 (%d chars)' % len(sl(bef)), sl(raw) == sl(bef))
else: print('SKIP (e) Slow block vs v65 (builds/ archive not present in this checkout)')
chk('(e) OscTask keeps the old task: stack 2048 words / prio idle+1', '#define OSC_TASK_STACK    2048' in amh and '#define OSC_TASK_PRIO     (tskIDLE_PRIORITY + 1)' in amh
    and 'xTaskCreate(OscTask,   "Osc",   OSC_TASK_STACK,   NULL, OSC_TASK_PRIO,   &osc_task_handle)' in raw)

# (f) run
vv = presched_violations(raw)
chk('(f) ZERO mutex Take/Give/OSC_* tokens and ZERO calls to locking functions in main() before vTaskStartScheduler()', not vv, str(vv))
mm = src[fn['main'][0]:fn['main'][1]]
chk('(f) no mutex token anywhere in main() except the xSemaphoreCreateMutex creation (ALL locking lives in the tasks)',
    not re.search(r'\b(OSC_LOCK|OSC_UNLOCK|OSC_SET|APP_LOCK|APP_UNLOCK|OscMeasGet|xSemaphoreTake|xSemaphoreGive)\b', mm))
chk('(f) init remains before mutex creation and no lock-taking function is called after creation but before scheduler start',
    ix('HdrInvalidate();') < ix('xSemaphoreCreateMutex') and not presched_violations(raw))
chk('(f) OscTask owns the initial UI draw prologue after scheduler start',
    all(k in body('OscTask')[:body('OscTask').index('for (;;)')] for k in ('UartPanel();', 'DrawGrid();', 'DrawCursor();', 'DrawTopPanel();')))

# (g) v95 task split: InputTask = the old per-pass HandleButtons/UartPoll, under the app lock, between OscTask passes
ot, itk = body('OscTask'), body('InputTask')
loop = ot[ot.index('for (;;)'):]
chk('(g) InputTask waits for the start-up screen, then polls under the app lock and wakes OscTask',
    re.search(r'ulTaskNotifyTake\(pdTRUE, portMAX_DELAY\);.*for \(;;\)\s*\{\s*APP_LOCK\(\);\s*HandleButtons\(HAL_GetTick\(\)\);\s*UartPoll\(\);\s*APP_UNLOCK\(\);'
              r'\s*xTaskNotifyGive\(osc_task_handle\);\s*vTaskDelay\(pdMS_TO_TICKS\(INPUT_POLL_MS\)\);\s*\}', itk, re.S) is not None)
chk('(g) OscTask releases InputTask only after the start-up draw', ot.index('xTaskNotifyGive(input_task_handle);') > ot.index('DrawTopPanel();') and ot.index('xTaskNotifyGive(input_task_handle);') < ot.index('for (;;)'))
chk('(g) HandleButtons / UartPoll are called only by InputTask', [n for n in fn if n != 'InputTask' and re.search(r'\b(HandleButtons|UartPoll)\s*\(', body(n))] == [])
chk('(g) one OscTask pass = APP_LOCK() .. APP_UNLOCK(), then the deadline sleep (no lock held while sleeping)',
    re.match(r'for \(;;\)\s*\{\s*APP_LOCK\(\);', loop) is not None and loop.count('APP_LOCK()') == 1 and loop.count('APP_UNLOCK()') == 1
    and loop.index('APP_UNLOCK()') < loop.index('ulTaskNotifyTake(') and not re.search(r'\b(return|continue)\b', loop))
chk('(g) InputTask outranks OscTask, 1 ms poll = HAL tick', '#define INPUT_TASK_PRIO   (tskIDLE_PRIORITY + 2)' in amh and '#define INPUT_POLL_MS     1 ' in amh)
chk('(g) lock order: OSC_LOCK only inside a pass/poll, never around APP_LOCK', all(not re.search(r'OSC_LOCK\(\);[^}]*APP_LOCK\(\)', body(n)) for n in fn))
broken = ROOT.parent/'builds/v66-rtos-step3a/main_broken.c'
if broken.exists():
    bv = presched_violations(broken.read_text(encoding='utf-8'))
    chk('(f) NEGATIVE CONTROL: the same check flags the hanging v66 main_broken.c (%d violations: %s)' % (len(bv), bv[:4]), len(bv) > 0)
else:
    print('SKIP (f) negative control (main_broken.c not present)')
print('rtos_step3a failures=%d' % fails)
sys.exit(1 if fails else 0)
