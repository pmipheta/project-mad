"""FreeRTOS refactor step 1 contracts: whole loop body moved verbatim into vMainTask, SysTick shared HAL+FreeRTOS, start order, Slow block unchanged."""
import re, pathlib
ROOT = pathlib.Path(__file__).resolve().parents[1]
src = (ROOT/'Core/Src/main.c').read_text(encoding='utf-8')
bef = (ROOT.parent/'builds/v65-rtos-step1/main_before.c').read_text(encoding='utf-8')
it  = (ROOT/'Core/Src/stm32f7xx_it.c').read_text()
fails = 0
def chk(name, ok):
    global fails
    print(('PASS ' if ok else 'FAIL ') + name); fails += (not ok)

def block_after(t, head_re):
    m = re.search(head_re, t); p = t.index('{', m.end()-1); d = 1; i = p+1
    while d:
        d += {'{': 1, '}': -1}.get(t[i], 0); i += 1
    return t[p+1:i-1], m.start(), i
def norm(b): return '\n'.join(l.rstrip() for l in b.strip().splitlines())

# (a) body identity: original while(1) of main() vs for(;;) of vMainTask
ob, o_s, o_e = block_after(bef, r'\n    while \(1\)\s*\n    \{')
tb, t_s, t_e = block_after(src, r'static void vMainTask\(void \*argument\)\n\{\n(?:.*\n)*?    for \(;;\)\s*\n    \{')
chk('vMainTask exists', 'static void vMainTask(void *argument)' in src)
chk('vMainTask body identical to original while(1) body (%d chars)' % len(ob), norm(ob) == norm(tb))
chk('body identical byte-for-byte (no re-indent)', ob.strip() == tb.strip())
chk('old while(1) removed from main()', not re.search(r'\n    while \(1\)', src[src.index('int main(void)'):src.index('void SystemClock_Config(void)\n{')]))
chk('Error_Handler while(1) untouched', src.count('while (1)') == bef.count('while (1)') - 1)

# (b) SysTick
st = it[it.index('void SysTick_Handler(void)'):]; st = st[:st.index('\n}\n')]
chk('SysTick calls HAL_IncTick unconditionally', re.search(r'^\s*HAL_IncTick\(\);', st, re.M) is not None)
chk('SysTick calls xPortSysTickHandler only if scheduler started',
    re.search(r'if \(xTaskGetSchedulerState\(\) != taskSCHEDULER_NOT_STARTED\) \{ xPortSysTickHandler\(\); \}', st) is not None)
chk('HAL_IncTick before xPortSysTickHandler', st.index('HAL_IncTick') < st.index('xPortSysTickHandler'))
chk('OscSlowTick still called from SysTick', 'OscSlowTick();' in st)

# (c) main() order
mn = src[src.index('int main(void)'):src.index('void SystemClock_Config(void)\n{')]
ix = lambda s: mn.index(s)
chk('xTaskCreate then vTaskStartScheduler', ix('xTaskCreate(vMainTask') < ix('vTaskStartScheduler();'))
chk('scheduler start after DrawTopPanel()', ix('DrawTopPanel();') < ix('xTaskCreate'))
init_calls = re.findall(r'^\s*((?:HAL_Init|SystemClock_Config|MX_\w+_Init|ILI9341_\w+|DrawGrid|DrawCursor|DrawTopPanel)\()', mn, re.M)
last_init = max(m.start() for m in re.finditer(r'^\s*(?:HAL_Init|SystemClock_Config|MX_\w+_Init|ILI9341_Init|DrawGrid|DrawCursor|DrawTopPanel)\(', mn, re.M))
chk('all init calls (%d) precede xTaskCreate' % len(init_calls), last_init < ix('xTaskCreate'))
chk('stack 2048 words, prio idle+1', 'xTaskCreate(vMainTask, "Main", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);' in mn)
b_init = lambda t: t[t.index('int main(void)'):t.index('    uint32_t last_frame_tick')] if 'uint32_t last_frame_tick' in t else ''
chk('init section of main() byte-identical to before', b_init(bef).replace('#include','') == src[src.index('int main(void)'):src.index('int main(void)')+len(b_init(bef))])

# (d) Slow block
sl = lambda t: t[t.index('SLOW_BEGIN ='):t.index('SLOW_END =')]
chk('Slow block byte-identical (%d chars)' % len(sl(bef)), sl(src) == sl(bef))

# misc: config sanity
cfg = (ROOT/'Core/Inc/FreeRTOSConfig.h').read_text()
chk('tick rate 1000, preemption on', 'configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )' in cfg and 'configUSE_PREEMPTION                    1' in cfg)
print('rtos_step1 failures=%d' % fails)
raise SystemExit(1 if fails else 0)
