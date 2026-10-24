"""v65 (FreeRTOS step 1) moved the main-loop body into vMainTask. The historical diff-budget contracts in test_realpic/zoom/eightbtn
compare main.c against older bare-metal versions, so they read main.c through unrtos(), which exactly undoes the step-1 restructuring
(wrapper, includes, task create/start). Body text is untouched, so all other checks see the real code."""
import re

_ACT = {
    'BtnEmit(BE_UP,   (uint8_t)n)': 'ActIncrease(n)', 'BtnEmit(BE_DOWN, (uint8_t)n)': 'ActDecrease(n)', 'BtnEmit(BE_AUTO, 0)': 'ActAuto()',
    'BtnEmit(BE_VDIV, 0)': 'ActSelectSetting(strcmp(OscSetName(), "V/DIV") == 0 ? "T/DIV" : "V/DIV")',
    'BtnEmit(BE_TDIV, 0)': 'ActSelectSetting(strcmp(OscSetName(), "FILTER") == 0 ? "TRIG" : "FILTER")',
    'BtnEmit(BE_FREEZE, 0)': 'ActFreeze()',
    'BtnEmit(BE_CURSOR_LONG, 0)': 'if (hold_active) ActSelectSetting(strcmp(OscSetName(), "C1") == 0 ? "C2" : "C1")',
    'BtnEmit(BE_STABLE, 0)': 'ActToggleStable()', 'BtnEmit(BE_AUTOMODE, 0)': 'ActAutoMode()', 'BtnEmit(BE_FASTSLOW, 0)': 'ActToggleFastSlow()',
}
_OLD_HDR = """/* --------------------------------------------------------------------------
 * HandleButtons
 * Call this every loop iteration (before frame logic).
 * Roles: HOLD pin = FREEZE | SET pin = AUTO | UP pin = UP/LEFT | DOWN pin = DOWN/RIGHT | PC13 = FAST<>SLOW (short) / AUTO FAST/SLOW (long)
 * -------------------------------------------------------------------------- */
"""
def unbtn3b(t):
    """v77 step 3b split HandleButtons into ButtonSample (queue producer) + BtnDispatchEv/HandleButtons (consumer). Reconstruct the single combined
    HandleButtons (text identical to the pre-3b source) so older diff-budget contracts and 'unchanged' checks see the same code."""
    a = t.find('/* --------------------------------------------------------------------------\n * v77 (RTOS step 3b)')
    if a < 0: return t
    z = t.index('#if SHOW_FPS', a)
    blk = t[a:z]
    b0 = blk.index('static void ButtonSample(uint32_t now)\n{\n    int n;\n') + len('static void ButtonSample(uint32_t now)\n{\n    int n;\n')
    b1 = blk.index('\n}\n\nstatic void BtnDispatchEv')
    body = blk[b0:b1]
    for k, v in _ACT.items():
        body = body.replace(k + ';', v + ';').replace(k + ' ', v + ' ')
    body = body.replace('{ fs_long = 1; BtnEmit(BE_AUTOMODE, 0); }', '{ fs_long = 1; ActAutoMode(); }')
    body = body.replace('{ man_long = 1; BtnEmit(BE_AUTOMODE, 0); }', '{ man_long = 1; ActAutoMode(); }')
    body = body.replace('            BtnEmit(BE_CURSOR_LONG, 0);', '            ' + _ACT['BtnEmit(BE_CURSOR_LONG, 0)'] + ';')
    body = body.replace('(only while frozen: checked at dispatch)', '(only while frozen)')
    body = body.replace('    (void)ButtonLongPress;\n', '    (void)ButtonLongPress;\n', 1)
    body = body.replace('\n\n    /* v64: PF3', '\n    (void)ActToggleTrigEdge;\n\n    /* v64: PF3', 1)
    old = _OLD_HDR + 'static void HandleButtons(uint32_t now)\n{\n    int n;\n    MvSync();\n' + body + '\n}\n\n'
    return t[:a] + old + t[z:]

def unrtos(t):
    t = unbtn3b(t)
    t = t.replace('#include "FreeRTOS.h"\n#include "task.h"\n', '', 1)
    t = t.replace('#include "queue.h"\n', '', 1)                       # v77 step 3b: queue + vInputTask creation block -> the old single xTaskCreate line
    t = re.sub(r'    /\* v77 step 3b: queue \+ sampler task[^\n]*\n    xBtnQ = xQueueCreate.*?\n    if \(xTaskCreate\(vMainTask, \"Main\", 2048, NULL, tskIDLE_PRIORITY \+ 1, NULL\) != pdPASS\) Error_Handler\(\);\n',
               '    xTaskCreate(vMainTask, "Main", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);\n', t, count=1, flags=re.S)
    m = re.search(r'/\* Step 1 of FreeRTOS refactor:[^\n]*\n(?:static void vMainTask\(void \*argument\)\n\{\n    \(void\)argument;\n'
                  r'    static uint32_t last_frame_tick  = 0;\n    static uint32_t last_header_tick = 0;\n\n    for \(;;\)\n    \{\n)(.*?)    \}\n\}\n\n', t, re.S)
    if not m: return t
    body = m.group(1)
    t = t[:m.start()] + t[m.end():]
    start = ('    xTaskCreate(vMainTask, "Main", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);\n'
             '    vTaskStartScheduler();   /* never returns */\n    for (;;) { }\n')
    assert start in t
    t = t.replace(start, '    while (1)\n    {\n' + body + '    }\n', 1)
    t = t.replace('    DrawTopPanel();\n\n    /* USER CODE END 2 */', '    DrawTopPanel();\n\n    uint32_t last_frame_tick  = 0;\n    uint32_t last_header_tick = 0;\n\n    /* USER CODE END 2 */', 1)
    return t
