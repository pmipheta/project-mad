"""v66 helper: textually revert the step-3a mutex edits so the older source-extraction tests see the v65 text.
unlock(main_v66) == main_v65 exactly (asserted in test_rtos_step3a.py), which is itself the proof that v66 only ADDS locking."""
import re
def unlock(s):
    s = s.replace('#include "semphr.h"\n', '', 1)
    # v66b: undo the relocation of the one-shot start-up draws from vMainTask back into main() (v65 placement)
    a = s.find('    /* v66b: one-shot start-up'); b = s.index('    for (;;)', a) if a >= 0 else -1
    if a >= 0: s = s[:a] + s[b:]
    s = s.replace('    /* v66b: UartPanel() moved to the start of OscTask (it reads osc_meas under the mutex; no Take before the scheduler) */\n',
                  '    UartPanel();              /* \u0e41\u0e2a\u0e14\u0e07\u0e40\u0e21\u0e19\u0e39/\u0e2a\u0e16\u0e32\u0e19\u0e30\u0e04\u0e23\u0e31\u0e49\u0e07\u0e41\u0e23\u0e01\u0e2d\u0e2d\u0e01\u0e1e\u0e2d\u0e23\u0e4c\u0e15 serial */\n', 1)
    s = s.replace('    /* v66b: initial DrawGrid/DrawCursor/DrawTopPanel moved to the start of OscTask (DrawTopPanel locks the mutex) */\n',
                  '    DrawGrid();\n    DrawCursor();\n    DrawTopPanel();\n', 1)
    a = s.find('/* --- v66 step 3a:')
    if a >= 0:
        b = s.index('/* --- Display double-buffer ---', a); s = s[:a] + s[b:]
    s = s.replace('\n    xOscMutex = xSemaphoreCreateMutex();   /* creation only (non-blocking). main() must NEVER Take/Give: scheduler not running yet */\n    if (xOscMutex == NULL) Error_Handler();\n', '', 1)
    s = re.sub(r'OSC_SET\((\w+)( *), (.*?)\);', r'\1 \2= \3;', s)
    s = re.sub(r'^[ \t]*OSC_(?:UN)?LOCK\(\);\n', '', s, flags=re.M)
    s = s.replace('OSC_LOCK(); ', '').replace(' OSC_UNLOCK();', '')
    s = s.replace('    xOscMutex = xSemaphoreCreateMutex();\n    if (xOscMutex == NULL) Error_Handler();\n', '', 1)
    s = re.sub(r'[ \t]*OscMeasure (?:sm|tm); OscMeasGet\(&(?:sm|tm)\);[^\n]*\n', '', s)
    s = re.sub(r'\b(?:sm|tm)\.(vmax|vmin|vpp|vrms|freq|dc_offset)\b', r'osc_meas.\1', s)
    return s
