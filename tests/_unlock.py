"""v66 helper: textually revert the step-3a mutex edits so the older source-extraction tests see the v65 text.
unlock(main_v66) == main_v65 exactly (asserted in test_rtos_step3a.py), which is itself the proof that v66 only ADDS locking."""
import re
def unlock(s):
    s = s.replace('#include "semphr.h"\n', '', 1)
    # v66b: undo the relocation of the one-shot start-up draws from vMainTask back into main() (v65 placement)
    a = s.find('    /* v66b: one-shot start-up'); b = s.index('    for (;;)', a) if a >= 0 else -1
    if a >= 0: s = s[:a] + s[b:]
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
