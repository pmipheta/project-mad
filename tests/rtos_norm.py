"""v65 (FreeRTOS step 1) moved the main-loop body into vMainTask. The historical diff-budget contracts in test_realpic/zoom/eightbtn
compare main.c against older bare-metal versions, so they read main.c through unrtos(), which exactly undoes the step-1 restructuring
(wrapper, includes, task create/start). Body text is untouched, so all other checks see the real code."""
import re
def unrtos(t):
    t = t.replace('#include "FreeRTOS.h"\n#include "task.h"\n', '', 1)
    m = re.search(r'/\* Step 1 of FreeRTOS refactor:[^\n]*\n(?:static void vMainTask\(void \*argument\)\n\{\n    \(void\)argument;\n'
                  r'    static uint32_t last_frame_tick  = 0;\n    static uint32_t last_header_tick = 0;\n\n    for \(;;\)\n    \{\n)(.*?)    \}\n\}\n\n', t, re.S)
    if not m: return t
    body = m.group(1)
    t = t[:m.start()] + t[m.end():]
    start = ('    xTaskCreate(vMainTask, "Main", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);\n'
             '    vTaskStartScheduler();   /* never returns */\n    for (;;) { }\n')
    if start not in t:      # since the module split the task start sits in AppStart() (app_main.c), without the for (;;)
        start = ('    xTaskCreate(vMainTask, "Main", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);\n'
                 '    vTaskStartScheduler();   /* never returns */\n')
    assert start in t
    t = t.replace(start, '    while (1)\n    {\n' + body + '    }\n', 1)
    t = t.replace('    DrawTopPanel();\n\n    /* USER CODE END 2 */', '    DrawTopPanel();\n\n    uint32_t last_frame_tick  = 0;\n    uint32_t last_header_tick = 0;\n\n    /* USER CODE END 2 */', 1)
    return t
