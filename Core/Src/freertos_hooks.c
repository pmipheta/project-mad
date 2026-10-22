/* FreeRTOS application hooks (Step 1). Halts on stack overflow so it is visible under a debugger. */
#include "FreeRTOS.h"
#include "task.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask; (void)pcTaskName;
    portDISABLE_INTERRUPTS();
    for (;;) { }
}
