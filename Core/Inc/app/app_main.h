/* app_main.h - Application start-up: called from main() after the CubeMX peripheral init. */
#ifndef APP_APP_MAIN_H
#define APP_APP_MAIN_H

#include "app/app_config.h"

/* Tasks: InputTask polls buttons + UART and outranks OscTask, so a press is handled as soon as OscTask finishes its pass. */
#define OSC_TASK_STACK    2048                    /* words */
#define OSC_TASK_PRIO     (tskIDLE_PRIORITY + 1)
#define INPUT_TASK_STACK  2048                    /* words: button actions reach MvBuildView/MvBuildZoom, commands reach snprintf */
#define INPUT_TASK_PRIO   (tskIDLE_PRIORITY + 2)
#define INPUT_POLL_MS     1                       /* = HAL tick: debounce/hold/repeat times are seen as with the old busy loop */

/* App lock: the task holding it may touch any scope state and the LCD. OscTask holds it for one whole pass (capture ->
 * draw -> header), InputTask for one poll. Nesting: xAppLock first, then xOscMutex (OSC_LOCK) inside, never the reverse. */
#define APP_LOCK()    ((void)xSemaphoreTake(xAppLock, portMAX_DELAY))
#define APP_UNLOCK()  ((void)xSemaphoreGive(xAppLock))
extern SemaphoreHandle_t xAppLock;
extern TaskHandle_t osc_task_handle, input_task_handle;

void AppInit(void);
void AppStart(void);

#endif /* APP_APP_MAIN_H */
