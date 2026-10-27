#ifndef APP_APP_MAIN_H
#define APP_APP_MAIN_H

#include "app/app_config.h"

#define OSC_TASK_STACK    2048
#define OSC_TASK_PRIO     (tskIDLE_PRIORITY + 1)
#define INPUT_TASK_STACK  2048
#define INPUT_TASK_PRIO   (tskIDLE_PRIORITY + 2)
#define INPUT_POLL_MS     1

#define APP_LOCK()    ((void)xSemaphoreTake(xAppLock, portMAX_DELAY))
#define APP_UNLOCK()  ((void)xSemaphoreGive(xAppLock))
extern SemaphoreHandle_t xAppLock;
extern TaskHandle_t osc_task_handle, input_task_handle;

void AppInit(void);
void AppStart(void);

#endif
