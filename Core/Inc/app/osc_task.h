/* osc_task.h - The oscilloscope task: capture -> process -> draw loop. */
#ifndef APP_OSC_TASK_H
#define APP_OSC_TASK_H

#include "app/app_config.h"

extern volatile uint32_t dbg_osc_sleep_ms;
extern volatile uint32_t dbg_stack_free[2];

void OscTask(void *argument);

#endif /* APP_OSC_TASK_H */
