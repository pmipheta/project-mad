#include "app/app.h"

void InputTask(void *argument)
{
    (void)argument;
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (;;)
    {
        APP_LOCK();
        HandleButtons(HAL_GetTick());
        APP_UNLOCK();
        xTaskNotifyGive(osc_task_handle);
        vTaskDelay(pdMS_TO_TICKS(INPUT_POLL_MS));
    }
}
