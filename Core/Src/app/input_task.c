/* input_task.c - Input task: polls the front-panel buttons and the UART console every 1 ms. */
#include "app/app.h"

/* The same HandleButtons() / UartPoll() the single main loop ran on every pass, now on a fixed 1 ms beat (the HAL tick is
 * 1 ms, so every debounce / hold / repeat time is seen exactly as before). It holds the app lock while it works, so a button
 * or a command still lands between two OscTask passes, never inside a frame; then it wakes OscTask. */
void InputTask(void *argument)
{
    (void)argument;
    (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* wait until OscTask has drawn the start-up screen */
    for (;;)
    {
        APP_LOCK();
        HandleButtons(HAL_GetTick());
        UartPoll();               /* รับคำสั่งจาก USART3 (แทนปุ่มจริง) */
        APP_UNLOCK();
        xTaskNotifyGive(osc_task_handle);
        vTaskDelay(pdMS_TO_TICKS(INPUT_POLL_MS));
    }
}
