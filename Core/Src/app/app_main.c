/* app_main.c - Application start-up: called from main() after the CubeMX peripheral init. */
#include "app/app.h"

/* Everything main() does after the CubeMX peripheral init: ADC input, buttons, UART console, LCD. */
void AppInit(void)
{
#if ADC_INPUT_ENABLE
    AdcInputInit();   /* p30: ADC1+PA0+DMA สำหรับอินพุตจริง */
#endif


    /* ปุ่ม PB0..PB3 = input + pull-up
     * ชั่วคราว: gpio.c ที่ CubeMX generate ยังไม่ได้ config ขาเหล่านี้
     * ของจริงควรตั้ง GPIO_Input + Pull-up ใน CubeMX แล้ว generate ใหม่     */
    {
        GPIO_InitTypeDef btn_init = {0};
        __HAL_RCC_GPIOB_CLK_ENABLE();
        btn_init.Pin  = BTN_UP_PIN | BTN_DOWN_PIN | BTN_HOLD_PIN | BTN_SET_PIN;
        btn_init.Mode = GPIO_MODE_INPUT;
        btn_init.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOB, &btn_init);
        /* v61: four more buttons, same manual init (PA3, PC0, PC3, PF3) */
        __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
        btn_init.Pin = BTN_VDIV_PIN;                  HAL_GPIO_Init(BTN_VDIV_PORT, &btn_init);
        btn_init.Pin = BTN_TDIV_PIN | BTN_FILT_PIN;   HAL_GPIO_Init(GPIOC, &btn_init);
        btn_init.Pin = BTN_TRIG_PIN;                  HAL_GPIO_Init(BTN_TRIG_PORT, &btn_init);
    }

    /* Button struct init (done here to avoid static-initializer cast issue) */
    btn_up   = (Button){BTN_UP_PORT,   BTN_UP_PIN,   0, 0, 0, 0};
    btn_down = (Button){BTN_DOWN_PORT, BTN_DOWN_PIN, 0, 0, 0, 0};
    btn_hold = (Button){BTN_HOLD_PORT, BTN_HOLD_PIN, 0, 0, 0, 0};
    btn_set  = (Button){BTN_SET_PORT,  BTN_SET_PIN,  0, 0, 0, 0};
    btn_vdiv = (Button){BTN_VDIV_PORT, BTN_VDIV_PIN, 0, 0, 0, 0};
    btn_tdiv = (Button){BTN_TDIV_PORT, BTN_TDIV_PIN, 0, 0, 0, 0};
    btn_filt = (Button){BTN_FILT_PORT, BTN_FILT_PIN, 0, 0, 0, 0};
    btn_trig = (Button){BTN_TRIG_PORT, BTN_TRIG_PIN, 0, 0, 0, 0};

    /* LCD init */
    UartInputInit();          /* เริ่มรับคำสั่งจาก USART3 (Virtual COM) */
    /* v66b: UartPanel() moved to the start of OscTask (it reads osc_meas under the mutex; no Take before the scheduler) */

    ILI9341_Init();
    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
    ILI9341_Fill_Screen(BLACK);
    HdrInvalidate();

#if SHOW_FPS
    /* --- benchmark ครั้งเดียวตอนบูต: วัดต้นทุนต่อพิกเซลของแต่ละวิธี --------- */
    {
        uint32_t tb;
        uint8_t  two[2] = {0xAA, 0x55};

        ILI9341_Draw_Filled_Rectangle_Coord(0, 224, 319, 239, BLACK);

        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
        tb = HAL_GetTick();
        for (int i = 0; i < 2000; i++) HAL_SPI_Transmit(&hspi5, two, 2, 1);
        dbg_b_spi_ms = HAL_GetTick() - tb;
        HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

        tb = HAL_GetTick();
        for (int i = 0; i < 2000; i++) ILI9341_Draw_Pixel(160, 230, YELLOW);
        dbg_b_px_ms = HAL_GetTick() - tb;

        tb = HAL_GetTick();
        for (int i = 0; i < 20; i++) ILI9341_Draw_Rectangle(2, 231, 300, 1, YELLOW);
        dbg_b_burst_ms = HAL_GetTick() - tb;

        ILI9341_Draw_Filled_Rectangle_Coord(0, 224, 319, 239, BLACK);
        DrawGrid();
    }
#endif

    /* Initial double-buffer state */
    for (int i = 0; i < N_SAMPLES; i++) old_y[i] = CENTER_Y;

    /* v66b: initial DrawGrid/DrawCursor/DrawTopPanel moved to the start of OscTask (DrawTopPanel locks the mutex) */

}

SemaphoreHandle_t xAppLock;                          /* one task at a time works on the scope state + LCD (see app_main.h) */
TaskHandle_t osc_task_handle, input_task_handle;

/* Creates the mutexes and the two tasks, then starts the scheduler (never returns). */
void AppStart(void)
{
    xOscMutex = xSemaphoreCreateMutex();
    xAppLock  = xSemaphoreCreateMutex();
    if (xOscMutex == NULL || xAppLock == NULL) Error_Handler();
    if (xTaskCreate(OscTask,   "Osc",   OSC_TASK_STACK,   NULL, OSC_TASK_PRIO,   &osc_task_handle)   != pdPASS ||
        xTaskCreate(InputTask, "Input", INPUT_TASK_STACK, NULL, INPUT_TASK_PRIO, &input_task_handle) != pdPASS)
        Error_Handler();
    vTaskStartScheduler();   /* never returns */
}
