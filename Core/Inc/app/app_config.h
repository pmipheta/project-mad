#ifndef APP_APP_CONFIG_H
#define APP_APP_CONFIG_H

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define ADC_RAW_MAX        65520u
#define OSC_NT             18
#define OSC_VDIV_DEF      12
#define OSC_TDIV_DEF       8
#define OSC_NT_FAST        12
#define ADC_SAMPLETIME_FAST ADC_SAMPLETIME_3CYCLES
#define ADC_PRE_POINTS     200

#define BTN_UP_PORT     GPIOB
#define BTN_UP_PIN      GPIO_PIN_0
#define BTN_DOWN_PORT   GPIOB
#define BTN_DOWN_PIN    GPIO_PIN_1
#define BTN_HOLD_PORT   GPIOB
#define BTN_HOLD_PIN    GPIO_PIN_2
#define BTN_SET_PORT    GPIOB
#define BTN_SET_PIN     GPIO_PIN_3
#define BTN_VDIV_PORT   GPIOA
#define BTN_VDIV_PIN    GPIO_PIN_3
#define BTN_TDIV_PORT   GPIOC
#define BTN_TDIV_PIN    GPIO_PIN_0
#define BTN_FILT_PORT   GPIOC
#define BTN_FILT_PIN    GPIO_PIN_3
#define BTN_TRIG_PORT   GPIOF
#define BTN_TRIG_PIN    GPIO_PIN_3
#define ADC_DC_ZERO_CT  415.0f

#define HEADER_H        29
#define WAVEFORM_TOP    HEADER_H
#define SCREEN_W        ILI9341_SCREEN_WIDTH
#define SCREEN_H        ILI9341_SCREEN_HEIGHT
#define CENTER_Y        120

#define GRID_Y_STEP     40
#define GRID_Y_OFFSET   0
#define GRID_X_DOT      6
#define GRID_X_STEP     32
#define GRID_V_DOT      4
#define IS_GRID_DOT(x, y) ((((y) % GRID_Y_STEP == GRID_Y_OFFSET) && ((x) % GRID_X_DOT == 0)) || \
                           (((y) >= WAVEFORM_TOP) && ((x) % GRID_X_STEP == 0) && ((y) % GRID_V_DOT == 0)))

#define ADC_MID         2048
#define ADC_FULL        4095
#define ADC_VREF        3.3f
#define ADC_INPUT_FC    0.60e6f
#define ADC_INPUT_N     1.7f
#define ADC_INPUT_GAIN  1.25f
#define ADC_INPUT_GMAX  3.5f

#define N_SAMPLES       240

#define FAST_O2         __attribute__((optimize("O2")))
#define FRAME_MS        33
#define HEADER_MS       300
#define BLINK_PERIOD_MS 300u
#define BTN_MAN_PORT    User_Blue_GPIO_Port
#define BTN_MAN_PIN     User_Blue_Pin
#define BTN_MAN_ACTIVE_HIGH 1
#define BTN_MAN_LONG_MS 1000
#define BTN_CURSOR_LONG_MS 1500
#define BTN_ACDC_LONG_MS   1500
#define BTN_DEBOUNCE_MS 200
#define BTN_LONG_MS     2000
#define BTN_REPEAT_FIRST_MS 300u
#define BTN_REPEAT_MS       60u
#define BTN_ACCEL_MS        1500u
#define BTN_ACCEL_STEPS     3
#define BTN_REL_MS          30u
#define BTN_CUR_REPEAT_MS   20u

#define SHOW_FPS        1

typedef enum { TRIG_AUTO = 0, TRIG_SINGLE, TRIG_NORM } TriggerMode;

typedef struct {
    float vmax;
    float vmin;
    float vpp;
    float vrms;
    float freq;
    float dc_offset;
} OscMeasure;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      last_press_tick;
    uint32_t      held_since_tick;
    uint8_t       is_held;
    uint8_t       long_fired;
    uint32_t      next_rep_tick;
    uint32_t      low_tick;
    uint8_t       low_pending;
} Button;

#define OSC_H_DIVS      10
#define OSC_V_DC_DIVS   2.0f
#define OSC_V_FILL_DIVS 4.0f
#define OSC_FIT_CYCLES  2.5f
#define OSC_SET_COUNT   4
#define OSC_HOLD_COUNT  3

#endif
