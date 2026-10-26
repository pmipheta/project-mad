/* app_config.h - build switches, pins, screen geometry, timing, shared types. */
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

/* ============================================================================
 * Board   : STM32F767ZIT6 (Nucleo-F767ZI)
 * Display : ILI9341 320x240 TFT via SPI5 (driver martnak/STM32-ILI9341)
 * Input   : ADC1 PA0 (FAST, burst DMA) and ADC3 (SLOW, 1 kHz history); see DEVELOPER_GUIDE.md
 * Controls: eight front-panel buttons + PC13 (pin map below); there is no serial console.
 * ============================================================================
 */

#define ADC_RAW_MAX        65520u  /* <=16-bit NDTR; whole 32-byte cache lines */
#define OSC_NT             18       /* number of T/DIV (12 FAST + 6 SLOW) */
#define OSC_VDIV_DEF      12       /* index of 1.0 V/div in osc_vdivs[] */
#define OSC_TDIV_DEF       8        /* index ของ 1.0 ms/div = ค่าเริ่มต้น */
#define OSC_NT_FAST        12       /* T/DIV 0..11 = FAST (burst DMA); 12..17 = SLOW (history, SysTick) */
#define ADC_SAMPLETIME_FAST ADC_SAMPLETIME_3CYCLES /* 12 MHz / 15 = 800 kSa/s */
#define ADC_PRE_POINTS     200      /* จำนวนจุด "ก่อน trigger" (ครึ่งจอซ้าย) — p38 */

/* ===== CONFIGURE BUTTONS — change port/pin to match your wiring ===========
 * Wire momentary buttons between pin and GND.
 * Set GPIO mode = Input, Pull = Pull-up in CubeMX (active LOW when pressed).
 * =========================================================================== */
#define BTN_UP_PORT     GPIOB
#define BTN_UP_PIN      GPIO_PIN_0    /* TODO: set to actual pin */
#define BTN_DOWN_PORT   GPIOB
#define BTN_DOWN_PIN    GPIO_PIN_1    /* TODO: set to actual pin */
#define BTN_HOLD_PORT   GPIOB
#define BTN_HOLD_PIN    GPIO_PIN_2    /* TODO: set to actual pin */
#define BTN_SET_PORT    GPIOB
#define BTN_SET_PIN     GPIO_PIN_3    /* TODO: set to actual pin */
/* v61 eight-button panel. Pin -> role: PB0 = INCREASE(+), PB1 = DECREASE(-), PB2 = AUTO, PB3 = FAST/SLOW (short) / auto FAST/SLOW (long),
 * v62 revised: PA3 = V/DIV<->T/DIV select toggle, PC0 = FILTER<->TRIG select toggle, PC3 = short FREEZE / long(1500ms) C1<->C2, PF3 = FREEZE toggle (duplicate). All active LOW, pull-up. */
#define BTN_VDIV_PORT   GPIOA
#define BTN_VDIV_PIN    GPIO_PIN_3
#define BTN_TDIV_PORT   GPIOC
#define BTN_TDIV_PIN    GPIO_PIN_0
#define BTN_FILT_PORT   GPIOC
#define BTN_FILT_PIN    GPIO_PIN_3
#define BTN_TRIG_PORT   GPIOF
#define BTN_TRIG_PIN    GPIO_PIN_3
#define ADC_DC_ZERO_CT  415.0f        /* v89: DC mode: ADC count of 0 V at the input. v90: 415 = 0.335 V at PA0 measured with no
                                      *      signal: D1 (1N5819) leaks ~71 uA from 3.3 V through R2. Temporary: drifts with temperature;
                                      *      set back to 0 after D1/D2 are low-leakage diodes. */

/* --- Screen geometry ------------------------------------------------------- */
#define HEADER_H        29           /* top panel height (px)                  */
#define WAVEFORM_TOP    HEADER_H     /* waveform Y start                       */
#define SCREEN_W        ILI9341_SCREEN_WIDTH    /* 320                         */
#define SCREEN_H        ILI9341_SCREEN_HEIGHT   /* 240                         */
#define CENTER_Y        120          /* waveform vertical midpoint              */

/* --- Grid ------------------------------------------------------------------ */
#define GRID_Y_STEP     40           /* px between horizontal grid lines        */
#define GRID_Y_OFFSET   0            /* grid rows y=40,80,120,...: row 120 = CENTER_Y (0 V) */
#define GRID_X_DOT      6            /* horizontal dot spacing                  */
#define GRID_X_STEP     32           /* v75: px between vertical grid lines = 1 T/DIV (SCREEN_W 320 / OSC_H_DIVS 10) */
#define GRID_V_DOT      4            /* v75: dot spacing along the vertical (time) lines */
/* v75: ONE rule for "this pixel is a grid dot" (horizontal V/DIV rows + vertical T/DIV lines); every draw/erase path uses it */
#define IS_GRID_DOT(x, y) ((((y) % GRID_Y_STEP == GRID_Y_OFFSET) && ((x) % GRID_X_DOT == 0)) || \
                           (((y) >= WAVEFORM_TOP) && ((x) % GRID_X_STEP == 0) && ((y) % GRID_V_DOT == 0)))

/* --- ADC reference --------------------------------------------------------- */
#define ADC_MID         2048         /* 12-bit midpoint (= 0 V after bias)     */
#define ADC_FULL        4095         /* 12-bit full scale                       */
#define ADC_VREF        3.3f         /* reference voltage (V)                   */
#define ADC_INPUT_FC    0.60e6f      /* v53: input roll-off corner (Hz); fit to 1-2.17 MHz board readings vs 1 kHz (+-3 %) */
#define ADC_INPUT_N     1.7f         /* v53: roll-off steepness (1.0 would be a plain RC)                              */
#define ADC_INPUT_GAIN  1.25f        /* v84: input volts per ADC-pin volt. Board: 1.00 Vpp set -> 0.80 V at PA0, 3.00 V -> 2.40 V (front-end divider).
                                      * Applied wherever volts are drawn or shown (trace, band, readout, AUTO V/DIV); the signal-detection
                                      * thresholds (0.05 / 0.30 V) stay at the pin. */
#define ADC_INPUT_GMAX  3.5f         /* never scale more than 3.5x (noise)      */

/* --- Sample buffer --------------------------------------------------------- */
#define N_SAMPLES       240

/* --- Frame timing ---------------------------------------------------------- */
/* Heavy sample loops of the FAST path are compiled at -O2 even in the Debug (-O0) build: AdcRawFrequency alone took 51 ms for 27k samples at -O0 */
#define FAST_O2         __attribute__((optimize("O2")))
#define FRAME_MS        33           /* ~30 FPS                                 */
#define HEADER_MS       300          /* header refresh interval (ms)            */
#define BLINK_PERIOD_MS 300u         /* v60: SET-selection blink, ms per ON or OFF phase (full cycle 600 ms ~1.7 Hz). Kept = HEADER_MS because the header is only redrawn every HEADER_MS, so each redraw lands in the opposite phase */
#define BTN_MAN_PORT    User_Blue_GPIO_Port   /* PC13 = Nucleo user button (blue) */
#define BTN_MAN_PIN     User_Blue_Pin
#define BTN_MAN_ACTIVE_HIGH 1                 /* Nucleo B1: pressed = 1 */
#define BTN_MAN_LONG_MS 1000                  /* hold this long = back to AUTO */
#define BTN_CURSOR_LONG_MS 1500               /* v62: PC3 held this long = toggle C1/C2 cursor */
#define BTN_ACDC_LONG_MS   1500               /* v90: PB2 (AUTO) held this long = toggle AC <-> DC coupling (replaces the PE0 input) */
#define BTN_DEBOUNCE_MS 200          /* button debounce window                  */
#define BTN_LONG_MS     2000         /* (v53 long-press threshold, no longer used: AUTO is a short press) */
#define BTN_REPEAT_FIRST_MS 300u     /* v54: UP/DOWN held: first step at once, repeat after this ...      */
#define BTN_REPEAT_MS       60u      /*      ... every 60 ms                                            */
#define BTN_ACCEL_MS        1500u    /*      ... and after this long ...                                */
#define BTN_ACCEL_STEPS     3        /*      ... 3 steps per repeat                                      */
#define BTN_REL_MS          30u      /* v54: a release must read stable this long (contact bounce)      */
#define BTN_CUR_REPEAT_MS   20u      /* v85: C1/C2 held (frozen): repeat this often, 1 sample per step  */

/* --- Debug: แสดง FPS จริงบนจอ (1 = เปิด, 0 = ปิด) --- */
#define SHOW_FPS        1


/* Trigger modes */
typedef enum { TRIG_AUTO = 0, TRIG_SINGLE, TRIG_NORM } TriggerMode;

/* Measurement values — teammate populates this after each ADC frame */
typedef struct {
    float vmax;
    float vmin;
    float vpp;
    float vrms;
    float freq;
    float dc_offset;
} OscMeasure;

/* Button state for debouncing */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      last_press_tick;  /* tick of last confirmed press            */
    uint32_t      held_since_tick;  /* tick of when held press began           */
    uint8_t       is_held;          /* 1 = currently held down                 */
    uint8_t       long_fired;       /* 1 = long-press action already triggered */
    uint32_t      next_rep_tick;    /* v54: next auto-repeat of a held UP/DOWN  */
    uint32_t      low_tick;         /* v54: tick when the pin first read released (release debounce) */
    uint8_t       low_pending;      /* v54: 1 = pin reads released, waiting BTN_REL_MS to be sure */
} Button;

/* settings menu + auto scale */
#define OSC_H_DIVS      10          /* จำนวนช่องแนวนอน                        */
#define OSC_V_DC_DIVS   2.0f        /* v90 auto scale, DC: the trace starts at 0 V on the centre line -> its peak may reach this many divisions from it */
#define OSC_V_FILL_DIVS 4.0f        /* v54 auto scale: Vpp may span at most this many divisions (the visible area is ~5.3 div: 2.2 above, 3 below the centre) */
#define OSC_FIT_CYCLES  2.5f        /* auto scale: อยากเห็นกี่คาบบนจอ (10 ช่อง) */
#define OSC_SET_COUNT   4           /* FILTER / TRIG / V/DIV / T/DIV            */
#define OSC_HOLD_COUNT  3           /* C1 / C2 / V/DIV (ตอน HOLD)               */

#endif /* APP_APP_CONFIG_H */
