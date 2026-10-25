/* ============================================================================
 * main.c  —  Oscilloscope Display Module  (Full Feature Mockup)
 *
 * Board   : STM32F767ZIT6 (Nucleo-F767ZI)
 * Display : ILI9341 320x240 TFT via SPI5
 * Driver  : martnak/STM32-ILI9341
 *
 * SCREEN LAYOUT (landscape 320x240)
 * ┌────────────────────────────────────────────────────────────────────┐
 * │ Row1 y=1  : [V/div] [T/div] [TrigMode] [Status]          (size 1) │
 * │ Row2 y=10 : [Vpp]   [Freq]  [Vmax]    [Vmin]   [RMS]    (size 1) │
 * │ ────────────────────────────── y=18 separator ──────────────────── │
 * │ y=19..239 : Waveform area (221 px, ~5.5 grid divisions)           │
 * └────────────────────────────────────────────────────────────────────┘
 *
 * BUTTONS  (configure the 4 macros below to match actual wiring)
 *   BTN_UP   — V/div increase (short) | T/div increase (after SET toggle)
 *   BTN_DOWN — V/div decrease
 *   BTN_HOLD — freeze / un-freeze  |  reset SINGLE capture
 *   BTN_SET  — short: cycle trigger mode (AUTO→SINGLE→NORM)
 *              long (2 s): Auto Scale
 *
 * MERGE WITH SAMPLING TEAMMATE
 *   1. Remove GenerateFakeSineWave() and its call inside while(1).
 *   2. Teammate fills raw_samples[N_SAMPLES] (0-4095) and sets new_frame_ready=1.
 *   3. Teammate fills osc_meas (vmax, vmin, vpp, vrms, freq).
 *   4. Integration point is clearly marked in the while(1) block.
 * ============================================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ===== p30: อินพุตจริงจากเครื่องกำเนิดสัญญาณ (ADC1 = PA0) ==================
 *   1 = ใช้ข้อมูลจริงจาก ADC   |   0 = ใช้ simulator เดิม (GenFrame ฯลฯ)
 *   สลับค่าเดียวนี้เพื่อกลับไปใช้ simulator ได้ทันที (โค้ดเดิมไม่ถูกแตะ)
 * ========================================================================== */
#define ADC_INPUT_ENABLE   1
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

/* USER CODE END PD */

/* USER CODE BEGIN PTD */

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

/* USER CODE END PTD */

/* USER CODE BEGIN PV */

/* --- Sample buffer (teammate writes here) ---------------------------------- */
volatile int16_t raw_samples[N_SAMPLES];  /* ADC counts 0-4095                */
volatile uint8_t new_frame_ready = 0;     /* teammate sets 1 when buffer ready */

/* --- Measurement results (teammate writes here) ---------------------------- */
volatile OscMeasure osc_meas = {0};

/* --- v66 step 3a: mutex around the oscilloscope state shared between future tasks -------------------------------
 * Protected: osc_meas, osc_sidx, hold_active, osc_dirty, osc_stable, osc_auto_tb.  Plain (non-recursive) mutex: every
 * critical section is a strict leaf (no lock is ever taken while held).  NEVER use from an ISR (OscSlowTick etc.). */
static SemaphoreHandle_t xOscMutex;
#define OSC_LOCK()    ((void)xSemaphoreTake(xOscMutex, portMAX_DELAY))
#define OSC_UNLOCK()  ((void)xSemaphoreGive(xOscMutex))
#define OSC_SET(v, x) do { OSC_LOCK(); (v) = (x); OSC_UNLOCK(); } while (0)
static void OscMeasGet(OscMeasure *o)
{
    OSC_LOCK();
    o->vmax = osc_meas.vmax; o->vmin = osc_meas.vmin; o->vpp = osc_meas.vpp;
    o->vrms = osc_meas.vrms; o->freq = osc_meas.freq; o->dc_offset = osc_meas.dc_offset;
    OSC_UNLOCK();
}

/* --- Display double-buffer ------------------------------------------------- */
static int16_t old_y[N_SAMPLES];
static int16_t new_y[N_SAMPLES];
/* v83 PEAK DETECT: min / max (ADC counts) of the raw samples behind each display point, valid when osc_env_on (FAST frame whose signal has
 * < 8 display points per period). MapToScreen turns them into env_y_*, TraceColumns paints the band between them. */
static uint16_t osc_env_lo_ct[N_SAMPLES], osc_env_hi_ct[N_SAMPLES];
static uint8_t  osc_env_on = 0;
static float    osc_env_mean_ct = 0.0f, osc_env_rms_ct = 0.0f;   /* mean / AC rms of the raw samples behind the window (readout fallback) */
static int16_t  env_y_lo[N_SAMPLES], env_y_hi[N_SAMPLES];   /* screen y of the max (top) and the min (bottom) */
static uint8_t  env_y_on = 0;

/* --- Display state --------------------------------------------------------- */
static float volts_per_div = 1.0f;

/* Time/div labels — shown in header; actual timebase is set by teammate's timer */
#define TDIV_COUNT  7

static TriggerMode trigger_mode    = TRIG_AUTO;
static uint8_t     hold_active     = 0;
static uint8_t     single_captured = 0;  /* 1 = SINGLE mode has captured frame */

/* --- Cursor ---------------------------------------------------------------- */
static uint16_t cursor_x = SCREEN_W / 2;  /* vertical cursor at screen centre  */
/* TODO: add BTN logic to move cursor_x left/right if needed                   */

/* --- Buttons --------------------------------------------------------------- */
static Button btn_up, btn_down, btn_hold, btn_set;
static Button btn_vdiv, btn_tdiv, btn_filt, btn_trig;   /* v61 */

/* --- Mock sine phase (remove after merge) ---------------------------------- */
static float mock_phase = 0.0f;

#if SHOW_FPS
static uint32_t fps_frames = 0, last_fps_tick = 0;

/* globals สำหรับวัดผลจากภายนอก (อ่านผ่าน SWD) — dbg_frames_total เป็น monotonic */
volatile uint32_t dbg_frames_total = 0;
/* ข้อความที่เพิ่งส่งไปจอ LCD (อ่านผ่าน SWD เพื่อยืนยันว่า indicator ขึ้นจริง) */
volatile char     dbg_lcd_set[24] = {0};    /* ช่อง setting มุมขวาบน */
volatile char     dbg_lcd_bar[40] = {0};    /* แถบ "กำลังตั้งค่าอะไร" บรรทัดล่าง */
volatile uint32_t dbg_fps_x10      = 0;
volatile uint32_t dbg_uptime_ms    = 0;
volatile uint32_t dbg_sig_hash     = 0;   /* FNV-1a ของ raw_samples[] เฟรมล่าสุด */
volatile uint32_t dbg_t0_ms        = 0;   /* tick ของเฟรมล่าสุด */

/* ตัววัดเพื่อหาคอขวด: พิกเซลต่อเฟรม + เวลาที่ใช้ในแต่ละส่วน */
volatile uint32_t dbg_px_frame = 0;
volatile uint32_t dbg_wf_ms    = 0;
volatile uint32_t dbg_hdr_ms   = 0;
volatile uint32_t dbg_loop_ms  = 0;
volatile uint32_t dbg_px_last   = 0;    /* px/frame ที่วัดได้เต็มเฟรม */
volatile uint32_t dbg_b_spi_ms  = 0;    /* 2000 x HAL_SPI_Transmit 2 bytes */
volatile uint32_t dbg_b_px_ms   = 0;    /* 2000 x ILI9341_Draw_Pixel */
volatile uint32_t dbg_b_burst_ms= 0;    /* 20 x Draw_Rectangle 300x1 (burst) */
#endif

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
static void     GenerateFakeSineWave(void);
static void     SimSamplingStream(void);
static void     GenFrame(void);
static void     OscProcessFrame(void);
static void     OscMeasureFrame(void);
static void     UartInputInit(void);
static void     UartPoll(void);
static void     UartPanel(void);
static void     OscAdjust(int dir);
static void     MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n);   /* v54: MOVE data block */
static void     MvFreezeCapture(void);
static void     MvBuildView(void);
static void     DrawCursors(void);
static int      OscCursorX(int idx);
static void     ClearCursors(void);
static void     SimUpdateMeasure(void);
static float    SimNoise(void);
static int      FindTrigger(void);
static void     MapToScreen(int trig_offset);
static void     DrawGrid(void);
static void     DrawLineBresenham(int x0, int y0, int x1, int y1, uint16_t color);
static void     DrawCursor(void);
static void     DrawWaveform(void);
static void     DrawTopPanel(void);
static void     DrawSettingBar(void);
static void     AutoScale(void);
static uint8_t  ButtonJustPressed(Button *btn, uint32_t now);
static uint8_t  ButtonLongPress(Button *btn, uint32_t now);
static void     HandleButtons(uint32_t now);
#if SHOW_FPS
static void     DrawFps(float fps);
#endif
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
volatile uint32_t dbg_adc_pre = 0;   /* pre-trigger display points used by the last FAST capture */
volatile uint32_t prof_us[8];   /* PROFILING (us): 0 capture, 1 process+measure+atb, 2 draw, 3 whole frame, 4 header, 5 loop period, 6 C1/C2 move (v85) */



/* ==========================================================================
 * INPUT ทาง USART3 (ใช้แทนปุ่มจริงที่ยังไม่ต่อ) + FUNCTION GENERATOR
 *   ทำตาม STM32 Oscilloscope Simulator (matplotlib) ฝั่ง "input" ทั้งหมด:
 *     SET  = วน setting ที่กำลังปรับ  (FILTER -> TRIG -> V/DIV -> T/DIV,
 *                                      ตอน HOLD: C1 -> C2 -> V/DIV)
 *     UP/DOWN = ปรับ setting ปัจจุบัน
 *     HOLD = freeze/ปลด freeze (ตอน HOLD จะโชว์ cursor C1/C2 + dV/dt)
 *     generator: wave / freq / amp / dc / noise (แทน sliders)
 *   คีย์: s=SET  u=UP  d=DOWN  h=HOLD  ?=แสดงสถานะ  g=สุ่มข้อมูลใหม่
 *         w[f|0..3]  f<Hz>  a<V>  o<V>  n<V>  v[+|-]  t[+|-]  c1<n> c2<n>  cal<0|1>
 * ========================================================================== */
#define OSC_H_DIVS      10          /* จำนวนช่องแนวนอน (เหมือน simulator)      */
#define SIM_STREAM_MODE  2          /* 2 = function generator (ตาม simulator) | 1 = stream | 0 = sine เดิม */
#define OSC_V_FILL_DIVS 4.0f        /* v54 auto scale: Vpp may span at most this many divisions (the visible area is ~5.3 div: 2.2 above, 3 below the centre) */
#define OSC_FIT_CYCLES  2.5f        /* auto scale: อยากเห็นกี่คาบบนจอ (10 ช่อง) */
#define OSC_SET_COUNT   4           /* FILTER / TRIG / V/DIV / T/DIV            */
#define OSC_HOLD_COUNT  3           /* C1 / C2 / V/DIV (ตอน HOLD)               */
#define UART_RX_SIZE    96
#define UART_IDLE_MS    250         /* หยุดพิมพ์เกินเท่านี้ = ถือว่าเป็นคำสั่งแบบเดี่ยว (ไม่ต้อง Enter) */
#define UART_DROP_MS    4000        /* บรรทัดค้าง (ไม่มี \r) เกินเท่านี้ = ทิ้ง */
#define OSC_PUSH        8           /* sample ใหม่ต่อเฟรม (คุมความเร็วไถล) */

static const char  *gen_wave_names[]  = {"SINE", "SQUARE", "TRI", "SAW"};
static const float  osc_vdivs[]       = {0.0001f, 0.0002f, 0.0005f,
                                          0.001f,  0.002f,  0.005f,
                                          0.01f,   0.02f,   0.05f,
                                          0.1f,    0.2f,    0.5f, 1.0f, 2.0f};
static const float  osc_tdivs_ms[]    = {0.005f, 0.01f, 0.015f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 6.0f,
                                         10.0f, 20.0f, 50.0f, 100.0f, 200.0f, 500.0f};   /* 0.015 ms/div = raw ADC samples (1.6 MSa/s); 10..500 ms/div = SLOW mode (10/20 ms/div: 30-100 Hz with 3-6 cycles instead of 15) */
static const char  *osc_set_names[]   = {"FILTER", "TRIG", "V/DIV", "T/DIV"};
static const char  *osc_hold_names[]  = {"C1", "C2", "V/DIV"};

/* --- generator (ส่วน input ของ simulator) --- */
static int      gen_wave      = 0;         /* 0=SINE 1=SQUARE 2=TRI 3=SAW      */
static float    gen_freq      = 1000.0f;   /* 100 .. 5000 Hz                   */
static float    gen_amp       = 0.5f;      /* 0.1 .. 1.5 V                     */
static float    gen_dc        = 1.65f;     /* 0.5 .. 2.5 V                     */
static float    gen_noise     = 0.05f;     /* 0 .. 0.3 V                       */
static float    gen_ph        = 0.0f;      /* เฟสสะสม (roll)                    */

/* --- oscilloscope settings --- */
static int      osc_vdiv_i    = OSC_VDIV_DEF; /* -> 1.0 V/div; table reaches 0.0001 V/div */
static int      osc_tdiv_i    = OSC_TDIV_DEF;  /* -> 1.0 ms/div                */
static int      osc_fwin      = 5;         /* moving average window (1..21 คี่) */
static int      osc_trig_fall = 0;         /* 0 = rising, 1 = falling          */
static int      osc_sidx      = 0;         /* setting ที่กำลังปรับ              */
static int      osc_c1        = 53, osc_c2 = 213;   /* v85: LCD x of the C1/C2 lines (0..SCREEN_W-1, 1 px per step; were sample indices 40 / 160) */
static uint8_t  osc_cur_dirty = 0;         /* v85: C1/C2 moved while frozen: repaint only the cursor columns */
static uint8_t  osc_calib     = 1;         /* AC coupling (calibrate offset)   */
static uint8_t  osc_dirty     = 1;         /* สั่งวาดใหม่แม้อยู่โหมด HOLD         */

/* โหมดจอ (ดูโหมดที่ใช้อยู่ได้ที่บรรทัด Trig: ในหน้ารายงาน / บนจอ LCD) */
static uint8_t  osc_stable    = 1;   /* v56: 1 = AUTO (default): lock to the trigger edge from 45 Hz up, free-running below / without an edge | 0 = ROLL: never lock (UART 'm r') */
static uint8_t  slow_trig_on  = 0;   /* v86: 1 = this SLOW frame is locked to an edge (header AUTO:R/F), 0 = it scrolls (ROLL) */
static uint8_t  osc_lock_auto = 0;   /* set by the auto time base: 1 while the signal is >= 45 Hz (hysteresis 35 / 45 Hz) */   /* 1 = นิ่ง (trigger) | 0 = ไถล (roll)  */
static int      trig_hold     = 0;   /* trigger offset ล่าสุดที่ใช้          */
static int      dbg_trig_off  = 0;   /* เก็บไว้ตรวจย้อนหลังผ่าน SWD          */
static float    osc_offset_ct = (float)ADC_MID;
static float    osc_offset_v  = 0.0f;
static float    osc_fs_eff    = 100000.0f; /* sample rate สุทธิของหน้าต่างที่แสดง */
static char     osc_det[8]    = "---";     /* ชนิดคลื่นที่ตรวจได้               */
static float    osc_v[N_SAMPLES];          /* ข้อมูลหลัง calibrate + filter    */
static uint8_t  osc_frame_valid = 0;
static volatile uint8_t slow_active = 0;  /* 1 = SLOW mode (T/DIV >= 50 ms) */
static volatile uint8_t slow_fast_flag = 0; /* SLOW sees fast content inside each 1 ms tick -> signal too fast for SLOW (aliased) */
static uint8_t  osc_auto_tb = 0;           /* v56: 0 = T/DIV stays where the user put it (power-on, default); 1 = firmware picks FAST/SLOW + T/DIV continuously (only the 'A' key / PC13 long sets it). 'z' is one-shot and never sets it. */
static float    osc_raw_freq = 0.0f, osc_raw_vpp = 0.0f;
static uint8_t  adc_f_held = 0;         /* 1 = the frequency shown is the 1.5 s HOLD of an earlier reading, not measured in this frame */
static int      osc_disp_cross = 0;     /* rising zero crossings in the displayed window (0..8) */
static float    osc_raw_mean_ct = 0.0f, osc_raw_rms_ct = 0.0f, osc_raw_min_ct = 0.0f, osc_raw_max_ct = 0.0f;   /* statistics over whole cycles of the raw capture (ADC counts) */
static uint8_t  osc_raw_valid = 0;     /* 1 = the values above belong to this frame (a clean periodic signal was found) */
static float    osc_capture_tdiv = 1.0f;
static uint32_t adc_measure_n = 3200u; /* adaptive, independent of display */
static uint8_t  adc_meas_lvl = 0;      /* position in adc_meas_seq[] used for the frequency measurement capture */
static const uint8_t adc_meas_seq[3] = { 1u, 0u, 4u };   /* 800 kSa/s (original), 1.6 MSa/s, 4.8 MSa/s (3 ADCs interleaved) */
static uint8_t  adc_too_fast = 0;      /* last measurement saw a signal above the usable range of its rate */
static uint8_t  adc_fast_content = 0;  /* the 800 kSa/s-or-faster capture is full of sample-to-sample change: signal is fast, not slow */
static uint8_t  adc_over_range = 0;    /* frequency above the readout limit (shown as >2.1MHz) */
static float    adc_last_good_vpp = 0.0f; /* Vpp of the last old-style measurement */
static float    adc_last_good_f = 0.0f;   /* last confirmed reading (old measurement or display capture) */
static uint32_t adc_last_good_t = 0u, adc_legacy_t = 0u;   /* its tick; tick of the last old two-capture measurement */
volatile uint32_t dbg_hold_cnt = 0;   /* frames whose waveform was not drawn (stale or unreadable FAST): SWD */
volatile uint32_t dbg_fm_cnt = 0;
volatile uint32_t dbg_fm_fail[3];     /* fast-path failures: unreadable / changed or amplitude moved / too fast (SWD) */
volatile uint32_t dbg_pipe_cnt = 0;   /* frames whose capture was started during the previous draw (SWD) */
static volatile uint8_t adc_pend = 0;  /* a display capture is running (started by AdcPrestart) */
static int adc_pd_tdiv, adc_pd_lvl;    /* settings it was started with */
static uint32_t adc_pd_n, adc_pd_w, adc_pd_pre, adc_pd_t0, adc_pd_tick;
static float adc_pd_step;      /* frames measured from the display capture (SWD) */
static float    adc_hold_f = 0.0f;     /* last good reading, held for a short time so one failed frame does not blank the display */
static uint32_t adc_hold_t = 0;
static uint32_t adc_probe_t = 0;       /* tick of the last alias probe (at most one per second) */
static float    adc_rough_ratio = 0.0f; /* mean |sample-to-sample change| / Vpp of the last measurement capture */
static uint32_t adc_over_t = 0;        /* tick of the last 'above readout limit' frame (display keeps >2.1MHz for 1.5 s: no blinking) */
static int      osc_plot_offset = 0;

/* --- v54: MOVE (only acts while frozen). All zero = identical to v53. --- */
static int      mv_y_steps  = 0;        /* vertical offset, 0.1 div per step, +-40 (display only) */
static int      mv_x_pts    = 0;        /* horizontal pan of the frozen capture in display points (<0 = earlier in time) */
static int      mv_x_min    = 0, mv_x_max = 0;   /* pan range that the frozen capture really holds (set at freeze) */
static uint8_t  mv_axis_x   = 0;        /* 0 = MOVE:Y, 1 = MOVE:X */
static uint8_t  mv_prev_hold = 0;       /* freeze edge detector */
static uint8_t  mv_view_on  = 0;        /* 1 = the panned view mv_view_v is drawn instead of osc_v */
static uint8_t  mv_zoom_x   = 1;        /* v57: time zoom x1..x64 (frozen only; v58 ceiling) */
static uint8_t  mv_zoom_step = 0;       /* v59: ONE shared zoom step 0..6 drives both axes (x = 1<<step, y = 1<<min(step,5)) */
static uint8_t  mv_zoom_y   = 1;        /* v57: volt zoom x1..x32 (frozen only; v58 ceiling) */
static int      mv_x_q      = 0;        /* v57: pan in 1/16 display point while mv_zoom_x > 1 (mv_x_pts = floor(mv_x_q/16)) */
static float    mv_view_v[N_SAMPLES];

/* --- UART RX ring buffer --- */
static volatile uint8_t  uart_rx[UART_RX_SIZE];
static volatile uint16_t uart_head = 0, uart_tail = 0;
static uint8_t           uart_byte;
static char              uart_line[24];
static uint8_t           uart_len = 0;
static uint32_t          uart_last_tick = 0;   /* เวลารับตัวอักษรล่าสุด */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uint16_t nx = (uint16_t)((uart_head + 1) % UART_RX_SIZE);
        if (nx != uart_tail) { uart_rx[uart_head] = uart_byte; uart_head = nx; }
        HAL_UART_Receive_IT(&huart3, &uart_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(&huart3, &uart_byte, 1);
    }
}

void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }

static void UartPrint(const char *s)
{
    /* timeout สั้น ๆ: ถ้าโปรแกรมฝั่ง PC ไม่ยอมอ่านพอร์ต การส่งจะหมดเวลาเร็ว
     * (ไม่บล็อกลูปหลัก/การวาดจอเป็นวินาที) — ข้อความสั้นใช้เวลา <1 ms ที่ 115200 */
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 15);
}

static void UartInputInit(void)
{
    GPIO_InitTypeDef g = {0};

    /* USART3 อยู่ที่ PB10/PB11 ตาม CubeMX แต่ Virtual COM ของ ST-LINK ต่อกับ
     * PD8/PD9 -> ย้ายขามาที่นี่เพื่อใช้สาย USB เส้นเดิมได้เลย
     * ถ้าต่อ USB-TTL กับ PB10/PB11 อยู่ ให้คอมเมนต์บล็อกนี้ทิ้ง */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &g);

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    HAL_UART_Receive_IT(&huart3, &uart_byte, 1);
}

/* --------------------------------------------------------------------------
 * ค่าโวลต์หลัง calibrate (AC coupling) + moving average filter
 * ทำงานเหมือน calibrate_offset() + moving_average_filter() ของ simulator
 * -------------------------------------------------------------------------- */
static int OscExtPoint(int k, float *v);   /* v76: real display point k (<0 or >=N_SAMPLES) of THIS FAST frame; defined in MOVE_DATA */
static int SlowExtPoint(int k, float *v);  /* v80: the same for a SLOW frame; defined in SLOW */

/* v82: display filter window actually used. The FILTER setting (osc_fwin) is a boxcar; at the measured frequency it keeps
 * H = sin(win*pi/ppc) / (win*sin(pi/ppc)) of the amplitude (ppc = display points per period). 5 points at 23.6 ppc (2 kHz, 0.5 ms/div)
 * drew the wave 7 % too small, so it no longer matched V/DIV on the grid. The window is narrowed (by 2) until H >= 0.98.
 * Unknown frequency, or < 2 points per period (not drawable as a wave, see v81): the setting as it is. */
static int OscFiltWin(void)
{
    int win = (osc_fwin < 1) ? 1 : osc_fwin;
    if ((win % 2) == 0) win++;
    if (osc_raw_freq > 0.0f && osc_fs_eff > 0.0f) {
        float ppc = osc_fs_eff / osc_raw_freq;
        if (ppc >= 2.0f) {
            float th = 3.14159265f / ppc;
            while (win > 1 && sinf((float)win * th) / ((float)win * sinf(th)) < 0.98f) win -= 2;
        }
    }
    return win;
}

static void OscProcessFrame(void)
{
    const float k  = ADC_VREF / (float)ADC_FULL;                  /* pin volts (DC readout) */
    const float kg = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN; /* v84: input volts (trace) */
    float       s = 0.0f;
    int         win, half;

    /* osc_fs_eff belongs to the captured frame, including HOLD redraws. */

    /* Calibration (AC coupling): the offset is the mean over WHOLE cycles of the raw capture (a partial cycle biases a
     * window mean by up to ~13 % of the amplitude). No clean period -> mid-range (exact for symmetric waves). */
    if (osc_calib) {
#if ADC_INPUT_ENABLE
        (void)s;
        if (osc_raw_valid && osc_frame_valid) {
            osc_offset_ct = osc_raw_mean_ct;
        } else {
            int16_t lo = raw_samples[0], hi = raw_samples[0];
            for (int i = 1; i < N_SAMPLES; i++) { if (raw_samples[i] < lo) lo = raw_samples[i]; if (raw_samples[i] > hi) hi = raw_samples[i]; }
            osc_offset_ct = ((float)lo + (float)hi) * 0.5f;
        }
#else
        for (int i = 0; i < N_SAMPLES; i++) s += (float)raw_samples[i];
        osc_offset_ct = s / (float)N_SAMPLES;
#endif
    } else {
        osc_offset_ct = (float)ADC_MID;
    }
    osc_offset_v = (osc_offset_ct - (float)ADC_MID) * k;

    /* Moving average filter (window 1..21, คี่) */
    win  = OscFiltWin();   /* v82: odd, never shrinks the wave by > 2 % */
    half = win / 2;

    for (int i = 0; i < N_SAMPLES; i++) {
        if (win <= 1) {
            osc_v[i] = ((float)raw_samples[i] - osc_offset_ct) * kg;
        }
    }
    if (win > 1) {
        /* Middle: the same centred moving average as before. */
        float flo = 1e9f, fhi = -1e9f, rlo = 1e9f, rhi = -1e9f;
        for (int i = half; i < N_SAMPLES - half; i++) {
            float acc = 0.0f;
            for (int j = -half; j <= half; j++) acc += (float)raw_samples[i + j];
            acc /= (float)win;
            osc_v[i] = (acc - osc_offset_ct) * kg;
            if (acc < flo) flo = acc;
            if (acc > fhi) fhi = acc;
            if (raw_samples[i] < rlo) rlo = (float)raw_samples[i];
            if (raw_samples[i] > rhi) rhi = (float)raw_samples[i];
        }
        /* v74/v76 edges (the first/last `half` points have no on-screen neighbour on one side).  The old clamp
         * repeated sample 0 / 239, so the edge points stayed almost unfiltered and stood out as spikes whenever the
         * filter visibly smooths the middle (2 MHz, 500 kHz, ...).
         *  1. FAST: the capture keeps the real display points just outside the window (mv_ext_lo/hi, same
         *     resampling as raw_samples) -> the edge point gets the SAME centred filter as the middle. Exact.
         *  2. no outside data (SLOW, sim): if the filter changes the middle by > 10 % slide the window inward
         *     (edge averages `win` real samples like the middle -> no spike), else shrink the centred window. */
        uint8_t crushed = (fhi - flo) < 0.9f * (rhi - rlo);
        for (int e = 0; e < 2 * half; e++) {
            int i = (e < half) ? e : (N_SAMPLES - 2 * half + e);
            float acc = 0.0f, ev;
            int j, ok = 1;
            for (j = -half; j <= half; j++) {
                int m = i + j;
                if (m >= 0 && m < N_SAMPLES) acc += (float)raw_samples[m];
                else if (OscExtPoint(m, &ev)) acc += ev;
                else { ok = 0; break; }
            }
            if (ok) { osc_v[i] = (acc / (float)win - osc_offset_ct) * k; continue; }
            int a, n;
            if (crushed) { a = (i < half) ? 0 : (N_SAMPLES - win); n = win; }
            else         { int hh = (i < half) ? i : (N_SAMPLES - 1 - i); a = i - hh; n = 2 * hh + 1; }
            acc = 0.0f;
            for (int m = a; m < a + n; m++) acc += (float)raw_samples[m];
            osc_v[i] = (acc / (float)n - osc_offset_ct) * k;
        }
    }
}

/* --------------------------------------------------------------------------
 * วัดค่า Vmax/Vmin/Vpp/RMS/Freq + ตรวจชนิดคลื่นจาก crest factor
 * เหมือน extract_features() + detect_waveform_type() ของ simulator
 * -------------------------------------------------------------------------- */
static void OscMeasureFrame(void)
{
    float vmin = 1e9f, vmax = -1e9f, s2 = 0.0f, prev = 0.0f, peak = 0.0f;
    float cross[8];
    int   ncross = 0;

    for (int i = 0; i < N_SAMPLES; i++) {          /* รอบ 1: min / max / RMS / peak */
        float v = osc_v[i];
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
        s2 += v * v;
        if (fabsf(v) > peak) peak = fabsf(v);
    }

    /* รอบ 2 (p42): นับขอบตัดศูนย์แบบ "มี hysteresis" — กัน noise สร้างขอบปลอม
     * เดิมนับทุกครั้งที่ข้าม 0: ที่ความถี่ต่ำ (หน้าต่างมี 1-2 คาบ) noise ทำให้ได้ขอบเกิน
     * -> คาบสั้นผิด (อ่านได้ 2 เท่า) หรือได้ขอบไม่ครบ -> อ่านได้ 0
     * ใหม่: หลังนับขอบหนึ่ง ต้องรอให้สัญญาณแกว่งพ้นครึ่งตรงข้ามก่อน จึงนับขอบถัดไป */
    {
        float hyst = 0.2f * (vmax - vmin);          /* ±20% ของ Vpp */
        int   armed = 1;
        if (hyst < 1e-6f) hyst = 1e-6f;
        prev = osc_v[0];
        for (int i = 1; i < N_SAMPLES; i++) {
            float v = osc_v[i];
            int up = (!osc_trig_fall && prev < 0.0f && v >= 0.0f);
            int dn = ( osc_trig_fall && prev >= 0.0f && v < 0.0f);
            if (armed && (up || dn)) {
                float f = (v != prev) ? (-prev / (v - prev)) : 0.0f;
                if (ncross < 8) cross[ncross++] = ((float)i - 1.0f + f) / osc_fs_eff;
                armed = 0;
            } else if (!armed) {
                if (osc_trig_fall ? (v > hyst) : (v < -hyst)) armed = 1;
            }
            prev = v;
        }
    }
    osc_disp_cross = ncross;

#if ADC_INPUT_ENABLE
    {   /* Mx / Mn / Rs / Vp from the UNFILTERED data: the display smoothing filter and the averaging used to draw fast
         * signals must not shrink the numbers (a 5-point filter alone costs ~6 % at 1 kHz). */
        const float kv = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;   /* v84: input volts, same scale as the trace */
        if (osc_raw_valid && osc_frame_valid) {            /* whole cycles of the raw capture */
            vmax = (osc_raw_max_ct - osc_raw_mean_ct) * kv;
            vmin = (osc_raw_min_ct - osc_raw_mean_ct) * kv;
            s2   = osc_raw_rms_ct * kv;  s2 = s2 * s2 * (float)N_SAMPLES;
        } else if (osc_env_on && !slow_active && osc_frame_valid) {   /* v83: no clean period, peak-detect frame: the band (the drawn points are only the mean) */
            float hi = 0.0f, lo = 1e9f;
            for (int i = 0; i < N_SAMPLES; i++) {
                if ((float)osc_env_hi_ct[i] > hi) hi = (float)osc_env_hi_ct[i];
                if ((float)osc_env_lo_ct[i] < lo) lo = (float)osc_env_lo_ct[i];
            }
            vmax = (hi - osc_env_mean_ct) * kv;
            vmin = (lo - osc_env_mean_ct) * kv;
            s2   = osc_env_rms_ct * kv;  s2 = s2 * s2 * (float)N_SAMPLES;
        } else {                                            /* no clean period: whole unfiltered window */
            vmax = -1e9f; vmin = 1e9f; s2 = 0.0f;
            for (int i = 0; i < N_SAMPLES; i++) {
                float v = ((float)raw_samples[i] - osc_offset_ct) * kv;
                if (v > vmax) vmax = v;
                if (v < vmin) vmin = v;
                s2 += v * v;
            }
        }
        peak = (fabsf(vmax) > fabsf(vmin)) ? fabsf(vmax) : fabsf(vmin);
    }
#endif
    OSC_LOCK();
    osc_meas.vmax = vmax;
    osc_meas.vmin = vmin;
    osc_meas.vpp  = vmax - vmin;
    osc_meas.vrms = sqrtf(s2 / (float)N_SAMPLES);
    osc_meas.dc_offset = osc_offset_v;              /* DC ก่อนทำ AC coupling */
    osc_meas.freq = 0.0f;
    if ((vmax - vmin) < 0.05f) {                    /* p43: สัญญาณเล็กเกิน (เช่นยังไม่ต่อสาย)
                                                     * -> ไม่รายงานความถี่ (เดิมโชว์ค่า noise) */
        osc_meas.freq = 0.0f;
    } else if (ncross >= 2) {                       /* วัดคาบจากจุดตัดศูนย์ */
        float span = cross[ncross - 1] - cross[0];
        if (span > 0.0f) osc_meas.freq = (float)(ncross - 1) / span;
    }

#if ADC_INPUT_ENABLE
    osc_meas.freq = osc_frame_valid ? osc_raw_freq : 0.0f;
#endif
    OSC_UNLOCK();
#if ADC_INPUT_ENABLE
    /* v52: analog input bandwidth compensation.  The ADC input (source resistance + sample-and-hold capacitor) acts as
     * a one-pole low-pass at about ADC_INPUT_FC: measured on the board, 2.1 MHz read 0.87 V for a ~1.93 V signal.
     * The raw samples are already that small, so the software cannot "see" more; instead the DISPLAYED numbers are
     * scaled back by the known filter gain at the measured frequency.  Below ~100 kHz the factor is < 1.005 (no change);
     * SLOW mode frequencies (< 100 Hz) give exactly 1.  Only osc_meas (screen/UART) is scaled - the raw values used by
     * auto time base, hold and trigger logic are untouched. */
    OSC_LOCK();
    {
        float g = 1.0f;
        if (osc_meas.freq > 0.0f) {
            float r = osc_meas.freq / ADC_INPUT_FC;
            g = sqrtf(1.0f + expf(ADC_INPUT_N * logf(r)));
            if (g > ADC_INPUT_GMAX) g = ADC_INPUT_GMAX;
        }
        osc_meas.vmax *= g; osc_meas.vmin *= g; osc_meas.vpp *= g; osc_meas.vrms *= g;
        peak *= g;
    }
    OSC_UNLOCK();
#endif
    if (peak < 0.001f) {
        strcpy(osc_det, "---");
    } else {
        float crest = osc_meas.vrms / peak;          /* sine .707 sq 1.0 tri .577 */
        if      (crest > 0.85f) strcpy(osc_det, "SQR");
        else if (crest > 0.65f) strcpy(osc_det, "SIN");
        else                    strcpy(osc_det, "TRI/SAW");
    }
}

/* --------------------------------------------------------------------------
 * Function generator: สร้าง raw_samples[] 12-bit (0..4095) เหมือน
 * generate_adc_data() ของ simulator + โหมด roll (ข้อมูลใหม่เข้าขวา)
 * -------------------------------------------------------------------------- */
static void GenFrame(void)
{
    const float w = 6.28318531f;
    uint32_t    sum = 2166136261u;
    float       dtwin = (float)OSC_PUSH / osc_fs_eff;    /* เวลาต่อเฟรม (คุมการไถล) */

    gen_ph = fmodf(gen_ph + w * gen_freq * dtwin, w);

    for (int i = 0; i < N_SAMPLES; i++) {
        float ph  = gen_ph + w * gen_freq * (float)i / osc_fs_eff;
        float cyc = ph / w;
        float frac = cyc - floorf(cyc);
        float sig;

        switch (gen_wave) {
            case 1:  sig = (sinf(ph) >= 0.0f) ? 1.0f : -1.0f; break;   /* SQUARE  */
            case 2:  sig = 4.0f * fabsf(frac - 0.5f) - 1.0f;   break;   /* TRI     */
            case 3:  sig = 2.0f * frac - 1.0f;                 break;   /* SAW     */
            default: sig = sinf(ph);                           break;   /* SINE    */
        }
        /* noise แบบ gaussian โดยประมาณ (ผลรวม 3 ตัวแปรสุ่มสม่ำเสมอ, sd = 1) */
        float nz  = SimNoise() + SimNoise() + SimNoise();
        float volt = gen_dc + gen_amp * sig + gen_noise * nz;          /* โวลต์ */
        float raw  = volt / ADC_VREF * (float)ADC_FULL;

        if (raw < 0.0f)            raw = 0.0f;
        if (raw > (float)ADC_FULL) raw = (float)ADC_FULL;
        raw_samples[i] = (int16_t)(raw + 0.5f);
        sum = (sum ^ (uint32_t)(uint16_t)raw_samples[i]) * 16777619u;
    }

#if SHOW_FPS
    dbg_sig_hash = sum;
    dbg_t0_ms    = HAL_GetTick();
#endif
}

/* --------------------------------------------------------------------------
 * MENU: SET วน setting, UP/DOWN ปรับค่า (เหมือน active_settings/osc_adjust)
 * -------------------------------------------------------------------------- */
static int OscSetCount(void)
{
    return hold_active ? OSC_HOLD_COUNT : OSC_SET_COUNT;
}

static const char *OscSetName(void)
{
    return hold_active ? osc_hold_names[osc_sidx % OSC_HOLD_COUNT]
                       : osc_set_names[osc_sidx % OSC_SET_COUNT];
}

static int OscCursorSelected(void)   /* v85: frozen and the setting being adjusted is C1 or C2 */
{
    return hold_active && (!strcmp(OscSetName(), "C1") || !strcmp(OscSetName(), "C2"));
}

static int OscClampI(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float OscClampF(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void OscSetValueText(char *out, int n)   /* "FILTER:5" / "TRIG:RISE" ... */
{
    const char *name = OscSetName();
    if      (!strcmp(name, "FILTER")) snprintf(out, n, "FILT:%d", osc_fwin);
    else if (!strcmp(name, "TRIG"))   snprintf(out, n, "TRIG:%s", osc_trig_fall ? "FALL" : "RISE");
    else if (!strcmp(name, "V/DIV"))  snprintf(out, n, "V:%.4f", (double)osc_vdivs[osc_vdiv_i]);
    else if (!strcmp(name, "T/DIV"))  snprintf(out, n, "T:%.4f", (double)osc_tdivs_ms[osc_tdiv_i]);
    else if (!strcmp(name, "C1"))     snprintf(out, n, "C1:%d", osc_c1);
    else if (!strcmp(name, "C2"))     snprintf(out, n, "C2:%d", osc_c2);
    else                              snprintf(out, n, "--");
}

static void OscSetIndex(int idx)
{
    OSC_SET(osc_sidx, idx % OscSetCount());
    if (osc_sidx < 0) OSC_SET(osc_sidx, 0);
}

static uint8_t osc_auto_stage;      /* defined with the AutoScale state machine below */

static void OscAdjust(int dir)      /* dir = +1 / -1 */
{
    const char *name = OscSetName();

    if (!strcmp(name, "FILTER")) {
        int v = osc_fwin + dir * 2;
        if ((v % 2) == 0) v += dir;
        osc_fwin = OscClampI(v, 1, 21);
    } else if (!strcmp(name, "TRIG")) {
        osc_trig_fall = !osc_trig_fall;
    } else if (!strcmp(name, "V/DIV")) {
        osc_vdiv_i = OscClampI(osc_vdiv_i + dir, 0, (int)(sizeof(osc_vdivs) / sizeof(float)) - 1);
        volts_per_div = osc_vdivs[osc_vdiv_i];
    } else if (!strcmp(name, "T/DIV")) {
        /* v77: T/DIV หมุนได้เฉพาะในโซนของโหมดปัจจุบัน (FAST = 0..OSC_NT_FAST-1, SLOW = OSC_NT_FAST..OSC_NT-1)
         * -> ปรับ T/DIV สูง<->ต่ำ ไม่ข้ามเส้น 10 ms เอง โหมด FAST/SLOW จึงคงเดิมตามที่ผู้ใช้ตั้งไว้
         *    (เปลี่ยนโหมดได้ด้วยคีย์ 'F' / ปุ่ม PB3 / ปุ่มน้ำเงิน PC13 เท่านั้น ซึ่งจะกระโดด T/DIV ไปโซนใหม่ให้) */
        int in_slow = (osc_tdiv_i >= OSC_NT_FAST);
        int lo = in_slow ? OSC_NT_FAST : 0;
        int hi = in_slow ? (OSC_NT - 1) : (OSC_NT_FAST - 1);
        osc_tdiv_i = OscClampI(osc_tdiv_i + dir, lo, hi);
        /* New timebase takes effect only on the next capture. */
        OSC_SET(osc_auto_tb, 0);            /* manual choice wins until 'z' (AutoScale) re-arms AUTO */
        osc_auto_stage = 0;                 /* a pending one-shot AutoScale would otherwise overwrite this T/DIV (and flip FAST/SLOW) a few frames later */
    } else if (!strcmp(name, "C1")) {
        osc_c1 = OscClampI(osc_c1 + dir, 0, SCREEN_W - 1);   /* v85: 1 LCD pixel (was 5 samples = 6.7 px), holding repeats every 20 ms */
    } else if (!strcmp(name, "C2")) {
        osc_c2 = OscClampI(osc_c2 + dir, 0, SCREEN_W - 1);
    }
    if (hold_active && (!strcmp(name, "C1") || !strcmp(name, "C2"))) osc_cur_dirty = 1;   /* v85: only the cursor columns change */
    else OSC_SET(osc_dirty, 1);             /* ให้วาดใหม่แม้อยู่ใน HOLD */
}

/* --------------------------------------------------------------------------
 * พิมพ์สถานะออกพอร์ต (เหมือน panel ฝั่งขวาของ simulator) — เลขจำนวนเต็ม mV
 * เพื่อเลี่ยงปัญหา %f ของ newlib-nano
 * -------------------------------------------------------------------------- */
static void UartNum(const char *label, float v, const char *unit)
{
    char b[32];
    int  n = (int)(v * 1000.0f);                 /* mV */
    int a = (n < 0) ? -n : n;    /* ค่าสัมบูรณ์ (n/1000 ของ -516 = 0 ต้องเติม - เอง) */
    snprintf(b, sizeof(b), "%s%s%d.%03d%s", label, (n < 0) ? "-" : "", a / 1000, a % 1000, unit);
    UartPrint(b);
}

/* One AutoScale state machine for UART z and long SET. Consume a fresh,
 * valid capture only; frequency and amplitude come from the raw acquisition. */
static uint8_t osc_auto_stage = 0;
static uint8_t osc_auto_tries = 0;   /* frames without a valid frequency */
volatile float dbg_auto_fsel = 0.0f;

static void OscAutoScale(void)
{
    if (hold_active) return; /* HOLD never replaces or retimes its capture. */
    osc_auto_stage = 1;
    /* v56: one-shot. 'z' does not touch osc_auto_tb (continuous auto time base only via 'A') */
    osc_auto_tries = 0;
    OSC_SET(osc_dirty, 1);
    UartPrint("\r\n AutoScale...\r\n");
}

static float SlowProbeFreq(int ti);          /* defined in the SLOW block */
static uint8_t SlowSlowish(void);            /* defined in the SLOW block: 1 = the newest history is a clearly moving, clearly slow signal */

static uint8_t SlowRecentFlat(void);   /* 1 = the newest 1.5 s of the slow history is flat (the signal has stopped) */
/* HOLD_BEGIN =============================================================
 * Do not draw a frame the board knows is wrong:
 *  1. a frame captured under another time base than the one now selected (the auto time base switched while it was being processed): stale, skip it;
 *  2. FAST has a real signal (>= 0.30 V) it cannot read (a slow wave looks like a straight line in its 60 ms window, e.g. 101 -> 1 Hz): keep the last good
 *     picture for at most 0.5 s while the board switches;
 *  3. the amplitude collapsed (< 0.30 V) within 2 s of a good picture (the generator blanks its output while a digit is edited): keep the picture up to 2 s;
 *     fast signals (adc_fast_content), SLOW frames, a changed time base and a collapse that lasts longer than 2 s are drawn.
 * Returns 1 = skip the waveform of this frame. Readable FAST frames record the time/base of the last good picture. */
/* The frequency of a FAST frame is only trusted when it was measured in this frame, or when the picture agrees with it. A held reading (1.5 s memory that keeps F steady while a
 * 100 Hz signal is read on every other frame) used to keep "F: 101 Hz" on the screen for up to 1.5 s after the signal changed to 1 Hz, while the window showed a sloping straight line:
 * the board believed it could read the signal, so it neither switched nor held the picture. A held F that promises >= 3 cycles in the window but finds < 2 crossings is not trusted. */
static uint8_t OscFreqTrusted(void)
{
    float cyc;
    if (osc_meas.freq <= 0.0f) return 0;
    if (!adc_f_held) return 1;
    cyc = osc_meas.freq * osc_capture_tdiv * (float)OSC_H_DIVS / 1000.0f;
    return (cyc >= 3.0f && osc_disp_cross < 2) ? 0 : 1;
}
#define HOLD_MAX_MS 500u
#define HOLD_COLLAPSE_MS 2000u   /* the generator blanks its output while a digit is edited (board log: 1.8 s of 0.06-0.5 V wiggles between 101 Hz and 1 Hz) */
static uint32_t osc_good_t    = 0;      /* tick of the last drawn readable FAST frame (0 = none) */
static int      osc_good_tdiv = -1;     /* its time base */
static uint8_t OscFrameHold(uint32_t now)
{
    if (osc_capture_tdiv != osc_tdivs_ms[osc_tdiv_i]) return 1;                              /* 1: captured under another base */
    if (slow_active) { osc_good_t = 0; return 0; }
    if (OscFreqTrusted()) { osc_good_t = now ? now : 1u; osc_good_tdiv = osc_tdiv_i; return 0; }
    if (osc_good_t && osc_good_tdiv == osc_tdiv_i) {
        uint32_t age = now - osc_good_t;
        if (osc_raw_vpp >= 0.30f) { if (age < HOLD_MAX_MS && !adc_fast_content) return 1; }  /* 2: unreadable real signal: keep the picture */
        else if (age < HOLD_COLLAPSE_MS) return 1;                                           /* 3: the signal collapsed to a flat sloping line after a good picture: keep it for up to 2 s */
    }
    return 0;
}
/* HOLD_END ================================================================ */


/* AUTO_TB_BEGIN =============================================================
 * AUTO time base: เลือก FAST/SLOW และ T/DIV เองตามความถี่ที่วัดได้ (ปิดเมื่อผู้ใช้ปรับ T/DIV เอง; 'z' เปิดใหม่)
 *   FAST (burst DMA)  วัดความถี่ได้ >= ~40 Hz       SLOW (SysTick ring) วัดได้ ~0.2-60 Hz
 *   กฎ:  FAST + f<45 Hz  -> SLOW (เลือก T/DIV ให้เห็น >= 3 คาบ และ f <= Fs/8)
 *        SLOW + f>55 Hz  -> FAST (เลือก T/DIV ให้เห็น ~2.5 คาบ)      (45-55 Hz = hysteresis กันสลับไปมา)
 *        FAST อ่านไม่ได้ -> ถามประวัติ SLOW (ADC3 เก็บตลอดเวลา) ทุก 250 ms: ถ้าอ่านได้ -> สลับทันที ไม่ต้องรอจอเต็ม
 *        SLOW อ่านไม่ได้ -> ลอง base อื่นจากประวัติ ถ้าไม่ได้ 1.5 s -> FAST
 *   ไม่มีสัญญาณ = ไม่เปลี่ยนอะไร  |  HOLD / AutoScale กำลังทำงาน = ไม่ทำงาน
 * ========================================================================== */
#define ATB_FAST_TO_SLOW_HZ  70.0f
#define ATB_SLOW_TO_FAST_HZ  90.0f
#define ATB_BAD_MS           1500u
#define ATB_FAST_CYC         6.0f     /* FAST: the window should show about this many cycles (SLOW shows 5-9 at the hand-off) */

static uint32_t atb_t_switch  = 0;      /* tick of the last switch */
static uint32_t atb_t_bad     = 0;      /* tick when SLOW first had a signal it could not read (0 = no) */
static uint32_t atb_t_probe   = 0;      /* tick of the last history probe */
static float    atb_cand_f    = 0.0f;   /* SLOW repick candidate: a new base is picked only after 3 consistent readings (a single bogus reading used to park a 1 Hz signal on 10 ms/div for 4 s) */
static uint8_t  atb_cand_n    = 0;
static uint32_t atb_t_unread  = 0;      /* tick when FAST first saw a real signal it could not read (a slow wave looks like a straight line in its 60 ms window) */
static uint32_t atb_t_hold    = 0;      /* tick of the last SLOW -> FAST switch caused by a fast signal: FAST needs ~1 s for its first reading and the history still holds the OLD slow signal */
static uint32_t atb_t_flat    = 0;      /* tick when SLOW first saw a flat window (no signal) */
static uint8_t  atb_slowish_n = 0;      /* consecutive probes that saw a slow moving signal while FAST could read nothing */
static float    atb_last_f    = 0.0f;   /* last frequency that was read in either mode (used by the manual toggle) */
static uint8_t  atb_last_fast = 0xFFu;  /* last FAST time base that gave a reading (0xFF = none yet) */
static uint32_t atb_last_fast_t = 0;    /* tick when it was set */
static int AtbFastFallback(uint32_t now)   /* recent FAST base, else 1 ms/div (a flat line at 5 us/div hides everything below 100 kHz) */
{ return (atb_last_fast != 0xFFu && (now - atb_last_fast_t) < 15000u) ? (int)atb_last_fast : OSC_TDIV_DEF; }
volatile uint8_t dbg_atb_switches = 0;  /* number of switches (SWD) */

static int AtbPickFast(float f)         /* FAST base ที่ได้ ~2.5 คาบ */
{
    int ti = 0; float best = 1.0e30f;
    for (int k = 0; k < OSC_NT_FAST; k++) {
        float d = fabsf(f * osc_tdivs_ms[k] * (float)OSC_H_DIVS / 1000.0f - ATB_FAST_CYC);
        if (d < best) { best = d; ti = k; }
    }
    return ti;
}

static int AtbPickSlow(float f)         /* SLOW base for frequency f. One base covers a WIDE range, so neighbouring frequencies (4, 5, 6 Hz; 11 -> 1 Hz; 20 -> 10 Hz) keep the same
                                         * time base and the picture just stretches like the signal does. The frequency is measured on whichever window can read it. */
{
    if (f <  0.9f)  return OSC_NT - 1;     /* 0.2 - 0.9 Hz : 500 ms/div */
    if (f < 12.0f)  return OSC_NT - 2;     /* 0.9 - 12 Hz  : 200 ms/div (1 Hz = 2 cycles, 12 Hz = 24 cycles) */
    if (f < 45.0f)  return OSC_NT - 4;     /* 12 - 45 Hz   :  50 ms/div */
    return OSC_NT - 6;                     /* 45 Hz and up :  10 ms/div (4.5 cycles at 45 Hz, 9 at 90 Hz: close to what FAST shows) */
}

volatile uint8_t dbg_atb_why = 0, dbg_atb_from = 0, dbg_atb_to = 0;   /* last switch: reason code, old base, new base */
volatile float   dbg_atb_arg = 0.0f;                                  /* the frequency that decided it */
volatile uint32_t dbg_atb_log[8];                                      /* ring of the last 8 switches: (why<<16)|(from<<8)|to */
volatile uint8_t  dbg_atb_log_i = 0;
volatile float    dbg_atb_arg_log[8];                                  /* the frequency that decided each logged switch */
static void AtbSwitch(int ti)
{
    if (ti == osc_tdiv_i) return;
    dbg_atb_from = (uint8_t)osc_tdiv_i; dbg_atb_to = (uint8_t)ti;
    dbg_atb_arg_log[dbg_atb_log_i & 7u] = dbg_atb_arg;
    dbg_atb_log[dbg_atb_log_i & 7u] = ((uint32_t)dbg_atb_why << 16) | ((uint32_t)osc_tdiv_i << 8) | (uint32_t)ti; dbg_atb_log_i++;
    osc_tdiv_i   = ti;
    atb_t_switch = HAL_GetTick();
    atb_t_bad    = 0;
    atb_t_flat   = 0;
    atb_t_unread = 0;
    atb_cand_n   = 0;
    OSC_SET(osc_dirty   , 1);
    dbg_atb_switches++;
}

/* time bases the history probe tries, best first: 100, 50, 200, 500 ms/div */
static const uint8_t atb_probe_order[6] = { OSC_NT_FAST + 3, OSC_NT_FAST + 2, OSC_NT_FAST + 4, OSC_NT_FAST + 5, OSC_NT_FAST + 1, OSC_NT_FAST + 0 };

/* first SLOW time base whose history window shows a clean frequency (-1 = none); *f_out = that frequency */
static int AtbProbeSlow(int skip, float *f_out)
{
    for (uint8_t k = 0; k < 6u; k++) {
        int ti = atb_probe_order[k];
        if (ti == skip) continue;
        float pf = SlowProbeFreq(ti);
        if (pf > 0.0f) { *f_out = pf; return ti; }
    }
    return -1;
}

static void OscAutoTimebase(void)       /* called every frame after measuring */
{
    uint32_t now;
    float f, cyc, pf;
    int ti;

    if (osc_meas.freq > 0.0f && osc_frame_valid) { atb_last_f = osc_meas.freq; if (osc_meas.freq >= 45.0f) osc_lock_auto = 1; else if (osc_meas.freq < 35.0f) osc_lock_auto = 0; if (!slow_active) { atb_last_fast = (uint8_t)osc_tdiv_i; atb_last_fast_t = HAL_GetTick(); } }
    if (!osc_auto_tb || hold_active || osc_auto_stage || !osc_frame_valid) return;

    now = HAL_GetTick();
    if ((now - atb_t_switch) < 300u) return;                  /* let the new setting produce a frame first */

    /* SLOW saw a fast signal: back to FAST at once */
    if (slow_active && slow_fast_flag) { atb_t_bad = 0; dbg_atb_why = 1; dbg_atb_arg = 0.0f; AtbSwitch(AtbFastFallback(now)); atb_t_hold = now; return; }

    f = osc_meas.freq;
    if (!slow_active && !OscFreqTrusted()) f = 0.0f;           /* FAST: a held F whose picture shows no cycles counts as unreadable */
    if (f > 0.0f) {                                           /* readable */
        atb_t_bad = 0; atb_t_flat = 0; atb_t_unread = 0;
        cyc = f * osc_tdivs_ms[osc_tdiv_i] * (float)OSC_H_DIVS / 1000.0f;
        if (!slow_active) {
            if (f < ATB_FAST_TO_SLOW_HZ)        { dbg_atb_why = 4; dbg_atb_arg = f; AtbSwitch(AtbPickSlow(f)); }
            else if (cyc < 3.0f || cyc > 14.0f) { dbg_atb_why = 5; dbg_atb_arg = f; AtbSwitch(AtbPickFast(f)); }
        } else {
            if (f > ATB_SLOW_TO_FAST_HZ)        { dbg_atb_why = 2; dbg_atb_arg = f; AtbSwitch(AtbPickFast(f)); atb_t_hold = now; }
            else if (cyc < 1.6f || f > (24000.0f / osc_tdivs_ms[osc_tdiv_i]) / 9.0f || (osc_tdiv_i == OSC_NT - 1 && f > 1.2f)) {
                if (atb_cand_n && fabsf(f - atb_cand_f) <= 0.10f * atb_cand_f) atb_cand_n++; else { atb_cand_f = f; atb_cand_n = 1; }
                if (atb_cand_n >= 3u) { atb_cand_n = 0; dbg_atb_why = 3; dbg_atb_arg = f; AtbSwitch(AtbPickSlow(f)); }
            } else atb_cand_n = 0;
        }
        return;
    }

    atb_cand_n = 0;
    /* not readable in the current mode */
    if (!slow_active) {
        /* FAST cannot see signals below ~40 Hz (they look flat in its 4-80 ms window). The history can: ask it, no waiting. */
        if ((now - atb_t_hold) < 1500u) return;                                        /* just came here because of a fast signal: give FAST time to read it; the history is stale */
        if (adc_fast_content) { atb_t_unread = 0; return; }                            /* clearly a fast signal right now */
        if (osc_raw_vpp < 0.30f) { atb_t_unread = 0; }                                 /* silence or a 0.15-0.2 Vpp noise band: stay (the FAST guards use the same 0.30 V) */
        else if (!atb_t_unread) atb_t_unread = now;
        else if ((now - atb_t_unread) >= 700u) {                                       /* 0.7 s of a signal FAST cannot read (101 -> 1 Hz: a straight line): show the history view now,
                                                                                        * the number follows when two periods exist. Readable frames reset the timer, so a 100 Hz signal that FAST reads on every other frame never gets here. */
            atb_t_unread = 0; dbg_atb_why = 12; dbg_atb_arg = 0.0f; AtbSwitch(OSC_NT - 2); return;
        }
        if ((now - atb_t_probe) < 250u) return;
        atb_t_probe = now;
        ti = AtbProbeSlow(-1, &pf);
        if (ti >= 0 && pf < ATB_FAST_TO_SLOW_HZ) { dbg_atb_why = 6; dbg_atb_arg = pf; atb_slowish_n = 0; AtbSwitch(AtbPickSlow(pf)); }
        else if (ti < 0 && SlowSlowish()) {                    /* nothing readable yet, but the history shows a slow moving signal (1 Hz after 0 Hz / after a fast signal):
                                                                * show it NOW in SLOW so the new wave rolls in from the right; the number follows when two periods exist */
            if (++atb_slowish_n >= 2u) { dbg_atb_why = 10; dbg_atb_arg = 0.0f; atb_slowish_n = 0; AtbSwitch(OSC_NT - 2); }
        } else atb_slowish_n = 0;
        return;
    }

    /* SLOW, current base shows nothing clean */
    if (osc_raw_vpp < 0.05f) {                                /* flat window = no signal */
        atb_t_bad = 0;
        /* PARK: after 2.4 s of silence (longer than the 2 s window of 200 ms/div, so the screen is already flat and nothing visibly changes) move to the 200 ms/div base.
         * The next slow signal then rolls in from the right at a base that fits 1-12 Hz, instead of filling a 0.5 s window in half a second and zooming 3 s later. */
        if (!atb_t_flat) atb_t_flat = now;
        else if ((now - atb_t_flat) >= 2400u && osc_tdiv_i < (OSC_NT - 2)) { atb_t_flat = 0; dbg_atb_why = 11; dbg_atb_arg = 0.0f; AtbSwitch(OSC_NT - 2); }
        return;
    }
    atb_t_flat = 0;
    if (SlowRecentFlat()) { atb_t_bad = 0; return; }          /* the signal just stopped (1 Hz -> 0 Hz): stay, let the old trace scroll out of the window by itself - no jump to FAST */
    if (!atb_t_bad) atb_t_bad = now;
    {   /* a window of 5 s at 500 ms/div needs ~2-3 s after a change (3 Hz -> 1 Hz) before two clean new periods exist: wait that long */
        uint32_t lim = 8000u;                                     /* the longest window (5 s) needs ~2.2 periods of the slowest signal we read (0.4 Hz) */
        if ((now - atb_t_bad) >= lim) { atb_t_bad = 0; dbg_atb_why = 9; dbg_atb_arg = 0.0f; AtbSwitch(AtbFastFallback(now)); }   /* SLOW cannot read it: FAST will */
    }
}

/* PC13 short press: switch FAST <-> SLOW by hand and pause AUTO. Long press re-arms AUTO. */
static void OscManualToggle(void)
{
    int ti;
    float pf = 0.0f;
    OSC_SET(osc_auto_tb, 0);
    if (!slow_active) {                                       /* FAST -> SLOW */
        ti = AtbProbeSlow(-1, &pf);
        if (ti >= 0)                                  ti = AtbPickSlow(pf);
        else if (atb_last_f > 0.0f && atb_last_f < ATB_SLOW_TO_FAST_HZ) ti = AtbPickSlow(atb_last_f);
        else                                          ti = OSC_NT_FAST + 4;      /* 200 ms/div */
    } else {                                                  /* SLOW -> FAST */
        ti = (atb_last_f >= ATB_SLOW_TO_FAST_HZ) ? AtbPickFast(atb_last_f) : ((atb_last_fast != 0xFFu) ? (int)atb_last_fast : OSC_TDIV_DEF);
    }
    AtbSwitch(ti);
}

static void OscAutoRearm(void)
{
    OSC_SET(osc_auto_tb , 1);
    atb_t_switch = HAL_GetTick();
    atb_t_bad    = 0;
    OSC_SET(osc_dirty   , 1);
}
/* AUTO_TB_END ============================================================== */

/* MOVE_BEGIN ================================================================
 * v54: the four button roles (FREEZE, AUTO, UP/LEFT, DOWN/RIGHT) and the extra keys. One Act*() per role; UART keys and buttons call the same function.
 *   MOVE acts ONLY while frozen (hold_active). Not frozen: UP/DOWN keep their v53 job (adjust the setting chosen with 's').
 *   Y move: vertical offset added at draw time (0.1 div per step, +-4 div).  X move: pan of the frozen capture (1/40 screen per step) inside the data that was captured.
 *   Numbers (Vpp / F) are never touched by moving. Unfreeze, AUTO and '0' reset both offsets.
 *   F = force FAST<->SLOW (turns the auto time base off, = the PC13 short press), A = automatic switching again (= the PC13 long press).
 * ========================================================================== */
#define MV_Y_MAX_STEPS   40                  /* +-4 div */
#define MV_X_STEP_PTS    (N_SAMPLES / 40)    /* 1/40 screen = 6 display points */
#define AS_MIN_VPP       0.05f               /* below this Vpp there is no signal */
#define AS_MIN_CYC       1.95f               /* v56: one-shot AUTO, real-scope style: the fastest FAST base that shows >= 2 cycles (2-5 on screen) */

static float MvShownV(int idx)               /* the voltage of display point idx as drawn (panned view while X-moved) */
{
    return (mv_view_on ? mv_view_v : osc_v)[((idx % N_SAMPLES) + N_SAMPLES) % N_SAMPLES];
}

/* v85: cursor readouts. C1/C2 are LCD x positions; point i is drawn at x = i*SCREEN_W/N_SAMPLES and joined by straight
 * lines, so the voltage under pixel x is the line between its two neighbouring points. */
static float OscCursorV(int x)
{
    float s = (float)x * (float)N_SAMPLES / (float)SCREEN_W;
    int   i = (int)s;
    float f = s - (float)i;
    if (i >= N_SAMPLES - 1) return MvShownV(N_SAMPLES - 1 + osc_plot_offset);
    return MvShownV(i + osc_plot_offset) * (1.0f - f) + MvShownV(i + 1 + osc_plot_offset) * f;
}
static float OscCursorDtS(void)               /* C2 - C1 in seconds (time zoom included) */
{
    return (float)(osc_c2 - osc_c1) * (float)N_SAMPLES / (float)SCREEN_W / osc_fs_eff / (float)mv_zoom_x;
}

static void MvReset(void)                    /* offsets back to zero: the picture is the captured one again */
{
    mv_y_steps = 0; mv_x_pts = 0; mv_view_on = 0;
    mv_zoom_x = 1; mv_zoom_y = 1; mv_zoom_step = 0; mv_x_q = 0;      /* v57: zoom back to x1 */
    OSC_SET(osc_dirty, 1);
}

static void MvSync(void)                     /* freeze / unfreeze edge: called from every Act*() and once per main-loop pass */
{
    if (hold_active && !mv_prev_hold)      { mv_y_steps = 0; mv_x_pts = 0; mv_view_on = 0; mv_zoom_x = 1; mv_zoom_y = 1; mv_zoom_step = 0; mv_x_q = 0; MvFreezeCapture(); }
    else if (!hold_active && mv_prev_hold) { MvReset(); mv_x_min = mv_x_max = 0; }
    mv_prev_hold = hold_active;
}

/* v57 ZOOM (frozen only). X: the view is 1/N of the screen around the screen centre; the pan (1/16 display point) may go as far as the stored data allows.
 * Left edge of the view = 120 - 120/N + p >= -nlo, right edge = 120 + 119/N + p <= 239 + nhi (N = 1 gives the v54 range). */
static void MvSetPanQ(int q)
{
    int zx = mv_zoom_x;
    q = OscClampI(q, mv_x_min * 16 - (1920 - 1920 / zx), mv_x_max * 16 + (1904 - 1904 / zx));
    mv_x_q = q;
    mv_x_pts = (q >= 0) ? (q / 16) : -((-q + 15) / 16);
}

static void ActZoom(int dir)                 /* '+' (dir +1) / '-' (dir -1): only while frozen; v59: BOTH axes together, MOVE axis not involved */
{
    MvSync();
    if (!hold_active) return;
    int st = OscClampI((int)mv_zoom_step + (dir > 0 ? 1 : -1), 0, 6);   /* time x1..x64 (step 0..6), volts x1..x32 (capped at step 5) */
    if (st != mv_zoom_step) {
        int zx = 1 << st, zy = 1 << ((st > 5) ? 5 : st);
        int q = (mv_zoom_x > 1) ? mv_x_q : mv_x_pts * 16;
        int xchg = (zx != mv_zoom_x);
        mv_zoom_step = (uint8_t)st; mv_zoom_y = (uint8_t)zy; mv_zoom_x = (uint8_t)zx;
        if (xchg) {
            if (zx > 1) MvSetPanQ(q);
            else { int r = q + 8; mv_x_pts = OscClampI((r >= 0) ? (r / 16) : -((-r + 15) / 16), mv_x_min, mv_x_max); mv_x_q = 0; }
        }
        MvBuildView();
    }
    OSC_SET(osc_dirty, 1);
}

static void MvFmtT(char *b, int n, float t_ms)       /* header T field: effective T/DIV (T/DIV / zoom), 4 decimals */
{
    if (t_ms < 0.1f) snprintf(b, (size_t)n, "T:%.4fus", (double)(t_ms * 1000.0f));
    else             snprintf(b, (size_t)n, "T:%.4fms", (double)t_ms);
}

static void MvFmtV(char *b, int n, float v)          /* header V field: effective V/DIV (V/DIV / zoom), 4 decimals */
{
    snprintf(b, (size_t)n, "V:%.4fV", (double)v);
}

static void MvFmtZ(char *b, int n)                   /* header Z field */
{
    if (mv_zoom_x > 1 && mv_zoom_y > 1 && mv_zoom_x != mv_zoom_y) snprintf(b, (size_t)n, "Z:%d/%d", mv_zoom_x, mv_zoom_y);   /* v59: time/volt only when they differ (x64/x32) */
    else if (mv_zoom_x > 1 && mv_zoom_x == mv_zoom_y) snprintf(b, (size_t)n, "Z:x%d   ", mv_zoom_x);
    else if (mv_zoom_x > 1)             snprintf(b, (size_t)n, "Z:x%d   ", mv_zoom_x);
    else if (mv_zoom_y > 1)             snprintf(b, (size_t)n, "Z:x%d   ", mv_zoom_y);
    else                                snprintf(b, (size_t)n, "        ");
}

static void MvMove(int dir, int n)           /* dir +1 = UP / LEFT, -1 = DOWN / RIGHT; only called while frozen */
{
    if (n < 1) n = 1;
    if (!mv_axis_x) {
        mv_y_steps = OscClampI(mv_y_steps + dir * n, -MV_Y_MAX_STEPS, MV_Y_MAX_STEPS);
    } else if (mv_zoom_x > 1) {                          /* v57: zoomed: finer steps (1/40 of the VIEW), range = the stored data */
        MvSetPanQ(mv_x_q - dir * n * ((MV_X_STEP_PTS * 16 + mv_zoom_x / 2) / mv_zoom_x));
        MvBuildView();
    } else {
        int p = OscClampI(mv_x_pts - dir * n * MV_X_STEP_PTS, mv_x_min, mv_x_max);
        if (p != mv_x_pts) { mv_x_pts = p; MvBuildView(); }
    }
    OSC_SET(osc_dirty, 1);
}

static void ActToggleStable(void)            /* v63: PC3 short / PF3 = same as UART 'm' bare toggle */
{
    OSC_SET(osc_stable, (uint8_t)!osc_stable);
    OSC_SET(osc_dirty , 1);
}

static void ActFreeze(void)                  /* FREEZE button / 'h' */
{
    OSC_SET(hold_active, !hold_active);
    OSC_SET(osc_sidx   , 0);
    OSC_SET(osc_dirty  , 1);
    MvSync();
}

static void ActAuto(void)                    /* AUTO button / 'z': unfreeze, offsets to zero, auto scale */
{
    if (hold_active) { OSC_SET(hold_active, 0); OSC_SET(osc_sidx, 0); }
    MvSync();
    MvReset();
    AutoScale();                             /* = OscAutoScale() */
}

static void ActUp(int n)                     /* UP/LEFT button / 'u' (n = steps, >1 only for a long hold) */
{
    MvSync();
    if (hold_active) MvMove(+1, n); else OscAdjust(+1);
}

static void ActDown(int n)                   /* DOWN/RIGHT button / 'd' */
{
    MvSync();
    if (hold_active) MvMove(-1, n); else OscAdjust(-1);
}

/* v61 physical + / - buttons: ALWAYS OscAdjust (setting adjust), never MvMove/zoom, even while frozen.
 * Move/zoom/freeze stay UART-only (8-button design); UART 'u'/'d' still use ActUp/ActDown which branch on hold_active. */
static void ActIncrease(int n)
{
    MvSync();
    for (int k = OscCursorSelected() ? n : 1; k > 0; k--) OscAdjust(+1);   /* v85: a long hold moves C1/C2 n samples per repeat */
}

static void ActDecrease(int n)
{
    MvSync();
    for (int k = OscCursorSelected() ? n : 1; k > 0; k--) OscAdjust(-1);
}

/* v61/v62: select a setting by name in the CURRENTLY ACTIVE name array (osc_hold_names while frozen, osc_set_names otherwise).
 * Name not in the active array = silent no-op (osc_sidx untouched). */
static void ActSelectSetting(const char *name)
{
    const char * const *names = hold_active ? osc_hold_names : osc_set_names;
    int n = OscSetCount();
    int i;
    for (i = 0; i < n; i++) {
        if (!strcmp(names[i], name)) { OscSetIndex(i); return; }
    }
}

/* v61 TRIGGER button: flip the edge directly, no need to select TRIG first. Works frozen or not. */
static void ActToggleTrigEdge(void)
{
    osc_trig_fall = !osc_trig_fall;
    OSC_SET(osc_dirty, 1);
}

static void ActToggleMoveAxis(void)          /* 'x' */
{
    mv_axis_x = (uint8_t)!mv_axis_x;
    OSC_SET(osc_dirty, 1);
}

static void ActMoveCenter(void)              /* '0' */
{
    MvSync();
    MvReset();
}

static void ActToggleFastSlow(void)          /* 'F' / PC13 short: force FAST <-> SLOW (opposite of the current one); auto time base off. Ignored while frozen. */
{
    if (hold_active) return;
    OscManualToggle();
}

static void ActAutoMode(void)                /* 'A' / PC13 long: automatic FAST/SLOW switching again */
{
    OscAutoRearm();
}

/* AUTO scale, real-scope style.
 * V/DIV : the smallest step that keeps Vpp within OSC_V_FILL_DIVS divisions (the screen shows ~5.3 divisions).
 * T/DIV : chosen with the SAME rules as the auto time base (AtbPickSlow below ATB_FAST_TO_SLOW_HZ, FAST with >= AS_MIN_CYC cycles),
 *         so the auto time base agrees with it and does not move it a moment later.
 * No signal (Vpp < AS_MIN_VPP): 1 V/div and the default FAST time base. */
static int OscAutoPickTdiv(float f)
{
    int want_slow = slow_active ? (f <= ATB_SLOW_TO_FAST_HZ) : (f < ATB_FAST_TO_SLOW_HZ);
    if (want_slow) return AtbPickSlow(f);
    for (int k = 0; k < OSC_NT_FAST; k++)
        if (f * osc_tdivs_ms[k] * (float)OSC_H_DIVS / 1000.0f >= AS_MIN_CYC) return k;
    return OSC_NT_FAST - 1;
}

static void OscAutoApply(float vpp, int ti)
{
    int vi = OSC_VDIV_DEF;                               /* default 1 V/div */
    if (vpp >= AS_MIN_VPP) {
        vi = (int)(sizeof(osc_vdivs) / sizeof(osc_vdivs[0])) - 1;
        for (int i = 0; i <= vi; i++) { if (osc_vdivs[i] * OSC_V_FILL_DIVS >= vpp) { vi = i; break; } }
    }
    osc_vdiv_i = vi;
    volts_per_div = osc_vdivs[vi];
    osc_tdiv_i = ti; /* captured time metadata changes only on acquisition */
    osc_auto_stage = 0;
    OSC_SET(osc_dirty, 1);
}

static void OscAutoScaleStep(void)
{
    if (!osc_auto_stage || hold_active || !osc_frame_valid || !new_frame_ready) return;
    float f = osc_meas.freq;
    float vpp = ADC_INPUT_ENABLE ? osc_raw_vpp * ADC_INPUT_GAIN : osc_meas.vpp;   /* v84: input volts, like the trace */
    if (f > 0.0f) {
        dbg_auto_fsel = f;
        OscAutoApply(vpp, OscAutoPickTdiv(f));
        return;
    }
    /* no periodic reading (yet) */
    if (osc_auto_tries < 250u) osc_auto_tries++;
    if (slow_active && vpp < AS_MIN_VPP) { OscAutoApply(0.0f, OSC_TDIV_DEF); return; }    /* the SLOW window is flat: no signal */
    if (!slow_active && osc_auto_tries == 15u) { osc_tdiv_i = OSC_NT_FAST + 4; OSC_SET(osc_dirty, 1); }   /* FAST sees only >= 40 Hz: look at the SLOW 200 ms/div window (2 s) */
    else if (slow_active && osc_auto_tries > 100u) OscAutoApply(vpp, OSC_TDIV_DEF);        /* not periodic: scale V/DIV, default FAST time base */
}
/* MOVE_END ================================================================= */

static void UartPanel(void)
{
    char b[96];
    UartPrint("\r\n== OSCILLOSCOPE =============================");
    OscSetValueText(b, sizeof(b));
    UartPrint("\r\n Setting: "); UartPrint(b);
    for (int i = 0; i < OscSetCount(); i++) {
        int save = osc_sidx;
        OSC_SET(osc_sidx, i);
        OscSetValueText(b, sizeof(b));
        OSC_SET(osc_sidx, save);
        UartPrint((i == osc_sidx) ? "\r\n  > " : "\r\n    ");
        UartPrint(b);
    }
    snprintf(b, sizeof(b), "\r\n HOLD:%s  CAL:%s  Trig:%s-%s  Det:%s",
             hold_active ? "ON" : "OFF", osc_calib ? "AC" : "DC",
             osc_stable ? "AUTO" : "ROLL", osc_trig_fall ? "FALL" : "RISE", osc_det);
    UartPrint(b);
    UartPrint(osc_auto_tb ? "\r\n TimeBase: CONTINUOUS AUTO (A; changing T/DIV = fixed)" : "\r\n TimeBase: FIXED (z = one-shot autoscale, A = continuous auto)");
    snprintf(b, sizeof(b), "\r\n Mode:%s%s  MOVE:%c (frozen only)  offY:%d (0.1 div)  offX:%d pts",
             slow_active ? "SLOW" : "FAST", osc_auto_tb ? "" : "*", mv_axis_x ? 'X' : 'Y', mv_y_steps, mv_x_pts);
    UartPrint(b);
    snprintf(b, sizeof(b), "\r\n Zoom: X x%d  Y x%d  (frozen only; + in / - out, both axes together)", mv_zoom_x, mv_zoom_y);   /* v57 */
    UartPrint(b);

    UartPrint("\r\n== FUNCTION GENERATOR (input) ================");
    snprintf(b, sizeof(b), "\r\n Wave:%s  Freq:%d Hz",
             gen_wave_names[gen_wave], (int)gen_freq);
    UartPrint(b);
    UartNum("\r\n Amp:", gen_amp, " V");  UartNum("  DC:", gen_dc, " V");
    UartNum("  Noise:", gen_noise, " V");
    snprintf(b, sizeof(b), "\r\n V/div:%d (%d.%d V)  T/div:%d (%d.%d ms)  Fs:%d Sa/s",
             osc_vdiv_i, (int)osc_vdivs[osc_vdiv_i],
             (int)(osc_vdivs[osc_vdiv_i] * 10.0f) % 10,
             osc_tdiv_i, (int)(osc_tdivs_ms[osc_tdiv_i]), (int)(osc_tdivs_ms[osc_tdiv_i] * 10.0f) % 10,
             (int)(osc_fs_eff / 1000.0f));
    UartPrint(b);

    UartPrint("\r\n== MEASUREMENTS ==============================");
    if (!osc_frame_valid) UartPrint("\r\n ADC ERR: measurements invalid; retrying acquisition");
    OscMeasure sm; OscMeasGet(&sm);   /* consistent snapshot for the readout */
    UartNum("\r\n Vmax:", sm.vmax, " V");  UartNum("   Vmin:", sm.vmin, " V");
    UartNum("\r\n Vpp: ", sm.vpp,  " V");  UartNum("   RMS: ", sm.vrms, " V");
    UartNum("\r\n DC:  ", sm.dc_offset, " V");
    snprintf(b, sizeof(b), "   Freq: %d Hz", (int)(sm.freq + 0.5f));   /* p44: ปัด */
    UartPrint(b);
    if (hold_active) {
        UartNum("\r\n U1:", OscCursorV(osc_c1), " V");  UartNum("   U2:", OscCursorV(osc_c2), " V");   /* v85: pixel cursors */
        UartNum("   dV:", OscCursorV(osc_c2) - OscCursorV(osc_c1), " V");
        snprintf(b, sizeof(b), "   dt: %d us", (int)(OscCursorDtS() * 1e6f));
        UartPrint(b);
    }
    UartNum("\r\n Calib offset:", osc_offset_ct, " cts");
    UartNum("   Dev:", osc_offset_ct - (float)ADC_MID, " cts");
    UartPrint("\r\n----------------------------------------------");
    UartPrint("\r\n keys: s=SET u=UP d=DOWN h=HOLD g=gen m=auto/roll z=autoscale ?=status");
    UartPrint("\r\n v54:  h=freeze z=auto u/d=up/left,down/right (frozen=move) x=MOVE:Y/X 0=recenter F=FAST<>SLOW(forced*) A=auto FAST/SLOW +/-=zoom in/out (frozen, MOVE axis)");
    UartPrint("\r\n ctrl: m a=auto(นิ่ง) m r=roll(ไถล) | พิมพ์คีย์เดี่ยวได้เลยไม่ต้อง Enter");
    UartPrint("\r\n  w=wave f<Hz> a<V> o<Vdc> n<Vnoise> v[+|-] t[+|-] c1<n> c2<n> cal<0|1>");
    UartPrint("\r\n> ");
}

static float UartArgF(const char *s, float def)
{
    if (!s || !*s) return def;
    if (*s == '+') return def;
    if (*s == '-') return def;
    return (float)atof(s);
}

static void UartCommand(char *line)
{
    char *tok = strtok(line, " ");
    char *arg = strtok(NULL, " ");
    int   has = (arg && *arg);

    if (!tok || !*tok) return;

    switch (tok[0]) {
        case 's': OscSetIndex(osc_sidx + 1); break;                     /* SET   */
        case 'u': if (has) OscSetIndex(atoi(arg)); else ActUp(1); break;   /* UP    (frozen: move up/left) */
        case 'd': ActDown(1); break;                                    /* DOWN  (frozen: move down/right) */
        case 'h': case 'r': ActFreeze(); break;                         /* HOLD  */
        case 'x': ActToggleMoveAxis(); break;                           /* MOVE:Y <-> MOVE:X */
        case '0': ActMoveCenter(); break;                               /* re-centre */
        case '+': ActZoom(+1); break;                                   /* v57: zoom in (frozen only) */
        case '-': ActZoom(-1); break;                                   /* v57: zoom out (frozen only) */
        case 'F': ActToggleFastSlow(); break;                           /* force FAST <-> SLOW (upper case: 'f' is the generator frequency) */
        case 'A': ActAutoMode(); break;                                 /* automatic FAST/SLOW again (upper case: 'a' is the generator amplitude) */
        case 'g':                                                       /* gen ใหม่ */
            gen_ph = fmodf(gen_ph + 1.7f, 6.28318531f);                  /* = reseed */
            OSC_SET(osc_dirty, 1);
            break;
        case 'w':                                                       /* waveform */
            if (has) {
                /* ไม่สนตัวพิมพ์: sin|s=0  sq|square=1  tri|t=2  saw=3  0..3 */
                char a0 = (arg[0] >= 'A' && arg[0] <= 'Z') ? (char)(arg[0] + 32) : arg[0];
                char a1 = (arg[1] >= 'A' && arg[1] <= 'Z') ? (char)(arg[1] + 32) : arg[1];
                if      (a0 >= '0' && a0 <= '3')        gen_wave = a0 - '0';
                else if (a0 == 's' && a1 == 'q')        gen_wave = 1;   /* SQUARE */
                else if (a0 == 's' && a1 == 'a')        gen_wave = 3;   /* SAW    */
                else if (a0 == 't')                     gen_wave = 2;   /* TRI    */
                else if (a0 == 's' || a0 == 'w')        gen_wave = 0;   /* SINE   */
                else                                    gen_wave = (gen_wave + 1) % 4;
            } else {
                gen_wave = (gen_wave + 1) % 4;
            }
            OSC_SET(osc_dirty, 1);
            break;
        case 'f': gen_freq  = OscClampF(has ? UartArgF(arg, gen_freq) : gen_freq + 100.0f, 100.0f, 5000.0f); OSC_SET(osc_dirty, 1); break;
        case 'a': gen_amp   = OscClampF(has ? UartArgF(arg, gen_amp)  : gen_amp  + 0.05f, 0.10f, 1.50f); OSC_SET(osc_dirty, 1); break;
        case 'o': gen_dc    = OscClampF(has ? UartArgF(arg, gen_dc)   : gen_dc   + 0.05f, 0.50f, 2.50f); OSC_SET(osc_dirty, 1); break;
        case 'n': gen_noise = OscClampF(has ? UartArgF(arg, gen_noise): gen_noise+ 0.005f, 0.0f, 0.30f); OSC_SET(osc_dirty, 1); break;
        case 'v': OscSetIndex(2); OscAdjust((arg && *arg == '-') ? -1 : +1); break;
        case 't': OscSetIndex(3); OscAdjust((arg && *arg == '-') ? -1 : +1); break;
        case 'c':
            if (tok[1] == '1' || tok[1] == '2') {
                int ci = (tok[1] == '1') ? 1 : 2;
                OSC_SET(osc_sidx, hold_active ? (ci == 1 ? 0 : 1) : osc_sidx);
                if (has) {
                    int v = OscClampI(atoi(arg), 0, SCREEN_W - 1);   /* v85: LCD x (pixels) */
                    if (ci == 1) osc_c1 = v; else osc_c2 = v;
                }
            } else if (tok[1] == 'a') {                                 /* cal 0/1 */
                osc_calib = (uint8_t)(has ? atoi(arg) : !osc_calib);
            }
            if (hold_active && (tok[1] == '1' || tok[1] == '2')) osc_cur_dirty = 1;   /* v85: cursor columns only */
            else OSC_SET(osc_dirty, 1);
            break;
        case 'm': {                                                     /* โหมดจอ */
            const char *a = has ? arg : tok + 1;       /* "m a" | "ma" | "m" */
            if (*a) {
                char a0 = (a[0] >= 'A' && a[0] <= 'Z') ? (char)(a[0] + 32) : a[0];
                if      (a0 == 'a') OSC_SET(osc_stable, 1);                  /* auto = นิ่ง  */
                else if (a0 == 'r') OSC_SET(osc_stable, 0);                  /* roll = ไถล   */
                else                OSC_SET(osc_stable, (uint8_t)(atoi(a) ? 1 : 0));
            } else {
                OSC_SET(osc_stable, (uint8_t)!osc_stable);
            }
            OSC_SET(osc_dirty, 1);
            break;
        }
        case 'z': case 'Z': ActAuto(); break;                            /* auto scale */
        case '?': default: break;
    }
    UartPanel();
}

static void UartPoll(void)
{
    while (uart_tail != uart_head) {
        char c = (char)uart_rx[uart_tail];
        uart_tail = (uint16_t)((uart_tail + 1) % UART_RX_SIZE);

        if (c == '\r' || c == '\n') {
            if (uart_len) { uart_line[uart_len] = 0; UartCommand(uart_line); uart_len = 0; }
            continue;
        }
        /* ทำทันทีเฉพาะคีย์ที่ไม่มีรูปแบบ "มีค่า" เท่านั้น (s/u/d/h/g/m ต้องรอ idle) */
        if (uart_len == 0 && (c == '+' || c == '-')) {         /* v57: zoom keys; "v +" / "t -" start with a letter, so a leading +/- is free */
            uart_line[0] = c; uart_line[1] = 0;
            UartCommand(uart_line);
            continue;
        }
        if (uart_len == 0 && strchr("sdhg?zxFA0", c)) {
            uart_line[0] = c; uart_line[1] = 0;
            UartCommand(uart_line);
            continue;
        }
        if (uart_len < (uint8_t)(sizeof(uart_line) - 1)) {
            uart_line[uart_len++] = c;
            uart_last_tick = HAL_GetTick();
        }
    }

    /* 'u' = UP หรือ "u 2" | 'm' = สลับโหมด หรือ "m a"/"m r" | 'z' = autoscale
     * => รอให้หยุดพิมพ์ UART_IDLE_MS ก่อน จึงถือว่าเป็นคำสั่งแบบเดี่ยว
     * บรรทัดที่ค้างโดยไม่มี \r จะถูกทิ้งเมื่อเงียบนาน UART_DROP_MS
     * (ไม่งั้น uart_len ไม่กลับเป็น 0 -> คีย์เดี่ยวทุกตัวหยุดทำงานถาวร)          */
    if (uart_len) {
        uint32_t idle = HAL_GetTick() - uart_last_tick;
        if (idle >= UART_IDLE_MS && uart_len == 1 && strchr("umz", uart_line[0])) {
            uart_line[1] = 0;
            UartCommand(uart_line);
            uart_len = 0;
        } else if (idle >= UART_DROP_MS) {
            uart_len = 0;                       /* ทิ้งบรรทัดค้าง กัน UI ค้างถาวร */
        }
    }
}

/* ==========================================================================
 * SimSamplingStream — จำลอง ADC ที่ "สุ่มข้อมูลต่อเนื่องตลอดเวลา" (mock)
 *
 *   ต่างจาก GenerateFakeSineWave() ตรงที่ของเดิมผูกเฟสกับ "เลขเฟรม"
 *   (mock_phase += 0.15 ทุกเฟรม) => ภาพซ้ำเป็นลูปทุก ~42 เฟรม เหมือน GIF
 *
 *   ตัวนี้ผูกกับ "เวลาจริง": หน้าต่างที่แสดงคือ N_SAMPLES ตัวอย่างที่เวลา
 *   [t, t + 2.4 ms] โดย t เดินตาม HAL_GetTick() => ทุกเฟรมเป็นข้อมูลชุดใหม่
 *   ที่ต่อเนื่องกัน และมี noise + แอมพลิจูด/ดีซี/ความถี่ลอยช้า ๆ + โทนที่ 2
 *   (1370 Hz ไม่ลงตัวกับ 1 kHz) => มองกี่นาทีก็ไม่เห็นภาพเดิมซ้ำ
 *
 *   ลบทั้งบล็อกนี้เมื่อทีม sampling ต่อข้อมูลจริงเข้าบัฟเฟอร์
 * ========================================================================== */


#define SIM_FS_HZ        100000.0f  /* สมมติ sample rate 100 kSa/s (หน้าต่าง 2.4 ms) */
#define SIM_FREQ_HZ      1000.0f    /* ความถี่ตั้งต้นของ function generator          */
#define SIM_NOISE        0.05f      /* noise floor (±5% ของครึ่งสเกล)                */
#define SIM_SCALE        0.62f      /* สเกล: 1 หน่วยสัญญาณ -> สัดส่วนของ ADC_MID    */
#define SIM_T_WRAP       3000.0f    /* วนเวลาสะสมที่ 3000 s (ทุกความถี่ลงตัวพอดี) */

/* โหมดจอ: สลับตอนรันได้ด้วยคีย์ m (หรือ m a / m r / m 1 / m 0)
 *   1 = จัดแนว trigger ให้กราฟนิ่ง (ขอบ RISE/FALL ตามค่า TRIG)
 *   0 = roll / chart-recorder: ข้อมูลใหม่เข้าขวา เส้นไถลซ้าย                 */
#define SIM_PUSH         8          /* sample ใหม่ต่อเฟรม (มาก=เลื่อนเร็ว/เป็นขั้น) */

static float    sim_t    = 0.0f;    /* เวลาจริงสะสม (วินาที)                     */
static float    sim_ph1  = 0.0f;    /* เฟสสะสมโทนหลัก (rad, 0..2pi)              */
static float    sim_ph2  = 0.0f;    /* เฟสสะสมโทนที่ 2                            */
static uint32_t sim_prev = 0;       /* tick ครั้งก่อน                             */
static uint32_t sim_seed = 0x2545F491u;

/* สัญญาณรบกวนแบบ xorshift32 -> [-1, 1] */
static float SimNoise(void)
{
    sim_seed ^= sim_seed << 13;
    sim_seed ^= sim_seed >> 17;
    sim_seed ^= sim_seed << 5;
    return ((float)((sim_seed >> 9) & 0xFFFFu) / 32768.0f) - 1.0f;
}

/* ค่าวัดทุกตัวคำนวณจากข้อมูลจริงในบัฟเฟอร์ -> ตัวเลขบนหัวจอตรงกับที่เห็น */
static void SimUpdateMeasure(void)
{
    float vmin = 1e9f, vmax = -1e9f, s1 = 0.0f, s2 = 0.0f, prev = 0.0f;
    float cross[8];
    int   ncross = 0;
    const float k = ADC_VREF / (float)ADC_FULL;

    for (int i = 0; i < N_SAMPLES; i++) {
        float v = ((float)raw_samples[i] - (float)ADC_MID) * k;
        if (v > vmax) vmax = v;
        if (v < vmin) vmin = v;
        s1 += v;
        s2 += v * v;
        if (i > 0 && prev < 0.0f && v >= 0.0f && ncross < 8) {   /* ตัดศูนย์ขึ้น */
            float f = (v != prev) ? (-prev / (v - prev)) : 0.0f;
            cross[ncross++] = ((float)i - 1.0f + f) / SIM_FS_HZ;
        }
        prev = v;
    }

    OSC_LOCK();
    osc_meas.vmax = vmax;
    osc_meas.vmin = vmin;
    osc_meas.vpp  = vmax - vmin;
    osc_meas.vrms = sqrtf(s2 / (float)N_SAMPLES);
    osc_meas.dc_offset = s1 / (float)N_SAMPLES;
    if (ncross >= 2) {                                     /* วัดคาบจากจุดตัดศูนย์ */
        float span = cross[ncross - 1] - cross[0];
        if (span > 0.0f) osc_meas.freq = (float)(ncross - 1) / span;
    }
    OSC_UNLOCK();
}

static void SimSamplingStream(void)
{
    const float w    = 6.28318531f;
    const float step = 1.0f / SIM_FS_HZ;                   /* คาบสุ่ม = 10 us */
    uint32_t    now  = HAL_GetTick();
    uint32_t    sum  = 2166136261u;                        /* FNV-1a: ตรวจว่าข้อมูลไม่ซ้ำ */

    if (sim_prev == 0) sim_prev = now;
    float dt = (float)(now - sim_prev) / 1000.0f;   /* เวลาจริงที่ผ่านไป (ใช้กับ drift) */
    float dtwin = (float)SIM_PUSH / SIM_FS_HZ;      /* เวลาหน้าต่างต่อเฟรม = ตัวคุมความเร็วไถล */
    sim_prev = now;

    /* เดินเวลาจริง + เฟสสะสม (แบบ fmod กัน float เสียความละเอียด) */
    sim_t   = fmodf(sim_t + dt, SIM_T_WRAP);
    float f1 = SIM_FREQ_HZ + 60.0f * sinf(w * 0.23f * sim_t);   /* ความถี่ลอยช้า ๆ */
    sim_ph1 = fmodf(sim_ph1 + w * f1 * dtwin, w);   /* เฟสเดินตามหน้าต่างที่เลื่อน */
    sim_ph2 = fmodf(sim_ph2 + w * 1370.0f * dtwin, w);

    float amp = 1.00f + 0.20f * sinf(w * 0.11f * sim_t + 0.7f);  /* แอมพลิจูดหายใจช้า ๆ */
    float off = 0.15f * sinf(w * 0.07f * sim_t + 1.3f);          /* DC ลอยช้า ๆ        */

    for (int i = 0; i < N_SAMPLES; i++) {
        float ti = (float)i * step;                        /* เวลาภายในหน้าต่าง */
        float v  = amp * sinf(sim_ph1 + w * f1 * ti)
                 + 0.22f * sinf(sim_ph2 + w * 1370.0f * ti) /* โทน 2 ไม่ลงตัวกับ 1 kHz */
                 + off
                 + SIM_NOISE * SimNoise();
        float raw = (float)ADC_MID * (1.0f + SIM_SCALE * v);
        if (raw < 0.0f)            raw = 0.0f;
        if (raw > (float)ADC_FULL) raw = (float)ADC_FULL;
        raw_samples[i] = (int16_t)(raw + 0.5f);
        sum = (sum ^ (uint32_t)(uint16_t)raw_samples[i]) * 16777619u;
    }

#if SHOW_FPS
    dbg_sig_hash = sum;          /* ลายนิ้วมือของเฟรมนี้ (ใช้ตรวจว่าไม่ซ้ำ) */
    dbg_t0_ms    = now;
#endif
    SimUpdateMeasure();
}
/* --------------------------------------------------------------------------
 * GenerateFakeSineWave
 * TODO: DELETE after merging with sampling teammate.
 *       Also fills osc_meas with mock values for display testing.
 * -------------------------------------------------------------------------- */
static void GenerateFakeSineWave(void)
{
    for (int i = 0; i < N_SAMPLES; i++) {
        float angle = (2.0f * 3.14159265f * (float)i / (float)N_SAMPLES) + mock_phase;
        raw_samples[i] = (int16_t)((float)ADC_MID + 1800.0f * sinf(angle));
    }
    mock_phase += 0.15f;
    if (mock_phase >= 2.0f * 3.14159265f) mock_phase -= 2.0f * 3.14159265f;

    /* Mock measurements — teammate replaces these with real calculations */
    float peak_v = 1800.0f * ADC_VREF / (float)ADC_FULL;  /* ~1.45 V */
    OSC_LOCK();
    ((OscMeasure *)&osc_meas)->vmax     =  peak_v;
    ((OscMeasure *)&osc_meas)->vmin     = -peak_v;
    ((OscMeasure *)&osc_meas)->vpp      =  peak_v * 2.0f;
    ((OscMeasure *)&osc_meas)->vrms     =  peak_v * 0.7071f;
    ((OscMeasure *)&osc_meas)->freq     =  1000.0f;
    ((OscMeasure *)&osc_meas)->dc_offset = 0.0f;
    OSC_UNLOCK();
}

/* --------------------------------------------------------------------------
 * FindTrigger
 * Scan raw_samples[] for the first rising edge crossing ADC_MID.
 * Returns the sample index to start rendering from (trigger offset).
 * -------------------------------------------------------------------------- */
static int FindTrigger(void)
{
    /* หาจุด trigger จากข้อมูลที่สอบเทียบ/กรองแล้ว (osc_v แกว่งรอบ 0 = auto level)
     * ใช้ขอบตามที่ตั้งไว้ใน TRIG: RISE = ผ่าน 0 ขาขึ้น | FALL = ผ่าน 0 ขาลง
     * ค้นแค่ 3/4 ของบัฟเฟอร์ เพื่อให้เหลือข้อมูลพอวาดเต็มจอหลังจุด trigger */
    int limit = (N_SAMPLES * 3) / 4;
    if (osc_trig_fall) {
        for (int i = 1; i < limit; i++)
            if (osc_v[i - 1] >= 0.0f && osc_v[i] < 0.0f) return i;
    } else {
        for (int i = 1; i < limit; i++)
            if (osc_v[i - 1] < 0.0f && osc_v[i] >= 0.0f) return i;
    }
    return -1;   /* หน้าต่างนี้ไม่เจอขอบ -> ใช้ค่าเดิม (ภาพไม่กระโดด) */
}

/* --------------------------------------------------------------------------
 * MapToScreen
 * Convert raw_samples[] to screen Y coordinates, starting from trig_offset.
 * Physical mapping: each grid division = GRID_Y_STEP px = volts_per_div V.
 * -------------------------------------------------------------------------- */
static void MapToScreen(int trig_offset)
{
    osc_plot_offset = trig_offset;
    const float *vs = mv_view_on ? mv_view_v : osc_v;       /* v54: panned view while frozen */
    for (int i = 0; i < N_SAMPLES; i++) {
        int src = (trig_offset + i);
        if (src >= N_SAMPLES) src -= N_SAMPLES;  /* wrap around buffer */

        /* Convert count -> voltage -> pixel offset */
        float v = vs[src];
        int y = CENTER_Y - (int)(v / volts_per_div * (float)(GRID_Y_STEP * mv_zoom_y)) - mv_y_steps * GRID_Y_STEP / 10;   /* v54: Y move (0 = v53) */

        /* Clamp to waveform area only (do not draw over header) */
        if (y < WAVEFORM_TOP) y = WAVEFORM_TOP;
        if (y >= SCREEN_H)    y = SCREEN_H - 1;
        new_y[i] = (int16_t)y;
    }
    /* v83: peak-detect band of this frame (v86: FAST and SLOW; not for a panned / zoomed frozen view, or the simulator) */
    env_y_on = (uint8_t)(osc_env_on && !mv_view_on);
    if (env_y_on) {
        const float kc = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;   /* v84: same scale as osc_v */
        for (int i = 0; i < N_SAMPLES; i++) {
            int src = (trig_offset + i);
            if (src >= N_SAMPLES) src -= N_SAMPLES;
            for (int e = 0; e < 2; e++) {
                float v = ((float)(e ? osc_env_lo_ct[src] : osc_env_hi_ct[src]) - osc_offset_ct) * kc;
                int y = CENTER_Y - (int)(v / volts_per_div * (float)(GRID_Y_STEP * mv_zoom_y)) - mv_y_steps * GRID_Y_STEP / 10;
                if (y < WAVEFORM_TOP) y = WAVEFORM_TOP;
                if (y >= SCREEN_H)    y = SCREEN_H - 1;
                if (e) env_y_hi[i] = (int16_t)y; else env_y_lo[i] = (int16_t)y;
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * DrawGrid
 * Draw sparse dot-pattern grid once (at startup or after full screen clear).
 * Grid rows: y = 40, 80, 120(centre), 160, 200 (GRID_Y_OFFSET + n*GRID_Y_STEP)
 * -------------------------------------------------------------------------- */
static void DrawGrid(void)
{
    for (int y = GRID_Y_OFFSET; y < SCREEN_H; y += GRID_Y_STEP)
        for (int x = 0; y >= WAVEFORM_TOP && x < SCREEN_W; x += GRID_X_DOT)
            ILI9341_Draw_Pixel((uint16_t)x, (uint16_t)y, DARKGREY);
    /* v75: vertical dotted line every 1 T/DIV */
    for (int x = 0; x < SCREEN_W; x += GRID_X_STEP)
        for (int y = WAVEFORM_TOP; y < SCREEN_H; y++)
            if (IS_GRID_DOT(x, y)) ILI9341_Draw_Pixel((uint16_t)x, (uint16_t)y, DARKGREY);
}

/* --------------------------------------------------------------------------
 * DrawLineBresenham
 * Rasterise a line. When erasing (color == BLACK):
 *   - Restore DARKGREY at grid dot positions      (Fix #2)
 *   - Restore DARKCYAN at cursor column           (cursor preservation)
 * -------------------------------------------------------------------------- */
static int dl_only_col = -1;      /* >= 0: DrawLineBresenham paints only the pixels of this column (neighbour repair) */
static void DrawLineBresenham(int x0, int y0, int x1, int y1, uint16_t color)
{
    int dx  =  abs(x1 - x0);
    int sx  =  (x0 < x1) ? 1 : -1;
    int dy  = -abs(y1 - y0);
    int sy  =  (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (1) {
        uint16_t draw_color = color;

        if (color == BLACK) {
            /* เส้น cursor C1/C2 (ตอน HOLD) สำคัญสุด > cursor เดิม > จุด grid */
            if (hold_active && (x0 == OscCursorX(osc_c1) || x0 == OscCursorX(osc_c2))) {
                draw_color = YELLOW;
            } else if (x0 == (int)cursor_x && y0 >= WAVEFORM_TOP) {
                draw_color = DARKCYAN;
            } else if (IS_GRID_DOT(x0, y0)) {
                draw_color = DARKGREY;
            }
        }

        if (dl_only_col < 0 || x0 == dl_only_col) {
        ILI9341_Draw_Pixel((uint16_t)x0, (uint16_t)y0, draw_color);

#if SHOW_FPS
        dbg_px_frame++;
#endif
        }

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* --------------------------------------------------------------------------
 * OscCursorX / DrawCursors
 * ตำแหน่ง x ของ cursor ตาม index ของ sample (ใช้สูตรเดียวกับ MapToScreen)
 * วาดเส้น C1/C2 สีเหลืองเฉพาะตอน HOLD (เหมือน simulator)
 * -------------------------------------------------------------------------- */
static int OscCursorX(int x)          /* v85: the cursor IS an LCD x position now */
{
    if (x < 0) x = 0;
    if (x >= SCREEN_W) x = SCREEN_W - 1;
    return x;
}

static uint8_t osc_full_redraw = 1;   /* 1 = the screen was cleared since the last frame: repaint every column */
static uint8_t cursors_on = 0;      /* 1 = มีเส้น cursor ค้างอยู่บนจอ */
static int     cur_x1 = -1, cur_x2 = -1;

/* ล้างเส้น cursor เก่า (คืนจุด grid กลับด้วย) ก่อนวาดเฟรมใหม่ */
static void ClearCursors(void)
{
    if (!cursors_on) return;
    osc_full_redraw = 1;   /* the cursor columns are repainted black: the trace has holes there */
    for (int k = 0; k < 2; k++) {
        int x = k ? cur_x2 : cur_x1;
        if (x < 0) continue;
        for (int y = WAVEFORM_TOP; y < SCREEN_H; y++) {
            uint16_t col = IS_GRID_DOT(x, y) ? DARKGREY : BLACK;
            ILI9341_Draw_Pixel((uint16_t)x, (uint16_t)y, col);
        }
    }
    cursors_on = 0;
    cur_x1 = cur_x2 = -1;
}

static void DrawCursors(void)
{
    if (!hold_active) return;
    cur_x1 = OscCursorX(osc_c1);
    cur_x2 = OscCursorX(osc_c2);
    for (int y = WAVEFORM_TOP; y < SCREEN_H; y++) {
        ILI9341_Draw_Pixel((uint16_t)cur_x1, (uint16_t)y, YELLOW);
        ILI9341_Draw_Pixel((uint16_t)cur_x2, (uint16_t)y, YELLOW);
    }
    cursors_on = 1;
}

/* --------------------------------------------------------------------------
 * DrawCursor
 * Draw a solid vertical DARKCYAN line at cursor_x in the waveform area.
 * Called once at startup and whenever cursor_x changes.
 * Bresenham's erase pass will preserve this line (see above).
 * -------------------------------------------------------------------------- */
static void DrawCursor(void)
{
    for (int y = WAVEFORM_TOP; y < SCREEN_H; y++)
        ILI9341_Draw_Pixel(cursor_x, (uint16_t)y, DARKCYAN);
}

/* --------------------------------------------------------------------------
 * DrawWaveform
 * Erase previous frame (BLACK, preserving grid+cursor), draw new frame (YELLOW).
 * Fix #1: x = (i * SCREEN_W) / N_SAMPLES  — fills full 320px width.
 * -------------------------------------------------------------------------- */
/* ---- column renderer -------------------------------------------------------------------------------------------
 * Old way: one SPI transaction triple per pixel (~58 us), erase + draw + neighbour repair = 270-700 ms per frame, and the picture
 * was wiped and repainted segment by segment, so a moving trace looked torn and jumpy.
 * New way: for every screen column the trace covers one contiguous run of pixels [lo,hi] (the same pixels Bresenham gives).
 * A changed column is repainted ONCE, top to bottom, in one SPI burst (old run + new run, everything else background/grid):
 * no gaps, no stale pixels, no half-old/half-new picture, and ~10x faster. Unchanged columns are not touched. */
static uint8_t col_lo_old[SCREEN_W], col_hi_old[SCREEN_W];
static uint8_t col_lo_new[SCREEN_W], col_hi_new[SCREEN_W];

static void LcdWriteColumn(uint16_t x, uint16_t y1, uint16_t y2, const uint8_t *bytes, uint32_t n_bytes)
{
    ILI9341_Set_Address(x, y1, x, y2);
    HAL_GPIO_WritePin(LCD_DC_PORT, LCD_DC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(HSPI_INSTANCE, (uint8_t *)bytes, (uint16_t)n_bytes, 10);
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);
}

/* DRAWWF_BEGIN */
static uint16_t BackgroundColour(int x, int y)       /* what is on the screen where no trace is: cursors > grid dot > black */
{
    if (hold_active && (x == OscCursorX(osc_c1) || x == OscCursorX(osc_c2))) return YELLOW;
    if (x == (int)cursor_x && y >= WAVEFORM_TOP) return DARKCYAN;
    if (IS_GRID_DOT(x, y)) return DARKGREY;
    return BLACK;
}

static void TraceColumns(uint8_t *lo, uint8_t *hi)   /* y-run of the polyline new_y[] in every screen column */
{
    for (int x = 0; x < SCREEN_W; x++) { lo[x] = 255; hi[x] = 0; }
    for (int i = 0; i < N_SAMPLES - 1; i++) {
        int x0 = (i * SCREEN_W) / N_SAMPLES, x1 = ((i + 1) * SCREEN_W) / N_SAMPLES;
        int y0 = new_y[i], y1 = new_y[i + 1];
        int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1, dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1, err = dx + dy;
        while (1) {
            if (x0 >= 0 && x0 < SCREEN_W) {
                if (y0 < lo[x0]) lo[x0] = (uint8_t)y0;
                if (y0 > hi[x0]) hi[x0] = (uint8_t)y0;
            }
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    if (env_y_on) {                                  /* v83 peak detect: every column also covers the min..max band of its display point */
        for (int x = 0; x < SCREEN_W; x++) {
            int i = (x * N_SAMPLES) / SCREEN_W;
            if (env_y_lo[i] < lo[x]) lo[x] = (uint8_t)env_y_lo[i];
            if (env_y_hi[i] > hi[x]) hi[x] = (uint8_t)env_y_hi[i];
        }
    }
}

static void DrawWaveform(void)
{
    uint8_t buf[2 * (SCREEN_H - WAVEFORM_TOP)];
    static uint8_t inited = 0;
    if (!inited) { for (int x = 0; x < SCREEN_W; x++) { col_lo_old[x] = 255; col_hi_old[x] = 0; } inited = 1; osc_full_redraw = 1; }
    TraceColumns(col_lo_new, col_hi_new);
    for (int x = 0; x < SCREEN_W; x++) {
        int nlo = col_lo_new[x], nhi = col_hi_new[x];
        int olo = col_lo_old[x], ohi = col_hi_old[x];
        int has_new = (nlo <= nhi), has_old = (olo <= ohi);
        if (!osc_full_redraw && nlo == olo && nhi == ohi) continue;       /* column unchanged */
        int y1, y2;
        if (has_new && has_old) { y1 = (nlo < olo) ? nlo : olo; y2 = (nhi > ohi) ? nhi : ohi; }
        else if (has_new)       { y1 = nlo; y2 = nhi; }
        else if (has_old)       { y1 = olo; y2 = ohi; }
        else continue;
        if (osc_full_redraw && has_new && !has_old) { }                    /* new column on a cleared screen */
        if (y1 < WAVEFORM_TOP) y1 = WAVEFORM_TOP;
        if (y2 >= SCREEN_H)    y2 = SCREEN_H - 1;
        if (y1 > y2) continue;
        uint32_t n = 0;
        for (int y = y1; y <= y2; y++) {
            uint16_t c = (has_new && y >= nlo && y <= nhi) ? YELLOW : BackgroundColour(x, y);
            buf[n++] = (uint8_t)(c >> 8); buf[n++] = (uint8_t)c;
        }
        LcdWriteColumn((uint16_t)x, (uint16_t)y1, (uint16_t)y2, buf, n);
#if SHOW_FPS
        dbg_px_frame += n / 2u;
#endif
    }
    memcpy(col_lo_old, col_lo_new, sizeof(col_lo_old));
    memcpy(col_hi_old, col_hi_new, sizeof(col_hi_old));
    memcpy(old_y, new_y, sizeof(new_y));
    osc_full_redraw = 0;
}

/* v85: repaint one whole column of the waveform area from what DrawWaveform last drew (col_*_old) and the background
 * (cursors / grid); rows skip_y1..skip_y2 are left untouched (-1 = none). */
static void RepaintColumn(int x, int skip_y1, int skip_y2)
{
    uint8_t buf[2 * (SCREEN_H - WAVEFORM_TOP)];
    int lo = col_lo_old[x], hi = col_hi_old[x], y = WAVEFORM_TOP;
    while (y < SCREEN_H) {
        if (y >= skip_y1 && y <= skip_y2) { y = skip_y2 + 1; continue; }
        int y_end = (skip_y1 > y) ? skip_y1 - 1 : SCREEN_H - 1;
        if (y_end > SCREEN_H - 1) y_end = SCREEN_H - 1;
        uint32_t n = 0;
        for (int yy = y; yy <= y_end; yy++) {
            uint16_t c = (yy >= lo && yy <= hi) ? YELLOW : BackgroundColour(x, yy);
            buf[n++] = (uint8_t)(c >> 8); buf[n++] = (uint8_t)c;
        }
        LcdWriteColumn((uint16_t)x, (uint16_t)y, (uint16_t)y_end, buf, n);
        y = y_end + 1;
    }
}
/* DRAWWF_END */

/* --------------------------------------------------------------------------
 * DrawTopPanel
 * Render the 2-row header bar and separator line.
 *
 * Row 1 (y=1):  V/div | T/div | Trigger mode | Run/Hold status
 * Row 2 (y=10): Vpp | Freq | Vmax | Vmin | RMS
 * Separator:    horizontal DARKGREY line at y=18
 *
 * Note: ILI9341_Draw_Text takes uint8_t X — all positions below are <= 255.
 * -------------------------------------------------------------------------- */
/* --------------------------------------------------------------------------
 * DrawSettingBar
 * แถวบรรทัดล่างของจอ: บอกว่า "ตอนนี้กำลังตั้งค่าอะไร + ค่าเท่าไร + UP/DOWN ทำอะไร"
 *   ต้องวาด *หลัง* DrawWaveform เพราะรูปคลื่นลบ/วาดทับบรรทัดล่างได้
 *   ตัวดำบนแถบฟ้า เพื่อให้เด่นบนพื้นกราฟ
 * -------------------------------------------------------------------------- */
#define BAR_X       2                 /* v85: setting bar text position / glyph cell (5x5 font in a 6x8 cell) */
#define BAR_Y       (SCREEN_H - 12)
#define BAR_CHAR_W  6
#define BAR_CHAR_H  8
static char    bar_txt[40];               /* v85: the bar text now on the screen */
static uint8_t bar_ok = 0;
static void SettingBarDraw(int fast)
{
    char buf[40];
    const char *name = OscSetName();

    if      (!strcmp(name, "FILTER")) snprintf(buf, sizeof(buf), "SET FILT=%d  UP/DOWN +-2", osc_fwin);
    else if (!strcmp(name, "TRIG"))   snprintf(buf, sizeof(buf), "SET TRIG=%s  UP/DOWN TOGGLE",
                                               osc_trig_fall ? "FALL" : "RISE");
    else if (!strcmp(name, "V/DIV"))  snprintf(buf, sizeof(buf), "SET VDIV=%.1fV  UP/DOWN STEP",
                                               (double)volts_per_div);
    else if (!strcmp(name, "T/DIV"))  snprintf(buf, sizeof(buf), "SET TDIV=%.1fms  UP/DOWN STEP",
                                               (double)osc_tdivs_ms[osc_tdiv_i]);
    else if (!strcmp(name, "C1"))     snprintf(buf, sizeof(buf), "SET C1=%3d  UP/DOWN HOLD", osc_c1);   /* v85: fixed width, 1 sample steps */
    else if (!strcmp(name, "C2"))     snprintf(buf, sizeof(buf), "SET C2=%3d  UP/DOWN HOLD", osc_c2);
    else                              snprintf(buf, sizeof(buf), "SET --");

    if (fast && bar_ok && strlen(buf) == strlen(bar_txt)) {        /* v85 cursor move: only the characters that changed */
        for (int k = 0; buf[k]; k++)
            if (buf[k] != bar_txt[k]) ILI9341_Draw_Char(buf[k], (uint16_t)(BAR_X + k * BAR_CHAR_W), BAR_Y, BLACK, 1, CYAN);
    } else {
        ILI9341_Draw_Text(buf, BAR_X, BAR_Y, BLACK, 1, CYAN);
    }
    snprintf(bar_txt, sizeof(bar_txt), "%s", buf); bar_ok = 1;
    snprintf((char *)dbg_lcd_bar, sizeof(dbg_lcd_bar), "%s", buf);
}

static void DrawSettingBar(void) { SettingBarDraw(0); }

/* v85: C1/C2 moved while frozen. A full redraw repainted all 320 columns (~98 ms per step, the "freeze"); the trace
 * on the screen is known (col_lo_old / col_hi_old), so only the old and the new cursor columns are repainted, leaving
 * the setting bar's rows alone (they are drawn over the cursor), then the bar's changed characters. ~1 ms per move. */
static void CursorsMoveFast(void)
{
    int nx1 = OscCursorX(osc_c1), nx2 = OscCursorX(osc_c2);
    int xs[4] = { cur_x1, cur_x2, nx1, nx2 };
    int bw = BAR_X + (int)strlen(bar_txt) * BAR_CHAR_W;
    cur_x1 = nx1; cur_x2 = nx2; cursors_on = 1;
    for (int k = 0; k < 4; k++) {
        int x = xs[k], dup = 0;
        if (x < 0 || x >= SCREEN_W) continue;
        for (int j = 0; j < k; j++) if (xs[j] == x) dup = 1;
        if (dup) continue;
        if (bar_ok && x >= BAR_X && x < bw) RepaintColumn(x, BAR_Y, BAR_Y + BAR_CHAR_H - 1);
        else                                RepaintColumn(x, -1, -1);
    }
    SettingBarDraw(1);
    osc_cur_dirty = 0;
}

#if ADC_INPUT_ENABLE
extern volatile uint16_t dbg_adc_min, dbg_adc_max;
#endif
/* ---- header text with a character cache: draw only what changed ------------------------------------------------------- */
#define HDR_FIELDS 32
static char     hdr_txt[HDR_FIELDS][24];
static uint16_t hdr_fg[HDR_FIELDS], hdr_bg[HDR_FIELDS];
static uint8_t  hdr_ok[HDR_FIELDS];
static void HdrInvalidate(void) { memset(hdr_ok, 0, sizeof hdr_ok); }
#define HDR_CHAR_W 6          /* glyph pitch of the 5x5 font at size 1 */
static void HdrText(int id, const char *t, uint8_t x, uint8_t y, uint16_t fg, uint16_t size, uint16_t bg)
{
    if (id < 0 || id >= HDR_FIELDS) { ILI9341_Draw_Text(t, x, y, fg, size, bg); return; }
    int n = (int)strlen(t); if (n > 23) n = 23;
    int m = hdr_ok[id] ? (int)strlen(hdr_txt[id]) : 0;
    int force = (!hdr_ok[id] || hdr_fg[id] != fg || hdr_bg[id] != bg);
    for (int k = 0; k < ((n > m) ? n : m); k++) {
        char ch = (k < n) ? t[k] : ' ';
        if (!force && k < m && ch == hdr_txt[id][k]) continue;         /* this character is already on the screen */
        ILI9341_Draw_Char(ch, (uint8_t)(x + k * HDR_CHAR_W), y, fg, 1, (k < n) ? bg : BLACK);
    }
    memcpy(hdr_txt[id], t, (size_t)n); hdr_txt[id][n] = 0;
    hdr_fg[id] = fg; hdr_bg[id] = bg; hdr_ok[id] = 1;
}
#define HDRT(...) HdrText(__COUNTER__, __VA_ARGS__)
static void DrawTopPanel(void)
{
    char buf[20];
    OscMeasure tm; OscMeasGet(&tm);   /* one consistent snapshot of osc_meas for the whole header */
    /* The header is NOT cleared every update any more (that blanked every number for ~60 ms, 3 times a second = the blinking F:/Vp:
     * values). Each text keeps what is on the screen; only the characters that changed are redrawn. A change between the normal
     * and the HOLD layout clears the header once. */
    {   static uint8_t last_hold = 255u;
        if (last_hold != (uint8_t)hold_active) { ILI9341_Draw_Filled_Rectangle_Coord(0, 0, SCREEN_W - 1, HEADER_H - 2, BLACK); HdrInvalidate(); last_hold = (uint8_t)hold_active; }
    }

    /* ── Row 1 ──────────────────────────────────────────────────────────── */

    /* ตัวที่กำลังปรับ (SET) ถูกไฮไลต์ => มองจอรู้เลยว่า UP/DOWN จะแก้ค่าอะไร
     * ไม่ต้องไปดูที่หน้าจอ serial เท่านั้น                                        */
    const char *selname = OscSetName();
    uint8_t blink_on = (uint8_t)((HAL_GetTick() / (BLINK_PERIOD_MS)) & 1u);   /* v60: one shared flag => all selected fields blink in sync */
    int sel_vdiv = (strcmp(selname, "V/DIV") == 0);
    int sel_tdiv = (strcmp(selname, "T/DIV") == 0);

    /* V/div — x=2 (MAGENTA = กำลังปรับตัวนี้อยู่), always 4 decimals */
    MvFmtV(buf, sizeof(buf), (hold_active && mv_zoom_y > 1) ? volts_per_div / (float)mv_zoom_y : volts_per_div);
    HDRT(buf, 2, 1, sel_vdiv ? (blink_on ? MAGENTA : YELLOW) : YELLOW, 1, BLACK);

    /* T/div label — x=72, always 4 decimals */
    MvFmtT(buf, sizeof(buf), (hold_active && mv_zoom_x > 1) ? osc_capture_tdiv / (float)mv_zoom_x : osc_capture_tdiv);
    HDRT(buf, 72, 1, sel_tdiv ? (blink_on ? MAGENTA : WHITE) : WHITE, 1, BLACK);

    /* Setting ที่กำลังปรับ (SET เลือก, UP/DOWN เปลี่ยนค่า) — x=150
     * ตัวดำบนแถบส้ม = "ช่องที่ถูกเลือกอยู่" */
    OscSetValueText(buf, sizeof(buf));
    /* v60: blinks by inverting fg/bg (never black-on-black, text always readable) */
    if (blink_on) HDRT(buf, 150, 1, BLACK, 1, ORANGE);
    else          HDRT(buf, 150, 1, ORANGE, 1, BLACK);
    snprintf((char *)dbg_lcd_set, sizeof(dbg_lcd_set), "%s", buf);

    /* Run / Hold status — x=226 */
    if (!osc_frame_valid) {
        HDRT("ADC ERR", 226, 1, RED, 1, BLACK);
    } else if (hold_active) {
        HDRT("HOLD", 226, 1, RED, 1, BLACK);
    } else if (trigger_mode == TRIG_SINGLE && !single_captured) {
        HDRT("WAIT", 226, 1, ORANGE, 1, BLACK);
    } else {
        HDRT("RUN ", 226, 1, GREEN, 1, BLACK);
    }

    /* ── Row 2 ──────────────────────────────────────────────────────────── */

    /* Vpp — x=2 */
    if (hold_active) {
        /* ── HOLD: ค่าที่ cursor C1/C2 (เหมือน simulator) ────────────────── */
        snprintf(buf, sizeof(buf), "U1=%.2fV", (double)OscCursorV(osc_c1));
        HDRT(buf, 2, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "U2=%.2fV", (double)OscCursorV(osc_c2));
        HDRT(buf, 72, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "dV=%.2fV", (double)(OscCursorV(osc_c2) - OscCursorV(osc_c1)));
        HDRT(buf, 148, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "dt=%.2fms",
                 (double)(OscCursorDtS() * 1000.0f));   /* v57: cursor distance in time shrinks with the zoom; v85: pixel cursors */
        HDRT(buf, 2, 20, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "MOVE:%c", mv_axis_x ? 'X' : 'Y');                  /* v54: frozen only */
        HDRT(buf, 72, 20, ORANGE, 1, BLACK);
        snprintf(buf, sizeof(buf), "%s%s ", slow_active ? "SLOW" : "FAST", osc_auto_tb ? "" : "*");   /* * = forced */
        HDRT(buf, 148, 20, osc_auto_tb ? GREEN : ORANGE, 1, BLACK);
        MvFmtZ(buf, sizeof(buf));                                                        /* v57 */
        HDRT(buf, 196, 20, CYAN, 1, BLACK);
    } else {
    snprintf(buf, sizeof(buf), "Vp:%.2fV", (double)tm.vpp);
    HDRT(buf, 2, 10, CYAN, 1, BLACK);

    /* Frequency — x=68 */
    {
        char fb[16];
        if (tm.freq >= 1000000.0f)  snprintf(fb, sizeof(fb), "%.3fMHz", (double)(tm.freq / 1000000.0f));
        else if (tm.freq >= 1000.0f) snprintf(fb, sizeof(fb), "%.1fkHz", (double)(tm.freq / 1000.0f));
        else if (tm.freq >= 10.0f)  snprintf(fb, sizeof(fb), "%.0fHz",  (double)tm.freq);
        else if (tm.freq > 0.0f)    snprintf(fb, sizeof(fb), "%.2fHz",  (double)tm.freq);   /* <10 Hz: 2 decimals */
        else if (adc_over_t && (HAL_GetTick() - adc_over_t) < 1500u && !slow_active) snprintf(fb, sizeof(fb), ">2.1MHz");               /* above the readout limit */
        else                              snprintf(fb, sizeof(fb), "---");                            /* unknown, not "0 Hz" */
        snprintf(buf, sizeof(buf), "F:%-8s", fb);
    }
    HDRT(buf,72, 10, GREEN, 1, BLACK);

    /* RMS — x=245 (245 < 255, safe for uint8_t) */
    snprintf(buf, sizeof(buf), "Rs:%.2f", (double)tm.vrms);
    HDRT(buf, 148, 10, LIGHTGREY, 1, BLACK);

    /* Vmax — x=130 */
    snprintf(buf, sizeof(buf), "Mx:%.2fV", (double)tm.vmax);
    HDRT(buf, 2, 20, LIGHTGREY, 1, BLACK);

    /* Vmin — x=188 */
    snprintf(buf, sizeof(buf), "Mn:%.2fV", (double)tm.vmin);
    HDRT(buf, 72, 20, LIGHTGREY, 1, BLACK);

    /* โหมด trigger — AUTO:R / AUTO:F = จัดแนวให้นิ่ง, ROLL = ไถล */
    if (slow_active)                                         /* v86: SLOW shows its real state too: locked to an edge or scrolling */
        snprintf(buf, sizeof(buf), !osc_stable ? "ROLL  " : (slow_trig_on ? (osc_trig_fall ? "AUTO:F" : "AUTO:R") : "AUTO  "));   /* v87: AUTO with no edge yet = "AUTO" */
    else if (osc_stable && osc_lock_auto)
        snprintf(buf, sizeof(buf), "AUTO:%s", osc_trig_fall ? "F" : "R");
    else
        snprintf(buf, sizeof(buf), "ROLL");
    HDRT(buf, 148, 20, CYAN, 1, BLACK);
    HDRT(osc_auto_tb ? "AUT  " : (slow_active ? "SLOW*" : "FAST*"), 232, 20, osc_auto_tb ? GREEN : ORANGE, 1, BLACK);   /* time base: automatic / forced (F key, PC13, T/DIV) */
#if ADC_INPUT_ENABLE
    /* CLIP: ADC ชน 0 หรือ 4095 => สัญญาณเกินช่วง 0-3.3 V (ยอดคลื่นแบนจริง ไม่ใช่บั๊กจอ) */
    HDRT((osc_frame_valid && (dbg_adc_min <= 8u || dbg_adc_max >= 4087u)) ? "CLIP" : "    ",
                      200, 20, RED, 1, BLACK);
#endif
    }

//    /* RMS — x=245 (245 < 255, safe for uint8_t) */
//    snprintf(buf, sizeof(buf), "Rs:%.2f", (double)osc_meas.vrms);
//    HDRT(buf, 2, 20, LIGHTGREY, 1, BLACK);

    /* Separator line */
    ILI9341_Draw_Horizontal_Line(0, HEADER_H - 1, SCREEN_W, DARKGREY);
}
#undef HDRT

/* --------------------------------------------------------------------------
 * AutoScale
 * Adjust V/div so the waveform Vpp spans approximately 4 vertical divisions.
 * Triggered by BTN_SET held for BTN_LONG_MS.
 * -------------------------------------------------------------------------- */
static void AutoScale(void)
{
    OscAutoScale();
}
/* --------------------------------------------------------------------------
 * ButtonJustPressed
 * Returns 1 on the first confirmed press (rising edge with debounce).
 * Active-LOW buttons: GPIO reads 0 when pressed.
 * -------------------------------------------------------------------------- */
static uint8_t ButtonJustPressed(Button *btn, uint32_t now)
{
    uint8_t raw = (HAL_GPIO_ReadPin(btn->port, btn->pin) == GPIO_PIN_RESET) ? 1 : 0;

    if (raw) {
        if (!btn->is_held) {
            /* Transition to pressed */
            btn->is_held       = 1;
            btn->held_since_tick = now;
            btn->long_fired    = 0;
            if (now - btn->last_press_tick >= BTN_DEBOUNCE_MS) {
                btn->last_press_tick = now;
                return 1;   /* confirmed short press */
            }
        }
    } else {
        btn->is_held    = 0;
        btn->long_fired = 0;
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * ButtonLongPress
 * Returns 1 once when the button has been held for at least BTN_LONG_MS.
 * -------------------------------------------------------------------------- */
static uint8_t ButtonLongPress(Button *btn, uint32_t now)
{
    if (btn->is_held && !btn->long_fired) {
        if (now - btn->held_since_tick >= BTN_LONG_MS) {
            btn->long_fired = 1;
            return 1;
        }
    }
    return 0;
}

/* v54: UP/DOWN are held, not toggled. Returns the number of steps due now: 1 on the press, then 1 every BTN_REPEAT_MS after BTN_REPEAT_FIRST_MS,
 * BTN_ACCEL_STEPS per repeat once held for BTN_ACCEL_MS. raw = 1 while the pin reads pressed. */
static uint32_t btn_rep_ms = BTN_REPEAT_MS;   /* v85: repeat period, BTN_CUR_REPEAT_MS while C1/C2 is adjusted */
static int ButtonRepeat(Button *b, uint8_t raw, uint32_t now)
{
    if (!raw) {
        if (b->is_held) {
            if (!b->low_pending) { b->low_pending = 1; b->low_tick = now; }
            else if ((now - b->low_tick) >= BTN_REL_MS) { b->is_held = 0; b->low_pending = 0; }
        }
        return 0;
    }
    b->low_pending = 0;
    if (!b->is_held) {
        b->is_held = 1; b->held_since_tick = now; b->next_rep_tick = now + BTN_REPEAT_FIRST_MS;
        return 1;
    }
    if ((int32_t)(now - b->next_rep_tick) >= 0) {
        b->next_rep_tick = now + btn_rep_ms;
        return ((now - b->held_since_tick) >= BTN_ACCEL_MS) ? BTN_ACCEL_STEPS : 1;
    }
    return 0;
}

static uint8_t ButtonRaw(const Button *b)
{
    return (HAL_GPIO_ReadPin(b->port, b->pin) == GPIO_PIN_RESET) ? 1u : 0u;   /* active LOW */
}

/* --------------------------------------------------------------------------
 * HandleButtons
 * Call this every loop iteration (before frame logic).
 * Roles: HOLD pin = FREEZE | SET pin = AUTO | UP pin = UP/LEFT | DOWN pin = DOWN/RIGHT | PC13 = FAST<>SLOW (short) / AUTO FAST/SLOW (long)
 * -------------------------------------------------------------------------- */
static void HandleButtons(uint32_t now)
{
    int n;
    MvSync();

    /* v61: PB0 = INCREASE, PB1 = DECREASE (hold-repeat as before; OscAdjust only, never move/zoom) */
    btn_rep_ms = OscCursorSelected() ? BTN_CUR_REPEAT_MS : BTN_REPEAT_MS;   /* v85: C1/C2 glide; other settings keep 60 ms */
    if ((n = ButtonRepeat(&btn_up,   ButtonRaw(&btn_up),   now)) > 0) ActIncrease(n);
    if ((n = ButtonRepeat(&btn_down, ButtonRaw(&btn_down), now)) > 0) ActDecrease(n);

    /* PB2 = AUTO (one-shot auto scale) */
    if (ButtonJustPressed(&btn_hold, now)) ActAuto();
    (void)ButtonLongPress;

    /* v62: PA3 = toggle V/DIV <-> T/DIV select; PC0 = toggle FILTER <-> TRIG select (no-op where the target does not exist, e.g. frozen) */
    if (ButtonJustPressed(&btn_vdiv, now)) ActSelectSetting(strcmp(OscSetName(), "V/DIV") == 0 ? "T/DIV" : "V/DIV");
    if (ButtonJustPressed(&btn_tdiv, now)) ActSelectSetting(strcmp(OscSetName(), "FILTER") == 0 ? "TRIG" : "FILTER");
    (void)ActToggleTrigEdge;

    /* v64: PF3 = true FREEZE toggle (ActFreeze, same as UART 'h') */
    if (ButtonJustPressed(&btn_trig, now)) ActFreeze();

    /* v62: PC3 = short (on release) FREEZE | held BTN_CURSOR_LONG_MS (fires once) toggle C1<->C2 cursor (only while frozen). Same pattern as PB3 below. */
    {
        static uint8_t  cu_prev = 0, cu_long = 0;
        static uint32_t cu_t0 = 0;
        uint8_t pressed = ButtonRaw(&btn_filt);
        if (pressed && !cu_prev) { cu_t0 = now; cu_long = 0; }
        if (pressed && !cu_long && (now - cu_t0) >= BTN_CURSOR_LONG_MS) {
            cu_long = 1;
            if (hold_active) ActSelectSetting(strcmp(OscSetName(), "C1") == 0 ? "C2" : "C1");
        }
        if (!pressed && cu_prev && !cu_long && (now - cu_t0) >= 40u) ActToggleStable();
        cu_prev = pressed;
    }

    /* PB3 = FAST/SLOW: short = toggle (on release), held BTN_MAN_LONG_MS = back to automatic. Same pattern as PC13 below. */
    {
        static uint8_t  fs_prev = 0, fs_long = 0;
        static uint32_t fs_t0 = 0;
        uint8_t pressed = ButtonRaw(&btn_set);
        if (pressed && !fs_prev) { fs_t0 = now; fs_long = 0; }
        if (pressed && !fs_long && (now - fs_t0) >= BTN_MAN_LONG_MS) { fs_long = 1; ActAutoMode(); }
        if (!pressed && fs_prev && !fs_long && (now - fs_t0) >= 40u) ActToggleFastSlow();
        fs_prev = pressed;
    }

    /* PC13: manual FAST/SLOW. Acts on release (short) or after 1 s held (back to AUTO). */
    {
        static uint8_t  man_prev = 0, man_long = 0;
        static uint32_t man_t0 = 0;
        uint8_t pressed = (HAL_GPIO_ReadPin(BTN_MAN_PORT, BTN_MAN_PIN) == (BTN_MAN_ACTIVE_HIGH ? GPIO_PIN_SET : GPIO_PIN_RESET)) ? 1u : 0u;
        if (pressed && !man_prev) { man_t0 = now; man_long = 0; }
        if (pressed && !man_long && (now - man_t0) >= BTN_MAN_LONG_MS) { man_long = 1; ActAutoMode(); }
        if (!pressed && man_prev && !man_long && (now - man_t0) >= 40u) ActToggleFastSlow();
        man_prev = pressed;
    }
}

#if SHOW_FPS
/* แสดงเฟรมเรตจริง (debug) — ฟอร์แมตกว้างคงที่ เพื่อไม่ให้เลขเดิมค้างบนจอ */
static void DrawFps(float fps)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%4.1ffps", (double)fps);
    ILI9341_Draw_Text(buf, 200, SCREEN_H - 12, DARKCYAN, 1, BLACK);   /* เผื่อที่ให้แถบ SET ทางซ้าย */
}
#endif

/* ==========================================================================
 * p30: ADC INPUT — รับข้อมูลจริงจากเครื่องกำเนิดสัญญาณ (ADC1_IN0 = PA0)
 *   ย้ายวิธีจาก MAD/project_2 (ADC1 + DMA2_Stream0, continuous + DMA NORMAL)
 *   ต่างจาก project_2: จับ n = N_SAMPLES x decim จุด แล้วเฉลี่ยแบบ boxcar
 *   เหลือ N_SAMPLES (240) จุด => ได้ "time base" จากจำนวนเต็ม (ไม่ต้อง reprogram timer)
 *   osc_fs_eff = ADC clock-derived rate / resampling step; DWT is diagnostic only.
 * ========================================================================== */
#if ADC_INPUT_ENABLE

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

ALIGN_32BYTES(uint16_t adc_raw_buf[ADC_RAW_MAX]);
static volatile uint8_t  adc_done = 0, adc_error = 0;
static volatile uint8_t  adc_async = 0;   /* 1 from AdcCaptureStart until the capture is collected or aborted */

/* ค่าที่อ่านผ่าน SWD เพื่อตรวจสอบ */
volatile float    dbg_adc_rate   = 0.0f;   /* ADC clock / conversion cycles */
volatile uint8_t  dbg_adc_lvl    = 0;      /* rate level of the last display capture (0 = 1.6 MSa/s) */
volatile uint32_t dbg_adc_decim  = 1;      /* ตัวหารที่ใช้เฟรมล่าสุด              */
volatile uint32_t dbg_adc_n      = 0;      /* จำนวนจุดดิบที่จับ                     */
volatile uint32_t dbg_adc_us     = 0;      /* เวลาจับข้อมูลจริง (µs)               */
volatile uint16_t dbg_adc_min    = 0;      /* ค่าต่ำสุด/สูงสุดในบัฟเฟอร์ดิบ         */
volatile uint16_t dbg_adc_max    = 0;
volatile uint32_t dbg_adc_frames = 0;
volatile uint8_t  dbg_adc_ok     = 0;      /* 0 = ยังวัดอัตราสุ่มไม่สำเร็จ         */
volatile float    dbg_adc_step   = 1.0f;   /* จุดดิบต่อ 1 จุดแสดงผล (step จริง)     */
volatile uint32_t dbg_adc_start  = 0;      /* index ที่เริ่มวาด (จุด trigger, p38)   */

static uint32_t AdcElapsedUs(uint32_t start)
{
    return (uint32_t)(DWT->CYCCNT - start) / (HAL_RCC_GetHCLKFreq() / 1000000UL);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adcHandle)
{
    if (adcHandle->Instance == ADC1) {
        GPIO_InitTypeDef g = {0};
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* PA0 = ADC1_IN0 (ขาว่างของโปรเจกต์นี้: ปุ่มใช้ PB0-PB3) */
        g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_ANALOG; g.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &g);

        hdma_adc1.Instance = DMA2_Stream0;
        hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_NORMAL;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_done = 1;
        if (adc_async) {                       /* async capture finished while the CPU is busy drawing: no more conversions, no overrun interrupts */
            ADC1->CR2 &= ~ADC_CR2_CONT;
            ADC1->CR1 &= ~ADC_CR1_OVRIE;
        }
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    /* Continuous ADC may overrun DR after normal-mode DMA has filled every
     * requested sample. That tail overrun cannot corrupt the completed buffer. */
    if (hadc->Instance == ADC1 && !(adc_done && hadc->ErrorCode == HAL_ADC_ERROR_OVR))
        adc_error = 1;
}

void ADC_IRQHandler(void) { HAL_ADC_IRQHandler(&hadc1); }
void DMA2_Stream0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_adc1); }

/* ---------- 3 ADC interleaved (ADC1 master + ADC2 + ADC3, all on PA0 = IN0) ----------
 * ADC clock 24 MHz, 15 cycles per conversion, the three ADCs start 5 cycles apart
 * -> one sample every 5 cycles = 4.8 MSa/s. DMA mode 2 returns two 16-bit results per 32-bit word,
 * in time order (ADC1, ADC2, ADC3, ADC1, ...), so the halfword buffer is already a plain time series. */
static ADC_HandleTypeDef hadc2, hadc3;
static volatile uint8_t slow_busy = 0;        /* 1 while a burst capture (ADC1 DMA / 3 ADCs) runs: the 1 kHz history sampler skips its tick */
static volatile uint8_t slow_adc3_dirty = 1;  /* 1 = ADC3 was switched off/reconfigured: the history sampler re-prepares it */
static uint8_t adc_tri_ready = 0;
static uint8_t adc_tri_on    = 0;       /* set by AdcSetRateLevel() for the 4.8 MSa/s level */
volatile uint32_t dbg_tri[6];            /* start ret, stop ret, done, error, errcode, ok */

static uint8_t AdcTripleInit(void)
{
    ADC_ChannelConfTypeDef c = {0};
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();
    hadc2.Instance = ADC2; hadc2.Init = hadc1.Init; hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc3.Instance = ADC3; hadc3.Init = hadc1.Init; hadc3.Init.DMAContinuousRequests = DISABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK || HAL_ADC_Init(&hadc3) != HAL_OK) return 0;
    c.Channel = ADC_CHANNEL_0; c.Rank = ADC_REGULAR_RANK_1; c.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &c) != HAL_OK || HAL_ADC_ConfigChannel(&hadc3, &c) != HAL_OK) return 0;
    adc_tri_ready = 1;
    return 1;
}

static uint8_t AdcCaptureTriple(uint32_t n, uint32_t *t_us)
{
    uint32_t t0, to, words = (n + 1u) / 2u;
    uint8_t ok;
    if (n == 0 || n > ADC_RAW_MAX || !t_us) return 0;
    if (!adc_tri_ready && !AdcTripleInit()) return 0;
    uint32_t bytes = (words * 4u + 31u) & ~31u;
    adc_done = adc_error = 0;
    ADC3->CR2 &= ~ADC_CR2_ADON;                       /* MULTI/prescaler may only change with every ADC off */
    ADC3->CR2 |= ADC_CR2_CONT; ADC2->CR2 |= ADC_CR2_CONT;
    ADC3->SMPR2 &= ~7u;                               /* channel 0: 3 cycles for the interleaved capture */
    ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~(ADC_CCR_MULTI | ADC_CCR_DMA | ADC_CCR_DDS | ADC_CCR_DELAY))
                       | ADC_TRIPLEMODE_INTERL | ADC_DMAACCESSMODE_2 | ADC_TWOSAMPLINGDELAY_5CYCLES;
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE)) | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1;   /* 32-bit */
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    HAL_ADC_Start(&hadc3);
    HAL_ADC_Start(&hadc2);
    t0 = DWT->CYCCNT;
    dbg_tri[0] = HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc_raw_buf, words);
    if (dbg_tri[0] != HAL_OK) {
        ok = 0;
    } else {
        to = HAL_GetTick();
        while (!adc_done && !adc_error && (HAL_GetTick() - to) < 200u) { }
        ok = adc_done && !adc_error;
    }
    dbg_tri[2] = adc_done; dbg_tri[3] = adc_error; dbg_tri[4] = hadc1.ErrorCode;
    dbg_tri[1] = HAL_ADCEx_MultiModeStop_DMA(&hadc1);
    /* HAL returns HAL_ERROR here when the DMA already finished (nothing left to abort): not a capture error */
    dbg_tri[5] = ok;
    HAL_ADC_Stop(&hadc2);
    HAL_ADC_Stop(&hadc3);
    slow_adc3_dirty = 1;
    *t_us = AdcElapsedUs(t0);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    /* back to single-ADC mode and 16-bit DMA for every other rate level */
    ADC123_COMMON->CCR &= ~(ADC_CCR_MULTI | ADC_CCR_DMA | ADC_CCR_DDS | ADC_CCR_DELAY);
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE)) | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0;
    return ok;
}

/* จับข้อมูลดิบ n จุด (blocking สั้น ๆ) -> 1 = สำเร็จ */
static uint8_t AdcCaptureRawInner(uint32_t n, uint32_t *t_us)
{
    uint32_t t0, to;
    if (adc_tri_on) return AdcCaptureTriple(n, t_us);
    if (n == 0 || n > ADC_RAW_MAX || n > 65535u || !t_us) return 0;
    uint32_t bytes = (n * 2u + 31u) & ~31u;
    adc_done = adc_error = 0;
    /* Aligned buffer and padded capacity own every invalidated cache line. */
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    t0 = DWT->CYCCNT;
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_buf, n) != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc1);
        return 0;
    }
    to = HAL_GetTick();
    while (!adc_done && !adc_error && (HAL_GetTick() - to) < 200u) { }
    uint8_t ok = adc_done && !adc_error;
    if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) ok = 0;
    *t_us = AdcElapsedUs(t0);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    return ok;
}

static uint8_t AdcCaptureRaw(uint32_t n, uint32_t *t_us)
{
    uint8_t ok;
    slow_busy = 1;
    ok = AdcCaptureRawInner(n, t_us);
    slow_busy = 0;
    return ok;
}

/* ASYNC single-ADC capture: Start returns at once (DMA runs in hardware), Collect waits for the end and returns the buffer to the CPU.
 * Used to capture the NEXT frame while the CPU draws the current one. slow_busy stays 1 for the whole time (the 1 kHz sampler must not touch ADC3). */
static uint8_t AdcCaptureStart(uint32_t n, uint32_t *t0)
{
    if (adc_tri_on || n == 0 || n > ADC_RAW_MAX || n > 65535u || !t0) return 0;
    uint32_t bytes = (n * 2u + 31u) & ~31u;
    adc_done = adc_error = 0;
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    *t0 = DWT->CYCCNT;
    ADC1->CR2 |= ADC_CR2_CONT;
    slow_busy = 1;
    adc_async = 1;
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_buf, n) != HAL_OK) {
        HAL_ADC_Stop_DMA(&hadc1);
        ADC1->CR2 |= ADC_CR2_CONT;
        adc_async = 0; slow_busy = 0;
        return 0;
    }
    return 1;
}

static uint8_t AdcCaptureCollect(uint32_t n, uint32_t t0, uint32_t *t_us)
{
    uint32_t bytes = (n * 2u + 31u) & ~31u, to = HAL_GetTick();
    uint8_t ok;
    while (!adc_done && !adc_error && (HAL_GetTick() - to) < 200u) { }
    ok = adc_done && !adc_error;
    if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK) ok = 0;
    ADC1->CR2 |= ADC_CR2_CONT;                       /* the callback cleared it: every other capture path expects continuous mode */
    adc_async = 0; slow_busy = 0;
    if (t_us) *t_us = AdcElapsedUs(t0);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    return ok;
}

static void AdcInputInit(void)
{
    ADC_ChannelConfTypeDef c = {0};
    uint32_t t_us = 0;

    /* ฟังก์ชัน simulator เดิมยังอยู่ครบ (ใช้เมื่อ ADC_INPUT_ENABLE = 0) — กัน warning */
    (void)GenFrame; (void)SimSamplingStream; (void)GenerateFakeSineWave;

    /* DWT ไมโครวินาที (โปรเจกต์นี้ยังไม่เปิด) */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    *((volatile uint32_t *)0xE0001FB0) = 0xC5ACCE55;        /* ปลดล็อก DWT (Cortex-M7) */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    __HAL_RCC_DMA2_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;   /* PCLK2/4 = 12 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    c.Channel = ADC_CHANNEL_0; c.Rank = ADC_REGULAR_RANK_1; c.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &c) != HAL_OK) Error_Handler();

    /* วัดอัตราสุ่มจริง (ใช้ 2000 จุด -> ไม่กี่ ms) แล้วเก็บเป็นแหล่งความจริง */
    adc_done = 0;
    if (AdcCaptureRaw(2000u, &t_us) && t_us > 0u) {
        dbg_adc_rate = 2000.0f * 1000000.0f / (float)t_us;
        dbg_adc_ok   = 1;
    }
    (void)AdcTripleInit();       /* ADC3 is the always-on slow-history sampler */
}

/* Use ADC clock timing, not DMA start/stop software latency, for sample spacing. */
#define ADC_RATE_LEVELS  5
#define ADC_MEAS_LEVELS  3
static const struct { uint32_t div; uint32_t smp; float cyc; float mult; } adc_rate_tab[ADC_RATE_LEVELS] = {
    { 2u, ADC_SAMPLETIME_3CYCLES,  15.0f, 1.0f },   /* L0: ADC 24 MHz / 15 = 1.600 MSa/s */
    { 4u, ADC_SAMPLETIME_3CYCLES,  15.0f, 1.0f },   /* L1: ADC 12 MHz / 15 = 800 kSa/s  (original FAST) */
    { 4u, ADC_SAMPLETIME_15CYCLES, 27.0f, 1.0f },   /* L2: 12 MHz / 27 = 444 kSa/s       (original 'slow' for 5/6 ms/div) */
    { 4u, ADC_SAMPLETIME_28CYCLES, 40.0f, 1.0f },   /* L3: 12 MHz / 40 = 300 kSa/s */
    { 2u, ADC_SAMPLETIME_3CYCLES,  15.0f, 3.0f },   /* L4: 3 ADCs interleaved at 24 MHz = 4.800 MSa/s */
};

static float AdcRateHz(int lvl)
{
    return (float)HAL_RCC_GetPCLK2Freq() / (float)adc_rate_tab[lvl].div / adc_rate_tab[lvl].cyc * adc_rate_tab[lvl].mult;
}

/* ADC must be disabled here (every capture stops it): common prescaler + channel sampling time */
static uint8_t AdcSetRateLevel(int lvl)
{
    ADC_ChannelConfTypeDef c = {0};
    if (lvl < 0 || lvl >= ADC_RATE_LEVELS) return 0;
    ADC3->CR2 &= ~ADC_CR2_ADON;                  /* history sampler's ADC off while the common prescaler changes */
    slow_adc3_dirty = 1;
    ADC123_COMMON->CCR = (ADC123_COMMON->CCR & ~ADC_CCR_ADCPRE)
                       | ((adc_rate_tab[lvl].div == 2u) ? ADC_CLOCK_SYNC_PCLK_DIV2 : ADC_CLOCK_SYNC_PCLK_DIV4);
    c.Channel = ADC_CHANNEL_0;
    c.Rank = ADC_REGULAR_RANK_1;
    c.SamplingTime = adc_rate_tab[lvl].smp;
    if (HAL_ADC_ConfigChannel(&hadc1, &c) != HAL_OK) return 0;
    adc_tri_on = (adc_rate_tab[lvl].mult > 1.5f) ? 1u : 0u;
    dbg_adc_rate = AdcRateHz(lvl);
    return 1;
}

/* buf = first sample to analyse, n samples: the legacy measurement passes the whole capture, the fast path only the newest ~3.8 periods of the display capture */
static float FAST_O2 AdcRawFrequency(const uint16_t *buf, uint32_t n, float fs)
{
    uint16_t mn = 4095, mx = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (buf[i] < mn) mn = buf[i];
        if (buf[i] > mx) mx = buf[i];
    }
    osc_raw_vpp = (mx - mn) * ADC_VREF / ADC_FULL;
    adc_fast_content = 0;
    adc_rough_ratio = 0.0f;
    osc_raw_valid = 0;
    if (osc_raw_vpp < 0.05f) return 0.0f;
    if (adc_tri_on && osc_raw_vpp < 0.30f) return 0.0f;   /* at 4.8 MSa/s a 0.15-0.2 Vpp noise band (generator off / 0 Hz) crosses its middle every ~2 samples = a false 2 MHz; real 2 MHz measured 0.6 Vpp */
    {   /* mean |sample-to-sample change| / Vpp: < 2 % for anything below ~50 Hz in this window, > 25 % when there are <= 8 samples per cycle */
        uint32_t acc = 0;
        for (uint32_t i = 1; i < n; i++) acc += (buf[i] > buf[i - 1]) ? (uint32_t)(buf[i] - buf[i - 1]) : (uint32_t)(buf[i - 1] - buf[i]);
        adc_rough_ratio = (float)acc / (float)(n - 1u) / (float)(mx - mn);
        if (osc_raw_vpp >= 0.30f && adc_rough_ratio > 0.25f && (float)acc / (float)(n - 1u) > 120.0f) adc_fast_content = 1;   /* a 0.2 Vpp noise band is not a fast signal */
    }
    float mid = (mx + mn) * 0.5f;
    /* Single ADC: one pass, hysteresis 20 % of Vpp, up to fs/5 (unchanged).
     * 3-ADC level: a 2nd pass with 8 % hysteresis reaches 2.2-2.4 samples per cycle, where a 20 % hysteresis
     * skips cycles. Readings above fs/8 must also have evenly spaced edges, so a skipped or doubled
     * edge can never turn into a plausible wrong number. */
    for (int pass = 0; pass < (adc_tri_on ? 2 : 1); pass++) {
        float hyst = (mx - mn) * (pass ? 0.08f : 0.2f);
        float first = 0.0f, last = 0.0f;
        uint32_t edges = 0;
        int armed = 0;
        for (uint32_t i = 1; i < n; i++) {
            float prev = buf[i - 1], v = buf[i];
            if (v < mid - hyst) armed = 1;
            if (armed && prev < mid && v >= mid) {
                float at = (float)(i - 1) + (mid - prev) / (v - prev);
                if (!edges) first = at;
                last = at;
                edges++;
                armed = 0;
            }
        }
        /* Need repeated complete periods; partial windows must not invent a frequency. */
        if (edges < 3 || last <= first) continue;
        float f = (edges - 1) * fs / (last - first);
        float fmax = adc_tri_on ? fs * 0.45f : fs / 5.0f;      /* 2.16 MHz at 4.8 MSa/s, 1.6 MSa/s: 320 kHz */
        if (f > fmax) { if (osc_raw_vpp >= 0.30f) adc_too_fast = 1; continue; }
        if (f < 39.9f) continue;
        /* A low frequency cannot coexist with large sample-to-sample change: that is a fast signal near/above Nyquist
         * beating with the 3-ADC pattern, so refuse the reading instead of showing a false low number. */
        /* Every rate: the sample-to-sample change must match a signal of this frequency (~0.64*sin(pi*f/fs) of Vpp, plus ADC noise).
         * A fast signal that aliases to a low number (e.g. 2.4 MHz = 3 x 800 kSa/s -> 30 kHz) is far rougher than that. */
        if (adc_rough_ratio > 3.0f * 0.64f * sinf(3.14159265f * f / fs) + 0.10f + 12.0f / (float)(mx - mn)) continue;
        if (adc_tri_on && f < fs / 9.0f && (adc_fast_content || (adc_over_t && (HAL_GetTick() - adc_over_t) < 3000u))) continue;
        {   /* every rate: the edge spacing must be regular - noise makes random edges, a sine/square/triangle does not */
            /* the sample-to-sample change must match a sine of this frequency (about 0.64*sin(pi*f/fs) of Vpp); a fast signal
             * near Nyquist that fakes a low frequency is far rougher than that */
            /* second scan: count edge spacings further than 30 % from the mean period; > 5 % of them = not a clean periodic signal */
            float mean = (last - first) / (float)(edges - 1), lastat = 0.0f;
            uint32_t bad = 0, cnt = 0, good = 0;
            float gsum = 0.0f;
            armed = 0;
            for (uint32_t i = 1; i < n; i++) {
                float prev = buf[i - 1], v = buf[i];
                if (v < mid - hyst) armed = 1;
                if (armed && prev < mid && v >= mid) {
                    float at = (float)(i - 1) + (mid - prev) / (v - prev);
                    if (cnt) { float dt = at - lastat; if (dt > 1.3f * mean || dt < 0.7f * mean) bad++; else { gsum += dt; good++; } }
                    lastat = at; cnt++; armed = 0;
                }
            }
            if (bad * 20u > cnt || good < 2u) continue;
            f = fs * (float)good / gsum;        /* average only the clean periods: a skipped edge (double interval) cannot bias it */
        }
        {   /* exact DC and RMS: only the whole cycles between the first and the last edge; min/max of the whole capture */
            uint32_t i0 = (uint32_t)(first + 1.0f), i1 = (uint32_t)last;
            if (i1 > i0 + 1u && i1 < n) {
                uint32_t sm = 0;
                float cnt = (float)(i1 - i0 + 1u), mean, sq = 0.0f;
                for (uint32_t i = i0; i <= i1; i++) sm += buf[i];
                mean = (float)sm / cnt;
                for (uint32_t i = i0; i <= i1; i++) { float d = (float)buf[i] - mean; sq += d * d; }
                osc_raw_mean_ct = mean; osc_raw_rms_ct = sqrtf(sq / cnt);
                osc_raw_min_ct = (float)mn; osc_raw_max_ct = (float)mx; osc_raw_valid = 1;
                {   /* v82: Mx / Mn of the raw min/max include the noise peaks: 2.47 V read for a 2.40 V sine, whose trace spans 2.4 div at 1 V/div,
                     * so the number did not match the grid. Take them from a boxcar of k = 1/32 period (3..64 samples, keeps > 99.8 % of a sine peak,
                     * noise / sqrt(k)) over the same whole cycles. Fewer than 96 samples per period: the plain min/max as before. */
                    uint32_t k = (uint32_t)(fs / f / 32.0f);
                    if (k > 64u) k = 64u;
                    if (k >= 3u && i1 > i0 + k) {
                        uint32_t run = 0;
                        float smn = 1e9f, smx = -1e9f;
                        for (uint32_t i = i0; i < i0 + k; i++) run += buf[i];
                        for (uint32_t i = i0 + k; ; i++) {
                            float a = (float)run / (float)k;
                            if (a < smn) smn = a;
                            if (a > smx) smx = a;
                            if (i > i1) break;
                            run += buf[i]; run -= buf[i - k];
                        }
                        osc_raw_min_ct = smn; osc_raw_max_ct = smx;
                    }
                }
            }
        }
        return f;
    }
    return 0.0f;
}

/* Pure planning (no hardware): display level + nominal step for this time base, and whether the measurement can be read from the display capture. */
static uint8_t AdcPlanFrame(float fs_want, int *lvl_o, float *step_o)
{
    int lvl = 1;                                  /* 800 kSa/s: the original rate, used for every time base it can serve */
    float step = AdcRateHz(1) / fs_want;
    uint8_t fast_meas = 0;
    if (step < 1.0f) {                            /* time base faster than 800 kSa/s can draw */
        lvl = 0; step = AdcRateHz(0) / fs_want;   /* 1.6 MSa/s */
        if (step < 1.0f) { lvl = 4; step = AdcRateHz(4) / fs_want; }   /* 4.8 MSa/s, 3 ADCs */
    } else if ((N_SAMPLES + ADC_PRE_POINTS + 1) * step + 3 > ADC_RAW_MAX) {     /* too slow for 800 kSa/s: longer capture */
        lvl = 2; step = AdcRateHz(2) / fs_want;
        if ((N_SAMPLES + ADC_PRE_POINTS + 1) * step + 3 > ADC_RAW_MAX) { lvl = 3; step = AdcRateHz(3) / fs_want; }
    }
    /* STEP 2 (frame rate): when the frequency is known and fresh, the display capture itself holds >= 3.4 periods at a rate >= 16 samples per cycle,
     * so the separate measurement capture (3.2 periods, ~50 ms + setup) is skipped and AdcRawFrequency reads the display buffer. The old two-capture
     * measurement (alias probe, rate hunting, growth of the window) still runs about once a second and whenever a reading is lost or changes. */
    {   uint32_t t0n = HAL_GetTick();
        if (adc_last_good_f > 0.0f && (t0n - adc_last_good_t) < 1200u && (t0n - adc_legacy_t) < 1000u && lvl != 4) {
            float R = AdcRateHz(lvl), f0 = adc_last_good_f;
            if (f0 <= R * (1.0f / 16.0f)) {
                uint32_t pp0 = (uint32_t)(1.3f * fs_want / f0) + 12u;
                float n0;
                if (pp0 > ADC_PRE_POINTS) pp0 = ADC_PRE_POINTS;
                if (pp0 < 40u) pp0 = 40u;
                n0 = ((float)(N_SAMPLES + pp0) * step + step + 2.0f);
                if (n0 * f0 / R >= 3.4f) fast_meas = 1;
            }
        }
    }
    *lvl_o = lvl; *step_o = step;
    return fast_meas;
}

/* display points recorded before the trigger search ends: 1.3 periods + margin when the frequency f is known, else the full ADC_PRE_POINTS */
static uint32_t AdcPreFor(float fs_want, float f)
{
    uint32_t pre = ADC_PRE_POINTS;
    if (f > 0.0f && fs_want > 0.0f) {
        uint32_t pp = (uint32_t)(1.3f * fs_want / f) + 12u;
        if (pp < pre) pre = pp;
        if (pre < 40u) pre = 40u;
    }
    return pre;
}

static void AdcAbortPending(void)
{
    if (!adc_pend) return;
    HAL_ADC_Stop_DMA(&hadc1);
    ADC1->CR2 |= ADC_CR2_CONT;
    adc_async = 0; slow_busy = 0; adc_pend = 0;
}

/* STEP 3 (frame rate): called by the main loop after the numbers of a frame are done and BEFORE it draws. Starts the capture of the next frame so the
 * DMA runs while the CPU draws (the draw no longer adds to the frame time). Only for the cheap case (fast measurement path, single ADC, FAST time base);
 * anything else is left to the normal blocking AdcCaptureFrame. */
static void AdcPrestart(void)
{
    float fs_want, step;
    int lvl;
    uint32_t w, pre, n;
    if (adc_pend || osc_tdiv_i >= OSC_NT_FAST || !osc_frame_valid) return;
    fs_want = (float)N_SAMPLES / (osc_tdivs_ms[osc_tdiv_i] * OSC_H_DIVS / 1000.0f);
    if (!AdcPlanFrame(fs_want, &lvl, &step) || lvl == 4) return;
    if (!AdcSetRateLevel(lvl)) return;
    step = dbg_adc_rate / fs_want;
    if (step < 1.0f) step = 1.0f;
    w = (uint32_t)(step + 0.5f);
    pre = AdcPreFor(fs_want, adc_last_good_f);
    n = (uint32_t)((N_SAMPLES + pre) * step) + w + 2u;
    if (n > ADC_RAW_MAX) return;
    if (!AdcCaptureStart(n, &adc_pd_t0)) return;
    adc_pd_tdiv = osc_tdiv_i; adc_pd_lvl = lvl; adc_pd_n = n; adc_pd_w = w; adc_pd_pre = pre; adc_pd_step = step; adc_pd_tick = HAL_GetTick();
    adc_pend = 1;
    dbg_pipe_cnt++;
}

#define PROF_START()   (DWT->CYCCNT)
#define PROF_END(i,t0)  (prof_us[i] = AdcElapsedUs(t0))
/* จับ 1 เฟรม: เลือกตัวหารจาก T/DIV -> เฉลี่ย -> raw_samples[N_SAMPLES] */
static void FAST_O2 AdcCaptureFrame(void)
{
    float fs_want = (float)N_SAMPLES / (osc_tdivs_ms[osc_tdiv_i] * OSC_H_DIVS / 1000.0f);
    uint32_t n, t_us = 0;
    uint16_t vmin = 0xFFFF, vmax = 0;
    new_frame_ready = 0;
    osc_frame_valid = 0;
    adc_f_held = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_raw_valid = 0;
    OSC_LOCK(); osc_meas = (OscMeasure){0}; OSC_UNLOCK();
    strcpy(osc_det, "INVALID");

    int lvl; float step;
    uint8_t fast_meas = AdcPlanFrame(fs_want, &lvl, &step);

    uint8_t use_pend = 0;                         /* STEP 3: a capture of THIS frame was started during the previous draw */
    if (adc_pend) {
        if (fast_meas && adc_pd_tdiv == osc_tdiv_i && adc_pd_lvl == lvl) use_pend = 1;
        else AdcAbortPending();                   /* time base or measurement path changed: forget it */
    }

    /* Fast, independent measurement: 4 ms initially, grow on later frames
     * until >=3 periods are available. Max 81.9 ms only for slow/no signal.
     * Every acquisition is contiguous; never stitch separate DMA blocks.
     * Supported periodic input: 40 Hz..100 kHz, >=50 mVpp, within 0..3.3 V.
     * External anti-alias filtering/source impedance remain hardware duties. */
    uint32_t pB0 = PROF_START(); prof_us[5] = 0;
    if (!fast_meas) {
    if (!AdcSetRateLevel(adc_meas_seq[adc_meas_lvl]) || !AdcCaptureRaw(adc_measure_n, &t_us)) goto failed;
    adc_too_fast = 0;
    osc_raw_freq = AdcRawFrequency(adc_raw_buf, adc_measure_n, dbg_adc_rate);
    adc_over_range = adc_too_fast;
    uint8_t meas_on_tri = adc_tri_on;
    uint8_t probe_due = ((HAL_GetTick() - adc_probe_t) >= 1000u);
    if (adc_tri_on && (adc_too_fast || (adc_fast_content && osc_raw_freq == 0.0f))) adc_over_t = HAL_GetTick() | 1u;   /* only judged on the fastest rate */
    if (osc_raw_freq > 0.0f) {
        if (adc_meas_lvl < ADC_MEAS_LEVELS - 1u && osc_raw_freq > dbg_adc_rate * (1.0f / 16.0f)) {
            /* few samples per cycle: a slower rate can alias a much faster signal into this range -> confirm on the fastest rate */
            adc_meas_lvl  = ADC_MEAS_LEVELS - 1u;
            adc_measure_n = 3200u;
            osc_raw_freq  = 0.0f;
            adc_hold_f    = 0.0f;                                   /* this reading is unconfirmed: never hold it */
        } else {
            float want = 3.2f * dbg_adc_rate / osc_raw_freq;
            adc_measure_n = (uint32_t)fminf(ADC_RAW_MAX, fmaxf(3200.0f, want));
        }
    } else if (adc_too_fast && adc_meas_lvl < ADC_MEAS_LEVELS - 1u) {
        adc_meas_lvl++;                                                   /* signal above this rate's range: go to a faster rate */
        adc_measure_n = 3200u;
    } else if (adc_measure_n >= ADC_RAW_MAX) {
        adc_meas_lvl  = (uint8_t)((adc_meas_lvl + 1u) % ADC_MEAS_LEVELS);   /* nothing found even with the longest window: next rate (800 k -> 1.6 M -> 4.8 M -> 800 k) */
        adc_measure_n = 3200u;
    } else {
        adc_measure_n = (adc_measure_n > ADC_RAW_MAX / 2u) ? ADC_RAW_MAX : adc_measure_n * 2u;
    }

    if (osc_raw_freq > 0.0f && !meas_on_tri && probe_due) {
        adc_probe_t = HAL_GetTick();
        /* ALIAS PROBE: a signal far above this rate's Nyquist (e.g. 2.4 MHz = 3 x 800 kSa/s) can alias to a clean low number.
         * Look at 0.33 ms on the fastest rate: a slow signal is smooth there (sample-to-sample change ~0), a fast one is not. */
        uint32_t tp = 0;
        if (AdcSetRateLevel(4) && AdcCaptureRaw(1600u, &tp)) {
            uint16_t pmn = 0xFFFF, pmx = 0; uint32_t pacc = 0;
            for (uint32_t i = 0; i < 1600u; i++) {
                if (adc_raw_buf[i] < pmn) pmn = adc_raw_buf[i];
                if (adc_raw_buf[i] > pmx) pmx = adc_raw_buf[i];
                if (i) pacc += (adc_raw_buf[i] > adc_raw_buf[i - 1]) ? (uint32_t)(adc_raw_buf[i] - adc_raw_buf[i - 1]) : (uint32_t)(adc_raw_buf[i - 1] - adc_raw_buf[i]);
            }
            if ((float)(pmx - pmn) * ADC_VREF / ADC_FULL >= 0.30f && (float)pacc / 1599.0f > 0.25f * (float)(pmx - pmn)
                && (float)pacc / 1599.0f > 120.0f) {
                osc_raw_freq  = 0.0f;                     /* aliased: never show it */
                adc_hold_f    = 0.0f;
                adc_over_t    = HAL_GetTick() | 1u;
                adc_meas_lvl  = ADC_MEAS_LEVELS - 1u;     /* measure on the fastest rate from now on */
                adc_measure_n = 3200u;
            }
        }
    }

    if (osc_raw_freq <= 0.0f) osc_raw_valid = 0;               /* statistics only for a confirmed reading */
    {   /* hold the last good reading for 1.5 s: a marginal signal (e.g. 2 MHz at 2.4 samples/cycle) fails on some frames */
        uint32_t tnow = HAL_GetTick();
        if (osc_raw_freq > 0.0f)                                { adc_hold_f = osc_raw_freq; adc_hold_t = tnow; adc_last_good_f = osc_raw_freq; adc_last_good_t = tnow; adc_last_good_vpp = osc_raw_vpp; }
        else if (osc_raw_vpp < 0.05f || adc_over_range)         { adc_hold_f = 0.0f; }
        else if (adc_hold_f > 0.0f && (tnow - adc_hold_t) < 1500u) { osc_raw_freq = adc_hold_f; adc_f_held = 1; }
    }
    adc_legacy_t = HAL_GetTick();
    PROF_END(5, pB0);
    }   /* !fast_meas */

    uint32_t pD0 = PROF_START();
    uint32_t w, dec, pre;
    if (use_pend) {                               /* started during the previous draw: just wait for it */
        step = adc_pd_step; w = adc_pd_w; pre = adc_pd_pre; n = adc_pd_n;
        dec = w;
        dbg_adc_lvl = (uint8_t)lvl;
        dbg_adc_pre = pre;
        if (!AdcCaptureCollect(n, adc_pd_t0, &t_us)) { adc_pend = 0; goto failed; }
        adc_pend = 0;
    } else {
        if (!AdcSetRateLevel(lvl)) goto failed;
        dbg_adc_lvl = (uint8_t)lvl;
        step = dbg_adc_rate / fs_want;
        if (step < 1.0f) step = 1.0f;
        w = (uint32_t)(step + 0.5f);
        dec = w;
        pre = AdcPreFor(fs_want, fast_meas ? adc_last_good_f : osc_raw_freq);
        dbg_adc_pre = pre;
        n = (uint32_t)((N_SAMPLES + pre) * step) + w + 2u;
        if (n > ADC_RAW_MAX || !AdcCaptureRaw(n, &t_us)) goto failed;
    }
    PROF_END(6, pD0);
    dbg_adc_ok = 1;
    dbg_adc_step = step;
    prof_us[7] = 0;
    if (fast_meas) {
        uint32_t pF0 = PROF_START();                                  /* measurement from the display capture (adc_raw_buf still holds it) */
        float f2;
        uint32_t tnow;
        adc_too_fast = 0;
        {   uint32_t m = (uint32_t)(3.8f * dbg_adc_rate / adc_last_good_f) + 64u;      /* the newest ~3.8 periods are enough for 3 edges: 3x less work */
            if (m > n) m = n;
            f2 = AdcRawFrequency(adc_raw_buf + (n - m), m, dbg_adc_rate);
        }
        adc_over_range = adc_too_fast;
        tnow = HAL_GetTick();
        dbg_fm_cnt++;
        PROF_END(7, pF0);
        /* the raw min-max of ~30 000 samples at 800 kSa/s sits ~5 % above the true Vpp (noise peaks: 3.25 V against 3.08 V from the old measurement and 3.05 V in the picture).
         * The reading of the old two-capture measurement is kept as the amplitude while the signal is steady; a clear change sends the frame back to that measurement. */
        if (f2 > 0.0f && !adc_too_fast && fabsf(f2 - adc_last_good_f) <= 0.10f * adc_last_good_f
            && osc_raw_vpp > 0.80f * adc_last_good_vpp && osc_raw_vpp < 1.25f * adc_last_good_vpp) {
            if (osc_raw_valid && osc_raw_vpp > 0.0f) {   /* Mx / Mn / Rs use the cycle statistics: scale the swing about the mean to the steady amplitude */
                float kk = adc_last_good_vpp / osc_raw_vpp;
                osc_raw_max_ct = osc_raw_mean_ct + (osc_raw_max_ct - osc_raw_mean_ct) * kk;
                osc_raw_min_ct = osc_raw_mean_ct + (osc_raw_min_ct - osc_raw_mean_ct) * kk;
                osc_raw_rms_ct *= kk;
            }
            osc_raw_vpp = adc_last_good_vpp;
            osc_raw_freq = f2; adc_last_good_f = f2; adc_last_good_t = tnow; adc_hold_f = f2; adc_hold_t = tnow;
        } else {                                      /* changed or unreadable: the old two-capture measurement takes over on the next frame */
            if (f2 <= 0.0f) { if (adc_too_fast) dbg_fm_fail[2]++; else dbg_fm_fail[0]++; } else dbg_fm_fail[1]++;
            osc_raw_freq = 0.0f; osc_raw_valid = 0; adc_last_good_t = 0u;
            if (osc_raw_vpp < 0.05f || adc_over_range)                 adc_hold_f = 0.0f;
            else if (adc_hold_f > 0.0f && (tnow - adc_hold_t) < 1500u) { osc_raw_freq = adc_hold_f; adc_f_held = 1; }
        }
    }

    /* --- หาจุด trigger ในข้อมูลดิบ (ระดับ = ค่ากลางของบัฟเฟอร์) แล้วเริ่มวาดที่จุดนั้น --- */
    /* v78: left margin so MvStoreExt always finds >= 10 real display points before `start`
     * (filter half-width max = 10). With start = 0 (no trigger / edge found at i=1) mv_nlo was 0, the first `half`
     * points fell back to one repeated value -> a flat block at a ripple extreme = the spike at the left edge. */
    uint32_t edge_m = (uint32_t)(10.0f * step + 0.5f) + w + 2u;
    uint8_t  edge_fit = ((float)edge_m + (float)N_SAMPLES * step + (float)w <= (float)n);
    /* v79: refine boxcar half-width, capped to P/8 (at 10 kHz / 800 kSa/s the old H = step + 1 = 34 spanned ~0.9 period and flattened the edge) */
    uint32_t trig_h = (uint32_t)(step * 1.0f) + 1u; if (trig_h > 64u) trig_h = 64u;
    {   float fp = fast_meas ? adc_last_good_f : osc_raw_freq;
        if (fp > 0.0f) { uint32_t hp = (uint32_t)(dbg_adc_rate / fp / 8.0f); if (hp < 1u) hp = 1u; if (trig_h > hp) trig_h = hp; }
    }
    /* v79: + lead + 3*trig_h, so refine (up to 3*trig_h back) and lead can never pull start below edge_m. The v78 value (edge_m + lead + 1)
     * let the edge_m clamp below fire on every frame: start stuck at edge_m = untriggered, the trace drifted with the capture phase. */
    uint32_t t_from = edge_fit ? (edge_m + (uint32_t)(2.0f * step + 1.0f) + 3u * trig_h + 1u) : 1u;
    uint32_t start = 0;
    if (osc_stable && osc_lock_auto) {
        float sum = 0.0f, mid;
        uint32_t lim = (uint32_t)((float)pre * step) + w;   /* ค้นเฉพาะส่วนก่อน trigger */
        if (lim < 4u) lim = 4u;
        if (lim > n) lim = n;
        uint16_t tmn = 0xFFFF, tmx = 0;
        for (uint32_t i = 0; i < n; i++) {
            uint16_t v = adc_raw_buf[i];
            sum += (float)v;
            if (v < tmn) tmn = v;
            if (v > tmx) tmx = v;
        }
        (void)sum;
        mid = 0.5f * (float)(tmx + tmn);       /* mid-range, not the buffer mean: the mean moves by +-5 % of Vpp with the number of partial cycles in the buffer and made the trace shudder */
        {   /* SCHMITT trigger: at 800 kSa/s a 200 Hz sine moves < 1 count per sample while the input has 30-100 counts of noise,
             * so a plain "below mid -> above mid" test fires on noise on either slope. The signal must first go clearly to the
             * other side (15 % of the range) before the mid crossing counts. */
            float hy = 0.15f * (float)(tmx - tmn);
            uint8_t armed = 0;
            if (osc_trig_fall) {
                for (uint32_t i = 1; i < lim; i++) {
                    float v = (float)adc_raw_buf[i];
                    if (v > mid + hy) armed = 1;
                    else if (armed && v < mid && i >= t_from) { start = i; break; }   /* v78: not inside the left margin */
                    else if (armed && v < mid) armed = 0;   /* v79: crossed before t_from: wait for the next full edge, not the first sample past t_from */
                }
            } else {
                for (uint32_t i = 1; i < lim; i++) {
                    float v = (float)adc_raw_buf[i];
                    if (v < mid - hy) armed = 1;
                    else if (armed && v >= mid && i >= t_from) { start = i; break; }   /* v78 */
                    else if (armed && v >= mid) armed = 0;   /* v79: same as above */
                }
            }
        }
        if (start > 0u) {   /* refine: first crossing of the boxcar-smoothed signal near the raw edge (noise of +-100 counts moves a raw crossing by +-50 samples) */
            uint32_t H = trig_h;   /* v79: P/8 cap, see t_from */
            uint32_t j0 = (start > 3u * H + H + 1u) ? (start - 3u * H) : (H + 1u), j1 = start + 3u * H;
            if (j1 + H + 1u > n) j1 = n - H - 1u;
            float prev = -1.0f; uint8_t got = 0; uint32_t found = start;
            for (uint32_t j = j0; j <= j1; j++) {
                float a = 0.0f; for (uint32_t q = j - H; q <= j + H; q++) a += (float)adc_raw_buf[q];
                a /= (float)(2u * H + 1u);
                if (prev >= 0.0f) {
                    if (!osc_trig_fall && prev < mid && a >= mid) { found = j; got = 1; break; }
                    if ( osc_trig_fall && prev >= mid && a < mid) { found = j; got = 1; break; }
                }
                prev = a;
            }
            if (got) start = found;
        }
        if ((float)start + (float)N_SAMPLES * step + (float)w > (float)n) start = 0;   /* กันหลุดขอบท้าย */

        /* เหลื่อมไปข้างหน้า ~2 จุดแสดงผล: ไม่ให้หน้าต่าง "เริ่มตรงจุดตัดศูนย์พอดี"
         * (ถ้าเริ่มพอดี ตัววัดความถี่ของ firmware จะนับขอบแรกพลาดเป็นครั้งคราว -> f กระพริบเป็น 0) */
        uint32_t lead = (uint32_t)(2.0f * step + 1.0f);
        start = (start > lead) ? (start - lead) : 0u;
    }
    if (edge_fit && start < edge_m) start = edge_m;   /* v78: untriggered / early edge -> keep the left margin */
    dbg_adc_start = start;

    /* v81: signal too fast for this time base (< 2 display points per period, e.g. 10 kHz at 6 ms/div: 444 kSa/s, w = 111 = 2.5 periods).
     * A boxcar of w samples then leaves a fraction of a period in every point; its phase walks along the screen and draws a beat pattern
     * instead of the mean. Average a WHOLE number of measured periods instead (nearest to w, >= 1): every point = the signal mean = a flat line. */
    uint32_t wa = w;
    {   float fp = fast_meas ? adc_last_good_f : osc_raw_freq;
        if (fp > 0.0f) {
            float P = dbg_adc_rate / fp;
            if (2.0f * step >= P) {
                float np = floorf((float)w / P + 0.5f);
                if (np < 1.0f) np = 1.0f;
                uint32_t wp = (uint32_t)(np * P + 0.5f);
                if (wp >= 1u && wp <= n) wa = wp;
            }
        }
    }
    for (int i = 0; i < N_SAMPLES; i++) {   /* resample แบบเศษ + boxcar เฉลี่ย wa จุด (เริ่มที่จุด trigger) */
        int32_t c = (int32_t)start + (int32_t)((float)i * step + 0.5f);
        int32_t lo = c - (int32_t)((wa - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0) lo = 0;
        if (lo + (int32_t)wa > (int32_t)n) lo = (int32_t)n - (int32_t)wa;
        if (lo < 0) lo = 0;
        for (uint32_t k = 0; k < wa; k++) acc += adc_raw_buf[(uint32_t)lo + k];
        uint16_t v = (uint16_t)(acc / wa);
        raw_samples[i] = (int16_t)v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    dec = wa;                        /* v81: dbg_adc_decim = samples actually averaged per point */
    /* v83 PEAK DETECT: with < 8 display points per period no line through the averaged points can show the wave: at 5 ms/div a 2 kHz signal
     * (2.4 points per period) drew ~25 % too small, at 6 ms/div (v81) a flat line at the mean. A real scope draws, per column, the min..max
     * of the samples behind it = a band of the full height. Each point keeps the min / max of its own interval [c - step/2, c + step/2)
     * (the intervals tile the capture), on a boxcar of k = 1/32 period so ADC noise does not make the band taller than the signal. */
    osc_env_on = 0;
    {   float fp = fast_meas ? adc_last_good_f : osc_raw_freq;
        if (fp > 0.0f && dbg_adc_rate / fp < 8.0f * step) {
            uint32_t k = (uint32_t)(dbg_adc_rate / fp / 16.0f);   /* 1/16 period: keeps > 99.3 % of a sine peak; 1/32 left the band ~4 % too tall at 444 kSa/s */
            if (k < 1u) k = 1u;
            if (k > 64u) k = 64u;
            if (n > 2u * k + 2u) {
                {   /* mean / AC rms of the samples behind the window: the readout when this frame has no clean period (the drawn points are the mean, v81) */
                    uint32_t q0 = start, q1 = start + (uint32_t)((float)N_SAMPLES * step);
                    float sm = 0.0f, sq = 0.0f, mu;
                    if (q1 > n) q1 = n;
                    if (q1 <= q0) q1 = q0 + 1u;
                    for (uint32_t q = q0; q < q1; q++) sm += (float)adc_raw_buf[q];
                    mu = sm / (float)(q1 - q0);
                    for (uint32_t q = q0; q < q1; q++) { float dq = (float)adc_raw_buf[q] - mu; sq += dq * dq; }
                    osc_env_mean_ct = mu; osc_env_rms_ct = sqrtf(sq / (float)(q1 - q0));
                }
                for (int i = 0; i < N_SAMPLES; i++) {
                    int32_t c = (int32_t)start + (int32_t)((float)i * step + 0.5f);
                    int32_t a = c - (int32_t)(step * 0.5f) - (int32_t)(k / 2u);   /* first boxcar start */
                    int32_t b = c + (int32_t)(step * 0.5f + 0.5f) - (int32_t)(k / 2u);   /* last boxcar start (exclusive) */
                    if (b <= a) b = a + 1;
                    if (a < 0) a = 0;
                    if (b > (int32_t)(n - k)) b = (int32_t)(n - k);
                    if (b <= a) a = b - 1;
                    if (a < 0) a = 0;
                    uint32_t run = 0, lo = 0xFFFFFFFFu, hi = 0;
                    for (uint32_t q = 0; q < k; q++) run += adc_raw_buf[(uint32_t)a + q];
                    for (int32_t s = a; ; s++) {
                        if (run < lo) lo = run;
                        if (run > hi) hi = run;
                        if (s + 1 >= b) break;
                        run += adc_raw_buf[(uint32_t)s + k]; run -= adc_raw_buf[(uint32_t)s];
                    }
                    osc_env_lo_ct[i] = (uint16_t)(lo / k);
                    osc_env_hi_ct[i] = (uint16_t)(hi / k);
                }
                osc_env_on = 1;
            }
        }
    }
    MvStoreExt(start, step, wa, n);  /* v54: keep the display points just outside the window, for X move while frozen (v81: same averaging as the window) */

    /* แกนเวลา: อัตราสุ่มจริงหลัง resample (ไม่ใช้ค่าที่ตั้งไว้) */
    osc_fs_eff = dbg_adc_rate / step;

    dbg_adc_decim  = dec;
    dbg_adc_n      = n;
    dbg_adc_us     = t_us;
    dbg_adc_min    = vmin;
    dbg_adc_max    = vmax;
    dbg_adc_frames++;
    osc_capture_tdiv = osc_tdivs_ms[osc_tdiv_i];
    osc_frame_valid = 1;
    new_frame_ready = 1;
    OSC_SET(osc_dirty      , 1);
    return;
failed:
    dbg_adc_ok = 0;
    osc_raw_valid = 0;
    adc_over_range = 0;
    adc_over_t = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    OSC_SET(osc_dirty, 1);
}

/* SLOW_BEGIN ================================================================
 * SLOW history: SysTick (1 kHz) reads ADC3 (burst of 16, averaged) ALL THE TIME, also while the display is in FAST mode.
 *   -> 8192 x 1 ms history (8.2 s).  SLOW display = newest 240 points, each the mean of the history inside one display
 *      period (T/DIV x 10 / 240).  Switching FAST <-> SLOW therefore needs no refill: the history is already there.
 *   Ticks that cannot sample (a burst capture owns the ADCs) are filled by linear interpolation at the next good tick.
 *   Frequency shown in SLOW comes from the same window (0.2 .. 60 Hz); never invented: 0 when not clean and periodic.
 *   The history can also be probed at any SLOW time base without touching the display (SlowProbeFreq) -> instant detection.
 * ========================================================================== */
#define SLOW_MIN_TDIV_MS  10.0f           /* T/DIV from this value up = SLOW mode */
#define SLOW_SPREAD_MAX   600u            /* ADC counts (~480 mV): burst spread this large cannot come from a slow signal (noisy rails show up to ~230) */
#define SLOW_ROUGH_FAST   100u            /* mean ms-to-ms jump of the history (counts) above this, with no readable frequency = a fast signal */
#define SLOW_BURST        16u             /* conversions per 1 ms tick */
#define SLOW_HIST_N       8192u           /* history length in ticks (ms) - power of two */
#define SLOW_ROUGH_TICKS  100u            /* alias guard looks at the newest 100 ms */

static volatile uint16_t slow_hist[SLOW_HIST_N];      /* burst mean per ms */
static volatile uint16_t slow_hist_rg[SLOW_HIST_N];   /* burst spread (max-min) per ms */
static volatile uint32_t slow_tick       = 0;         /* SysTick count since start */
static volatile uint32_t slow_last_t     = 0;         /* tick of the newest REAL sample (history is complete up to here) */
static volatile uint16_t slow_last_v     = 0;
static volatile uint32_t slow_valid_cnt  = 0;         /* number of real samples taken */
static volatile uint32_t slow_period_us  = 2083;      /* time per display point in SLOW mode (us) */
volatile uint16_t        dbg_slow_spread = 0;         /* mean burst spread over the newest 100 ms (SWD) */
volatile uint16_t        dbg_slow_rough  = 0;         /* mean ms-to-ms jump over the newest 100 ms (SWD) */
volatile uint32_t        dbg_slow_pts    = 0;         /* real samples taken (SWD) */
volatile uint16_t        dbg_slow_last   = 0;

#ifndef OSC_SLOW_HOST_TEST
/* ADC3: single conversion, channel 0 (PA0), 15-cycle sampling, no DMA. Called whenever slow_adc3_dirty is set. */
static void SlowAdcPrepare(void)
{
    ADC3->CR2 &= ~ADC_CR2_ADON;
    ADC3->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS);
    ADC3->SQR1 = 0u;                          /* 1 conversion in the sequence */
    ADC3->SQR3 = 0u;                          /* rank 1 = channel 0 */
    ADC3->SMPR2 = (ADC3->SMPR2 & ~7u) | 1u;   /* channel 0: 15 cycles */
    ADC3->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t d = 0; d < 2000u; d++) { }   /* tSTAB */
}

static volatile uint8_t slow_adc_fail = 0;   /* 1 = the last ADC3 conversion never finished (ADC3 switched off) */
static inline uint16_t SlowAdcRead(void)
{
    uint32_t guard = 600u;                      /* one conversion = 1.25 us = ~100 loop passes: 600 is generous, and short enough that SysTick can never starve the main loop */
    ADC3->SR = 0;
    ADC3->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC3->SR & ADC_SR_EOC)) { if (!--guard) { slow_adc_fail = 1; break; } }
    return (uint16_t)(ADC3->DR & 0x0FFFu);
}
#endif

/* called from SysTick_Handler every 1 ms */
void OscSlowTick(void)
{
    uint32_t t = slow_tick++;
    uint32_t acc = 0;
    uint16_t mn = 0xFFFF, mx = 0, v, rg;

    if (slow_busy) return;                                    /* a burst capture owns the ADCs: this tick is interpolated later */
#ifndef OSC_SLOW_HOST_TEST
    if (!adc_tri_ready) return;
    if (!(ADC3->CR2 & ADC_CR2_ADON)) slow_adc3_dirty = 1;     /* something switched ADC3 off: it never converts again (and every tick then burns its whole timeout) */
    if (slow_adc3_dirty) { SlowAdcPrepare(); slow_adc3_dirty = 0; }
    slow_adc_fail = 0;
#endif
    for (uint32_t k = 0; k < SLOW_BURST; k++) {
        for (volatile uint32_t d = (k * 3u) % 7u; d > 0u; d--) { }   /* dither the spacing so a signal at a multiple of the burst rate cannot hide inside the burst */
        uint16_t a = SlowAdcRead();
#ifndef OSC_SLOW_HOST_TEST
        if (slow_adc_fail) { slow_adc3_dirty = 1; return; }   /* re-prepare ADC3 on the next tick, keep this tick short; the gap is interpolated */
#endif
        acc += a; if (a < mn) mn = a; if (a > mx) mx = a;
    }
    v  = (uint16_t)(acc / SLOW_BURST);
    rg = (uint16_t)(mx - mn);

    if (slow_valid_cnt == 0) {                                /* very first sample: flat history until real data replaces it */
        for (uint32_t i = 0; i < SLOW_HIST_N; i++) { slow_hist[i] = v; slow_hist_rg[i] = rg; }
    } else {
        uint32_t gap = t - slow_last_t;
        if (gap > 1u) {                                       /* missed ticks: straight line from the last sample to this one */
            uint32_t k0 = (gap > SLOW_HIST_N) ? (gap - SLOW_HIST_N + 1u) : 1u;
            for (uint32_t k = k0; k < gap; k++) {
                int32_t iv = (int32_t)slow_last_v + (((int32_t)v - (int32_t)slow_last_v) * (int32_t)k) / (int32_t)gap;
                slow_hist[(slow_last_t + k) & (SLOW_HIST_N - 1u)]    = (uint16_t)iv;
                slow_hist_rg[(slow_last_t + k) & (SLOW_HIST_N - 1u)] = rg;
            }
        }
    }
    slow_hist[t & (SLOW_HIST_N - 1u)]    = v;
    slow_hist_rg[t & (SLOW_HIST_N - 1u)] = rg;
    slow_last_v = v;
    slow_last_t = t;                                          /* written last: everything up to here is complete */
    slow_valid_cnt++;
    dbg_slow_pts = slow_valid_cnt;
    dbg_slow_last = v;
}

/* T/DIV -> SLOW or FAST display. No ADC change: the history always runs. */
static void SlowSync(void)
{
    uint8_t want = (osc_tdivs_ms[osc_tdiv_i] >= SLOW_MIN_TDIV_MS);
    if (want) slow_period_us = (uint32_t)(osc_tdivs_ms[osc_tdiv_i] * (float)OSC_H_DIVS * 1000.0f / (float)N_SAMPLES);
    slow_active = want;
}

/* history -> N_SAMPLES display points of period_us each, newest point last; returns the alias-guard spread and,
 * through *rough_out, the mean ms-to-ms jump of the newest 100 ms */
static uint16_t SlowBuildPointsAt(uint16_t *out, uint32_t period_us, uint32_t t_view, uint32_t back_us, uint16_t *rough_out)
{
    uint32_t t_end = slow_last_t + 1u;                        /* exclusive; history is complete below this: spread / roughness always use the NEWEST data */
    uint32_t rs = 0, rn = 0, rj = 0;
    if (period_us < 1000u) {                                  /* 10 / 20 ms/div: a point is shorter than one history tick -> linear interpolation */
        for (uint32_t j = 0; j < N_SAMPLES; j++) {
            uint32_t off_us = (N_SAMPLES - 1u - j) * period_us + back_us;      /* newest displayed point = tick (t_view-1) minus back_us */
            t_end = t_view;
            uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
            uint32_t t_low = (t_end - 1u) - off_ms - (rem ? 1u : 0u);
            uint32_t a = slow_hist[t_low & (SLOW_HIST_N - 1u)];
            uint32_t b = rem ? slow_hist[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
            out[j] = (uint16_t)((a * rem + b * (1000u - rem)) / 1000u);
        }
        t_end = slow_last_t + 1u;
    } else
    for (uint32_t j = 0; j < N_SAMPLES; j++) {
        uint32_t off_a = ((N_SAMPLES - j) * period_us + 999u) / 1000u;
        t_end = t_view;
        uint32_t off_b = ((N_SAMPLES - 1u - j) * period_us + 999u) / 1000u;
        uint32_t sum = 0, cnt = 0;
        for (uint32_t tk = t_end - off_a; tk != t_end - off_b; tk++) { sum += slow_hist[tk & (SLOW_HIST_N - 1u)]; cnt++; }
        out[j] = (uint16_t)(cnt ? (sum / cnt) : slow_hist[(t_end - 1u) & (SLOW_HIST_N - 1u)]);
    }
    t_end = slow_last_t + 1u;
    for (uint32_t k = 1u; k <= SLOW_ROUGH_TICKS; k++) {
        uint16_t a = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)], b = slow_hist[(t_end - k - 1u) & (SLOW_HIST_N - 1u)];
        rs += slow_hist_rg[(t_end - k) & (SLOW_HIST_N - 1u)]; rn++;
        rj += (a > b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
    }
    *rough_out = (uint16_t)(rj / rn);
    return (uint16_t)(rs / rn);
}

/* v86 PEAK DETECT for SLOW (the twin of v83 in FAST): a point >= 1 ms averages its history ticks, so with < 8 points per period the
 * drawn wave shrank (30 Hz: 125 px at 50 ms/div, 121 at 100, 110 at 200, a flat line at 500) while the readout stayed. Min / max of the ticks behind
 * each point (the same ranges as SlowBuildPointsAt) -> osc_env_lo_ct / osc_env_hi_ct, drawn as a band by MapToScreen / TraceColumns.
 * Points shorter than a tick (10 / 20 ms/div) always have >= 8 points per period at SLOW frequencies: nothing to do. */
static void SlowEnvAt(uint32_t period_us, uint32_t t_view)
{
    if (period_us < 1000u) return;
    for (uint32_t j = 0; j < N_SAMPLES; j++) {
        uint32_t off_a = ((N_SAMPLES - j) * period_us + 999u) / 1000u;
        uint32_t off_b = ((N_SAMPLES - 1u - j) * period_us + 999u) / 1000u;
        uint16_t lo = 0xFFFF, hi = 0;
        for (uint32_t tk = t_view - off_a; tk != t_view - off_b; tk++) {
            uint16_t v = slow_hist[tk & (SLOW_HIST_N - 1u)];
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (lo > hi) lo = hi = slow_hist[(t_view - 1u) & (SLOW_HIST_N - 1u)];
        osc_env_lo_ct[j] = lo; osc_env_hi_ct[j] = hi;
    }
    osc_env_on = 1;
}

static uint16_t SlowBuildPoints(uint16_t *out, uint32_t period_us, uint16_t *rough_out)
{
    return SlowBuildPointsAt(out, period_us, slow_last_t + 1u, 0u, rough_out);
}

/* v80: the real display points just BEFORE the SLOW window (point -1-c), same averaging as SlowBuildPointsAt, for the display filter's left
 * neighbours (OscExtPoint). Without them OscProcessFrame slid the edge window inward and the first `half` points came out as one flat block
 * = the spike at the left edge (the FAST twin was fixed in v76/v78 with mv_ext_lo). Points older than the history are not stored. */
#define SLOW_EXT_N 10                                         /* display filter half-width max (window 21) */
static uint16_t slow_ext_lo[SLOW_EXT_N];
static int      slow_nlo = 0;
static void SlowStoreExt(uint32_t period_us, uint32_t t_view, uint32_t back_us)
{
    uint32_t have = (slow_valid_cnt < SLOW_HIST_N) ? slow_valid_cnt : SLOW_HIST_N;   /* ticks of history that exist */
    uint32_t newest = slow_last_t;
    slow_nlo = 0;
    for (uint32_t c = 0; c < SLOW_EXT_N; c++) {
        uint32_t k = N_SAMPLES + c;                           /* points back from the newest displayed point */
        if (period_us < 1000u) {
            uint32_t off_us = k * period_us + back_us;
            uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
            uint32_t t_low = (t_view - 1u) - off_ms - (rem ? 1u : 0u);
            if (newest - t_low + 1u > have) break;
            uint32_t a = slow_hist[t_low & (SLOW_HIST_N - 1u)];
            uint32_t b = rem ? slow_hist[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
            slow_ext_lo[c] = (uint16_t)((a * rem + b * (1000u - rem)) / 1000u);
        } else {
            uint32_t off_a = ((k + 1u) * period_us + 999u) / 1000u;
            uint32_t off_b = (k * period_us + 999u) / 1000u;
            uint32_t sum = 0, cnt = 0;
            if (newest + 1u - (t_view - off_a) > have) break;
            for (uint32_t tk = t_view - off_a; tk != t_view - off_b; tk++) { sum += slow_hist[tk & (SLOW_HIST_N - 1u)]; cnt++; }
            if (!cnt) break;
            slow_ext_lo[c] = (uint16_t)(sum / cnt);
        }
        slow_nlo = (int)c + 1;
    }
}

/* v80: the right-edge neighbours of a SLOW frame. In roll mode the newest displayed point IS the newest tick, so nothing real exists after it.
 * Continue the trace one (or m) measured periods back: raw_samples[N + c - m*P], linear interpolation at the fractional position. A periodic
 * signal (sine, square, triangle) continues exactly, so the edge gets ~the same centred filter as the middle instead of the slide that repeated
 * one value over the last `half` points (the flat block). slow_ext_P = 0 (no frequency): the old fallback rule. */
static float slow_ext_P = 0.0f;                               /* display points per period of THIS frame's raw_samples */
static int SlowExtHi(int c, float *v)
{
    if (slow_ext_P < 2.0f || c < 0 || c >= SLOW_EXT_N) return 0;
    float m = ceilf((float)(c + 1) / slow_ext_P);
    float x = (float)(N_SAMPLES + c) - m * slow_ext_P;
    if (x < 0.0f) return 0;
    int   i0 = (int)x;
    float fr = x - (float)i0;
    if (i0 >= N_SAMPLES - 1) { *v = (float)raw_samples[N_SAMPLES - 1]; return 1; }
    *v = (float)raw_samples[i0] * (1.0f - fr) + (float)raw_samples[i0 + 1] * fr;
    return 1;
}

static int SlowExtPoint(int k, float *v)                      /* v80: display filter neighbour k (<0 or >=N_SAMPLES) of THIS SLOW frame */
{
    if (k < 0 && (-k - 1) < slow_nlo) { *v = (float)slow_ext_lo[-k - 1]; return 1; }   /* left: real history */
    if (k >= N_SAMPLES) return SlowExtHi(k - N_SAMPLES, v);                           /* right: one period back */
    return 0;
}

/* TRIGGERED VIEW for 10..50 ms/div (windows of 0.1-0.5 s): lock the picture to the most recent rising (or falling) edge of the history,
 * with the edge 10 % from the left, like a scope. The picture then stands still instead of scrolling, and a change of frequency
 * or amplitude shows up within one period. v87: every SLOW T/DIV in AUTO (the 8 s history covers 1.5 windows even at 500 ms/div); ROLL scrolls.
 * Returns 1 and the view end (tick, sub-tick offset) when a clean edge exists, 0 = free-run (no signal / no edge). */
static volatile uint8_t dbg_slow_trig = 0;
static float slow_pv_cur = 0.0f; static uint32_t slow_pv_t = 0u;   /* drawn time base (us per point) and the tick of its last update */
static uint8_t SlowFindTrigger(uint32_t period_us, uint32_t *t_view, uint32_t *back_us)
{
    uint32_t w_ms = (uint32_t)(((uint64_t)period_us * N_SAMPLES) / 1000u);          /* window length in ms */
    uint32_t last = slow_last_t;
    uint32_t pre  = w_ms / 10u;                                                      /* edge sits 10 % from the left */
    uint32_t reach = w_ms + w_ms / 2u;                                               /* look back 1.5 windows */
    if (!osc_stable || slow_valid_cnt < reach + 8u) return 0;   /* v87: AUTO locks at every SLOW T/DIV and frequency (was >= 45 Hz and <= 50 ms/div only, so ROLL/AUTO did nothing at 30 Hz); ROLL never */
    uint32_t newest_ok = last - (w_ms - pre) - 1u;                                   /* view must end before the newest tick */
    uint32_t s0 = last - reach;
    uint16_t mn = 0xFFFF, mx = 0;
    for (uint32_t t = s0; t <= last; t++) {
        uint16_t v = slow_hist[t & (SLOW_HIST_N - 1u)];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    if ((uint32_t)(mx - mn) < 100u) return 0;                                        /* no signal worth locking to */
    float mid = 0.5f * (float)(mx + mn), hy = 0.2f * (float)(mx - mn);
    int found = 0; float tc = 0.0f;
    uint8_t armed = 0;
    uint16_t pv = slow_hist[s0 & (SLOW_HIST_N - 1u)];
    armed = osc_trig_fall ? ((float)pv > mid) : ((float)pv < mid);
    for (uint32_t t = s0 + 1u; t <= newest_ok; t++) {
        float v = (float)slow_hist[t & (SLOW_HIST_N - 1u)], p = (float)slow_hist[(t - 1u) & (SLOW_HIST_N - 1u)];
        if (osc_trig_fall) {
            if (v > mid + hy) armed = 1;
            else if (armed && p >= mid && v < mid) { tc = (float)(t - 1u) + (p - mid) / (p - v); found = 1; armed = 0; }
        } else {
            if (v < mid - hy) armed = 1;
            else if (armed && p < mid && v >= mid) { tc = (float)(t - 1u) + (mid - p) / (v - p); found = 1; armed = 0; }
        }
    }
    if (!found) return 0;
    {   /* newest displayed point = edge + 0.9 window - one point; express it as tick (ceil) minus a sub-tick offset */
        float t_new = tc + (float)(w_ms - pre) - (float)period_us / 1000.0f;
        uint32_t tick = (uint32_t)ceilf(t_new);
        if (tick > last) return 0;
        *t_view = tick + 1u;                                                         /* exclusive end */
        *back_us = (uint32_t)(((float)tick - t_new) * 1000.0f);
        if (*back_us > 999u) *back_us = 999u;
    }
    return 1;
}

/* 1 = the newest 100 ms of history cannot belong to a slow signal of frequency f (0 = not readable) and amplitude vpp_counts:
 * a slow sine/triangle moves 0.002*f*Vpp counts per ms on average (square less); allow twice that plus noise. */
static uint8_t SlowLooksFast(float f, float vpp_counts, uint16_t spread, uint16_t rough)
{
    if (spread > SLOW_SPREAD_MAX) return 1;
    if (f > 0.0f) return (rough > (uint32_t)(0.004f * f * vpp_counts) + 60u) ? 1u : 0u;
    {   uint32_t thr = (uint32_t)(0.26f * vpp_counts);       /* a slow signal (< 55 Hz) moves at most ~0.11 x Vpp per ms; 0.16 x = about 80 Hz */
        if (thr < SLOW_ROUGH_FAST) thr = SLOW_ROUGH_FAST;
        return (rough > thr) ? 1u : 0u;
    }
}

/* Frequency of a SLOW window: mid-level rising edges with 20 % hysteresis. Needs >= 2 edges, regularly spaced
 * (a noise-only trace has random spacing) - returns 0 otherwise. Never invents a number. */
static float SlowFreqFromPts(const uint16_t *p, float fs, float *vpp_out, int store)
{
    uint16_t mn = 4095, mx = 0;
    float at_[40];
    uint32_t edges = 0;
    int armed = 0;
    for (uint32_t i = 0; i < N_SAMPLES; i++) { if (p[i] < mn) mn = p[i]; if (p[i] > mx) mx = p[i]; }
    float vpp = (float)(mx - mn) * ADC_VREF / ADC_FULL;
    if (vpp_out) *vpp_out = vpp;
    if (vpp < 0.05f) return 0.0f;
    float mid = (float)(mx + mn) * 0.5f, hyst = (float)(mx - mn) * 0.2f;
    for (uint32_t i = 1; i < N_SAMPLES; i++) {
        float prev = (float)p[i - 1], v = (float)p[i];
        if (v < mid - hyst) armed = 1;
        if (armed && prev < mid && v >= mid) {
            if (edges < 40u) at_[edges] = (float)(i - 1) + (mid - prev) / (v - prev);
            edges++;
            armed = 0;
        }
    }
    if (edges < 3u || edges > 40u) return 0.0f;                /* 3 edges = 2 intervals that must agree; 2 edges can be a level change, not a period */
    /* Use only the NEWEST run of evenly spaced edges. After a frequency change (1 Hz -> 2 Hz -> 3 Hz) the window holds the old
     * and the new signal; averaging over all edges gave in-between numbers (2.14, 2.67 ...). The run goes back from the newest
     * edge while the spacing stays within 15 % of the newest period, and needs 3 edges (2 equal periods). */
    {
        uint32_t k0 = edges - 1u;
        float ref = at_[edges - 1u] - at_[edges - 2u];
        if (ref <= 0.0f) return 0.0f;
        while (k0 > 0u) {
            float dt = at_[k0] - at_[k0 - 1u];
            if (dt > 1.15f * ref || dt < 0.85f * ref) break;
            k0--;
        }
        if (edges - k0 < 3u) return 0.0f;                     /* newest run is shorter than 2 periods: nothing clean to show yet */
        {   /* shift the run to the front so the rest of the function sees only it */
            uint32_t m = edges - k0;
            for (uint32_t k = 0; k < m; k++) at_[k] = at_[k0 + k];
            edges = m;
        }
    }
    float first = at_[0], last = at_[edges - 1u];
    if (last <= first) return 0.0f;
    float mean = (last - first) / (float)(edges - 1u);
    uint32_t bad = 0;
    for (uint32_t k = 1; k < edges; k++) {
        float dt = at_[k] - at_[k - 1u];
        if (dt > 1.3f * mean || dt < 0.7f * mean) bad++;
    }
    if (bad > 0u) return 0.0f;                                /* spacing not regular: not a clean periodic signal */
    float f = (float)(edges - 1u) * fs / (last - first);
    if (!(f >= 0.2f && f <= fs / 8.0f)) return 0.0f;
    if (store) {                                              /* exact DC / RMS over the whole cycles between the first and last edge */
        uint32_t i0 = (uint32_t)(first + 1.0f), i1 = (uint32_t)last;
        if (i1 > i0 + 1u && i1 < N_SAMPLES) {
            uint32_t sm = 0;
            float cnt = (float)(i1 - i0 + 1u), mean, sq = 0.0f;
            for (uint32_t i = i0; i <= i1; i++) sm += p[i];
            mean = (float)sm / cnt;
            for (uint32_t i = i0; i <= i1; i++) { float d = (float)p[i] - mean; sq += d * d; }
            osc_raw_mean_ct = mean; osc_raw_rms_ct = sqrtf(sq / cnt);
            osc_raw_min_ct = (float)mn; osc_raw_max_ct = (float)mx; osc_raw_valid = 1;
        }
    }
    return f;
}

static uint8_t SlowRecentFlat(void)
{
    uint32_t t_end = slow_last_t + 1u;
    uint16_t mn = 0xFFFF, mx = 0;
    if (slow_valid_cnt < 1500u) return 0;
    for (uint32_t k = 1u; k <= 1500u; k++) {
        uint16_t v = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    return ((float)(mx - mn) * ADC_VREF / ADC_FULL < 0.20f) ? 1u : 0u;
}

/* 1 = the newest 1.5 s of history holds a clearly moving signal (>= 0.3 V) whose speed is clearly slow (< ~40 Hz): estimated from the mean ms-to-ms step,
 * a sine moves 0.002 * f * Vpp counts per ms. Needs no readable frequency, so it works within ~0.3 s of a signal appearing. */
static uint8_t SlowSlowish(void)
{
    uint32_t t_end = slow_last_t + 1u, rs = 0;
    uint16_t mn = 0xFFFF, mx = 0;
    if (slow_valid_cnt < 1500u) return 0;
    for (uint32_t k = 1u; k <= 1500u; k++) {
        uint16_t v = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    if ((float)(mx - mn) * ADC_VREF / ADC_FULL < 0.30f) return 0;
    for (uint32_t k = 1u; k <= SLOW_ROUGH_TICKS; k++) {
        uint16_t a = slow_hist[(t_end - k) & (SLOW_HIST_N - 1u)], b = slow_hist[(t_end - k - 1u) & (SLOW_HIST_N - 1u)];
        rs += (a > b) ? (uint32_t)(a - b) : (uint32_t)(b - a);
    }
    return ((float)(rs / SLOW_ROUGH_TICKS) < 0.08f * (float)(mx - mn) + 12.0f) ? 1u : 0u;
}

/* What would SLOW time base ti show? (0 = nothing clean). Reads the history only: the display is not touched. */
static float SlowProbeFreq(int ti)
{
    uint16_t pts[N_SAMPLES], rough, spread;
    float vpp = 0.0f, f;
    uint32_t pus = (uint32_t)(osc_tdivs_ms[ti] * (float)OSC_H_DIVS * 1000.0f / (float)N_SAMPLES);
    if (slow_valid_cnt < 1000u) return 0.0f;
    spread = SlowBuildPoints(pts, pus, &rough);
    f = SlowFreqFromPts(pts, 1.0e6f / (float)pus, &vpp, 0);
    if (f <= 0.0f) return 0.0f;
    return SlowLooksFast(f, vpp * (float)ADC_FULL / ADC_VREF, spread, rough) ? 0.0f : f;
}

/* history -> raw_samples[] like AdcCaptureFrame(), plus the osc_* values the readout needs */
static void SlowCaptureFrame(void)
{
    uint16_t pts[N_SAMPLES];
    uint16_t vmin = 0xFFFF, vmax = 0, spread, rough;

    new_frame_ready = 0;
    osc_frame_valid = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_meas = (OscMeasure){0};
    strcpy(osc_det, "INVALID");
    slow_ext_P = 0.0f;                                        /* v80: set below once this frame's frequency is known */
    osc_env_on = 0;                                           /* v86: set below when the picture needs the peak-detect band */

    if (slow_valid_cnt == 0) { dbg_adc_ok = 0; osc_dirty = 1; return; }
    uint32_t ext_tv, ext_bk = 0, ext_pus = slow_period_us;    /* v80: view + time base of the drawn points, for SlowStoreExt / slow_ext_P */
    {   uint32_t tv = 0, bk = 0;
        if (SlowFindTrigger(slow_period_us, &tv, &bk)) { spread = SlowBuildPointsAt(pts, slow_period_us, tv, bk, &rough); dbg_slow_trig = 1; ext_tv = tv; ext_bk = bk; }
        else { ext_tv = slow_last_t + 1u; spread = SlowBuildPointsAt(pts, slow_period_us, ext_tv, 0u, &rough); dbg_slow_trig = 0; }   /* v80: = SlowBuildPoints, view kept */
    }
    for (uint32_t i = 0; i < N_SAMPLES; i++) {
        if (pts[i] < vmin) vmin = pts[i];
        if (pts[i] > vmax) vmax = pts[i];
    }
    {   /* DISPLAY ONLY (the measurement below does not depend on it): when the selected time base changes, the drawn one moves toward it
         * by 15 % per frame instead of jumping (a 10x change takes ~1.4 s) */
        uint16_t dpts[N_SAMPLES], r2;
        const uint16_t *src = pts;
        uint32_t nowt = HAL_GetTick();
        float tgt = (float)slow_period_us;
        if (slow_pv_cur < 1.0f || (nowt - slow_pv_t) > 1500u) slow_pv_cur = tgt;
        slow_pv_t = nowt;
        if (slow_pv_cur < tgt)      { slow_pv_cur *= 1.15f; if (slow_pv_cur > tgt) slow_pv_cur = tgt; }
        else if (slow_pv_cur > tgt) { slow_pv_cur /= 1.15f; if (slow_pv_cur < tgt) slow_pv_cur = tgt; }
        if (!dbg_slow_trig && slow_pv_cur != tgt) { (void)SlowBuildPointsAt(dpts, (uint32_t)slow_pv_cur, ext_tv, 0u, &r2); src = dpts; }
        for (uint32_t i = 0; i < N_SAMPLES; i++) raw_samples[i] = (int16_t)src[i];
        ext_pus = (src == dpts) ? (uint32_t)slow_pv_cur : slow_period_us;
        SlowStoreExt(ext_pus, ext_tv, ext_bk);   /* v80 */
    }
    osc_fs_eff   = 1000000.0f / (float)slow_period_us;
    osc_raw_valid = 0;
    osc_raw_freq = SlowFreqFromPts(pts, osc_fs_eff, &osc_raw_vpp, 1);
    if (osc_raw_freq <= 0.0f) {   /* the picture's window shows < 2 clean periods (1 Hz in a 200 ms/div window): measure on ANY SLOW window that does.
                                   * Picture and numbers are independent, so the picture can keep its time base while the frequency changes. */
        for (uint32_t k = 0; k < 6u; k++) {
            int ti = (int)(OSC_NT - 1u - k);
            uint16_t mp[N_SAMPLES], mr = 0, msp;
            float mv = 0.0f, mf;
            uint32_t mus;
            if (ti == osc_tdiv_i) continue;
            mus = (uint32_t)(osc_tdivs_ms[ti] * (float)OSC_H_DIVS * 1000.0f / (float)N_SAMPLES);
            msp = SlowBuildPoints(mp, mus, &mr);
            mf  = SlowFreqFromPts(mp, 1.0e6f / (float)mus, &mv, 1);
            if (mf <= 0.0f || mf > 130.0f) continue;                  /* SLOW never reports a frequency above ~130 Hz (FAST's job) */
            if (SlowLooksFast(mf, mv * (float)ADC_FULL / ADC_VREF, msp, mr)) { osc_raw_valid = 0; continue; }
            osc_raw_freq = mf; osc_raw_vpp = mv; spread = msp; rough = mr;
            break;
        }
    }
    dbg_slow_spread = spread;
    dbg_slow_rough  = rough;
    slow_fast_flag  = 0;
    if (SlowLooksFast(osc_raw_freq, osc_raw_vpp * (float)ADC_FULL / ADC_VREF, spread, rough)) {   /* ms-to-ms jumps a slow signal cannot make */
        slow_fast_flag = 1;
        osc_raw_freq   = 0.0f;               /* never show an aliased number */
        osc_raw_valid  = 0;
    }
    if (osc_raw_freq > 0.0f && ext_pus > 0u) slow_ext_P = (1.0e6f / (float)ext_pus) / osc_raw_freq;   /* v80: points per period of the drawn picture */
    slow_trig_on = dbg_slow_trig;                             /* v86 header */
    if (slow_ext_P > 0.0f && slow_ext_P < 12.0f) SlowEnvAt(ext_pus, ext_tv);   /* v86: < 12 points per period (8 still drew 3.5 % short): draw the band */
    osc_capture_tdiv = osc_tdivs_ms[osc_tdiv_i];
    dbg_adc_ok       = 1;
    dbg_adc_min      = vmin;
    dbg_adc_max      = vmax;
    dbg_adc_frames++;
    osc_frame_valid  = 1;
    new_frame_ready  = 1;
    osc_dirty        = 1;
}
/* SLOW_END ================================================================= */

/* MOVE_DATA_BEGIN ============================================================
 * The data that X move pans over. FAST: the display points just outside the window (before it / after it), computed from the raw capture of THIS frame
 * (the next capture overwrites adc_raw_buf while the picture is drawn, so it cannot be read later). SLOW: a copy of the 8 s history taken at the moment of
 * the freeze (the live history keeps running). Nothing is invented: outside these ranges X move stops. */
#define MV_EXT_MAX 256
static uint16_t mv_ext_lo[MV_EXT_MAX], mv_ext_hi[MV_EXT_MAX];   /* lo[j] = display point -1-j, hi[j] = display point N_SAMPLES+j */
static int      mv_nlo = 0, mv_nhi = 0;                          /* how many of each exist */
static uint16_t mv_snap[SLOW_HIST_N];
static uint32_t mv_snap_end = 0, mv_snap_period = 1000u;
static uint8_t  mv_slow_mode = 0;
/* v57 ZOOM data: a copy of the raw capture region behind the display window (the next DMA capture overwrites adc_raw_buf), taken together with the ext points. */
static uint16_t mv_raw[ADC_RAW_MAX];
static uint32_t mv_raw_off = 0, mv_raw_n = 0, mv_z_w = 1;     /* mv_raw[j] = adc_raw_buf[mv_raw_off + j]; mv_raw_n = 0: no raw data */
static float    mv_z_step = 1.0f;
static uint32_t mv_z_start = 0;

static int OscExtPoint(int k, float *v)       /* v76: the filter's real neighbours just outside the screen (FAST only) */
{
    if (slow_active) {                                           /* SLOW frames do not refresh mv_ext_*: never use stale points */
        return SlowExtPoint(k, v);                               /* v80: SLOW neighbours of THIS frame (defined in SLOW) */
    }
    if (k < 0 && (-k - 1) < mv_nlo)            { *v = (float)mv_ext_lo[-k - 1]; return 1; }
    if (k >= N_SAMPLES && (k - N_SAMPLES) < mv_nhi) { *v = (float)mv_ext_hi[k - N_SAMPLES]; return 1; }
    return 0;
}

static FAST_O2 void MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n)
{
    int k, cnt;
    mv_nlo = 0; mv_nhi = 0;
    for (cnt = 0, k = -1; cnt < MV_EXT_MAX; cnt++, k--) {
        int32_t c = (int32_t)start + (int32_t)floorf((float)k * step + 0.5f);
        int32_t lo = c - (int32_t)((w - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0 || lo + (int32_t)w > (int32_t)n) break;
        for (uint32_t q = 0; q < w; q++) acc += adc_raw_buf[(uint32_t)lo + q];
        mv_ext_lo[cnt] = (uint16_t)(acc / w);
        mv_nlo = cnt + 1;
    }
    for (cnt = 0, k = N_SAMPLES; cnt < MV_EXT_MAX; cnt++, k++) {
        int32_t c = (int32_t)start + (int32_t)((float)k * step + 0.5f);
        int32_t lo = c - (int32_t)((w - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0 || lo + (int32_t)w > (int32_t)n) break;
        for (uint32_t q = 0; q < w; q++) acc += adc_raw_buf[(uint32_t)lo + q];
        mv_ext_hi[cnt] = (uint16_t)(acc / w);
        mv_nhi = cnt + 1;
    }
    {   /* v57: keep the raw samples of everything the pan can show (FAST_O2 memcpy of at most ADC_RAW_MAX samples) */
        float lo_f = (float)start - ((float)mv_nlo + 1.0f) * step - (float)w;
        float hi_f = (float)start + ((float)(N_SAMPLES + mv_nhi) + 1.0f) * step + 2.0f * (float)w + 2.0f;
        uint32_t a = (lo_f > 0.0f) ? (uint32_t)lo_f : 0u, b = (hi_f < (float)n) ? (uint32_t)hi_f : n;
        if (b > ADC_RAW_MAX) b = ADC_RAW_MAX;
        if (b > a) { memcpy(mv_raw, &adc_raw_buf[a], (size_t)(b - a) * sizeof(uint16_t)); mv_raw_off = a; mv_raw_n = b - a; }
        else mv_raw_n = 0;
        mv_z_start = start; mv_z_step = step; mv_z_w = w;
    }
}

static void MvFreezeCapture(void)            /* called once, at the moment of the freeze: fixes the pan range */
{
    mv_x_min = 0; mv_x_max = 0; mv_slow_mode = 0;
    if (slow_active) {
        uint32_t valid = slow_valid_cnt;
        if (valid > SLOW_HIST_N) valid = SLOW_HIST_N;
        if (valid < 64u) return;
        for (uint32_t i = 0; i < SLOW_HIST_N; i++) mv_snap[i] = slow_hist[i];
        mv_snap_end    = slow_last_t + 1u;
        mv_snap_period = slow_period_us ? slow_period_us : 1000u;
        {   uint32_t win   = (uint32_t)(((uint64_t)N_SAMPLES * mv_snap_period + 999u) / 1000u) + 2u;     /* ticks the window needs */
            uint32_t avail = valid - 4u;                                                                  /* ticks of real history behind the newest */
            if (avail > win) {
                uint32_t pts = (uint32_t)(((uint64_t)(avail - win) * 1000u) / mv_snap_period);
                mv_x_min = -(int)((pts > 30000u) ? 30000u : pts);
            }
        }
        mv_slow_mode = 1;
    } else {
        mv_x_min = -mv_nlo;
        mv_x_max = mv_nhi;
    }
}

static uint16_t MvSlowPoint(int j, int pan)  /* display point j of the SLOW window moved |pan| points earlier (same averaging as SlowBuildPointsAt) */
{
    uint32_t period = mv_snap_period;
    uint32_t back_us = (uint32_t)(-pan) * period;
    uint32_t t_end = mv_snap_end;
    if (period < 1000u) {
        uint32_t off_us = (N_SAMPLES - 1u - (uint32_t)j) * period + back_us;
        uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
        uint32_t t_low = (t_end - 1u) - off_ms - (rem ? 1u : 0u);
        uint32_t a = mv_snap[t_low & (SLOW_HIST_N - 1u)];
        uint32_t b = rem ? mv_snap[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
        return (uint16_t)((a * rem + b * (1000u - rem)) / 1000u);
    } else {
        uint32_t tb = back_us / 1000u;
        uint32_t off_a = (((N_SAMPLES - (uint32_t)j) * period + 999u) / 1000u) + tb;
        uint32_t off_b = (((N_SAMPLES - 1u - (uint32_t)j) * period + 999u) / 1000u) + tb;
        uint32_t sum = 0, cnt = 0;
        for (uint32_t tk = t_end - off_a; tk != t_end - off_b; tk++) { sum += mv_snap[tk & (SLOW_HIST_N - 1u)]; cnt++; }
        return (uint16_t)(cnt ? (sum / cnt) : mv_snap[(t_end - off_b) & (SLOW_HIST_N - 1u)]);
    }
}

/* v57: time zoom. Output pixel i shows display position u = (1920 + (i - 120) * 16 / N + mv_x_q) / 16 of the frozen picture (N = mv_zoom_x).
 * FAST: raw position r = start + u * step. Pixels are step/N raw samples apart: >= 1 -> boxcar of round(w/N) raw samples (real detail),
 *       < 1 (or no raw copy) -> linear interpolation between neighbours. SLOW: the frozen 1 ms history, same rule (interpolate below 1 ms, average above).
 * The display filter shrinks with the zoom (window / N) so the extra detail is not smeared away again. */
static float MvZoomCount(int32_t U)
{
    const int N = mv_zoom_x;
    if (mv_slow_mode) {
        uint32_t period = mv_snap_period, t_end = mv_snap_end;
        int64_t off64 = ((int64_t)((N_SAMPLES - 1) * 16) - (int64_t)U) * (int64_t)period / 16;
        uint32_t off_us = (off64 > 0) ? (uint32_t)off64 : 0u, span = period / (uint32_t)N;
        if (span < 1000u) {
            uint32_t off_ms = off_us / 1000u, rem = off_us % 1000u;
            uint32_t t_low = (t_end - 1u) - off_ms - (rem ? 1u : 0u);
            uint32_t a = mv_snap[t_low & (SLOW_HIST_N - 1u)];
            uint32_t b = rem ? mv_snap[(t_low + 1u) & (SLOW_HIST_N - 1u)] : a;
            return (float)((a * rem + b * (1000u - rem)) / 1000u);
        } else {
            uint32_t off_a = (off_us + span + 999u) / 1000u, off_b = (off_us + 999u) / 1000u, sum = 0, c = 0;
            for (uint32_t tk = t_end - off_a; tk != t_end - off_b; tk++) { sum += mv_snap[tk & (SLOW_HIST_N - 1u)]; c++; }
            return (float)(c ? (sum / c) : mv_snap[(t_end - off_b) & (SLOW_HIST_N - 1u)]);
        }
    }
    if (mv_raw_n > 0) {
        float r = (float)mv_z_start + ((float)U / 16.0f) * mv_z_step - (float)mv_raw_off;     /* index inside mv_raw */
        float sp = mv_z_step / (float)N;
        int last = (int)mv_raw_n - 1;
        if (sp >= 1.0f) {
            int w2 = ((int)mv_z_w + N / 2) / N, lo; if (w2 < 1) w2 = 1;
            lo = (int)floorf(r + 0.5f) - (w2 - 1) / 2;
            if (lo + w2 > last + 1) lo = last + 1 - w2;
            if (lo < 0) lo = 0;
            if (lo + w2 > last + 1) w2 = last + 1 - lo;
            { uint32_t acc = 0; for (int q = 0; q < w2; q++) acc += mv_raw[lo + q]; return (float)acc / (float)w2; }
        } else {
            int i0 = (int)floorf(r); float fr = r - (float)i0; int i1 = i0 + 1;
            i0 = OscClampI(i0, 0, last); i1 = OscClampI(i1, 0, last);
            return (float)mv_raw[i0] + ((float)mv_raw[i1] - (float)mv_raw[i0]) * fr;
        }
    }
    {   /* no stored samples: straight lines between the 240 display points */
        float u = (float)U / 16.0f; int i0; float fr;
        if (u < 0.0f) u = 0.0f;
        if (u > (float)(N_SAMPLES - 1)) u = (float)(N_SAMPLES - 1);
        i0 = (int)floorf(u); fr = u - (float)i0;
        return (float)raw_samples[i0] + (float)(raw_samples[OscClampI(i0 + 1, 0, N_SAMPLES - 1)] - raw_samples[i0]) * fr;
    }
}

static void MvBuildZoom(void)
{
    float cnt[N_SAMPLES];
    const float kv = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;   /* v84 */
    int win = OscFiltWin(), half, zw;   /* v82: same window as OscProcessFrame */
    zw = win / mv_zoom_x; if (zw < 1) zw = 1; if ((zw % 2) == 0) zw++;
    half = zw / 2;
    for (int i = 0; i < N_SAMPLES; i++) cnt[i] = MvZoomCount(1920 + ((i - N_SAMPLES / 2) * 16) / mv_zoom_x + mv_x_q);
    for (int i = 0; i < N_SAMPLES; i++) {
        float acc = 0.0f;
        for (int j = -half; j <= half; j++) acc += cnt[OscClampI(i + j, 0, N_SAMPLES - 1)];
        mv_view_v[i] = (acc / (float)zw - osc_offset_ct) * kv;
    }
    mv_view_on = 1;
}

static void MvBuildView(void)                /* mv_view_v[] = the frozen capture panned by mv_x_pts (same calibration + filter as OscProcessFrame) */
{
    float cnt[N_SAMPLES];
    if (mv_zoom_x > 1) { MvBuildZoom(); return; }       /* v57 */
    const float kv = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;   /* v84 */
    int win = OscFiltWin(), half;       /* v82: same window as OscProcessFrame */
    half = win / 2;
    for (int i = 0; i < N_SAMPLES; i++) {
        int k = i + mv_x_pts;
        if (mv_slow_mode)  cnt[i] = (float)MvSlowPoint(i, mv_x_pts);
        else if (k < 0)    cnt[i] = (float)mv_ext_lo[OscClampI(-k - 1, 0, MV_EXT_MAX - 1)];
        else if (k < N_SAMPLES) cnt[i] = (float)raw_samples[k];
        else               cnt[i] = (float)mv_ext_hi[OscClampI(k - N_SAMPLES, 0, MV_EXT_MAX - 1)];
    }
    for (int i = 0; i < N_SAMPLES; i++) {
        float acc = 0.0f;
        for (int j = -half; j <= half; j++) acc += cnt[OscClampI(i + j, 0, N_SAMPLES - 1)];
        mv_view_v[i] = (acc / (float)win - osc_offset_ct) * kv;
    }
    mv_view_on = (mv_x_pts != 0);
}
/* MOVE_DATA_END ============================================================= */

#else  /* !ADC_INPUT_ENABLE */
static void MvStoreExt(uint32_t start, float step, uint32_t w, uint32_t n) { (void)start; (void)step; (void)w; (void)n; }
static int OscExtPoint(int k, float *v) { (void)k; (void)v; return 0; }
static void MvFreezeCapture(void) { mv_x_min = mv_x_max = 0; }
static void MvBuildView(void) { mv_view_on = 0; }
void OscSlowTick(void) { }
static float SlowProbeFreq(int ti) { (void)ti; return 0.0f; }
static uint8_t SlowSlowish(void) { return 0u; }
#endif /* ADC_INPUT_ENABLE */

/* USER CODE END 0 */

void SystemClock_Config(void);

/* Step 1 of FreeRTOS refactor: the whole former main-loop body, verbatim, in one task */
static void vMainTask(void *argument)
{
    (void)argument;
    static uint32_t last_frame_tick  = 0;
    static uint32_t last_header_tick = 0;

    /* v66b: one-shot start-up, runs AFTER vTaskStartScheduler() so xSemaphoreTake/Give are legal
     * (xOscMutex was created in main() before xTaskCreate; creation does not block, only Take/Give must wait for the scheduler). */
    UartPanel();              /* แสดงเมนู/สถานะครั้งแรกออกพอร์ต serial */
    DrawGrid();
    DrawCursor();
    DrawTopPanel();

    for (;;)
    {
        /* USER CODE BEGIN 3 */

        uint32_t now = HAL_GetTick();

#if ADC_INPUT_ENABLE
        if (adc_pend && (hold_active || osc_auto_stage || (now - adc_pd_tick) > 400u)) AdcAbortPending();   /* nobody collected it: give the ADC and the 1 kHz sampler back */
#endif
        /* --- Button handling (every loop) --------------------------------- */
        HandleButtons(now);
        UartPoll();               /* รับคำสั่งจาก USART3 (แทนปุ่มจริง) */
        if (osc_cur_dirty) {      /* v85: C1/C2 moved while frozen: repaint the cursor columns now, not on the 33 ms frame */
            if (hold_active && cursors_on && !osc_dirty) {
                { uint32_t pq0 = DWT->CYCCNT; CursorsMoveFast(); prof_us[6] = AdcElapsedUs(pq0); }
                if ((now - last_header_tick) >= 50u && (now - last_header_tick) < HEADER_MS) last_header_tick = now - HEADER_MS;   /* U1/U2/dV/dt follow */
            } else if (!hold_active) osc_cur_dirty = 0;
        }


        /* --- Waveform frame update (~30 FPS) ------------------------------ */
        if (now - last_frame_tick >= FRAME_MS)
        {
            last_frame_tick = now;
            uint32_t pf0 = DWT->CYCCNT;

#if SHOW_FPS
            {
                static uint32_t t_prev = 0;
                dbg_loop_ms = (t_prev == 0) ? 0 : (now - t_prev);
                t_prev = now;
            }
#endif

            /* Determine if we should redraw this frame */
            uint8_t should_draw = 0;
            if      (trigger_mode == TRIG_AUTO)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_NORM)   should_draw = !hold_active;
            else if (trigger_mode == TRIG_SINGLE) should_draw = (!hold_active && !single_captured);
            if (osc_dirty) should_draw = 1;      /* ปรับ setting ขณะ HOLD -> วาดใหม่ 1 เฟรม */

            if (should_draw)
            {
                /* ============================================================
                 * INTEGRATION POINT — replace everything in this block when merging
                 * ============================================================
                 * Current (mockup):
                 *   GenerateFakeSineWave() fills raw_samples[] + osc_meas.
                 *
                 * After merge (teammate's code):
                 *   if (!new_frame_ready) goto skip_draw;
                 *   new_frame_ready = 0;
                 *   (teammate has already filled raw_samples[] and osc_meas)
                 * ============================================================ */
                /* SIM_STREAM_MODE: 2 = function generator (ตาม simulator, ค่าเริ่มต้น)
                 *                  1 = สตรีมต่อเนื่องเดิม   0 = sine เดิมที่วนลูป
                 * ทีม sampling ต่อข้อมูลจริงตรงนี้ (แทน GenFrame)                    */
                if (!hold_active) {               /* HOLD = freeze ข้อมูลไว้ ไม่ generate ใหม่ */
#if ADC_INPUT_ENABLE
                    SlowSync();                   /* T/DIV >= 50 ms -> SLOW (ring), ไม่งั้น FAST (burst) */
                    { uint32_t pc0 = DWT->CYCCNT;
                    if (slow_active) { AdcAbortPending(); OSC_LOCK(); SlowCaptureFrame(); OSC_UNLOCK(); }
                    else             AdcCaptureFrame();   /* p30: อ่านข้อมูลจริงจาก ADC (PA0) */
                    prof_us[0] = AdcElapsedUs(pc0); }
                    if (!osc_frame_valid) goto skip_draw;
#else
                    osc_capture_tdiv = osc_tdivs_ms[osc_tdiv_i];
                    osc_fs_eff = N_SAMPLES / (osc_capture_tdiv * OSC_H_DIVS / 1000.0f);
                    if      (SIM_STREAM_MODE == 2) GenFrame();
                    else if (SIM_STREAM_MODE == 1) SimSamplingStream();
                    else                           GenerateFakeSineWave();
                    osc_frame_valid = 1;
                    new_frame_ready = 1;
#endif
                }
                if (!osc_frame_valid) goto skip_draw;
                uint32_t pm0 = DWT->CYCCNT;
                OscProcessFrame();      /* calibrate offset (AC) + moving average filter */
                OscMeasureFrame();      /* วัดค่า Vmax/Vmin/Vpp/RMS/Freq + ชนิดคลื่น */
                /* AutoScale ขั้น 2: พอวัดใหม่ที่ time base เร็วสุดแล้ว -> เลือกสเกล */
                if (osc_auto_stage) OscAutoScaleStep();
                OscAutoTimebase();      /* AUTO: สลับ FAST/SLOW + T/DIV ตามความถี่ (ถ้าไม่ได้ปรับเอง) */
                prof_us[1] = AdcElapsedUs(pm0);
                new_frame_ready = 0;
#if ADC_INPUT_ENABLE
                if (!hold_active && !osc_auto_stage && trigger_mode != TRIG_SINGLE && !slow_active) AdcPrestart();   /* the next capture runs by DMA while we draw */
#endif

#if SHOW_FPS
                dbg_px_frame = 0;
                uint32_t t_wf0 = HAL_GetTick();
#endif
                /* ============================================================ */

                /* osc_stable = 1: จัดแนว trigger (ใช้ขอบ RISE/FALL ตามค่า TRIG) -> กราฟนิ่ง
                 * osc_stable = 0: ไม่จัดแนว -> ข้อมูลใหม่เข้าขวา เส้นไถลซ้าย (roll) */
                int trig_offset = 0;
                if (osc_stable) {
#if ADC_INPUT_ENABLE
                    /* p38: จัดแนว trigger ตั้งแต่ตอนจับข้อมูล (AdcCaptureFrame) แล้ว
                     * -> หน้าต่างเริ่มที่จุด trigger จริง ไม่ต้องหาใหม่/ไม่ต้องวนกลับบัฟเฟอร์
                     *    (เดิม FindTrigger หาใหม่แล้วได้คนละจุด ทำให้จุดวนกลับเลื่อนกลางจอ) */
                    trig_offset = 0;
                    trig_hold   = 0;
                    (void)FindTrigger;   /* p39: โหมด ADC จัดแนวตอนจับข้อมูลแล้ว */
#else
                    int t = FindTrigger();
                    if (t >= 0) {
                        trig_hold   = t;
                        trig_offset = t;
                    } else {
                        /* หน้าต่างสั้นกว่าหนึ่งคาบ (ความถี่ต่ำกว่า time base) -> หาขอบไม่ได้
                         * เดิมใช้ค่า trig_hold เก่า => รูปวาดไม่ตรงจุด -> กระโดด/ขาด
                         * ใหม่: วาดแบบไถล (offset 0) ต่อเนื่องแทน */
                        trig_offset = 0;
                    }
#endif
                }
                dbg_trig_off = trig_offset;
                uint32_t pd0 = DWT->CYCCNT;
                uint8_t hold_frame = OscFrameHold(HAL_GetTick());
                dbg_hold_cnt += hold_frame;
                ClearCursors();         /* ล้างเส้น cursor เก่าก่อนวาดเฟรมใหม่ */
                if (!hold_frame) { MapToScreen(trig_offset);
                DrawWaveform(); }
                DrawCursors();          /* HOLD: วาดเส้น C1/C2 ทับหลังวาดรูป */
                DrawSettingBar();       /* แถบ "กำลังตั้งค่าอะไร" ทับหลังวาดรูป */
                prof_us[2] = AdcElapsedUs(pd0);
                OSC_SET(osc_dirty, 0);
                osc_cur_dirty = 0;      /* v85: the full redraw drew the cursors too */

#if SHOW_FPS
                dbg_wf_ms = HAL_GetTick() - t_wf0;
                dbg_px_last = dbg_px_frame;
#endif

#if SHOW_FPS
                fps_frames++;          /* นับเฉพาะเฟรมที่วาดจริง */
                dbg_frames_total++;
#endif

                /* SINGLE mode: lock after first captured frame */
                if (trigger_mode == TRIG_SINGLE) {
                    single_captured = 1;
                    OSC_SET(hold_active    , 1);
                }
            }
            skip_draw: ;
            prof_us[3] = AdcElapsedUs(pf0);
        }

        /* --- Header update (every 300 ms) --------------------------------- */
        if (now - last_header_tick >= HEADER_MS)
        {
            last_header_tick = now;

#if SHOW_FPS
            uint32_t t_h0 = HAL_GetTick();
#endif
            { uint32_t ph0 = DWT->CYCCNT; DrawTopPanel(); prof_us[4] = AdcElapsedUs(ph0); }

#if SHOW_FPS
            dbg_hdr_ms = HAL_GetTick() - t_h0;
#endif

#if SHOW_FPS
            /* FPS จริงทุก ~500 ms (ผูกกับรอบ refresh header) */
            if (now - last_fps_tick >= 500)
            {
                DrawFps((float)fps_frames * 1000.0f / (float)(now - last_fps_tick));
                dbg_fps_x10   = (uint32_t)(fps_frames * 10000u / (now - last_fps_tick));
                dbg_uptime_ms = now;
                fps_frames    = 0;

                last_fps_tick = now;
            }
#endif
        }

        /* USER CODE END 3 */
    }
}

/* ==========================================================================
 * main()
 * ========================================================================== */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    SCB_EnableICache();
    SCB_EnableDCache();

    HAL_Init();
    SystemClock_Config();

    /* Peripheral init — identical to Core/Src/main.c */
    MX_GPIO_Init();
    MX_RNG_Init();
    MX_SPI5_Init();
    MX_TIM1_Init();
    MX_USART3_UART_Init();
    MX_I2C1_Init();
#if ADC_INPUT_ENABLE
    AdcInputInit();   /* p30: ADC1+PA0+DMA สำหรับอินพุตจริง */
#endif

    /* USER CODE BEGIN 2 */

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
    /* v66b: UartPanel() moved to the start of vMainTask (it reads osc_meas under the mutex; no Take before the scheduler) */

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

    /* v66b: initial DrawGrid/DrawCursor/DrawTopPanel moved to the start of vMainTask (DrawTopPanel locks the mutex) */

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    xOscMutex = xSemaphoreCreateMutex();
    if (xOscMutex == NULL) Error_Handler();
    xTaskCreate(vMainTask, "Main", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();   /* never returns */
    for (;;) { }
    /* USER CODE END WHILE */
}

/* ==========================================================================
 * SystemClock_Config — identical to Core/Src/main.c
 * 8 MHz HSE -> PLL: M=8, N=192, P=2 -> SYSCLK = 96 MHz
 * ========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 192;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 9;
    RCC_OscInitStruct.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1) {}
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    (void)file; (void)line;
    /* USER CODE END 6 */
}
#endif


