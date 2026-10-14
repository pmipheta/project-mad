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
#define OSC_NT             16       /* จำนวน T/DIV ทั้งหมด (12 FAST + 4 SLOW) */
#define OSC_TDIV_DEF       8        /* index ของ 1.0 ms/div = ค่าเริ่มต้น */
#define OSC_NT_FAST        12       /* T/DIV 0..11 = FAST (burst DMA); 12..15 = SLOW (ring, SysTick) */
#define ADC_SAMPLETIME_FAST ADC_SAMPLETIME_3CYCLES /* 12 MHz / 15 = 800 kSa/s */
#define ADC_PRE_POINTS     120      /* จำนวนจุด "ก่อน trigger" (ครึ่งจอซ้าย) — p38 */

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

/* --- ADC reference --------------------------------------------------------- */
#define ADC_MID         2048         /* 12-bit midpoint (= 0 V after bias)     */
#define ADC_FULL        4095         /* 12-bit full scale                       */
#define ADC_VREF        3.3f         /* reference voltage (V)                   */

/* --- Sample buffer --------------------------------------------------------- */
#define N_SAMPLES       240

/* --- Frame timing ---------------------------------------------------------- */
#define FRAME_MS        33           /* ~30 FPS                                 */
#define HEADER_MS       300          /* header refresh interval (ms)            */
#define BTN_DEBOUNCE_MS 200          /* button debounce window                  */
#define BTN_LONG_MS     2000         /* long-press threshold for auto scale     */

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
} Button;

/* USER CODE END PTD */

/* USER CODE BEGIN PV */

/* --- Sample buffer (teammate writes here) ---------------------------------- */
volatile int16_t raw_samples[N_SAMPLES];  /* ADC counts 0-4095                */
volatile uint8_t new_frame_ready = 0;     /* teammate sets 1 when buffer ready */

/* --- Measurement results (teammate writes here) ---------------------------- */
volatile OscMeasure osc_meas = {0};

/* --- Display double-buffer ------------------------------------------------- */
static int16_t old_y[N_SAMPLES];
static int16_t new_y[N_SAMPLES];

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
#define OSC_V_FILL_DIVS 5.2f        /* auto scale: ให้ Vpp กินกี่ช่องแนวตั้ง (grid มี 5 ช่อง, เผื่อเกินนิดหน่อย) */
#define OSC_FIT_CYCLES  2.5f        /* auto scale: อยากเห็นกี่คาบบนจอ (10 ช่อง) */
#define OSC_SET_COUNT   4           /* FILTER / TRIG / V/DIV / T/DIV            */
#define OSC_HOLD_COUNT  3           /* C1 / C2 / V/DIV (ตอน HOLD)               */
#define UART_RX_SIZE    96
#define UART_IDLE_MS    250         /* หยุดพิมพ์เกินเท่านี้ = ถือว่าเป็นคำสั่งแบบเดี่ยว (ไม่ต้อง Enter) */
#define UART_DROP_MS    4000        /* บรรทัดค้าง (ไม่มี \r) เกินเท่านี้ = ทิ้ง */
#define OSC_PUSH        8           /* sample ใหม่ต่อเฟรม (คุมความเร็วไถล) */

static const char  *gen_wave_names[]  = {"SINE", "SQUARE", "TRI", "SAW"};
static const float  osc_vdivs[]       = {0.1f, 0.2f, 0.5f, 1.0f, 2.0f};
static const float  osc_tdivs_ms[]    = {0.005f, 0.01f, 0.015f, 0.02f, 0.05f, 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 6.0f,
                                         50.0f, 100.0f, 200.0f, 500.0f};   /* 0.015 ms/div = raw ADC samples (1.6 MSa/s); 50..500 ms/div = SLOW mode */
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
static int      osc_vdiv_i    = 3;         /* -> 1.0 V/div                     */
static int      osc_tdiv_i    = OSC_TDIV_DEF;  /* -> 1.0 ms/div                */
static int      osc_fwin      = 5;         /* moving average window (1..21 คี่) */
static int      osc_trig_fall = 0;         /* 0 = rising, 1 = falling          */
static int      osc_sidx      = 0;         /* setting ที่กำลังปรับ              */
static int      osc_c1        = 40, osc_c2 = 160;
static uint8_t  osc_calib     = 1;         /* AC coupling (calibrate offset)   */
static uint8_t  osc_dirty     = 1;         /* สั่งวาดใหม่แม้อยู่โหมด HOLD         */

/* โหมดจอ (ดูโหมดที่ใช้อยู่ได้ที่บรรทัด Trig: ในหน้ารายงาน / บนจอ LCD) */
static uint8_t  osc_stable    = 1;   /* 1 = นิ่ง (trigger) | 0 = ไถล (roll)  */
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
static uint8_t  osc_auto_tb = 1;           /* 1 = firmware picks FAST/SLOW + T/DIV itself; 0 = manual (set by T/DIV keys, re-armed by 'z') */
static float    osc_raw_freq = 0.0f, osc_raw_vpp = 0.0f;
static float    osc_capture_tdiv = 1.0f;
static uint32_t adc_measure_n = 3200u; /* adaptive, independent of display */
static uint8_t  adc_meas_lvl = 0;      /* position in adc_meas_seq[] used for the frequency measurement capture */
static const uint8_t adc_meas_seq[3] = { 1u, 0u, 4u };   /* 800 kSa/s (original), 1.6 MSa/s, 4.8 MSa/s (3 ADCs interleaved) */
static uint8_t  adc_too_fast = 0;      /* last measurement saw a signal above the usable range of its rate */
static uint8_t  adc_fast_content = 0;  /* the 800 kSa/s-or-faster capture is full of sample-to-sample change: signal is fast, not slow */
static uint8_t  adc_over_range = 0;    /* frequency above the readout limit (shown as >2.1MHz) */
static float    adc_hold_f = 0.0f;     /* last good reading, held for a short time so one failed frame does not blank the display */
static uint32_t adc_hold_t = 0;
static float    adc_rough_ratio = 0.0f; /* mean |sample-to-sample change| / Vpp of the last measurement capture */
static uint32_t adc_over_t = 0;        /* tick of the last 'above readout limit' frame (display keeps >2.1MHz for 1.5 s: no blinking) */
static int      osc_plot_offset = 0;

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
static void OscProcessFrame(void)
{
    const float k = ADC_VREF / (float)ADC_FULL;
    float       s = 0.0f;
    int         win, half;

    /* osc_fs_eff belongs to the captured frame, including HOLD redraws. */

    /* Calibration: หา DC offset จริงในบัฟเฟอร์แล้วทำ AC coupling */
    if (osc_calib) {
        for (int i = 0; i < N_SAMPLES; i++) s += (float)raw_samples[i];
        osc_offset_ct = s / (float)N_SAMPLES;
    } else {
        osc_offset_ct = (float)ADC_MID;
    }
    osc_offset_v = (osc_offset_ct - (float)ADC_MID) * k;

    /* Moving average filter (window 1..21, คี่) */
    win  = (osc_fwin < 1) ? 1 : osc_fwin;
    if ((win % 2) == 0) win++;
    half = win / 2;

    for (int i = 0; i < N_SAMPLES; i++) {
        if (win <= 1) {
            osc_v[i] = ((float)raw_samples[i] - osc_offset_ct) * k;
        } else {
            float acc = 0.0f;
            for (int j = -half; j <= half; j++) {
                int m = i + j;
                if (m < 0) m = 0;
                if (m >= N_SAMPLES) m = N_SAMPLES - 1;
                acc += (float)raw_samples[m];
            }
            osc_v[i] = (acc / (float)win - osc_offset_ct) * k;
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
    else if (!strcmp(name, "V/DIV"))  snprintf(out, n, "VDIV:%.1f", (double)osc_vdivs[osc_vdiv_i]);
    else if (!strcmp(name, "T/DIV"))  snprintf(out, n, "TDIV:%.1f", (double)osc_tdivs_ms[osc_tdiv_i]);
    else if (!strcmp(name, "C1"))     snprintf(out, n, "C1:%d", osc_c1);
    else if (!strcmp(name, "C2"))     snprintf(out, n, "C2:%d", osc_c2);
    else                              snprintf(out, n, "--");
}

static void OscSetIndex(int idx)
{
    osc_sidx = idx % OscSetCount();
    if (osc_sidx < 0) osc_sidx = 0;
}

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
        osc_tdiv_i = OscClampI(osc_tdiv_i + dir, 0, (int)(sizeof(osc_tdivs_ms) / sizeof(float)) - 1);
        /* New timebase takes effect only on the next capture. */
        osc_auto_tb = 0;            /* manual choice wins until 'z' (AutoScale) re-arms AUTO */
    } else if (!strcmp(name, "C1")) {
        osc_c1 = OscClampI(osc_c1 + dir * 5, 0, N_SAMPLES - 1);
    } else if (!strcmp(name, "C2")) {
        osc_c2 = OscClampI(osc_c2 + dir * 5, 0, N_SAMPLES - 1);
    }
    osc_dirty = 1;                  /* ให้วาดใหม่แม้อยู่ใน HOLD */
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
    osc_auto_tb = 1;                 /* z / long SET = back to AUTO time base */
    osc_auto_tries = 0;
    osc_dirty = 1;
    UartPrint("\r\n AutoScale...\r\n");
}

static void OscAutoScaleStep(void)
{
    if (!osc_auto_stage || hold_active || !osc_frame_valid || !new_frame_ready) return;
    float f = osc_meas.freq;
    if (!(f > 0.0f)) {       /* adaptive measurement still acquiring periods */
        /* FAST capture only sees >= 40 Hz: after ~15 frames look at the SLOW 200 ms/div window (2 s);
         * give up (no periodic signal) after the SLOW window had time to fill. */
        if (osc_auto_tries < 250u) osc_auto_tries++;
        if (!slow_active && osc_auto_tries == 15u) { osc_tdiv_i = OSC_NT_FAST + 2; osc_dirty = 1; }
        else if (slow_active && osc_auto_tries > 100u) { osc_auto_stage = 0; }
        return;
    }
    float vpp = ADC_INPUT_ENABLE ? osc_raw_vpp : osc_meas.vpp;
    int vi = (int)(sizeof(osc_vdivs) / sizeof(osc_vdivs[0])) - 1;
    for (int i = 0; i <= vi; i++) {
        if (osc_vdivs[i] * OSC_V_FILL_DIVS >= vpp) { vi = i; break; }
    }
    int ti = 0;
    float best = 1.0e30f;
    int k0 = 0, k1 = OSC_NT_FAST;                         /* >= 45 Hz: FAST bases (as before) */
    if (f < 45.0f) { k0 = OSC_NT_FAST; k1 = OSC_NT; ti = OSC_NT_FAST + 2; }   /* slow signal: SLOW bases */
    for (int i = k0; i < k1; i++) {
        float cyc = f * osc_tdivs_ms[i] * OSC_H_DIVS / 1000.0f;
        float d = fabsf(cyc - OSC_FIT_CYCLES);
        if (k0 && cyc < 2.0f) d += 10.0f;                 /* SLOW: need >= 2 periods to measure */
        if (d < best) { best = d; ti = i; }
    }
    osc_vdiv_i = vi;
    volts_per_div = osc_vdivs[vi];
    osc_tdiv_i = ti; /* captured time metadata changes only on acquisition */
    dbg_auto_fsel = f;
    osc_auto_stage = 0;
    osc_dirty = 1;
}


/* AUTO_TB_BEGIN =============================================================
 * AUTO time base: เลือก FAST/SLOW และ T/DIV เองตามความถี่ที่วัดได้ (ปิดเมื่อผู้ใช้ปรับ T/DIV เอง; 'z' เปิดใหม่)
 *   FAST (burst DMA)  วัดความถี่ได้ >= ~40 Hz       SLOW (SysTick ring) วัดได้ ~0.2-60 Hz
 *   กฎ:  FAST + f<45 Hz  -> SLOW (เลือก T/DIV ให้เห็น >= 3 คาบ และ f <= Fs/8)
 *        SLOW + f>55 Hz  -> FAST (เลือก T/DIV ให้เห็น ~2.5 คาบ)      (45-55 Hz = hysteresis กันสลับไปมา)
 *        มีสัญญาณ (>=50 mVpp) แต่อ่านความถี่ไม่ได้นาน 2 s -> ไล่ลอง SLOW 200 -> 500 -> 100 -> 50 ms/div
 *        แล้วกลับ FAST และหยุดลอง 120 s (กันจอกระโดดเมื่อเป็น noise ล้วน; สัญญาณหายแล้วกลับมา = เริ่มลองใหม่ทันที)
 *   ไม่มีสัญญาณ = ไม่เปลี่ยนอะไร  |  HOLD / AutoScale กำลังทำงาน = ไม่ทำงาน
 * ========================================================================== */
#define ATB_FAST_TO_SLOW_HZ  45.0f
#define ATB_SLOW_TO_FAST_HZ  55.0f
#define ATB_BAD_MS           1200u
#define ATB_BACKOFF_MS       120000u

static uint32_t atb_t_switch  = 0;      /* tick ที่สลับครั้งล่าสุด */
static uint32_t atb_t_bad     = 0;      /* tick ที่เริ่มเจอ "มีสัญญาณแต่อ่าน f ไม่ได้" (0 = ไม่เจอ) */
static uint32_t atb_t_backoff = 0;      /* ห้ามเริ่มลองใหม่จนถึง tick นี้ */
static uint8_t  atb_try       = 0;      /* ขั้นที่กำลังลองในลำดับ SLOW */
static uint8_t  atb_last_slow = 0;      /* SLOW base (index) that last gave a reading: tried first next time (0 = none yet) */
static uint8_t  atb_bo_fast   = 0;      /* 1 = the 120 s pause was set because SLOW saw a FAST signal (lifted as soon as the signal is no longer fast) */
volatile uint8_t dbg_atb_switches = 0;  /* จำนวนครั้งที่สลับ (ดูผ่าน SWD) */

static int AtbPickFast(float f)         /* FAST base ที่ได้ ~2.5 คาบ */
{
    int ti = 0; float best = 1.0e30f;
    for (int k = 0; k < OSC_NT_FAST; k++) {
        float d = fabsf(f * osc_tdivs_ms[k] * (float)OSC_H_DIVS / 1000.0f - OSC_FIT_CYCLES);
        if (d < best) { best = d; ti = k; }
    }
    return ti;
}

static int AtbPickSlow(float f)         /* SLOW base: >= 3 คาบ, f <= Fs/8, ใกล้ 4 คาบที่สุด */
{
    int ti = OSC_NT - 1; float best = 1.0e30f;
    for (int k = OSC_NT_FAST; k < OSC_NT; k++) {
        float cyc = f * osc_tdivs_ms[k] * (float)OSC_H_DIVS / 1000.0f;
        float fs  = 24000.0f / osc_tdivs_ms[k];                /* = 240 / (T/DIV x 10) */
        float d   = fabsf(cyc - 4.0f) + ((cyc >= 3.0f && f <= fs / 8.0f) ? 0.0f : 100.0f);
        if (d < best) { best = d; ti = k; }
    }
    return ti;
}

static void AtbSwitch(int ti)
{
    if (ti == osc_tdiv_i) return;
    osc_tdiv_i   = ti;
    atb_t_switch = HAL_GetTick();
    atb_t_bad    = 0;
    osc_dirty    = 1;
    dbg_atb_switches++;
}

static void OscAutoTimebase(void)       /* เรียกทุกเฟรมหลังวัดค่า */
{
    static const uint8_t base_order[4] = { OSC_NT_FAST + 1, OSC_NT_FAST + 0, OSC_NT_FAST + 2, OSC_NT_FAST + 3 };   /* 100, 50, 200, 500 ms/div: 2-60 Hz (the range just below FAST) first */
    uint8_t order[4];
    uint32_t now, settle;
    float f, cyc;

    if (!osc_auto_tb || hold_active || osc_auto_stage || !osc_frame_valid) return;

    now    = HAL_GetTick();
    settle = (slow_active ? (uint32_t)(osc_tdivs_ms[osc_tdiv_i] * (float)OSC_H_DIVS) : 0u) + 1500u;
    if (slow_active && atb_try == 1u && atb_last_slow) settle = 2500u;   /* first try = the base that worked last time: judge it after 2.5 s, not a full window */
    {   /* remembered base first, then the usual order without repeating it */
        uint8_t n = 0;
        if (atb_last_slow) order[n++] = atb_last_slow;
        for (uint8_t i = 0; i < 4u && n < 4u; i++) if (base_order[i] != atb_last_slow) order[n++] = base_order[i];
    }
    if (!(slow_active && slow_fast_flag && (now - atb_t_switch) >= 600u) && (now - atb_t_switch) < settle) return;   /* รอ ring เต็ม / ตัววัดวอร์มอัพ */

    if (slow_active && slow_fast_flag && (now - atb_t_switch) >= 600u) {   /* SLOW เห็นสัญญาณเร็วเกินไป: กลับ FAST เลย (รอแค่ 0.6 s ไม่ต้องรอจอเต็ม) */
        atb_try = 0; atb_t_backoff = now + ATB_BACKOFF_MS; atb_bo_fast = 1; AtbSwitch(OSC_TDIV_DEF);
        return;
    }

    f = osc_meas.freq;
    if (f > 0.0f) {                                         /* อ่านความถี่ได้ */
        atb_t_bad = 0; atb_try = 0;
        if (slow_active) atb_last_slow = (uint8_t)osc_tdiv_i;
        cyc = f * osc_tdivs_ms[osc_tdiv_i] * (float)OSC_H_DIVS / 1000.0f;
        if (!slow_active) {
            if (f < ATB_FAST_TO_SLOW_HZ)      AtbSwitch(AtbPickSlow(f));
            else if (cyc < 1.2f || cyc > 12.0f) AtbSwitch(AtbPickFast(f));
        } else {
            if (f > ATB_SLOW_TO_FAST_HZ)      AtbSwitch(AtbPickFast(f));
            else if (cyc < 3.0f || cyc > 30.0f || f > (24000.0f / osc_tdivs_ms[osc_tdiv_i]) / 9.0f)
                AtbSwitch(AtbPickSlow(f));
        }
        return;
    }

    if (osc_raw_vpp < 0.05f) { atb_t_bad = 0; atb_try = 0; atb_t_backoff = 0; atb_bo_fast = 0; return; }   /* ไม่มีสัญญาณ: ไม่ทำอะไร + เปิดให้ลองใหม่ได้ทันทีเมื่อสัญญาณกลับมา */

    /* มีสัญญาณ แต่อ่านความถี่ไม่ได้ */
    if (!slow_active && adc_fast_content) { atb_t_bad = 0; return; }      /* สัญญาณเร็ว (เกินช่วงอ่าน): อยู่ FAST ไม่ต้องไล่ลอง SLOW */
    if (!slow_active && atb_bo_fast && (now - adc_over_t) > 1500u) { atb_t_backoff = 0; atb_bo_fast = 0; }   /* signal is slow now: search SLOW again after the normal 2 s */
    if (!atb_t_bad) { atb_t_bad = now; return; }
    if (!slow_active) {
        if (now < atb_t_backoff) return;
        if ((now - atb_t_bad) < ATB_BAD_MS) return;
        atb_try = 1; AtbSwitch(order[0]);                   /* เริ่มลอง SLOW 200 ms/div */
    } else {
        if (atb_try < 4u) AtbSwitch(order[atb_try++]);      /* ลอง base ถัดไป (ผ่านช่วง settle แล้ว) */
        else { atb_try = 0; atb_t_backoff = now + ATB_BACKOFF_MS; atb_bo_fast = 0; AtbSwitch(OSC_TDIV_DEF); }  /* หมดลำดับ: กลับ FAST 1 ms/div */
    }
}
/* AUTO_TB_END ============================================================== */

static void UartPanel(void)
{
    char b[96];
    UartPrint("\r\n== OSCILLOSCOPE =============================");
    OscSetValueText(b, sizeof(b));
    UartPrint("\r\n Setting: "); UartPrint(b);
    for (int i = 0; i < OscSetCount(); i++) {
        int save = osc_sidx;
        osc_sidx = i;
        OscSetValueText(b, sizeof(b));
        osc_sidx = save;
        UartPrint((i == osc_sidx) ? "\r\n  > " : "\r\n    ");
        UartPrint(b);
    }
    snprintf(b, sizeof(b), "\r\n HOLD:%s  CAL:%s  Trig:%s-%s  Det:%s",
             hold_active ? "ON" : "OFF", osc_calib ? "AC" : "DC",
             osc_stable ? "AUTO" : "ROLL", osc_trig_fall ? "FALL" : "RISE", osc_det);
    UartPrint(b);
    UartPrint(osc_auto_tb ? "\r\n TimeBase: AUTO (change T/DIV = manual, z = AUTO again)" : "\r\n TimeBase: MANUAL (z = AUTO)");

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
    UartNum("\r\n Vmax:", osc_meas.vmax, " V");  UartNum("   Vmin:", osc_meas.vmin, " V");
    UartNum("\r\n Vpp: ", osc_meas.vpp,  " V");  UartNum("   RMS: ", osc_meas.vrms, " V");
    UartNum("\r\n DC:  ", osc_meas.dc_offset, " V");
    snprintf(b, sizeof(b), "   Freq: %d Hz", (int)(osc_meas.freq + 0.5f));   /* p44: ปัด */
    UartPrint(b);
    if (hold_active) {
        UartNum("\r\n U1:", osc_v[(osc_c1 + osc_plot_offset) % N_SAMPLES], " V");  UartNum("   U2:", osc_v[(osc_c2 + osc_plot_offset) % N_SAMPLES], " V");
        UartNum("   dV:", osc_v[(osc_c2 + osc_plot_offset) % N_SAMPLES] - osc_v[(osc_c1 + osc_plot_offset) % N_SAMPLES], " V");
        snprintf(b, sizeof(b), "   dt: %d us",
                 (int)(((float)(osc_c2 - osc_c1) / osc_fs_eff) * 1e6f));
        UartPrint(b);
    }
    UartNum("\r\n Calib offset:", osc_offset_ct, " cts");
    UartNum("   Dev:", osc_offset_ct - (float)ADC_MID, " cts");
    UartPrint("\r\n----------------------------------------------");
    UartPrint("\r\n keys: s=SET u=UP d=DOWN h=HOLD g=gen m=auto/roll z=autoscale ?=status");
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
        case 'u': if (has) OscSetIndex(atoi(arg)); else OscAdjust(+1); break;
        case 'd': OscAdjust(-1); break;                                 /* DOWN  */
        case 'h': case 'r':                                             /* HOLD  */
            hold_active = !hold_active;
            osc_sidx    = 0;
            osc_dirty   = 1;
            break;
        case 'g':                                                       /* gen ใหม่ */
            gen_ph = fmodf(gen_ph + 1.7f, 6.28318531f);                  /* = reseed */
            osc_dirty = 1;
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
            osc_dirty = 1;
            break;
        case 'f': gen_freq  = OscClampF(has ? UartArgF(arg, gen_freq) : gen_freq + 100.0f, 100.0f, 5000.0f); osc_dirty = 1; break;
        case 'a': gen_amp   = OscClampF(has ? UartArgF(arg, gen_amp)  : gen_amp  + 0.05f, 0.10f, 1.50f); osc_dirty = 1; break;
        case 'o': gen_dc    = OscClampF(has ? UartArgF(arg, gen_dc)   : gen_dc   + 0.05f, 0.50f, 2.50f); osc_dirty = 1; break;
        case 'n': gen_noise = OscClampF(has ? UartArgF(arg, gen_noise): gen_noise+ 0.005f, 0.0f, 0.30f); osc_dirty = 1; break;
        case 'v': OscSetIndex(2); OscAdjust((arg && *arg == '-') ? -1 : +1); break;
        case 't': OscSetIndex(3); OscAdjust((arg && *arg == '-') ? -1 : +1); break;
        case 'c':
            if (tok[1] == '1' || tok[1] == '2') {
                int ci = (tok[1] == '1') ? 1 : 2;
                osc_sidx = hold_active ? (ci == 1 ? 0 : 1) : osc_sidx;
                if (has) {
                    int v = OscClampI(atoi(arg), 0, N_SAMPLES - 1);
                    if (ci == 1) osc_c1 = v; else osc_c2 = v;
                }
            } else if (tok[1] == 'a') {                                 /* cal 0/1 */
                osc_calib = (uint8_t)(has ? atoi(arg) : !osc_calib);
            }
            osc_dirty = 1;
            break;
        case 'm': {                                                     /* โหมดจอ */
            const char *a = has ? arg : tok + 1;       /* "m a" | "ma" | "m" */
            if (*a) {
                char a0 = (a[0] >= 'A' && a[0] <= 'Z') ? (char)(a[0] + 32) : a[0];
                if      (a0 == 'a') osc_stable = 1;                  /* auto = นิ่ง  */
                else if (a0 == 'r') osc_stable = 0;                  /* roll = ไถล   */
                else                osc_stable = (uint8_t)(atoi(a) ? 1 : 0);
            } else {
                osc_stable = (uint8_t)!osc_stable;
            }
            osc_dirty = 1;
            break;
        }
        case 'z': case 'Z': OscAutoScale(); break;                      /* auto scale */
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
        if (uart_len == 0 && strchr("sdhg?z", c)) {
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

    osc_meas.vmax = vmax;
    osc_meas.vmin = vmin;
    osc_meas.vpp  = vmax - vmin;
    osc_meas.vrms = sqrtf(s2 / (float)N_SAMPLES);
    osc_meas.dc_offset = s1 / (float)N_SAMPLES;
    if (ncross >= 2) {                                     /* วัดคาบจากจุดตัดศูนย์ */
        float span = cross[ncross - 1] - cross[0];
        if (span > 0.0f) osc_meas.freq = (float)(ncross - 1) / span;
    }
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
    ((OscMeasure *)&osc_meas)->vmax     =  peak_v;
    ((OscMeasure *)&osc_meas)->vmin     = -peak_v;
    ((OscMeasure *)&osc_meas)->vpp      =  peak_v * 2.0f;
    ((OscMeasure *)&osc_meas)->vrms     =  peak_v * 0.7071f;
    ((OscMeasure *)&osc_meas)->freq     =  1000.0f;
    ((OscMeasure *)&osc_meas)->dc_offset = 0.0f;
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
    for (int i = 0; i < N_SAMPLES; i++) {
        int src = (trig_offset + i);
        if (src >= N_SAMPLES) src -= N_SAMPLES;  /* wrap around buffer */

        /* Convert count -> voltage -> pixel offset */
        float v = osc_v[src];
        int y = CENTER_Y - (int)(v / volts_per_div * (float)GRID_Y_STEP);

        /* Clamp to waveform area only (do not draw over header) */
        if (y < WAVEFORM_TOP) y = WAVEFORM_TOP;
        if (y >= SCREEN_H)    y = SCREEN_H - 1;
        new_y[i] = (int16_t)y;
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
}

/* --------------------------------------------------------------------------
 * DrawLineBresenham
 * Rasterise a line. When erasing (color == BLACK):
 *   - Restore DARKGREY at grid dot positions      (Fix #2)
 *   - Restore DARKCYAN at cursor column           (cursor preservation)
 * -------------------------------------------------------------------------- */
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
            } else if ((y0 % GRID_Y_STEP == GRID_Y_OFFSET) && (x0 % GRID_X_DOT == 0)) {
                draw_color = DARKGREY;
            }
        }

        ILI9341_Draw_Pixel((uint16_t)x0, (uint16_t)y0, draw_color);

#if SHOW_FPS
        dbg_px_frame++;
#endif

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
static int OscCursorX(int idx)
{
    if (idx < 0) idx = 0;
    if (idx >= N_SAMPLES) idx = N_SAMPLES - 1;
    return (idx * SCREEN_W) / N_SAMPLES;
}

static uint8_t cursors_on = 0;      /* 1 = มีเส้น cursor ค้างอยู่บนจอ */
static int     cur_x1 = -1, cur_x2 = -1;

/* ล้างเส้น cursor เก่า (คืนจุด grid กลับด้วย) ก่อนวาดเฟรมใหม่ */
static void ClearCursors(void)
{
    if (!cursors_on) return;
    for (int k = 0; k < 2; k++) {
        int x = k ? cur_x2 : cur_x1;
        if (x < 0) continue;
        for (int y = WAVEFORM_TOP; y < SCREEN_H; y++) {
            uint16_t col = (((y % GRID_Y_STEP) == GRID_Y_OFFSET) && ((x % GRID_X_DOT) == 0))
                           ? DARKGREY : BLACK;
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
static void DrawWaveform(void)
{
    for (int i = 0; i < N_SAMPLES - 1; i++) {
        int x0 = (i       * SCREEN_W) / N_SAMPLES;  /* Fix #1 */
        int x1 = ((i + 1) * SCREEN_W) / N_SAMPLES;

        DrawLineBresenham(x0, old_y[i],     x1, old_y[i + 1], BLACK);
        DrawLineBresenham(x0, new_y[i],     x1, new_y[i + 1], YELLOW);
    }
    memcpy(old_y, new_y, sizeof(new_y));
}

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
static void DrawSettingBar(void)
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
    else if (!strcmp(name, "C1"))     snprintf(buf, sizeof(buf), "SET C1=%d  UP/DOWN +-5", osc_c1);
    else if (!strcmp(name, "C2"))     snprintf(buf, sizeof(buf), "SET C2=%d  UP/DOWN +-5", osc_c2);
    else                              snprintf(buf, sizeof(buf), "SET --");

    ILI9341_Draw_Text(buf, 2, SCREEN_H - 12, BLACK, 1, CYAN);
    snprintf((char *)dbg_lcd_bar, sizeof(dbg_lcd_bar), "%s", buf);
}

#if ADC_INPUT_ENABLE
extern volatile uint16_t dbg_adc_min, dbg_adc_max;
#endif
static void DrawTopPanel(void)
{
    char buf[20];
    /* Clear header area */
    ILI9341_Draw_Filled_Rectangle_Coord(0, 0, SCREEN_W - 1, HEADER_H - 2, BLACK);

    /* ── Row 1 ──────────────────────────────────────────────────────────── */

    /* ตัวที่กำลังปรับ (SET) ถูกไฮไลต์ => มองจอรู้เลยว่า UP/DOWN จะแก้ค่าอะไร
     * ไม่ต้องไปดูที่หน้าจอ serial เท่านั้น                                        */
    const char *selname = OscSetName();
    int sel_vdiv = (strcmp(selname, "V/DIV") == 0);
    int sel_tdiv = (strcmp(selname, "T/DIV") == 0);

    /* V/div — x=2 (MAGENTA = กำลังปรับตัวนี้อยู่) */
    snprintf(buf, sizeof(buf), "V:%.1fV/d", (double)volts_per_div);
    ILI9341_Draw_Text(buf, 2, 1, sel_vdiv ? MAGENTA : YELLOW, 1, BLACK);

    /* T/div label — x=72 */
    snprintf(buf, sizeof(buf), "T:%.1fms/d", (double)osc_capture_tdiv);
    ILI9341_Draw_Text(buf, 72, 1, sel_tdiv ? MAGENTA : WHITE, 1, BLACK);

    /* Setting ที่กำลังปรับ (SET เลือก, UP/DOWN เปลี่ยนค่า) — x=150
     * ตัวดำบนแถบส้ม = "ช่องที่ถูกเลือกอยู่" */
    OscSetValueText(buf, sizeof(buf));
    ILI9341_Draw_Text(buf, 150, 1, BLACK, 1, ORANGE);
    snprintf((char *)dbg_lcd_set, sizeof(dbg_lcd_set), "%s", buf);

    /* Run / Hold status — x=226 */
    if (!osc_frame_valid) {
        ILI9341_Draw_Text("ADC ERR", 226, 1, RED, 1, BLACK);
    } else if (hold_active) {
        ILI9341_Draw_Text("HOLD", 226, 1, RED, 1, BLACK);
    } else if (trigger_mode == TRIG_SINGLE && !single_captured) {
        ILI9341_Draw_Text("WAIT", 226, 1, ORANGE, 1, BLACK);
    } else {
        ILI9341_Draw_Text("RUN ", 226, 1, GREEN, 1, BLACK);
    }

    /* ── Row 2 ──────────────────────────────────────────────────────────── */

    /* Vpp — x=2 */
    if (hold_active) {
        /* ── HOLD: ค่าที่ cursor C1/C2 (เหมือน simulator) ────────────────── */
        snprintf(buf, sizeof(buf), "U1=%.2fV", (double)osc_v[(osc_c1 + osc_plot_offset) % N_SAMPLES]);
        ILI9341_Draw_Text(buf, 2, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "U2=%.2fV", (double)osc_v[(osc_c2 + osc_plot_offset) % N_SAMPLES]);
        ILI9341_Draw_Text(buf, 72, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "dV=%.2fV", (double)(osc_v[(osc_c2 + osc_plot_offset) % N_SAMPLES] - osc_v[(osc_c1 + osc_plot_offset) % N_SAMPLES]));
        ILI9341_Draw_Text(buf, 148, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "dt=%.2fms",
                 (double)((float)(osc_c2 - osc_c1) / osc_fs_eff * 1000.0f));
        ILI9341_Draw_Text(buf, 2, 20, YELLOW, 1, BLACK);
    } else {
    snprintf(buf, sizeof(buf), "Vp:%.2fV", (double)osc_meas.vpp);
    ILI9341_Draw_Text(buf, 2, 10, CYAN, 1, BLACK);

    /* Frequency — x=68 */
    {
        char fb[16];
        if (osc_meas.freq >= 1000000.0f)  snprintf(fb, sizeof(fb), "%.3fMHz", (double)(osc_meas.freq / 1000000.0f));
        else if (osc_meas.freq >= 1000.0f) snprintf(fb, sizeof(fb), "%.1fkHz", (double)(osc_meas.freq / 1000.0f));
        else if (osc_meas.freq >= 10.0f)  snprintf(fb, sizeof(fb), "%.0fHz",  (double)osc_meas.freq);
        else if (osc_meas.freq > 0.0f)    snprintf(fb, sizeof(fb), "%.2fHz",  (double)osc_meas.freq);   /* <10 Hz: 2 decimals */
        else if (adc_over_t && (HAL_GetTick() - adc_over_t) < 1500u && !slow_active) snprintf(fb, sizeof(fb), ">2.1MHz");               /* above the readout limit */
        else                              snprintf(fb, sizeof(fb), "---");                            /* unknown, not "0 Hz" */
        snprintf(buf, sizeof(buf), "F:%-8s", fb);
    }
    ILI9341_Draw_Text(buf,72, 10, GREEN, 1, BLACK);

    /* RMS — x=245 (245 < 255, safe for uint8_t) */
    snprintf(buf, sizeof(buf), "Rs:%.2f", (double)osc_meas.vrms);
    ILI9341_Draw_Text(buf, 148, 10, LIGHTGREY, 1, BLACK);

    /* Vmax — x=130 */
    snprintf(buf, sizeof(buf), "Mx:%.2fV", (double)osc_meas.vmax);
    ILI9341_Draw_Text(buf, 2, 20, LIGHTGREY, 1, BLACK);

    /* Vmin — x=188 */
    snprintf(buf, sizeof(buf), "Mn:%.2fV", (double)osc_meas.vmin);
    ILI9341_Draw_Text(buf, 72, 20, LIGHTGREY, 1, BLACK);

    /* โหมด trigger — AUTO:R / AUTO:F = จัดแนวให้นิ่ง, ROLL = ไถล */
    if (slow_active)
        snprintf(buf, sizeof(buf), "SLOW  ");
    else if (osc_stable)
        snprintf(buf, sizeof(buf), "AUTO:%s", osc_trig_fall ? "F" : "R");
    else
        snprintf(buf, sizeof(buf), "ROLL");
    ILI9341_Draw_Text(buf, 148, 20, CYAN, 1, BLACK);
#if ADC_INPUT_ENABLE
    /* CLIP: ADC ชน 0 หรือ 4095 => สัญญาณเกินช่วง 0-3.3 V (ยอดคลื่นแบนจริง ไม่ใช่บั๊กจอ) */
    ILI9341_Draw_Text((osc_frame_valid && (dbg_adc_min <= 8u || dbg_adc_max >= 4087u)) ? "CLIP" : "    ",
                      200, 20, RED, 1, BLACK);
#endif
    }

//    /* RMS — x=245 (245 < 255, safe for uint8_t) */
//    snprintf(buf, sizeof(buf), "Rs:%.2f", (double)osc_meas.vrms);
//    ILI9341_Draw_Text(buf, 2, 20, LIGHTGREY, 1, BLACK);

    /* Separator line */
    ILI9341_Draw_Horizontal_Line(0, HEADER_H - 1, SCREEN_W, DARKGREY);
}

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

/* --------------------------------------------------------------------------
 * HandleButtons
 * Call this every loop iteration (before frame logic).
 * -------------------------------------------------------------------------- */
static void HandleButtons(uint32_t now)
{
    /* ทำตาม simulator: UP/DOWN ปรับ setting ที่เลือก, SET วน setting, HOLD freeze */
    if (ButtonJustPressed(&btn_up, now))   OscAdjust(+1);

    if (ButtonJustPressed(&btn_down, now)) OscAdjust(-1);

    if (ButtonJustPressed(&btn_hold, now)) {
        hold_active = !hold_active;
        osc_sidx    = 0;
        osc_dirty   = 1;
    }

    if (ButtonJustPressed(&btn_set, now)) {
        OscSetIndex(osc_sidx + 1);
        osc_dirty = 1;
    }

    /* BTN_SET long press (2 s) — auto scale */
    if (ButtonLongPress(&btn_set, now)) {
        AutoScale();
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
    if (hadc->Instance == ADC1) adc_done = 1;
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
    *t_us = AdcElapsedUs(t0);
    SCB_InvalidateDCache_by_Addr((uint32_t *)adc_raw_buf, bytes);
    __DSB();
    /* back to single-ADC mode and 16-bit DMA for every other rate level */
    ADC123_COMMON->CCR &= ~(ADC_CCR_MULTI | ADC_CCR_DMA | ADC_CCR_DDS | ADC_CCR_DELAY);
    DMA2_Stream0->CR = (DMA2_Stream0->CR & ~(DMA_SxCR_PSIZE | DMA_SxCR_MSIZE)) | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0;
    return ok;
}

/* จับข้อมูลดิบ n จุด (blocking สั้น ๆ) -> 1 = สำเร็จ */
static uint8_t AdcCaptureRaw(uint32_t n, uint32_t *t_us)
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

static float AdcRawFrequency(uint32_t n, float fs)
{
    uint16_t mn = 4095, mx = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (adc_raw_buf[i] < mn) mn = adc_raw_buf[i];
        if (adc_raw_buf[i] > mx) mx = adc_raw_buf[i];
    }
    osc_raw_vpp = (mx - mn) * ADC_VREF / ADC_FULL;
    adc_fast_content = 0;
    adc_rough_ratio = 0.0f;
    if (osc_raw_vpp < 0.05f) return 0.0f;
    {   /* mean |sample-to-sample change| / Vpp: < 2 % for anything below ~50 Hz in this window, > 25 % when there are <= 8 samples per cycle */
        uint32_t acc = 0;
        for (uint32_t i = 1; i < n; i++) acc += (adc_raw_buf[i] > adc_raw_buf[i - 1]) ? (uint32_t)(adc_raw_buf[i] - adc_raw_buf[i - 1]) : (uint32_t)(adc_raw_buf[i - 1] - adc_raw_buf[i]);
        adc_rough_ratio = (float)acc / (float)(n - 1u) / (float)(mx - mn);
        if (osc_raw_vpp >= 0.15f && adc_rough_ratio > 0.25f) adc_fast_content = 1;
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
            float prev = adc_raw_buf[i - 1], v = adc_raw_buf[i];
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
        if (f > fmax) { adc_too_fast = 1; continue; }
        if (f < 39.9f) continue;
        /* A low frequency cannot coexist with large sample-to-sample change: that is a fast signal near/above Nyquist
         * beating with the 3-ADC pattern, so refuse the reading instead of showing a false low number. */
        /* Every rate: the sample-to-sample change must match a signal of this frequency (~0.64*sin(pi*f/fs) of Vpp, plus ADC noise).
         * A fast signal that aliases to a low number (e.g. 2.4 MHz = 3 x 800 kSa/s -> 30 kHz) is far rougher than that. */
        if (adc_rough_ratio > 3.0f * 0.64f * sinf(3.14159265f * f / fs) + 0.10f + 12.0f / (float)(mx - mn)) continue;
        if (adc_tri_on && f < fs / 9.0f && (adc_fast_content || (adc_over_t && (HAL_GetTick() - adc_over_t) < 3000u))) continue;
        if (adc_tri_on) {
            /* the sample-to-sample change must match a sine of this frequency (about 0.64*sin(pi*f/fs) of Vpp); a fast signal
             * near Nyquist that fakes a low frequency is far rougher than that */
            /* second scan: count edge spacings further than 30 % from the mean period; > 5 % of them = not a clean periodic signal */
            float mean = (last - first) / (float)(edges - 1), lastat = 0.0f;
            uint32_t bad = 0, cnt = 0, good = 0;
            float gsum = 0.0f;
            armed = 0;
            for (uint32_t i = 1; i < n; i++) {
                float prev = adc_raw_buf[i - 1], v = adc_raw_buf[i];
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
        return f;
    }
    return 0.0f;
}

/* จับ 1 เฟรม: เลือกตัวหารจาก T/DIV -> เฉลี่ย -> raw_samples[N_SAMPLES] */
static void AdcCaptureFrame(void)
{
    float fs_want = (float)N_SAMPLES / (osc_tdivs_ms[osc_tdiv_i] * OSC_H_DIVS / 1000.0f);
    uint32_t n, t_us = 0;
    uint16_t vmin = 0xFFFF, vmax = 0;
    new_frame_ready = 0;
    osc_frame_valid = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_meas = (OscMeasure){0};
    strcpy(osc_det, "INVALID");

    /* Fast, independent measurement: 4 ms initially, grow on later frames
     * until >=3 periods are available. Max 81.9 ms only for slow/no signal.
     * Every acquisition is contiguous; never stitch separate DMA blocks.
     * Supported periodic input: 40 Hz..100 kHz, >=50 mVpp, within 0..3.3 V.
     * External anti-alias filtering/source impedance remain hardware duties. */
    if (!AdcSetRateLevel(adc_meas_seq[adc_meas_lvl]) || !AdcCaptureRaw(adc_measure_n, &t_us)) goto failed;
    adc_too_fast = 0;
    osc_raw_freq = AdcRawFrequency(adc_measure_n, dbg_adc_rate);
    adc_over_range = adc_too_fast;
    uint8_t meas_on_tri = adc_tri_on;
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

    if (osc_raw_freq > 0.0f && !meas_on_tri) {
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
            if ((float)(pmx - pmn) * ADC_VREF / ADC_FULL >= 0.15f && (float)pacc / 1599.0f > 0.25f * (float)(pmx - pmn)) {
                osc_raw_freq  = 0.0f;                     /* aliased: never show it */
                adc_hold_f    = 0.0f;
                adc_over_t    = HAL_GetTick() | 1u;
                adc_meas_lvl  = ADC_MEAS_LEVELS - 1u;     /* measure on the fastest rate from now on */
                adc_measure_n = 3200u;
            }
        }
    }

    {   /* hold the last good reading for 1.5 s: a marginal signal (e.g. 2 MHz at 2.4 samples/cycle) fails on some frames */
        uint32_t tnow = HAL_GetTick();
        if (osc_raw_freq > 0.0f)                                { adc_hold_f = osc_raw_freq; adc_hold_t = tnow; }
        else if (osc_raw_vpp < 0.05f || adc_over_range)         { adc_hold_f = 0.0f; }
        else if (adc_hold_f > 0.0f && (tnow - adc_hold_t) < 1500u) { osc_raw_freq = adc_hold_f; }
    }

    /* Display capture: the FASTEST rate level whose capture still fits the buffer (never truncate,
     * never stitch). 15 us/div = raw samples at 1.6 MSa/s; 6 ms/div falls to the 444 kSa/s level. */
    int lvl = 1;                                  /* 800 kSa/s: the original rate, used for every time base it can serve */
    float step = AdcRateHz(1) / fs_want;
    if (step < 1.0f) {                            /* time base faster than 800 kSa/s can draw */
        lvl = 0; step = AdcRateHz(0) / fs_want;   /* 1.6 MSa/s */
        if (step < 1.0f) { lvl = 4; step = AdcRateHz(4) / fs_want; }   /* 4.8 MSa/s, 3 ADCs */
    } else if ((N_SAMPLES + ADC_PRE_POINTS + 1) * step + 3 > ADC_RAW_MAX) {     /* too slow for 800 kSa/s: longer capture */
        lvl = 2; step = AdcRateHz(2) / fs_want;
        if ((N_SAMPLES + ADC_PRE_POINTS + 1) * step + 3 > ADC_RAW_MAX) { lvl = 3; step = AdcRateHz(3) / fs_want; }
    }
    if (!AdcSetRateLevel(lvl)) goto failed;
    dbg_adc_lvl = (uint8_t)lvl;
    step = dbg_adc_rate / fs_want;
    if (step < 1.0f) step = 1.0f;
    uint32_t w = (uint32_t)(step + 0.5f);
    uint32_t dec = w;
    n = (uint32_t)((N_SAMPLES + ADC_PRE_POINTS) * step) + w + 2u;
    if (n > ADC_RAW_MAX || !AdcCaptureRaw(n, &t_us)) goto failed;
    dbg_adc_ok = 1;
    dbg_adc_step = step;

    /* --- หาจุด trigger ในข้อมูลดิบ (ระดับ = ค่ากลางของบัฟเฟอร์) แล้วเริ่มวาดที่จุดนั้น --- */
    uint32_t start = 0;
    if (osc_stable) {
        float sum = 0.0f, mid;
        uint32_t lim = (uint32_t)((float)ADC_PRE_POINTS * step) + w;   /* ค้นเฉพาะส่วนก่อน trigger */
        if (lim < 4u) lim = 4u;
        if (lim > n) lim = n;
        for (uint32_t i = 0; i < n; i++) sum += (float)adc_raw_buf[i];
        mid = sum / (float)n;
        if (osc_trig_fall) {
            for (uint32_t i = 1; i < lim; i++)
                if ((float)adc_raw_buf[i - 1u] >= mid && (float)adc_raw_buf[i] < mid) { start = i; break; }
        } else {
            for (uint32_t i = 1; i < lim; i++)
                if ((float)adc_raw_buf[i - 1u] < mid && (float)adc_raw_buf[i] >= mid) { start = i; break; }
        }
        if ((float)start + (float)N_SAMPLES * step + (float)w > (float)n) start = 0;   /* กันหลุดขอบท้าย */

        /* เหลื่อมไปข้างหน้า ~2 จุดแสดงผล: ไม่ให้หน้าต่าง "เริ่มตรงจุดตัดศูนย์พอดี"
         * (ถ้าเริ่มพอดี ตัววัดความถี่ของ firmware จะนับขอบแรกพลาดเป็นครั้งคราว -> f กระพริบเป็น 0) */
        uint32_t lead = (uint32_t)(2.0f * step + 1.0f);
        start = (start > lead) ? (start - lead) : 0u;
    }
    dbg_adc_start = start;

    for (int i = 0; i < N_SAMPLES; i++) {   /* resample แบบเศษ + boxcar เฉลี่ย w จุด (เริ่มที่จุด trigger) */
        int32_t c = (int32_t)start + (int32_t)((float)i * step + 0.5f);
        int32_t lo = c - (int32_t)((w - 1u) / 2u);
        uint32_t acc = 0;
        if (lo < 0) lo = 0;
        if (lo + (int32_t)w > (int32_t)n) lo = (int32_t)n - (int32_t)w;
        if (lo < 0) lo = 0;
        for (uint32_t k = 0; k < w; k++) acc += adc_raw_buf[(uint32_t)lo + k];
        uint16_t v = (uint16_t)(acc / w);
        raw_samples[i] = (int16_t)v;
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }

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
    osc_dirty       = 1;
    return;
failed:
    dbg_adc_ok = 0;
    adc_over_range = 0;
    adc_over_t = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_dirty = 1;
}

/* SLOW_BEGIN ================================================================
 * SLOW mode (T/DIV >= 50 ms => หน้าต่าง >= 0.5 s) สำหรับสัญญาณความถี่ต่ำ เช่น 1 Hz
 *   FAST เดิม: burst DMA ที่ ~800 kSa/s ลง adc_raw_buf (<= 65520 จุด = ~82 ms) จึงครอบคลุม 0.5-5 s ไม่ได้
 *   SLOW ใหม่: SysTick (1 kHz) อ่าน ADC1 ทีละ 1 ค่า -> เฉลี่ยเป็นจุดแสดงผล 1 จุดทุก period_us
 *              -> ring 240 จุด (เก่า -> ใหม่) = โหมดไถล (roll) ไม่ต้องรอ trigger
 *   Fs ของจุดแสดงผล = 240 / (T/DIV x 10 ช่อง)  เช่น 200 ms/div -> 120 Sa/s (2 s ต่อหน้าจอ)
 *   ความถี่ที่แสดงมาจากหน้าต่างนี้เอง (0.2-60 Hz) ไม่ใช่การสุ่มเร็วของโหมด FAST (>= 40 Hz)
 *   สลับ ADC: SLOW = single conversion (ปิด CONT/DMA)  <->  FAST = continuous + DMA (ค่าเดิมของ AdcCaptureRaw)
 * ========================================================================== */
#define SLOW_MIN_TDIV_MS  50.0f           /* T/DIV ตั้งแต่ค่านี้ขึ้นไป = SLOW mode */
#define SLOW_SPREAD_MAX   200u            /* ADC counts (~120 mV): bursts on a real board show ~100 counts of noise even on a clean slow signal */

static volatile uint16_t slow_ring[N_SAMPLES];
static volatile uint32_t slow_head       = 0;   /* ตำแหน่งที่จะเขียนถัดไป (= จุดเก่าสุด) */
static volatile uint32_t slow_filled     = 0;   /* 0 = ยังไม่มีข้อมูลเลย */
static volatile uint32_t slow_period_us  = 2083;/* เวลาต่อ 1 จุดแสดงผล (us)              */
static volatile uint32_t slow_phase_us   = 0;
static volatile uint32_t slow_sum        = 0;
static volatile uint32_t slow_cnt        = 0;
#define SLOW_BURST        16u     /* conversions per 1 ms tick */
static volatile uint16_t slow_rough[N_SAMPLES];   /* mean burst spread (max-min, ADC counts) per display point */
static volatile uint32_t slow_dsum       = 0, slow_dcnt = 0;
volatile uint16_t        dbg_slow_spread = 0;     /* mean burst spread over the window (SWD) */
static int               slow_cfg_tdiv   = -1;  /* T/DIV ที่ ring ถูกสร้างไว้ (-1 = ADC อยู่โหมด FAST) */
volatile uint32_t        dbg_slow_pts    = 0;   /* นับจุดที่สร้างแล้ว (ดูผ่าน SWD) */
volatile uint16_t        dbg_slow_last   = 0;

#ifndef OSC_SLOW_HOST_TEST
/* อ่าน ADC1 หนึ่งครั้ง (single conversion, polling ~1-2 us) */
static inline uint16_t SlowAdcRead(void)
{
    uint32_t guard = 20000u;
    ADC1->SR = 0;                              /* ล้าง EOC/OVR/STRT */
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC) && --guard) { }
    return (uint16_t)(ADC1->DR & 0x0FFFu);
}
#endif

/* เรียกจาก SysTick_Handler ทุก 1 ms */
void OscSlowTick(void)
{
    if (!slow_active) return;

    {
        uint32_t acc = 0; uint16_t mn = 0xFFFF, mx = 0;
        for (uint32_t k = 0; k < SLOW_BURST; k++) {
            for (volatile uint32_t d = (k * 3u) % 7u; d > 0u; d--) { }   /* dither the spacing so a signal at a multiple of the burst rate cannot hide inside the burst */
            uint16_t a = SlowAdcRead();
            acc += a; if (a < mn) mn = a; if (a > mx) mx = a;
        }
        slow_sum += acc / SLOW_BURST;
        slow_dsum += (uint32_t)(mx - mn); slow_dcnt++;
    }
    slow_cnt++;
    slow_phase_us += 1000u;

    if (slow_phase_us >= slow_period_us) {
        slow_phase_us -= slow_period_us;
        uint16_t v = (uint16_t)(slow_sum / slow_cnt);   /* boxcar: เฉลี่ยทุกค่าที่อ่านใน period นี้ */
        slow_sum = 0; slow_cnt = 0;
        uint16_t rg = (uint16_t)(slow_dcnt ? (slow_dsum / slow_dcnt) : 0u);
        slow_dsum = 0; slow_dcnt = 0;

        if (slow_filled == 0) {                          /* จุดแรก: เติมทั้งหน้าต่างด้วยค่านี้ (เส้นราบ) แล้วไถลต่อ */
            for (uint32_t i = 0; i < N_SAMPLES; i++) { slow_ring[i] = v; slow_rough[i] = rg; }
            slow_filled = 1;
        }
        slow_ring[slow_head] = v;
        slow_rough[slow_head] = rg;
        slow_head = (slow_head + 1u) % N_SAMPLES;
        dbg_slow_pts++;
        dbg_slow_last = v;
    }
}

/* ตั้ง/คืนโหมด ADC และ ring ให้ตรงกับ T/DIV ปัจจุบัน — เรียกก่อนจับเฟรมทุกครั้ง (ไม่เรียกตอน HOLD) */
static void SlowSync(void)
{
    uint8_t want = (osc_tdivs_ms[osc_tdiv_i] >= SLOW_MIN_TDIV_MS);

    if (want && slow_cfg_tdiv != osc_tdiv_i) {
        slow_active = 0;                                  /* หยุด ISR ก่อนแตะรีจิสเตอร์ */
#ifndef OSC_SLOW_HOST_TEST
        if (slow_cfg_tdiv < 0) {                          /* เพิ่งเข้า SLOW จาก FAST (ADC หยุดอยู่แล้ว: AdcCaptureRaw เรียก Stop_DMA ทุกครั้ง) */
            AdcSetRateLevel(1);                           /* SLOW reads in a burst: always 12 MHz / 15 cycles (1.25 us per conversion), not whatever the last FAST capture left behind */
            ADC1->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_DDS);
            ADC1->CR2 |= ADC_CR2_ADON;
            for (volatile int d = 0; d < 3000; d++) { }   /* tSTAB หลังเปิด ADC */
        }
#endif
        slow_period_us = (uint32_t)(osc_tdivs_ms[osc_tdiv_i] * (float)OSC_H_DIVS * 1000.0f / (float)N_SAMPLES);
        slow_phase_us = 0; slow_sum = 0; slow_cnt = 0;
        slow_dsum = 0; slow_dcnt = 0; slow_fast_flag = 0;
        slow_head = 0; slow_filled = 0;
        slow_cfg_tdiv = osc_tdiv_i;
        slow_active = 1;
    } else if (!want && slow_cfg_tdiv >= 0) {             /* กลับ FAST: continuous + DMA ตามเดิม */
        slow_active = 0;
#ifndef OSC_SLOW_HOST_TEST
        ADC1->CR2 &= ~ADC_CR2_ADON;
        ADC1->SR = 0;
        ADC1->CR2 |= ADC_CR2_CONT;
#endif
        slow_cfg_tdiv = -1;
    }
}

/* ความถี่จากหน้าต่าง SLOW (raw_samples[]): จุดตัดกึ่งกลางขาขึ้น + hysteresis 20% — ต้องมี >= 2 ขอบ
 * (FAST ใช้ >= 3 ขอบ แต่ที่นี่หน้าต่าง 2-5 คาบ) คืน 0 ถ้าไม่พอ — ไม่แต่งตัวเลข */
static float SlowRawFrequency(float fs)
{
    uint16_t mn = 4095, mx = 0;
    for (uint32_t i = 0; i < N_SAMPLES; i++) {
        uint16_t v = (uint16_t)raw_samples[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
    }
    osc_raw_vpp = (float)(mx - mn) * ADC_VREF / ADC_FULL;
    if (osc_raw_vpp < 0.05f) return 0.0f;
    float mid = (float)(mx + mn) * 0.5f, hyst = (float)(mx - mn) * 0.2f;
    float first = 0.0f, last = 0.0f;
    uint32_t edges = 0;
    int armed = 0;
    for (uint32_t i = 1; i < N_SAMPLES; i++) {
        float prev = (float)raw_samples[i - 1], v = (float)raw_samples[i];
        if (v < mid - hyst) armed = 1;
        if (armed && prev < mid && v >= mid) {
            float at = (float)(i - 1) + (mid - prev) / (v - prev);
            if (!edges) first = at;
            last = at;
            edges++;
            armed = 0;
        }
    }
    if (edges < 2 || last <= first) return 0.0f;
    float f = (float)(edges - 1) * fs / (last - first);
    return (f >= 0.2f && f <= fs / 8.0f) ? f : 0.0f;
}

/* ดึง ring (เก่า -> ใหม่) ลง raw_samples[] เหมือน AdcCaptureFrame() และตั้งค่า osc_* ที่ตัวแสดงผล/วัดค่าต้องใช้ */
static void SlowCaptureFrame(void)
{
    uint16_t vmin = 0xFFFF, vmax = 0;
    uint32_t pm, rsum = 0;

    new_frame_ready = 0;
    osc_frame_valid = 0;
    osc_raw_freq = osc_raw_vpp = 0.0f;
    osc_meas = (OscMeasure){0};
    strcpy(osc_det, "INVALID");

#ifndef OSC_SLOW_HOST_TEST
    {   /* จุดแรกของ ring มาถึงภายใน 1 period (<= ~21 ms) — รอสั้น ๆ แทนการขึ้น ADC ERR */
        uint32_t t0 = HAL_GetTick();
        while (!slow_filled && (HAL_GetTick() - t0) < 80u) { }
    }
#endif
    if (!slow_filled) { dbg_adc_ok = 0; osc_dirty = 1; return; }

    pm = __get_PRIMASK();
    __disable_irq();
    {
        uint32_t h = slow_head;
        for (uint32_t i = 0; i < N_SAMPLES; i++) {
            uint16_t v = slow_ring[(h + i) % N_SAMPLES];
            if (i >= N_SAMPLES - 8u) rsum += slow_rough[(h + i) % N_SAMPLES];     /* newest 8 points only: a jump to a fast signal shows up within ~8 points */
            raw_samples[i] = (int16_t)v;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
    }
    __set_PRIMASK(pm);

    osc_fs_eff       = 1000000.0f / (float)slow_period_us;     /* Fs จริงของจุดแสดงผล */
    osc_raw_freq     = SlowRawFrequency(osc_fs_eff);
    {   /* ALIAS GUARD: within one ~20 us burst a <=60 Hz signal barely moves (noise only, spread ~10 counts);
         * a fast signal swings by a large part of its amplitude (spread 30+ counts = ~25 mV). */
        uint16_t spread = (uint16_t)(rsum / 8u);
        dbg_slow_spread = spread;
        slow_fast_flag  = 0;
        /* fast content = the 16-conversion burst spans a large part of the whole signal swing (a slow signal never does) */
        if (spread > SLOW_SPREAD_MAX) {
            slow_fast_flag = 1;                                        /* too fast for SLOW */
            osc_raw_freq   = 0.0f;                                     /* never show an aliased number */
        }
    }
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

#else  /* !ADC_INPUT_ENABLE */
void OscSlowTick(void) { }
#endif /* ADC_INPUT_ENABLE */

/* USER CODE END 0 */

void SystemClock_Config(void);

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
    }

    /* Button struct init (done here to avoid static-initializer cast issue) */
    btn_up   = (Button){BTN_UP_PORT,   BTN_UP_PIN,   0, 0, 0, 0};
    btn_down = (Button){BTN_DOWN_PORT, BTN_DOWN_PIN, 0, 0, 0, 0};
    btn_hold = (Button){BTN_HOLD_PORT, BTN_HOLD_PIN, 0, 0, 0, 0};
    btn_set  = (Button){BTN_SET_PORT,  BTN_SET_PIN,  0, 0, 0, 0};

    /* LCD init */
    UartInputInit();          /* เริ่มรับคำสั่งจาก USART3 (Virtual COM) */
    UartPanel();              /* แสดงเมนู/สถานะครั้งแรกออกพอร์ต serial */

    ILI9341_Init();
    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
    ILI9341_Fill_Screen(BLACK);

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

    DrawGrid();
    DrawCursor();
    DrawTopPanel();

    uint32_t last_frame_tick  = 0;
    uint32_t last_header_tick = 0;

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE BEGIN 3 */

        uint32_t now = HAL_GetTick();

        /* --- Button handling (every loop) --------------------------------- */
        HandleButtons(now);
        UartPoll();               /* รับคำสั่งจาก USART3 (แทนปุ่มจริง) */


        /* --- Waveform frame update (~30 FPS) ------------------------------ */
        if (now - last_frame_tick >= FRAME_MS)
        {
            last_frame_tick = now;

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
                    if (slow_active) SlowCaptureFrame();
                    else             AdcCaptureFrame();   /* p30: อ่านข้อมูลจริงจาก ADC (PA0) */
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
                OscProcessFrame();      /* calibrate offset (AC) + moving average filter */
                OscMeasureFrame();      /* วัดค่า Vmax/Vmin/Vpp/RMS/Freq + ชนิดคลื่น */
                /* AutoScale ขั้น 2: พอวัดใหม่ที่ time base เร็วสุดแล้ว -> เลือกสเกล */
                if (osc_auto_stage) OscAutoScaleStep();
                OscAutoTimebase();      /* AUTO: สลับ FAST/SLOW + T/DIV ตามความถี่ (ถ้าไม่ได้ปรับเอง) */
                new_frame_ready = 0;

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
                ClearCursors();         /* ล้างเส้น cursor เก่าก่อนวาดเฟรมใหม่ */
                MapToScreen(trig_offset);
                DrawWaveform();
                DrawCursors();          /* HOLD: วาดเส้น C1/C2 ทับหลังวาดรูป */
                DrawSettingBar();       /* แถบ "กำลังตั้งค่าอะไร" ทับหลังวาดรูป */
                osc_dirty = 0;

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
                    hold_active     = 1;
                }
            }
            skip_draw: ;
        }

        /* --- Header update (every 300 ms) --------------------------------- */
        if (now - last_header_tick >= HEADER_MS)
        {
            last_header_tick = now;

#if SHOW_FPS
            uint32_t t_h0 = HAL_GetTick();
#endif
            DrawTopPanel();

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


