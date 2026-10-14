#ifndef OSC_CONFIG_H
#define OSC_CONFIG_H
/* ===== p30: อินพุตจริงจากเครื่องกำเนิดสัญญาณ (ADC1 = PA0) ==================
 *   1 = ใช้ข้อมูลจริงจาก ADC   |   0 = ใช้ simulator เดิม (GenFrame ฯลฯ)
 *   สลับค่าเดียวนี้เพื่อกลับไปใช้ simulator ได้ทันที (โค้ดเดิมไม่ถูกแตะ)
 * ========================================================================== */
#ifndef ADC_INPUT_ENABLE
#define ADC_INPUT_ENABLE   1
#endif
#define ADC_RAW_MAX        65520u  /* <=16-bit NDTR; whole 32-byte cache lines */
#define OSC_NT             7        /* จำนวน T/DIV (p44: เพิ่ม 5.0 ms/div ให้ช่วง 40-100 Hz) */
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
#define SCREEN_W        320    /* 320                         */
#define SCREEN_H        240   /* 240                         */
#define CENTER_Y        120          /* waveform vertical midpoint              */

/* --- Grid ------------------------------------------------------------------ */
#define GRID_Y_STEP     40           /* px between horizontal grid lines        */
#define GRID_Y_OFFSET   20           /* first grid line Y (just below header)   */
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
#define SIM_FS_HZ        100000.0f  /* สมมติ sample rate 100 kSa/s (หน้าต่าง 2.4 ms) */
#define SIM_FREQ_HZ      1000.0f    /* ความถี่ตั้งต้นของ function generator          */
#define SIM_NOISE        0.05f      /* noise floor (±5% ของครึ่งสเกล)                */
#define SIM_SCALE        0.62f      /* สเกล: 1 หน่วยสัญญาณ -> สัดส่วนของ ADC_MID    */
#define SIM_T_WRAP       3000.0f    /* วนเวลาสะสมที่ 3000 s (ทุกความถี่ลงตัวพอดี) */

/* โหมดจอ: สลับตอนรันได้ด้วยคีย์ m (หรือ m a / m r / m 1 / m 0)
 *   1 = จัดแนว trigger ให้กราฟนิ่ง (ขอบ RISE/FALL ตามค่า TRIG)
 *   0 = roll / chart-recorder: ข้อมูลใหม่เข้าขวา เส้นไถลซ้าย                 */
#define SIM_PUSH         8          /* sample ใหม่ต่อเฟรม (มาก=เลื่อนเร็ว/เป็นขั้น) */

#endif
