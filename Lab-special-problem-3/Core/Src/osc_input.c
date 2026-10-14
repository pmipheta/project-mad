/* Input concern; see README.md for ownership and extension points. */
#include "osc_input.h"
#include "osc_signal.h"
#include "osc_simulator.h"
#include "osc_app.h"
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void USART3_IRQHandler(void);
void UartPrint(const char *s);
static void UartInputInit(void);
static int OscSetCount(void);
const char *OscSetName(void);
static int OscClampI(int v, int lo, int hi);
static float OscClampF(float v, float lo, float hi);
void OscSetValueText(char *out, int n);
static void OscSetIndex(int idx);
static void OscAdjust(int dir);
static void UartNum(const char *label, float v, const char *unit);
static void UartPanel(void);
static float UartArgF(const char *s, float def);
static void UartCommand(char *line);
void UartPoll(void);
void AutoScale(void);

void HandleButtons(uint32_t now);
void OscInputInit(void);

/* Button state for debouncing */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint32_t      last_press_tick;  /* tick of last confirmed press            */
    uint32_t      held_since_tick;  /* tick of when held press began           */
    uint8_t       is_held;          /* 1 = currently held down                 */
    uint8_t       long_fired;       /* 1 = long-press action already triggered */
} Button;
/* --- Buttons --------------------------------------------------------------- */
static Button btn_up, btn_down, btn_hold, btn_set;
static const char  *osc_set_names[]   = {"FILTER", "TRIG", "V/DIV", "T/DIV"};
static const char  *osc_hold_names[]  = {"C1", "C2", "V/DIV"};
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

void UartPrint(const char *s)
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
static int OscSetCount(void)
{
    return hold_active ? OSC_HOLD_COUNT : OSC_SET_COUNT;
}

const char *OscSetName(void)
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

void OscSetValueText(char *out, int n)   /* "FILTER:5" / "TRIG:RISE" ... */
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

    UartPrint("\r\n== FUNCTION GENERATOR (input) ================");
    snprintf(b, sizeof(b), "\r\n Wave:%s  Freq:%d Hz",
             gen_wave_names[gen_wave], (int)gen_freq);
    UartPrint(b);
    UartNum("\r\n Amp:", gen_amp, " V");  UartNum("  DC:", gen_dc, " V");
    UartNum("  Noise:", gen_noise, " V");
    snprintf(b, sizeof(b), "\r\n V/div:%d (%d.%d V)  T/div:%d (%d.%d ms)  Fs:%d kSa/s",
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

void UartPoll(void)
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
void AutoScale(void)
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
void HandleButtons(uint32_t now)
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

void OscInputInit(void)
{
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
}
