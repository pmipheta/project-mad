/* uart_console.c - USART3 console: RX interrupt ring buffer, status panel, command parser. */
#include "app/app.h"

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

void UartInputInit(void)
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
void UartPanel(void)
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
