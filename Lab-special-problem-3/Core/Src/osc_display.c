/* Display concern; see README.md for ownership and extension points. */
#include "osc_display.h"
#include "osc_signal.h"
#include "osc_input.h"
#include "osc_app.h"
#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void MapToScreen(int trig_offset);
static void DrawGrid(void);
static void DrawLineBresenham(int x0, int y0, int x1, int y1, uint16_t color);
static int OscCursorX(int idx);
void ClearCursors(void);
void DrawCursors(void);
static void DrawCursor(void);
void DrawWaveform(void);
void DrawSettingBar(void);
void DrawTopPanel(void);
void DrawFps(float fps);
void OscDisplayInit(void);

/* --- Display double-buffer ------------------------------------------------- */
static int16_t old_y[N_SAMPLES];
static int16_t new_y[N_SAMPLES];
/* --- Cursor ---------------------------------------------------------------- */
static uint16_t cursor_x = SCREEN_W / 2;  /* vertical cursor at screen centre  */
/* TODO: add BTN logic to move cursor_x left/right if needed                   */
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

void MapToScreen(int trig_offset)
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
 * Grid rows: y = 20, 60, 100, 140, 180, 220 (GRID_Y_OFFSET + n*GRID_Y_STEP)
 * -------------------------------------------------------------------------- */
static void DrawGrid(void)
{
    for (int y = GRID_Y_OFFSET; y < SCREEN_H; y += GRID_Y_STEP)
        for (int x = 0; x < SCREEN_W; x += GRID_X_DOT)
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
void ClearCursors(void)
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

void DrawCursors(void)
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
void DrawWaveform(void)
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
void DrawSettingBar(void)
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

void DrawTopPanel(void)
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
    if (osc_meas.freq >= 1000.0f)
        snprintf(buf, sizeof(buf), "F:%.1fkHz", (double)(osc_meas.freq / 1000.0f));
    else
        snprintf(buf, sizeof(buf), "F:%.0fHz",  (double)osc_meas.freq);
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
    if (osc_stable)
        snprintf(buf, sizeof(buf), "AUTO:%s", osc_trig_fall ? "F" : "R");
    else
        snprintf(buf, sizeof(buf), "ROLL");
    ILI9341_Draw_Text(buf, 148, 20, CYAN, 1, BLACK);
    }

//    /* RMS — x=245 (245 < 255, safe for uint8_t) */
//    snprintf(buf, sizeof(buf), "Rs:%.2f", (double)osc_meas.vrms);
//    ILI9341_Draw_Text(buf, 2, 20, LIGHTGREY, 1, BLACK);

    /* Separator line */
    ILI9341_Draw_Horizontal_Line(0, HEADER_H - 1, SCREEN_W, DARKGREY);
}
#if SHOW_FPS
/* แสดงเฟรมเรตจริง (debug) — ฟอร์แมตกว้างคงที่ เพื่อไม่ให้เลขเดิมค้างบนจอ */
void DrawFps(float fps)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%4.1ffps", (double)fps);
    ILI9341_Draw_Text(buf, 200, SCREEN_H - 12, DARKCYAN, 1, BLACK);   /* เผื่อที่ให้แถบ SET ทางซ้าย */
}
#endif

void OscDisplayInit(void)
{
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
}
