#include "app/app.h"

static int      OscCursorX(int idx);
static void     DrawLineBresenham(int x0, int y0, int x1, int y1, uint16_t color);

void MapToScreen(int trig_offset)
{
    osc_plot_offset = trig_offset;
    const float *vs = osc_v;
    float dc_sh = 0.0f;
    if (!osc_calib) {
        float lo = vs[0], hi = vs[0];
        for (int i = 1; i < N_SAMPLES; i++) { if (vs[i] < lo) lo = vs[i]; if (vs[i] > hi) hi = vs[i]; }
        if (osc_env_on) {
            const float kc = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;
            for (int i = 0; i < N_SAMPLES; i++) {
                float bl = ((float)osc_env_lo_ct[i] - osc_offset_ct) * kc, bh = ((float)osc_env_hi_ct[i] - osc_offset_ct) * kc;
                if (bl < 0.0f) bl = 0.0f;
                if (bh < 0.0f) bh = 0.0f;
                if (bl < lo) lo = bl;
                if (bh > hi) hi = bh;
            }
        }
        dc_sh = 0.5f * (lo + hi);
    }
    osc_dc_shift_v = dc_sh;
    for (int i = 0; i < N_SAMPLES; i++) {
        int src = (trig_offset + i);
        if (src >= N_SAMPLES) src -= N_SAMPLES;

        float v = vs[src] - dc_sh;
        int y = CENTER_Y - (int)(v / volts_per_div * (float)GRID_Y_STEP);

        if (y < WAVEFORM_TOP) y = WAVEFORM_TOP;
        if (y >= SCREEN_H)    y = SCREEN_H - 1;
        new_y[i] = (int16_t)y;
    }
    env_y_on = osc_env_on;
    if (env_y_on) {
        const float kc = ADC_VREF / (float)ADC_FULL * ADC_INPUT_GAIN;
        for (int i = 0; i < N_SAMPLES; i++) {
            int src = (trig_offset + i);
            if (src >= N_SAMPLES) src -= N_SAMPLES;
            for (int e = 0; e < 2; e++) {
                float v = ((float)(e ? osc_env_lo_ct[src] : osc_env_hi_ct[src]) - osc_offset_ct) * kc;
                if (!osc_calib && v < 0.0f) v = 0.0f;
                v -= dc_sh;
                int y = CENTER_Y - (int)(v / volts_per_div * (float)GRID_Y_STEP);
                if (y < WAVEFORM_TOP) y = WAVEFORM_TOP;
                if (y >= SCREEN_H)    y = SCREEN_H - 1;
                if (e) env_y_hi[i] = (int16_t)y; else env_y_lo[i] = (int16_t)y;
            }
        }
    }
}

void DrawGrid(void)
{
    for (int y = GRID_Y_OFFSET; y < SCREEN_H; y += GRID_Y_STEP)
        for (int x = 0; y >= WAVEFORM_TOP && x < SCREEN_W; x += GRID_X_DOT)
            ILI9341_Draw_Pixel((uint16_t)x, (uint16_t)y, DARKGREY);
    for (int x = 0; x < SCREEN_W; x += GRID_X_STEP)
        for (int y = WAVEFORM_TOP; y < SCREEN_H; y++)
            if (IS_GRID_DOT(x, y)) ILI9341_Draw_Pixel((uint16_t)x, (uint16_t)y, DARKGREY);
}

static int dl_only_col = -1;
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

static int OscCursorX(int x)
{
    if (x < 0) x = 0;
    if (x >= SCREEN_W) x = SCREEN_W - 1;
    return x;
}

static uint8_t osc_full_redraw = 1;
uint8_t cursors_on = 0;
static int     cur_x1 = -1, cur_x2 = -1;

void ClearCursors(void)
{
    if (!cursors_on) return;
    osc_full_redraw = 1;
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

void DrawCursor(void)
{
    for (int y = WAVEFORM_TOP; y < SCREEN_H; y++)
        ILI9341_Draw_Pixel(cursor_x, (uint16_t)y, DARKCYAN);
}

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

static uint16_t BackgroundColour(int x, int y)
{
    if (hold_active && (x == OscCursorX(osc_c1) || x == OscCursorX(osc_c2))) return YELLOW;
    if (x == (int)cursor_x && y >= WAVEFORM_TOP) return DARKCYAN;
    if (IS_GRID_DOT(x, y)) return DARKGREY;
    return BLACK;
}

static void TraceColumns(uint8_t *lo, uint8_t *hi)
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
    if (env_y_on) {
        for (int x = 0; x < SCREEN_W; x++) {
            int i = (x * N_SAMPLES) / SCREEN_W;
            if (env_y_lo[i] < lo[x]) lo[x] = (uint8_t)env_y_lo[i];
            if (env_y_hi[i] > hi[x]) hi[x] = (uint8_t)env_y_hi[i];
        }
    }
}

void DrawWaveform(void)
{
    uint8_t buf[2 * (SCREEN_H - WAVEFORM_TOP)];
    static uint8_t inited = 0;
    if (!inited) { for (int x = 0; x < SCREEN_W; x++) { col_lo_old[x] = 255; col_hi_old[x] = 0; } inited = 1; osc_full_redraw = 1; }
    TraceColumns(col_lo_new, col_hi_new);
    for (int x = 0; x < SCREEN_W; x++) {
        int nlo = col_lo_new[x], nhi = col_hi_new[x];
        int olo = col_lo_old[x], ohi = col_hi_old[x];
        int has_new = (nlo <= nhi), has_old = (olo <= ohi);
        if (!osc_full_redraw && nlo == olo && nhi == ohi) continue;
        int y1, y2;
        if (has_new && has_old) { y1 = (nlo < olo) ? nlo : olo; y2 = (nhi > ohi) ? nhi : ohi; }
        else if (has_new)       { y1 = nlo; y2 = nhi; }
        else if (has_old)       { y1 = olo; y2 = ohi; }
        else continue;
        if (osc_full_redraw && has_new && !has_old) { }
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

#define BAR_X       2
#define BAR_Y       (SCREEN_H - 12)
#define BAR_CHAR_W  6
#define BAR_CHAR_H  8
static char    bar_txt[40];
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
    else if (!strcmp(name, "C1"))     snprintf(buf, sizeof(buf), "SET C1=%3d  UP/DOWN HOLD", osc_c1);
    else if (!strcmp(name, "C2"))     snprintf(buf, sizeof(buf), "SET C2=%3d  UP/DOWN HOLD", osc_c2);
    else                              snprintf(buf, sizeof(buf), "SET --");

    if (fast && bar_ok && strlen(buf) == strlen(bar_txt)) {
        for (int k = 0; buf[k]; k++)
            if (buf[k] != bar_txt[k]) ILI9341_Draw_Char(buf[k], (uint16_t)(BAR_X + k * BAR_CHAR_W), BAR_Y, BLACK, 1, CYAN);
    } else {
        ILI9341_Draw_Text(buf, BAR_X, BAR_Y, BLACK, 1, CYAN);
    }
    snprintf(bar_txt, sizeof(bar_txt), "%s", buf); bar_ok = 1;
    snprintf((char *)dbg_lcd_bar, sizeof(dbg_lcd_bar), "%s", buf);
}

void DrawSettingBar(void) { SettingBarDraw(0); }

void CursorsMoveFast(void)
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

extern volatile uint16_t dbg_adc_min, dbg_adc_max;
#define HDR_FIELDS 32
static char     hdr_txt[HDR_FIELDS][24];
static uint16_t hdr_fg[HDR_FIELDS], hdr_bg[HDR_FIELDS];
static uint8_t  hdr_ok[HDR_FIELDS];
void HdrInvalidate(void) { memset(hdr_ok, 0, sizeof hdr_ok); }
#define HDR_CHAR_W 6
static void HdrText(int id, const char *t, uint16_t x, uint16_t y, uint16_t fg, uint16_t size, uint16_t bg)
{
    if (id < 0 || id >= HDR_FIELDS) { ILI9341_Draw_Text(t, x, y, fg, size, bg); return; }
    int n = (int)strlen(t); if (n > 23) n = 23;
    int m = hdr_ok[id] ? (int)strlen(hdr_txt[id]) : 0;
    int force = (!hdr_ok[id] || hdr_fg[id] != fg || hdr_bg[id] != bg);
    for (int k = 0; k < ((n > m) ? n : m); k++) {
        char ch = (k < n) ? t[k] : ' ';
        if (!force && k < m && ch == hdr_txt[id][k]) continue;
        ILI9341_Draw_Char(ch, (uint16_t)(x + k * HDR_CHAR_W), y, fg, 1, (k < n) ? bg : BLACK);
    }
    memcpy(hdr_txt[id], t, (size_t)n); hdr_txt[id][n] = 0;
    hdr_fg[id] = fg; hdr_bg[id] = bg; hdr_ok[id] = 1;
}
#define HDRT(...) HdrText(__COUNTER__, __VA_ARGS__)
void DrawTopPanel(void)
{
    char buf[20];
    OscMeasure tm; OscMeasGet(&tm);
    {   static uint8_t last_hold = 255u;
        if (last_hold != (uint8_t)hold_active) { ILI9341_Draw_Filled_Rectangle_Coord(0, 0, SCREEN_W - 1, HEADER_H - 2, BLACK); HdrInvalidate(); last_hold = (uint8_t)hold_active; }
    }

    const char *selname = OscSetName();
    uint8_t blink_on = (uint8_t)((HAL_GetTick() / (BLINK_PERIOD_MS)) & 1u);
    int sel_vdiv = (strcmp(selname, "V/DIV") == 0);
    int sel_tdiv = (strcmp(selname, "T/DIV") == 0);

    MvFmtV(buf, sizeof(buf), volts_per_div);
    HDRT(buf, 2, 1, sel_vdiv ? (blink_on ? MAGENTA : YELLOW) : YELLOW, 1, BLACK);

    MvFmtT(buf, sizeof(buf), osc_capture_tdiv);
    HDRT(buf, 72, 1, sel_tdiv ? (blink_on ? MAGENTA : WHITE) : WHITE, 1, BLACK);

    OscSetValueText(buf, sizeof(buf));
    if (blink_on) HDRT(buf, 150, 1, BLACK, 1, ORANGE);
    else          HDRT(buf, 150, 1, ORANGE, 1, BLACK);
    snprintf((char *)dbg_lcd_set, sizeof(dbg_lcd_set), "%s", buf);

    const char *st_txt; uint16_t st_fg;
    if (!osc_frame_valid)                             { st_txt = "ADC ERR"; st_fg = RED; }
    else if (hold_active)                             { st_txt = "HOLD";    st_fg = RED; }
    else if (trigger_mode == TRIG_SINGLE && !single_captured) { st_txt = "WAIT"; st_fg = ORANGE; }
    else                                              { st_txt = "RUN ";    st_fg = GREEN; }
    HdrText(31, st_txt, 226, 1, st_fg, 1, BLACK);
    HDRT(osc_calib ? "AC" : "DC", 296, 1, osc_calib ? CYAN : ORANGE, 1, BLACK);

    if (hold_active) {
        snprintf(buf, sizeof(buf), "U1=%.2fV", (double)OscCursorV(osc_c1));
        HDRT(buf, 2, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "U2=%.2fV", (double)OscCursorV(osc_c2));
        HDRT(buf, 72, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "dV=%.2fV", (double)(OscCursorV(osc_c2) - OscCursorV(osc_c1)));
        HDRT(buf, 148, 10, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "dt=%.2fms",
                 (double)(OscCursorDtS() * 1000.0f));
        HDRT(buf, 2, 20, YELLOW, 1, BLACK);
        snprintf(buf, sizeof(buf), "%s%s ", slow_active ? "SLOW" : "FAST", osc_auto_tb ? "" : "*");
        HDRT(buf, 148, 20, osc_auto_tb ? GREEN : ORANGE, 1, BLACK);
    } else {
    snprintf(buf, sizeof(buf), "Vp:%.2fV", (double)tm.vpp);
    HDRT(buf, 2, 10, CYAN, 1, BLACK);

    {
        char fb[16];
        if (tm.freq >= 1000000.0f)  snprintf(fb, sizeof(fb), "%.3fMHz", (double)(tm.freq / 1000000.0f));
        else if (tm.freq >= 1000.0f) snprintf(fb, sizeof(fb), "%.1fkHz", (double)(tm.freq / 1000.0f));
        else if (tm.freq >= 10.0f)  snprintf(fb, sizeof(fb), "%.0fHz",  (double)tm.freq);
        else if (tm.freq > 0.0f)    snprintf(fb, sizeof(fb), "%.2fHz",  (double)tm.freq);
        else if (adc_over_t && (HAL_GetTick() - adc_over_t) < 1500u && !slow_active) snprintf(fb, sizeof(fb), ">2.1MHz");
        else                              snprintf(fb, sizeof(fb), "---");
        snprintf(buf, sizeof(buf), "F:%-8s", fb);
    }
    HDRT(buf,72, 10, GREEN, 1, BLACK);

    snprintf(buf, sizeof(buf), "Rs:%.2f", (double)tm.vrms);
    HDRT(buf, 148, 10, LIGHTGREY, 1, BLACK);

    snprintf(buf, sizeof(buf), "Mx:%.2fV", (double)tm.vmax);
    HDRT(buf, 2, 20, LIGHTGREY, 1, BLACK);

    snprintf(buf, sizeof(buf), "Mn:%.2fV", (double)tm.vmin);
    HDRT(buf, 72, 20, LIGHTGREY, 1, BLACK);

    if (slow_active)
        snprintf(buf, sizeof(buf), !osc_stable ? "ROLL  " : (slow_trig_on ? (osc_trig_fall ? "AUTO:F" : "AUTO:R") : "AUTO  "));
    else if (osc_stable && osc_lock_auto)
        snprintf(buf, sizeof(buf), "AUTO:%s", osc_trig_fall ? "F" : "R");
    else
        snprintf(buf, sizeof(buf), "ROLL");
    HDRT(buf, 148, 20, CYAN, 1, BLACK);
    HDRT(osc_auto_tb ? "AUT  " : (slow_active ? "SLOW*" : "FAST*"), 232, 20, osc_auto_tb ? GREEN : ORANGE, 1, BLACK);
    HDRT((osc_frame_valid && (dbg_adc_min <= 8u || dbg_adc_max >= 4087u)) ? "CLIP" : "    ",
                      200, 20, RED, 1, BLACK);
    }

    ILI9341_Draw_Horizontal_Line(0, HEADER_H - 1, SCREEN_W, DARKGREY);
}
#undef HDRT
#if SHOW_FPS
void DrawFps(float fps)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%4.1ffps", (double)fps);
    ILI9341_Draw_Text(buf, 200, SCREEN_H - 12, DARKCYAN, 1, BLACK);
}
#endif
