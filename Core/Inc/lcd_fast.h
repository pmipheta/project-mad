/* lcd_fast.h - v78 fast LCD path (register-level SPI5/GPIOC writes). Included by main.c after the ILI9341 driver header and FAST_O2.
 * Kept in its own file so main.c stays small; tests/test_fast_lcd.py compiles this block against a fake register model. */
#ifndef LCD_FAST_H
#define LCD_FAST_H
/* LCDFAST_BEGIN ==========================================================================================================
 * v78 fast LCD path. The HAL/driver path costs ~12 HAL_SPI_Transmit + ~36 HAL_GPIO_WritePin per column (and 3+... per cursor pixel) at -O0,
 * ~60 us of CPU for ~8 us of SPI time. Here the SAME byte stream (same commands, same parameters, same DC level per byte, then the data)
 * is written straight to the SPI5/GPIOC registers, with CS low once for the whole column. Rollback: LCD_FAST_SPI 0 = the old code exactly (main.c LcdWriteColumn dispatches on LCD_FAST_SPI and keeps the old HAL body unchanged).
 * Rules mirrored from HAL_SPI_Transmit (stm32f7xx_hal_spi.c): enable SPE only if it is not set; send bytes when TXE; at the end wait FTLVL==0
 * then BSY==0 (SPI_EndRxTxTransaction); clear OVR by reading DR then SR (RX is never read). hspi5.State/Lock are never touched (stay READY/unlocked).
 * DC is only flipped, and CS only raised, while the SPI is idle (the last bit must have left the shift register first). */
#define LCD_FAST_SPI 1
#if LCD_FAST_SPI
extern volatile uint16_t LCD_WIDTH, LCD_HEIGHT;   /* defined in ILI9341_STM32_Driver.c (bounds check of Draw_Pixel) */
#ifndef LCD_DR_WR
#define LCD_DR_WR(b)    (*(volatile uint8_t *)&SPI5->DR = (uint8_t)(b))   /* 8-bit access = one frame */
#endif
#ifndef LCD_DR_RD
#define LCD_DR_RD()     (SPI5->DR)
#endif
#ifndef LCD_SR_RD
#define LCD_SR_RD()     (SPI5->SR)
#endif
#ifndef LCD_BSRR
#define LCD_BSRR(port, v) ((port)->BSRR = (v))
#endif
#define LCD_CS_LOW()    LCD_BSRR(LCD_CS_PORT, (uint32_t)LCD_CS_PIN << 16)
#define LCD_CS_HIGH()   LCD_BSRR(LCD_CS_PORT, (uint32_t)LCD_CS_PIN)
#define LCD_DC_LOW()    LCD_BSRR(LCD_DC_PORT, (uint32_t)LCD_DC_PIN << 16)
#define LCD_DC_HIGH()   LCD_BSRR(LCD_DC_PORT, (uint32_t)LCD_DC_PIN)

static FAST_O2 void LcdSpiBytes(const uint8_t *p, uint32_t n)
{
    if (!(SPI5->CR1 & SPI_CR1_SPE)) SPI5->CR1 |= SPI_CR1_SPE;           /* like __HAL_SPI_ENABLE */
    while (n--) {
        while (!(LCD_SR_RD() & SPI_SR_TXE)) { }
        LCD_DR_WR(*p++);
    }
    while (LCD_SR_RD() & SPI_SR_FTLVL) { }                              /* TX FIFO empty ... */
    while (LCD_SR_RD() & SPI_SR_BSY)   { }                              /* ... and last bit shifted out: now DC/CS may change */
    while (LCD_SR_RD() & SPI_SR_FRLVL) (void)LCD_DR_RD();              /* drain RX FIFO like SPI_EndRxTxTransaction (FRLVL empty) */
    (void)LCD_DR_RD();                                                  /* clear OVR exactly like __HAL_SPI_CLEAR_OVRFLAG: read DR, then SR */
    (void)LCD_SR_RD();
}

static FAST_O2 void LcdSpiCmd(uint8_t c)                { LCD_DC_LOW();  LcdSpiBytes(&c, 1); }
static FAST_O2 void LcdSpiData(const uint8_t *p, uint32_t n) { LCD_DC_HIGH(); LcdSpiBytes(p, n); }

static FAST_O2 void LcdWriteColumnFast(uint16_t x, uint16_t y1, uint16_t y2, const uint8_t *bytes, uint32_t n_bytes)
{
    uint8_t a[4];
    LCD_CS_LOW();                                                       /* once for the whole column */
    LcdSpiCmd(0x2A);
    a[0] = (uint8_t)(x >> 8);  a[1] = (uint8_t)x;  a[2] = (uint8_t)(x >> 8);  a[3] = (uint8_t)x;
    LcdSpiData(a, 4);
    LcdSpiCmd(0x2B);
    a[0] = (uint8_t)(y1 >> 8); a[1] = (uint8_t)y1; a[2] = (uint8_t)(y2 >> 8); a[3] = (uint8_t)y2;
    LcdSpiData(a, 4);
    LcdSpiCmd(0x2C);
    LcdSpiData(bytes, (uint16_t)n_bytes);
    LCD_CS_HIGH();
}

static FAST_O2 void LcdFastPixel(uint16_t X, uint16_t Y, uint16_t Colour)   /* same byte stream as ILI9341_Draw_Pixel */
{
    uint8_t a[4];
    if ((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT)) return;
    LCD_CS_LOW();
    LcdSpiCmd(0x2A);
    a[0] = (uint8_t)(X >> 8); a[1] = (uint8_t)X; a[2] = (uint8_t)((X + 1) >> 8); a[3] = (uint8_t)(X + 1);
    LcdSpiData(a, 4);
    LcdSpiCmd(0x2B);
    a[0] = (uint8_t)(Y >> 8); a[1] = (uint8_t)Y; a[2] = (uint8_t)((Y + 1) >> 8); a[3] = (uint8_t)(Y + 1);
    LcdSpiData(a, 4);
    LcdSpiCmd(0x2C);
    a[0] = (uint8_t)(Colour >> 8); a[1] = (uint8_t)Colour;
    LcdSpiData(a, 2);
    LCD_CS_HIGH();
}
#else
static void LcdFastPixel(uint16_t X, uint16_t Y, uint16_t Colour) { ILI9341_Draw_Pixel(X, Y, Colour); }   /* old path */
#define LcdWriteColumnFast(x, y1, y2, bytes, n)  ((void)0)   /* never reached: main.c LcdWriteColumn keeps the old HAL body */
#endif
/* LCDFAST_END ============================================================================================================ */
#endif /* LCD_FAST_H */
