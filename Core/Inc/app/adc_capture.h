#ifndef APP_ADC_CAPTURE_H
#define APP_ADC_CAPTURE_H

#include "app/app_config.h"

extern uint16_t adc_raw_buf[ADC_RAW_MAX];
extern volatile uint16_t dbg_adc_min;
extern volatile uint16_t dbg_adc_max;
extern volatile uint32_t dbg_adc_frames;
extern volatile uint8_t dbg_adc_ok;
uint32_t AdcElapsedUs(uint32_t start);
extern volatile uint8_t slow_busy;
extern volatile uint8_t slow_adc3_dirty;
extern uint8_t adc_tri_ready;
void AdcInputInit(void);
void AdcAbortPending(void);
void AdcPrestart(void);
void AdcCaptureFrame(void);

#endif
