"""Compile real capture routine against HAL/cache/DWT fault injection."""
from app_src import source as app_source   # the app modules as one text (see app_src.py)
from pathlib import Path
import re, subprocess, os
root=Path(__file__).resolve().parents[1]
s=app_source()
out=Path(__import__('tempfile').gettempdir())/'osc_regression';out.mkdir(exist_ok=True)
def fn(name):
 m=re.search(r'^(?:static\s+)?(?:inline\s+)?(?:void|uint8_t|uint32_t)\s+'+name+r'\([^;]*?\)\s*\{',s,re.M);assert m,name
 i=s.index('{',m.start())+1;d=1
 while d:d+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[m.start():i]
pre=r'''#include <stdint.h>
#include <stdio.h>
#define ADC_RAW_MAX 65520u
typedef struct{uint32_t CR1,CR2;} ADC_TypeDef;static ADC_TypeDef adc1_regs;
#define ADC1 (&adc1_regs)
#define ADC_CR2_CONT 0x2u
#define ADC_CR1_OVRIE 0x4000000u
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_ADC_ERROR_OVR 2
#define __DSB() ((void)0)
typedef struct{ADC_TypeDef*Instance;uint32_t ErrorCode;} ADC_HandleTypeDef;
static ADC_HandleTypeDef hadc1={ADC1};
static struct {uint32_t CYCCNT;} dwt;
#define DWT (&dwt)
static uint16_t adc_raw_buf[ADC_RAW_MAX] __attribute__((aligned(32)));
static volatile uint8_t adc_done,adc_error,slow_busy,adc_async;static int busy_seen;
static uint8_t adc_tri_on=0;static uint8_t AdcCaptureTriple(uint32_t n,uint32_t*t){(void)n;(void)t;return 0;}
static int mode,stops,clean,invalid,alignment,failed;
static uint32_t tick;
static uint32_t HAL_RCC_GetHCLKFreq(void){return 96000000;}
static uint32_t HAL_GetTick(void){return tick++;}
static void SCB_CleanInvalidateDCache_by_Addr(uint32_t*p,int n){clean++;if((uintptr_t)p%32||n%32||n>sizeof(adc_raw_buf))alignment++;}
static void SCB_InvalidateDCache_by_Addr(uint32_t*p,int n){invalid++;if((uintptr_t)p%32||n%32||n>sizeof(adc_raw_buf))alignment++;}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*);
static int HAL_ADC_Start_DMA(ADC_HandleTypeDef*h,uint32_t*p,uint32_t n){
 (void)p;(void)n;dwt.CYCCNT+=144000;if(slow_busy)busy_seen++;
 if(mode==1)return HAL_ERROR;
 if(mode==2)return HAL_OK;
 if(mode==3){HAL_ADC_ErrorCallback(h);return HAL_OK;}
 adc_done=1;return HAL_OK;
}
static int HAL_ADC_Stop_DMA(ADC_HandleTypeDef*h){(void)h;stops++;return mode==4?HAL_ERROR:HAL_OK;}
'''
body=''
if 'static inline uint32_t AdcMicros(' in s:body+=fn('AdcMicros')
if 'static uint32_t AdcElapsedUs(' in s:body+=fn('AdcElapsedUs')
if 'void HAL_ADC_ErrorCallback(' in s:body+=fn('HAL_ADC_ErrorCallback')
else:body+='void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*h){(void)h;}\n'
body+=fn('HAL_ADC_ConvCpltCallback');body+=fn('AdcCaptureRawInner');body+=fn('AdcCaptureRaw');body+=fn('AdcCaptureStart');body+=fn('AdcCaptureCollect')
main=r'''#define CHECK(x) do{if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);failed++;}}while(0)
int main(void){uint32_t us=0,t0=0;
 dwt.CYCCNT=UINT32_MAX-95999;
 CHECK(AdcCaptureRaw(2001,&us));CHECK(busy_seen>0);CHECK(slow_busy==0);CHECK(us==1500);CHECK(clean==1);CHECK(invalid==1);CHECK(alignment==0);
 CHECK(!AdcCaptureRaw(65536,&us));CHECK(!AdcCaptureRaw(0,&us));
 for(mode=1;mode<=4;mode++){int before=stops;CHECK(!AdcCaptureRaw(2001,&us));CHECK(stops>before);}
 mode=0;CHECK(AdcCaptureRaw(2001,&us));
 adc_done=1;adc_error=0;hadc1.ErrorCode=HAL_ADC_ERROR_OVR;HAL_ADC_ErrorCallback(&hadc1);CHECK(adc_error==0);
 adc_done=0;HAL_ADC_ErrorCallback(&hadc1);CHECK(adc_error==1);
 /* ASYNC capture: Start returns at once and keeps slow_busy; the completion callback stops the continuous conversions; Collect restores everything */
 mode=0;adc1_regs.CR2=0;adc1_regs.CR1=ADC_CR1_OVRIE;slow_busy=0;adc_async=0;adc_done=0;adc_error=0;invalid=0;clean=0;
 CHECK(AdcCaptureStart(2001,&t0));CHECK(slow_busy==1);CHECK(adc_async==1);CHECK(adc1_regs.CR2&ADC_CR2_CONT);CHECK(clean==1);
 HAL_ADC_ConvCpltCallback(&hadc1);CHECK(!(adc1_regs.CR2&ADC_CR2_CONT));CHECK(!(adc1_regs.CR1&ADC_CR1_OVRIE));
 { int st=stops; CHECK(AdcCaptureCollect(2001,t0,&us));CHECK(stops==st+1);CHECK(slow_busy==0);CHECK(adc_async==0);CHECK(adc1_regs.CR2&ADC_CR2_CONT);CHECK(invalid==1);CHECK(alignment==0); }
 adc1_regs.CR2=ADC_CR2_CONT;adc_async=0;HAL_ADC_ConvCpltCallback(&hadc1);CHECK(adc1_regs.CR2&ADC_CR2_CONT);   /* a blocking capture is never touched by the callback */
 mode=1;CHECK(!AdcCaptureStart(2001,&t0));CHECK(slow_busy==0);CHECK(adc_async==0);CHECK(adc1_regs.CR2&ADC_CR2_CONT);
 mode=2;adc_done=0;CHECK(AdcCaptureStart(2001,&t0));CHECK(slow_busy==1);CHECK(!AdcCaptureCollect(2001,t0,&us));CHECK(slow_busy==0);CHECK(adc_async==0);   /* never finished: timeout, busy flag released */
 mode=3;CHECK(AdcCaptureStart(2001,&t0));CHECK(!AdcCaptureCollect(2001,t0,&us));CHECK(slow_busy==0);
 mode=4;adc_done=0;CHECK(AdcCaptureStart(2001,&t0));adc_done=1;CHECK(!AdcCaptureCollect(2001,t0,&us));CHECK(slow_busy==0);CHECK(adc_async==0);mode=0;
 CHECK(!AdcCaptureStart(0,&t0));CHECK(!AdcCaptureStart(65536,&t0));adc_tri_on=1;CHECK(!AdcCaptureStart(2001,&t0));adc_tri_on=0;CHECK(slow_busy==0);
 printf("DMA fault/cache/wrap failures=%d\n",failed);return failed?1:0;}
'''
(out/'dma.c').write_text(pre+body+'\n'+main)
subprocess.run(['clang','-O0',str(out/'dma.c'),'-o',str(out/'dma')],check=True)
raise SystemExit(subprocess.run([str(out/'dma')]).returncode)
