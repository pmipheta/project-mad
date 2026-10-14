"""Compile real capture routine against HAL/cache/DWT fault injection."""
from pathlib import Path
import re, subprocess, os
root=Path(__file__).resolve().parents[1]
s=(root/'Core/Src/osc_acquisition.c').read_text()
out=(Path.home()/'.hermes/cache/scratch')/'osc_regression';out.mkdir(exist_ok=True)
def fn(name):
 m=re.search(r'^(?:static\s+)?(?:inline\s+)?(?:void|uint8_t|uint32_t)\s+'+name+r'\([^;]*?\)\s*\{',s,re.M);assert m,name
 i=s.index('{',m.start())+1;d=1
 while d:d+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[m.start():i]
pre=r'''#include <stdint.h>
#include <stdio.h>
#define ADC_RAW_MAX 65520u
#define ADC1 1
#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_ADC_ERROR_OVR 2
#define __DSB() ((void)0)
typedef struct{int Instance;uint32_t ErrorCode;} ADC_HandleTypeDef;
static ADC_HandleTypeDef hadc1={ADC1};
static struct {uint32_t CYCCNT;} dwt;
#define DWT (&dwt)
static uint16_t adc_raw_buf[ADC_RAW_MAX] __attribute__((aligned(32)));
static volatile uint8_t adc_done,adc_error;
static int mode,stops,clean,invalid,alignment,failed;
static uint32_t tick;
static uint32_t HAL_RCC_GetHCLKFreq(void){return 96000000;}
static uint32_t HAL_GetTick(void){return tick++;}
static void SCB_CleanInvalidateDCache_by_Addr(uint32_t*p,int n){clean++;if((uintptr_t)p%32||n%32||n>sizeof(adc_raw_buf))alignment++;}
static void SCB_InvalidateDCache_by_Addr(uint32_t*p,int n){invalid++;if((uintptr_t)p%32||n%32||n>sizeof(adc_raw_buf))alignment++;}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef*);
static int HAL_ADC_Start_DMA(ADC_HandleTypeDef*h,uint32_t*p,uint32_t n){
 (void)p;(void)n;dwt.CYCCNT+=144000;
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
body+=fn('AdcCaptureRaw')
main=r'''#define CHECK(x) do{if(!(x)){printf("FAIL line %d: %s\n",__LINE__,#x);failed++;}}while(0)
int main(void){uint32_t us=0;
 dwt.CYCCNT=UINT32_MAX-95999;
 CHECK(AdcCaptureRaw(2001,&us));CHECK(us==1500);CHECK(clean==1);CHECK(invalid==1);CHECK(alignment==0);
 CHECK(!AdcCaptureRaw(65536,&us));CHECK(!AdcCaptureRaw(0,&us));
 for(mode=1;mode<=4;mode++){int before=stops;CHECK(!AdcCaptureRaw(2001,&us));CHECK(stops>before);}
 mode=0;CHECK(AdcCaptureRaw(2001,&us));
 adc_done=1;adc_error=0;hadc1.ErrorCode=HAL_ADC_ERROR_OVR;HAL_ADC_ErrorCallback(&hadc1);CHECK(adc_error==0);
 adc_done=0;HAL_ADC_ErrorCallback(&hadc1);CHECK(adc_error==1);
 printf("DMA fault/cache/wrap failures=%d\n",failed);return failed?1:0;}
'''
(out/'dma.c').write_text(pre+body+'\n'+main)
subprocess.run(['clang','-O0',str(out/'dma.c'),'-o',str(out/'dma')],check=True)
raise SystemExit(subprocess.run([str(out/'dma')]).returncode)
