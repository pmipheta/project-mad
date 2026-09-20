/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_WIDTH       320
#define DMA_BUFFER_SIZE     960   // 320 * 3 samples (รองรับ Double buffering)
#define FILTER_WINDOW_SIZE  5     // ขนาด Window ของ Moving Average (เช่น 3 หรือ 5 จุด)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
/* บัฟเฟอร์รับข้อมูลดิบจาก DMA Multi-Mode */
ALIGN_32BYTES(uint32_t adc_dma_buffer[DMA_BUFFER_SIZE]);
/* บัฟเฟอร์สำหรับเก็บจุดที่เรียงตามลำดับเวลาและนำไปกรอง */
uint16_t raw_waveform[DISPLAY_WIDTH];
uint16_t filtered_waveform[DISPLAY_WIDTH];

/* ตัวแปร Global 2 ตัวสำหรับอ่านค่าก่อนและหลัง Filter ดูผ่าน Live Expressions */
volatile uint16_t debug_adc_raw = 0;
volatile uint16_t debug_adc_filtered = 0;

/* Flag แจ้งเตือนว่ามีข้อมูลพร้อมประมวลผล */
volatile bool buffer_ready_flag = false;
volatile uint16_t *current_processing_ptr = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void Apply_MovingAverage(const uint16_t *input, uint16_t *output, uint16_t length, uint8_t window_size);
void Draw_Oscilloscope_Waveform(const uint16_t *waveform_data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  ฟังก์ชัน Moving Average Filter สำหรับกำจัด Noise ความถี่สูงในรูปคลื่น
  */
void Apply_MovingAverage(const uint16_t *input, uint16_t *output, uint16_t length, uint8_t window_size)
{
    if (window_size < 2) {
        memcpy(output, input, length * sizeof(uint16_t));
        return;
    }

    uint8_t half_window = window_size / 2;

    for (int i = 0; i < length; i++) {
        uint32_t sum = 0;
        int count = 0;

        for (int j = -half_window; j <= half_window; j++) {
            int idx = i + j;
            if (idx >= 0 && idx < length) {
                sum += input[idx];
                count++;
            }
        }
        output[i] = (uint16_t)(sum / count);
    }
}

/**
  * @brief  ฟังก์ชันจำลองสำหรับส่งข้อมูลไปวาดบนจอ LCD 240x320
  */
void Draw_Oscilloscope_Waveform(const uint16_t *waveform_data, uint16_t length)
{
    // ตัวอย่างการ Mapping: ADC 12-bit (0-4095) แปลงลงความสูงหน้าจอ (เช่น สูง 240 พิกเซล)
    // สำหรับนำไปเชื่อมต่อกับฟังก์ชัน LCD_DrawPixel หรือ DMA2D ต่อไป:
    /*
    for (uint16_t x = 0; x < length; x++) {
        uint16_t y = (uint16_t)(((4095 - waveform_data[x]) * 239) / 4095);
        // LCD_DrawPixel(x, y, LCD_COLOR_GREEN);
    }
    */
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start(&hadc3);
  HAL_ADC_Start(&hadc2);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)adc_dma_buffer, DMA_BUFFER_SIZE); //[cite: 1]
    // เริ่มต้น Timer TRGO กระตุ้นการ Sampling
  HAL_TIM_Base_Start(&htim2); // เปลี่ยน htim2 เป็นตัวที่เลือกใช้
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (buffer_ready_flag)
	      {
	          buffer_ready_flag = false;

	          // 1. นำข้อมูล Interleaved 320 จุดจากชุดที่พร้อมมาใส่ใน raw_waveform
	          for (uint16_t i = 0; i < DISPLAY_WIDTH; i++) {
	              raw_waveform[i] = current_processing_ptr[i];
	          }

	          // 2. กรองข้อมูลลด Noise ด้วย Moving Average
	          Apply_MovingAverage(raw_waveform, filtered_waveform, DISPLAY_WIDTH, FILTER_WINDOW_SIZE);

	          // 3. อัปเดตตัวแปร Global สุ่มหยิบตำแหน่งตรงกลางเฟรมเพื่อดูใน Debug Expressions
	          debug_adc_raw = raw_waveform[DISPLAY_WIDTH / 2];
	          debug_adc_filtered = filtered_waveform[DISPLAY_WIDTH / 2];

	          // 4. ส่งข้อมูลรูปคลื่นไปพล็อตลงจอ LCD
	          Draw_Oscilloscope_Waveform(filtered_waveform, DISPLAY_WIDTH);
	      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// เมื่อ DMA แปลงข้อมูลครบครึ่งแรก
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_dma_buffer[0], (DMA_BUFFER_SIZE / 2) * sizeof(uint32_t));
        // แปลงกลับเป็น uint16_t* เพื่อให้ลูปใน while(1) ดึงค่าแบบสลับ ADC1, ADC2 ได้ตามปกติ
        current_processing_ptr = (uint16_t *)&adc_dma_buffer[0];
        buffer_ready_flag = true;
    }
}

// เมื่อ DMA แปลงข้อมูลครบครึ่งหลัง
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        SCB_InvalidateDCache_by_Addr((uint32_t *)&adc_dma_buffer[DMA_BUFFER_SIZE / 2], (DMA_BUFFER_SIZE / 2) * sizeof(uint32_t));
        current_processing_ptr = (uint16_t *)&adc_dma_buffer[DMA_BUFFER_SIZE / 2];
        buffer_ready_flag = true;
    }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
