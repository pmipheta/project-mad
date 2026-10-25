#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H
/* FreeRTOS config - STM32F767ZI: OscTask + InputTask (Core/Src/app). Hand-written, not CubeMX (kernel in ThirdParty/FreeRTOS). */
#include <stdint.h>
extern uint32_t SystemCoreClock;

/* SYSCLK = HSE 8MHz / M8 * N192 / P2 = 96 MHz (see SystemClock_Config); SystemCoreClock holds it at runtime */
#define configCPU_CLOCK_HZ                      ( SystemCoreClock )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 24 * 1024 ) )   /* OscTask + InputTask stacks (2 x 2048 words = 16 KB) + idle + TCBs + 2 mutexes */
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configQUEUE_REGISTRY_SIZE               0
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configENABLE_FPU                        1
#define configENABLE_MPU                        0
#define configENABLE_TRUSTZONE                  0
#define configRUN_FREERTOS_SECURE_ONLY          1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_TIMERS                        0
#define configUSE_EVENT_GROUPS                  1
#define configUSE_TRACE_FACILITY                0
#define configGENERATE_RUN_TIME_STATS           0

/* Cortex-M interrupt priorities (4 bits implemented on STM32F7; HAL_Init selects NVIC group 4) */
#define configPRIO_BITS                         4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5   /* ISRs at 5 (UART/DMA/ADC) may call *FromISR APIs later */
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

#define configASSERT( x ) if( ( x ) == 0 ) { portDISABLE_INTERRUPTS(); for( ;; ); }

#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/* Map port handlers onto the CMSIS vector-table names */
#define vPortSVCHandler     SVC_Handler
#define xPortPendSVHandler  PendSV_Handler
/* xPortSysTickHandler is called from SysTick_Handler (stm32f7xx_it.c) after HAL_IncTick() */
#endif
