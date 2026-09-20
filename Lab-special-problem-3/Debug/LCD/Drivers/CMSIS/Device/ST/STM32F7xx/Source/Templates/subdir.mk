################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.c 

OBJS += \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.o 

C_DEPS += \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.d 


# Each subdirectory must supply rules for building sources it contributes
LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/%.o LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/%.su LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/%.cyclo: ../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/%.c LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/microcon/Lab-special-problem-3/LCD/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD-2f-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F7xx-2f-Source-2f-Templates

clean-LCD-2f-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F7xx-2f-Source-2f-Templates:
	-$(RM) ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.cyclo ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/system_stm32f7xx.su

.PHONY: clean-LCD-2f-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F7xx-2f-Source-2f-Templates

