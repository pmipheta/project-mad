################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD/Src/ILI9341/ILI9341_GFX.c \
../LCD/Src/ILI9341/ILI9341_STM32_Driver.c \
../LCD/Src/ILI9341/ILI9341_Touchscreen.c 

OBJS += \
./LCD/Src/ILI9341/ILI9341_GFX.o \
./LCD/Src/ILI9341/ILI9341_STM32_Driver.o \
./LCD/Src/ILI9341/ILI9341_Touchscreen.o 

C_DEPS += \
./LCD/Src/ILI9341/ILI9341_GFX.d \
./LCD/Src/ILI9341/ILI9341_STM32_Driver.d \
./LCD/Src/ILI9341/ILI9341_Touchscreen.d 


# Each subdirectory must supply rules for building sources it contributes
LCD/Src/ILI9341/%.o LCD/Src/ILI9341/%.su LCD/Src/ILI9341/%.cyclo: ../LCD/Src/ILI9341/%.c LCD/Src/ILI9341/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/microcon/Lab-special-problem-3/LCD/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD-2f-Src-2f-ILI9341

clean-LCD-2f-Src-2f-ILI9341:
	-$(RM) ./LCD/Src/ILI9341/ILI9341_GFX.cyclo ./LCD/Src/ILI9341/ILI9341_GFX.d ./LCD/Src/ILI9341/ILI9341_GFX.o ./LCD/Src/ILI9341/ILI9341_GFX.su ./LCD/Src/ILI9341/ILI9341_STM32_Driver.cyclo ./LCD/Src/ILI9341/ILI9341_STM32_Driver.d ./LCD/Src/ILI9341/ILI9341_STM32_Driver.o ./LCD/Src/ILI9341/ILI9341_STM32_Driver.su ./LCD/Src/ILI9341/ILI9341_Touchscreen.cyclo ./LCD/Src/ILI9341/ILI9341_Touchscreen.d ./LCD/Src/ILI9341/ILI9341_Touchscreen.o ./LCD/Src/ILI9341/ILI9341_Touchscreen.su

.PHONY: clean-LCD-2f-Src-2f-ILI9341

