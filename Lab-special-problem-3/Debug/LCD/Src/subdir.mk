################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD/Src/gpio.c \
../LCD/Src/main.c \
../LCD/Src/rng.c \
../LCD/Src/spi.c \
../LCD/Src/stm32f7xx_hal_msp.c \
../LCD/Src/stm32f7xx_it.c \
../LCD/Src/tim.c \
../LCD/Src/usart.c 

OBJS += \
./LCD/Src/gpio.o \
./LCD/Src/main.o \
./LCD/Src/rng.o \
./LCD/Src/spi.o \
./LCD/Src/stm32f7xx_hal_msp.o \
./LCD/Src/stm32f7xx_it.o \
./LCD/Src/tim.o \
./LCD/Src/usart.o 

C_DEPS += \
./LCD/Src/gpio.d \
./LCD/Src/main.d \
./LCD/Src/rng.d \
./LCD/Src/spi.d \
./LCD/Src/stm32f7xx_hal_msp.d \
./LCD/Src/stm32f7xx_it.d \
./LCD/Src/tim.d \
./LCD/Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
LCD/Src/%.o LCD/Src/%.su LCD/Src/%.cyclo: ../LCD/Src/%.c LCD/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/microcon/Lab-special-problem-3/LCD/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD-2f-Src

clean-LCD-2f-Src:
	-$(RM) ./LCD/Src/gpio.cyclo ./LCD/Src/gpio.d ./LCD/Src/gpio.o ./LCD/Src/gpio.su ./LCD/Src/main.cyclo ./LCD/Src/main.d ./LCD/Src/main.o ./LCD/Src/main.su ./LCD/Src/rng.cyclo ./LCD/Src/rng.d ./LCD/Src/rng.o ./LCD/Src/rng.su ./LCD/Src/spi.cyclo ./LCD/Src/spi.d ./LCD/Src/spi.o ./LCD/Src/spi.su ./LCD/Src/stm32f7xx_hal_msp.cyclo ./LCD/Src/stm32f7xx_hal_msp.d ./LCD/Src/stm32f7xx_hal_msp.o ./LCD/Src/stm32f7xx_hal_msp.su ./LCD/Src/stm32f7xx_it.cyclo ./LCD/Src/stm32f7xx_it.d ./LCD/Src/stm32f7xx_it.o ./LCD/Src/stm32f7xx_it.su ./LCD/Src/tim.cyclo ./LCD/Src/tim.d ./LCD/Src/tim.o ./LCD/Src/tim.su ./LCD/Src/usart.cyclo ./LCD/Src/usart.d ./LCD/Src/usart.o ./LCD/Src/usart.su

.PHONY: clean-LCD-2f-Src

