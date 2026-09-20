################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f745xx.s \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f746xx.s \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f756xx.s \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f765xx.s \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f767xx.s \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f769xx.s \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f777xx.s \
../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f779xx.s 

OBJS += \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f745xx.o \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f746xx.o \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f756xx.o \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f765xx.o \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f767xx.o \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f769xx.o \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f777xx.o \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f779xx.o 

S_DEPS += \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f745xx.d \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f746xx.d \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f756xx.d \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f765xx.d \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f767xx.d \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f769xx.d \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f777xx.d \
./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f779xx.d 


# Each subdirectory must supply rules for building sources it contributes
LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/%.o: ../LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/%.s LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-LCD-2f-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F7xx-2f-Source-2f-Templates-2f-arm

clean-LCD-2f-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F7xx-2f-Source-2f-Templates-2f-arm:
	-$(RM) ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f745xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f745xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f746xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f746xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f756xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f756xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f765xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f765xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f767xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f767xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f769xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f769xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f777xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f777xx.o ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f779xx.d ./LCD/Drivers/CMSIS/Device/ST/STM32F7xx/Source/Templates/arm/startup_stm32f779xx.o

.PHONY: clean-LCD-2f-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32F7xx-2f-Source-2f-Templates-2f-arm

