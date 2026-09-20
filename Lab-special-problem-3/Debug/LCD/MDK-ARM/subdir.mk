################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../LCD/MDK-ARM/startup_stm32f746xx.s 

OBJS += \
./LCD/MDK-ARM/startup_stm32f746xx.o 

S_DEPS += \
./LCD/MDK-ARM/startup_stm32f746xx.d 


# Each subdirectory must supply rules for building sources it contributes
LCD/MDK-ARM/%.o: ../LCD/MDK-ARM/%.s LCD/MDK-ARM/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-LCD-2f-MDK-2d-ARM

clean-LCD-2f-MDK-2d-ARM:
	-$(RM) ./LCD/MDK-ARM/startup_stm32f746xx.d ./LCD/MDK-ARM/startup_stm32f746xx.o

.PHONY: clean-LCD-2f-MDK-2d-ARM

