################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.c \
../LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.c 

OBJS += \
./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.o \
./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.o 

C_DEPS += \
./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.d \
./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.d 


# Each subdirectory must supply rules for building sources it contributes
LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.o LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.su LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.cyclo: ../LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/%.c LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F767xx -c -I../Core/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I"D:/microcon/Lab-special-problem-3/LCD/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD-2f-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-CommonTables

clean-LCD-2f-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-CommonTables:
	-$(RM) ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.cyclo ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.d ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.o ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_common_tables.su ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.cyclo ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.d ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.o ./LCD/Drivers/CMSIS/DSP_Lib/Source/CommonTables/arm_const_structs.su

.PHONY: clean-LCD-2f-Drivers-2f-CMSIS-2f-DSP_Lib-2f-Source-2f-CommonTables

