################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/audio/RCFilter.c \
../Core/Src/audio/audio.c 

OBJS += \
./Core/Src/audio/RCFilter.o \
./Core/Src/audio/audio.o 

C_DEPS += \
./Core/Src/audio/RCFilter.d \
./Core/Src/audio/audio.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/audio/%.o Core/Src/audio/%.su Core/Src/audio/%.cyclo: ../Core/Src/audio/%.c Core/Src/audio/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/mantoumben/git/ESE_TP_Synthese_Kenny_Mantou/TPAutomobile/shell" -I"/home/mantoumben/git/ESE_TP_Synthese_Kenny_Mantou/TPAutomobile/sgtl5000" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-audio

clean-Core-2f-Src-2f-audio:
	-$(RM) ./Core/Src/audio/RCFilter.cyclo ./Core/Src/audio/RCFilter.d ./Core/Src/audio/RCFilter.o ./Core/Src/audio/RCFilter.su ./Core/Src/audio/audio.cyclo ./Core/Src/audio/audio.d ./Core/Src/audio/audio.o ./Core/Src/audio/audio.su

.PHONY: clean-Core-2f-Src-2f-audio

