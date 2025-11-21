################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MCP23S17_/mcp23s17.c 

OBJS += \
./MCP23S17_/mcp23s17.o 

C_DEPS += \
./MCP23S17_/mcp23s17.d 


# Each subdirectory must supply rules for building sources it contributes
MCP23S17_/%.o MCP23S17_/%.su MCP23S17_/%.cyclo: ../MCP23S17_/%.c MCP23S17_/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"/home/mantoumben/git/ESE_TP_Synthese_Kenny_Mantou/TPAutomobile/shell" -I"/home/mantoumben/git/ESE_TP_Synthese_Kenny_Mantou/TPAutomobile/MCP23S17_" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MCP23S17_

clean-MCP23S17_:
	-$(RM) ./MCP23S17_/mcp23s17.cyclo ./MCP23S17_/mcp23s17.d ./MCP23S17_/mcp23s17.o ./MCP23S17_/mcp23s17.su

.PHONY: clean-MCP23S17_

