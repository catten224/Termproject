################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/UART_FILE/UART.c 

OBJS += \
./Drivers/UART_FILE/UART.o 

C_DEPS += \
./Drivers/UART_FILE/UART.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/UART_FILE/%.o Drivers/UART_FILE/%.su Drivers/UART_FILE/%.cyclo: ../Drivers/UART_FILE/%.c Drivers/UART_FILE/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F439xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/B_MOTOR -I../Drivers/STEP_MOTOR -I../Drivers/UART_FILE -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-UART_FILE

clean-Drivers-2f-UART_FILE:
	-$(RM) ./Drivers/UART_FILE/UART.cyclo ./Drivers/UART_FILE/UART.d ./Drivers/UART_FILE/UART.o ./Drivers/UART_FILE/UART.su

.PHONY: clean-Drivers-2f-UART_FILE

