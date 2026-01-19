################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/B_MOTOR/B_MOTOR.c 

OBJS += \
./Drivers/B_MOTOR/B_MOTOR.o 

C_DEPS += \
./Drivers/B_MOTOR/B_MOTOR.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/B_MOTOR/%.o Drivers/B_MOTOR/%.su Drivers/B_MOTOR/%.cyclo: ../Drivers/B_MOTOR/%.c Drivers/B_MOTOR/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F439xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/B_MOTOR -I../Drivers/STEP_MOTOR -I../Drivers/UART_FILE -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-B_MOTOR

clean-Drivers-2f-B_MOTOR:
	-$(RM) ./Drivers/B_MOTOR/B_MOTOR.cyclo ./Drivers/B_MOTOR/B_MOTOR.d ./Drivers/B_MOTOR/B_MOTOR.o ./Drivers/B_MOTOR/B_MOTOR.su

.PHONY: clean-Drivers-2f-B_MOTOR

