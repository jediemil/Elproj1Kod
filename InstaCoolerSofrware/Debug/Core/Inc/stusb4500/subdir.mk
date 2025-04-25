################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/stusb4500/stusb4500.c \
../Core/Inc/stusb4500/stusb4500_nvm.c 

C_DEPS += \
./Core/Inc/stusb4500/stusb4500.d \
./Core/Inc/stusb4500/stusb4500_nvm.d 

OBJS += \
./Core/Inc/stusb4500/stusb4500.o \
./Core/Inc/stusb4500/stusb4500_nvm.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/stusb4500/%.o Core/Inc/stusb4500/%.su Core/Inc/stusb4500/%.cyclo: ../Core/Inc/stusb4500/%.c Core/Inc/stusb4500/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-stusb4500

clean-Core-2f-Inc-2f-stusb4500:
	-$(RM) ./Core/Inc/stusb4500/stusb4500.cyclo ./Core/Inc/stusb4500/stusb4500.d ./Core/Inc/stusb4500/stusb4500.o ./Core/Inc/stusb4500/stusb4500.su ./Core/Inc/stusb4500/stusb4500_nvm.cyclo ./Core/Inc/stusb4500/stusb4500_nvm.d ./Core/Inc/stusb4500/stusb4500_nvm.o ./Core/Inc/stusb4500/stusb4500_nvm.su

.PHONY: clean-Core-2f-Inc-2f-stusb4500

