################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/Control/Status.cpp \
../Core/Src/Control/UserInterface.cpp 

OBJS += \
./Core/Src/Control/Status.o \
./Core/Src/Control/UserInterface.o 

CPP_DEPS += \
./Core/Src/Control/Status.d \
./Core/Src/Control/UserInterface.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Control/%.o Core/Src/Control/%.su Core/Src/Control/%.cyclo: ../Core/Src/Control/%.cpp Core/Src/Control/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Control

clean-Core-2f-Src-2f-Control:
	-$(RM) ./Core/Src/Control/Status.cyclo ./Core/Src/Control/Status.d ./Core/Src/Control/Status.o ./Core/Src/Control/Status.su ./Core/Src/Control/UserInterface.cyclo ./Core/Src/Control/UserInterface.d ./Core/Src/Control/UserInterface.o ./Core/Src/Control/UserInterface.su

.PHONY: clean-Core-2f-Src-2f-Control

