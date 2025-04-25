################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/User/Status.cpp \
../Core/Src/User/UserInterface.cpp \
../Core/Src/User/main_cpp.cpp 

OBJS += \
./Core/Src/User/Status.o \
./Core/Src/User/UserInterface.o \
./Core/Src/User/main_cpp.o 

CPP_DEPS += \
./Core/Src/User/Status.d \
./Core/Src/User/UserInterface.d \
./Core/Src/User/main_cpp.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/User/%.o Core/Src/User/%.su Core/Src/User/%.cyclo: ../Core/Src/User/%.cpp Core/Src/User/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m33 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32U575xx -c -I../Core/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc -I../Drivers/STM32U5xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32U5xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-User

clean-Core-2f-Src-2f-User:
	-$(RM) ./Core/Src/User/Status.cyclo ./Core/Src/User/Status.d ./Core/Src/User/Status.o ./Core/Src/User/Status.su ./Core/Src/User/UserInterface.cyclo ./Core/Src/User/UserInterface.d ./Core/Src/User/UserInterface.o ./Core/Src/User/UserInterface.su ./Core/Src/User/main_cpp.cyclo ./Core/Src/User/main_cpp.d ./Core/Src/User/main_cpp.o ./Core/Src/User/main_cpp.su

.PHONY: clean-Core-2f-Src-2f-User

