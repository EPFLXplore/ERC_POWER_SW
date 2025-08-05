################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Libraries/Utils/ExceptionTracker.cpp \
../System/Libraries/Utils/Operators.cpp 

OBJS += \
./System/Libraries/Utils/ExceptionTracker.o \
./System/Libraries/Utils/Operators.o 

CPP_DEPS += \
./System/Libraries/Utils/ExceptionTracker.d \
./System/Libraries/Utils/Operators.d 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/Utils/%.o System/Libraries/Utils/%.su System/Libraries/Utils/%.cyclo: ../System/Libraries/Utils/%.cpp System/Libraries/Utils/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-Utils

clean-System-2f-Libraries-2f-Utils:
	-$(RM) ./System/Libraries/Utils/ExceptionTracker.cyclo ./System/Libraries/Utils/ExceptionTracker.d ./System/Libraries/Utils/ExceptionTracker.o ./System/Libraries/Utils/ExceptionTracker.su ./System/Libraries/Utils/Operators.cyclo ./System/Libraries/Utils/Operators.d ./System/Libraries/Utils/Operators.o ./System/Libraries/Utils/Operators.su

.PHONY: clean-System-2f-Libraries-2f-Utils

