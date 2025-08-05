################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Logging.cpp \
../System/System.cpp \
../System/Thread.cpp 

OBJS += \
./System/Logging.o \
./System/System.o \
./System/Thread.o 

CPP_DEPS += \
./System/Logging.d \
./System/System.d \
./System/Thread.d 


# Each subdirectory must supply rules for building sources it contributes
System/%.o System/%.su System/%.cyclo: ../System/%.cpp System/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System

clean-System:
	-$(RM) ./System/Logging.cyclo ./System/Logging.d ./System/Logging.o ./System/Logging.su ./System/System.cyclo ./System/System.d ./System/System.o ./System/System.su ./System/Thread.cyclo ./System/Thread.d ./System/Thread.o ./System/Thread.su

.PHONY: clean-System

