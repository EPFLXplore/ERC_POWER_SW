################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Debug/Console.cpp \
../System/Debug/Debug.cpp \
../System/Debug/Monitor.cpp \
../System/Debug/Profiler.cpp \
../System/Debug/Shell.cpp \
../System/Debug/Terminal.cpp 

OBJS += \
./System/Debug/Console.o \
./System/Debug/Debug.o \
./System/Debug/Monitor.o \
./System/Debug/Profiler.o \
./System/Debug/Shell.o \
./System/Debug/Terminal.o 

CPP_DEPS += \
./System/Debug/Console.d \
./System/Debug/Debug.d \
./System/Debug/Monitor.d \
./System/Debug/Profiler.d \
./System/Debug/Shell.d \
./System/Debug/Terminal.d 


# Each subdirectory must supply rules for building sources it contributes
System/Debug/%.o System/Debug/%.su System/Debug/%.cyclo: ../System/Debug/%.cpp System/Debug/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Debug

clean-System-2f-Debug:
	-$(RM) ./System/Debug/Console.cyclo ./System/Debug/Console.d ./System/Debug/Console.o ./System/Debug/Console.su ./System/Debug/Debug.cyclo ./System/Debug/Debug.d ./System/Debug/Debug.o ./System/Debug/Debug.su ./System/Debug/Monitor.cyclo ./System/Debug/Monitor.d ./System/Debug/Monitor.o ./System/Debug/Monitor.su ./System/Debug/Profiler.cyclo ./System/Debug/Profiler.d ./System/Debug/Profiler.o ./System/Debug/Profiler.su ./System/Debug/Shell.cyclo ./System/Debug/Shell.d ./System/Debug/Shell.o ./System/Debug/Shell.su ./System/Debug/Terminal.cyclo ./System/Debug/Terminal.d ./System/Debug/Terminal.o ./System/Debug/Terminal.su

.PHONY: clean-System-2f-Debug

