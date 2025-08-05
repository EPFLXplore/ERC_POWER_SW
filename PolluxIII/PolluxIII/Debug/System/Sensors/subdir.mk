################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Sensors/IMUThread.cpp \
../System/Sensors/PowerMonitors.cpp \
../System/Sensors/PowerThread.cpp 

OBJS += \
./System/Sensors/IMUThread.o \
./System/Sensors/PowerMonitors.o \
./System/Sensors/PowerThread.o 

CPP_DEPS += \
./System/Sensors/IMUThread.d \
./System/Sensors/PowerMonitors.d \
./System/Sensors/PowerThread.d 


# Each subdirectory must supply rules for building sources it contributes
System/Sensors/%.o System/Sensors/%.su System/Sensors/%.cyclo: ../System/Sensors/%.cpp System/Sensors/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Sensors

clean-System-2f-Sensors:
	-$(RM) ./System/Sensors/IMUThread.cyclo ./System/Sensors/IMUThread.d ./System/Sensors/IMUThread.o ./System/Sensors/IMUThread.su ./System/Sensors/PowerMonitors.cyclo ./System/Sensors/PowerMonitors.d ./System/Sensors/PowerMonitors.o ./System/Sensors/PowerMonitors.su ./System/Sensors/PowerThread.cyclo ./System/Sensors/PowerThread.d ./System/Sensors/PowerThread.o ./System/Sensors/PowerThread.su

.PHONY: clean-System-2f-Sensors

