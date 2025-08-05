################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Libraries/EmbeddedGL/Components/Box.cpp \
../System/Libraries/EmbeddedGL/Components/Component.cpp \
../System/Libraries/EmbeddedGL/Components/Gauge.cpp \
../System/Libraries/EmbeddedGL/Components/Text.cpp 

OBJS += \
./System/Libraries/EmbeddedGL/Components/Box.o \
./System/Libraries/EmbeddedGL/Components/Component.o \
./System/Libraries/EmbeddedGL/Components/Gauge.o \
./System/Libraries/EmbeddedGL/Components/Text.o 

CPP_DEPS += \
./System/Libraries/EmbeddedGL/Components/Box.d \
./System/Libraries/EmbeddedGL/Components/Component.d \
./System/Libraries/EmbeddedGL/Components/Gauge.d \
./System/Libraries/EmbeddedGL/Components/Text.d 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/EmbeddedGL/Components/%.o System/Libraries/EmbeddedGL/Components/%.su System/Libraries/EmbeddedGL/Components/%.cyclo: ../System/Libraries/EmbeddedGL/Components/%.cpp System/Libraries/EmbeddedGL/Components/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-EmbeddedGL-2f-Components

clean-System-2f-Libraries-2f-EmbeddedGL-2f-Components:
	-$(RM) ./System/Libraries/EmbeddedGL/Components/Box.cyclo ./System/Libraries/EmbeddedGL/Components/Box.d ./System/Libraries/EmbeddedGL/Components/Box.o ./System/Libraries/EmbeddedGL/Components/Box.su ./System/Libraries/EmbeddedGL/Components/Component.cyclo ./System/Libraries/EmbeddedGL/Components/Component.d ./System/Libraries/EmbeddedGL/Components/Component.o ./System/Libraries/EmbeddedGL/Components/Component.su ./System/Libraries/EmbeddedGL/Components/Gauge.cyclo ./System/Libraries/EmbeddedGL/Components/Gauge.d ./System/Libraries/EmbeddedGL/Components/Gauge.o ./System/Libraries/EmbeddedGL/Components/Gauge.su ./System/Libraries/EmbeddedGL/Components/Text.cyclo ./System/Libraries/EmbeddedGL/Components/Text.d ./System/Libraries/EmbeddedGL/Components/Text.o ./System/Libraries/EmbeddedGL/Components/Text.su

.PHONY: clean-System-2f-Libraries-2f-EmbeddedGL-2f-Components

