################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Libraries/RoCo/Src/BufferedIODriver.cpp \
../System/Libraries/RoCo/Src/EspressifUARTDriver.cpp \
../System/Libraries/RoCo/Src/IOBus.cpp \
../System/Libraries/RoCo/Src/LWIPClientIO.cpp \
../System/Libraries/RoCo/Src/LoopbackDriver.cpp \
../System/Libraries/RoCo/Src/MessageBus.cpp \
../System/Libraries/RoCo/Src/NetworkBus.cpp \
../System/Libraries/RoCo/Src/NetworkClientIO.cpp \
../System/Libraries/RoCo/Src/NetworkServerIO.cpp \
../System/Libraries/RoCo/Src/PowerBus.cpp \
../System/Libraries/RoCo/Src/RadioBus.cpp \
../System/Libraries/RoCo/Src/STMUARTDriver.cpp \
../System/Libraries/RoCo/Src/UDevDriver.cpp \
../System/Libraries/RoCo/Src/ZephyrUARTDriver.cpp 

OBJS += \
./System/Libraries/RoCo/Src/BufferedIODriver.o \
./System/Libraries/RoCo/Src/EspressifUARTDriver.o \
./System/Libraries/RoCo/Src/IOBus.o \
./System/Libraries/RoCo/Src/LWIPClientIO.o \
./System/Libraries/RoCo/Src/LoopbackDriver.o \
./System/Libraries/RoCo/Src/MessageBus.o \
./System/Libraries/RoCo/Src/NetworkBus.o \
./System/Libraries/RoCo/Src/NetworkClientIO.o \
./System/Libraries/RoCo/Src/NetworkServerIO.o \
./System/Libraries/RoCo/Src/PowerBus.o \
./System/Libraries/RoCo/Src/RadioBus.o \
./System/Libraries/RoCo/Src/STMUARTDriver.o \
./System/Libraries/RoCo/Src/UDevDriver.o \
./System/Libraries/RoCo/Src/ZephyrUARTDriver.o 

CPP_DEPS += \
./System/Libraries/RoCo/Src/BufferedIODriver.d \
./System/Libraries/RoCo/Src/EspressifUARTDriver.d \
./System/Libraries/RoCo/Src/IOBus.d \
./System/Libraries/RoCo/Src/LWIPClientIO.d \
./System/Libraries/RoCo/Src/LoopbackDriver.d \
./System/Libraries/RoCo/Src/MessageBus.d \
./System/Libraries/RoCo/Src/NetworkBus.d \
./System/Libraries/RoCo/Src/NetworkClientIO.d \
./System/Libraries/RoCo/Src/NetworkServerIO.d \
./System/Libraries/RoCo/Src/PowerBus.d \
./System/Libraries/RoCo/Src/RadioBus.d \
./System/Libraries/RoCo/Src/STMUARTDriver.d \
./System/Libraries/RoCo/Src/UDevDriver.d \
./System/Libraries/RoCo/Src/ZephyrUARTDriver.d 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/RoCo/Src/%.o System/Libraries/RoCo/Src/%.su System/Libraries/RoCo/Src/%.cyclo: ../System/Libraries/RoCo/Src/%.cpp System/Libraries/RoCo/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-RoCo-2f-Src

clean-System-2f-Libraries-2f-RoCo-2f-Src:
	-$(RM) ./System/Libraries/RoCo/Src/BufferedIODriver.cyclo ./System/Libraries/RoCo/Src/BufferedIODriver.d ./System/Libraries/RoCo/Src/BufferedIODriver.o ./System/Libraries/RoCo/Src/BufferedIODriver.su ./System/Libraries/RoCo/Src/EspressifUARTDriver.cyclo ./System/Libraries/RoCo/Src/EspressifUARTDriver.d ./System/Libraries/RoCo/Src/EspressifUARTDriver.o ./System/Libraries/RoCo/Src/EspressifUARTDriver.su ./System/Libraries/RoCo/Src/IOBus.cyclo ./System/Libraries/RoCo/Src/IOBus.d ./System/Libraries/RoCo/Src/IOBus.o ./System/Libraries/RoCo/Src/IOBus.su ./System/Libraries/RoCo/Src/LWIPClientIO.cyclo ./System/Libraries/RoCo/Src/LWIPClientIO.d ./System/Libraries/RoCo/Src/LWIPClientIO.o ./System/Libraries/RoCo/Src/LWIPClientIO.su ./System/Libraries/RoCo/Src/LoopbackDriver.cyclo ./System/Libraries/RoCo/Src/LoopbackDriver.d ./System/Libraries/RoCo/Src/LoopbackDriver.o ./System/Libraries/RoCo/Src/LoopbackDriver.su ./System/Libraries/RoCo/Src/MessageBus.cyclo ./System/Libraries/RoCo/Src/MessageBus.d ./System/Libraries/RoCo/Src/MessageBus.o ./System/Libraries/RoCo/Src/MessageBus.su ./System/Libraries/RoCo/Src/NetworkBus.cyclo ./System/Libraries/RoCo/Src/NetworkBus.d ./System/Libraries/RoCo/Src/NetworkBus.o ./System/Libraries/RoCo/Src/NetworkBus.su ./System/Libraries/RoCo/Src/NetworkClientIO.cyclo ./System/Libraries/RoCo/Src/NetworkClientIO.d ./System/Libraries/RoCo/Src/NetworkClientIO.o ./System/Libraries/RoCo/Src/NetworkClientIO.su ./System/Libraries/RoCo/Src/NetworkServerIO.cyclo ./System/Libraries/RoCo/Src/NetworkServerIO.d ./System/Libraries/RoCo/Src/NetworkServerIO.o ./System/Libraries/RoCo/Src/NetworkServerIO.su ./System/Libraries/RoCo/Src/PowerBus.cyclo ./System/Libraries/RoCo/Src/PowerBus.d ./System/Libraries/RoCo/Src/PowerBus.o ./System/Libraries/RoCo/Src/PowerBus.su ./System/Libraries/RoCo/Src/RadioBus.cyclo ./System/Libraries/RoCo/Src/RadioBus.d ./System/Libraries/RoCo/Src/RadioBus.o ./System/Libraries/RoCo/Src/RadioBus.su ./System/Libraries/RoCo/Src/STMUARTDriver.cyclo ./System/Libraries/RoCo/Src/STMUARTDriver.d ./System/Libraries/RoCo/Src/STMUARTDriver.o ./System/Libraries/RoCo/Src/STMUARTDriver.su ./System/Libraries/RoCo/Src/UDevDriver.cyclo ./System/Libraries/RoCo/Src/UDevDriver.d ./System/Libraries/RoCo/Src/UDevDriver.o ./System/Libraries/RoCo/Src/UDevDriver.su ./System/Libraries/RoCo/Src/ZephyrUARTDriver.cyclo ./System/Libraries/RoCo/Src/ZephyrUARTDriver.d ./System/Libraries/RoCo/Src/ZephyrUARTDriver.o ./System/Libraries/RoCo/Src/ZephyrUARTDriver.su

.PHONY: clean-System-2f-Libraries-2f-RoCo-2f-Src

