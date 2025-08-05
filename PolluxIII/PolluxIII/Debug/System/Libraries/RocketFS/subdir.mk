################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Libraries/RocketFS/BlockManagement.cpp \
../System/Libraries/RocketFS/File.cpp \
../System/Libraries/RocketFS/FileSystem.cpp \
../System/Libraries/RocketFS/Stream.cpp 

OBJS += \
./System/Libraries/RocketFS/BlockManagement.o \
./System/Libraries/RocketFS/File.o \
./System/Libraries/RocketFS/FileSystem.o \
./System/Libraries/RocketFS/Stream.o 

CPP_DEPS += \
./System/Libraries/RocketFS/BlockManagement.d \
./System/Libraries/RocketFS/File.d \
./System/Libraries/RocketFS/FileSystem.d \
./System/Libraries/RocketFS/Stream.d 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/RocketFS/%.o System/Libraries/RocketFS/%.su System/Libraries/RocketFS/%.cyclo: ../System/Libraries/RocketFS/%.cpp System/Libraries/RocketFS/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-RocketFS

clean-System-2f-Libraries-2f-RocketFS:
	-$(RM) ./System/Libraries/RocketFS/BlockManagement.cyclo ./System/Libraries/RocketFS/BlockManagement.d ./System/Libraries/RocketFS/BlockManagement.o ./System/Libraries/RocketFS/BlockManagement.su ./System/Libraries/RocketFS/File.cyclo ./System/Libraries/RocketFS/File.d ./System/Libraries/RocketFS/File.o ./System/Libraries/RocketFS/File.su ./System/Libraries/RocketFS/FileSystem.cyclo ./System/Libraries/RocketFS/FileSystem.d ./System/Libraries/RocketFS/FileSystem.o ./System/Libraries/RocketFS/FileSystem.su ./System/Libraries/RocketFS/Stream.cyclo ./System/Libraries/RocketFS/Stream.d ./System/Libraries/RocketFS/Stream.o ./System/Libraries/RocketFS/Stream.su

.PHONY: clean-System-2f-Libraries-2f-RocketFS

