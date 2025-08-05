################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Supplies/Supplies.cpp \
../System/Supplies/SupplyStates.cpp \
../System/Supplies/SupplyThread.cpp 

OBJS += \
./System/Supplies/Supplies.o \
./System/Supplies/SupplyStates.o \
./System/Supplies/SupplyThread.o 

CPP_DEPS += \
./System/Supplies/Supplies.d \
./System/Supplies/SupplyStates.d \
./System/Supplies/SupplyThread.d 


# Each subdirectory must supply rules for building sources it contributes
System/Supplies/%.o System/Supplies/%.su System/Supplies/%.cyclo: ../System/Supplies/%.cpp System/Supplies/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Supplies

clean-System-2f-Supplies:
	-$(RM) ./System/Supplies/Supplies.cyclo ./System/Supplies/Supplies.d ./System/Supplies/Supplies.o ./System/Supplies/Supplies.su ./System/Supplies/SupplyStates.cyclo ./System/Supplies/SupplyStates.d ./System/Supplies/SupplyStates.o ./System/Supplies/SupplyStates.su ./System/Supplies/SupplyThread.cyclo ./System/Supplies/SupplyThread.d ./System/Supplies/SupplyThread.o ./System/Supplies/SupplyThread.su

.PHONY: clean-System-2f-Supplies

