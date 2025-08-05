################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/GUI/ChargeGauge.cpp \
../System/GUI/DetailsScreen.cpp \
../System/GUI/GUIThread.cpp \
../System/GUI/MainScreen.cpp \
../System/GUI/PowerBox.cpp \
../System/GUI/SupplyBox.cpp \
../System/GUI/UnregulatedBox.cpp 

OBJS += \
./System/GUI/ChargeGauge.o \
./System/GUI/DetailsScreen.o \
./System/GUI/GUIThread.o \
./System/GUI/MainScreen.o \
./System/GUI/PowerBox.o \
./System/GUI/SupplyBox.o \
./System/GUI/UnregulatedBox.o 

CPP_DEPS += \
./System/GUI/ChargeGauge.d \
./System/GUI/DetailsScreen.d \
./System/GUI/GUIThread.d \
./System/GUI/MainScreen.d \
./System/GUI/PowerBox.d \
./System/GUI/SupplyBox.d \
./System/GUI/UnregulatedBox.d 


# Each subdirectory must supply rules for building sources it contributes
System/GUI/%.o System/GUI/%.su System/GUI/%.cyclo: ../System/GUI/%.cpp System/GUI/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-GUI

clean-System-2f-GUI:
	-$(RM) ./System/GUI/ChargeGauge.cyclo ./System/GUI/ChargeGauge.d ./System/GUI/ChargeGauge.o ./System/GUI/ChargeGauge.su ./System/GUI/DetailsScreen.cyclo ./System/GUI/DetailsScreen.d ./System/GUI/DetailsScreen.o ./System/GUI/DetailsScreen.su ./System/GUI/GUIThread.cyclo ./System/GUI/GUIThread.d ./System/GUI/GUIThread.o ./System/GUI/GUIThread.su ./System/GUI/MainScreen.cyclo ./System/GUI/MainScreen.d ./System/GUI/MainScreen.o ./System/GUI/MainScreen.su ./System/GUI/PowerBox.cyclo ./System/GUI/PowerBox.d ./System/GUI/PowerBox.o ./System/GUI/PowerBox.su ./System/GUI/SupplyBox.cyclo ./System/GUI/SupplyBox.d ./System/GUI/SupplyBox.o ./System/GUI/SupplyBox.su ./System/GUI/UnregulatedBox.cyclo ./System/GUI/UnregulatedBox.d ./System/GUI/UnregulatedBox.o ./System/GUI/UnregulatedBox.su

.PHONY: clean-System-2f-GUI

