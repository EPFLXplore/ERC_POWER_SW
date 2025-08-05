################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../System/Libraries/FlashAPI/Src/flash_state_debugger.cpp \
../System/Libraries/FlashAPI/Src/io_driver.cpp \
../System/Libraries/FlashAPI/Src/qspi_driver.cpp 

OBJS += \
./System/Libraries/FlashAPI/Src/flash_state_debugger.o \
./System/Libraries/FlashAPI/Src/io_driver.o \
./System/Libraries/FlashAPI/Src/qspi_driver.o 

CPP_DEPS += \
./System/Libraries/FlashAPI/Src/flash_state_debugger.d \
./System/Libraries/FlashAPI/Src/io_driver.d \
./System/Libraries/FlashAPI/Src/qspi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/FlashAPI/Src/%.o System/Libraries/FlashAPI/Src/%.su System/Libraries/FlashAPI/Src/%.cyclo: ../System/Libraries/FlashAPI/Src/%.cpp System/Libraries/FlashAPI/Src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -D_GLIBCXX_DEBUG -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../System/Libraries -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-FlashAPI-2f-Src

clean-System-2f-Libraries-2f-FlashAPI-2f-Src:
	-$(RM) ./System/Libraries/FlashAPI/Src/flash_state_debugger.cyclo ./System/Libraries/FlashAPI/Src/flash_state_debugger.d ./System/Libraries/FlashAPI/Src/flash_state_debugger.o ./System/Libraries/FlashAPI/Src/flash_state_debugger.su ./System/Libraries/FlashAPI/Src/io_driver.cyclo ./System/Libraries/FlashAPI/Src/io_driver.d ./System/Libraries/FlashAPI/Src/io_driver.o ./System/Libraries/FlashAPI/Src/io_driver.su ./System/Libraries/FlashAPI/Src/qspi_driver.cyclo ./System/Libraries/FlashAPI/Src/qspi_driver.d ./System/Libraries/FlashAPI/Src/qspi_driver.o ./System/Libraries/FlashAPI/Src/qspi_driver.su

.PHONY: clean-System-2f-Libraries-2f-FlashAPI-2f-Src

