################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/Libraries/CANOpen/CANopen.c \
../System/Libraries/CANOpen/CO_driver_stm32h735g-dk.c \
../System/Libraries/CANOpen/OD.c 

C_DEPS += \
./System/Libraries/CANOpen/CANopen.d \
./System/Libraries/CANOpen/CO_driver_stm32h735g-dk.d \
./System/Libraries/CANOpen/OD.d 

OBJS += \
./System/Libraries/CANOpen/CANopen.o \
./System/Libraries/CANOpen/CO_driver_stm32h735g-dk.o \
./System/Libraries/CANOpen/OD.o 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/CANOpen/%.o System/Libraries/CANOpen/%.su System/Libraries/CANOpen/%.cyclo: ../System/Libraries/CANOpen/%.c System/Libraries/CANOpen/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../System/Libraries -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-CANOpen

clean-System-2f-Libraries-2f-CANOpen:
	-$(RM) ./System/Libraries/CANOpen/CANopen.cyclo ./System/Libraries/CANOpen/CANopen.d ./System/Libraries/CANOpen/CANopen.o ./System/Libraries/CANOpen/CANopen.su ./System/Libraries/CANOpen/CO_driver_stm32h735g-dk.cyclo ./System/Libraries/CANOpen/CO_driver_stm32h735g-dk.d ./System/Libraries/CANOpen/CO_driver_stm32h735g-dk.o ./System/Libraries/CANOpen/CO_driver_stm32h735g-dk.su ./System/Libraries/CANOpen/OD.cyclo ./System/Libraries/CANOpen/OD.d ./System/Libraries/CANOpen/OD.o ./System/Libraries/CANOpen/OD.su

.PHONY: clean-System-2f-Libraries-2f-CANOpen

