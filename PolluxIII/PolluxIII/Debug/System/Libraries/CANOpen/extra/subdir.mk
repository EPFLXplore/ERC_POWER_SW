################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/Libraries/CANOpen/extra/CO_trace.c 

C_DEPS += \
./System/Libraries/CANOpen/extra/CO_trace.d 

OBJS += \
./System/Libraries/CANOpen/extra/CO_trace.o 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/CANOpen/extra/%.o System/Libraries/CANOpen/extra/%.su System/Libraries/CANOpen/extra/%.cyclo: ../System/Libraries/CANOpen/extra/%.c System/Libraries/CANOpen/extra/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../System/Libraries -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-CANOpen-2f-extra

clean-System-2f-Libraries-2f-CANOpen-2f-extra:
	-$(RM) ./System/Libraries/CANOpen/extra/CO_trace.cyclo ./System/Libraries/CANOpen/extra/CO_trace.d ./System/Libraries/CANOpen/extra/CO_trace.o ./System/Libraries/CANOpen/extra/CO_trace.su

.PHONY: clean-System-2f-Libraries-2f-CANOpen-2f-extra

