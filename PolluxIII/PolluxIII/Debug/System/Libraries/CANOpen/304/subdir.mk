################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/Libraries/CANOpen/304/CO_GFC.c \
../System/Libraries/CANOpen/304/CO_SRDO.c 

C_DEPS += \
./System/Libraries/CANOpen/304/CO_GFC.d \
./System/Libraries/CANOpen/304/CO_SRDO.d 

OBJS += \
./System/Libraries/CANOpen/304/CO_GFC.o \
./System/Libraries/CANOpen/304/CO_SRDO.o 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/CANOpen/304/%.o System/Libraries/CANOpen/304/%.su System/Libraries/CANOpen/304/%.cyclo: ../System/Libraries/CANOpen/304/%.c System/Libraries/CANOpen/304/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../System/Libraries -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-CANOpen-2f-304

clean-System-2f-Libraries-2f-CANOpen-2f-304:
	-$(RM) ./System/Libraries/CANOpen/304/CO_GFC.cyclo ./System/Libraries/CANOpen/304/CO_GFC.d ./System/Libraries/CANOpen/304/CO_GFC.o ./System/Libraries/CANOpen/304/CO_GFC.su ./System/Libraries/CANOpen/304/CO_SRDO.cyclo ./System/Libraries/CANOpen/304/CO_SRDO.d ./System/Libraries/CANOpen/304/CO_SRDO.o ./System/Libraries/CANOpen/304/CO_SRDO.su

.PHONY: clean-System-2f-Libraries-2f-CANOpen-2f-304

