################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../System/Libraries/CANOpen/301/CO_Emergency.c \
../System/Libraries/CANOpen/301/CO_HBconsumer.c \
../System/Libraries/CANOpen/301/CO_NMT_Heartbeat.c \
../System/Libraries/CANOpen/301/CO_ODinterface.c \
../System/Libraries/CANOpen/301/CO_PDO.c \
../System/Libraries/CANOpen/301/CO_SDOclient.c \
../System/Libraries/CANOpen/301/CO_SDOserver.c \
../System/Libraries/CANOpen/301/CO_SYNC.c \
../System/Libraries/CANOpen/301/CO_TIME.c \
../System/Libraries/CANOpen/301/CO_fifo.c \
../System/Libraries/CANOpen/301/crc16-ccitt.c 

C_DEPS += \
./System/Libraries/CANOpen/301/CO_Emergency.d \
./System/Libraries/CANOpen/301/CO_HBconsumer.d \
./System/Libraries/CANOpen/301/CO_NMT_Heartbeat.d \
./System/Libraries/CANOpen/301/CO_ODinterface.d \
./System/Libraries/CANOpen/301/CO_PDO.d \
./System/Libraries/CANOpen/301/CO_SDOclient.d \
./System/Libraries/CANOpen/301/CO_SDOserver.d \
./System/Libraries/CANOpen/301/CO_SYNC.d \
./System/Libraries/CANOpen/301/CO_TIME.d \
./System/Libraries/CANOpen/301/CO_fifo.d \
./System/Libraries/CANOpen/301/crc16-ccitt.d 

OBJS += \
./System/Libraries/CANOpen/301/CO_Emergency.o \
./System/Libraries/CANOpen/301/CO_HBconsumer.o \
./System/Libraries/CANOpen/301/CO_NMT_Heartbeat.o \
./System/Libraries/CANOpen/301/CO_ODinterface.o \
./System/Libraries/CANOpen/301/CO_PDO.o \
./System/Libraries/CANOpen/301/CO_SDOclient.o \
./System/Libraries/CANOpen/301/CO_SDOserver.o \
./System/Libraries/CANOpen/301/CO_SYNC.o \
./System/Libraries/CANOpen/301/CO_TIME.o \
./System/Libraries/CANOpen/301/CO_fifo.o \
./System/Libraries/CANOpen/301/crc16-ccitt.o 


# Each subdirectory must supply rules for building sources it contributes
System/Libraries/CANOpen/301/%.o System/Libraries/CANOpen/301/%.su System/Libraries/CANOpen/301/%.cyclo: ../System/Libraries/CANOpen/301/%.c System/Libraries/CANOpen/301/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H750xx -DBUILD_FOR_POWER -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../System -I../System/Libraries -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Utilities/JPEG -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-System-2f-Libraries-2f-CANOpen-2f-301

clean-System-2f-Libraries-2f-CANOpen-2f-301:
	-$(RM) ./System/Libraries/CANOpen/301/CO_Emergency.cyclo ./System/Libraries/CANOpen/301/CO_Emergency.d ./System/Libraries/CANOpen/301/CO_Emergency.o ./System/Libraries/CANOpen/301/CO_Emergency.su ./System/Libraries/CANOpen/301/CO_HBconsumer.cyclo ./System/Libraries/CANOpen/301/CO_HBconsumer.d ./System/Libraries/CANOpen/301/CO_HBconsumer.o ./System/Libraries/CANOpen/301/CO_HBconsumer.su ./System/Libraries/CANOpen/301/CO_NMT_Heartbeat.cyclo ./System/Libraries/CANOpen/301/CO_NMT_Heartbeat.d ./System/Libraries/CANOpen/301/CO_NMT_Heartbeat.o ./System/Libraries/CANOpen/301/CO_NMT_Heartbeat.su ./System/Libraries/CANOpen/301/CO_ODinterface.cyclo ./System/Libraries/CANOpen/301/CO_ODinterface.d ./System/Libraries/CANOpen/301/CO_ODinterface.o ./System/Libraries/CANOpen/301/CO_ODinterface.su ./System/Libraries/CANOpen/301/CO_PDO.cyclo ./System/Libraries/CANOpen/301/CO_PDO.d ./System/Libraries/CANOpen/301/CO_PDO.o ./System/Libraries/CANOpen/301/CO_PDO.su ./System/Libraries/CANOpen/301/CO_SDOclient.cyclo ./System/Libraries/CANOpen/301/CO_SDOclient.d ./System/Libraries/CANOpen/301/CO_SDOclient.o ./System/Libraries/CANOpen/301/CO_SDOclient.su ./System/Libraries/CANOpen/301/CO_SDOserver.cyclo ./System/Libraries/CANOpen/301/CO_SDOserver.d ./System/Libraries/CANOpen/301/CO_SDOserver.o ./System/Libraries/CANOpen/301/CO_SDOserver.su ./System/Libraries/CANOpen/301/CO_SYNC.cyclo ./System/Libraries/CANOpen/301/CO_SYNC.d ./System/Libraries/CANOpen/301/CO_SYNC.o ./System/Libraries/CANOpen/301/CO_SYNC.su ./System/Libraries/CANOpen/301/CO_TIME.cyclo ./System/Libraries/CANOpen/301/CO_TIME.d ./System/Libraries/CANOpen/301/CO_TIME.o ./System/Libraries/CANOpen/301/CO_TIME.su ./System/Libraries/CANOpen/301/CO_fifo.cyclo ./System/Libraries/CANOpen/301/CO_fifo.d ./System/Libraries/CANOpen/301/CO_fifo.o ./System/Libraries/CANOpen/301/CO_fifo.su ./System/Libraries/CANOpen/301/crc16-ccitt.cyclo ./System/Libraries/CANOpen/301/crc16-ccitt.d ./System/Libraries/CANOpen/301/crc16-ccitt.o ./System/Libraries/CANOpen/301/crc16-ccitt.su

.PHONY: clean-System-2f-Libraries-2f-CANOpen-2f-301

