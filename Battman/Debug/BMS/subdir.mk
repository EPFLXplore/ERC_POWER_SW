################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../BMS/BMS.c \
../BMS/LTC6811.c \
../BMS/LTC681x.c \
../BMS/LT_SPI.c 

OBJS += \
./BMS/BMS.o \
./BMS/LTC6811.o \
./BMS/LTC681x.o \
./BMS/LT_SPI.o 

C_DEPS += \
./BMS/BMS.d \
./BMS/LTC6811.d \
./BMS/LTC681x.d \
./BMS/LT_SPI.d 


# Each subdirectory must supply rules for building sources it contributes
BMS/%.o BMS/%.su BMS/%.cyclo: ../BMS/%.c BMS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m33 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L552xx -c -I../USB_Device/App -I/BMS -I../USB_Device/Target -I../Core/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc -I../Drivers/STM32L5xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L5xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-BMS

clean-BMS:
	-$(RM) ./BMS/BMS.cyclo ./BMS/BMS.d ./BMS/BMS.o ./BMS/BMS.su ./BMS/LTC6811.cyclo ./BMS/LTC6811.d ./BMS/LTC6811.o ./BMS/LTC6811.su ./BMS/LTC681x.cyclo ./BMS/LTC681x.d ./BMS/LTC681x.o ./BMS/LTC681x.su ./BMS/LT_SPI.cyclo ./BMS/LT_SPI.d ./BMS/LT_SPI.o ./BMS/LT_SPI.su

.PHONY: clean-BMS

