################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../bsp/ds3231.c \
../bsp/lcd.c \
../bsp/mylcd.c 

OBJS += \
./bsp/ds3231.o \
./bsp/lcd.o \
./bsp/mylcd.o 

C_DEPS += \
./bsp/ds3231.d \
./bsp/lcd.d \
./bsp/mylcd.d 


# Each subdirectory must supply rules for building sources it contributes
bsp/%.o bsp/%.su bsp/%.cyclo: ../bsp/%.c bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/nicth/STM32CubeIDE/workspace_1.14.1/DS3231_RTC_Module_and_STM32_cubeide_I2C_LCD/bsp" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-bsp

clean-bsp:
	-$(RM) ./bsp/ds3231.cyclo ./bsp/ds3231.d ./bsp/ds3231.o ./bsp/ds3231.su ./bsp/lcd.cyclo ./bsp/lcd.d ./bsp/lcd.o ./bsp/lcd.su ./bsp/mylcd.cyclo ./bsp/mylcd.d ./bsp/mylcd.o ./bsp/mylcd.su

.PHONY: clean-bsp

