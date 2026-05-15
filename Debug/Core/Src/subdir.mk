################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ESP01.c \
../Core/Src/button_key.c \
../Core/Src/buzzer_app.c \
../Core/Src/control_systems.c \
../Core/Src/fonts.c \
../Core/Src/line_sensors.c \
../Core/Src/main.c \
../Core/Src/mpu6050_app.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/uner_protocol.c 

OBJS += \
./Core/Src/ESP01.o \
./Core/Src/button_key.o \
./Core/Src/buzzer_app.o \
./Core/Src/control_systems.o \
./Core/Src/fonts.o \
./Core/Src/line_sensors.o \
./Core/Src/main.o \
./Core/Src/mpu6050_app.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/uner_protocol.o 

C_DEPS += \
./Core/Src/ESP01.d \
./Core/Src/button_key.d \
./Core/Src/buzzer_app.d \
./Core/Src/control_systems.d \
./Core/Src/fonts.d \
./Core/Src/line_sensors.d \
./Core/Src/main.d \
./Core/Src/mpu6050_app.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/uner_protocol.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ESP01.cyclo ./Core/Src/ESP01.d ./Core/Src/ESP01.o ./Core/Src/ESP01.su ./Core/Src/button_key.cyclo ./Core/Src/button_key.d ./Core/Src/button_key.o ./Core/Src/button_key.su ./Core/Src/buzzer_app.cyclo ./Core/Src/buzzer_app.d ./Core/Src/buzzer_app.o ./Core/Src/buzzer_app.su ./Core/Src/control_systems.cyclo ./Core/Src/control_systems.d ./Core/Src/control_systems.o ./Core/Src/control_systems.su ./Core/Src/fonts.cyclo ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/fonts.su ./Core/Src/line_sensors.cyclo ./Core/Src/line_sensors.d ./Core/Src/line_sensors.o ./Core/Src/line_sensors.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mpu6050_app.cyclo ./Core/Src/mpu6050_app.d ./Core/Src/mpu6050_app.o ./Core/Src/mpu6050_app.su ./Core/Src/ssd1306.cyclo ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/ssd1306.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/uner_protocol.cyclo ./Core/Src/uner_protocol.d ./Core/Src/uner_protocol.o ./Core/Src/uner_protocol.su

.PHONY: clean-Core-2f-Src

