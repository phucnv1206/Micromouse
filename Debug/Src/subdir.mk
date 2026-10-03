################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/button.c \
../Src/config.c \
../Src/encoder.c \
../Src/i2c_bus.c \
../Src/led.c \
../Src/main.c \
../Src/motor_pwm.c \
../Src/odometry.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c \
../Src/tof_sensors.c \
../Src/tof_xshut.c \
../Src/uart_protocol.c \
../Src/vl53l0x_async.c \
../Src/vl53l0x_stm32f4_port.c \
../Src/vl53l1x_stm32f4_port.c 

OBJS += \
./Src/button.o \
./Src/config.o \
./Src/encoder.o \
./Src/i2c_bus.o \
./Src/led.o \
./Src/main.o \
./Src/motor_pwm.o \
./Src/odometry.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o \
./Src/tof_sensors.o \
./Src/tof_xshut.o \
./Src/uart_protocol.o \
./Src/vl53l0x_async.o \
./Src/vl53l0x_stm32f4_port.o \
./Src/vl53l1x_stm32f4_port.o 

C_DEPS += \
./Src/button.d \
./Src/config.d \
./Src/encoder.d \
./Src/i2c_bus.d \
./Src/led.d \
./Src/main.d \
./Src/motor_pwm.d \
./Src/odometry.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d \
./Src/tof_sensors.d \
./Src/tof_xshut.d \
./Src/uart_protocol.d \
./Src/vl53l0x_async.d \
./Src/vl53l0x_stm32f4_port.d \
./Src/vl53l1x_stm32f4_port.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Inc -I../Drivers/VL53L0X/Inc -I../Drivers/VL53L1X/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/button.cyclo ./Src/button.d ./Src/button.o ./Src/button.su ./Src/config.cyclo ./Src/config.d ./Src/config.o ./Src/config.su ./Src/encoder.cyclo ./Src/encoder.d ./Src/encoder.o ./Src/encoder.su ./Src/i2c_bus.cyclo ./Src/i2c_bus.d ./Src/i2c_bus.o ./Src/i2c_bus.su ./Src/led.cyclo ./Src/led.d ./Src/led.o ./Src/led.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/motor_pwm.cyclo ./Src/motor_pwm.d ./Src/motor_pwm.o ./Src/motor_pwm.su ./Src/odometry.cyclo ./Src/odometry.d ./Src/odometry.o ./Src/odometry.su ./Src/stm32f4xx_hal_msp.cyclo ./Src/stm32f4xx_hal_msp.d ./Src/stm32f4xx_hal_msp.o ./Src/stm32f4xx_hal_msp.su ./Src/stm32f4xx_it.cyclo ./Src/stm32f4xx_it.d ./Src/stm32f4xx_it.o ./Src/stm32f4xx_it.su ./Src/system_stm32f4xx.cyclo ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su ./Src/tof_sensors.cyclo ./Src/tof_sensors.d ./Src/tof_sensors.o ./Src/tof_sensors.su ./Src/tof_xshut.cyclo ./Src/tof_xshut.d ./Src/tof_xshut.o ./Src/tof_xshut.su ./Src/uart_protocol.cyclo ./Src/uart_protocol.d ./Src/uart_protocol.o ./Src/uart_protocol.su ./Src/vl53l0x_async.cyclo ./Src/vl53l0x_async.d ./Src/vl53l0x_async.o ./Src/vl53l0x_async.su ./Src/vl53l0x_stm32f4_port.cyclo ./Src/vl53l0x_stm32f4_port.d ./Src/vl53l0x_stm32f4_port.o ./Src/vl53l0x_stm32f4_port.su ./Src/vl53l1x_stm32f4_port.cyclo ./Src/vl53l1x_stm32f4_port.d ./Src/vl53l1x_stm32f4_port.o ./Src/vl53l1x_stm32f4_port.su

.PHONY: clean-Src

