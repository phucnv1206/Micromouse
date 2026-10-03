################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/VL53L1X/Src/VL53L1X_api.c 

OBJS += \
./Drivers/VL53L1X/Src/VL53L1X_api.o 

C_DEPS += \
./Drivers/VL53L1X/Src/VL53L1X_api.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/VL53L1X/Src/%.o Drivers/VL53L1X/Src/%.su Drivers/VL53L1X/Src/%.cyclo: ../Drivers/VL53L1X/Src/%.c Drivers/VL53L1X/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../Inc -I../Drivers/VL53L0X/Inc -I../Drivers/VL53L1X/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-VL53L1X-2f-Src

clean-Drivers-2f-VL53L1X-2f-Src:
	-$(RM) ./Drivers/VL53L1X/Src/VL53L1X_api.cyclo ./Drivers/VL53L1X/Src/VL53L1X_api.d ./Drivers/VL53L1X/Src/VL53L1X_api.o ./Drivers/VL53L1X/Src/VL53L1X_api.su

.PHONY: clean-Drivers-2f-VL53L1X-2f-Src

