################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LCD/lcd.c 

OBJS += \
./LCD/lcd.o 

C_DEPS += \
./LCD/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
LCD/%.o LCD/%.su LCD/%.cyclo: ../LCD/%.c LCD/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/joseram/Desktop/Cursos/Curso freertos/LCD_test_freertos/LCD" -I"C:/Users/joseram/Desktop/Cursos/Curso freertos/LCD_test_freertos/SevenSegmentDisplay" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LCD

clean-LCD:
	-$(RM) ./LCD/lcd.cyclo ./LCD/lcd.d ./LCD/lcd.o ./LCD/lcd.su

.PHONY: clean-LCD

