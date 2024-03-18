################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Drivers/ADXL375.c \
../Core/Src/Drivers/BMI088.c \
../Core/Src/Drivers/BMP388.c \
../Core/Src/Drivers/W25Q128.c 

OBJS += \
./Core/Src/Drivers/ADXL375.o \
./Core/Src/Drivers/BMI088.o \
./Core/Src/Drivers/BMP388.o \
./Core/Src/Drivers/W25Q128.o 

C_DEPS += \
./Core/Src/Drivers/ADXL375.d \
./Core/Src/Drivers/BMI088.d \
./Core/Src/Drivers/BMP388.d \
./Core/Src/Drivers/W25Q128.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Drivers/%.o Core/Src/Drivers/%.su Core/Src/Drivers/%.cyclo: ../Core/Src/Drivers/%.c Core/Src/Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L476xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Drivers

clean-Core-2f-Src-2f-Drivers:
	-$(RM) ./Core/Src/Drivers/ADXL375.cyclo ./Core/Src/Drivers/ADXL375.d ./Core/Src/Drivers/ADXL375.o ./Core/Src/Drivers/ADXL375.su ./Core/Src/Drivers/BMI088.cyclo ./Core/Src/Drivers/BMI088.d ./Core/Src/Drivers/BMI088.o ./Core/Src/Drivers/BMI088.su ./Core/Src/Drivers/BMP388.cyclo ./Core/Src/Drivers/BMP388.d ./Core/Src/Drivers/BMP388.o ./Core/Src/Drivers/BMP388.su ./Core/Src/Drivers/W25Q128.cyclo ./Core/Src/Drivers/W25Q128.d ./Core/Src/Drivers/W25Q128.o ./Core/Src/Drivers/W25Q128.su

.PHONY: clean-Core-2f-Src-2f-Drivers

