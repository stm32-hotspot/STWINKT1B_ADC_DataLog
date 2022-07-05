################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/system_stm32l4xx.c 

C_DEPS += \
./Drivers/CMSIS/system_stm32l4xx.d 

OBJS += \
./Drivers/CMSIS/system_stm32l4xx.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32l4xx.o: C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/system_stm32l4xx.c Drivers/CMSIS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R9xx -c -I../../FATFS/Target -I../../FATFS/App -I../../Core/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FatFs/src -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/system_stm32l4xx.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS

clean-Drivers-2f-CMSIS:
	-$(RM) ./Drivers/CMSIS/system_stm32l4xx.d ./Drivers/CMSIS/system_stm32l4xx.o ./Drivers/CMSIS/system_stm32l4xx.su

.PHONY: clean-Drivers-2f-CMSIS

