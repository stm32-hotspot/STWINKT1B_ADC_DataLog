################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Application/User/Core/RingBuffer.c \
C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/main.c \
C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/stm32l4xx_hal_msp.c \
C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/stm32l4xx_it.c \
../Application/User/Core/syscalls.c \
../Application/User/Core/sysmem.c 

C_DEPS += \
./Application/User/Core/RingBuffer.d \
./Application/User/Core/main.d \
./Application/User/Core/stm32l4xx_hal_msp.d \
./Application/User/Core/stm32l4xx_it.d \
./Application/User/Core/syscalls.d \
./Application/User/Core/sysmem.d 

OBJS += \
./Application/User/Core/RingBuffer.o \
./Application/User/Core/main.o \
./Application/User/Core/stm32l4xx_hal_msp.o \
./Application/User/Core/stm32l4xx_it.o \
./Application/User/Core/syscalls.o \
./Application/User/Core/sysmem.o 


# Each subdirectory must supply rules for building sources it contributes
Application/User/Core/%.o Application/User/Core/%.su: ../Application/User/Core/%.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R9xx -c -I../../FATFS/Target -I../../FATFS/App -I../../Core/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FatFs/src -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/main.o: C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/main.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R9xx -c -I../../FATFS/Target -I../../FATFS/App -I../../Core/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FatFs/src -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/Core/main.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32l4xx_hal_msp.o: C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/stm32l4xx_hal_msp.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R9xx -c -I../../FATFS/Target -I../../FATFS/App -I../../Core/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FatFs/src -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/Core/stm32l4xx_hal_msp.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/Core/stm32l4xx_it.o: C:/Users/troy\ davis/OneDrive\ -\ STMicroelectronics/Documents/STM32_Hotspot/ADCDataLogLocal/Core/Src/stm32l4xx_it.c Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R9xx -c -I../../FATFS/Target -I../../FATFS/App -I../../Core/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc -I../../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../../Middlewares/Third_Party/FatFs/src -I../../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Application/User/Core/stm32l4xx_it.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User-2f-Core

clean-Application-2f-User-2f-Core:
	-$(RM) ./Application/User/Core/RingBuffer.d ./Application/User/Core/RingBuffer.o ./Application/User/Core/RingBuffer.su ./Application/User/Core/main.d ./Application/User/Core/main.o ./Application/User/Core/main.su ./Application/User/Core/stm32l4xx_hal_msp.d ./Application/User/Core/stm32l4xx_hal_msp.o ./Application/User/Core/stm32l4xx_hal_msp.su ./Application/User/Core/stm32l4xx_it.d ./Application/User/Core/stm32l4xx_it.o ./Application/User/Core/stm32l4xx_it.su ./Application/User/Core/syscalls.d ./Application/User/Core/syscalls.o ./Application/User/Core/syscalls.su ./Application/User/Core/sysmem.d ./Application/User/Core/sysmem.o ./Application/User/Core/sysmem.su

.PHONY: clean-Application-2f-User-2f-Core

