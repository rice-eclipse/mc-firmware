################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/cJSON.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/interface.c \
../Core/Src/jsmn.c \
../Core/Src/main.c \
../Core/Src/mongoose.c \
../Core/Src/rng.c \
../Core/Src/sdio.c \
../Core/Src/spi.c \
../Core/Src/stm32f2xx_hal_msp.c \
../Core/Src/stm32f2xx_hal_timebase_tim.c \
../Core/Src/stm32f2xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f2xx.c \
../Core/Src/tim.c \
../Core/Src/usart.c \
../Core/Src/usb_otg.c \
../Core/Src/utils.c 

OBJS += \
./Core/Src/cJSON.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/interface.o \
./Core/Src/jsmn.o \
./Core/Src/main.o \
./Core/Src/mongoose.o \
./Core/Src/rng.o \
./Core/Src/sdio.o \
./Core/Src/spi.o \
./Core/Src/stm32f2xx_hal_msp.o \
./Core/Src/stm32f2xx_hal_timebase_tim.o \
./Core/Src/stm32f2xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f2xx.o \
./Core/Src/tim.o \
./Core/Src/usart.o \
./Core/Src/usb_otg.o \
./Core/Src/utils.o 

C_DEPS += \
./Core/Src/cJSON.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/interface.d \
./Core/Src/jsmn.d \
./Core/Src/main.d \
./Core/Src/mongoose.d \
./Core/Src/rng.d \
./Core/Src/sdio.d \
./Core/Src/spi.d \
./Core/Src/stm32f2xx_hal_msp.d \
./Core/Src/stm32f2xx_hal_timebase_tim.d \
./Core/Src/stm32f2xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f2xx.d \
./Core/Src/tim.d \
./Core/Src/usart.d \
./Core/Src/usb_otg.d \
./Core/Src/utils.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F207xx -c -I../Core/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc -I../Drivers/STM32F2xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F2xx/Include -I../Drivers/CMSIS/Include -I../LWIP/App -I../LWIP/Target -I../Middlewares/Third_Party/LwIP/src/include -I../Middlewares/Third_Party/LwIP/system -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/Third_Party/LwIP/src/include/netif/ppp -I../Middlewares/Third_Party/LwIP/src/include/lwip -I../Middlewares/Third_Party/LwIP/src/include/lwip/apps -I../Middlewares/Third_Party/LwIP/src/include/lwip/priv -I../Middlewares/Third_Party/LwIP/src/include/lwip/prot -I../Middlewares/Third_Party/LwIP/src/include/netif -I../Middlewares/Third_Party/LwIP/src/include/posix -I../Middlewares/Third_Party/LwIP/src/include/posix/sys -I../Middlewares/Third_Party/LwIP/system/arch -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FatFs/src/drivers -O0 -ffunction-sections -fdata-sections -Wall -fcommon -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/cJSON.cyclo ./Core/Src/cJSON.d ./Core/Src/cJSON.o ./Core/Src/cJSON.su ./Core/Src/freertos.cyclo ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.cyclo ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/interface.cyclo ./Core/Src/interface.d ./Core/Src/interface.o ./Core/Src/interface.su ./Core/Src/jsmn.cyclo ./Core/Src/jsmn.d ./Core/Src/jsmn.o ./Core/Src/jsmn.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mongoose.cyclo ./Core/Src/mongoose.d ./Core/Src/mongoose.o ./Core/Src/mongoose.su ./Core/Src/rng.cyclo ./Core/Src/rng.d ./Core/Src/rng.o ./Core/Src/rng.su ./Core/Src/sdio.cyclo ./Core/Src/sdio.d ./Core/Src/sdio.o ./Core/Src/sdio.su ./Core/Src/spi.cyclo ./Core/Src/spi.d ./Core/Src/spi.o ./Core/Src/spi.su ./Core/Src/stm32f2xx_hal_msp.cyclo ./Core/Src/stm32f2xx_hal_msp.d ./Core/Src/stm32f2xx_hal_msp.o ./Core/Src/stm32f2xx_hal_msp.su ./Core/Src/stm32f2xx_hal_timebase_tim.cyclo ./Core/Src/stm32f2xx_hal_timebase_tim.d ./Core/Src/stm32f2xx_hal_timebase_tim.o ./Core/Src/stm32f2xx_hal_timebase_tim.su ./Core/Src/stm32f2xx_it.cyclo ./Core/Src/stm32f2xx_it.d ./Core/Src/stm32f2xx_it.o ./Core/Src/stm32f2xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f2xx.cyclo ./Core/Src/system_stm32f2xx.d ./Core/Src/system_stm32f2xx.o ./Core/Src/system_stm32f2xx.su ./Core/Src/tim.cyclo ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/usart.cyclo ./Core/Src/usart.d ./Core/Src/usart.o ./Core/Src/usart.su ./Core/Src/usb_otg.cyclo ./Core/Src/usb_otg.d ./Core/Src/usb_otg.o ./Core/Src/usb_otg.su ./Core/Src/utils.cyclo ./Core/Src/utils.d ./Core/Src/utils.o ./Core/Src/utils.su

.PHONY: clean-Core-2f-Src

