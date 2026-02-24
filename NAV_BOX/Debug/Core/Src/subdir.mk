################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/NBKF.c \
../Core/Src/bmp180_for_stm32_hal.c \
../Core/Src/bno055.c \
../Core/Src/main.c \
../Core/Src/nmea_parse.c \
../Core/Src/nslp_dma.c \
../Core/Src/sens_fusion.c \
../Core/Src/stm32f3xx_hal_msp.c \
../Core/Src/stm32f3xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f3xx.c \
../Core/Src/ubx_nav_sol.c 

C_DEPS += \
./Core/Src/NBKF.d \
./Core/Src/bmp180_for_stm32_hal.d \
./Core/Src/bno055.d \
./Core/Src/main.d \
./Core/Src/nmea_parse.d \
./Core/Src/nslp_dma.d \
./Core/Src/sens_fusion.d \
./Core/Src/stm32f3xx_hal_msp.d \
./Core/Src/stm32f3xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f3xx.d \
./Core/Src/ubx_nav_sol.d 

OBJS += \
./Core/Src/NBKF.o \
./Core/Src/bmp180_for_stm32_hal.o \
./Core/Src/bno055.o \
./Core/Src/main.o \
./Core/Src/nmea_parse.o \
./Core/Src/nslp_dma.o \
./Core/Src/sens_fusion.o \
./Core/Src/stm32f3xx_hal_msp.o \
./Core/Src/stm32f3xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f3xx.o \
./Core/Src/ubx_nav_sol.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F303x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/NBKF.cyclo ./Core/Src/NBKF.d ./Core/Src/NBKF.o ./Core/Src/NBKF.su ./Core/Src/bmp180_for_stm32_hal.cyclo ./Core/Src/bmp180_for_stm32_hal.d ./Core/Src/bmp180_for_stm32_hal.o ./Core/Src/bmp180_for_stm32_hal.su ./Core/Src/bno055.cyclo ./Core/Src/bno055.d ./Core/Src/bno055.o ./Core/Src/bno055.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/nmea_parse.cyclo ./Core/Src/nmea_parse.d ./Core/Src/nmea_parse.o ./Core/Src/nmea_parse.su ./Core/Src/nslp_dma.cyclo ./Core/Src/nslp_dma.d ./Core/Src/nslp_dma.o ./Core/Src/nslp_dma.su ./Core/Src/sens_fusion.cyclo ./Core/Src/sens_fusion.d ./Core/Src/sens_fusion.o ./Core/Src/sens_fusion.su ./Core/Src/stm32f3xx_hal_msp.cyclo ./Core/Src/stm32f3xx_hal_msp.d ./Core/Src/stm32f3xx_hal_msp.o ./Core/Src/stm32f3xx_hal_msp.su ./Core/Src/stm32f3xx_it.cyclo ./Core/Src/stm32f3xx_it.d ./Core/Src/stm32f3xx_it.o ./Core/Src/stm32f3xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f3xx.cyclo ./Core/Src/system_stm32f3xx.d ./Core/Src/system_stm32f3xx.o ./Core/Src/system_stm32f3xx.su ./Core/Src/ubx_nav_sol.cyclo ./Core/Src/ubx_nav_sol.d ./Core/Src/ubx_nav_sol.o ./Core/Src/ubx_nav_sol.su

.PHONY: clean-Core-2f-Src

