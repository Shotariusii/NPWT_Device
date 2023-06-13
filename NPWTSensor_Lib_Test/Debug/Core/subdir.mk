################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/HX710B.c \
../Core/dwt.c 

OBJS += \
./Core/HX710B.o \
./Core/dwt.o 

C_DEPS += \
./Core/HX710B.d \
./Core/dwt.d 


# Each subdirectory must supply rules for building sources it contributes
Core/%.o Core/%.su: ../Core/%.c Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I"C:/Users/Niko/Desktop/NPWT Device/Software/NPWTSensor_Lib_Test/Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM3" -I"C:/Users/Niko/Desktop/NPWT Device/Software/NPWTSensor_Lib_Test/Middlewares/Third_Party/FreeRTOS/Source/portable" -I"C:/Users/Niko/Desktop/NPWT Device/Software/NPWTSensor_Lib_Test/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/Niko/Desktop/NPWT Device/Software/NPWTSensor_Lib_Test/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2" -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core

clean-Core:
	-$(RM) ./Core/HX710B.d ./Core/HX710B.o ./Core/HX710B.su ./Core/dwt.d ./Core/dwt.o ./Core/dwt.su

.PHONY: clean-Core

