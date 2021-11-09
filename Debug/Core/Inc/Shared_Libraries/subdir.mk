################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/Shared_Libraries/SX1278.c \
../Core/Inc/Shared_Libraries/aes.c 

OBJS += \
./Core/Inc/Shared_Libraries/SX1278.o \
./Core/Inc/Shared_Libraries/aes.o 

C_DEPS += \
./Core/Inc/Shared_Libraries/SX1278.d \
./Core/Inc/Shared_Libraries/aes.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/Shared_Libraries/%.o: ../Core/Inc/Shared_Libraries/%.c Core/Inc/Shared_Libraries/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

