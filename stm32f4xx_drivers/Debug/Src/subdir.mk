################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/012i2c_slave_tx_string.c 

OBJS += \
./Src/012i2c_slave_tx_string.o 

C_DEPS += \
./Src/012i2c_slave_tx_string.d 


# Each subdirectory must supply rules for building sources it contributes
Src/012i2c_slave_tx_string.o: ../Src/012i2c_slave_tx_string.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/thaithinhtran/Downloads/Documents_MCU1/MCU1/stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/012i2c_slave_tx_string.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

