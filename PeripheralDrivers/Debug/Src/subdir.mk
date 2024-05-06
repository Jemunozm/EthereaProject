################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/AdcDriver.c \
../Src/BasicTimer.c \
../Src/ExtiDriver.c \
../Src/GPIOxDriver.c \
../Src/I2CxDriver.c \
../Src/PLLDriver.c \
../Src/PWMDriver.c \
../Src/RTCxDriver.c \
../Src/SysTickDriver.c \
../Src/USARTxDriver.c 

OBJS += \
./Src/AdcDriver.o \
./Src/BasicTimer.o \
./Src/ExtiDriver.o \
./Src/GPIOxDriver.o \
./Src/I2CxDriver.o \
./Src/PLLDriver.o \
./Src/PWMDriver.o \
./Src/RTCxDriver.o \
./Src/SysTickDriver.o \
./Src/USARTxDriver.o 

C_DEPS += \
./Src/AdcDriver.d \
./Src/BasicTimer.d \
./Src/ExtiDriver.d \
./Src/GPIOxDriver.d \
./Src/I2CxDriver.d \
./Src/PLLDriver.d \
./Src/PWMDriver.d \
./Src/RTCxDriver.d \
./Src/SysTickDriver.d \
./Src/USARTxDriver.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/ingfisica/JesusE/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ingfisica/JesusE/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/home/ingfisica/GitHub/EthereaProject/PeripheralDrivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/AdcDriver.d ./Src/AdcDriver.o ./Src/AdcDriver.su ./Src/BasicTimer.d ./Src/BasicTimer.o ./Src/BasicTimer.su ./Src/ExtiDriver.d ./Src/ExtiDriver.o ./Src/ExtiDriver.su ./Src/GPIOxDriver.d ./Src/GPIOxDriver.o ./Src/GPIOxDriver.su ./Src/I2CxDriver.d ./Src/I2CxDriver.o ./Src/I2CxDriver.su ./Src/PLLDriver.d ./Src/PLLDriver.o ./Src/PLLDriver.su ./Src/PWMDriver.d ./Src/PWMDriver.o ./Src/PWMDriver.su ./Src/RTCxDriver.d ./Src/RTCxDriver.o ./Src/RTCxDriver.su ./Src/SysTickDriver.d ./Src/SysTickDriver.o ./Src/SysTickDriver.su ./Src/USARTxDriver.d ./Src/USARTxDriver.o ./Src/USARTxDriver.su

.PHONY: clean-Src

