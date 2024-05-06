################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ControllerFunctions/ControllerFunctions.c \
../Src/ControllerFunctions/arm_pid_init_f32.c \
../Src/ControllerFunctions/arm_pid_init_q15.c \
../Src/ControllerFunctions/arm_pid_init_q31.c \
../Src/ControllerFunctions/arm_pid_reset_f32.c \
../Src/ControllerFunctions/arm_pid_reset_q15.c \
../Src/ControllerFunctions/arm_pid_reset_q31.c \
../Src/ControllerFunctions/arm_sin_cos_f32.c \
../Src/ControllerFunctions/arm_sin_cos_q31.c 

OBJS += \
./Src/ControllerFunctions/ControllerFunctions.o \
./Src/ControllerFunctions/arm_pid_init_f32.o \
./Src/ControllerFunctions/arm_pid_init_q15.o \
./Src/ControllerFunctions/arm_pid_init_q31.o \
./Src/ControllerFunctions/arm_pid_reset_f32.o \
./Src/ControllerFunctions/arm_pid_reset_q15.o \
./Src/ControllerFunctions/arm_pid_reset_q31.o \
./Src/ControllerFunctions/arm_sin_cos_f32.o \
./Src/ControllerFunctions/arm_sin_cos_q31.o 

C_DEPS += \
./Src/ControllerFunctions/ControllerFunctions.d \
./Src/ControllerFunctions/arm_pid_init_f32.d \
./Src/ControllerFunctions/arm_pid_init_q15.d \
./Src/ControllerFunctions/arm_pid_init_q31.d \
./Src/ControllerFunctions/arm_pid_reset_f32.d \
./Src/ControllerFunctions/arm_pid_reset_q15.d \
./Src/ControllerFunctions/arm_pid_reset_q31.d \
./Src/ControllerFunctions/arm_sin_cos_f32.d \
./Src/ControllerFunctions/arm_sin_cos_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Src/ControllerFunctions/%.o Src/ControllerFunctions/%.su: ../Src/ControllerFunctions/%.c Src/ControllerFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/ingfisica/JesusE/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ingfisica/JesusE/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/home/ingfisica/GitHub/EthereaProject/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-ControllerFunctions

clean-Src-2f-ControllerFunctions:
	-$(RM) ./Src/ControllerFunctions/ControllerFunctions.d ./Src/ControllerFunctions/ControllerFunctions.o ./Src/ControllerFunctions/ControllerFunctions.su ./Src/ControllerFunctions/arm_pid_init_f32.d ./Src/ControllerFunctions/arm_pid_init_f32.o ./Src/ControllerFunctions/arm_pid_init_f32.su ./Src/ControllerFunctions/arm_pid_init_q15.d ./Src/ControllerFunctions/arm_pid_init_q15.o ./Src/ControllerFunctions/arm_pid_init_q15.su ./Src/ControllerFunctions/arm_pid_init_q31.d ./Src/ControllerFunctions/arm_pid_init_q31.o ./Src/ControllerFunctions/arm_pid_init_q31.su ./Src/ControllerFunctions/arm_pid_reset_f32.d ./Src/ControllerFunctions/arm_pid_reset_f32.o ./Src/ControllerFunctions/arm_pid_reset_f32.su ./Src/ControllerFunctions/arm_pid_reset_q15.d ./Src/ControllerFunctions/arm_pid_reset_q15.o ./Src/ControllerFunctions/arm_pid_reset_q15.su ./Src/ControllerFunctions/arm_pid_reset_q31.d ./Src/ControllerFunctions/arm_pid_reset_q31.o ./Src/ControllerFunctions/arm_pid_reset_q31.su ./Src/ControllerFunctions/arm_sin_cos_f32.d ./Src/ControllerFunctions/arm_sin_cos_f32.o ./Src/ControllerFunctions/arm_sin_cos_f32.su ./Src/ControllerFunctions/arm_sin_cos_q31.d ./Src/ControllerFunctions/arm_sin_cos_q31.o ./Src/ControllerFunctions/arm_sin_cos_q31.su

.PHONY: clean-Src-2f-ControllerFunctions

