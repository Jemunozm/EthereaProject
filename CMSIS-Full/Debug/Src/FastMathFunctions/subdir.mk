################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/FastMathFunctions/FastMathFunctions.c \
../Src/FastMathFunctions/FastMathFunctionsF16.c \
../Src/FastMathFunctions/arm_atan2_f16.c \
../Src/FastMathFunctions/arm_atan2_f32.c \
../Src/FastMathFunctions/arm_atan2_q15.c \
../Src/FastMathFunctions/arm_atan2_q31.c \
../Src/FastMathFunctions/arm_cos_f32.c \
../Src/FastMathFunctions/arm_cos_q15.c \
../Src/FastMathFunctions/arm_cos_q31.c \
../Src/FastMathFunctions/arm_divide_q15.c \
../Src/FastMathFunctions/arm_divide_q31.c \
../Src/FastMathFunctions/arm_sin_f32.c \
../Src/FastMathFunctions/arm_sin_q15.c \
../Src/FastMathFunctions/arm_sin_q31.c \
../Src/FastMathFunctions/arm_sqrt_q15.c \
../Src/FastMathFunctions/arm_sqrt_q31.c \
../Src/FastMathFunctions/arm_vexp_f16.c \
../Src/FastMathFunctions/arm_vexp_f32.c \
../Src/FastMathFunctions/arm_vexp_f64.c \
../Src/FastMathFunctions/arm_vinverse_f16.c \
../Src/FastMathFunctions/arm_vlog_f16.c \
../Src/FastMathFunctions/arm_vlog_f32.c \
../Src/FastMathFunctions/arm_vlog_f64.c \
../Src/FastMathFunctions/arm_vlog_q15.c \
../Src/FastMathFunctions/arm_vlog_q31.c 

OBJS += \
./Src/FastMathFunctions/FastMathFunctions.o \
./Src/FastMathFunctions/FastMathFunctionsF16.o \
./Src/FastMathFunctions/arm_atan2_f16.o \
./Src/FastMathFunctions/arm_atan2_f32.o \
./Src/FastMathFunctions/arm_atan2_q15.o \
./Src/FastMathFunctions/arm_atan2_q31.o \
./Src/FastMathFunctions/arm_cos_f32.o \
./Src/FastMathFunctions/arm_cos_q15.o \
./Src/FastMathFunctions/arm_cos_q31.o \
./Src/FastMathFunctions/arm_divide_q15.o \
./Src/FastMathFunctions/arm_divide_q31.o \
./Src/FastMathFunctions/arm_sin_f32.o \
./Src/FastMathFunctions/arm_sin_q15.o \
./Src/FastMathFunctions/arm_sin_q31.o \
./Src/FastMathFunctions/arm_sqrt_q15.o \
./Src/FastMathFunctions/arm_sqrt_q31.o \
./Src/FastMathFunctions/arm_vexp_f16.o \
./Src/FastMathFunctions/arm_vexp_f32.o \
./Src/FastMathFunctions/arm_vexp_f64.o \
./Src/FastMathFunctions/arm_vinverse_f16.o \
./Src/FastMathFunctions/arm_vlog_f16.o \
./Src/FastMathFunctions/arm_vlog_f32.o \
./Src/FastMathFunctions/arm_vlog_f64.o \
./Src/FastMathFunctions/arm_vlog_q15.o \
./Src/FastMathFunctions/arm_vlog_q31.o 

C_DEPS += \
./Src/FastMathFunctions/FastMathFunctions.d \
./Src/FastMathFunctions/FastMathFunctionsF16.d \
./Src/FastMathFunctions/arm_atan2_f16.d \
./Src/FastMathFunctions/arm_atan2_f32.d \
./Src/FastMathFunctions/arm_atan2_q15.d \
./Src/FastMathFunctions/arm_atan2_q31.d \
./Src/FastMathFunctions/arm_cos_f32.d \
./Src/FastMathFunctions/arm_cos_q15.d \
./Src/FastMathFunctions/arm_cos_q31.d \
./Src/FastMathFunctions/arm_divide_q15.d \
./Src/FastMathFunctions/arm_divide_q31.d \
./Src/FastMathFunctions/arm_sin_f32.d \
./Src/FastMathFunctions/arm_sin_q15.d \
./Src/FastMathFunctions/arm_sin_q31.d \
./Src/FastMathFunctions/arm_sqrt_q15.d \
./Src/FastMathFunctions/arm_sqrt_q31.d \
./Src/FastMathFunctions/arm_vexp_f16.d \
./Src/FastMathFunctions/arm_vexp_f32.d \
./Src/FastMathFunctions/arm_vexp_f64.d \
./Src/FastMathFunctions/arm_vinverse_f16.d \
./Src/FastMathFunctions/arm_vlog_f16.d \
./Src/FastMathFunctions/arm_vlog_f32.d \
./Src/FastMathFunctions/arm_vlog_f64.d \
./Src/FastMathFunctions/arm_vlog_q15.d \
./Src/FastMathFunctions/arm_vlog_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Src/FastMathFunctions/%.o Src/FastMathFunctions/%.su: ../Src/FastMathFunctions/%.c Src/FastMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/ingfisica/JesusE/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/ingfisica/JesusE/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/home/ingfisica/GitHub/EthereaProject/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-FastMathFunctions

clean-Src-2f-FastMathFunctions:
	-$(RM) ./Src/FastMathFunctions/FastMathFunctions.d ./Src/FastMathFunctions/FastMathFunctions.o ./Src/FastMathFunctions/FastMathFunctions.su ./Src/FastMathFunctions/FastMathFunctionsF16.d ./Src/FastMathFunctions/FastMathFunctionsF16.o ./Src/FastMathFunctions/FastMathFunctionsF16.su ./Src/FastMathFunctions/arm_atan2_f16.d ./Src/FastMathFunctions/arm_atan2_f16.o ./Src/FastMathFunctions/arm_atan2_f16.su ./Src/FastMathFunctions/arm_atan2_f32.d ./Src/FastMathFunctions/arm_atan2_f32.o ./Src/FastMathFunctions/arm_atan2_f32.su ./Src/FastMathFunctions/arm_atan2_q15.d ./Src/FastMathFunctions/arm_atan2_q15.o ./Src/FastMathFunctions/arm_atan2_q15.su ./Src/FastMathFunctions/arm_atan2_q31.d ./Src/FastMathFunctions/arm_atan2_q31.o ./Src/FastMathFunctions/arm_atan2_q31.su ./Src/FastMathFunctions/arm_cos_f32.d ./Src/FastMathFunctions/arm_cos_f32.o ./Src/FastMathFunctions/arm_cos_f32.su ./Src/FastMathFunctions/arm_cos_q15.d ./Src/FastMathFunctions/arm_cos_q15.o ./Src/FastMathFunctions/arm_cos_q15.su ./Src/FastMathFunctions/arm_cos_q31.d ./Src/FastMathFunctions/arm_cos_q31.o ./Src/FastMathFunctions/arm_cos_q31.su ./Src/FastMathFunctions/arm_divide_q15.d ./Src/FastMathFunctions/arm_divide_q15.o ./Src/FastMathFunctions/arm_divide_q15.su ./Src/FastMathFunctions/arm_divide_q31.d ./Src/FastMathFunctions/arm_divide_q31.o ./Src/FastMathFunctions/arm_divide_q31.su ./Src/FastMathFunctions/arm_sin_f32.d ./Src/FastMathFunctions/arm_sin_f32.o ./Src/FastMathFunctions/arm_sin_f32.su ./Src/FastMathFunctions/arm_sin_q15.d ./Src/FastMathFunctions/arm_sin_q15.o ./Src/FastMathFunctions/arm_sin_q15.su ./Src/FastMathFunctions/arm_sin_q31.d ./Src/FastMathFunctions/arm_sin_q31.o ./Src/FastMathFunctions/arm_sin_q31.su ./Src/FastMathFunctions/arm_sqrt_q15.d ./Src/FastMathFunctions/arm_sqrt_q15.o ./Src/FastMathFunctions/arm_sqrt_q15.su ./Src/FastMathFunctions/arm_sqrt_q31.d ./Src/FastMathFunctions/arm_sqrt_q31.o ./Src/FastMathFunctions/arm_sqrt_q31.su ./Src/FastMathFunctions/arm_vexp_f16.d ./Src/FastMathFunctions/arm_vexp_f16.o ./Src/FastMathFunctions/arm_vexp_f16.su ./Src/FastMathFunctions/arm_vexp_f32.d ./Src/FastMathFunctions/arm_vexp_f32.o ./Src/FastMathFunctions/arm_vexp_f32.su ./Src/FastMathFunctions/arm_vexp_f64.d ./Src/FastMathFunctions/arm_vexp_f64.o ./Src/FastMathFunctions/arm_vexp_f64.su ./Src/FastMathFunctions/arm_vinverse_f16.d ./Src/FastMathFunctions/arm_vinverse_f16.o ./Src/FastMathFunctions/arm_vinverse_f16.su ./Src/FastMathFunctions/arm_vlog_f16.d ./Src/FastMathFunctions/arm_vlog_f16.o ./Src/FastMathFunctions/arm_vlog_f16.su ./Src/FastMathFunctions/arm_vlog_f32.d ./Src/FastMathFunctions/arm_vlog_f32.o ./Src/FastMathFunctions/arm_vlog_f32.su ./Src/FastMathFunctions/arm_vlog_f64.d ./Src/FastMathFunctions/arm_vlog_f64.o ./Src/FastMathFunctions/arm_vlog_f64.su ./Src/FastMathFunctions/arm_vlog_q15.d ./Src/FastMathFunctions/arm_vlog_q15.o ./Src/FastMathFunctions/arm_vlog_q15.su ./Src/FastMathFunctions/arm_vlog_q31.d ./Src/FastMathFunctions/arm_vlog_q31.o ./Src/FastMathFunctions/arm_vlog_q31.su

.PHONY: clean-Src-2f-FastMathFunctions

