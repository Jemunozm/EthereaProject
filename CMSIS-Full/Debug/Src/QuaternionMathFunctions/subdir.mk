################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/QuaternionMathFunctions/QuaternionMathFunctions.c \
../Src/QuaternionMathFunctions/arm_quaternion2rotation_f32.c \
../Src/QuaternionMathFunctions/arm_quaternion_conjugate_f32.c \
../Src/QuaternionMathFunctions/arm_quaternion_inverse_f32.c \
../Src/QuaternionMathFunctions/arm_quaternion_norm_f32.c \
../Src/QuaternionMathFunctions/arm_quaternion_normalize_f32.c \
../Src/QuaternionMathFunctions/arm_quaternion_product_f32.c \
../Src/QuaternionMathFunctions/arm_quaternion_product_single_f32.c \
../Src/QuaternionMathFunctions/arm_rotation2quaternion_f32.c 

OBJS += \
./Src/QuaternionMathFunctions/QuaternionMathFunctions.o \
./Src/QuaternionMathFunctions/arm_quaternion2rotation_f32.o \
./Src/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o \
./Src/QuaternionMathFunctions/arm_quaternion_inverse_f32.o \
./Src/QuaternionMathFunctions/arm_quaternion_norm_f32.o \
./Src/QuaternionMathFunctions/arm_quaternion_normalize_f32.o \
./Src/QuaternionMathFunctions/arm_quaternion_product_f32.o \
./Src/QuaternionMathFunctions/arm_quaternion_product_single_f32.o \
./Src/QuaternionMathFunctions/arm_rotation2quaternion_f32.o 

C_DEPS += \
./Src/QuaternionMathFunctions/QuaternionMathFunctions.d \
./Src/QuaternionMathFunctions/arm_quaternion2rotation_f32.d \
./Src/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d \
./Src/QuaternionMathFunctions/arm_quaternion_inverse_f32.d \
./Src/QuaternionMathFunctions/arm_quaternion_norm_f32.d \
./Src/QuaternionMathFunctions/arm_quaternion_normalize_f32.d \
./Src/QuaternionMathFunctions/arm_quaternion_product_f32.d \
./Src/QuaternionMathFunctions/arm_quaternion_product_single_f32.d \
./Src/QuaternionMathFunctions/arm_rotation2quaternion_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Src/QuaternionMathFunctions/%.o Src/QuaternionMathFunctions/%.su: ../Src/QuaternionMathFunctions/%.c Src/QuaternionMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-QuaternionMathFunctions

clean-Src-2f-QuaternionMathFunctions:
	-$(RM) ./Src/QuaternionMathFunctions/QuaternionMathFunctions.d ./Src/QuaternionMathFunctions/QuaternionMathFunctions.o ./Src/QuaternionMathFunctions/QuaternionMathFunctions.su ./Src/QuaternionMathFunctions/arm_quaternion2rotation_f32.d ./Src/QuaternionMathFunctions/arm_quaternion2rotation_f32.o ./Src/QuaternionMathFunctions/arm_quaternion2rotation_f32.su ./Src/QuaternionMathFunctions/arm_quaternion_conjugate_f32.d ./Src/QuaternionMathFunctions/arm_quaternion_conjugate_f32.o ./Src/QuaternionMathFunctions/arm_quaternion_conjugate_f32.su ./Src/QuaternionMathFunctions/arm_quaternion_inverse_f32.d ./Src/QuaternionMathFunctions/arm_quaternion_inverse_f32.o ./Src/QuaternionMathFunctions/arm_quaternion_inverse_f32.su ./Src/QuaternionMathFunctions/arm_quaternion_norm_f32.d ./Src/QuaternionMathFunctions/arm_quaternion_norm_f32.o ./Src/QuaternionMathFunctions/arm_quaternion_norm_f32.su ./Src/QuaternionMathFunctions/arm_quaternion_normalize_f32.d ./Src/QuaternionMathFunctions/arm_quaternion_normalize_f32.o ./Src/QuaternionMathFunctions/arm_quaternion_normalize_f32.su ./Src/QuaternionMathFunctions/arm_quaternion_product_f32.d ./Src/QuaternionMathFunctions/arm_quaternion_product_f32.o ./Src/QuaternionMathFunctions/arm_quaternion_product_f32.su ./Src/QuaternionMathFunctions/arm_quaternion_product_single_f32.d ./Src/QuaternionMathFunctions/arm_quaternion_product_single_f32.o ./Src/QuaternionMathFunctions/arm_quaternion_product_single_f32.su ./Src/QuaternionMathFunctions/arm_rotation2quaternion_f32.d ./Src/QuaternionMathFunctions/arm_rotation2quaternion_f32.o ./Src/QuaternionMathFunctions/arm_rotation2quaternion_f32.su

.PHONY: clean-Src-2f-QuaternionMathFunctions

