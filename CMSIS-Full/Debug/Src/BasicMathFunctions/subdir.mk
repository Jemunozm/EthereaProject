################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/BasicMathFunctions/BasicMathFunctions.c \
../Src/BasicMathFunctions/BasicMathFunctionsF16.c \
../Src/BasicMathFunctions/arm_abs_f16.c \
../Src/BasicMathFunctions/arm_abs_f32.c \
../Src/BasicMathFunctions/arm_abs_f64.c \
../Src/BasicMathFunctions/arm_abs_q15.c \
../Src/BasicMathFunctions/arm_abs_q31.c \
../Src/BasicMathFunctions/arm_abs_q7.c \
../Src/BasicMathFunctions/arm_add_f16.c \
../Src/BasicMathFunctions/arm_add_f32.c \
../Src/BasicMathFunctions/arm_add_f64.c \
../Src/BasicMathFunctions/arm_add_q15.c \
../Src/BasicMathFunctions/arm_add_q31.c \
../Src/BasicMathFunctions/arm_add_q7.c \
../Src/BasicMathFunctions/arm_and_u16.c \
../Src/BasicMathFunctions/arm_and_u32.c \
../Src/BasicMathFunctions/arm_and_u8.c \
../Src/BasicMathFunctions/arm_clip_f16.c \
../Src/BasicMathFunctions/arm_clip_f32.c \
../Src/BasicMathFunctions/arm_clip_q15.c \
../Src/BasicMathFunctions/arm_clip_q31.c \
../Src/BasicMathFunctions/arm_clip_q7.c \
../Src/BasicMathFunctions/arm_dot_prod_f16.c \
../Src/BasicMathFunctions/arm_dot_prod_f32.c \
../Src/BasicMathFunctions/arm_dot_prod_f64.c \
../Src/BasicMathFunctions/arm_dot_prod_q15.c \
../Src/BasicMathFunctions/arm_dot_prod_q31.c \
../Src/BasicMathFunctions/arm_dot_prod_q7.c \
../Src/BasicMathFunctions/arm_mult_f16.c \
../Src/BasicMathFunctions/arm_mult_f32.c \
../Src/BasicMathFunctions/arm_mult_f64.c \
../Src/BasicMathFunctions/arm_mult_q15.c \
../Src/BasicMathFunctions/arm_mult_q31.c \
../Src/BasicMathFunctions/arm_mult_q7.c \
../Src/BasicMathFunctions/arm_negate_f16.c \
../Src/BasicMathFunctions/arm_negate_f32.c \
../Src/BasicMathFunctions/arm_negate_f64.c \
../Src/BasicMathFunctions/arm_negate_q15.c \
../Src/BasicMathFunctions/arm_negate_q31.c \
../Src/BasicMathFunctions/arm_negate_q7.c \
../Src/BasicMathFunctions/arm_not_u16.c \
../Src/BasicMathFunctions/arm_not_u32.c \
../Src/BasicMathFunctions/arm_not_u8.c \
../Src/BasicMathFunctions/arm_offset_f16.c \
../Src/BasicMathFunctions/arm_offset_f32.c \
../Src/BasicMathFunctions/arm_offset_f64.c \
../Src/BasicMathFunctions/arm_offset_q15.c \
../Src/BasicMathFunctions/arm_offset_q31.c \
../Src/BasicMathFunctions/arm_offset_q7.c \
../Src/BasicMathFunctions/arm_or_u16.c \
../Src/BasicMathFunctions/arm_or_u32.c \
../Src/BasicMathFunctions/arm_or_u8.c \
../Src/BasicMathFunctions/arm_scale_f16.c \
../Src/BasicMathFunctions/arm_scale_f32.c \
../Src/BasicMathFunctions/arm_scale_f64.c \
../Src/BasicMathFunctions/arm_scale_q15.c \
../Src/BasicMathFunctions/arm_scale_q31.c \
../Src/BasicMathFunctions/arm_scale_q7.c \
../Src/BasicMathFunctions/arm_shift_q15.c \
../Src/BasicMathFunctions/arm_shift_q31.c \
../Src/BasicMathFunctions/arm_shift_q7.c \
../Src/BasicMathFunctions/arm_sub_f16.c \
../Src/BasicMathFunctions/arm_sub_f32.c \
../Src/BasicMathFunctions/arm_sub_f64.c \
../Src/BasicMathFunctions/arm_sub_q15.c \
../Src/BasicMathFunctions/arm_sub_q31.c \
../Src/BasicMathFunctions/arm_sub_q7.c \
../Src/BasicMathFunctions/arm_xor_u16.c \
../Src/BasicMathFunctions/arm_xor_u32.c \
../Src/BasicMathFunctions/arm_xor_u8.c 

OBJS += \
./Src/BasicMathFunctions/BasicMathFunctions.o \
./Src/BasicMathFunctions/BasicMathFunctionsF16.o \
./Src/BasicMathFunctions/arm_abs_f16.o \
./Src/BasicMathFunctions/arm_abs_f32.o \
./Src/BasicMathFunctions/arm_abs_f64.o \
./Src/BasicMathFunctions/arm_abs_q15.o \
./Src/BasicMathFunctions/arm_abs_q31.o \
./Src/BasicMathFunctions/arm_abs_q7.o \
./Src/BasicMathFunctions/arm_add_f16.o \
./Src/BasicMathFunctions/arm_add_f32.o \
./Src/BasicMathFunctions/arm_add_f64.o \
./Src/BasicMathFunctions/arm_add_q15.o \
./Src/BasicMathFunctions/arm_add_q31.o \
./Src/BasicMathFunctions/arm_add_q7.o \
./Src/BasicMathFunctions/arm_and_u16.o \
./Src/BasicMathFunctions/arm_and_u32.o \
./Src/BasicMathFunctions/arm_and_u8.o \
./Src/BasicMathFunctions/arm_clip_f16.o \
./Src/BasicMathFunctions/arm_clip_f32.o \
./Src/BasicMathFunctions/arm_clip_q15.o \
./Src/BasicMathFunctions/arm_clip_q31.o \
./Src/BasicMathFunctions/arm_clip_q7.o \
./Src/BasicMathFunctions/arm_dot_prod_f16.o \
./Src/BasicMathFunctions/arm_dot_prod_f32.o \
./Src/BasicMathFunctions/arm_dot_prod_f64.o \
./Src/BasicMathFunctions/arm_dot_prod_q15.o \
./Src/BasicMathFunctions/arm_dot_prod_q31.o \
./Src/BasicMathFunctions/arm_dot_prod_q7.o \
./Src/BasicMathFunctions/arm_mult_f16.o \
./Src/BasicMathFunctions/arm_mult_f32.o \
./Src/BasicMathFunctions/arm_mult_f64.o \
./Src/BasicMathFunctions/arm_mult_q15.o \
./Src/BasicMathFunctions/arm_mult_q31.o \
./Src/BasicMathFunctions/arm_mult_q7.o \
./Src/BasicMathFunctions/arm_negate_f16.o \
./Src/BasicMathFunctions/arm_negate_f32.o \
./Src/BasicMathFunctions/arm_negate_f64.o \
./Src/BasicMathFunctions/arm_negate_q15.o \
./Src/BasicMathFunctions/arm_negate_q31.o \
./Src/BasicMathFunctions/arm_negate_q7.o \
./Src/BasicMathFunctions/arm_not_u16.o \
./Src/BasicMathFunctions/arm_not_u32.o \
./Src/BasicMathFunctions/arm_not_u8.o \
./Src/BasicMathFunctions/arm_offset_f16.o \
./Src/BasicMathFunctions/arm_offset_f32.o \
./Src/BasicMathFunctions/arm_offset_f64.o \
./Src/BasicMathFunctions/arm_offset_q15.o \
./Src/BasicMathFunctions/arm_offset_q31.o \
./Src/BasicMathFunctions/arm_offset_q7.o \
./Src/BasicMathFunctions/arm_or_u16.o \
./Src/BasicMathFunctions/arm_or_u32.o \
./Src/BasicMathFunctions/arm_or_u8.o \
./Src/BasicMathFunctions/arm_scale_f16.o \
./Src/BasicMathFunctions/arm_scale_f32.o \
./Src/BasicMathFunctions/arm_scale_f64.o \
./Src/BasicMathFunctions/arm_scale_q15.o \
./Src/BasicMathFunctions/arm_scale_q31.o \
./Src/BasicMathFunctions/arm_scale_q7.o \
./Src/BasicMathFunctions/arm_shift_q15.o \
./Src/BasicMathFunctions/arm_shift_q31.o \
./Src/BasicMathFunctions/arm_shift_q7.o \
./Src/BasicMathFunctions/arm_sub_f16.o \
./Src/BasicMathFunctions/arm_sub_f32.o \
./Src/BasicMathFunctions/arm_sub_f64.o \
./Src/BasicMathFunctions/arm_sub_q15.o \
./Src/BasicMathFunctions/arm_sub_q31.o \
./Src/BasicMathFunctions/arm_sub_q7.o \
./Src/BasicMathFunctions/arm_xor_u16.o \
./Src/BasicMathFunctions/arm_xor_u32.o \
./Src/BasicMathFunctions/arm_xor_u8.o 

C_DEPS += \
./Src/BasicMathFunctions/BasicMathFunctions.d \
./Src/BasicMathFunctions/BasicMathFunctionsF16.d \
./Src/BasicMathFunctions/arm_abs_f16.d \
./Src/BasicMathFunctions/arm_abs_f32.d \
./Src/BasicMathFunctions/arm_abs_f64.d \
./Src/BasicMathFunctions/arm_abs_q15.d \
./Src/BasicMathFunctions/arm_abs_q31.d \
./Src/BasicMathFunctions/arm_abs_q7.d \
./Src/BasicMathFunctions/arm_add_f16.d \
./Src/BasicMathFunctions/arm_add_f32.d \
./Src/BasicMathFunctions/arm_add_f64.d \
./Src/BasicMathFunctions/arm_add_q15.d \
./Src/BasicMathFunctions/arm_add_q31.d \
./Src/BasicMathFunctions/arm_add_q7.d \
./Src/BasicMathFunctions/arm_and_u16.d \
./Src/BasicMathFunctions/arm_and_u32.d \
./Src/BasicMathFunctions/arm_and_u8.d \
./Src/BasicMathFunctions/arm_clip_f16.d \
./Src/BasicMathFunctions/arm_clip_f32.d \
./Src/BasicMathFunctions/arm_clip_q15.d \
./Src/BasicMathFunctions/arm_clip_q31.d \
./Src/BasicMathFunctions/arm_clip_q7.d \
./Src/BasicMathFunctions/arm_dot_prod_f16.d \
./Src/BasicMathFunctions/arm_dot_prod_f32.d \
./Src/BasicMathFunctions/arm_dot_prod_f64.d \
./Src/BasicMathFunctions/arm_dot_prod_q15.d \
./Src/BasicMathFunctions/arm_dot_prod_q31.d \
./Src/BasicMathFunctions/arm_dot_prod_q7.d \
./Src/BasicMathFunctions/arm_mult_f16.d \
./Src/BasicMathFunctions/arm_mult_f32.d \
./Src/BasicMathFunctions/arm_mult_f64.d \
./Src/BasicMathFunctions/arm_mult_q15.d \
./Src/BasicMathFunctions/arm_mult_q31.d \
./Src/BasicMathFunctions/arm_mult_q7.d \
./Src/BasicMathFunctions/arm_negate_f16.d \
./Src/BasicMathFunctions/arm_negate_f32.d \
./Src/BasicMathFunctions/arm_negate_f64.d \
./Src/BasicMathFunctions/arm_negate_q15.d \
./Src/BasicMathFunctions/arm_negate_q31.d \
./Src/BasicMathFunctions/arm_negate_q7.d \
./Src/BasicMathFunctions/arm_not_u16.d \
./Src/BasicMathFunctions/arm_not_u32.d \
./Src/BasicMathFunctions/arm_not_u8.d \
./Src/BasicMathFunctions/arm_offset_f16.d \
./Src/BasicMathFunctions/arm_offset_f32.d \
./Src/BasicMathFunctions/arm_offset_f64.d \
./Src/BasicMathFunctions/arm_offset_q15.d \
./Src/BasicMathFunctions/arm_offset_q31.d \
./Src/BasicMathFunctions/arm_offset_q7.d \
./Src/BasicMathFunctions/arm_or_u16.d \
./Src/BasicMathFunctions/arm_or_u32.d \
./Src/BasicMathFunctions/arm_or_u8.d \
./Src/BasicMathFunctions/arm_scale_f16.d \
./Src/BasicMathFunctions/arm_scale_f32.d \
./Src/BasicMathFunctions/arm_scale_f64.d \
./Src/BasicMathFunctions/arm_scale_q15.d \
./Src/BasicMathFunctions/arm_scale_q31.d \
./Src/BasicMathFunctions/arm_scale_q7.d \
./Src/BasicMathFunctions/arm_shift_q15.d \
./Src/BasicMathFunctions/arm_shift_q31.d \
./Src/BasicMathFunctions/arm_shift_q7.d \
./Src/BasicMathFunctions/arm_sub_f16.d \
./Src/BasicMathFunctions/arm_sub_f32.d \
./Src/BasicMathFunctions/arm_sub_f64.d \
./Src/BasicMathFunctions/arm_sub_q15.d \
./Src/BasicMathFunctions/arm_sub_q31.d \
./Src/BasicMathFunctions/arm_sub_q7.d \
./Src/BasicMathFunctions/arm_xor_u16.d \
./Src/BasicMathFunctions/arm_xor_u32.d \
./Src/BasicMathFunctions/arm_xor_u8.d 


# Each subdirectory must supply rules for building sources it contributes
Src/BasicMathFunctions/%.o Src/BasicMathFunctions/%.su: ../Src/BasicMathFunctions/%.c Src/BasicMathFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-BasicMathFunctions

clean-Src-2f-BasicMathFunctions:
	-$(RM) ./Src/BasicMathFunctions/BasicMathFunctions.d ./Src/BasicMathFunctions/BasicMathFunctions.o ./Src/BasicMathFunctions/BasicMathFunctions.su ./Src/BasicMathFunctions/BasicMathFunctionsF16.d ./Src/BasicMathFunctions/BasicMathFunctionsF16.o ./Src/BasicMathFunctions/BasicMathFunctionsF16.su ./Src/BasicMathFunctions/arm_abs_f16.d ./Src/BasicMathFunctions/arm_abs_f16.o ./Src/BasicMathFunctions/arm_abs_f16.su ./Src/BasicMathFunctions/arm_abs_f32.d ./Src/BasicMathFunctions/arm_abs_f32.o ./Src/BasicMathFunctions/arm_abs_f32.su ./Src/BasicMathFunctions/arm_abs_f64.d ./Src/BasicMathFunctions/arm_abs_f64.o ./Src/BasicMathFunctions/arm_abs_f64.su ./Src/BasicMathFunctions/arm_abs_q15.d ./Src/BasicMathFunctions/arm_abs_q15.o ./Src/BasicMathFunctions/arm_abs_q15.su ./Src/BasicMathFunctions/arm_abs_q31.d ./Src/BasicMathFunctions/arm_abs_q31.o ./Src/BasicMathFunctions/arm_abs_q31.su ./Src/BasicMathFunctions/arm_abs_q7.d ./Src/BasicMathFunctions/arm_abs_q7.o ./Src/BasicMathFunctions/arm_abs_q7.su ./Src/BasicMathFunctions/arm_add_f16.d ./Src/BasicMathFunctions/arm_add_f16.o ./Src/BasicMathFunctions/arm_add_f16.su ./Src/BasicMathFunctions/arm_add_f32.d ./Src/BasicMathFunctions/arm_add_f32.o ./Src/BasicMathFunctions/arm_add_f32.su ./Src/BasicMathFunctions/arm_add_f64.d ./Src/BasicMathFunctions/arm_add_f64.o ./Src/BasicMathFunctions/arm_add_f64.su ./Src/BasicMathFunctions/arm_add_q15.d ./Src/BasicMathFunctions/arm_add_q15.o ./Src/BasicMathFunctions/arm_add_q15.su ./Src/BasicMathFunctions/arm_add_q31.d ./Src/BasicMathFunctions/arm_add_q31.o ./Src/BasicMathFunctions/arm_add_q31.su ./Src/BasicMathFunctions/arm_add_q7.d ./Src/BasicMathFunctions/arm_add_q7.o ./Src/BasicMathFunctions/arm_add_q7.su ./Src/BasicMathFunctions/arm_and_u16.d ./Src/BasicMathFunctions/arm_and_u16.o ./Src/BasicMathFunctions/arm_and_u16.su ./Src/BasicMathFunctions/arm_and_u32.d ./Src/BasicMathFunctions/arm_and_u32.o ./Src/BasicMathFunctions/arm_and_u32.su ./Src/BasicMathFunctions/arm_and_u8.d ./Src/BasicMathFunctions/arm_and_u8.o ./Src/BasicMathFunctions/arm_and_u8.su ./Src/BasicMathFunctions/arm_clip_f16.d ./Src/BasicMathFunctions/arm_clip_f16.o ./Src/BasicMathFunctions/arm_clip_f16.su ./Src/BasicMathFunctions/arm_clip_f32.d ./Src/BasicMathFunctions/arm_clip_f32.o ./Src/BasicMathFunctions/arm_clip_f32.su ./Src/BasicMathFunctions/arm_clip_q15.d ./Src/BasicMathFunctions/arm_clip_q15.o ./Src/BasicMathFunctions/arm_clip_q15.su ./Src/BasicMathFunctions/arm_clip_q31.d ./Src/BasicMathFunctions/arm_clip_q31.o ./Src/BasicMathFunctions/arm_clip_q31.su ./Src/BasicMathFunctions/arm_clip_q7.d ./Src/BasicMathFunctions/arm_clip_q7.o ./Src/BasicMathFunctions/arm_clip_q7.su ./Src/BasicMathFunctions/arm_dot_prod_f16.d ./Src/BasicMathFunctions/arm_dot_prod_f16.o ./Src/BasicMathFunctions/arm_dot_prod_f16.su ./Src/BasicMathFunctions/arm_dot_prod_f32.d ./Src/BasicMathFunctions/arm_dot_prod_f32.o ./Src/BasicMathFunctions/arm_dot_prod_f32.su ./Src/BasicMathFunctions/arm_dot_prod_f64.d ./Src/BasicMathFunctions/arm_dot_prod_f64.o ./Src/BasicMathFunctions/arm_dot_prod_f64.su ./Src/BasicMathFunctions/arm_dot_prod_q15.d ./Src/BasicMathFunctions/arm_dot_prod_q15.o ./Src/BasicMathFunctions/arm_dot_prod_q15.su ./Src/BasicMathFunctions/arm_dot_prod_q31.d ./Src/BasicMathFunctions/arm_dot_prod_q31.o ./Src/BasicMathFunctions/arm_dot_prod_q31.su ./Src/BasicMathFunctions/arm_dot_prod_q7.d ./Src/BasicMathFunctions/arm_dot_prod_q7.o ./Src/BasicMathFunctions/arm_dot_prod_q7.su ./Src/BasicMathFunctions/arm_mult_f16.d ./Src/BasicMathFunctions/arm_mult_f16.o ./Src/BasicMathFunctions/arm_mult_f16.su ./Src/BasicMathFunctions/arm_mult_f32.d ./Src/BasicMathFunctions/arm_mult_f32.o ./Src/BasicMathFunctions/arm_mult_f32.su ./Src/BasicMathFunctions/arm_mult_f64.d ./Src/BasicMathFunctions/arm_mult_f64.o ./Src/BasicMathFunctions/arm_mult_f64.su ./Src/BasicMathFunctions/arm_mult_q15.d ./Src/BasicMathFunctions/arm_mult_q15.o ./Src/BasicMathFunctions/arm_mult_q15.su ./Src/BasicMathFunctions/arm_mult_q31.d ./Src/BasicMathFunctions/arm_mult_q31.o ./Src/BasicMathFunctions/arm_mult_q31.su ./Src/BasicMathFunctions/arm_mult_q7.d ./Src/BasicMathFunctions/arm_mult_q7.o ./Src/BasicMathFunctions/arm_mult_q7.su ./Src/BasicMathFunctions/arm_negate_f16.d ./Src/BasicMathFunctions/arm_negate_f16.o ./Src/BasicMathFunctions/arm_negate_f16.su ./Src/BasicMathFunctions/arm_negate_f32.d ./Src/BasicMathFunctions/arm_negate_f32.o ./Src/BasicMathFunctions/arm_negate_f32.su ./Src/BasicMathFunctions/arm_negate_f64.d ./Src/BasicMathFunctions/arm_negate_f64.o ./Src/BasicMathFunctions/arm_negate_f64.su ./Src/BasicMathFunctions/arm_negate_q15.d ./Src/BasicMathFunctions/arm_negate_q15.o ./Src/BasicMathFunctions/arm_negate_q15.su ./Src/BasicMathFunctions/arm_negate_q31.d ./Src/BasicMathFunctions/arm_negate_q31.o ./Src/BasicMathFunctions/arm_negate_q31.su ./Src/BasicMathFunctions/arm_negate_q7.d ./Src/BasicMathFunctions/arm_negate_q7.o ./Src/BasicMathFunctions/arm_negate_q7.su ./Src/BasicMathFunctions/arm_not_u16.d ./Src/BasicMathFunctions/arm_not_u16.o ./Src/BasicMathFunctions/arm_not_u16.su ./Src/BasicMathFunctions/arm_not_u32.d ./Src/BasicMathFunctions/arm_not_u32.o ./Src/BasicMathFunctions/arm_not_u32.su ./Src/BasicMathFunctions/arm_not_u8.d ./Src/BasicMathFunctions/arm_not_u8.o ./Src/BasicMathFunctions/arm_not_u8.su ./Src/BasicMathFunctions/arm_offset_f16.d ./Src/BasicMathFunctions/arm_offset_f16.o ./Src/BasicMathFunctions/arm_offset_f16.su ./Src/BasicMathFunctions/arm_offset_f32.d ./Src/BasicMathFunctions/arm_offset_f32.o ./Src/BasicMathFunctions/arm_offset_f32.su ./Src/BasicMathFunctions/arm_offset_f64.d ./Src/BasicMathFunctions/arm_offset_f64.o ./Src/BasicMathFunctions/arm_offset_f64.su ./Src/BasicMathFunctions/arm_offset_q15.d ./Src/BasicMathFunctions/arm_offset_q15.o ./Src/BasicMathFunctions/arm_offset_q15.su ./Src/BasicMathFunctions/arm_offset_q31.d ./Src/BasicMathFunctions/arm_offset_q31.o ./Src/BasicMathFunctions/arm_offset_q31.su ./Src/BasicMathFunctions/arm_offset_q7.d
	-$(RM) ./Src/BasicMathFunctions/arm_offset_q7.o ./Src/BasicMathFunctions/arm_offset_q7.su ./Src/BasicMathFunctions/arm_or_u16.d ./Src/BasicMathFunctions/arm_or_u16.o ./Src/BasicMathFunctions/arm_or_u16.su ./Src/BasicMathFunctions/arm_or_u32.d ./Src/BasicMathFunctions/arm_or_u32.o ./Src/BasicMathFunctions/arm_or_u32.su ./Src/BasicMathFunctions/arm_or_u8.d ./Src/BasicMathFunctions/arm_or_u8.o ./Src/BasicMathFunctions/arm_or_u8.su ./Src/BasicMathFunctions/arm_scale_f16.d ./Src/BasicMathFunctions/arm_scale_f16.o ./Src/BasicMathFunctions/arm_scale_f16.su ./Src/BasicMathFunctions/arm_scale_f32.d ./Src/BasicMathFunctions/arm_scale_f32.o ./Src/BasicMathFunctions/arm_scale_f32.su ./Src/BasicMathFunctions/arm_scale_f64.d ./Src/BasicMathFunctions/arm_scale_f64.o ./Src/BasicMathFunctions/arm_scale_f64.su ./Src/BasicMathFunctions/arm_scale_q15.d ./Src/BasicMathFunctions/arm_scale_q15.o ./Src/BasicMathFunctions/arm_scale_q15.su ./Src/BasicMathFunctions/arm_scale_q31.d ./Src/BasicMathFunctions/arm_scale_q31.o ./Src/BasicMathFunctions/arm_scale_q31.su ./Src/BasicMathFunctions/arm_scale_q7.d ./Src/BasicMathFunctions/arm_scale_q7.o ./Src/BasicMathFunctions/arm_scale_q7.su ./Src/BasicMathFunctions/arm_shift_q15.d ./Src/BasicMathFunctions/arm_shift_q15.o ./Src/BasicMathFunctions/arm_shift_q15.su ./Src/BasicMathFunctions/arm_shift_q31.d ./Src/BasicMathFunctions/arm_shift_q31.o ./Src/BasicMathFunctions/arm_shift_q31.su ./Src/BasicMathFunctions/arm_shift_q7.d ./Src/BasicMathFunctions/arm_shift_q7.o ./Src/BasicMathFunctions/arm_shift_q7.su ./Src/BasicMathFunctions/arm_sub_f16.d ./Src/BasicMathFunctions/arm_sub_f16.o ./Src/BasicMathFunctions/arm_sub_f16.su ./Src/BasicMathFunctions/arm_sub_f32.d ./Src/BasicMathFunctions/arm_sub_f32.o ./Src/BasicMathFunctions/arm_sub_f32.su ./Src/BasicMathFunctions/arm_sub_f64.d ./Src/BasicMathFunctions/arm_sub_f64.o ./Src/BasicMathFunctions/arm_sub_f64.su ./Src/BasicMathFunctions/arm_sub_q15.d ./Src/BasicMathFunctions/arm_sub_q15.o ./Src/BasicMathFunctions/arm_sub_q15.su ./Src/BasicMathFunctions/arm_sub_q31.d ./Src/BasicMathFunctions/arm_sub_q31.o ./Src/BasicMathFunctions/arm_sub_q31.su ./Src/BasicMathFunctions/arm_sub_q7.d ./Src/BasicMathFunctions/arm_sub_q7.o ./Src/BasicMathFunctions/arm_sub_q7.su ./Src/BasicMathFunctions/arm_xor_u16.d ./Src/BasicMathFunctions/arm_xor_u16.o ./Src/BasicMathFunctions/arm_xor_u16.su ./Src/BasicMathFunctions/arm_xor_u32.d ./Src/BasicMathFunctions/arm_xor_u32.o ./Src/BasicMathFunctions/arm_xor_u32.su ./Src/BasicMathFunctions/arm_xor_u8.d ./Src/BasicMathFunctions/arm_xor_u8.o ./Src/BasicMathFunctions/arm_xor_u8.su

.PHONY: clean-Src-2f-BasicMathFunctions

