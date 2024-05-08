################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MatrixFunctions/MatrixFunctions.c \
../Src/MatrixFunctions/MatrixFunctionsF16.c \
../Src/MatrixFunctions/arm_mat_add_f16.c \
../Src/MatrixFunctions/arm_mat_add_f32.c \
../Src/MatrixFunctions/arm_mat_add_q15.c \
../Src/MatrixFunctions/arm_mat_add_q31.c \
../Src/MatrixFunctions/arm_mat_cholesky_f16.c \
../Src/MatrixFunctions/arm_mat_cholesky_f32.c \
../Src/MatrixFunctions/arm_mat_cholesky_f64.c \
../Src/MatrixFunctions/arm_mat_cmplx_mult_f16.c \
../Src/MatrixFunctions/arm_mat_cmplx_mult_f32.c \
../Src/MatrixFunctions/arm_mat_cmplx_mult_q15.c \
../Src/MatrixFunctions/arm_mat_cmplx_mult_q31.c \
../Src/MatrixFunctions/arm_mat_cmplx_trans_f16.c \
../Src/MatrixFunctions/arm_mat_cmplx_trans_f32.c \
../Src/MatrixFunctions/arm_mat_cmplx_trans_q15.c \
../Src/MatrixFunctions/arm_mat_cmplx_trans_q31.c \
../Src/MatrixFunctions/arm_mat_init_f16.c \
../Src/MatrixFunctions/arm_mat_init_f32.c \
../Src/MatrixFunctions/arm_mat_init_q15.c \
../Src/MatrixFunctions/arm_mat_init_q31.c \
../Src/MatrixFunctions/arm_mat_inverse_f16.c \
../Src/MatrixFunctions/arm_mat_inverse_f32.c \
../Src/MatrixFunctions/arm_mat_inverse_f64.c \
../Src/MatrixFunctions/arm_mat_ldlt_f32.c \
../Src/MatrixFunctions/arm_mat_ldlt_f64.c \
../Src/MatrixFunctions/arm_mat_mult_f16.c \
../Src/MatrixFunctions/arm_mat_mult_f32.c \
../Src/MatrixFunctions/arm_mat_mult_f64.c \
../Src/MatrixFunctions/arm_mat_mult_fast_q15.c \
../Src/MatrixFunctions/arm_mat_mult_fast_q31.c \
../Src/MatrixFunctions/arm_mat_mult_opt_q31.c \
../Src/MatrixFunctions/arm_mat_mult_q15.c \
../Src/MatrixFunctions/arm_mat_mult_q31.c \
../Src/MatrixFunctions/arm_mat_mult_q7.c \
../Src/MatrixFunctions/arm_mat_scale_f16.c \
../Src/MatrixFunctions/arm_mat_scale_f32.c \
../Src/MatrixFunctions/arm_mat_scale_q15.c \
../Src/MatrixFunctions/arm_mat_scale_q31.c \
../Src/MatrixFunctions/arm_mat_solve_lower_triangular_f16.c \
../Src/MatrixFunctions/arm_mat_solve_lower_triangular_f32.c \
../Src/MatrixFunctions/arm_mat_solve_lower_triangular_f64.c \
../Src/MatrixFunctions/arm_mat_solve_upper_triangular_f16.c \
../Src/MatrixFunctions/arm_mat_solve_upper_triangular_f32.c \
../Src/MatrixFunctions/arm_mat_solve_upper_triangular_f64.c \
../Src/MatrixFunctions/arm_mat_sub_f16.c \
../Src/MatrixFunctions/arm_mat_sub_f32.c \
../Src/MatrixFunctions/arm_mat_sub_f64.c \
../Src/MatrixFunctions/arm_mat_sub_q15.c \
../Src/MatrixFunctions/arm_mat_sub_q31.c \
../Src/MatrixFunctions/arm_mat_trans_f16.c \
../Src/MatrixFunctions/arm_mat_trans_f32.c \
../Src/MatrixFunctions/arm_mat_trans_f64.c \
../Src/MatrixFunctions/arm_mat_trans_q15.c \
../Src/MatrixFunctions/arm_mat_trans_q31.c \
../Src/MatrixFunctions/arm_mat_trans_q7.c \
../Src/MatrixFunctions/arm_mat_vec_mult_f16.c \
../Src/MatrixFunctions/arm_mat_vec_mult_f32.c \
../Src/MatrixFunctions/arm_mat_vec_mult_q15.c \
../Src/MatrixFunctions/arm_mat_vec_mult_q31.c \
../Src/MatrixFunctions/arm_mat_vec_mult_q7.c 

OBJS += \
./Src/MatrixFunctions/MatrixFunctions.o \
./Src/MatrixFunctions/MatrixFunctionsF16.o \
./Src/MatrixFunctions/arm_mat_add_f16.o \
./Src/MatrixFunctions/arm_mat_add_f32.o \
./Src/MatrixFunctions/arm_mat_add_q15.o \
./Src/MatrixFunctions/arm_mat_add_q31.o \
./Src/MatrixFunctions/arm_mat_cholesky_f16.o \
./Src/MatrixFunctions/arm_mat_cholesky_f32.o \
./Src/MatrixFunctions/arm_mat_cholesky_f64.o \
./Src/MatrixFunctions/arm_mat_cmplx_mult_f16.o \
./Src/MatrixFunctions/arm_mat_cmplx_mult_f32.o \
./Src/MatrixFunctions/arm_mat_cmplx_mult_q15.o \
./Src/MatrixFunctions/arm_mat_cmplx_mult_q31.o \
./Src/MatrixFunctions/arm_mat_cmplx_trans_f16.o \
./Src/MatrixFunctions/arm_mat_cmplx_trans_f32.o \
./Src/MatrixFunctions/arm_mat_cmplx_trans_q15.o \
./Src/MatrixFunctions/arm_mat_cmplx_trans_q31.o \
./Src/MatrixFunctions/arm_mat_init_f16.o \
./Src/MatrixFunctions/arm_mat_init_f32.o \
./Src/MatrixFunctions/arm_mat_init_q15.o \
./Src/MatrixFunctions/arm_mat_init_q31.o \
./Src/MatrixFunctions/arm_mat_inverse_f16.o \
./Src/MatrixFunctions/arm_mat_inverse_f32.o \
./Src/MatrixFunctions/arm_mat_inverse_f64.o \
./Src/MatrixFunctions/arm_mat_ldlt_f32.o \
./Src/MatrixFunctions/arm_mat_ldlt_f64.o \
./Src/MatrixFunctions/arm_mat_mult_f16.o \
./Src/MatrixFunctions/arm_mat_mult_f32.o \
./Src/MatrixFunctions/arm_mat_mult_f64.o \
./Src/MatrixFunctions/arm_mat_mult_fast_q15.o \
./Src/MatrixFunctions/arm_mat_mult_fast_q31.o \
./Src/MatrixFunctions/arm_mat_mult_opt_q31.o \
./Src/MatrixFunctions/arm_mat_mult_q15.o \
./Src/MatrixFunctions/arm_mat_mult_q31.o \
./Src/MatrixFunctions/arm_mat_mult_q7.o \
./Src/MatrixFunctions/arm_mat_scale_f16.o \
./Src/MatrixFunctions/arm_mat_scale_f32.o \
./Src/MatrixFunctions/arm_mat_scale_q15.o \
./Src/MatrixFunctions/arm_mat_scale_q31.o \
./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o \
./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o \
./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o \
./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o \
./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o \
./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o \
./Src/MatrixFunctions/arm_mat_sub_f16.o \
./Src/MatrixFunctions/arm_mat_sub_f32.o \
./Src/MatrixFunctions/arm_mat_sub_f64.o \
./Src/MatrixFunctions/arm_mat_sub_q15.o \
./Src/MatrixFunctions/arm_mat_sub_q31.o \
./Src/MatrixFunctions/arm_mat_trans_f16.o \
./Src/MatrixFunctions/arm_mat_trans_f32.o \
./Src/MatrixFunctions/arm_mat_trans_f64.o \
./Src/MatrixFunctions/arm_mat_trans_q15.o \
./Src/MatrixFunctions/arm_mat_trans_q31.o \
./Src/MatrixFunctions/arm_mat_trans_q7.o \
./Src/MatrixFunctions/arm_mat_vec_mult_f16.o \
./Src/MatrixFunctions/arm_mat_vec_mult_f32.o \
./Src/MatrixFunctions/arm_mat_vec_mult_q15.o \
./Src/MatrixFunctions/arm_mat_vec_mult_q31.o \
./Src/MatrixFunctions/arm_mat_vec_mult_q7.o 

C_DEPS += \
./Src/MatrixFunctions/MatrixFunctions.d \
./Src/MatrixFunctions/MatrixFunctionsF16.d \
./Src/MatrixFunctions/arm_mat_add_f16.d \
./Src/MatrixFunctions/arm_mat_add_f32.d \
./Src/MatrixFunctions/arm_mat_add_q15.d \
./Src/MatrixFunctions/arm_mat_add_q31.d \
./Src/MatrixFunctions/arm_mat_cholesky_f16.d \
./Src/MatrixFunctions/arm_mat_cholesky_f32.d \
./Src/MatrixFunctions/arm_mat_cholesky_f64.d \
./Src/MatrixFunctions/arm_mat_cmplx_mult_f16.d \
./Src/MatrixFunctions/arm_mat_cmplx_mult_f32.d \
./Src/MatrixFunctions/arm_mat_cmplx_mult_q15.d \
./Src/MatrixFunctions/arm_mat_cmplx_mult_q31.d \
./Src/MatrixFunctions/arm_mat_cmplx_trans_f16.d \
./Src/MatrixFunctions/arm_mat_cmplx_trans_f32.d \
./Src/MatrixFunctions/arm_mat_cmplx_trans_q15.d \
./Src/MatrixFunctions/arm_mat_cmplx_trans_q31.d \
./Src/MatrixFunctions/arm_mat_init_f16.d \
./Src/MatrixFunctions/arm_mat_init_f32.d \
./Src/MatrixFunctions/arm_mat_init_q15.d \
./Src/MatrixFunctions/arm_mat_init_q31.d \
./Src/MatrixFunctions/arm_mat_inverse_f16.d \
./Src/MatrixFunctions/arm_mat_inverse_f32.d \
./Src/MatrixFunctions/arm_mat_inverse_f64.d \
./Src/MatrixFunctions/arm_mat_ldlt_f32.d \
./Src/MatrixFunctions/arm_mat_ldlt_f64.d \
./Src/MatrixFunctions/arm_mat_mult_f16.d \
./Src/MatrixFunctions/arm_mat_mult_f32.d \
./Src/MatrixFunctions/arm_mat_mult_f64.d \
./Src/MatrixFunctions/arm_mat_mult_fast_q15.d \
./Src/MatrixFunctions/arm_mat_mult_fast_q31.d \
./Src/MatrixFunctions/arm_mat_mult_opt_q31.d \
./Src/MatrixFunctions/arm_mat_mult_q15.d \
./Src/MatrixFunctions/arm_mat_mult_q31.d \
./Src/MatrixFunctions/arm_mat_mult_q7.d \
./Src/MatrixFunctions/arm_mat_scale_f16.d \
./Src/MatrixFunctions/arm_mat_scale_f32.d \
./Src/MatrixFunctions/arm_mat_scale_q15.d \
./Src/MatrixFunctions/arm_mat_scale_q31.d \
./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d \
./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d \
./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d \
./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d \
./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d \
./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d \
./Src/MatrixFunctions/arm_mat_sub_f16.d \
./Src/MatrixFunctions/arm_mat_sub_f32.d \
./Src/MatrixFunctions/arm_mat_sub_f64.d \
./Src/MatrixFunctions/arm_mat_sub_q15.d \
./Src/MatrixFunctions/arm_mat_sub_q31.d \
./Src/MatrixFunctions/arm_mat_trans_f16.d \
./Src/MatrixFunctions/arm_mat_trans_f32.d \
./Src/MatrixFunctions/arm_mat_trans_f64.d \
./Src/MatrixFunctions/arm_mat_trans_q15.d \
./Src/MatrixFunctions/arm_mat_trans_q31.d \
./Src/MatrixFunctions/arm_mat_trans_q7.d \
./Src/MatrixFunctions/arm_mat_vec_mult_f16.d \
./Src/MatrixFunctions/arm_mat_vec_mult_f32.d \
./Src/MatrixFunctions/arm_mat_vec_mult_q15.d \
./Src/MatrixFunctions/arm_mat_vec_mult_q31.d \
./Src/MatrixFunctions/arm_mat_vec_mult_q7.d 


# Each subdirectory must supply rules for building sources it contributes
Src/MatrixFunctions/%.o Src/MatrixFunctions/%.su: ../Src/MatrixFunctions/%.c Src/MatrixFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-MatrixFunctions

clean-Src-2f-MatrixFunctions:
	-$(RM) ./Src/MatrixFunctions/MatrixFunctions.d ./Src/MatrixFunctions/MatrixFunctions.o ./Src/MatrixFunctions/MatrixFunctions.su ./Src/MatrixFunctions/MatrixFunctionsF16.d ./Src/MatrixFunctions/MatrixFunctionsF16.o ./Src/MatrixFunctions/MatrixFunctionsF16.su ./Src/MatrixFunctions/arm_mat_add_f16.d ./Src/MatrixFunctions/arm_mat_add_f16.o ./Src/MatrixFunctions/arm_mat_add_f16.su ./Src/MatrixFunctions/arm_mat_add_f32.d ./Src/MatrixFunctions/arm_mat_add_f32.o ./Src/MatrixFunctions/arm_mat_add_f32.su ./Src/MatrixFunctions/arm_mat_add_q15.d ./Src/MatrixFunctions/arm_mat_add_q15.o ./Src/MatrixFunctions/arm_mat_add_q15.su ./Src/MatrixFunctions/arm_mat_add_q31.d ./Src/MatrixFunctions/arm_mat_add_q31.o ./Src/MatrixFunctions/arm_mat_add_q31.su ./Src/MatrixFunctions/arm_mat_cholesky_f16.d ./Src/MatrixFunctions/arm_mat_cholesky_f16.o ./Src/MatrixFunctions/arm_mat_cholesky_f16.su ./Src/MatrixFunctions/arm_mat_cholesky_f32.d ./Src/MatrixFunctions/arm_mat_cholesky_f32.o ./Src/MatrixFunctions/arm_mat_cholesky_f32.su ./Src/MatrixFunctions/arm_mat_cholesky_f64.d ./Src/MatrixFunctions/arm_mat_cholesky_f64.o ./Src/MatrixFunctions/arm_mat_cholesky_f64.su ./Src/MatrixFunctions/arm_mat_cmplx_mult_f16.d ./Src/MatrixFunctions/arm_mat_cmplx_mult_f16.o ./Src/MatrixFunctions/arm_mat_cmplx_mult_f16.su ./Src/MatrixFunctions/arm_mat_cmplx_mult_f32.d ./Src/MatrixFunctions/arm_mat_cmplx_mult_f32.o ./Src/MatrixFunctions/arm_mat_cmplx_mult_f32.su ./Src/MatrixFunctions/arm_mat_cmplx_mult_q15.d ./Src/MatrixFunctions/arm_mat_cmplx_mult_q15.o ./Src/MatrixFunctions/arm_mat_cmplx_mult_q15.su ./Src/MatrixFunctions/arm_mat_cmplx_mult_q31.d ./Src/MatrixFunctions/arm_mat_cmplx_mult_q31.o ./Src/MatrixFunctions/arm_mat_cmplx_mult_q31.su ./Src/MatrixFunctions/arm_mat_cmplx_trans_f16.d ./Src/MatrixFunctions/arm_mat_cmplx_trans_f16.o ./Src/MatrixFunctions/arm_mat_cmplx_trans_f16.su ./Src/MatrixFunctions/arm_mat_cmplx_trans_f32.d ./Src/MatrixFunctions/arm_mat_cmplx_trans_f32.o ./Src/MatrixFunctions/arm_mat_cmplx_trans_f32.su ./Src/MatrixFunctions/arm_mat_cmplx_trans_q15.d ./Src/MatrixFunctions/arm_mat_cmplx_trans_q15.o ./Src/MatrixFunctions/arm_mat_cmplx_trans_q15.su ./Src/MatrixFunctions/arm_mat_cmplx_trans_q31.d ./Src/MatrixFunctions/arm_mat_cmplx_trans_q31.o ./Src/MatrixFunctions/arm_mat_cmplx_trans_q31.su ./Src/MatrixFunctions/arm_mat_init_f16.d ./Src/MatrixFunctions/arm_mat_init_f16.o ./Src/MatrixFunctions/arm_mat_init_f16.su ./Src/MatrixFunctions/arm_mat_init_f32.d ./Src/MatrixFunctions/arm_mat_init_f32.o ./Src/MatrixFunctions/arm_mat_init_f32.su ./Src/MatrixFunctions/arm_mat_init_q15.d ./Src/MatrixFunctions/arm_mat_init_q15.o ./Src/MatrixFunctions/arm_mat_init_q15.su ./Src/MatrixFunctions/arm_mat_init_q31.d ./Src/MatrixFunctions/arm_mat_init_q31.o ./Src/MatrixFunctions/arm_mat_init_q31.su ./Src/MatrixFunctions/arm_mat_inverse_f16.d ./Src/MatrixFunctions/arm_mat_inverse_f16.o ./Src/MatrixFunctions/arm_mat_inverse_f16.su ./Src/MatrixFunctions/arm_mat_inverse_f32.d ./Src/MatrixFunctions/arm_mat_inverse_f32.o ./Src/MatrixFunctions/arm_mat_inverse_f32.su ./Src/MatrixFunctions/arm_mat_inverse_f64.d ./Src/MatrixFunctions/arm_mat_inverse_f64.o ./Src/MatrixFunctions/arm_mat_inverse_f64.su ./Src/MatrixFunctions/arm_mat_ldlt_f32.d ./Src/MatrixFunctions/arm_mat_ldlt_f32.o ./Src/MatrixFunctions/arm_mat_ldlt_f32.su ./Src/MatrixFunctions/arm_mat_ldlt_f64.d ./Src/MatrixFunctions/arm_mat_ldlt_f64.o ./Src/MatrixFunctions/arm_mat_ldlt_f64.su ./Src/MatrixFunctions/arm_mat_mult_f16.d ./Src/MatrixFunctions/arm_mat_mult_f16.o ./Src/MatrixFunctions/arm_mat_mult_f16.su ./Src/MatrixFunctions/arm_mat_mult_f32.d ./Src/MatrixFunctions/arm_mat_mult_f32.o ./Src/MatrixFunctions/arm_mat_mult_f32.su ./Src/MatrixFunctions/arm_mat_mult_f64.d ./Src/MatrixFunctions/arm_mat_mult_f64.o ./Src/MatrixFunctions/arm_mat_mult_f64.su ./Src/MatrixFunctions/arm_mat_mult_fast_q15.d ./Src/MatrixFunctions/arm_mat_mult_fast_q15.o ./Src/MatrixFunctions/arm_mat_mult_fast_q15.su ./Src/MatrixFunctions/arm_mat_mult_fast_q31.d ./Src/MatrixFunctions/arm_mat_mult_fast_q31.o ./Src/MatrixFunctions/arm_mat_mult_fast_q31.su ./Src/MatrixFunctions/arm_mat_mult_opt_q31.d ./Src/MatrixFunctions/arm_mat_mult_opt_q31.o ./Src/MatrixFunctions/arm_mat_mult_opt_q31.su ./Src/MatrixFunctions/arm_mat_mult_q15.d ./Src/MatrixFunctions/arm_mat_mult_q15.o ./Src/MatrixFunctions/arm_mat_mult_q15.su ./Src/MatrixFunctions/arm_mat_mult_q31.d ./Src/MatrixFunctions/arm_mat_mult_q31.o ./Src/MatrixFunctions/arm_mat_mult_q31.su ./Src/MatrixFunctions/arm_mat_mult_q7.d ./Src/MatrixFunctions/arm_mat_mult_q7.o ./Src/MatrixFunctions/arm_mat_mult_q7.su ./Src/MatrixFunctions/arm_mat_scale_f16.d ./Src/MatrixFunctions/arm_mat_scale_f16.o ./Src/MatrixFunctions/arm_mat_scale_f16.su ./Src/MatrixFunctions/arm_mat_scale_f32.d ./Src/MatrixFunctions/arm_mat_scale_f32.o ./Src/MatrixFunctions/arm_mat_scale_f32.su ./Src/MatrixFunctions/arm_mat_scale_q15.d ./Src/MatrixFunctions/arm_mat_scale_q15.o ./Src/MatrixFunctions/arm_mat_scale_q15.su ./Src/MatrixFunctions/arm_mat_scale_q31.d ./Src/MatrixFunctions/arm_mat_scale_q31.o ./Src/MatrixFunctions/arm_mat_scale_q31.su ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f16.d ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f16.o ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f16.su ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f32.d ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f32.o ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f32.su ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f64.d ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f64.o ./Src/MatrixFunctions/arm_mat_solve_lower_triangular_f64.su ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f16.d ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f16.o ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f16.su ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f32.d ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f32.o ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f32.su
	-$(RM) ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f64.d ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f64.o ./Src/MatrixFunctions/arm_mat_solve_upper_triangular_f64.su ./Src/MatrixFunctions/arm_mat_sub_f16.d ./Src/MatrixFunctions/arm_mat_sub_f16.o ./Src/MatrixFunctions/arm_mat_sub_f16.su ./Src/MatrixFunctions/arm_mat_sub_f32.d ./Src/MatrixFunctions/arm_mat_sub_f32.o ./Src/MatrixFunctions/arm_mat_sub_f32.su ./Src/MatrixFunctions/arm_mat_sub_f64.d ./Src/MatrixFunctions/arm_mat_sub_f64.o ./Src/MatrixFunctions/arm_mat_sub_f64.su ./Src/MatrixFunctions/arm_mat_sub_q15.d ./Src/MatrixFunctions/arm_mat_sub_q15.o ./Src/MatrixFunctions/arm_mat_sub_q15.su ./Src/MatrixFunctions/arm_mat_sub_q31.d ./Src/MatrixFunctions/arm_mat_sub_q31.o ./Src/MatrixFunctions/arm_mat_sub_q31.su ./Src/MatrixFunctions/arm_mat_trans_f16.d ./Src/MatrixFunctions/arm_mat_trans_f16.o ./Src/MatrixFunctions/arm_mat_trans_f16.su ./Src/MatrixFunctions/arm_mat_trans_f32.d ./Src/MatrixFunctions/arm_mat_trans_f32.o ./Src/MatrixFunctions/arm_mat_trans_f32.su ./Src/MatrixFunctions/arm_mat_trans_f64.d ./Src/MatrixFunctions/arm_mat_trans_f64.o ./Src/MatrixFunctions/arm_mat_trans_f64.su ./Src/MatrixFunctions/arm_mat_trans_q15.d ./Src/MatrixFunctions/arm_mat_trans_q15.o ./Src/MatrixFunctions/arm_mat_trans_q15.su ./Src/MatrixFunctions/arm_mat_trans_q31.d ./Src/MatrixFunctions/arm_mat_trans_q31.o ./Src/MatrixFunctions/arm_mat_trans_q31.su ./Src/MatrixFunctions/arm_mat_trans_q7.d ./Src/MatrixFunctions/arm_mat_trans_q7.o ./Src/MatrixFunctions/arm_mat_trans_q7.su ./Src/MatrixFunctions/arm_mat_vec_mult_f16.d ./Src/MatrixFunctions/arm_mat_vec_mult_f16.o ./Src/MatrixFunctions/arm_mat_vec_mult_f16.su ./Src/MatrixFunctions/arm_mat_vec_mult_f32.d ./Src/MatrixFunctions/arm_mat_vec_mult_f32.o ./Src/MatrixFunctions/arm_mat_vec_mult_f32.su ./Src/MatrixFunctions/arm_mat_vec_mult_q15.d ./Src/MatrixFunctions/arm_mat_vec_mult_q15.o ./Src/MatrixFunctions/arm_mat_vec_mult_q15.su ./Src/MatrixFunctions/arm_mat_vec_mult_q31.d ./Src/MatrixFunctions/arm_mat_vec_mult_q31.o ./Src/MatrixFunctions/arm_mat_vec_mult_q31.su ./Src/MatrixFunctions/arm_mat_vec_mult_q7.d ./Src/MatrixFunctions/arm_mat_vec_mult_q7.o ./Src/MatrixFunctions/arm_mat_vec_mult_q7.su

.PHONY: clean-Src-2f-MatrixFunctions

