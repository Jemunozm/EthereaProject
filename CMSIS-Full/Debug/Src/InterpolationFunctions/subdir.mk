################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/InterpolationFunctions/InterpolationFunctions.c \
../Src/InterpolationFunctions/InterpolationFunctionsF16.c \
../Src/InterpolationFunctions/arm_bilinear_interp_f16.c \
../Src/InterpolationFunctions/arm_bilinear_interp_f32.c \
../Src/InterpolationFunctions/arm_bilinear_interp_q15.c \
../Src/InterpolationFunctions/arm_bilinear_interp_q31.c \
../Src/InterpolationFunctions/arm_bilinear_interp_q7.c \
../Src/InterpolationFunctions/arm_linear_interp_f16.c \
../Src/InterpolationFunctions/arm_linear_interp_f32.c \
../Src/InterpolationFunctions/arm_linear_interp_q15.c \
../Src/InterpolationFunctions/arm_linear_interp_q31.c \
../Src/InterpolationFunctions/arm_linear_interp_q7.c \
../Src/InterpolationFunctions/arm_spline_interp_f32.c \
../Src/InterpolationFunctions/arm_spline_interp_init_f32.c 

OBJS += \
./Src/InterpolationFunctions/InterpolationFunctions.o \
./Src/InterpolationFunctions/InterpolationFunctionsF16.o \
./Src/InterpolationFunctions/arm_bilinear_interp_f16.o \
./Src/InterpolationFunctions/arm_bilinear_interp_f32.o \
./Src/InterpolationFunctions/arm_bilinear_interp_q15.o \
./Src/InterpolationFunctions/arm_bilinear_interp_q31.o \
./Src/InterpolationFunctions/arm_bilinear_interp_q7.o \
./Src/InterpolationFunctions/arm_linear_interp_f16.o \
./Src/InterpolationFunctions/arm_linear_interp_f32.o \
./Src/InterpolationFunctions/arm_linear_interp_q15.o \
./Src/InterpolationFunctions/arm_linear_interp_q31.o \
./Src/InterpolationFunctions/arm_linear_interp_q7.o \
./Src/InterpolationFunctions/arm_spline_interp_f32.o \
./Src/InterpolationFunctions/arm_spline_interp_init_f32.o 

C_DEPS += \
./Src/InterpolationFunctions/InterpolationFunctions.d \
./Src/InterpolationFunctions/InterpolationFunctionsF16.d \
./Src/InterpolationFunctions/arm_bilinear_interp_f16.d \
./Src/InterpolationFunctions/arm_bilinear_interp_f32.d \
./Src/InterpolationFunctions/arm_bilinear_interp_q15.d \
./Src/InterpolationFunctions/arm_bilinear_interp_q31.d \
./Src/InterpolationFunctions/arm_bilinear_interp_q7.d \
./Src/InterpolationFunctions/arm_linear_interp_f16.d \
./Src/InterpolationFunctions/arm_linear_interp_f32.d \
./Src/InterpolationFunctions/arm_linear_interp_q15.d \
./Src/InterpolationFunctions/arm_linear_interp_q31.d \
./Src/InterpolationFunctions/arm_linear_interp_q7.d \
./Src/InterpolationFunctions/arm_spline_interp_f32.d \
./Src/InterpolationFunctions/arm_spline_interp_init_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Src/InterpolationFunctions/%.o Src/InterpolationFunctions/%.su: ../Src/InterpolationFunctions/%.c Src/InterpolationFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-InterpolationFunctions

clean-Src-2f-InterpolationFunctions:
	-$(RM) ./Src/InterpolationFunctions/InterpolationFunctions.d ./Src/InterpolationFunctions/InterpolationFunctions.o ./Src/InterpolationFunctions/InterpolationFunctions.su ./Src/InterpolationFunctions/InterpolationFunctionsF16.d ./Src/InterpolationFunctions/InterpolationFunctionsF16.o ./Src/InterpolationFunctions/InterpolationFunctionsF16.su ./Src/InterpolationFunctions/arm_bilinear_interp_f16.d ./Src/InterpolationFunctions/arm_bilinear_interp_f16.o ./Src/InterpolationFunctions/arm_bilinear_interp_f16.su ./Src/InterpolationFunctions/arm_bilinear_interp_f32.d ./Src/InterpolationFunctions/arm_bilinear_interp_f32.o ./Src/InterpolationFunctions/arm_bilinear_interp_f32.su ./Src/InterpolationFunctions/arm_bilinear_interp_q15.d ./Src/InterpolationFunctions/arm_bilinear_interp_q15.o ./Src/InterpolationFunctions/arm_bilinear_interp_q15.su ./Src/InterpolationFunctions/arm_bilinear_interp_q31.d ./Src/InterpolationFunctions/arm_bilinear_interp_q31.o ./Src/InterpolationFunctions/arm_bilinear_interp_q31.su ./Src/InterpolationFunctions/arm_bilinear_interp_q7.d ./Src/InterpolationFunctions/arm_bilinear_interp_q7.o ./Src/InterpolationFunctions/arm_bilinear_interp_q7.su ./Src/InterpolationFunctions/arm_linear_interp_f16.d ./Src/InterpolationFunctions/arm_linear_interp_f16.o ./Src/InterpolationFunctions/arm_linear_interp_f16.su ./Src/InterpolationFunctions/arm_linear_interp_f32.d ./Src/InterpolationFunctions/arm_linear_interp_f32.o ./Src/InterpolationFunctions/arm_linear_interp_f32.su ./Src/InterpolationFunctions/arm_linear_interp_q15.d ./Src/InterpolationFunctions/arm_linear_interp_q15.o ./Src/InterpolationFunctions/arm_linear_interp_q15.su ./Src/InterpolationFunctions/arm_linear_interp_q31.d ./Src/InterpolationFunctions/arm_linear_interp_q31.o ./Src/InterpolationFunctions/arm_linear_interp_q31.su ./Src/InterpolationFunctions/arm_linear_interp_q7.d ./Src/InterpolationFunctions/arm_linear_interp_q7.o ./Src/InterpolationFunctions/arm_linear_interp_q7.su ./Src/InterpolationFunctions/arm_spline_interp_f32.d ./Src/InterpolationFunctions/arm_spline_interp_f32.o ./Src/InterpolationFunctions/arm_spline_interp_f32.su ./Src/InterpolationFunctions/arm_spline_interp_init_f32.d ./Src/InterpolationFunctions/arm_spline_interp_init_f32.o ./Src/InterpolationFunctions/arm_spline_interp_init_f32.su

.PHONY: clean-Src-2f-InterpolationFunctions

