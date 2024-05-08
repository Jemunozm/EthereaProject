################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/SupportFunctions/SupportFunctions.c \
../Src/SupportFunctions/SupportFunctionsF16.c \
../Src/SupportFunctions/arm_barycenter_f16.c \
../Src/SupportFunctions/arm_barycenter_f32.c \
../Src/SupportFunctions/arm_bitonic_sort_f32.c \
../Src/SupportFunctions/arm_bubble_sort_f32.c \
../Src/SupportFunctions/arm_copy_f16.c \
../Src/SupportFunctions/arm_copy_f32.c \
../Src/SupportFunctions/arm_copy_f64.c \
../Src/SupportFunctions/arm_copy_q15.c \
../Src/SupportFunctions/arm_copy_q31.c \
../Src/SupportFunctions/arm_copy_q7.c \
../Src/SupportFunctions/arm_f16_to_float.c \
../Src/SupportFunctions/arm_f16_to_q15.c \
../Src/SupportFunctions/arm_fill_f16.c \
../Src/SupportFunctions/arm_fill_f32.c \
../Src/SupportFunctions/arm_fill_f64.c \
../Src/SupportFunctions/arm_fill_q15.c \
../Src/SupportFunctions/arm_fill_q31.c \
../Src/SupportFunctions/arm_fill_q7.c \
../Src/SupportFunctions/arm_float_to_f16.c \
../Src/SupportFunctions/arm_float_to_q15.c \
../Src/SupportFunctions/arm_float_to_q31.c \
../Src/SupportFunctions/arm_float_to_q7.c \
../Src/SupportFunctions/arm_heap_sort_f32.c \
../Src/SupportFunctions/arm_insertion_sort_f32.c \
../Src/SupportFunctions/arm_merge_sort_f32.c \
../Src/SupportFunctions/arm_merge_sort_init_f32.c \
../Src/SupportFunctions/arm_q15_to_f16.c \
../Src/SupportFunctions/arm_q15_to_float.c \
../Src/SupportFunctions/arm_q15_to_q31.c \
../Src/SupportFunctions/arm_q15_to_q7.c \
../Src/SupportFunctions/arm_q31_to_float.c \
../Src/SupportFunctions/arm_q31_to_q15.c \
../Src/SupportFunctions/arm_q31_to_q7.c \
../Src/SupportFunctions/arm_q7_to_float.c \
../Src/SupportFunctions/arm_q7_to_q15.c \
../Src/SupportFunctions/arm_q7_to_q31.c \
../Src/SupportFunctions/arm_quick_sort_f32.c \
../Src/SupportFunctions/arm_selection_sort_f32.c \
../Src/SupportFunctions/arm_sort_f32.c \
../Src/SupportFunctions/arm_sort_init_f32.c \
../Src/SupportFunctions/arm_weighted_sum_f16.c \
../Src/SupportFunctions/arm_weighted_sum_f32.c 

OBJS += \
./Src/SupportFunctions/SupportFunctions.o \
./Src/SupportFunctions/SupportFunctionsF16.o \
./Src/SupportFunctions/arm_barycenter_f16.o \
./Src/SupportFunctions/arm_barycenter_f32.o \
./Src/SupportFunctions/arm_bitonic_sort_f32.o \
./Src/SupportFunctions/arm_bubble_sort_f32.o \
./Src/SupportFunctions/arm_copy_f16.o \
./Src/SupportFunctions/arm_copy_f32.o \
./Src/SupportFunctions/arm_copy_f64.o \
./Src/SupportFunctions/arm_copy_q15.o \
./Src/SupportFunctions/arm_copy_q31.o \
./Src/SupportFunctions/arm_copy_q7.o \
./Src/SupportFunctions/arm_f16_to_float.o \
./Src/SupportFunctions/arm_f16_to_q15.o \
./Src/SupportFunctions/arm_fill_f16.o \
./Src/SupportFunctions/arm_fill_f32.o \
./Src/SupportFunctions/arm_fill_f64.o \
./Src/SupportFunctions/arm_fill_q15.o \
./Src/SupportFunctions/arm_fill_q31.o \
./Src/SupportFunctions/arm_fill_q7.o \
./Src/SupportFunctions/arm_float_to_f16.o \
./Src/SupportFunctions/arm_float_to_q15.o \
./Src/SupportFunctions/arm_float_to_q31.o \
./Src/SupportFunctions/arm_float_to_q7.o \
./Src/SupportFunctions/arm_heap_sort_f32.o \
./Src/SupportFunctions/arm_insertion_sort_f32.o \
./Src/SupportFunctions/arm_merge_sort_f32.o \
./Src/SupportFunctions/arm_merge_sort_init_f32.o \
./Src/SupportFunctions/arm_q15_to_f16.o \
./Src/SupportFunctions/arm_q15_to_float.o \
./Src/SupportFunctions/arm_q15_to_q31.o \
./Src/SupportFunctions/arm_q15_to_q7.o \
./Src/SupportFunctions/arm_q31_to_float.o \
./Src/SupportFunctions/arm_q31_to_q15.o \
./Src/SupportFunctions/arm_q31_to_q7.o \
./Src/SupportFunctions/arm_q7_to_float.o \
./Src/SupportFunctions/arm_q7_to_q15.o \
./Src/SupportFunctions/arm_q7_to_q31.o \
./Src/SupportFunctions/arm_quick_sort_f32.o \
./Src/SupportFunctions/arm_selection_sort_f32.o \
./Src/SupportFunctions/arm_sort_f32.o \
./Src/SupportFunctions/arm_sort_init_f32.o \
./Src/SupportFunctions/arm_weighted_sum_f16.o \
./Src/SupportFunctions/arm_weighted_sum_f32.o 

C_DEPS += \
./Src/SupportFunctions/SupportFunctions.d \
./Src/SupportFunctions/SupportFunctionsF16.d \
./Src/SupportFunctions/arm_barycenter_f16.d \
./Src/SupportFunctions/arm_barycenter_f32.d \
./Src/SupportFunctions/arm_bitonic_sort_f32.d \
./Src/SupportFunctions/arm_bubble_sort_f32.d \
./Src/SupportFunctions/arm_copy_f16.d \
./Src/SupportFunctions/arm_copy_f32.d \
./Src/SupportFunctions/arm_copy_f64.d \
./Src/SupportFunctions/arm_copy_q15.d \
./Src/SupportFunctions/arm_copy_q31.d \
./Src/SupportFunctions/arm_copy_q7.d \
./Src/SupportFunctions/arm_f16_to_float.d \
./Src/SupportFunctions/arm_f16_to_q15.d \
./Src/SupportFunctions/arm_fill_f16.d \
./Src/SupportFunctions/arm_fill_f32.d \
./Src/SupportFunctions/arm_fill_f64.d \
./Src/SupportFunctions/arm_fill_q15.d \
./Src/SupportFunctions/arm_fill_q31.d \
./Src/SupportFunctions/arm_fill_q7.d \
./Src/SupportFunctions/arm_float_to_f16.d \
./Src/SupportFunctions/arm_float_to_q15.d \
./Src/SupportFunctions/arm_float_to_q31.d \
./Src/SupportFunctions/arm_float_to_q7.d \
./Src/SupportFunctions/arm_heap_sort_f32.d \
./Src/SupportFunctions/arm_insertion_sort_f32.d \
./Src/SupportFunctions/arm_merge_sort_f32.d \
./Src/SupportFunctions/arm_merge_sort_init_f32.d \
./Src/SupportFunctions/arm_q15_to_f16.d \
./Src/SupportFunctions/arm_q15_to_float.d \
./Src/SupportFunctions/arm_q15_to_q31.d \
./Src/SupportFunctions/arm_q15_to_q7.d \
./Src/SupportFunctions/arm_q31_to_float.d \
./Src/SupportFunctions/arm_q31_to_q15.d \
./Src/SupportFunctions/arm_q31_to_q7.d \
./Src/SupportFunctions/arm_q7_to_float.d \
./Src/SupportFunctions/arm_q7_to_q15.d \
./Src/SupportFunctions/arm_q7_to_q31.d \
./Src/SupportFunctions/arm_quick_sort_f32.d \
./Src/SupportFunctions/arm_selection_sort_f32.d \
./Src/SupportFunctions/arm_sort_f32.d \
./Src/SupportFunctions/arm_sort_init_f32.d \
./Src/SupportFunctions/arm_weighted_sum_f16.d \
./Src/SupportFunctions/arm_weighted_sum_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Src/SupportFunctions/%.o Src/SupportFunctions/%.su: ../Src/SupportFunctions/%.c Src/SupportFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-SupportFunctions

clean-Src-2f-SupportFunctions:
	-$(RM) ./Src/SupportFunctions/SupportFunctions.d ./Src/SupportFunctions/SupportFunctions.o ./Src/SupportFunctions/SupportFunctions.su ./Src/SupportFunctions/SupportFunctionsF16.d ./Src/SupportFunctions/SupportFunctionsF16.o ./Src/SupportFunctions/SupportFunctionsF16.su ./Src/SupportFunctions/arm_barycenter_f16.d ./Src/SupportFunctions/arm_barycenter_f16.o ./Src/SupportFunctions/arm_barycenter_f16.su ./Src/SupportFunctions/arm_barycenter_f32.d ./Src/SupportFunctions/arm_barycenter_f32.o ./Src/SupportFunctions/arm_barycenter_f32.su ./Src/SupportFunctions/arm_bitonic_sort_f32.d ./Src/SupportFunctions/arm_bitonic_sort_f32.o ./Src/SupportFunctions/arm_bitonic_sort_f32.su ./Src/SupportFunctions/arm_bubble_sort_f32.d ./Src/SupportFunctions/arm_bubble_sort_f32.o ./Src/SupportFunctions/arm_bubble_sort_f32.su ./Src/SupportFunctions/arm_copy_f16.d ./Src/SupportFunctions/arm_copy_f16.o ./Src/SupportFunctions/arm_copy_f16.su ./Src/SupportFunctions/arm_copy_f32.d ./Src/SupportFunctions/arm_copy_f32.o ./Src/SupportFunctions/arm_copy_f32.su ./Src/SupportFunctions/arm_copy_f64.d ./Src/SupportFunctions/arm_copy_f64.o ./Src/SupportFunctions/arm_copy_f64.su ./Src/SupportFunctions/arm_copy_q15.d ./Src/SupportFunctions/arm_copy_q15.o ./Src/SupportFunctions/arm_copy_q15.su ./Src/SupportFunctions/arm_copy_q31.d ./Src/SupportFunctions/arm_copy_q31.o ./Src/SupportFunctions/arm_copy_q31.su ./Src/SupportFunctions/arm_copy_q7.d ./Src/SupportFunctions/arm_copy_q7.o ./Src/SupportFunctions/arm_copy_q7.su ./Src/SupportFunctions/arm_f16_to_float.d ./Src/SupportFunctions/arm_f16_to_float.o ./Src/SupportFunctions/arm_f16_to_float.su ./Src/SupportFunctions/arm_f16_to_q15.d ./Src/SupportFunctions/arm_f16_to_q15.o ./Src/SupportFunctions/arm_f16_to_q15.su ./Src/SupportFunctions/arm_fill_f16.d ./Src/SupportFunctions/arm_fill_f16.o ./Src/SupportFunctions/arm_fill_f16.su ./Src/SupportFunctions/arm_fill_f32.d ./Src/SupportFunctions/arm_fill_f32.o ./Src/SupportFunctions/arm_fill_f32.su ./Src/SupportFunctions/arm_fill_f64.d ./Src/SupportFunctions/arm_fill_f64.o ./Src/SupportFunctions/arm_fill_f64.su ./Src/SupportFunctions/arm_fill_q15.d ./Src/SupportFunctions/arm_fill_q15.o ./Src/SupportFunctions/arm_fill_q15.su ./Src/SupportFunctions/arm_fill_q31.d ./Src/SupportFunctions/arm_fill_q31.o ./Src/SupportFunctions/arm_fill_q31.su ./Src/SupportFunctions/arm_fill_q7.d ./Src/SupportFunctions/arm_fill_q7.o ./Src/SupportFunctions/arm_fill_q7.su ./Src/SupportFunctions/arm_float_to_f16.d ./Src/SupportFunctions/arm_float_to_f16.o ./Src/SupportFunctions/arm_float_to_f16.su ./Src/SupportFunctions/arm_float_to_q15.d ./Src/SupportFunctions/arm_float_to_q15.o ./Src/SupportFunctions/arm_float_to_q15.su ./Src/SupportFunctions/arm_float_to_q31.d ./Src/SupportFunctions/arm_float_to_q31.o ./Src/SupportFunctions/arm_float_to_q31.su ./Src/SupportFunctions/arm_float_to_q7.d ./Src/SupportFunctions/arm_float_to_q7.o ./Src/SupportFunctions/arm_float_to_q7.su ./Src/SupportFunctions/arm_heap_sort_f32.d ./Src/SupportFunctions/arm_heap_sort_f32.o ./Src/SupportFunctions/arm_heap_sort_f32.su ./Src/SupportFunctions/arm_insertion_sort_f32.d ./Src/SupportFunctions/arm_insertion_sort_f32.o ./Src/SupportFunctions/arm_insertion_sort_f32.su ./Src/SupportFunctions/arm_merge_sort_f32.d ./Src/SupportFunctions/arm_merge_sort_f32.o ./Src/SupportFunctions/arm_merge_sort_f32.su ./Src/SupportFunctions/arm_merge_sort_init_f32.d ./Src/SupportFunctions/arm_merge_sort_init_f32.o ./Src/SupportFunctions/arm_merge_sort_init_f32.su ./Src/SupportFunctions/arm_q15_to_f16.d ./Src/SupportFunctions/arm_q15_to_f16.o ./Src/SupportFunctions/arm_q15_to_f16.su ./Src/SupportFunctions/arm_q15_to_float.d ./Src/SupportFunctions/arm_q15_to_float.o ./Src/SupportFunctions/arm_q15_to_float.su ./Src/SupportFunctions/arm_q15_to_q31.d ./Src/SupportFunctions/arm_q15_to_q31.o ./Src/SupportFunctions/arm_q15_to_q31.su ./Src/SupportFunctions/arm_q15_to_q7.d ./Src/SupportFunctions/arm_q15_to_q7.o ./Src/SupportFunctions/arm_q15_to_q7.su ./Src/SupportFunctions/arm_q31_to_float.d ./Src/SupportFunctions/arm_q31_to_float.o ./Src/SupportFunctions/arm_q31_to_float.su ./Src/SupportFunctions/arm_q31_to_q15.d ./Src/SupportFunctions/arm_q31_to_q15.o ./Src/SupportFunctions/arm_q31_to_q15.su ./Src/SupportFunctions/arm_q31_to_q7.d ./Src/SupportFunctions/arm_q31_to_q7.o ./Src/SupportFunctions/arm_q31_to_q7.su ./Src/SupportFunctions/arm_q7_to_float.d ./Src/SupportFunctions/arm_q7_to_float.o ./Src/SupportFunctions/arm_q7_to_float.su ./Src/SupportFunctions/arm_q7_to_q15.d ./Src/SupportFunctions/arm_q7_to_q15.o ./Src/SupportFunctions/arm_q7_to_q15.su ./Src/SupportFunctions/arm_q7_to_q31.d ./Src/SupportFunctions/arm_q7_to_q31.o ./Src/SupportFunctions/arm_q7_to_q31.su ./Src/SupportFunctions/arm_quick_sort_f32.d ./Src/SupportFunctions/arm_quick_sort_f32.o ./Src/SupportFunctions/arm_quick_sort_f32.su ./Src/SupportFunctions/arm_selection_sort_f32.d ./Src/SupportFunctions/arm_selection_sort_f32.o ./Src/SupportFunctions/arm_selection_sort_f32.su ./Src/SupportFunctions/arm_sort_f32.d ./Src/SupportFunctions/arm_sort_f32.o ./Src/SupportFunctions/arm_sort_f32.su ./Src/SupportFunctions/arm_sort_init_f32.d ./Src/SupportFunctions/arm_sort_init_f32.o ./Src/SupportFunctions/arm_sort_init_f32.su ./Src/SupportFunctions/arm_weighted_sum_f16.d ./Src/SupportFunctions/arm_weighted_sum_f16.o ./Src/SupportFunctions/arm_weighted_sum_f16.su ./Src/SupportFunctions/arm_weighted_sum_f32.d ./Src/SupportFunctions/arm_weighted_sum_f32.o ./Src/SupportFunctions/arm_weighted_sum_f32.su

.PHONY: clean-Src-2f-SupportFunctions

