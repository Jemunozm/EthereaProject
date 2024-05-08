################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/TransformFunctions/TransformFunctions.c \
../Src/TransformFunctions/TransformFunctionsF16.c \
../Src/TransformFunctions/arm_bitreversal.c \
../Src/TransformFunctions/arm_bitreversal2.c \
../Src/TransformFunctions/arm_bitreversal_f16.c \
../Src/TransformFunctions/arm_cfft_f16.c \
../Src/TransformFunctions/arm_cfft_f32.c \
../Src/TransformFunctions/arm_cfft_f64.c \
../Src/TransformFunctions/arm_cfft_init_f16.c \
../Src/TransformFunctions/arm_cfft_init_f32.c \
../Src/TransformFunctions/arm_cfft_init_f64.c \
../Src/TransformFunctions/arm_cfft_init_q15.c \
../Src/TransformFunctions/arm_cfft_init_q31.c \
../Src/TransformFunctions/arm_cfft_q15.c \
../Src/TransformFunctions/arm_cfft_q31.c \
../Src/TransformFunctions/arm_cfft_radix2_f16.c \
../Src/TransformFunctions/arm_cfft_radix2_f32.c \
../Src/TransformFunctions/arm_cfft_radix2_init_f16.c \
../Src/TransformFunctions/arm_cfft_radix2_init_f32.c \
../Src/TransformFunctions/arm_cfft_radix2_init_q15.c \
../Src/TransformFunctions/arm_cfft_radix2_init_q31.c \
../Src/TransformFunctions/arm_cfft_radix2_q15.c \
../Src/TransformFunctions/arm_cfft_radix2_q31.c \
../Src/TransformFunctions/arm_cfft_radix4_f16.c \
../Src/TransformFunctions/arm_cfft_radix4_f32.c \
../Src/TransformFunctions/arm_cfft_radix4_init_f16.c \
../Src/TransformFunctions/arm_cfft_radix4_init_f32.c \
../Src/TransformFunctions/arm_cfft_radix4_init_q15.c \
../Src/TransformFunctions/arm_cfft_radix4_init_q31.c \
../Src/TransformFunctions/arm_cfft_radix4_q15.c \
../Src/TransformFunctions/arm_cfft_radix4_q31.c \
../Src/TransformFunctions/arm_cfft_radix8_f16.c \
../Src/TransformFunctions/arm_cfft_radix8_f32.c \
../Src/TransformFunctions/arm_dct4_f32.c \
../Src/TransformFunctions/arm_dct4_init_f32.c \
../Src/TransformFunctions/arm_dct4_init_q15.c \
../Src/TransformFunctions/arm_dct4_init_q31.c \
../Src/TransformFunctions/arm_dct4_q15.c \
../Src/TransformFunctions/arm_dct4_q31.c \
../Src/TransformFunctions/arm_mfcc_f16.c \
../Src/TransformFunctions/arm_mfcc_f32.c \
../Src/TransformFunctions/arm_mfcc_init_f16.c \
../Src/TransformFunctions/arm_mfcc_init_f32.c \
../Src/TransformFunctions/arm_mfcc_init_q15.c \
../Src/TransformFunctions/arm_mfcc_init_q31.c \
../Src/TransformFunctions/arm_mfcc_q15.c \
../Src/TransformFunctions/arm_mfcc_q31.c \
../Src/TransformFunctions/arm_rfft_f32.c \
../Src/TransformFunctions/arm_rfft_fast_f16.c \
../Src/TransformFunctions/arm_rfft_fast_f32.c \
../Src/TransformFunctions/arm_rfft_fast_f64.c \
../Src/TransformFunctions/arm_rfft_fast_init_f16.c \
../Src/TransformFunctions/arm_rfft_fast_init_f32.c \
../Src/TransformFunctions/arm_rfft_fast_init_f64.c \
../Src/TransformFunctions/arm_rfft_init_f32.c \
../Src/TransformFunctions/arm_rfft_init_q15.c \
../Src/TransformFunctions/arm_rfft_init_q31.c \
../Src/TransformFunctions/arm_rfft_q15.c \
../Src/TransformFunctions/arm_rfft_q31.c 

S_UPPER_SRCS += \
../Src/TransformFunctions/arm_bitreversal2.S 

OBJS += \
./Src/TransformFunctions/TransformFunctions.o \
./Src/TransformFunctions/TransformFunctionsF16.o \
./Src/TransformFunctions/arm_bitreversal.o \
./Src/TransformFunctions/arm_bitreversal2.o \
./Src/TransformFunctions/arm_bitreversal_f16.o \
./Src/TransformFunctions/arm_cfft_f16.o \
./Src/TransformFunctions/arm_cfft_f32.o \
./Src/TransformFunctions/arm_cfft_f64.o \
./Src/TransformFunctions/arm_cfft_init_f16.o \
./Src/TransformFunctions/arm_cfft_init_f32.o \
./Src/TransformFunctions/arm_cfft_init_f64.o \
./Src/TransformFunctions/arm_cfft_init_q15.o \
./Src/TransformFunctions/arm_cfft_init_q31.o \
./Src/TransformFunctions/arm_cfft_q15.o \
./Src/TransformFunctions/arm_cfft_q31.o \
./Src/TransformFunctions/arm_cfft_radix2_f16.o \
./Src/TransformFunctions/arm_cfft_radix2_f32.o \
./Src/TransformFunctions/arm_cfft_radix2_init_f16.o \
./Src/TransformFunctions/arm_cfft_radix2_init_f32.o \
./Src/TransformFunctions/arm_cfft_radix2_init_q15.o \
./Src/TransformFunctions/arm_cfft_radix2_init_q31.o \
./Src/TransformFunctions/arm_cfft_radix2_q15.o \
./Src/TransformFunctions/arm_cfft_radix2_q31.o \
./Src/TransformFunctions/arm_cfft_radix4_f16.o \
./Src/TransformFunctions/arm_cfft_radix4_f32.o \
./Src/TransformFunctions/arm_cfft_radix4_init_f16.o \
./Src/TransformFunctions/arm_cfft_radix4_init_f32.o \
./Src/TransformFunctions/arm_cfft_radix4_init_q15.o \
./Src/TransformFunctions/arm_cfft_radix4_init_q31.o \
./Src/TransformFunctions/arm_cfft_radix4_q15.o \
./Src/TransformFunctions/arm_cfft_radix4_q31.o \
./Src/TransformFunctions/arm_cfft_radix8_f16.o \
./Src/TransformFunctions/arm_cfft_radix8_f32.o \
./Src/TransformFunctions/arm_dct4_f32.o \
./Src/TransformFunctions/arm_dct4_init_f32.o \
./Src/TransformFunctions/arm_dct4_init_q15.o \
./Src/TransformFunctions/arm_dct4_init_q31.o \
./Src/TransformFunctions/arm_dct4_q15.o \
./Src/TransformFunctions/arm_dct4_q31.o \
./Src/TransformFunctions/arm_mfcc_f16.o \
./Src/TransformFunctions/arm_mfcc_f32.o \
./Src/TransformFunctions/arm_mfcc_init_f16.o \
./Src/TransformFunctions/arm_mfcc_init_f32.o \
./Src/TransformFunctions/arm_mfcc_init_q15.o \
./Src/TransformFunctions/arm_mfcc_init_q31.o \
./Src/TransformFunctions/arm_mfcc_q15.o \
./Src/TransformFunctions/arm_mfcc_q31.o \
./Src/TransformFunctions/arm_rfft_f32.o \
./Src/TransformFunctions/arm_rfft_fast_f16.o \
./Src/TransformFunctions/arm_rfft_fast_f32.o \
./Src/TransformFunctions/arm_rfft_fast_f64.o \
./Src/TransformFunctions/arm_rfft_fast_init_f16.o \
./Src/TransformFunctions/arm_rfft_fast_init_f32.o \
./Src/TransformFunctions/arm_rfft_fast_init_f64.o \
./Src/TransformFunctions/arm_rfft_init_f32.o \
./Src/TransformFunctions/arm_rfft_init_q15.o \
./Src/TransformFunctions/arm_rfft_init_q31.o \
./Src/TransformFunctions/arm_rfft_q15.o \
./Src/TransformFunctions/arm_rfft_q31.o 

S_UPPER_DEPS += \
./Src/TransformFunctions/arm_bitreversal2.d 

C_DEPS += \
./Src/TransformFunctions/TransformFunctions.d \
./Src/TransformFunctions/TransformFunctionsF16.d \
./Src/TransformFunctions/arm_bitreversal.d \
./Src/TransformFunctions/arm_bitreversal2.d \
./Src/TransformFunctions/arm_bitreversal_f16.d \
./Src/TransformFunctions/arm_cfft_f16.d \
./Src/TransformFunctions/arm_cfft_f32.d \
./Src/TransformFunctions/arm_cfft_f64.d \
./Src/TransformFunctions/arm_cfft_init_f16.d \
./Src/TransformFunctions/arm_cfft_init_f32.d \
./Src/TransformFunctions/arm_cfft_init_f64.d \
./Src/TransformFunctions/arm_cfft_init_q15.d \
./Src/TransformFunctions/arm_cfft_init_q31.d \
./Src/TransformFunctions/arm_cfft_q15.d \
./Src/TransformFunctions/arm_cfft_q31.d \
./Src/TransformFunctions/arm_cfft_radix2_f16.d \
./Src/TransformFunctions/arm_cfft_radix2_f32.d \
./Src/TransformFunctions/arm_cfft_radix2_init_f16.d \
./Src/TransformFunctions/arm_cfft_radix2_init_f32.d \
./Src/TransformFunctions/arm_cfft_radix2_init_q15.d \
./Src/TransformFunctions/arm_cfft_radix2_init_q31.d \
./Src/TransformFunctions/arm_cfft_radix2_q15.d \
./Src/TransformFunctions/arm_cfft_radix2_q31.d \
./Src/TransformFunctions/arm_cfft_radix4_f16.d \
./Src/TransformFunctions/arm_cfft_radix4_f32.d \
./Src/TransformFunctions/arm_cfft_radix4_init_f16.d \
./Src/TransformFunctions/arm_cfft_radix4_init_f32.d \
./Src/TransformFunctions/arm_cfft_radix4_init_q15.d \
./Src/TransformFunctions/arm_cfft_radix4_init_q31.d \
./Src/TransformFunctions/arm_cfft_radix4_q15.d \
./Src/TransformFunctions/arm_cfft_radix4_q31.d \
./Src/TransformFunctions/arm_cfft_radix8_f16.d \
./Src/TransformFunctions/arm_cfft_radix8_f32.d \
./Src/TransformFunctions/arm_dct4_f32.d \
./Src/TransformFunctions/arm_dct4_init_f32.d \
./Src/TransformFunctions/arm_dct4_init_q15.d \
./Src/TransformFunctions/arm_dct4_init_q31.d \
./Src/TransformFunctions/arm_dct4_q15.d \
./Src/TransformFunctions/arm_dct4_q31.d \
./Src/TransformFunctions/arm_mfcc_f16.d \
./Src/TransformFunctions/arm_mfcc_f32.d \
./Src/TransformFunctions/arm_mfcc_init_f16.d \
./Src/TransformFunctions/arm_mfcc_init_f32.d \
./Src/TransformFunctions/arm_mfcc_init_q15.d \
./Src/TransformFunctions/arm_mfcc_init_q31.d \
./Src/TransformFunctions/arm_mfcc_q15.d \
./Src/TransformFunctions/arm_mfcc_q31.d \
./Src/TransformFunctions/arm_rfft_f32.d \
./Src/TransformFunctions/arm_rfft_fast_f16.d \
./Src/TransformFunctions/arm_rfft_fast_f32.d \
./Src/TransformFunctions/arm_rfft_fast_f64.d \
./Src/TransformFunctions/arm_rfft_fast_init_f16.d \
./Src/TransformFunctions/arm_rfft_fast_init_f32.d \
./Src/TransformFunctions/arm_rfft_fast_init_f64.d \
./Src/TransformFunctions/arm_rfft_init_f32.d \
./Src/TransformFunctions/arm_rfft_init_q15.d \
./Src/TransformFunctions/arm_rfft_init_q31.d \
./Src/TransformFunctions/arm_rfft_q15.d \
./Src/TransformFunctions/arm_rfft_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Src/TransformFunctions/%.o Src/TransformFunctions/%.su: ../Src/TransformFunctions/%.c Src/TransformFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/TransformFunctions/%.o: ../Src/TransformFunctions/%.S Src/TransformFunctions/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Src-2f-TransformFunctions

clean-Src-2f-TransformFunctions:
	-$(RM) ./Src/TransformFunctions/TransformFunctions.d ./Src/TransformFunctions/TransformFunctions.o ./Src/TransformFunctions/TransformFunctions.su ./Src/TransformFunctions/TransformFunctionsF16.d ./Src/TransformFunctions/TransformFunctionsF16.o ./Src/TransformFunctions/TransformFunctionsF16.su ./Src/TransformFunctions/arm_bitreversal.d ./Src/TransformFunctions/arm_bitreversal.o ./Src/TransformFunctions/arm_bitreversal.su ./Src/TransformFunctions/arm_bitreversal2.d ./Src/TransformFunctions/arm_bitreversal2.o ./Src/TransformFunctions/arm_bitreversal2.su ./Src/TransformFunctions/arm_bitreversal_f16.d ./Src/TransformFunctions/arm_bitreversal_f16.o ./Src/TransformFunctions/arm_bitreversal_f16.su ./Src/TransformFunctions/arm_cfft_f16.d ./Src/TransformFunctions/arm_cfft_f16.o ./Src/TransformFunctions/arm_cfft_f16.su ./Src/TransformFunctions/arm_cfft_f32.d ./Src/TransformFunctions/arm_cfft_f32.o ./Src/TransformFunctions/arm_cfft_f32.su ./Src/TransformFunctions/arm_cfft_f64.d ./Src/TransformFunctions/arm_cfft_f64.o ./Src/TransformFunctions/arm_cfft_f64.su ./Src/TransformFunctions/arm_cfft_init_f16.d ./Src/TransformFunctions/arm_cfft_init_f16.o ./Src/TransformFunctions/arm_cfft_init_f16.su ./Src/TransformFunctions/arm_cfft_init_f32.d ./Src/TransformFunctions/arm_cfft_init_f32.o ./Src/TransformFunctions/arm_cfft_init_f32.su ./Src/TransformFunctions/arm_cfft_init_f64.d ./Src/TransformFunctions/arm_cfft_init_f64.o ./Src/TransformFunctions/arm_cfft_init_f64.su ./Src/TransformFunctions/arm_cfft_init_q15.d ./Src/TransformFunctions/arm_cfft_init_q15.o ./Src/TransformFunctions/arm_cfft_init_q15.su ./Src/TransformFunctions/arm_cfft_init_q31.d ./Src/TransformFunctions/arm_cfft_init_q31.o ./Src/TransformFunctions/arm_cfft_init_q31.su ./Src/TransformFunctions/arm_cfft_q15.d ./Src/TransformFunctions/arm_cfft_q15.o ./Src/TransformFunctions/arm_cfft_q15.su ./Src/TransformFunctions/arm_cfft_q31.d ./Src/TransformFunctions/arm_cfft_q31.o ./Src/TransformFunctions/arm_cfft_q31.su ./Src/TransformFunctions/arm_cfft_radix2_f16.d ./Src/TransformFunctions/arm_cfft_radix2_f16.o ./Src/TransformFunctions/arm_cfft_radix2_f16.su ./Src/TransformFunctions/arm_cfft_radix2_f32.d ./Src/TransformFunctions/arm_cfft_radix2_f32.o ./Src/TransformFunctions/arm_cfft_radix2_f32.su ./Src/TransformFunctions/arm_cfft_radix2_init_f16.d ./Src/TransformFunctions/arm_cfft_radix2_init_f16.o ./Src/TransformFunctions/arm_cfft_radix2_init_f16.su ./Src/TransformFunctions/arm_cfft_radix2_init_f32.d ./Src/TransformFunctions/arm_cfft_radix2_init_f32.o ./Src/TransformFunctions/arm_cfft_radix2_init_f32.su ./Src/TransformFunctions/arm_cfft_radix2_init_q15.d ./Src/TransformFunctions/arm_cfft_radix2_init_q15.o ./Src/TransformFunctions/arm_cfft_radix2_init_q15.su ./Src/TransformFunctions/arm_cfft_radix2_init_q31.d ./Src/TransformFunctions/arm_cfft_radix2_init_q31.o ./Src/TransformFunctions/arm_cfft_radix2_init_q31.su ./Src/TransformFunctions/arm_cfft_radix2_q15.d ./Src/TransformFunctions/arm_cfft_radix2_q15.o ./Src/TransformFunctions/arm_cfft_radix2_q15.su ./Src/TransformFunctions/arm_cfft_radix2_q31.d ./Src/TransformFunctions/arm_cfft_radix2_q31.o ./Src/TransformFunctions/arm_cfft_radix2_q31.su ./Src/TransformFunctions/arm_cfft_radix4_f16.d ./Src/TransformFunctions/arm_cfft_radix4_f16.o ./Src/TransformFunctions/arm_cfft_radix4_f16.su ./Src/TransformFunctions/arm_cfft_radix4_f32.d ./Src/TransformFunctions/arm_cfft_radix4_f32.o ./Src/TransformFunctions/arm_cfft_radix4_f32.su ./Src/TransformFunctions/arm_cfft_radix4_init_f16.d ./Src/TransformFunctions/arm_cfft_radix4_init_f16.o ./Src/TransformFunctions/arm_cfft_radix4_init_f16.su ./Src/TransformFunctions/arm_cfft_radix4_init_f32.d ./Src/TransformFunctions/arm_cfft_radix4_init_f32.o ./Src/TransformFunctions/arm_cfft_radix4_init_f32.su ./Src/TransformFunctions/arm_cfft_radix4_init_q15.d ./Src/TransformFunctions/arm_cfft_radix4_init_q15.o ./Src/TransformFunctions/arm_cfft_radix4_init_q15.su ./Src/TransformFunctions/arm_cfft_radix4_init_q31.d ./Src/TransformFunctions/arm_cfft_radix4_init_q31.o ./Src/TransformFunctions/arm_cfft_radix4_init_q31.su ./Src/TransformFunctions/arm_cfft_radix4_q15.d ./Src/TransformFunctions/arm_cfft_radix4_q15.o ./Src/TransformFunctions/arm_cfft_radix4_q15.su ./Src/TransformFunctions/arm_cfft_radix4_q31.d ./Src/TransformFunctions/arm_cfft_radix4_q31.o ./Src/TransformFunctions/arm_cfft_radix4_q31.su ./Src/TransformFunctions/arm_cfft_radix8_f16.d ./Src/TransformFunctions/arm_cfft_radix8_f16.o ./Src/TransformFunctions/arm_cfft_radix8_f16.su ./Src/TransformFunctions/arm_cfft_radix8_f32.d ./Src/TransformFunctions/arm_cfft_radix8_f32.o ./Src/TransformFunctions/arm_cfft_radix8_f32.su ./Src/TransformFunctions/arm_dct4_f32.d ./Src/TransformFunctions/arm_dct4_f32.o ./Src/TransformFunctions/arm_dct4_f32.su ./Src/TransformFunctions/arm_dct4_init_f32.d ./Src/TransformFunctions/arm_dct4_init_f32.o ./Src/TransformFunctions/arm_dct4_init_f32.su ./Src/TransformFunctions/arm_dct4_init_q15.d ./Src/TransformFunctions/arm_dct4_init_q15.o ./Src/TransformFunctions/arm_dct4_init_q15.su ./Src/TransformFunctions/arm_dct4_init_q31.d ./Src/TransformFunctions/arm_dct4_init_q31.o ./Src/TransformFunctions/arm_dct4_init_q31.su ./Src/TransformFunctions/arm_dct4_q15.d ./Src/TransformFunctions/arm_dct4_q15.o ./Src/TransformFunctions/arm_dct4_q15.su ./Src/TransformFunctions/arm_dct4_q31.d ./Src/TransformFunctions/arm_dct4_q31.o ./Src/TransformFunctions/arm_dct4_q31.su ./Src/TransformFunctions/arm_mfcc_f16.d ./Src/TransformFunctions/arm_mfcc_f16.o ./Src/TransformFunctions/arm_mfcc_f16.su ./Src/TransformFunctions/arm_mfcc_f32.d ./Src/TransformFunctions/arm_mfcc_f32.o ./Src/TransformFunctions/arm_mfcc_f32.su ./Src/TransformFunctions/arm_mfcc_init_f16.d ./Src/TransformFunctions/arm_mfcc_init_f16.o ./Src/TransformFunctions/arm_mfcc_init_f16.su ./Src/TransformFunctions/arm_mfcc_init_f32.d ./Src/TransformFunctions/arm_mfcc_init_f32.o ./Src/TransformFunctions/arm_mfcc_init_f32.su ./Src/TransformFunctions/arm_mfcc_init_q15.d
	-$(RM) ./Src/TransformFunctions/arm_mfcc_init_q15.o ./Src/TransformFunctions/arm_mfcc_init_q15.su ./Src/TransformFunctions/arm_mfcc_init_q31.d ./Src/TransformFunctions/arm_mfcc_init_q31.o ./Src/TransformFunctions/arm_mfcc_init_q31.su ./Src/TransformFunctions/arm_mfcc_q15.d ./Src/TransformFunctions/arm_mfcc_q15.o ./Src/TransformFunctions/arm_mfcc_q15.su ./Src/TransformFunctions/arm_mfcc_q31.d ./Src/TransformFunctions/arm_mfcc_q31.o ./Src/TransformFunctions/arm_mfcc_q31.su ./Src/TransformFunctions/arm_rfft_f32.d ./Src/TransformFunctions/arm_rfft_f32.o ./Src/TransformFunctions/arm_rfft_f32.su ./Src/TransformFunctions/arm_rfft_fast_f16.d ./Src/TransformFunctions/arm_rfft_fast_f16.o ./Src/TransformFunctions/arm_rfft_fast_f16.su ./Src/TransformFunctions/arm_rfft_fast_f32.d ./Src/TransformFunctions/arm_rfft_fast_f32.o ./Src/TransformFunctions/arm_rfft_fast_f32.su ./Src/TransformFunctions/arm_rfft_fast_f64.d ./Src/TransformFunctions/arm_rfft_fast_f64.o ./Src/TransformFunctions/arm_rfft_fast_f64.su ./Src/TransformFunctions/arm_rfft_fast_init_f16.d ./Src/TransformFunctions/arm_rfft_fast_init_f16.o ./Src/TransformFunctions/arm_rfft_fast_init_f16.su ./Src/TransformFunctions/arm_rfft_fast_init_f32.d ./Src/TransformFunctions/arm_rfft_fast_init_f32.o ./Src/TransformFunctions/arm_rfft_fast_init_f32.su ./Src/TransformFunctions/arm_rfft_fast_init_f64.d ./Src/TransformFunctions/arm_rfft_fast_init_f64.o ./Src/TransformFunctions/arm_rfft_fast_init_f64.su ./Src/TransformFunctions/arm_rfft_init_f32.d ./Src/TransformFunctions/arm_rfft_init_f32.o ./Src/TransformFunctions/arm_rfft_init_f32.su ./Src/TransformFunctions/arm_rfft_init_q15.d ./Src/TransformFunctions/arm_rfft_init_q15.o ./Src/TransformFunctions/arm_rfft_init_q15.su ./Src/TransformFunctions/arm_rfft_init_q31.d ./Src/TransformFunctions/arm_rfft_init_q31.o ./Src/TransformFunctions/arm_rfft_init_q31.su ./Src/TransformFunctions/arm_rfft_q15.d ./Src/TransformFunctions/arm_rfft_q15.o ./Src/TransformFunctions/arm_rfft_q15.su ./Src/TransformFunctions/arm_rfft_q31.d ./Src/TransformFunctions/arm_rfft_q31.o ./Src/TransformFunctions/arm_rfft_q31.su

.PHONY: clean-Src-2f-TransformFunctions

