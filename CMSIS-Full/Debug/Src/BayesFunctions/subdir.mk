################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/BayesFunctions/BayesFunctions.c \
../Src/BayesFunctions/BayesFunctionsF16.c \
../Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.c \
../Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.c 

OBJS += \
./Src/BayesFunctions/BayesFunctions.o \
./Src/BayesFunctions/BayesFunctionsF16.o \
./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o \
./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o 

C_DEPS += \
./Src/BayesFunctions/BayesFunctions.d \
./Src/BayesFunctions/BayesFunctionsF16.d \
./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d \
./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Src/BayesFunctions/%.o Src/BayesFunctions/%.su: ../Src/BayesFunctions/%.c Src/BayesFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-BayesFunctions

clean-Src-2f-BayesFunctions:
	-$(RM) ./Src/BayesFunctions/BayesFunctions.d ./Src/BayesFunctions/BayesFunctions.o ./Src/BayesFunctions/BayesFunctions.su ./Src/BayesFunctions/BayesFunctionsF16.d ./Src/BayesFunctions/BayesFunctionsF16.o ./Src/BayesFunctions/BayesFunctionsF16.su ./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.d ./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.o ./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f16.su ./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.d ./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.o ./Src/BayesFunctions/arm_gaussian_naive_bayes_predict_f32.su

.PHONY: clean-Src-2f-BayesFunctions

