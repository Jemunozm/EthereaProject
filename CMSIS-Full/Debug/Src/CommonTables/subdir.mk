################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/CommonTables/CommonTables.c \
../Src/CommonTables/CommonTablesF16.c \
../Src/CommonTables/arm_common_tables.c \
../Src/CommonTables/arm_common_tables_f16.c \
../Src/CommonTables/arm_const_structs.c \
../Src/CommonTables/arm_const_structs_f16.c \
../Src/CommonTables/arm_mve_tables.c \
../Src/CommonTables/arm_mve_tables_f16.c 

OBJS += \
./Src/CommonTables/CommonTables.o \
./Src/CommonTables/CommonTablesF16.o \
./Src/CommonTables/arm_common_tables.o \
./Src/CommonTables/arm_common_tables_f16.o \
./Src/CommonTables/arm_const_structs.o \
./Src/CommonTables/arm_const_structs_f16.o \
./Src/CommonTables/arm_mve_tables.o \
./Src/CommonTables/arm_mve_tables_f16.o 

C_DEPS += \
./Src/CommonTables/CommonTables.d \
./Src/CommonTables/CommonTablesF16.d \
./Src/CommonTables/arm_common_tables.d \
./Src/CommonTables/arm_common_tables_f16.d \
./Src/CommonTables/arm_const_structs.d \
./Src/CommonTables/arm_const_structs_f16.d \
./Src/CommonTables/arm_mve_tables.d \
./Src/CommonTables/arm_mve_tables_f16.d 


# Each subdirectory must supply rules for building sources it contributes
Src/CommonTables/%.o Src/CommonTables/%.su: ../Src/CommonTables/%.c Src/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F411RE -DSTM32 -DSTM32F4 -DSTM32F411RETx -DSTM32F411xE -c -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"/home/e420-00/STM32CubeIDE/workspaceyoUN/CMSIS-repo/STM32Cube_FW_F4_V1.27.0/Drivers/CMSIS/Include" -I../Inc -I"/CMSIS-Full/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-CommonTables

clean-Src-2f-CommonTables:
	-$(RM) ./Src/CommonTables/CommonTables.d ./Src/CommonTables/CommonTables.o ./Src/CommonTables/CommonTables.su ./Src/CommonTables/CommonTablesF16.d ./Src/CommonTables/CommonTablesF16.o ./Src/CommonTables/CommonTablesF16.su ./Src/CommonTables/arm_common_tables.d ./Src/CommonTables/arm_common_tables.o ./Src/CommonTables/arm_common_tables.su ./Src/CommonTables/arm_common_tables_f16.d ./Src/CommonTables/arm_common_tables_f16.o ./Src/CommonTables/arm_common_tables_f16.su ./Src/CommonTables/arm_const_structs.d ./Src/CommonTables/arm_const_structs.o ./Src/CommonTables/arm_const_structs.su ./Src/CommonTables/arm_const_structs_f16.d ./Src/CommonTables/arm_const_structs_f16.o ./Src/CommonTables/arm_const_structs_f16.su ./Src/CommonTables/arm_mve_tables.d ./Src/CommonTables/arm_mve_tables.o ./Src/CommonTables/arm_mve_tables.su ./Src/CommonTables/arm_mve_tables_f16.d ./Src/CommonTables/arm_mve_tables_f16.o ./Src/CommonTables/arm_mve_tables_f16.su

.PHONY: clean-Src-2f-CommonTables

