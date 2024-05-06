################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Src/TransformFunctions/subdir.mk
-include Src/SupportFunctions/subdir.mk
-include Src/StatisticsFunctions/subdir.mk
-include Src/SVMFunctions/subdir.mk
-include Src/QuaternionMathFunctions/subdir.mk
-include Src/MatrixFunctions/subdir.mk
-include Src/InterpolationFunctions/subdir.mk
-include Src/FilteringFunctions/subdir.mk
-include Src/FastMathFunctions/subdir.mk
-include Src/DistanceFunctions/subdir.mk
-include Src/ControllerFunctions/subdir.mk
-include Src/ComplexMathFunctions/subdir.mk
-include Src/CommonTables/subdir.mk
-include Src/BayesFunctions/subdir.mk
-include Src/BasicMathFunctions/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := CMSIS-Full
BUILD_ARTIFACT_EXTENSION := a
BUILD_ARTIFACT_PREFIX := lib
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
ARCHIVES += \
libCMSIS-Full.a \


# All Target
all: main-build

# Main-build Target
main-build: libCMSIS-Full.a secondary-outputs

# Tool invocations
libCMSIS-Full.a: $(OBJS) $(USER_OBJS) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-ar -r  "libCMSIS-Full.a" @"objects.list" $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) libCMSIS-Full.a
	-@echo ' '

secondary-outputs:

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
