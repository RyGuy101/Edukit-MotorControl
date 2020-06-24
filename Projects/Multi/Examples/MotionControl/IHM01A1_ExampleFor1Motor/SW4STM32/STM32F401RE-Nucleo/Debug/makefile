################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include Example/User/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/subdir.mk
-include Drivers/CMSIS/subdir.mk
-include Drivers/BSP/X-NUCLEO-IHMxx/subdir.mk
-include Drivers/BSP/STM32F4xx-Nucleo/subdir.mk
-include Drivers/BSP/Components/l6474/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F401RE-Nucleo.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F401RE-Nucleo.list \

OBJCOPY_BIN += \
STM32F401RE-Nucleo.bin \


# All Target
all: STM32F401RE-Nucleo.elf secondary-outputs

# Tool invocations
STM32F401RE-Nucleo.elf: $(OBJS) $(USER_OBJS) /Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/SW4STM32/STM32F401RE-Nucleo/LinkerScript.ld
	arm-none-eabi-gcc -o "STM32F401RE-Nucleo.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/wkaiser/Documents/STMicroelectronics_Motor_Control_Curriculum/Edukit_Resources/Edukit_Firmware/Release_6-10-2020/Project/MC_curriculum/Projects/Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/SW4STM32/STM32F401RE-Nucleo/LinkerScript.ld" --specs=nosys.specs -Wl,-Map="STM32F401RE-Nucleo.map" -Wl,--gc-sections -static  -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F401RE-Nucleo.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F401RE-Nucleo.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32F401RE-Nucleo.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32F401RE-Nucleo.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
