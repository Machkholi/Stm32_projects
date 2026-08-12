################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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


BUILD_ARTIFACT_NAME := DWIN_UART_POLLING_TEMPLATE
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
DWIN_UART_POLLING_TEMPLATE.elf \

MAP_FILES += \
DWIN_UART_POLLING_TEMPLATE.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
DWIN_UART_POLLING_TEMPLATE.list \


# All Target
all: main-build

# Main-build Target
main-build: DWIN_UART_POLLING_TEMPLATE.elf secondary-outputs

# Tool invocations
DWIN_UART_POLLING_TEMPLATE.elf DWIN_UART_POLLING_TEMPLATE.map: $(OBJS) $(USER_OBJS) D:\rahul\Stm32_projects\stm32f302\ workspace\TEMPLATE_CODE_FILE\DWIN_UART_POLLING_TEMPLATE\STM32F302R8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "DWIN_UART_POLLING_TEMPLATE.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\rahul\Stm32_projects\stm32f302 workspace\TEMPLATE_CODE_FILE\DWIN_UART_POLLING_TEMPLATE\STM32F302R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="DWIN_UART_POLLING_TEMPLATE.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

DWIN_UART_POLLING_TEMPLATE.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "DWIN_UART_POLLING_TEMPLATE.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) DWIN_UART_POLLING_TEMPLATE.elf DWIN_UART_POLLING_TEMPLATE.list DWIN_UART_POLLING_TEMPLATE.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
