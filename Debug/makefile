################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
stop_mode_2_with_rtc.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
stop_mode_2_with_rtc.list \

OBJCOPY_BIN += \
stop_mode_2_with_rtc.bin \


# All Target
all: stop_mode_2_with_rtc.elf secondary-outputs

# Tool invocations
stop_mode_2_with_rtc.elf: $(OBJS) $(USER_OBJS) C:\Users\Elad_m\Desktop\test\stop_mode_2_with_rtc\STM32L452RETX_FLASH.ld
	arm-none-eabi-gcc -o "stop_mode_2_with_rtc.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Elad_m\Desktop\test\stop_mode_2_with_rtc\STM32L452RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="stop_mode_2_with_rtc.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

stop_mode_2_with_rtc.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "stop_mode_2_with_rtc.list"
	@echo 'Finished building: $@'
	@echo ' '

stop_mode_2_with_rtc.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "stop_mode_2_with_rtc.bin"
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
