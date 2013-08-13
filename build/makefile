########################################
#   @author  Spark Application Team    #
#   @version V1.0.0                    #
#   @date    20-June-2013              #
########################################

-include ../makefile.init

RM := rm -rf
CORE_FIRMWARE?=core-firmware
USER_DIR?=user


# All of the sources participating in the build are defined here
-include sources.mk
-include startup/subdir.mk
-include src/subdir.mk
-include ../$(USER_DIR)/subdir.mk
-include libraries/STM32_USB-FS-Device_Driver/src/subdir.mk
-include libraries/STM32F10x_StdPeriph_Driver/src/subdir.mk
-include libraries/CC3000_Host_Driver/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs



# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
$(CORE_FIRMWARE).hex \

SECONDARY_LIST += \
$(CORE_FIRMWARE).lst \

SECONDARY_SIZE += \
$(CORE_FIRMWARE).siz \


# All Target
all: $(CORE_FIRMWARE).elf secondary-outputs

# Tool invocations
$(CORE_FIRMWARE).elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target..'
	arm-none-eabi-gcc -T"../linker/linker_stm32f10x_md_dfu.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,$(CORE_FIRMWARE).map -mcpu=cortex-m3 -mthumb -g3 -gdwarf-2 -o "$(CORE_FIRMWARE).elf" $(OBJS) $(USER_OBJS) $(LIBS)
	$(MAKE) --no-print-directory post-build

$(CORE_FIRMWARE).hex: $(CORE_FIRMWARE).elf
	arm-none-eabi-objcopy -O ihex $(CORE_FIRMWARE).elf  "$(CORE_FIRMWARE).hex"

$(CORE_FIRMWARE).lst: $(CORE_FIRMWARE).elf
	arm-none-eabi-objdump -h -S $(CORE_FIRMWARE).elf > "$(CORE_FIRMWARE).lst"

$(CORE_FIRMWARE).siz: $(CORE_FIRMWARE).elf
	arm-none-eabi-size  --format=berkeley $(CORE_FIRMWARE).elf
	-@echo 'Finished building target'

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(ASM_DEPS)$(S_UPPER_DEPS)$(EXECUTABLES)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE) $(CORE_FIRMWARE).map $(CORE_FIRMWARE).elf $(CORE_FIRMWARE).bin

post-build:
	-arm-none-eabi-objcopy -S -O binary "$(CORE_FIRMWARE).elf" "$(CORE_FIRMWARE).bin"

secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
