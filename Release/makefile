################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
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

# Add inputs and outputs from these tool invocations to the build variables 
FLASH_IMAGE += \
Test4x7-SegDisp.hex \

EEPROM_IMAGE += \
Test4x7-SegDisp.eep \

SIZEDUMMY += \
sizedummy \

AVRDUDEDUMMY += \
avrdudedummy \


# All Target
all: Test4x7-SegDisp.elf secondary-outputs

# Tool invocations
Test4x7-SegDisp.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C Linker'
	avr-gcc -Wl,-Map,Test4x7-SegDisp.map -mmcu=attiny2313a -o "Test4x7-SegDisp.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

Test4x7-SegDisp.hex: Test4x7-SegDisp.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex Test4x7-SegDisp.elf  "Test4x7-SegDisp.hex"
	@echo 'Finished building: $@'
	@echo ' '

Test4x7-SegDisp.eep: Test4x7-SegDisp.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex Test4x7-SegDisp.elf  "Test4x7-SegDisp.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: Test4x7-SegDisp.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny2313a Test4x7-SegDisp.elf
	@echo 'Finished building: $@'
	@echo ' '

avrdudedummy: Test4x7-SegDisp.elf
	@echo 'Invoking: AVRDude'
	/usr/bin/avrdude -pt2313a -cusbasp -Uflash:w:Test4x7-SegDisp.hex:a -Ueeprom:w:Test4x7-SegDisp.eep:a
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(FLASH_IMAGE)$(ELFS)$(OBJS)$(ASM_DEPS)$(EEPROM_IMAGE)$(AVRDUDEDUMMY)$(S_DEPS)$(SIZEDUMMY)$(S_UPPER_DEPS)$(C_DEPS) Test4x7-SegDisp.elf
	-@echo ' '

secondary-outputs: $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY) $(AVRDUDEDUMMY)

.PHONY: all clean dependents

-include ../makefile.targets
