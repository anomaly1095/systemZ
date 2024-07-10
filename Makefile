# Toolchain commands
AS = arm-none-eabi-as     # Assembler
LD = arm-none-eabi-ld     # Linker
OBJDUMP = arm-none-eabi-objdump   # Object dump
OBJCOPY = arm-none-eabi-objcopy   # Object copy

# Compiler and linker flags
DBG_FLAGS = -g -Wall       # Debugging and warning flags
ASM_FLAGS = -mcpu=cortex-m4 -mthumb $(DBG_FLAGS)   # Assembler flags
LD_FLAGS = --script linker-script.ld -e reset_handler -Map=$(MAP2)   # Linker flags

# Directories
SRC = src       # Source directory
BIN = bin       # Binary output directory
DOC = docs      # Documentation directory

# Source files and objects
SRCS = $(wildcard $(SRC)/*.s) $(wildcard $(SRC)/drivers/core/*.s) $(wildcard $(SRC)/drivers/extern/*.s)   # Assembly source files
OBJS = $(patsubst $(SRC)/%.s,$(BIN)/%.o,$(SRCS))   # Object files list
ELF = $(BIN)/firmware.elf   # Executable ELF file
MAP1 = $(DOC)/firmware1.map   # Memory map output file 1
MAP2 = $(DOC)/firmware2.map   # Memory map output file 2
BIN_FILE = $(BIN)/firmware.bin   # Binary output file

# Default target
all: $(ELF) dump $(BIN_FILE)

# Linking step
$(ELF): $(OBJS) | $(BIN)
	$(LD) $(LD_FLAGS) -o $@ $^
	# $(LD): Link object files ($(OBJS)) into executable ($@) using $(LD_FLAGS)

# Assembly compilation step
$(BIN)/%.o: $(SRC)/%.s | $(@D)
	$(AS) $(ASM_FLAGS) -o $@ $<
	# $(AS): Assemble source file ($<) into object file ($@) using $(ASM_FLAGS)

# Generate memory map
dump: $(ELF)
	$(OBJDUMP) -h $< > $(MAP1)
	# $(OBJDUMP): Generate memory map ($<) and save to $(MAP1)

# Flash firmware to STM32 using OpenOCD
flash:
	sudo openocd -f openocd.cfg -d -c "program $(ELF) verify reset exit"
	# Flash $(ELF) to STM32 using OpenOCD

# Create binary file
$(BIN_FILE): $(ELF)
	$(OBJCOPY) -O binary $< $@
	# $(OBJCOPY): Create binary file ($@) from ELF file ($<)

# Clean up
clean:
	rm -f $(OBJS) $(ELF) $(MAP1) $(MAP2) $(BIN_FILE)
	# Clean up object files, executable, maps, and binary file

# Phony targets
.PHONY: all dump clean flash
	# Declare all, dump, clean, and flash as phony targets
