
# # SystemZ Kernel <PRODUCTION BRANCH>
# 
# Copyright (C) 2024 Connexion Nord, Inc. or its affiliates. All Rights Reserved.
# 
# SPDX-License-Identifier: MIT
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# 
# <https://github.com/anomaly1095/systemZ>
# Author: Youssef Azaiez


AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OBJDUMP = arm-none-eabi-objdump
OBJCOPY = arm-none-eabi-objcopy

# Compiler and linker flags
DBG_FLAGS = -g -Wall
ASM_FLAGS = -mcpu=cortex-m4 -mthumb $(DBG_FLAGS)
LD_FLAGS = --script linker-script.ld -e reset_handler -Map=$(MAP2)

# Directories
SRC = src
BIN = bin
DOC = docs

# Source files and objects
SRCS = $(wildcard $(SRC)/*.s)
OBJS = $(patsubst $(SRC)/%.s,$(BIN)/%.o,$(SRCS))
ELF = $(BIN)/firmware.elf
MAP1 = $(DOC)/firmware1.map
MAP2 = $(DOC)/firmware2.map
BIN_FILE = $(BIN)/firmware.bin

# Default target
all: $(ELF) dump $(BIN_FILE)

# Linking step
$(ELF): $(OBJS) | $(BIN)
	$(LD) $(LD_FLAGS) -o $@ $^

# Assembly compilation step
$(BIN)/%.o: $(SRC)/%.s | $(BIN)
	$(AS) $(ASM_FLAGS) -o $@ $<

# Create output directory if it doesn't exist
$(BIN):
	mkdir -p $(BIN)

# Generate memory map
dump: $(ELF)
	$(OBJDUMP) -h $< > $(MAP1)

# Create binary file
$(BIN_FILE): $(ELF)
	$(OBJCOPY) -O binary $< $@

# Clean up
clean:
	rm -f $(OBJS) $(ELF) $(MAP) $(BIN_FILE)

# Phony targets
.PHONY: all dump clean flash
