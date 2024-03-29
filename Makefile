# Makefile for PIC programming using GPASM
# Copyright (c) 2021, Andrew Williams (LabRat)
# based on works from Matt Sarnoff
# Copyright (c) 2015, Matt Sarnoff (msarnoff.org)
# v1.0, February 12, 2015
# Released under a 3-clause BSD license: see the accompanying LICENSE file.
#
# Run `make` to build the project as a .hex file.
# Run `make flash` to program the device.
#
# MPLAB X is required if using a PICkit 3 to program the device.
# This Makefile assumes it's installed in /Applications/microchip.

########## Project-specific definitions ##########

# Project name
OUT = nrf_loader

# Source files to assemble
ASM = nrf_loader.asm

# USB serial number for the device
SERIAL_NUMBER=1

# (use `make list-devices` if not known)
AS_DEVICE = p16f1823

# The MDB-specific part number of the chip, used for programming with MDB
# (should be the actual PIC part number, e.g. PIC16LF1454)
MDB_DEVICE = PIC16F1823

########## Build settings ##########

AS = gpasm
DASM = gpdasm
MPLABX_DIR = /Applications/microchip/mplabx
MDB = $(MPLABX_DIR)/mplab_ide.app/Contents/Resources/mplab_ide/bin/mdb.sh



########## Make rules ##########

HEX = $(OUT).hex

import_list.inc: export_list $(HEX) ./export_symbols.sh
	./export_symbols.sh

# Link
$(HEX): $(ASM)
	$(AS) -p $(AS_DEVICE) -DSERIAL_NUMBER=$(SERIAL_NUMBER) -o $(HEX) $(ASM) -r DEC

# Disassemble
dis: $(HEX)
	$(DASM) -p p$(AS_DEVICE) $(HEX)

# Flash
flash: $(HEX)
	@echo "Device $(MDB_DEVICE)" \
		"\nSet system.disableerrormsg true" \
		"\nHwtool PICkit3 -p" \
		"\nSet programoptions.eraseb4program true" \
		"\nProgram \"$(HEX)\"" \
		"\nQuit\n" > __prog.cmd
	@$(MDB) __prog.cmd; status=$$?; rm -f __prog.cmd MPLABXLog.*; exit $$status

# Test
test:
	./nrf16f1prog  -l 2 -r 768 -D 0x1d0002 -p /dev/ttyACM0 -B 115200 -w 16 sample_app/blink.hex

# List supported device types
list-devices:
	@$(AS) -l

# Clean
clean:
	rm -f $(ASM:.asm=.lst) $(HEX) $(OUT).cod $(OUT).lst import_list.inc

.PHONY: all flash clean list-devices

