# Makefile for NRF based dimmer
# Andrew Williams
# Apr 20, 2022
#
# Run `make` to build the project as a .hex file.
# Run `make flash` to program the device.
#

########## Project-specific definitions ##########

# Project name
OUT = nrf_factory

# Assembly files to assemble
ASM = nrf_factory.asm

# The SDCC-specific (case sensitive) part number of the chip
# (use `make list-devices` if not known)
AS_DEVICE = 16f1823


########## Build settings ##########

PROG = ../nrf16f1prog	# change this once you've installed the script
AS = gpasm
DASM = gpdasm
PORT = -p /dev/ttyACM0

########## Make rules ##########

HEX = $(OUT).hex


# Link
$(HEX): $(ASM) 
	$(AS) -I .. -p$(AS_DEVICE) -o $@ $<


# Disassemble
dis: $(HEX)
	$(DASM) -p p$(AS_DEVICE) $(HEX) | less

# Flash
flash: $(HEX)
	sudo $(PROG) $(PORT) $(HEX)

# Clean
clean:
	rm -f $(ASM:.asm=.lst) $(HEX) $(OUT).cod $(OUT).lst

.PHONY: all flash clean dis

