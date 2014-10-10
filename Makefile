# This Makefile compiles / uploads for AfroNfet boards & original Turnigy based boards only.
# It is part of my simpleSimonK project for learning purposes and carries no implied support or garauntees whatsoever!

# Note - usbmodem00065771 should be replaced with FTDI device used for hack flashing to the SPI pads
# Note - SLAB_USBtoUART is the silicon labs driver that I use for the AfroESC programmer

SHELL = /bin/bash

.SUFFIXES: .inc .hex

ALL_TARGETS = bs_nfet.hex afro_nfet.hex tgy.hex

all: $(ALL_TARGETS)

$(ALL_TARGETS): main.asm

.inc.hex:
	#********************************************************************************************
	#                                                                                           *
	#                                                                                           *
	# Warning : this will not make using AVRA on OS X Mavericks, use OS X Mountain Lion instead *
	#                                                                                           *
	#                                                                                           *
	#********************************************************************************************
	@test -e $*.asm || ln -s main.asm $*.asm
	@echo "avra -fI -o $@ -D $*_esc -e $*.eeprom -d $*.obj $*.asm"
	@set -o pipefail; avra -fI -o $@ -D $*_esc -e $*.eeprom -d $*.obj $*.asm 2>&1 | grep -v 'PRAGMA directives currently ignored'
	@test -L $*.asm && rm -f $*.asm || true

test: all

clean:
	-rm -f $(ALL_TARGETS) *.obj *.eep.hex *.eeprom *.hex *.cof afro_nfet.asm


upload:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -U flash:w:tgy.hex:i

upload_afro:
	avrdude -c stk500v2 -p m8 -P /dev/tty.SLAB_USBtoUART -b 9600 -U flash:w:afro_nfet.hex:i

read:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h

read_afro:
	avrdude -c stk500v2 -p m8 -P /dev/tty.SLAB_USBtoUART -b 9600 -v -U flash:r:flash_afroRead.hex:i -U eeprom:r:eeprom_afroRead.hex:i -U lfuse:r:-:h -U hfuse:r:-:h
