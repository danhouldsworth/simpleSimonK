# This Makefile is part of my simpleSimonK project for learning purposes and carries no implied support or garauntees whatsoever!

# Note#1 - usbmodem00065771 should be replaced with FTDI device used for hack flashing to the SPI pads
# Note#2 - SLAB_USBtoUART is the silicon labs driver that I use for the AfroESC programmer
# Note#3 - To my knowledge, reading / setting fuses over PWM-pin bootloader is meaningless, but flash / prog mem can be done.

SHELL = /bin/bash

# TARGET = afro_nfet
TARGET = afro_hv
# TARGET = bs_nfet
# TARGET = tgy

all:
	#********************************************************************************************
	#                                                                                           *
	#                                                                                           *
	# Warning#1 : this should now make using AVRA on OS X Mavericks but untested as yet 	    *
	# Warning#2 : I've overhauled the Makefile considerably! so must test soon...	 	    *
	# Warning#3 : Using a modified version of m8def.inc to make Pragma use (or lack of) clear   *
	#                                                                                           *
	#                                                                                           *
	#********************************************************************************************
	avra -fI --define $(TARGET)_esc main.asm

clean:
	-rm -f *.obj *.eep.hex *.eeprom *.hex *.cof

upload_ISP:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -U flash:w:$(TARGET).hex:i

upload_PWM:
	avrdude -c stk500v2 -p m8 -P /dev/tty.SLAB_USBtoUART -b 9600 -U flash:w:$(TARGET).hex:i

read_ISP:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h

read_PWM:
	# Note : To my knowledge, reading / setting fuses with PWM is meaningless, but flash / prog mem can be done.
	avrdude -c stk500v2 -p m8 -P /dev/tty.SLAB_USBtoUART -b 9600 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h
