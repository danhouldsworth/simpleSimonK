# This Makefile is part of my simpleSimonK project for learning purposes and carries no implied support or garauntees whatsoever!

# Note#1 - usbmodem00065771 should be replaced with FTDI device used for hack flashing to the SPI pads
# Note#2 - SLAB_USBtoUART is the silicon labs driver that I use for the AfroESC programmer
# Note#3 - To my knowledge, reading / setting fuses over PWM-pin bootloader is meaningless, but flash / prog mem can be done.

# SHELL = /bin/bash

TARGET = afro_nfet
# TARGET = afro_hv // Only for 3-8s !! Not the new 4-6s which uses the nfet.inc
# TARGET = bs_nfet
# TARGET = tgy

all:
	#********************************************************************************************
	#                                                                                           *
	#                                                                                           *
	# Warning ! : Careful with Afros - the ~HV.inc burned the new 4-6s which work on ~nfet.inc  *
	# Warning#1 : this should now make using AVRA on OS X Mavericks but untested as yet 	    *
	# Warning#2 : I've overhauled the Makefile considerably! so must test soon...	     	    *
	# Warning#3 : Using a modified version of m8def.inc to make Pragma use (or lack of) clear   *
	# Warning#4 : Recently stripped out huge amounts of main.asm code so need to field test!    *
	#                                                                                           *
	#                                                                                           *
	#********************************************************************************************
	avra -fI --define $(TARGET)_esc main.asm

clean:
	-rm -f *.obj *.eep.hex *.eeprom *.hex *.cof

USB_flash:
	avrdude -c usbasp -P usb -p m8 -U flash:w:main.hex:i

USB_read:
	avrdude -c usbasp -P usb -p m8 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h

ISP_flash:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -U flash:w:main.hex:i

ISP_read:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h

PWM_flash:
	avrdude -c stk500v2 -p m8 -P /dev/tty.SLAB_USBtoUART -b 9600 -U flash:w:main.hex:i

PWM_read:
	# Note : To my knowledge, reading / setting fuses with PWM is meaningless, but flash / prog mem can be done.
	avrdude -c stk500v2 -p m8 -P /dev/tty.SLAB_USBtoUART -b 9600 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h
