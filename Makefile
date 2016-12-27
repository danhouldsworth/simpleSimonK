# SHELL = /bin/bash

all:
	avra -fI main.asm

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
