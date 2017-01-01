# SHELL = /bin/bash

asm:
	avra -fI main.asm

build:
	avr-gcc 		-mmcu=ATmega8 -Wall -DF_CPU=16000000 main.c -o main.elf
	avr-objcopy 	-O ihex main.elf main.hex

clean:
	-rm -f *.obj *.eep.hex *.eeprom *.hex *.cof *.elf

USB_flash:
	avrdude -c usbasp -P usb -p m8 -U flash:w:main.hex:i

USB_read:
	avrdude -c usbasp -P usb -p m8 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h

ISP_flash:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -U flash:w:main.hex:i

ISP_read:
	avrdude -c avrisp2 -p m8 -P /dev/tty.usbmodem00065771 -v -U flash:r:flash.hex:i -U eeprom:r:eeprom.hex:i -U lfuse:r:-:h -U hfuse:r:-:h
