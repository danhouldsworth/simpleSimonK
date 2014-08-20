# simpleSimonK

## A cleaned, tailored version of the SimonK ESC firmware to better facilitate my learning and use

The code libraries have been modified, and stripped down for my needs.

The only ESCs it supports are :

* The Turnigy / 3DR / jDrones (which I have many old ones lying around) which SimonKs main file is based on.
* My favourite ESC the AfroNfet 12Amp, 20Amp, 20Amp-slim & 30Amp

Personal views driving some of the changes :

  1) I have a strong desire to avoid manual calibration, which is so cumbersome it almost takes the fun out of multi-copters. I far prefer to re-flash with absolute, known and recorded settings.
  2) Some features (like I2C as a method of communicating to ESC's as is too slow) will not get used.
  3) The support of dozens of boards / motor combos is distracting from being able to understand the code.

That, and I'm trying to learn all this stuff, so have tinkered where appropriate...!

Notes to self :

* Use Sublime Text with AVR ASM to parse, and stick with tab space of 8.
* Build tool chain AVRA and AVRDUDE are installed with homebrew.
* AVRA 1.3.0 on OS Mavericks has issue with command on same line as labels / C debugger. Rather than reformat wholesale (this would harm readability) I've elected to complile on OSX Mountain lion until AVRA is fixed.

-----

I've included a link to Simon's README below as is very comprehensive - I've learned a thing or two about motors, ESCs and Atmel assembler through reading this alone.

https://github.com/sim-/tgy
