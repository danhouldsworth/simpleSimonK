# simpleSimonK

## A cleaned, tailored version of the SimonK ESC firmware to better facilitate my learning and use

The code libraries have been modified, and stripped down for my needs.

### The only ESCs it supports are :

* The Turnigy / 3DR / jDrones (which I have many old ones lying around) which SimonKs main file is based on.
* My favourite ESC the AfroNfet 12Amp, 20Amp, 20Amp-slim & 30Amp

### Personal views driving some of the changes :

* I have a strong desire to avoid manual calibration, which is so cumbersome it almost takes the fun out of multi-copters. I far prefer to re-flash with absolute, known and recorded settings.
* Some features (like I2C as a method of communicating to ESC's as is too slow) will not get used.
* The support of dozens of boards / motor combos is distracting me from being able to understand the code.
* I'm interested in multicopters with fast repsonse rates and won't be using ESC/motor/prop combos that require throttle smoothing or ramping features.

That, and I'm trying to learn all this stuff, so have tinkered where appropriate (like the power on tune...!)

### Notes to self :

* Use Sublime Text with AVR ASM to parse, and stick with tab space of 8.
* Build tool chain AVRA and AVRDUDE are installed with homebrew.
* AVRA 1.3.0 on OS Mavericks has issue with command on same line as labels / C debugger. Rather than reformat wholesale (this would harm readability) I've elected to complile on OSX Mountain lion until AVRA is fixed.

### Questions to self:

1. Understand the underlying mechanics / electronics of 'Active Freewheeling', 'Regenerative Braking', 'Demag compensation' and whether setting COMP_PWM = 1 is a true implementation of either?
2. Understand response times (such as those measured by http://owenduffy.net/blog/?p=1439), and a suitable measurement / metric. Ie. timing from i) the initiation of signal from FC loop, ii) the sending of signal from FC board, iii) the receipt of signal at ESC, iv) the parsing of signal at ESC.... and measurement to i) current draw, ii) rpm. [Obviously the most useful metric is FC loop to rpm, but this is effected by choice of PWM / I2C speed etc]

### Non-exhaustive list of concepts learned :

* Active Freewheeling (aka Synchronous Rectification) - uses MOSFET (very low resistance) rather than diode (~0.7v drop) during undriven phase of PWM duty cycle. Saves energy in low duty cycle throttle settings.
* Regenerative Braking - Effectively shorts one or more of the DrivenHigh, DrivenLow and Comparator phases. Rotational energy is converted into current flowing in that phase, but as far as I can tell *is not* recaptured by the LiPo or ESC caps. I need to look further as to whether the current induces current in the next phase and *saves* the battery from powering the next commutation but this seems unlikely.
* Demag compensation - related to issues with detetecting zero crossing (ZC) of the comparator phase in situations where high currents in long windings can take longer than 30degs to demagnetise and therefore spoil a normal reading. Compensation seems to be allowing longer, and cutting power if several failed ZC detections have occured (syncronisation loss).

### TODO :

So far I've simply neatened and stripped out the parts of code that weren't getting used (by me).
Going forward, I'd like to experiment with:

* Try increasing BRAKE_POWER and BRAKE_SPEED to see effect on stabilty
* Controlling via UART, and then using MOTOR_ID to autoset up motordirection for clean wiring, and timing the power on tunes so can clearly hear which motor is communicating in turn.
* Attempt to pass back commutation speed with Motor_ID back along UART [where FC can calculate RPM based on pole count, and proxies for output power based on prop size]

-----

I've included a link to Simon's README below as is very comprehensive - I've learned a thing or two about motors, ESCs and Atmel assembler through reading this alone.

https://github.com/sim-/tgy
