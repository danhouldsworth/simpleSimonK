# simpleSimonK

## A learning project to create a simplified version of the popular SimonK ESC for my bespoke setup

The code libraries have been modified and stripped down for my needs.

### The only ESCs it supports are :

* My favourite ESC the AfroNfet 12Amp, 20Amp, 20Amp-slim & 30Amp (using PWM bootloading)
* (As I have many old ones lying around) The Turnigy / 3DR / jDrones stock ESC (using ISP flashing tool)

### My bespoke needs driving the project :

* I'm interested in multicopters with fast response rates (both up and down the throttle range) and won't be using ESC/motor/prop combos that require throttle smoothing or ramping features.
* The support of dozens of boards / motor combos is distracting me from being able to understand the code.
* Some features like I2C (as is too slow), or RC_Reverse will not get used in my applications.
* I'd like to avoid manual calibration (which I find so cumbersome it almost takes the fun out of multi-copters). I far prefer to re-flash with known and recorded settings.

That, and I'm trying to learn all this stuff, so have tinkered where appropriate (like the power on melody...!)

### Notes to self :

* Use Sublime Text with AVR ASM to parse, and stick with tab space of 8.
* Build tool chain AVRA and AVRDUDE are installed with homebrew.
* AVRA 1.3.0 on OS Mavericks has issue with command on same line as labels / C debugger. Rather than reformat wholesale (this would harm readability) I've elected to complile on OSX Mountain lion until AVRA is fixed.

### Questions on learning topics:

1. Understand the underlying mechanics / electronics of 'Active Freewheeling', 'Regenerative Braking', 'Demag compensation' and whether setting COMP_PWM = 1 is a true implementation of either?
2. Understand response times (such as those measured by http://owenduffy.net/blog/?p=1439 or http://www.rcgroups.com/forums/showthread.php?t=1250488&page=2), and a suitable measurement / metric. Ie. timing from i) the initiation of signal from FC loop, ii) the sending of signal from FC board, iii) the receipt of signal at ESC, iv) the parsing of signal at ESC.... and measurement to i) current draw, ii) rpm. [Obviously the most useful metric is FC loop to rpm, but this is effected by choice of PWM / I2C speed etc]
3. Understand if / when a bootloader overwrites itself. Ideally I should include bootloader only when I know its changed / corrupted and ISP programming. Is there ever a case for including it when PWM flashing?

### Non-exhaustive list of concepts learned :

* Active Freewheeling (aka Synchronous Rectification) - uses MOSFET (very low resistance) rather than diode (~0.7v drop) during undriven phase of PWM duty cycle. Saves energy in low duty cycle throttle settings.
* Regenerative Braking - Effectively shorts one or more of the DrivenHigh, DrivenLow and Comparator phases. Rotational energy is converted into current flowing in that phase, but as far as I can tell *is not* recaptured by the LiPo or ESC caps. I need to look further as to whether the current induces current in the next phase and *saves* the battery from powering the next commutation but this seems unlikely.
* Demag compensation - related to issues with detetecting zero crossing (ZC) of the comparator phase in situations where high currents in long windings can take longer than 30degs to demagnetise and therefore spoil a normal reading. Compensation seems to be allowing longer, and cutting power if several failed ZC detections have occured (syncronisation loss).

### TODO :

So far I've simply neatened and stripped out the parts of code that weren't getting used (by me), and set my preferred parameters including maximum braking speed. It flies very well, on the CNC258 with Afro30amp and enabled a slight increase of PIDs when braking was increased.

Going forward, I'd like to experiment with:

* Use MOTOR_ID to autoset motor direction for clean wiring, and use conditional timing to take turns in playing the status-ok tune. (I should then make a point of wiring all quads so that Motor1 has correct spin with MOTOR_REVERSE=0, or annotate a list in the repo)
* Controlling via UART - however I've noticed that braking is not queried unless using PWM input? Maybe UART is not very well supported? This might be a project in itself....
* Attempt to pass back commutation speed with Motor_ID back along UART [where FC can calculate RPM based on pole count, and proxies for output power based on prop size]
* Learn more about Makefiles. Folders for board files and a build folder
* Make an ISP flashing tool for the Afro (different for the 20Amp Slim board) like I have for the Turnigy based ESC's. See question on Bootloaders above.

-----

I've included a link to Simon's README below which is very comprehensive - I've learned a thing or two about motors, ESCs and Atmel assembler through reading this alone.

https://github.com/sim-/tgy
