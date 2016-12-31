#include <custom/hardware.inc>

// Global settings

// Timer settings
// Use OSCAL to force 8MHz clock to pretty much 16MHz
// Timer 0 =  2MHz 		Beep control / delays
// Timer 1 = 16MHz		Commutation timeing / RC pulse measurement
// Timer 2 = 16MHz 		FET PWM

// Enable fastest watchdog reset (16.3ms)

// Copies RC pulse settings to RAM. First bytes redundent. Neutral redundent. Could do this direct
// Scale the pulse calcs
// Neatural appears redundent (as we don't have reverse)

// * We should calculate (puls_find_multiplicand:) manually, as would cut out quite a bit of code, progmem, defs, and SRAM
// * Also will understand it better


// Check out :
// better braking https://github.com/sim-/tgy/commit/d7f80f6e2242c6232fc88de376e5c3637b3e020e
// Faster timing (16bit vs 24bit timing for 25000rpm+) https://github.com/sim-/tgy/commit/103edb5a08571cd4b7ccf0be5427eb3065666533
// RedBrick 70A (PortD fets) https://github.com/sim-/tgy/commit/2d790fba39122ac8b296402fdb17ad8ba102288f


/*
BEEP:
ON  : 16us
OFF :
	200/2  *  16 (=1120 : ~1.5% duty)   REPEAT  80 times == 129k
	180/2  *  16 (=1120 : ~1.5% duty)   REPEAT 100 times == 145k
	160/2  *  16 (=1120 : ~1.5% duty)   REPEAT 120 times == 155k
	140/2  *  16 (=1120 : ~1.5% duty)   REPEAT 140 times == 159k

Not entirely equal durations...
*/