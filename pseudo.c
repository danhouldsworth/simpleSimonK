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


