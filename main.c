// #include <avr/interrupt.h>
#include <avr/io.h>

int main(void){
	DDRC = (1 << PORTC3) | (1 << PORTC2);
	PORTC = 0; // LEDs on with logic 0

	while(1){}
}

// Global settings

// Timer 0 =  2MHz 		Beep control / delays
// Timer 1 = 16MHz		Commutation timeing / RC pulse measurement
// Timer 2 = 16MHz 		FET PWM

// Enable fastest watchdog reset (16.3ms)
// Copies RC pulse settings to RAM.


// Check out :
// better braking https://github.com/sim-/tgy/commit/d7f80f6e2242c6232fc88de376e5c3637b3e020e
// Faster timing (16bit vs 24bit timing for 25000rpm+) https://github.com/sim-/tgy/commit/103edb5a08571cd4b7ccf0be5427eb3065666533
// RedBrick 70A (PortD fets) https://github.com/sim-/tgy/commit/2d790fba39122ac8b296402fdb17ad8ba102288f