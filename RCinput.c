#include <avr/interrupt.h>

void evaluate_rc(){
	flags1 &= ~(1 << EVAL_RC);	// clear flag EVAL_RC
}

ISR(reset){
	// Clear all SRAM
		// ; Initialize ports
		// 	outi	PORTB, INIT_PB
		// 	outi	DDRB, DIR_PB
		// 	outi	PORTC, INIT_PC
		// 	outi	DDRC, DIR_PC
		// 	outi	PORTD, INIT_PD
		// 	outi	DDRD, DIR_PD

		// ; Start timers except output PWM
		// 	outi	TCCR0, T0CLK		; timer0: beep control, delays
		// 	outi	TCCR1B, T1CLK		; timer1: commutation timing, RC pulse measurement
		// 	out	TCCR2, ZH		; timer2: PWM, stopped

		// ; Enable watchdog (WDTON may be set or unset)
		// 	outi	WDTCR, (1<<WDCE) | (1<<WDE)
		// 	outi	WDTCR, (1<<WDE)		; Fastest option: ~16.3ms timeout

		// ; Enable timer interrupts (we only do this late to improve beep quality)
		// 	ldi	temp1, (1<<TOIE1) | (1<<OCIE1A) | (1<<TOIE2) | (1<<TICIE1)
		// 	out	TIFR, temp1		; Clear TOIE1, OCIE1A, and TOIE2 flags
		// 	out	TIMSK, temp1		; Enable t1ovfl_int, t1oca_int, t2ovfl_int

		// ; set up SRAM parameters
		// 	stsi 	puls_high_l, low(32000) ; FULL_RC_PULS * CPU_MHZ
		// 	stsi 	puls_high_h, high(32000)
		// 	stsi 	puls_low_l, low(16000)
		// 	stsi 	puls_low_h, high(16000)
		// 	stsi 	fwd_scale_l, low(4096) 	; (POWER_RANGE - MIN_DUTY) * 0xffff / (1000us * 16MHz)
		// 	stsi 	fwd_scale_h, high(4096)

	wait_ms(30);
}

ISR(icp1_int){
	// rcp_int:
	// 		in	i_temp1, ICR1L		; get captured timer values
	// 		in	i_temp2, ICR1H
	// 		in	i_temp3, TCCR1B
	// 		sbrs	i_temp3, ICES1		; evaluate edge of this interrupt
	// 		rjmp	falling_edge
	// rising_edge:
	// 		in	i_sreg, SREG
	// 		; Stuff this rise time plus MAX_RC_PULS into OCR1B.
	// 		; We use this both to save the time it went high and
	// 		; to get an interrupt to indicate high timeout.
	// 		ldi 	i_temp3, low(MAX_RC_PULS * CPU_MHZ)
	// 		add 	i_temp1, i_temp3
	// 		ldi 	i_temp3, high(MAX_RC_PULS * CPU_MHZ)
	// 		adc 	i_temp2, i_temp3
	// 		out	OCR1BH, i_temp2
	// 		out	OCR1BL, i_temp1
	// 		outi	TCCR1B, T1CLK & ~(1<<ICES1) 	; Set next int to falling edge
	// 		ldi	i_temp1, (1<<OCF1B)		; Clear OCF1B flag
	// 		out	TIFR, i_temp1
	// 		out	SREG, i_sreg
	// 		reti

	// rcpint_fail:
	// 		in	i_sreg, SREG
	// 		clr	rc_timeout
	// 		rjmp	rcpint_exit

	// falling_edge:
	// 		in	i_sreg, TIFR
	// 		sbrc	i_sreg, OCF1B		; Too long high would set OCF1B
	// 		rjmp	rcpint_fail
	// 		in	i_sreg, SREG
	// 		movw	rx_l, i_temp1		; Guaranteed to be valid, store immediately
	// 		in	i_temp1, OCR1BL		; No atomic temp register used to read OCR1* registers
	// 		in	i_temp2, OCR1BH
	// 		subi 	i_temp1, byte1(MAX_RC_PULS * CPU_MHZ)	; Put back to start time
	// 		sbci 	i_temp2, byte2(MAX_RC_PULS * CPU_MHZ)	; Put back to start time
	// 		sub	rx_l, i_temp1		; Subtract start time from current time
	// 		sbc	rx_h, i_temp2
	// 		sbr	flags1, (1<<EVAL_RC)
	// rcpint_exit:	outi	TCCR1B, T1CLK 		; Set next int to rising edge
	// 		out	SREG, i_sreg
	// 		reti
}

ISR(t1oca_int){
	ocr1ax--; // So our 24bit countdown compare === (ocr1ax << 16) | (ocr1ah << 8) | ocr1al
	if (ocr1ax == 0) {
		flags0 &= ~(1 << OCT1_PENDING);
	}
}

ISR(t1ovfl_int){
	tcnt1x++; // So our 24bit timer === (tcnt1x << 16) | TNCT1
}

ISR(t2ovfl_int){
	//Possible IJMPs
	pwm_on
	pwm_off
	pwm_on_ptr
	pwm_wdr
}
