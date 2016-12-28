.cseg
.org 0
		rjmp reset	; reset
		reti		; ext_int0	INT0addr=$001	; External Interrupt0 Vector Address
		reti		; ext_int1	INT1addr=$002	; External Interrupt1 Vector Address
		reti		; t2oc_int	OC2addr =$003	; Output Compare2 Interrupt Vector Addre
		ijmp		; t2ovfl_int	OVF2addr=$004	; Overflow2 Interrupt Vector Address
		rjmp rcp_int	; icp1_int	ICP1addr=$005	; Input Capture1 Interrupt Vector Addres
		rjmp t1oca_int	; t1oca_int	OC1Aaddr=$006	; Output Compare1A Interrupt Vector Addr
		reti		; t1ocb_int	OC1Baddr=$007	; Output Compare1B Interrupt Vector Addr
		rjmp t1ovfl_int	; t1ovfl_int	OVF1addr=$008	; Overflow1 Interrupt Vector Address
		reti		; t0ovfl_int	OVF0addr=$009	; Overflow0 Interrupt Vector Address
		reti		; spi_int	SPIaddr =$00a	; SPI Interrupt Vector Address
		reti 		; urxc		URXCaddr=$00b	; USART Receive Complete Interrupt Vecto
		reti		; udre		UDREaddr=$00c	; USART Data Register Empty Interrupt Ve
		reti		; utxc		UTXCaddr=$00d	; USART Transmit Complete Interrupt Vect
		reti		; adc_int	ADCCaddr=$00e	; ADC Interrupt Vector Address
		reti		; eep_int	ERDYaddr=$00f	; EEPROM Interrupt Vector Address
		reti		; aci_int	ACIaddr =$010	; Analog Comparator Interrupt Vector Add
		reti 	 	; twi_int	TWIaddr =$011	; Irq. vector address for Two-Wire Inter
		reti		; spmc_int	SPMaddr =$012	; SPM complete Interrupt Vector Address

;-----bko-----------------------------------------------------------------
; timer1 output compare interrupt
t1oca_int:	in	i_sreg, SREG
		lds	i_temp1, ocr1ax
		subi	i_temp1, 1
		brcc	t1oca_int1
		cbr	flags0, (1<<OCT1_PENDING)	; signal OCT1A passed
t1oca_int1:	sts	ocr1ax, i_temp1
		out	SREG, i_sreg
		reti

;-----bko-----------------------------------------------------------------
; timer1 overflow interrupt (happens every 4096µs)
t1ovfl_int:	in	i_sreg, SREG
		lds	i_temp1, tcnt1x
		inc	i_temp1
		sts	tcnt1x, i_temp1
		andi	i_temp1, 15			; Every 16 overflows
		brne	t1ovfl_int1
		tst	rc_timeout
		breq	t1ovfl_int1
		dec	rc_timeout
t1ovfl_int1:	out	SREG, i_sreg
		reti
;-----bko-----------------------------------------------------------------

;-----bko-----------------------------------------------------------------
; NOTE: This interrupt uses the 16-bit atomic timer read/write register
; by reading TCNT1L and TCNT1H, so this interrupt must be disabled before
; any other 16-bit timer options happen that might use the same register
; (see "Accessing 16-bit registers" in the Atmel documentation)
; icp1 = rc pulse input, if enabled
rcp_int:
		in	i_temp1, ICR1L		; get captured timer values
		in	i_temp2, ICR1H
		in	i_temp3, TCCR1B
		sbrs	i_temp3, ICES1		; evaluate edge of this interrupt
		rjmp	falling_edge
rising_edge:
		in	i_sreg, SREG
		; Stuff this rise time plus MAX_RC_PULS into OCR1B.
		; We use this both to save the time it went high and
		; to get an interrupt to indicate high timeout.
		adiwx	i_temp1, i_temp2, MAX_RC_PULS * CPU_MHZ
		out	OCR1BH, i_temp2
		out	OCR1BL, i_temp1
		outi	TCCR1B, T1CLK & ~(1<<ICES1) 	; Set next int to falling edge
		ldi	i_temp1, (1<<OCF1B)		; Clear OCF1B flag
		out	TIFR, i_temp1
		out	SREG, i_sreg
		reti

rcpint_fail:
		in	i_sreg, SREG
		clr	rc_timeout
		rjmp	rcpint_exit

falling_edge:
		in	i_sreg, TIFR
		sbrc	i_sreg, OCF1B		; Too long high would set OCF1B
		rjmp	rcpint_fail
		in	i_sreg, SREG
		movw	rx_l, i_temp1		; Guaranteed to be valid, store immediately
		in	i_temp1, OCR1BL		; No atomic temp register used to read OCR1* registers
		in	i_temp2, OCR1BH
		sbi2	i_temp1, i_temp2, MAX_RC_PULS * CPU_MHZ	; Put back to start time
		sub	rx_l, i_temp1		; Subtract start time from current time
		sbc	rx_h, i_temp2
		sbr	flags1, (1<<EVAL_RC)
rcpint_exit:	outi	TCCR1B, T1CLK 		; Set next int to rising edge
		out	SREG, i_sreg
		reti

;-----bko-----------------------------------------------------------------
reset:		clr	r0
		out	SREG, r0		; Clear interrupts and flags
		ldi2	ZL, ZH, RAMEND 		; Set up stack
		out	SPH, ZH
		out	SPL, ZL
clear_loop:	st	-Z, r0 			; Clear RAM and all registers
		cpi	ZL, SRAM_START
		cpc	ZH, r0
		brne	clear_loop1
		ldi	ZL, 0x1e		; Start clearing registers
clear_loop1:	cp	ZL, r0
		cpc	ZH, r0
		brne	clear_loop		; Leaves with all registers (r0 through ZH) at 0

	; Initialize ports
		outi	PORTB, INIT_PB
		outi	DDRB, DIR_PB
		outi	PORTC, INIT_PC
		outi	DDRC, DIR_PC
		outi	PORTD, INIT_PD
		outi	DDRD, DIR_PD

	; Start timers except output PWM
		outi	TCCR0, T0CLK		; timer0: beep control, delays
		outi	TCCR1B, T1CLK		; timer1: commutation timing, RC pulse measurement
		out	TCCR2, ZH		; timer2: PWM, stopped

	; Enable watchdog (WDTON may be set or unset)
		outi	WDTCR, (1<<WDCE) | (1<<WDE)
		outi	WDTCR, (1<<WDE)		; Fastest option: ~16.3ms timeout

	; Enable timer interrupts (we only do this late to improve beep quality)
		ldi	temp1, (1<<TOIE1) | (1<<OCIE1A) | (1<<TOIE2) | (1<<TICIE1)
		out	TIFR, temp1		; Clear TOIE1, OCIE1A, and TOIE2 flags
		out	TIMSK, temp1		; Enable t1ovfl_int, t1oca_int, t2ovfl_int

		rcall	wait30ms		; Wait for power to settle -- this must be no longer than 64ms

	; set up SRAM parameters
		stsi 	puls_high_l, low(32000) ; FULL_RC_PULS * CPU_MHZ
		stsi 	puls_high_h, high(32000)
		stsi 	puls_low_l, low(16000)
		stsi 	puls_low_h, high(16000)
		stsi 	fwd_scale_l, low(4096) 	; (POWER_RANGE - MIN_DUTY) * 65536 / (1000us * 16MHz)
		stsi 	fwd_scale_h, high(4096)
		stsi 	brake_sub, 0xff 	; BRAKE_SPEED fastest

		rjmp	control_start
