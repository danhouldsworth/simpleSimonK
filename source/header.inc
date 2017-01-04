;-- Device ----------------------------------------------------------------
.include "../include/m8def.inc"

;*********************
; PORT B definitions *
;*********************
;.equ			= 7
;.equ			= 6
;.equ			= 5	(sck)
;.equ			= 4	(miso)
;.equ			= 3	(mosi)
.equ	BpFET		= 2	;o
.equ	CpFET		= 1	;o
.equ	rcp_in		= 0	;ICP1 r/c pulse input

.equ	INIT_PB		= (1 << BpFET) | (1 << CpFET)
.equ	DIR_PB		= (1 << BpFET) | (1 << CpFET)

;*********************
; PORT C definitions *
;*********************
;.equ	mux_voltage	= 7	; ADC7 voltage input (18k from Vbat, 3.3k to gnd, 10.10V -> 1.565V at ADC7)
;.equ	mux_temperature	= 6	; ADC6 temperature input (3.3k from +5V, 10k NTC to gnd)
;.equ	i2c_clk		= 5	; ADC5/SCL
;.equ	i2c_data	= 4	; ADC4/SDA
.equ	red_led		= 3	; o
.equ	green_led	= 2	; o
.equ	mux_b		= 1	; ADC1 phase input
.equ	mux_a		= 0	; ADC0 phase input

.equ	INIT_PC		= 0
.equ	DIR_PC		= 0

;*********************
; PORT D definitions *
;*********************
;.equ	mux_c		= 7 (comparator AN1)
;.equ	sense_star	= 6 (comparator AN0)
.equ	CnFET		= 5
.equ	BnFET		= 4
.equ	AnFET		= 3
.equ	ApFET		= 2
;.equ	txd		= 1
;.equ	rxd		= 0

.equ	INIT_PD		= (1 << ApFET)
.equ	DIR_PD		= (1 << AnFET) | (1 << BnFET) | (1 << CnFET) | (1 << ApFET)
;--------------------------------------------------------------------------

.equ	RCP_TOT		= 2	; Number of 65536us periods before considering rc pulse lost

.equ	MAX_RC_PULS	= 2100	; Throw away any pulses longer than this
.equ	MIN_RC_PULS	= 900	; Throw away any pulses shorter than this

.equ	CPU_MHZ		= 16 	; Ticks per us

; Minimum PWM on-time (too low and FETs won't turn on, hard starting)
.equ	MIN_DUTY	= 56 	; * CPU_MHZ / 16

; Number of PWM steps (too high and PWM frequency drops into audible range)
.equ	POWER_RANGE	= 1000 + MIN_DUTY

.equ	MAX_POWER	= (POWER_RANGE-1)
.equ	PWR_COOL_START	= (POWER_RANGE/24) ; Power limit while starting to reduce heating
.equ	PWR_MIN_START	= (POWER_RANGE/6) ; Power limit while starting (to start)
.equ	PWR_MAX_START	= (POWER_RANGE/4) ; Power limit while starting (if still not running)

.equ	TIMEOUT_START	= 48000	; Timeout per commutation for ZC during starting
.equ	START_DSTEP_US	= 8	; Microseconds per start delay step
.equ	START_DELAY_INC	= 15	; Wait step count increase (wraps in a byte)

.equ	ENOUGH_GOODIES	= 12	; This many start cycles without timeout will transition to running mode
.equ	ZC_CHECK_FAST	= 12	; Number of ZC check loops under which PWM noise should not matter
.equ	ZC_CHECK_MAX	= POWER_RANGE / 32 ; ~27 Limit ZC checking to about 1/2 PWM interval
.equ	ZC_CHECK_MIN	= 3

.equ	T0CLK		= (1<<CS01)	; clk/8 == 2MHz
.equ	T1CLK		= (1<<CS10) | (1<<ICES1) | (1<<ICNC1)	; clk/1 == 16MHz
.equ	T2CLK		= (1<<CS20)	; clk/1 == 16MHz


;**** **** **** **** ****
; Register Definitions
.def	temp5		= r0		; aux temporary (L) (limited operations)
.def	temp6		= r1		; aux temporary (H) (limited operations)
.def	duty_l		= r2		; on duty cycle low, one's complement
.def	duty_h		= r3		; on duty cycle high
.def	off_duty_l	= r4		; off duty cycle low, one's complement
.def	off_duty_h	= r5		; off duty cycle high
.def	rx_l		= r6		; received throttle low
.def	rx_h		= r7		; received throttle high
.def	tcnt2h		= r8		; timer2 high byte
.def	i_sreg		= r9		; status register save in interrupts
.def	temp7		= r10		; really aux temporary (limited operations)
;.def	rc_timeout	= r11
.def	sys_control_l	= r12		; duty limit low (word register aligned)
.def	sys_control_h	= r13		; duty limit high
.def	timing_duty_l	= r14		; timing duty limit low
.def	timing_duty_h	= r15		; timing duty limit high
.def	flags0		= r16	; state flags
	.equ	OCT1_PENDING	= 0	; if set, output compare interrupt is pending
	.equ	SET_DUTY	= 1	; if set when armed, set duty during evaluate_rc
.def	flags1		= r17	; state flags
	.equ	POWER_ON	= 0	; if set, switching fets is enabled
	.equ	FULL_POWER	= 1	; 100% on - don't switch off, but do OFF_CYCLE working
	.equ	EVAL_RC		= 4	; if set, evaluate rc command while waiting for OCT1
	.equ	ACO_EDGE_HIGH	= 5	; if set, looking for ACO high - same bit position as ACO
	.equ	STARTUP		= 6	; if set, startup-phase is active
.def	flags2		= r18
	.equ	A_FET		= 0	; if set, A FET is being PWMed
	.equ	B_FET		= 1	; if set, B FET is being PWMed
	.equ	C_FET		= 2	; if set, C FET is being PWMed
.def	i_temp3		= r19		; interrupt temporary
.def	i_temp1		= r20		; interrupt temporary
.def	i_temp2		= r21		; interrupt temporary
.def	temp3		= r22		; main temporary (L)
.def	temp4		= r23		; main temporary (H)
.def	temp1		= r24		; main temporary (L), adiw-capable
.def	temp2		= r25		; main temporary (H), adiw-capable

; XL: general temporary
; XH: general temporary
; YL: general temporary
; YH: general temporary
; ZL: Next PWM interrupt vector (low)
; ZH: Next PWM interrupt vector (high, stays at zero) -- used as "zero" register

;-- FET driving macros ---------------------------------------------------
.macro AnFET_on
		sbi	PORTD, AnFET
.endmacro
.macro AnFET_off
		cbi	PORTD, AnFET
.endmacro
.macro ApFET_on
		cbi	PORTD, ApFET
.endmacro
.macro ApFET_off
		sbi	PORTD, ApFET
.endmacro
.macro BnFET_on
		sbi	PORTD, BnFET
.endmacro
.macro BnFET_off
		cbi	PORTD, BnFET
.endmacro
.macro BpFET_on
		cbi	PORTB, BpFET
.endmacro
.macro BpFET_off
		sbi	PORTB, BpFET
.endmacro
.macro CnFET_on
		sbi	PORTD, CnFET
.endmacro
.macro CnFET_off
		cbi	PORTD, CnFET
.endmacro
.macro CpFET_on
		cbi	PORTB, CpFET
.endmacro
.macro CpFET_off
		sbi	PORTB, CpFET
.endmacro

;-- Analog comparator sense macros ---------------------------------------
; We enable and disable the ADC to override ACME when one of the sense
; pins is AIN1 instead of an ADC pin. In the future, this will allow
; reading from the ADC at the same time.
.macro set_comp_phase_a
		cbi	ADMUX, 0	; set comparator multiplexer to phase A
		cbi	ADCSRA, ADEN	; Disable ADC if we enabled it to get AIN1
.endmacro
.macro set_comp_phase_b
		sbi	ADMUX, 0	; set comparator multiplexer to phase B
		cbi	ADCSRA, ADEN	; Disable ADC if we enabled it to get AIN1
.endmacro
.macro set_comp_phase_c
		sbi	ADCSRA, ADEN	; Enable ADC to effectively disable ACME
.endmacro

;-----bko-----------------------------------------------------------------

.macro com1com2
		; Bp off, Ap on
		set_comp_phase_b
		BpFET_off
		sbrc	flags1, POWER_ON
		ApFET_on
.endmacro

.macro com2com3
		; Cn off, Bn on
		set_comp_phase_c
		ldi	flags2, (1<<B_FET) ; Clear all but this FET flag
		CpFET_off
		CnFET_off
.endmacro

.macro com3com4
		; Ap off, Cp on
		set_comp_phase_a
		ApFET_off
		sbrc	flags1, POWER_ON
		CpFET_on
.endmacro

.macro com4com5
		; Bn off, An on
		set_comp_phase_b
		ldi	flags2, (1<<A_FET) ; Clear all but this FET flag
		BpFET_off
		BnFET_off
.endmacro

.macro com5com6
		; Cp off, Bp on
		set_comp_phase_c
		CpFET_off
		sbrc	flags1, POWER_ON
		BpFET_on
.endmacro

.macro com6com1
		; An off, Cn on
		set_comp_phase_a
		ldi	flags2, (1<<C_FET) ; Clear all but this FET flag
		ApFET_off
		AnFET_off
.endmacro

.MACRO RED_on
	sbi	DDRC, red_led
.ENDMACRO
.MACRO RED_off
	cbi	DDRC, red_led
.ENDMACRO
.MACRO GRN_on
	sbi	DDRC, green_led
.ENDMACRO
.MACRO GRN_off
	cbi	DDRC, green_led
.ENDMACRO
