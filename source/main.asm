;--------------------------------------------------------------------------
; simpleSimonK - Super simple ESC firmware
;--------------------------------------------------------------------------

;-- Device ----------------------------------------------------------------
.include "m8def.inc"
;-- Declarations ----------------------------------------------------------
.include "header.inc"
;-- Instruction extension macros ------------------------------------------
.include "macros.inc"
;-- SRAM definitions / DSEG -----------------------------------------------
.include "sram.inc"
;-- Interupt table & service routines -------------------------------------
.include "interrupts.inc"
;-- MAIN ROUTINES ---------------------------------------------------------
.include "commutations.inc"
;--------------------------------------------------------------------------

; Store immediate in SRAM
.macro stsi
		ldi	temp1, @1
		sts	@0, temp1
.endmacro

; Immediate out to any port via temp1 (all reg on mega8 are <0x3f)
.macro outi
	ldi	temp1, @1
	out	@0, temp1
.endmacro

; Load 2-byte immediate
.macro ldi2
		ldi	@0, byte1(@2)
		ldi	@1, byte2(@2)
.endmacro

; Load 3-byte immediate
.macro ldi3
		ldi	@0, byte1(@3)
		ldi	@1, byte2(@3)
		ldi	@2, byte3(@3)
.endmacro

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

.macro all_pFETs_off
		ApFET_off
		BpFET_off
		CpFET_off
.endmacro

.macro all_nFETs_off 		; 3clks vs 6above !!
		in	@0, PORTD
		cbr	@0, (1<<AnFET) | (1<<BnFET) | (1<<CnFET)
		out	PORTD, @0
.endmacro

.macro nFET_brake
		in	i_temp1, PORTD
		sbr	i_temp1, (1<<AnFET) | (1<<BnFET) | (1<<CnFET)
		out	PORTD, i_temp1
.endmacro

.macro set_only_this_flag
		;cbr	flags2, (1 << A_FET) | (1 << B_FET) | (1 << C_FET)
		ldi	flags2, (1 << @0) 	; could just do ldi if careful for 1clk vs 2
.endmacro

;-- Analog comparator sense macros ---------------------------------------
; We enable and disable the ADC to override ACME when one of the sense
; pins is AIN1 instead of an ADC pin. In the future, this will allow
; reading from the ADC at the same time.

.macro comp_init
		in	@0, SFIOR
		sbr	@0, (1<<ACME)	; set Analog Comparator Multiplexer Enable
		out	SFIOR, @0
.endmacro
.macro set_comp_phase_a
		outi	ADMUX, mux_a	; set comparator multiplexer to phase A
		cbi	ADCSRA, ADEN	; Disable ADC if we enabled it to get AIN1
.endmacro
.macro set_comp_phase_b
		outi	ADMUX, mux_b	; set comparator multiplexer to phase B
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
		set_only_this_flag B_FET
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
		set_only_this_flag A_FET
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
		set_only_this_flag C_FET
		ApFET_off
		AnFET_off
.endmacro

; REVERSE ORDER
.macro com1com6
		; An on, Cn off
		set_comp_phase_c
		set_only_this_flag A_FET
		CpFET_off
		CnFET_off
.endmacro

.macro com2com1
		; Bp on, Ap off
		set_comp_phase_a
		ApFET_off
		sbrc	flags1, POWER_ON
		BpFET_on
.endmacro

.macro com3com2
		; Cn on, Bn off
		set_comp_phase_b
		set_only_this_flag C_FET
		BpFET_off
		BnFET_off
.endmacro

.macro com4com3
		; Ap on, Cp off
		set_comp_phase_c
		CpFET_off
		sbrc	flags1, POWER_ON
		ApFET_on
.endmacro

.macro com5com4
		; Bn on, An off
		set_comp_phase_a
		set_only_this_flag B_FET
		ApFET_off
		AnFET_off
.endmacro

.macro com6com5
		; Cp on, Bp off
		set_comp_phase_b
		BpFET_off
		sbrc	flags1, POWER_ON
		CpFET_on
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

