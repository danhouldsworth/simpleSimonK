;-- Instruction extension macros -----------------------------------------
;
; macros.inc
;
;--------------------------------------------------------------------------

; Add any 16-bit immediate to a register pair (@0:@1 += @2), no Z flag
.macro adi2
	.if byte1(-@2)
		subi	@0, byte1(-@2)
		sbci	@1, byte1(-byte2(@2 + 0xff))
	.else
		subi	@1, byte1(-byte2(@2 + 0xff))
	.endif
.endmacro

; Smaller version for r24 and above, Z flag not reliable
.macro adiwx
	.if (@2) & ~0x3f
		adi2	@0, @1, @2
	.else
		adiw	@0, @2
	.endif
.endmacro

; Compare any 16-bit immediate from a register pair (@0:@1 -= @2, maybe clobbering @3)
.macro cpiz2
		cpi	@0, byte1(@2)
	.if byte2(@2)
		ldi	@3, byte2(@2)
		cpc	@1, @3
	.else
		cpc	@1, ZH
	.endif
.endmacro

; Compare any 16-bit immediate from a register pair (@0:@1 -= @2, maybe clobbering @3), no Z flag
; Do not follow by Z flag tests like breq, brne, brlt, brge, brlo, brsh!
; The idea here is that the low byte being compared with (subtracted by)
; 0 will never set carry, so skipping it and cpi'ing the high byte is the
; same other than the result of the Z flag.
.macro cpi2
	.if byte1(@2)
		cpiz2	@0, @1, @2, @3
	.else
		cpi	@1, byte2(@2)
	.endif
.endmacro

; Compare any 24-bit immediate from a register triplet (@0:@1:@2 -= @3, maybe clobbering @4)
.macro cpiz3
		cpi	@0, byte1(@3)
	.if byte2(@3)
		ldi	@4, byte2(@3)
		cpc	@1, @4
	.else
		cpc	@1, ZH
	.endif
	.if byte3(@3)
		ldi	@4, byte3(@3)
		cpc	@2, @4
	.else
		cpc	@2, ZH
	.endif
.endmacro

; Compare any 24-bit immediate from a register triplet (@0:@1:@2 -= @3, maybe clobbering @4)
; May not set Z flag, as above.
.macro cpi3
	.if byte1(@3)
		cpiz3	@0, @1, @2, @3, @4
	.else
		cpi2	@1, @2, @3 >> 8, @4
	.endif
.endmacro

; Subtract any 16-bit immediate from a register pair (@0:@1 -= @2), no Z flag
.macro sbi2
	.if byte1(@2)
		subi	@0, byte1(@2)
		sbci	@1, byte2(@2)
	.else
		subi	@1, byte2(@2)
	.endif
.endmacro

; Smaller version for r24 and above, Z flag not reliable
.macro sbiwx
	.if (@2) & ~0x3f
		sbi2	@0, @1, @2
	.else
		sbiw	@0, @2
	.endif
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
		in	@0, PORTD
		sbr	@0, (1<<AnFET) | (1<<BnFET) | (1<<CnFET)
		out	PORTD, @0
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
.macro comp_adc_disable
		cbi	ADCSRA, ADEN	; Disable ADC if we enabled it to get AIN1
.endmacro
.macro comp_adc_enable
		sbi	ADCSRA, ADEN	; Enable ADC to effectively disable ACME
.endmacro
.macro set_comp_phase_a
		outi	ADMUX, mux_a	; set comparator multiplexer to phase A
		comp_adc_disable
.endmacro
.macro set_comp_phase_b
		outi	ADMUX, mux_b	; set comparator multiplexer to phase B
		comp_adc_disable
.endmacro
.macro set_comp_phase_c
		comp_adc_enable
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
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<B_FET) 	; could just do ldi if careful for 1clk vs 2
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
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<A_FET)
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
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<C_FET)
		ApFET_off
		AnFET_off
.endmacro

; REVERSE ORDER
.macro com1com6
		; An on, Cn off
		set_comp_phase_c
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<A_FET)
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
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<C_FET)
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
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<B_FET)
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
