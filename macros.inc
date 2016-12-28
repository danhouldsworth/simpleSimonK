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

; Immediate out to any port via temp1 (all reg on mega8 are <0x3f)
.macro outi
	ldi	temp1, @1
	out	@0, temp1
.endmacro

;-- FET driving macros ---------------------------------------------------
; Careful: "if" conditions split over multiple lines (with backslashes)
; work with arva, but avrasm2.exe silently produces wrong results.

.macro FET_on
.if (INIT_PB & ((@0 == PORTB) << @1)) | (INIT_PC & ((@0 == PORTC) << @1)) | (INIT_PD & ((@0 == PORTD) << @1))
		cbi	@0, @1
.else
		sbi	@0, @1
.endif
.endmacro

.macro FET_off
.if (INIT_PB & ((@0 == PORTB) << @1)) | (INIT_PC & ((@0 == PORTC) << @1)) | (INIT_PD & ((@0 == PORTD) << @1))
		sbi	@0, @1
.else
		cbi	@0, @1
.endif
.endmacro

.macro AnFET_on
		FET_on	AnFET_port, AnFET
.endmacro
.macro AnFET_off
		FET_off	AnFET_port, AnFET
.endmacro
.macro ApFET_on
		FET_on	ApFET_port, ApFET
.endmacro
.macro ApFET_off
		FET_off	ApFET_port, ApFET
.endmacro
.macro BnFET_on
		FET_on	BnFET_port, BnFET
.endmacro
.macro BnFET_off
		FET_off	BnFET_port, BnFET
.endmacro
.macro BpFET_on
		FET_on	BpFET_port, BpFET
.endmacro
.macro BpFET_off
		FET_off	BpFET_port, BpFET
.endmacro
.macro CnFET_on
		FET_on	CnFET_port, CnFET
.endmacro
.macro CnFET_off
		FET_off	CnFET_port, CnFET
.endmacro
.macro CpFET_on
		FET_on	CpFET_port, CpFET
.endmacro
.macro CpFET_off
		FET_off	CpFET_port, CpFET
.endmacro

.macro all_pFETs_off
.if ApFET_port != BpFET_port || ApFET_port != CpFET_port
		ApFET_off
		BpFET_off
		CpFET_off
.else
		in	@0, ApFET_port
	.if (INIT_PB & ((ApFET_port == PORTB) << ApFET)) | (INIT_PC & ((ApFET_port == PORTC) << ApFET)) | (INIT_PD & ((ApFET_port == PORTD) << ApFET))
		sbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
	.else
		cbr	@0, (1<<ApFET)+(1<<BpFET)+(1<<CpFET)
	.endif
		out	ApFET_port, @0
.endif
.endmacro

.macro all_nFETs_off
.if AnFET_port != BnFET_port || AnFET_port != CnFET_port
		AnFET_off
		BnFET_off
		CnFET_off
.else
		in	@0, AnFET_port
	.if (INIT_PB & ((AnFET_port == PORTB) << AnFET)) | (INIT_PC & ((AnFET_port == PORTC) << AnFET)) | (INIT_PD & ((AnFET_port == PORTD) << AnFET))
		sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	.else
		cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	.endif
		out	AnFET_port, @0
.endif
.endmacro

.macro nFET_brake
.if AnFET_port != BnFET_port || AnFET_port != CnFET_port
		AnFET_on
		BnFET_on
		CnFET_on
.else
		in	@0, AnFET_port
	.if (INIT_PB & ((AnFET_port == PORTB) << AnFET)) | (INIT_PC & ((AnFET_port == PORTC) << AnFET)) | (INIT_PD & ((AnFET_port == PORTD) << AnFET))
		cbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	.else
		sbr	@0, (1<<AnFET)+(1<<BnFET)+(1<<CnFET)
	.endif
		out	AnFET_port, @0
.endif
.endmacro

;-- Analog comparator sense macros ---------------------------------------
; We enable and disable the ADC to override ACME when one of the sense
; pins is AIN1 instead of an ADC pin. In the future, this will allow
; reading from the ADC at the same time.

.macro comp_init
		in	@0, SFIOR
		sbr	@0, (1<<ACME)	; set Analog Comparator Multiplexer Enable
		out	SFIOR, @0
	.if defined(mux_a) && defined(mux_b) && defined(mux_c)
		cbi	ADCSRA, ADEN	; Disable ADC to make sure ACME works
	.endif
.endmacro
.macro comp_adc_disable
	.if !defined(mux_a) || !defined(mux_b) || !defined(mux_c)
		cbi	ADCSRA, ADEN	; Disable ADC if we enabled it to get AIN1
	.endif
.endmacro
.macro comp_adc_enable
		sbi	ADCSRA, ADEN	; Eisable ADC to effectively disable ACME
.endmacro
.macro set_comp_phase_a
	.if defined(mux_a)
		ldi	@0, mux_a	; set comparator multiplexer to phase A
		out	ADMUX, @0
		comp_adc_disable
	.else
		comp_adc_enable
	.endif
.endmacro
.macro set_comp_phase_b
	.if defined(mux_b)
		ldi	@0, mux_b	; set comparator multiplexer to phase B
		out	ADMUX, @0
		comp_adc_disable
	.else
		comp_adc_enable
	.endif
.endmacro
.macro set_comp_phase_c
	.if defined(mux_c)
		ldi	@0, mux_c	; set comparator multiplexer to phase C
		out	ADMUX, @0
		comp_adc_disable
	.else
		comp_adc_enable
	.endif
.endmacro

; Short cycle delay without clobbering flags
.equ	MAX_BUSY_WAIT_CYCLES	= 32
.macro cycle_delay
.if @0 >= MAX_BUSY_WAIT_CYCLES
.error "cycle_delay too long"
.endif
.if @0 > 0
	.if @0 & 1
		nop
	.endif
	.if @0 & 2
		rjmp	PC + 1
	.endif
	.if @0 & 4
		rjmp	PC + 1
		rjmp	PC + 1
	.endif
	.if @0 & 8
		nop
		rcall	wait_ret		; 3 cycles to call + 4 to return
	.endif
	.if @0 & 16
		rjmp	PC + 1
		rcall	wait_ret
		rcall	wait_ret
	.endif
.endif
.endmacro

;-----bko-----------------------------------------------------------------
; *** commutation utilities ***

.macro com1com2
		; Bp off, Ap on
		set_comp_phase_b temp1
		BpFET_off
		sbrc	flags1, POWER_ON
		ApFET_on
.endmacro

.macro com2com1
		; Bp on, Ap off
		set_comp_phase_a temp1
		ApFET_off
		sbrc	flags1, POWER_ON
		BpFET_on
.endmacro

.macro com2com3
		; Cn off, Bn on
		set_comp_phase_c temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<B_FET)

		CpFET_off

		in	temp1, CnFET_port
		CnFET_off
		in	temp2, CnFET_port
		cpse	temp1, temp2
		BnFET_on
		sei
.endmacro

.macro com3com2
		; Cn on, Bn off
		set_comp_phase_b temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<C_FET)

		BpFET_off

		in	temp1, BnFET_port
		BnFET_off
		in	temp2, BnFET_port
		cpse	temp1, temp2
		CnFET_on
		sei
.endmacro

.macro com3com4
		; Ap off, Cp on
		set_comp_phase_a temp1
		ApFET_off
		sbrc	flags1, POWER_ON
		CpFET_on
.endmacro

.macro com4com3
		; Ap on, Cp off
		set_comp_phase_c temp1
		CpFET_off
		sbrc	flags1, POWER_ON
		ApFET_on
.endmacro

.macro com4com5
		; Bn off, An on
		set_comp_phase_b temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<A_FET)

		BpFET_off

		in	temp1, BnFET_port
		BnFET_off
		in	temp2, BnFET_port
		cpse	temp1, temp2
		AnFET_on
		sei
.endmacro

.macro com5com4
		; Bn on, An off
		set_comp_phase_a temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<B_FET)

		ApFET_off

		in	temp1, AnFET_port
		AnFET_off
		in	temp2, AnFET_port
		cpse	temp1, temp2
		BnFET_on
		sei
.endmacro

.macro com5com6
		; Cp off, Bp on
		set_comp_phase_c temp1
		CpFET_off
		sbrc	flags1, POWER_ON
		BpFET_on
.endmacro

.macro com6com5
		; Cp on, Bp off
		set_comp_phase_b temp1
		BpFET_off
		sbrc	flags1, POWER_ON
		CpFET_on
.endmacro

.macro com6com1
		; An off, Cn on
		set_comp_phase_a temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<C_FET)

		ApFET_off

		in	temp1, AnFET_port
		AnFET_off
		in	temp2, AnFET_port
		cpse	temp1, temp2
		CnFET_on
		sei
.endmacro

.macro com1com6
		; An on, Cn off
		set_comp_phase_c temp1
		cli
		cbr	flags2, ALL_FETS
		sbr	flags2, (1<<A_FET)

		CpFET_off

		in	temp1, CnFET_port
		CnFET_off
		in	temp2, CnFET_port
		cpse	temp1, temp2
		AnFET_on
		sei
.endmacro

