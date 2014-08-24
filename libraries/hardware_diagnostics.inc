;-------------------------------------------------------------------------
;
; hardware_diagnostics.inc
;
;-------------------------------------------------------------------------

;-- Hardware diagnostics -------------------------------------------------
; Any brushless ESC based on the ATmega8 or similar must tie the sense
; neutral star to AIN0, and the three sense lines to three ADC pins or
; two ADC pins and AIN1. All of these pins are also normal I/O pins, so
; we can drive them and see if the ADC values move. A value that does not
; move indicates a shorted FET or that an incorrect board target has been
; flashed.
;
; Note: Some FET drivers such as the LM5109 can pull up the output a bit,
; making the "stuck high" test return a false positive. Perhaps it would
; be sufficient to test that the phases read as 0 _or_ can be pulled down
; by driving AIN0 to ground.
;
; In typical conditions on the ATmega8, I/O pins transition to low at
; about 1.42V and to high at about 1.86V. The ADC is 10-bit, however, and
; should work even with a strong sense voltage divider.
;
; Throughout all of this, the motor may be spinning. If so, we should wait
; long enough that each phase falls to 0V and all tests succeed.
;
.if CHECK_HARDWARE
.set ADC_READ_NEEDED = 1
.equ MAX_CHECK_LOOPS = 5000			; ADC check takes ~200us

hardware_check:
		clt

		; First, check that all sense lines are low.
		.if defined(mux_a)
		ldi	XL, 1			; Error code 1: Phase A stuck high
		ldi	temp4, mux_a
		rcall	check_sense_low
		.endif

		.if defined(mux_b)
		ldi	XL, 2			; Error code 2: Phase B stuck high
		ldi	temp4, mux_b
		rcall	check_sense_low
		.endif

		.if defined(mux_c)
		ldi	XL, 3			; Error code 3: Phase C stuck high
		ldi	temp4, mux_c
		rcall	check_sense_low
		.endif

		.if !defined(mux_a) || !defined(mux_b) || !defined(mux_c)
		ldi	XL, 4			; Error code 4: AIN1 stuck high
		ldi2	YL, YH, MAX_CHECK_LOOPS
check_ain1_low:	sbiw	YL, 1
		sbic	PIND, 7			; Skip loop if AIN1 low
		brne	check_ain1_low
		rcall	hw_error_eq
		.endif

		ldi	XL, 5			; Error code 5: AIN0 stuck high
		ldi2	YL, YH, MAX_CHECK_LOOPS
check_ain0_low:	sbiw	YL, 1
		sbic	PIND, 6			; Skip loop if AIN0 low
		brne	check_ain0_low
		rcall	hw_error_eq

		brts	hardware_check		; Do not allow further tests if stuck high

		; If nothing is stuck high, pull up the motor by driving
		; the emulated neutral (AIN0) high, and try driving each
		; phase low. While driven, leakage through the star is
		; eliminated, so one phase will not influence another
		; unless a motor is connected. A phase on AIN1 cannot be
		; read by the ADC, so we must skip it.

		sbi	DDRD, 6
		sbi	PORTD, 6		; Drive AIN0 high

		.if defined(mux_a)
		rcall	wait30ms		; There might be some capacitance
		ldi	XL, 6			; Error code 6: Phase A low-side drive broken
		ldi	temp4, mux_a
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		AnFET_on			; Drive down this phase (we've established that it was 0V above).
		rcall	adc_read		; FET turn-on will easily beat ADC initialization
		AnFET_off
		rcall	hw_error_y_le_temp12
		.endif

		.if defined(mux_b)
		rcall	wait30ms
		ldi	XL, 7			; Error code 7: Phase B low-side drive broken
		ldi	temp4, mux_b
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		BnFET_on			; Drive down this phase (we've established that it was 0V above).
		rcall	adc_read		; FET turn-on will easily beat ADC initialization
		BnFET_off
		rcall	hw_error_y_le_temp12
		.endif

		.if defined(mux_c)
		rcall	wait30ms
		ldi	XL, 8			; Error code 8: Phase C low-side drive broken
		ldi	temp4, mux_c
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		CnFET_on			; Drive down this phase (we've established that it was 0V above).
		rcall	adc_read		; FET turn-on will easily beat ADC initialization
		CnFET_off
		rcall	hw_error_y_le_temp12
		.endif

		cbi	PORTD, 6		; Sink on AIN0 (help to pull down the outputs)
		rcall	wait30ms

		.if defined(mux_a)
		ldi	XL, 9			; Error code 9: Phase A high-side drive broken
		ldi	temp4, mux_a
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		ApFET_on			; Drive up this phase.
		rcall	adc_read		; Waste time for high side to turn off
		ApFET_off
		rcall	hw_error_temp12_le_y
		.endif

		.if defined(mux_b)
		ldi	XL, 10			; Error code 10: Phase B high-side drive broken
		ldi	temp4, mux_b
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		BpFET_on			; Drive up this phase.
		rcall	adc_read		; Waste time for high side to turn off
		BpFET_off
		rcall	hw_error_temp12_le_y
		.endif

		.if defined(mux_c)
		ldi	XL, 11			; Error code 11: Phase C high-side drive broken
		ldi	temp4, mux_c
		rcall	adc_read
		movw	YL, temp1		; Save ADC value (hopefully non-zero)
		CpFET_on			; Drive up this phase.
		rcall	adc_read		; Waste time for high side to turn off
		CpFET_off
		rcall	hw_error_temp12_le_y
		.endif

		cbi	DDRD, 6			; Restore tristated AIN0
		ret

check_sense_low:
		ldi2	YL, YH, MAX_CHECK_LOOPS
check_sense_low1:
		rcall	adc_read
		; Up to 3.5V to account for ADC offset or driver pull-up.
		.equ	OFF_MAX_ADC = 35 * 1024 * O_GROUND / (50 * (O_POWER + O_GROUND))
		sbiwx	temp1, temp2, OFF_MAX_ADC
		brcs	check_sense_low_ret	; Return if pin reads low
		sbiw	YL, 1
		brne	check_sense_low1	; Loop until timeout
		rjmp	hw_error
check_sense_low_ret:
		ret

hw_error_temp12_le_y:
		cp	YL, temp1
		cpc	YH, temp2
		brcc	hw_error
		ret

hw_error_y_le_temp12:
		cp	temp1, YL
		cpc	temp2, YH
		brcc	hw_error
		ret

;-- Hardware error -------------------------------------------------------
; Blink an LED or beep XL times to indicate a hardware error.
; Beeping is possibly unsafe. The only other option is to stop.
hw_error_eq:
		brne	hw_error_ret
hw_error:
		mov	YL, XL
hw_error1:
		.if defined(red_led)
		RED_on
		rcall	wait120ms
		RED_off
		.elif defined(green_led)
		GRN_on
		rcall	wait120ms
		GRN_off
		.else
		rcall	beep_f1			; Low frequency is safer
		.endif
		rcall	wait240ms
		dec	YL
		brne	hw_error1
		rcall	wait240ms
		rcall	wait240ms
		set
hw_error_ret:	ret

.endif

