;-----bko-----------------------------------------------------------------
; Timer2 overflow interrupt (output PWM) -- the interrupt vector actually
; "ijmp"s to Z, which should point to one of these entry points.
;
; We try to avoid clobbering (and thus needing to save/restore) flags;
; in, out, mov, ldi, cpse, etc. do not modify any flags, while dec does.
;
; We used to check the comparator (ACSR) here to help starting, since PWM
; switching is what introduces noise that affects the comparator result.
; However, timing of this is very sensitive to FET characteristics, and
; would work well on some boards but not at all on others without waiting
; another 100-200ns, which was enough to break other boards. So, instead,
; we do all of the ACSR sampling outside of the interrupt and do digital
; filtering. The AVR interrupt overhead also helps to shield the noise.
;
; We reload TCNT2 as the very last step so as to reduce PWM dead areas
; between the reti and the next interrupt vector execution, which still
; takes a good 4 (reti) + 4 (interrupt call) + 2 (ijmp) cycles. We also
; try to keep the switch on close to the start of pwm_on and switch off
; close to the end of pwm_off to minimize the power bump at full power.
;
; pwm_*_high and pwm_off_again are called when the particular on/off cycle
; is longer than will fit in 8 bits. This is tracked in tcnt2h.

pwm_on_high:
		in	i_sreg, SREG
		dec	tcnt2h
		brne	pwm_on_again
		ldi	ZL, low(pwm_on)
pwm_on_again:	out	SREG, i_sreg
		reti

pwm_off_again:
		in	i_sreg, SREG
		dec	tcnt2h
		out	SREG, i_sreg
		reti

pwm_on:
		sbrc	flags2, A_FET
		ApFET_off
		sbrc	flags2, B_FET
		BpFET_off
		sbrc	flags2, C_FET
		CpFET_off
		sbrc	flags2, A_FET
		AnFET_on
		sbrc	flags2, B_FET
		BnFET_on
		sbrc	flags2, C_FET
		CnFET_on
		ldi	ZL, low(pwm_off)
		mov	tcnt2h, duty_h
		out	TCNT2, duty_l
		reti

pwm_wdr:	wdr				; Just reset watchdog
		reti

pwm_off:
		cpse	tcnt2h, ZH		; 2 cycles to skip when tcnt2h is 0
		rjmp	pwm_off_again
		wdr				; 1 cycle: watchdog reset
		sbrc	flags1, FULL_POWER	; 2 cycles to skip if not full power
		rjmp	pwm_on			; None of this off stuff if full power
		lds	ZL, pwm_on_ptr		; 2 cycles
		mov	tcnt2h, off_duty_h	; 1 cycle
		sbrc	flags2, A_FET		; 2 cycles if skip, 1 cycle otherwise
		AnFET_off			; 2 cycles (off at 12 cycles from entry)
		sbrc	flags2, B_FET		; Offset by 2 cycles here,
		BnFET_off			; but still equal on-time
		sbrc	flags2, C_FET
		CnFET_off
		out	TCNT2, off_duty_l	; 1 cycle
		reti

;-----bko-----------------------------------------------------------------
update_timing:
		cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		lds	temp3, tcnt1x
		in	temp4, TIFR
		sei
		cpi	temp2, 0x80		; tcnt1x is right when TCNT1h[7] set;
		sbrc	temp4, TOV1		; otherwise, if TOV1 is/was pending,
		adc	temp3, ZH		; increment our copy of tcnt1x.

	; Calculate the timing from the last two zero-crossings
		lds	YL, last_tcnt1_l	; last -> Y
		lds	YH, last_tcnt1_h
		lds	temp7, last_tcnt1_x
		sts	last_tcnt1_l, temp1
		sts	last_tcnt1_h, temp2
		sts	last_tcnt1_x, temp3
		lds	temp5, l2_tcnt1_l	; last2 -> temp5
		lds	temp6, l2_tcnt1_h
		lds	temp4, l2_tcnt1_x
		sts	l2_tcnt1_l, YL
		sts	l2_tcnt1_h, YH
		sts	l2_tcnt1_x, temp7

	; Cancel DC bias by starting our timing from the average of the
	; last two zero-crossings. Commutation phases always alternate.
	; Next start = (cur(c) - last2(a)) / 2 + last(b)
	; -> start=(c-b+(c-a)/2)/2+b
	;
	;                  (c - a)
	;         (c - b + -------)
	;                     2
	; start = ----------------- + b
	;                 2

		sub	temp1, temp5		; c' = c - a
		sbc	temp2, temp6
		sbc	temp3, temp4

		ldi2	XL, XH, MAX_POWER
		movw	timing_duty_l, XL

		sts	timing_l, temp1		; Store timing (120 degrees)
		sts	timing_h, temp2
		sts	timing_x, temp3

		lsr	temp3			; c'>>= 1 (shift to 60 degrees)
		ror	temp2
		ror	temp1

		lds	YL, last_tcnt1_l	; restore original c as a'
		lds	YH, last_tcnt1_h
		lds	temp7, last_tcnt1_x

		ldi	temp4, 256 * 15 / 60 	; Delay from timing advance (30degs advance would be zero delay. Set to 15.)
		rcall	update_timing_add_degrees

		sts	com_time_l, YL		; Store start of next commutation
		sts	com_time_h, YH
		sts	com_time_x, temp7
		rcall	set_ocr1a_abs		; Set timer for start of next commutation

		sbrc	flags1, EVAL_RC
		rjmp	evaluate_rc		; Set new duty either way
		; fall through to set_new_duty
;-----bko-----------------------------------------------------------------
; set_new_duty:	Unlike update_timing above, we try not to clobber XL, XH used as a loop
; counter in wait_for_edge.
		lds	YL, rc_duty_l
		lds	YH, rc_duty_h
set_new_duty_l:	cp	YL, timing_duty_l
		cpc	YH, timing_duty_h
		brcs	set_new_duty10
		movw	YL, timing_duty_l	; Limit duty to timing_duty
set_new_duty10:	cp	YL, sys_control_l
		cpc	YH, sys_control_h
		brcs	set_new_duty11
		movw	YL, sys_control_l	; Limit duty to sys_control
set_new_duty11:
		ldi2	temp1, temp2, MAX_POWER
		sub	temp1, YL		; Calculate OFF duty
		sbc	temp2, YH
		breq	set_new_duty_full
		adiw	YL, 0
		breq	set_new_duty_zero
		; Not off and not full power
		cbr	flags1, (1<<FULL_POWER)
		sbr	flags1, (1<<POWER_ON)
set_new_duty_set:
		; When off duty is short, skip complementary PWM; otherwise,
		; compensate the off_duty time to account for the overhead.
		ldi	temp4, pwm_on		; Short off period: skip complementary PWM
		cpse	temp2, ZH
		ldi	temp4, pwm_on_high	; Off period >= 0x100
set_new_duty21:
		com	YL			; Save one's complement of both
		com	temp1			; low bytes for up-counting TCNT2
		movw	duty_l, YL		; Atomic set new ON duty for PWM interrupt
		cli				; Critical section (off_duty & flags together)
		movw	off_duty_l, temp1	; Set new OFF duty for PWM interrupt
		sts	pwm_on_ptr, temp4	; Set Next PWM ON interrupt vector
		sei
		ret
set_new_duty_full:
		; Full power
		sbr	flags1, (1<<FULL_POWER) | (1<<POWER_ON)
		rjmp	set_new_duty_set
set_new_duty_zero:
		; Power off
		cbr	flags1, (1<<FULL_POWER) | (1<<POWER_ON)
		ldi	temp4, pwm_off		; Skip the on phase entirely
		rjmp	set_new_duty21
;-----bko-----------------------------------------------------------------
; Multiply the 24-bit timing in temp1:temp2:temp3 by temp4 and add the top
; 24-bits to YL:YH:temp7.
update_timing_add_degrees:
		mul	temp1, temp4
		add	YL, temp6		; Discard byte 1 already
		adc	YH, ZH
		adc	temp7, ZH
		mul	temp2, temp4
		add	YL, temp5
		adc	YH, temp6
		adc	temp7, ZH
		mul	temp3, temp4
		add	YH, temp5
		adc	temp7, temp6
		ret
load_timing:
		lds	temp1, timing_l
		lds	temp2, timing_h
		lds	temp3, timing_x
		lds	YL, com_time_l
		lds	YH, com_time_h
		lds	temp7, com_time_x
		ret
set_timing_degrees:
		rcall	load_timing
		rcall	update_timing_add_degrees
	; Fall through to set_ocr1a_abs
;-----bko-----------------------------------------------------------------
; Set OCT1_PENDING until the absolute time specified by YL:YH:temp7 passes.
; Returns current TCNT1(L:H:X) value in temp1:temp2:temp3.
;
; tcnt1x may not be updated until many instructions later, even with
; interrupts enabled, because the AVR always executes one non-interrupt
; instruction between interrupts, and several other higher-priority
; interrupts may (have) come up. So, we must save tcnt1x and TIFR with
; interrupts disabled, then do a correction.
set_ocr1a_abs:
		in	temp4, TIMSK
		mov	temp5, temp4
		cbr	temp4, (1<<TOIE1) | (1<<OCIE1A)
		out	TIMSK, temp4		; Disable TOIE1 and OCIE1A temporarily
		ldi	temp4, (1<<OCF1A)
		cli
		out	OCR1AH, YH
		out	OCR1AL, YL
		out	TIFR, temp4		; Clear any pending OCF1A interrupt
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		sei
		sbr	flags0, (1<<OCT1_PENDING)
		lds	temp3, tcnt1x
		in	temp4, TIFR
		cpi	temp2, 0x80		; tcnt1x is right when TCNT1h[7] set;
		sbrc	temp4, TOV1		; otherwise, if TOV1 is/was pending,
		adc	temp3, ZH		; increment our copy of tcnt1x.
		sub	YL, temp1		; Check that time might have already
		sbc	YH, temp2		; passed -- if so, clear pending flag.
		sbc	temp7, temp3
		sts	ocr1ax, temp7
		brpl	set_ocr1a_abs1		; Skip set if time has passed
		cbr	flags0, (1<<OCT1_PENDING)
set_ocr1a_abs1:	out	TIMSK, temp5		; Enable TOIE1 and OCIE1A again
		ret
;-----bko-----------------------------------------------------------------
; Set OCT1_PENDING until the relative time specified by YL:YH:temp7 passes.
set_ocr1a_rel:	adiw	YL, 7			; Compensate for timer increment during in-add-out
		ldi	temp4, (1<<OCF1A)
		cli
		in	temp1, TCNT1L
		in	temp2, TCNT1H
		add	YL, temp1
		adc	YH, temp2
		out	OCR1AH, YH
		out	OCR1AL, YL
		out	TIFR, temp4		; Clear any pending OCF1A interrupt (7 cycles from TCNT1 read)
		sts	ocr1ax, temp7
		sbr	flags0, (1<<OCT1_PENDING)
		sei
		ret
;-----bko-----------------------------------------------------------------
wait_OCT1_tot:	sbrc	flags1, EVAL_RC
		rcall	evaluate_rc
		sbrc	flags0, OCT1_PENDING
		rjmp	wait_OCT1_tot		; Wait for commutation time
		ret
;-----bko-----------------------------------------------------------------
switch_power_off:
		out	TCCR2, ZH		; Disable PWM
		ldi	temp1, (1<<TOV2)
		out	TIFR, temp1		; Clear pending PWM interrupts
		ldi	ZL, low(pwm_wdr)	; Stop PWM switching
		all_pFETs_off temp1
		all_nFETs_off temp1
		ret
;-----bko-----------------------------------------------------------------
waitForValidAndZeroDutyToArm:
		cbr	flags1, (1<<EVAL_RC)
waitEVAL_RC:	wdr
		GRN_on
		sbrs	flags1, EVAL_RC
		rjmp	waitEVAL_RC
		rcall	evaluate_rc
		lds	YL, rc_duty_l
		lds	YH, rc_duty_h
		adiw	YL, 0			; Test for zero
		brne	waitForValidAndZeroDutyToArm
		rcall	switch_power_off	; Disables PWM timer, turns off all FETs
		cbr	flags0, (1<<SET_DUTY)	; Do not yet set duty on input
		ldi2	YL, YH, PWR_MIN_START	; Start with limited power to reduce the chance that we
		movw	sys_control_l, YL	; align to a timing harmonic

		sbr	flags0, (1<<SET_DUTY)
		; Set STARTUP flag and call update_timing which will set
		; last_tcnt1 and set the duty (limited by STARTUP) and
		; set POWER_ON.
		rcall	wait_timeout_init
		sts	start_delay, ZH
		ldi	temp1, 6		; Do not enable FETs during first cycle to
		sts	powerskip, temp1	; see if motor is running, and align to it.
		outi	TCCR2, T2CLK		; Enable PWM (ZL has been set to pwm_wdr)

runLoop:
		rcall	wait_for_high
		com1com2
		rcall	wait_for_low
		com2com3
		rcall	wait_for_high
		com3com4
		rcall	wait_for_low
		com4com5
		rcall	wait_for_high
		com5com6
		rcall	wait_for_low
		com6com1

		lds	temp1, goodies
		movw	YL, sys_control_l
		cpi	temp1, ENOUGH_GOODIES
		brcc	startComplete 		; ELSE {goodies++; power += small step}
		inc	temp1
		sts	goodies, temp1
		; Build up sys_control to PWR_MAX_START in steps.
		adiw	YL, (POWER_RANGE + 47) / 48
		ldi2	temp1, temp2, PWR_MAX_START
startComplete:
		cbr	flags1, (1<<STARTUP)
		RED_off
		; Build up sys_control to MAX_POWER in steps.
		; once running, sys_control
		; will stay at MAX_POWER unless timing is lost.
		adiw	YL, (POWER_RANGE + 31) / 32
		ldi2	temp1, temp2, MAX_POWER
		cp	YL, temp1
		cpc	YH, temp2
		brcs	lessThanMAX
		movw	sys_control_l, temp1
		rjmp	runLoop
lessThanMAX:	movw	sys_control_l, YL
		rjmp	runLoop

;-----bko-----------------------------------------------------------------
wait_timeout:
		sbrc	flags1, STARTUP
		rjmp	wait_timeout_start
		cpi	XH, ZC_CHECK_FAST
		brcs	wait_timeout_run
		ldi	XH, ZC_CHECK_FAST - 1	; Limit back-tracking
		cp	XL, XH
		brcc	wait_timeout1
		mov	XL, XH			; Clip current distance from crossing
wait_timeout1:	rcall	load_timing
		add	YL, temp1
		adc	YH, temp2
		adc	temp7, temp3
		add	YL, temp1
		adc	YH, temp2
		adc	temp7, temp3
		rcall	set_ocr1a_abs		; Set zero-crossing timeout to 240 degrees
		rjmp	wait_for_edge2
wait_timeout_run:
		RED_on				; Turn on red LED
wait_timeout_start:
		sts	goodies, ZH		; Clear good commutation count
		lds	temp4, start_delay
		subi	temp4, -START_DELAY_INC	; Increase start (blanking) delay,
		sbrc	flags1, STARTUP		; unless we were running
		sts	start_delay, temp4
wait_timeout_init:
		sbr	flags1, (1<<STARTUP)	; Leave running mode
		rjmp	wait_commutation	; Update timing and duty.
;-----bko-----------------------------------------------------------------
; Here we wait for the zero-crossing on the undriven phase to synchronize
; with the motor timing. The voltage of the undriven phase should cross
; the average of all three phases at half of the way into the 60-degree
; commutation period.
;
; The voltage on the undriven phase is affected by noise from PWM (mutual
; inductance) and also the demagnetization from the previous commutation
; step. Demagnetization time is proportional to motor current, and in
; extreme cases, may take more than 30 degrees to complete. To avoid
; sensing erroneous early zero-crossings in this case and losing motor
; synchronization, we check that demagnetization has finished after the
; minimum blanking period. If we do not see it by the maximum blanking
; period (about 30 degrees since we commutated last), we turn off power
; and continue as if the ZC had occurred. PWM is enabled again after the
; next commutation step.
;
; Normally, we wait for the blanking window to pass, look for the
; comparator to swing as the sign of the zero crossing, wait for the
; timing delay, and then commutate.
;
; Simulations show that the demagnetization period shows up on the phase
; being monitored by the comparator with no PWM-induced noise. As such,
; we do not need any filtering. However, it may not show up immediately
; due to filtering capacitors, hence the initial blind minimum blanking
; period.
;
; Special case: powerskipping during start. The idea here is to learn the
; timing of a possibly-spinning motor while not driving it, which would
; induce demagnetization and PWM noise that we cannot ignore until we
; know the timing. We use twice the timeout that would otherwise bring
; ZC check count to 0xff. A motor spinning twice the speed or slower will
; fall through to regular startup with ZC check count at 0xff. This lets
; us start from braking, RC timeout, or power-up without misaligning.
;
wait_for_low:	cbr	flags1, (1<<ACO_EDGE_HIGH)
		rjmp	wait_for_edge
wait_for_high:	sbr	flags1, (1<<ACO_EDGE_HIGH)
wait_for_edge:	lds	temp1, powerskip	; Are we trying to track a maybe running motor?
		subi	temp1, 1
		brcs	wait_pwm_enable
		sts	powerskip, temp1 	; powerskip--
		sbrs	flags1, STARTUP
		rjmp	wait_for_edge0
		ldi	YL, byte1(0xff * 0x100)	; Timing is 120 degrees, so wait for
		ldi	YH, byte2(0xff * 0x100)	; what would be 0xff at 60 degrees
		mov	temp7, ZH
		rcall	set_ocr1a_rel
		ldi	XL, ZC_CHECK_MIN	; There shouldn't be much noise with no power
		rjmp	wait_for_edge1
wait_pwm_enable:
		cpi	ZL, low(pwm_wdr)
		brne	wait_pwm_running
		ldi	ZL, low(pwm_off)	; Re-enable PWM if disabled for powerskip or sync loss avoidance
		RED_off				; wait_timeout would have happened if motor not spinning during powerskip
wait_pwm_running:
		sbrc	flags1, STARTUP
		rjmp	wait_startup
		ldi	temp4, 13 * 256 / 120
		rcall	set_timing_degrees
		rcall	wait_OCT1_tot		; Wait for the minimum blanking period

		ldi	temp4, 42 * 256 / 120
		rcall	set_timing_degrees	; Set timeout for maximum blanking period
waitUntilDemagged:
		in	temp3, ACSR
		eor	temp3, flags1
		sbrc	temp3, ACO		; Check for opposite level (demagnetization)
		rjmp	waitUntilDemagged
wait_for_edge0:
		rcall	load_timing
		mov	XL, temp2		; Copy high and extended byte
		mov	XH, temp3		; to calculate the ZC check count
		lsr	XH			; Quarter to obtain timing / 1024
		ror	XL
		lsr	XH
		ror	XL
		cpi	XL, ZC_CHECK_MIN
		cpc	XH, ZH
		brcs	wait_for_edge_fast_min
		breq	wait_for_edge_fast
		cpi	XL, ZC_CHECK_MAX
		cpc	XH, ZH
		brcs	wait_for_edge_below_max
		ldi	XL, ZC_CHECK_MAX	; Limit to ZC_CHECK_MAX
wait_for_edge_below_max:
		cpi	XL, ZC_CHECK_FAST	; For faster timing, set normal ZC timeout
		brcs	wait_for_edge_fast
		ldi	temp4, 24 * 256 / 120	; With slower (longer) timing, set timer
		rcall	set_timing_degrees	; for limited backtracing
		rjmp	wait_for_edge1
wait_for_edge_fast_min:
		ldi	XL, ZC_CHECK_MIN
wait_for_edge_fast:
		add	YL, temp1
		adc	YH, temp2
		adc	temp7, temp3
		add	YL, temp1
		adc	YH, temp2
		adc	temp7, temp3
		rcall	set_ocr1a_abs		; Set zero-crossing timeout to 240 degrees

wait_for_edge1:	mov	XH, XL
wait_for_edge2:	sbrs	flags0, OCT1_PENDING
		rjmp	wait_timeout
		in	temp3, ACSR
		eor	temp3, flags1
		sbrc	temp3, ACO
		rjmp	wait_for_edge3
		cp	XL, XH			; Not yet crossed
		adc	XL, ZH			; Increment if not at zc_filter
		rjmp	wait_for_edge2
wait_for_edge3:	dec	XL			; Zero-cross has happened
		brne	wait_for_edge2		; Check again unless temp1 is zero

wait_commutation:
		rcall	update_timing
		sbrs	flags1, STARTUP
		rcall	wait_OCT1_tot
		lds	temp1, powerskip
		cpse	temp1, ZH
		cbr	flags1, (1<<POWER_ON)	; Disable power when powerskipping
		ret

; When starting, we have no idea what sort of motor may be connected,
; or how much time it will take for the previous commutation current
; to stop flowing. If the motor is not spinning and the sensing while
; powerskipping fails, we apply power and check for back-EMF after an
; increasing delay, building up further if the motor remains stalled.
; This will stretch out short commutations caused by the comparator
; sitting low or high when inputs are below the minimum offset.
wait_startup:
		ldi3	YL, YH, temp4, 0 	; Initial post-commutation wait during starting
		mov	temp7, temp4
		lds	temp1, goodies
		cpi	temp1, 2		; After some good commutations,
		brcc	wait_startup1		; skip additional delay injection
		lds	temp4, start_delay
		ldi3	temp1, temp2, temp3, START_DSTEP_US * CPU_MHZ * 0x100
		rcall	update_timing_add_degrees	; Add temp4 (start_delay) START_DSTEPs of wait
wait_startup1:	rcall	set_ocr1a_rel
		rcall	wait_OCT1_tot
		ldi3	YL, YH, temp4, TIMEOUT_START * CPU_MHZ
		mov	temp7, temp4
		rcall	set_ocr1a_rel
; Powered startup: Use a fixed (long) ZC check count until goodies reaches
; ENOUGH_GOODIES and the STARTUP flag is cleared.
		ldi	XL, 0xff * CPU_MHZ / 16
		rjmp	wait_for_edge1
