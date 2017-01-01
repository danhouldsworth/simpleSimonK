;-----bko-----------------------------------------------------------------
evaluate_rc:
		cbr	flags1, (1<<EVAL_RC)
		sts	brake_want, ZH
		movw	temp1, rx_l		; Atomic copy of rc pulse length
 		cpi	temp1, byte1(MIN_RC_PULS * CPU_MHZ) ; (temp1:2 == MIN_RC_PULS) ?
 		ldi	temp3, byte2(MIN_RC_PULS * CPU_MHZ)
 		cpc	temp2, temp3
		brcc	puls_long_enough
		ret
puls_long_enough:
		lds	YL, puls_low_l 		;
		lds	YH, puls_low_h		;
		sub	temp1, YL		; Offset input to neutral
		sbc	temp2, YH
		brcc	puls_plus
		; Fall through to stop/zero in no reverse case
puls_zero_brake:
		ldi	YL, 1
		sts	brake_want, YL		; Set desired brake to 1 (neutral brake)
puls_zero:	clr	YL
		clr	YH
		rjmp	rc_duty_set
puls_plus:
		lds	temp3, fwd_scale_l	; Load forward scaling factor
		lds	temp4, fwd_scale_h
	; The following is used by all input modes
rc_do_scale:	ldi2	YL, YH, MIN_DUTY	; Offset result so that 0 is MIN_DUTY
		;rcall	mul_y_12x34		; Scaled result is now in Y

						; Multiply temp1:temp2 by temp3:temp4 and add high 16 bits of result to Y.
						; Clobbers temp5, temp6, and leaves the lower byte in temp7.
mul_y_12x34:
		mul	temp1, temp3		; Scale raw pulse length to POWER_RANGE: 16x16->32 (bottom 16 discarded)
		mov	temp7, temp6		; Save byte 2 of result, discard byte 1 already
		mul	temp2, temp3
		add	temp7, temp5
		adc	YL, temp6
		adc	YH, ZH
		mul	temp1, temp4
		add	temp7, temp5
		adc	YL, temp6
		adc	YH, ZH
		mul	temp2, temp4
		add	YL, temp5
		adc	YH, temp6		; Product is now in Y, flags set
		cpi	YL, 	byte1(MAX_POWER); (YL:YH == MAX_POWER)?
 		ldi	temp1, 	byte2(MAX_POWER)
 		cpc	YH, temp1
		brcs	rc_duty_set
		ldi2	YL, YH, MAX_POWER
rc_duty_set:	sts	rc_duty_l, YL
		sts	rc_duty_h, YH
		sbrs	flags0, SET_DUTY
		rjmp	rc_no_set_duty
		ldi	temp1, RCP_TOT
		mov	rc_timeout, temp1	; Short rc_timeout when driving
		rjmp	set_new_duty_l		; Skip reload into YL:YH
rc_no_set_duty:	ldi	temp1, 12		; More than 10 needed to arm
		cp	rc_timeout, temp1
		adc	rc_timeout, ZH
		ret
