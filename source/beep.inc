;-----bko-----------------------------------------------------------------
; beeper: timer0 is set to 1µs/count
beep_f1:	ldi	temp2, 80
		ldi	temp4, 200
		RED_on
beep_f1_on:	BpFET_on
		AnFET_on
		rcall	beep
		brne	beep_f1_on
		RED_off
		ret

beep_f2:	ldi	temp2, 100
		ldi	temp4, 180
		GRN_on
beep_f2_on:	CpFET_on
		BnFET_on
		rcall	beep
		brne	beep_f2_on
		GRN_off
		ret

beep_f3:	ldi	temp2, 120
		ldi	temp4, 160
beep_f3_on:	ApFET_on
		CnFET_on
		rcall	beep
		brne	beep_f3_on
		ret

beep_f4:	ldi	temp2, 140
beep_f4_freq:	ldi	temp4, 140
beep_f4_fets:	RED_on
		GRN_on
beep_f4_on:	CpFET_on
		AnFET_on
		rcall	beep
		brne	beep_f4_on
		GRN_off
		RED_off
		ret
;-----bko-----------------------------------------------------------------
; Interrupts no longer need to be disabled to beep, but the PWM interrupt
; must be muted first
beep:		out	TCNT0, ZH
beep1:		in	temp1, TCNT0
		cpi	temp1, 2*CPU_MHZ	; 32µs on
		brlo	beep1
		all_nFETs_off temp3
		all_pFETs_off temp3
		ldi	temp3, CPU_MHZ
beep2:		out	TCNT0, ZH
		wdr
beep3:		in	temp1, TCNT0
		cp	temp1, temp4
		brlo	beep3
		dec	temp3
		brne	beep2
		dec	temp2
		ret

wait240ms:	rcall	wait120ms
wait120ms:	rcall	wait60ms
wait60ms:	rcall	wait30ms
wait30ms:	ldi	temp2, 15
wait1:		ldi	temp3, CPU_MHZ
wait2:		out	TCNT0, ZH
		ldi	temp1, (1<<TOV0)	; Clear TOV0 by setting it
		out	TIFR, temp1
		wdr
wait3:		in	temp1, TIFR
		sbrs	temp1, TOV0
		rjmp	wait3
		dec	temp3
		brne	wait2
		dec	temp2
		brne	wait1
wait_ret:	ret