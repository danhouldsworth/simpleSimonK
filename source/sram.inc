
;**** **** **** **** ****
; RAM Definitions
.dseg				; DATA segment
.org SRAM_START

goodies:	.byte	1	; Number of rounds without timeout
powerskip:	.byte	1	; Skip power through this number of steps
ocr1ax:		.byte	1	; 3rd byte of OCR1A
tcnt1x:		.byte	1	; 3rd byte of TCNT1
pwm_on_ptr:	.byte	1	; Next PWM ON vector
last_tcnt1_l:	.byte	1	; last timer1 value
last_tcnt1_h:	.byte	1
last_tcnt1_x:	.byte	1
l2_tcnt1_l:	.byte	1	; last last timer1 value
l2_tcnt1_h:	.byte	1
l2_tcnt1_x:	.byte	1
timing_l:	.byte	1	; interval of 2 commutations
timing_h:	.byte	1
timing_x:	.byte	1
com_time_l:	.byte	1	; time of last commutation
com_time_h:	.byte	1
com_time_x:	.byte	1
start_delay:	.byte	1	; delay count after starting commutations before checking back-EMF
start_modulate:	.byte	1	; Start modulation counter (to reduce heating from PWR_MAX_START if stuck)
start_fail:	.byte   1	; Number of start_modulate loops for eventual failure and disarm
rc_duty_l:	.byte	1	; desired duty cycle
rc_duty_h:	.byte	1
;**** **** **** **** ****
; The following entries are RAM settings
puls_high_l:	.byte	1	;
puls_high_h:	.byte	1	;
puls_low_l:	.byte	1	;
puls_low_h:	.byte	1	;
fwd_scale_l:	.byte	1	; 16.16 multipliers to scale input RC pulse to POWER_RANGE
fwd_scale_h:	.byte	1
RAM_end:	.byte	1
