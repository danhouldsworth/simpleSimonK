;--------------------------------------------------------------------------
; simpleSimonK - Super simple ESC firmware
;--------------------------------------------------------------------------

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

;-- Declarations ----------------------------------------------------------
.include "header.inc"
;-- SRAM definitions / DSEG -----------------------------------------------
.include "sram.inc"
;-- Interupt table & service routines -------------------------------------
.include "interrupts.inc"
;-- MAIN ROUTINES ---------------------------------------------------------
.include "commutations.inc"
;--------------------------------------------------------------------------

