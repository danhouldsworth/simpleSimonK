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
;-- Beep and Mult routines ------------------------------------------------
.include "beep.inc"
;--------------------------------------------------------------------------

