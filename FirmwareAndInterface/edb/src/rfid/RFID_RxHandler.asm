; Adapted from wisp-base project (github.com/wisp/wisp5)
; RFID/RX_ISR.asm file
; Graham Harvey
; 13 April 2015

	.cdecls	C, LIST, "rfid.h"
	.def	RFID_RxHandler
	.global	RFID_RxHandler


RFID_RxHandler:
	;**************************
	; Too early (delim < 6us)
	;**************************
	; MCLK is 21.921792 MHz (see UCS_setMainFreq)
	; MCLK period is 0.045617 us
	; For each BIT.B 3 cycles
	; For each JNZ or JZ not taken 2 cycles
	; For JNZ or JZ taken 3 or 4 cycles

	; 1.87 us
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=1.87..1.92us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=2.10..2.14us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=2.33..2.37us
	JNZ		badDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[3]t=2.55..2.60us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=2.78..2.83us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=3.01..3.06us
	JNZ		badDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[3]t=3.24..3.28us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=3.47..3.51us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=3.69..3.74us
	JNZ		badDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[3]t=3.92..3.97us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=4.15..4.20us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=4.38..4.42us
	JNZ		badDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[3]t=4.61..4.65us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=4.84..4.88us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=5.06..5.11us
	JNZ		badDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[3]t=5.29..5.34us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=5.52..5.57us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=5.74..5.79us
	JNZ		badDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[3]t=5.98..6.02us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=6.20..6.25us
	JNZ		badDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[3]t=6.43..6.48us
	JNZ		badDelim				;[2]

	;*****************************************************
	; Just right (6.66 < delim < 14.00us @ ~21.921792 MHz
	;*****************************************************
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=6.66..6.71us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=6.89..6.93us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=7.11..7.16us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=7.34..7.39us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=7.57..7.62us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=7.80..7.85us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=8.03..8.07us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=8.26..8.30us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=8.48..8.53us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=8.71..8.76us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=8.94..8.99us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=9.17..9.21us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=9.40..9.44us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=9.62..9.67us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=9.85..9.90us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=10.08..10.13us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=10.31..10.35us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=10.54..10.58us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=10.77..10.81us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=10.99..11.04us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=11.22..11.27us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=11.45..11.50us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=11.68..11.72us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=11.91..11.95us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=12.13..12.18us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=12.36..12.41us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=12.59..12.64us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=12.82..12.86us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=13.05..13.09us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=13.27..13.32us
	JNZ		goodDelim				;[2]

	BIT.B	#PIN_RX,	&PRXIN		;[4]t=13.50..13.55us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=13.73..13.78us
	JNZ		goodDelim				;[2]
	BIT.B	#PIN_RX,	&PRXIN		;[4]t=13.96..14.00us
	JNZ		goodDelim				;[2]

	;******************************
	; Too long
	;******************************

badDelim:
;	BIT.B	R15,	R14				;[] I have no idea why this is here -gh
	BIC		#CCIFG,	&TA0CCTL1		;[] clear the interrupt flag for Timer0A1 capture (safety)
	CLR		&TA0R					;[] reset TAR value
	CLR		&(rfid.edge_capture_prev_ccr) ;[] clear previous value of CCR capture
;	CLR.B	&PRXIFG					;[] clear port 1 flag
	BIC.B	#PIN_RX,	&PRXIFG		;[] clear Rx interrupt flag

; record this failure event for the host
;	PUSH	R12						; save since will be clobbered by append_event
;	PUSH	R11						; save since will be clobbered by append_event
;	PUSH	R15						; save since will be clobbered by append_event
;	MOV		#rf_event_type_t.RF_EVENT_ERR_BAD_DELIM, R12 ;[] arg for the following function call
;	CALLA	#append_event			; add event to rf events buffer
;	POP		R15						; restore reg clobbered by append_event
;	POP		R11						; restore reg clobbered by append_event
;	POP		R12						; restore reg clobbered by append_event

	RETA

;************************************
; Continue Rx state machine
;************************************
goodDelim:
	BIS.B	#PIN_RX,	&PRXSEL		;[4] Enable Timer0A1
;	CLR.B	&PRXIE					;[4] disable the port 1 interrupt
	BIC.B	#PIN_RX,	&PRXIE		;[4] disable the Rx interrupt
	PUSHM.A	#1,			R15
	MOV		#FORCE_SKIP_INTO_RTCAL,	R15

delimDelay:
	DEC		R15
	JNZ		delimDelay
	POPM.A	#1,			R15

startup_T0A1_ISR:
	; Timer0A1 interrupt should not fire at falling edge of delay cycle that
	; happens after the good delimiter
	BIC		#CCIFG,	&TA0CCTL1		;[] clear the interrupt flag for Timer0A1 capture
	BIS.W	#(CM_2 + CCIE),	&TA0CCTL1 ; [] turn on timer by setting capture mode (falling edge) + interrupt

	CLR		&TA0R					;[] reset TAR value
	CLR		&(rfid.edge_capture_prev_ccr) ;[] reset previous value of CCR capture
;	CLR		&(rfid.edge_capture_prev_ccr)
;	CLR.B	&PRXIFG					;[] clear port 1 flag
	BIC.B	#PIN_RX,	&PRXIFG		;[] clear Rx interrupt flag
;	ADD		#36,	&TA0R			;[] may need to be adjusted
	RETA

	.end
