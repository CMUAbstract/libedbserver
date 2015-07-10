; Adapted from wisp-base project (github.com/wisp/wisp5)
; RFID/Timer0A0_ISR.asm file
; Graham Harvey
; 13 April 2015

;/***********************************************************************************************************************************/
;/**@file		Timer0A1_ISR.asm
;* 	@brief		Receive chain decoding routines.
;* 	@details
;*
;* 	@author		Justin Reina, UW Sensor Systems Lab
;* 	@created
;* 	@last rev	
;*
;* 	@notes
;*
;*	@todo
;*/
;/***********************************************************************************************************************************/

	.cdecls C, LIST, "rfid.h"
	.sect ".text:_isr"
	.define "0", SR_SP_OFF
	.retain
	.retainrefs

;Register Defs
R_dest			.set  R4
R_bits			.set  R5
R_bitCt			.set  R6
R_newCt			.set  R7
R_pivot			.set  R8
R_scratch2		.set  R9
R_scratch0  	.set  R15


;*************************************************************************************************************************************
;	Timer0A1 ISR: On Entry Latch,Reset T0A0R. Then Parse and Deploy Mode
;*************************************************************************************************************************************
Timer0A1_ISR:
	; preserve registers
	PUSHM.A	#12,	R15

	; restore saved values
	MOV.W	&R_bits_saved,		R_bits
	MOV.W	&R_bitCt_saved,		R_bitCt
	MOV.W	&R_dest_saved,		R_dest
	MOV.W	&R_scratch0_saved, 	R_scratch0
	MOV.W	&R_scratch2_saved, 	R_scratch2
	MOV.W	&R_pivot_saved, 	R_pivot

	MOV		&TA0CCR1,	R_newCt ;[3] latch TA0CCR1 value into the new count reg
	SUB		&(rfid.edge_capture_prev_ccr), R_newCt; Compute delta by subtracting CCR value captured on previous edge
	MOV		&TA0CCR1,	&(rfid.edge_capture_prev_ccr); Update previous CCR value with current value (preparing for next edge)

	BIC		#CCIFG,		&TA0CCTL1	;[4] clear the interrupt flag

;Check Which Mode We Are In
	CMP		#(2),		R_bits		;[2]
	JGE		ModeD_process			;[2] deploy mode D (does a signed comparison)
	CMP		#(1),		R_bits		;[2]
	JEQ		ModeC_process			;[2] deploy mode C
	CMP		#(0),		R_bits		;[2]
	JEQ 	ModeB_process			;[2] deploy mode B
									;[0] else deploy mode A

;*************************************************************************************************************************************
;	MODE A: RTCal
;*************************************************************************************************************************************
ModeA_process:
; DEBUG!!
	CMP		#RTCAL_MIN,  R_newCt 	;[2]error catch: check if valid RTCal (i.e. if TA0CCR1<2TARI)
	JL		failed_RTCal		 	;[2] break if doesn't work.
    CMP		#RTCAL_MAX,	 R_newCt 	;[2]
    JGE		failed_RTCal		 	;[2] break if doesn't work.

	;ok valid RTCal, now process to get pivot(R8)
    MOV    	R_newCt, 	 R_scratch2 ;[1] Load R7 into a scratch reg(R9) so we can use this RTCal value in Mode 2 Processing next
    ADD		#RTCAL_OFFS, R_newCt 	;[2] Compensate for lost cycles in this measurement method to get to actual RTCal val.
    RRA   	R_newCt             	;[1] Divide R7 by 2
    MOV    	#(-1),		 R_pivot    ;[1] preload pivot value with MAX (i.e. 0xFFFFh)
    SUB    	R_newCt, 	 R_pivot  	;[1] pivotReg = pivotReg-(currCount/2) <-- This sets up the pivot reg for calc discussed.
    INC    	R_bits               	;[1] (r5=2) use r5 as a flag for the next entry (entry into mode 2)
    JMP		restore_return
;    RETI                        	;[5] return from interrupt


;*************************************************************************************************************************************
;	MODE B: TRCal/bit0
;*************************************************************************************************************************************
ModeB_process:
	CMP    	R_scratch2, R_newCt		;[1] is currCount>RTCal? if so it's TRCal, else data
    JGE    	ModeB_TRCal        		;[2] if currCount>RTCal then jump to process TRCal


	;else it's databit0. store it!
ModeB_dataBit:
    ADD    	R_pivot, 	R_newCt    	;[1] do pivotTest (currCount = currCount+pivotReg. if Carry, its data1) <-note: thus dataBit is stored in carry.
    ADDC.B 	@R_dest+,	-1(R_dest) 	;[5] shift cmd[curr] by one (i.e. add it to itself), then store dataBit(carryFlag), into cmd[currCmdByte], then increment currCmdByte (all as one asm:))

    INC    	R_bits                  ;[1] update R5(bits) cause we got a databit
    INC    	R_bitCt                 ;[1] mark that we've stored a bit into r6(currCmdBits)
    DEC		R_dest
    JMP		restore_return
;    RETI                        	;[5] return from interrupt


    ;Check if Valid TRCal (i.e. ~50.2us).
ModeB_TRCal:
    CMP		#TRCAL_MIN, R_newCt		;[2]
    JL		failed_TRCal 			;[2] reset RX State Machine if TRCal is too short!! we won't check if too long, because 320kHz uses the max TRCal length.
    CMP		#TRCAL_MAX,  R_newCt 	;[2] error catch: check if valid TRCal (i.e. if TA0CCR1>2TARI)
	JGE		failed_TRCal			;[2]

    CLR    	R_bitCt					;[1] start the counting of R6 fresh (cause databits will come immeadiately following)
    JMP		restore_return
;    RETI                        	;[5] return from interrupt


;*************************************************************************************************************************************
;	MODE C: bit1
;*************************************************************************************************************************************
ModeC_process:
    ADD    	R_pivot, 	R_newCt    	;[1] do pivotTest (currCount = currCount+pivotReg. if Carry, its data1) <-note: thus dataBit is stored in carry.
    ADDC.B 	@R_dest+,	-1(R_dest) 	;[5] shift cmd[curr] by one (i.e. add it to itself), then store dataBit(carryFlag), into cmd[currCmdByte], then increment currCmdByte (all as one asm:))
	;Check if Query Rep
	MOV.B	(cmd),		R_scratch0	;[2] pull in the first two dataBits
	AND.B	#0x03,		R_scratch0	;[2] mask out everything but b1b0
    CMP.B	#0x00,		R_scratch0	;[1] check if we got a queryRep
	JEQ		ModeC_queryRep			;[2]

    INC    	R_bits               	;[1] update R5(bits) cause we got a databit
    INC    	R_bitCt                 ;[1] mark that we've stored a bit into r6(currCmdBits)
    DEC		R_dest
    JMP		restore_return
;	RETI                        	;[5] return from interrupt


ModeC_queryRep:
	MOV.W	#(0), 		&TA0CTL				;[4] turn off the timer
	MOV.B	#CMD_PARSE_AS_QUERY_REP, &cmd	;[4] set the cmd[0] to a known value for queryRep to parse.
	CALLA	#decodeCmd
	JMP		restore_return
;    RETI                        			;[5] return from interrupt


;*************************************************************************************************************************************
;	MODE D: Plain ol' Data Bit (bits>=2)
;*************************************************************************************************************************************
ModeD_process:
    ADD    	R_pivot, 	R_newCt    	;[1] do pivotTest (currCount = currCount+pivotReg. if Carry, its data1) <-note: thus dataBit is stored in carry.
    ADDC.B 	@R_dest+,	-1(R_dest) 	;[5] shift cmd[curr] by one (i.e. add it to itself), then store dataBit(carryFlag), into cmd[currCmdByte], then increment currCmdByte (all as one asm:))

    INC    	R_bits                  ;[1] update R5(bits) cause we got a databit
    INC    	R_bitCt                 ;[1] mark that we've stored a bit into r6(currCmdBits)

	;Check if that finished off a whole byte in cmd[]
	CMP.W		#(8),		R_bitCt	;[1] Check if we have gotten 8 bits
	JGE		ModeD_setupNewByte		;[2]
	DEC		R_dest
	JMP		restore_return
;    RETI                        	;[5] return from interrupt


ModeD_setupNewByte:
	CLR		R_bitCt					;[1] Clear the Current Bit Count to reset for the next byte
	CALLA	#decodeCmd
	JMP		restore_return
;    RETI                        	;[5] return from interrupt


;*************************************************************************************************************************************
;	MODE FAILS: Reset RX State Machine																								 *
;*************************************************************************************************************************************
failed_RTCal:
;	PUSH R12	; save since will be clobbered by append_event
;	MOV		#rf_event_type_t.RF_EVENT_ERR_RT_CAL, R12 ;[] arg for append_event
;	JMP		failed_Cal

failed_TRCal:
;	PUSH R12	; save since will be clobbered by append_event
;	MOV		#rf_event_type_t.RF_EVENT_ERR_TR_CAL, R12 ;[] arg for append_event
;	JMP		failed_Cal

failed_Cal:
;	CLR 	R_bits					;[1] reset R5 for rentry into RX State Machine
	MOV		#RESET_BITS_VAL,	R_bits
	CLR		R_bitCt					;[]
	BIS.B	#PIN_RX,	&PRXIES		;[4] wait again for #1 to fire on falling edge
;	MOV.B	#PIN_RX, &PRXIE			; Enable the interrupt
	BIS.B	#PIN_RX,	&PRXIE		;[] enable Rx interrupt
;	CLR.B	&PRXIFG					;[] clr any pending flasgs (safety)
	BIC.B	#PIN_RX,	&PRXIFG		;[] clear pending Rx flags (safety)
	BIC.B	#PIN_RX,	&PRXSEL		;[] disable timer
	BIC.W	#(CM0+CM1+CCIE), &TA0CCTL1	;[5] Turn off timer by seting capture mode to none and dis int

;	PUSH	R11						; save since will be clobbered by append_event
;	PUSH	R15						; save since will be clobbered by append_event
;	CALLA	#append_event			;[] record this failure event for the host
;	POP		R15						; restore reg clobbered by append_event
;	POP		R11						; restore reg clobbered by append_event
;	POP		R12						; restore reg clobbered by append_event

 ;   RETI                        	;[5] return from interrupt

restore_return:
	; save registers for next call
	MOV.W	R_dest,		&R_dest_saved		;[]
	MOV.W	R_pivot, 	&R_pivot_saved		;[]
	MOV.W	R_scratch2,	&R_scratch2_saved	;[]
	MOV.W	R_scratch0, &R_scratch0_saved	;[]
	MOV.W	R_bitCt,	&R_bitCt_saved		;[]
	MOV.W	R_bits,		&R_bits_saved		;[]
	MOV.W	R_newCt,	&R_newCt_saved		;[]

	; restore registers
	POPM.A	#12,	R15
	RETI

;*************************************************************************************************************************************
; DEFINE THE INTERRUPT VECTOR ASSIGNMENT																							 *
;*************************************************************************************************************************************
	.sect ".int52"					; Timer0_A1 Vector
	.short Timer0A1_ISR				; This sect/short pair sets int52 = Timer0A1_ISR addr.
	.end
