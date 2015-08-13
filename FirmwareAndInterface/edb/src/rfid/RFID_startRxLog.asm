; Adapted from wisp-base project (github.com/wisp/wisp5)
; RFID/WISP_doRFID.asm file
; Graham Harvey
; 13 April 2015

;/***********************************************************************************************************************************/
;/**@file		RFID_startRxLog.asm
;* 	@brief		This is the main RFID loop that executes an RFID transaction
;* 	@details
;*
;* 	@author		Justin Reina, UW Sensor Systems Lab
;* 	@created	6.14.11
;* 	@last rev
;*
;* 	@notes
;*
;*	@todo		Maybe doRFID() should 1) take arguments which set RFID response mode, and 2) return last cmd to which it responded
;*	@todo		doRFID() should sleep_till_full_power() after a transaction when it's in a mode where it doesn't always return...
;*/
;/***********************************************************************************************************************************/

;/INCLUDES----------------------------------------------------------------------------------------------------------------------------
    .cdecls C, LIST, "rfid.h", "ucs.h"
	.def  RFID_startRxLog
	.global RFID_startRxLog

;/PRESERVED REGISTERS-----------------------------------------------------------------------------------------------------------------
R_bitCt			.set  R6
R_bits			.set  R5
R_dest			.set  R4
R_scratch0  	.set  R15



RFID_startRxLog:
	;Initial Config of RFID Transaction
	MOV.B	#FALSE, &(rfid.abortFlag);[] Initialize abort flag

keepDoingRFID:
	DINT 									;[] safety
	NOP

;/************************************************************************************************************************************
;/										CONFIG, ENABLE The RX State Machine															 *
;/************************************************************************************************************************************
	;Configure Hardware (Port, Rx Comp)
	BIC.B	#PIN_RX, &PRXSEL			;[] make sure TimerA is disabled (safety)

	;TIMER A CONFIG (pre before RX ISR)
	CLR		&TA0CTL				;[] Disable TimerA before config (required)
	CLR		&TA0R				;[] Clear Ctr
	CLR		&(rfid.edge_capture_prev_ccr) ;[] Clear previous value of CCR capture
	MOV		#0xFFFF, &TA0CCR0	;[] Don't let Timer0A0 overflow

	;@us change:clear TA0
	CLR		&TA0CCTL1

	MOV		#(SCS + CAP + CCIS_0), &TA0CCTL1 ;[] Sync on Cap Src and Cap Mode on Rising Edge(Inverted).  Don't set all bits until in the RX ISR though.
	MOV		#(TASSEL__SMCLK+MC__CONTINOUS) , &TA0CTL 		;[] SMCLK and continuous mode.  (TASSEL1 + MC1)

	;Setup rfid_rxSM vars
	MOV		#RESET_BITS_VAL, R_bits	 ;[]MOD
	CLR		R_bitCt					 ;[]
	MOV		#(cmd), R_dest			 ;[]load the R_dest to the reg!

	MOV.W	R_bits,		&R_bits_saved
	MOV.W	R_bitCt,	&R_bitCt_saved
	MOV.W	R_dest,		&R_dest_saved

	; set up port interrupt for delimiter falling edge
	BIS.B	#PIN_RX,	&PRXIES		;[] Make falling edge for port interrupt to detect start of delimiter
	BIC.B	#PIN_RX,	&PRXIFG		;[] clear Rx interrupt flag
;	CLR.B	&PRXIFG					;[] Clear interrupt flag
	BIS.B	#PIN_RX,	&PRXIE		;[] enable Rx interrupt
;	MOV.B	#PIN_RX,	&PRXIE		;[] Enable Port1 interrupt


	NOP
	EINT
	NOP
	RETA


;/************************************************************************************************************************************
;/												DECODE COMMAND				                                                         *
;/	level1 = cmd[0] | 0xC0 (just examine the first two bits)																		 *
;/***********************************************************************************************************************************/
decodeCmd:
decodeCmd_lvl1:
	MOV.B 	(cmd),  R_scratch0	;[] bring in cmd[0] to parse
	AND.B	#0xC0,  R_scratch0	;[] just compare the first two bits

	CMP.B	#0xC0,	R_scratch0
	JEQ		decodeCmd_lvl2_11
	CMP.B	#0x80,	R_scratch0
	JEQ		decodeCmd_lvl2_10

	CMP.B	#0x40,	R_scratch0
	JEQ		callAckHandler

	CMP.B	#0x00, R_scratch0
	JEQ		callQRHandler

	JMP		endDoRFID

;either Req_RN/Read/Write.
decodeCmd_lvl2_11:
	MOV.B 	(cmd),  R_scratch0	;[] bring in cmd[0] to parse
	CMP.B	#0xC0,	R_scratch0	;[] is it NAK?
	JEQ		callNakHandler		;[]
	CMP.B	#0xC1,	R_scratch0	;[] is it reqRN?
	JEQ		callReqRNHandler	;[]
	CMP.B	#0xC2,	R_scratch0	;[] is it read?
	JEQ		callReadHandler		;[]
	CMP.B	#0xC3,	R_scratch0	;[] is it write?
	JEQ		callWriteHandler	;[]
	CMP.B	#0xC4,	R_scratch0	;[] is it kill?
	JEQ		callKillHandler		;[]
	CMP.B	#0xC5,	R_scratch0	;[] is it lock?
	JEQ		callLockHandler		;[]
	CMP.B	#0xC6,	R_scratch0	;[] is it access?
	JEQ		callAccessHandler	;[]
	CMP.B	#0xC7,	R_scratch0	;[] is it BlockWrite?
	JEQ		callBlockWriteHandler;[]
	CMP.B	#0xC8,	R_scratch0	;[] is it BlockErase?
	JEQ		callBlockEraseHandler;[]
	CMP.B	#0xC9,	R_scratch0	;[] is it BlockPermalock?
	JEQ		callBlockPermalockHandler;[]
	CMP.B	#0xD2,	R_scratch0	;[] is it ReadBuffer?
	JEQ		callReadBufferHandler;[]
	CMP.B	#0xD3,	R_scratch0	;[] is it FileOpen?
	JEQ		callFileOpenHandler;[]
	CMP.B	#0xD4,	R_scratch0	;[] is it challenge?
	JEQ		callChallengeHandler;[]
	CMP.B	#0xD5,	R_scratch0	;[] is it authenticate?
	JEQ		callAuthenticateHandler;[]
	CMP.B	#0xD6,	R_scratch0	;[] is it SecureComm?
	JEQ		callSecureCommHandler;[]
	CMP.B	#0xD7,	R_scratch0	;[] is it AuthComm?
	JEQ		callAuthCommHandler	;[]
	JMP		endDoRFID 			;[] come back and handle after query is working.


; either Select/QA/Query
; level2 = cmd[0] | 0x30
decodeCmd_lvl2_10:

	MOV.B 	(cmd), 	R_scratch0	;[] bring in cmd[0] to parse
	AND.B	#0x30,  R_scratch0	;[] just compare the second two bits

	CMP.B	#0x20,	R_scratch0	;[] is it select?
	JEQ		callSelectHandler	;[]

	CMP.B	#0x10,	R_scratch0	;[] is it queryAdjust?
	JEQ		callQAHandler		;[]
	CMP.B	#0x00,	R_scratch0	;[] is it query?
	JEQ		callQueryHandler	;[]
	JMP		endDoRFID			;[] come back and handle after query is working.


;/************************************************************************************************************************************
;/								CALL APPROPRIATE COMMAND HANDLER						                                     		 *
;/************************************************************************************************************************************/

callSelectHandler:
	CALLA	#handleSelect
	JMP		endDoRFID

callQueryHandler:
	CALLA	#handleQuery
	JMP		endDoRFID

callQRHandler:
	CALLA	#handleQR
	JMP		endDoRFID

callQAHandler:
	CALLA	#handleQA
	JMP		endDoRFID

callAckHandler:
	CALLA	#handleAck
	JMP		endDoRFID

callNakHandler:
	CALLA	#handleNak
	JMP		endDoRFID

callReqRNHandler:
	CALLA	#handleReqRN
	JMP		endDoRFID

callReadHandler:
	CALLA	#handleRead
	JMP		endDoRFID

callWriteHandler:
	CALLA	#handleWrite
	JMP		endDoRFID

callKillHandler:
	CALLA	#handleKill
	JMP		endDoRFID

callLockHandler:
	CALLA	#handleLock
	JMP		endDoRFID

callAccessHandler:
	CALLA	#handleAccess
	JMP		endDoRFID

callBlockWriteHandler:
	CALLA	#handleBlockWrite
	JMP		endDoRFID

callBlockEraseHandler:
	CALLA	#handleBlockErase
	JMP		endDoRFID

callBlockPermalockHandler:
	CALLA	#handleBlockPermalock
	JMP		endDoRFID

callReadBufferHandler:
	CALLA	#handleReadBuffer
	JMP		endDoRFID

callFileOpenHandler:
	CALLA	#handleFileOpen
	JMP		endDoRFID

callChallengeHandler:
	CALLA	#handleChallenge
	JMP		endDoRFID

callAuthenticateHandler:
	CALLA	#handleAuthenticate
	JMP		endDoRFID

callSecureCommHandler:
	CALLA	#handleSecureComm
	JMP		endDoRFID

callAuthCommHandler:
	CALLA	#handleAuthComm
	JMP		endDoRFID

;/************************************************************************************************************************************/
;/										DECIDE IF STAYING IN RFID LOOP		                                                         *
;/																																	 *
;/ If the abort flag has been set during the RFID transaction, return! Otherwise, keep doing RFID.									 *
;/************************************************************************************************************************************/
endDoRFID:
	TST.B	&(rfid.abortFlag)
	JZ		keepDoingRFID
	CALLA	#RFID_stopRxLog
	RETA

	.end
