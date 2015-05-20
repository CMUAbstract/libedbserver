/**
 * @file
 * @author	Graham Harvey
 * @date	1 May 2015
 * @brief	Functions allowing debugging functionality on the WISP.
 */

#include <stdint.h>

#include "debug.h"
#include "wisp-base.h"

#define DEBUG_RETURN			0x0001

uint16_t debug_flags = 0;

clkInfo_t clkInfo;

uint16_t *wisp_sp; // stack pointer on debug entry

void debug_main()
{
	// save clock configuration
	clkInfo.CSCTL0 = CSCTL0;
	clkInfo.CSCTL1 = CSCTL1;
	clkInfo.CSCTL2 = CSCTL2;
	clkInfo.CSCTL3 = CSCTL3;
	clkInfo.CSCTL4 = CSCTL4;
	clkInfo.CSCTL5 = CSCTL5;
	clkInfo.CSCTL6 = CSCTL6;

	// set up 8 MHz clock for UART drivers
	CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
	CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK = MCLK = DCO
											  // ACLK = VLOCLK
	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
	CSCTL0_H = 0;                             // Lock CS registers

	UART_init(); // enable UART

	// expecting 2-byte messages from the debugger (identifier byte + descriptor byte)
	uint8_t uartRxBuf[DEBUG_UART_BUF_LEN];

	while(1) {
		UART_receive(uartRxBuf, DEBUG_UART_BUF_LEN, 0xFF); // block until we receive a message
		debug_parseAndExecute(uartRxBuf, DEBUG_UART_BUF_LEN); // parse the message and perform actions

		if(debug_flags & DEBUG_RETURN) {
			debug_flags &= ~DEBUG_RETURN;
			return;
		}
	}
}

void debug_parseAndExecute(uint8_t *msg, uint8_t len)
{
	static msgState_t state = MSG_STATE_IDENTIFIER;

	uint8_t i;
	for(i = 0; i < len; i++) {
		switch(state)
		{
		case MSG_STATE_IDENTIFIER:
		{
			uint8_t identifier = msg[i];
			if(identifier == UART_IDENTIFIER_WISP) {
				// good identifier byte
				state = MSG_STATE_DESCRIPTOR;
			}

			// else we had a bad identifier byte, so don't change the state
			break;
		}

		case MSG_STATE_DESCRIPTOR:
			switch(msg[i])
			{
			case WISP_CMD_GET_PC:
			{
				// get the program counter
				uint16_t wisp_pc = *(wisp_sp + 11); // 22-byte offset to PC

				// stick to the UART message structure
				uint8_t txBuf[5] = { UART_IDENTIFIER_WISP, WISP_RSP_PC,
									 sizeof(uint16_t), 0x00, 0x00 };
				txBuf[3] = wisp_pc & 0xFF;
				txBuf[4] = (wisp_pc >> 8) & 0xFF;
				UART_send(txBuf, 6); // For some reason, UART_send seems to send size - 1 bytes.
									 // This message is only 5 bytes long.

				state = MSG_STATE_IDENTIFIER;
				break;
			}

			case WISP_CMD_EXAMINE_MEMORY:
				// not yet implemented
				break;

			case WISP_CMD_EXIT_ACTIVE_DEBUG:
				debug_pre_exit();
				return;

			default:
				break;
			}
			break;

		case MSG_STATE_DATALEN:
		case MSG_STATE_DATA:
		default:
			break;
		}
	}
}

void debug_pre_exit()
{
	// raise AUX 2 to let the debugger know we're preparing to exit
	P3OUT |= PIN_AUX2;

	// disable UART
	// Not sure how to do this best, but set all UCA0* registers to
	// their default values.  See User's Guide for default values.
	PUART_TXSEL0 &= ~PIN_UART_TX;
	PUART_TXSEL1 &= ~PIN_UART_TX;
	P2DIR &= ~PIN_UART_TX; // match pin initialization
	P2OUT &= ~PIN_UART_TX;
	PUART_RXSEL0 &= ~PIN_UART_RX;
	PUART_RXSEL1 &= ~PIN_UART_RX;
	P2DIR &= ~PIN_UART_RX; // match pin initialization
	P2OUT &= ~PIN_UART_RX;
	UCA0CTLW0 = 0x0001;
	UCA0BR0 = 0x0000;
	UCA0MCTLW = 0x0000;
	UCA0IE = 0x0000;
	UCA0IFG = 0x0000;

	// restore clock configuration
	CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
	CSCTL0 = clkInfo.CSCTL0;
	CSCTL1 = clkInfo.CSCTL1;
	CSCTL2 = clkInfo.CSCTL2;
	CSCTL3 = clkInfo.CSCTL3;
	CSCTL4 = clkInfo.CSCTL4;
	CSCTL5 = clkInfo.CSCTL5;
	CSCTL6 = clkInfo.CSCTL6;
	CSCTL0_H = 0;                             // Lock CS registers

	// set up to return from debug_main
	debug_flags |= DEBUG_RETURN;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT3_VECTOR
__interrupt void Port_3(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT3_VECTOR))) Port_3(void)
#else
#error Compiler not supported!
#endif
{
	switch(__even_in_range(P3IV, P3IV_P3IFG7))
	{
	case P3IV_NONE:
		break;
	case P3IV_P3IFG0:
		break;
	case P3IV_P3IFG1:
		break;
	case P3IV_P3IFG2:
		break;
	case P3IV_P3IFG3:
		break;
	case P3IV_P3IFG4:
		// AUX 1 interrupt -- active debug mode
		if(P3IN & PIN_AUX1) {
			// Enter active debug mode

			// set up active debug mode exit interrupt
			P3IES |= PIN_AUX1;	// switch to falling edge

			// get stack pointer so we can easily access state information
			wisp_sp = (uint16_t *) __get_SP_register();

			__enable_interrupt();

			debug_main(); // main debugging loop

			// When debug_main returns, we want to go to sleep and return to WISP actions
			// when we wake up.

			// pull AUX_2 low to indicate to the debugger
			// that the WISP is ready to exit debug mode
			P3OUT &= ~PIN_AUX2;	// output low
			__bis_SR_register(LPM4_bits + GIE); // WISP will continue normal execution when it wakes
			return; // PC will be restored on return from interrupt
		} else {
			// Exit active debug mode

			// The exit from active debug mode is split into two parts:
				//	1. Debugger sends UART message to WISP
				//		- WISP raises GPIO AUX_2 while performing pre-exit tasks
				//		- WISP performs pre-exit tasks (debug_pre_exit)
				//		- WISP pulls AUX_2 low when ready to return, and sleeps
				//	2. Debugger pulls AUX_1 low, which only wakes the WISP and returns from interrupt (here)

			// set up interrupt for entering active debug mode again
			P3IES &= ~PIN_AUX1; // switch to rising edge

			__bic_SR_register_on_exit(LPM4_bits);
		}

		break;
	case P3IV_P3IFG5:
		break;
	case P3IV_P3IFG6:
		break;
	case P3IV_P3IFG7:
		break;
	}

	P3IFG &= ~PIN_AUX1;
}
