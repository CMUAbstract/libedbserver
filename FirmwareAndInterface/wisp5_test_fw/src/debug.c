/**
 * @file
 * @author	Graham Harvey
 * @date	1 May 2015
 * @brief	Functions allowing debugging functionality on the WISP.
 */

#include <stdint.h>

#include "debug.h"
#include "wisp-base.h"
#include "pin_assign.h"

#define DEBUG_RETURN			0x0001 // signals debug main loop to stop

typedef enum {
    STATE_IDLE = 0,
    STATE_DEBUG,
    STATE_SUSPENDED,
} state_t;

typedef struct {
	uint16_t CSCTL0;
	uint16_t CSCTL1;
	uint16_t CSCTL2;
	uint16_t CSCTL3;
	uint16_t CSCTL4;
	uint16_t CSCTL5;
	uint16_t CSCTL6;
} clkInfo_t;

typedef enum {
	MSG_STATE_IDENTIFIER,	//!< UART identifier byte
	MSG_STATE_DESCRIPTOR,	//!< UART descriptor byte
	MSG_STATE_DATALEN,		//!< data length byte
	MSG_STATE_DATA			//!< UART data
} msgState_t;

static state_t state = STATE_IDLE;

static uint16_t debug_flags = 0;

static clkInfo_t clkInfo;

static uint16_t *wisp_sp; // stack pointer on debug entry

static void set_state(state_t new_state)
{
    state = new_state;

    // Encode state onto two indicator pins
    GPIO(PORT_STATE, OUT) &= ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1)); // clear
    GPIO(PORT_STATE, OUT) |= (new_state & 0x1 ? BIT(PIN_STATE_0) : 0) |
                             (new_state & 0x2 ? BIT(PIN_STATE_1) : 0);
}

static void signal_debugger()
{
    // pulse the signal line

    // target signal line starts in high imedence state
    GPIO(PORT_SIG, OUT) |= BIT(PIN_SIG);        // output high
    GPIO(PORT_SIG, DIR) |= BIT(PIN_SIG);        // output enable
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);    // output low
    GPIO(PORT_SIG, DIR) &= ~BIT(PIN_SIG);    // back to high impedence state
}

static void unmask_debugger_signal()
{
    GPIO(PORT_SIG, IE) |= BIT(PIN_SIG); // enable interrupt
    GPIO(PORT_SIG, IES) &= ~BIT(PIN_SIG); // rising edge
}

static void mask_debugger_signal()
{
    GPIO(PORT_SIG, IE) &= ~BIT(PIN_SIG); // disable interrupt
}

static void enter_debug_mode()
{
    __enable_interrupt();

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

    set_state(STATE_DEBUG);
    PLED2OUT |= PIN_LED2;
}

void exit_debug_mode()
{
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

    PLED2OUT &= ~PIN_LED2;
}

/**
 * @brief	Parse and handle cmds that come from the debugger over UART
 */
static void parseAndExecute(uint8_t *msg, uint8_t len)
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
                exit_debug_mode();
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

/**
 * @brief    Debug mode main loop.  This executes when the WISP enters debug mode,
 *             and should allow debugging functionality.
 */
static void debug_main()
{
    // expecting 2-byte messages from the debugger (identifier byte + descriptor byte)
    uint8_t uartRxBuf[DEBUG_UART_BUF_LEN];

    while(1) {
        UART_receive(uartRxBuf, DEBUG_UART_BUF_LEN, 0xFF); // block until we receive a message
        parseAndExecute(uartRxBuf, DEBUG_UART_BUF_LEN); // parse the message and perform actions

        if(debug_flags & DEBUG_RETURN) {
            debug_flags &= ~DEBUG_RETURN;
            return;
        }
    }
}

static inline void handle_debugger_signal()
{
    switch (state) {
        case STATE_IDLE: // debugger requested us to enter debug mode
            enter_debug_mode();
            signal_debugger();
            debug_main();
            // debug loop exited (due to UART cmd to exit debugger)
            signal_debugger(); // tell debugger we have shutdown UART
            unmask_debugger_signal();
            set_state(STATE_SUSPENDED); // sleep and wait for debugger to restore energy
            break;
        case STATE_SUSPENDED: // debugger finished restoring the energy level
            set_state(STATE_IDLE); // return to the application code
            break;
        default:
            // received an unexpected signal from the debugger
            break;
    }
}

void debug_setup()
{
    // these pins report state of the debugger state machine on the target
    GPIO(PORT_STATE, OUT) &= ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1)); // output low
    GPIO(PORT_STATE, DIR) |= BIT(PIN_STATE_0) | BIT(PIN_STATE_1); // output

    GPIO(PORT_SIG, DIR) &= ~BIT(PIN_SIG); // input

    unmask_debugger_signal();
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

		// Clear the int flag, because during active debug mode, we are
		// in the interrupt context (we return from interrupt on exit
		// from the debug node) and we re-use the signal pin before exit.
		GPIO(PORT_SIG, IFG) &= ~PIN_SIG;

		// Save application stack pointer
		// TODO: ideally this would be in enter_debug_mode, but then
		// would need to subtract the extra call frames.
		wisp_sp = (uint16_t *) __get_SP_register();

		mask_debugger_signal();

		handle_debugger_signal();

		/* Power state manipulation is required to be inside the ISR */
		switch (state) {
			case STATE_SUSPENDED: /* DEBUG->SUSPENDED just happened */
				__bis_SR_register(LPM4_bits + GIE); // go to sleep
				break;
			case STATE_IDLE: /* SUSPENDED->IDLE just happened */
				__bic_SR_register_on_exit(LPM4_bits); // resume execution upon return from isr
				break;
			default: /* nothing to do */
				break;
		}
		break;
	case P3IV_P3IFG5:
		break;
	case P3IV_P3IFG6:
		break;
	case P3IV_P3IFG7:
		break;
	}
}
