/**
 * @file
 * @author	Graham Harvey
 * @date	1 May 2015
 * @brief	Functions allowing debugging functionality on the WISP.
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <wisp-base.h>

#include "debug.h"
#include "pin_assign.h"

#define DEBUG_RETURN			0x0001 // signals debug main loop to stop

#define LED_IN_DEBUG_STATE

#define TX_BUF_SIZE 16

static uint8_t tx_buf[TX_BUF_SIZE];

typedef enum {
    STATE_OFF = 0,
    STATE_IDLE,
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

typedef struct {
    uint8_t descriptor;
    uint8_t len;
    uint8_t *data;
} cmd_t;

static state_t state = STATE_OFF;

static uint16_t debug_flags = 0;

static uint16_t *wisp_sp; // stack pointer on debug entry

// expecting 2-byte messages from the debugger (identifier byte + descriptor byte)
static uint8_t uartRxBuf[DEBUG_UART_BUF_LEN];

static uint8_t cmd_data_buf[DEBUG_CMD_MAX_LEN];

static void set_state(state_t new_state)
{
#if CONFIG_STATE_PINS
    uint8_t port_value;
#endif

    state = new_state;

#if CONFIG_STATE_PINS
    // Encode state onto two indicator pins
    port_value = GPIO(PORT_STATE, OUT);
    port_value &= ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1)); // clear
    port_value |= (new_state & 0x1 ? BIT(PIN_STATE_0) : 0) |
                  (new_state & 0x2 ? BIT(PIN_STATE_1) : 0);
    GPIO(PORT_STATE, OUT) = port_value;
#endif
}

static void signal_debugger()
{
    // pulse the signal line

    // target signal line starts in high imedence state
    GPIO(PORT_SIG, OUT) |= BIT(PIN_SIG);        // output high
    GPIO(PORT_SIG, DIR) |= BIT(PIN_SIG);        // output enable
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);    // output low
    GPIO(PORT_SIG, DIR) &= ~BIT(PIN_SIG);    // back to high impedence state
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear interrupt flag (might have been set by the above)
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

#if 0 // TODO: save
    // save clock configuration
    clkInfo.CSCTL0 = CSCTL0;
    clkInfo.CSCTL1 = CSCTL1;
    clkInfo.CSCTL2 = CSCTL2;
    clkInfo.CSCTL3 = CSCTL3;
#endif

    // set up 8 MHz clock for UART drivers
    CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
    CSCTL1 = DCOFSEL_6;                       // Set DCO to 8MHz
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;  // Set SMCLK = MCLK = DCO
                                              // ACLK = VLOCLK
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
    CSCTL0_H = 0;                             // Lock CS registers

    UART_init(); // enable UART

    set_state(STATE_DEBUG);
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
#if 0 // TODO: restore (this causes reset, probably need to do bit by bit)
    CSCTL0 = clkInfo.CSCTL0;
    CSCTL1 = clkInfo.CSCTL1;
    CSCTL2 = clkInfo.CSCTL2;
    CSCTL3 = clkInfo.CSCTL3;
#else
    CSCTL1 = DCOFSEL0 + DCOFSEL1; //4MHz
    CSCTL2 = SELA_0 + SELS_3 + SELM_3;
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;
#endif
    CSCTL0_H = 0;                             // Lock CS registers

    // set up to return from debug_main
    debug_flags |= DEBUG_RETURN;
}

uint8_t *mem_addr_from_bytes(uint8_t *buf)
{
    return (uint8_t *)
        (((uint32_t)buf[3] << 24) |
        ((uint32_t)buf[2] << 16) |
        ((uint32_t)buf[1] << 8) |
        ((uint32_t)buf[0] << 0));
}

static void execute_cmd(cmd_t *cmd)
{
    uint8_t msg_len;
    uint8_t *address;
    uint8_t offset;
    uint8_t len;
    uint8_t i;

    switch (cmd->descriptor)
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
            break;
        }
        case WISP_CMD_READ_MEM:
        {
            // TODO: assert(msg->len == 4)
            uint8_t max_len = TX_BUF_SIZE - (1 + 1 + sizeof(uint32_t)); /* id, desc, addr */

            offset = 0;
            address = mem_addr_from_bytes(&cmd->data[offset]);
            offset += sizeof(uint32_t);
            len = cmd->data[offset];
            offset += sizeof(uint8_t);

            if (len > max_len)
                len = max_len;

            msg_len = 0;
            tx_buf[msg_len++] = UART_IDENTIFIER_WISP;
            tx_buf[msg_len++] = WISP_RSP_MEMORY;
            tx_buf[msg_len++] = sizeof(uint32_t) + len;
            tx_buf[msg_len++] = ((uint32_t)address >> 0) & 0xff;
            tx_buf[msg_len++] = ((uint32_t)address >> 8) & 0xff;
            tx_buf[msg_len++] = ((uint32_t)address >> 16) & 0xff;
            tx_buf[msg_len++] = ((uint32_t)address >> 24) & 0xff;

            for (i = 0; i < len; ++i)
                tx_buf[msg_len++] = *address++;

            UART_send(tx_buf, msg_len + 1); // +1 since send sends - 1 bytes (TODO)
            break;
        }
        case WISP_CMD_WRITE_MEM:
        {
            // TODO: assert(msg->len >= 5)

            offset = 0;
            address = mem_addr_from_bytes(&cmd->data[offset]);
            offset += sizeof(uint32_t);
            len = cmd->data[offset];
            offset += sizeof(uint8_t);
            uint8_t *value = &cmd->data[offset];

            for (i =  0; i < len; ++i) {
                *address = *value++;
                address++;
            }

            msg_len = 0;
            tx_buf[msg_len++] = UART_IDENTIFIER_WISP;
            tx_buf[msg_len++] = WISP_RSP_MEMORY;
            tx_buf[msg_len++] = sizeof(uint32_t) + sizeof(uint8_t);
            tx_buf[msg_len++] = ((uint32_t)address >> 0) & 0xff;
            tx_buf[msg_len++] = ((uint32_t)address >> 8) & 0xff;
            tx_buf[msg_len++] = ((uint32_t)address >> 16) & 0xff;
            tx_buf[msg_len++] = ((uint32_t)address >> 24) & 0xff;
            tx_buf[msg_len++] = *address;

            UART_send(tx_buf, msg_len + 1); // +1 since send sends - 1 bytes (TODO)
            break;
        }
        case WISP_CMD_EXIT_ACTIVE_DEBUG:
            exit_debug_mode();
            break;

        default: // invalid cmd
            break;
    }
}

/**
 * @brief	Parse and handle cmds that come from the debugger over UART
 * @return  Whether a command was parsed and is ready for execution
 */
static bool parse_cmd(cmd_t *cmd, uint8_t *msg, uint8_t len)
{
    static msgState_t msg_state = MSG_STATE_IDENTIFIER;
    static uint8_t data_len = 0;

    uint8_t i;
    for(i = 0; i < len; i++) {
        switch(msg_state)
        {
            case MSG_STATE_IDENTIFIER:
                {
                    uint8_t identifier = msg[i];
                    if(identifier == UART_IDENTIFIER_WISP) {
                        // good identifier byte
                        msg_state = MSG_STATE_DESCRIPTOR;
                    }

                    // else we had a bad identifier byte, so don't change the state
                    break;
                }

            case MSG_STATE_DESCRIPTOR:
                data_len = 0;
                cmd->descriptor = msg[i];
                cmd->len = 0;

                // TODO: info about whether there is more data or not should be
                // implicit in the length field. This function should be agnostic
                // to actual descriptor values cmds.
                switch(msg[i])
                {
                    // cmds without any data
                    case WISP_CMD_GET_PC:
                    case WISP_CMD_EXAMINE_MEMORY:
                    case WISP_CMD_EXIT_ACTIVE_DEBUG:
                        msg_state = MSG_STATE_IDENTIFIER;
                        return true;

                        // cmds with data
                    case WISP_CMD_READ_MEM:
                    case WISP_CMD_WRITE_MEM:
                        msg_state = MSG_STATE_DATALEN;
                        break;

                    default: // unknown cmd
                        break;
                }
                break;

            case MSG_STATE_DATALEN:
                data_len = msg[i]; // decremented as data bytes are parsed
                msg_state = MSG_STATE_DATA;
                break;
            case MSG_STATE_DATA:
                if (data_len)
                    cmd->data[cmd->len++] = msg[i];
                if (--data_len == 0) {
                    msg_state = MSG_STATE_IDENTIFIER;
                    return true;
                }
                break;
            default:
                break;
        }
    }

    return false;
}

/**
 * @brief    Debug mode main loop.  This executes when the WISP enters debug mode,
 *             and should allow debugging functionality.
 */
static void debug_main()
{
    cmd_t cmd = { .data = cmd_data_buf };

#ifdef LED_IN_DEBUG_STATE
    PLED2OUT |= PIN_LED2;
#endif

    while(1) {
        UART_receive(uartRxBuf, 1, 0xFF); // block until we receive a message
        if (parse_cmd(&cmd, uartRxBuf, 1))
            execute_cmd(&cmd);

        if(debug_flags & DEBUG_RETURN) {
            debug_flags &= ~DEBUG_RETURN;
            break;
        }
    }

#ifdef LED_IN_DEBUG_STATE
    PLED2OUT &= ~PIN_LED2;
#endif
}

static inline void handle_debugger_signal()
{
    switch (state) {
        case STATE_IDLE: // debugger requested us to enter debug mode
            enter_debug_mode();
            signal_debugger();
            debug_main();
            // debug loop exited (due to UART cmd to exit debugger)
            set_state(STATE_SUSPENDED); // sleep and wait for debugger to restore energy
            signal_debugger(); // tell debugger we have shutdown UART
            unmask_debugger_signal();
            break;
        case STATE_SUSPENDED: // debugger finished restoring the energy level
            set_state(STATE_IDLE); // return to the application code
            unmask_debugger_signal(); // listen for the next enter request
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
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear interrupt flag (might have been set by the above)

    __enable_interrupt();

    unmask_debugger_signal();

    set_state(STATE_IDLE);
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1(void)
#else
#error Compiler not supported!
#endif
{
	switch(__even_in_range(P1IV, P1IV_P1IFG7))
	{
        case INTFLAG(PORT_SIG, PIN_SIG):

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
	}
}
