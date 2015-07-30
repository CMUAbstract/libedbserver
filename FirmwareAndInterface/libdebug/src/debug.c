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
#include "target_comm.h"

/* The linker script needs to allocate .fram_vars section into FRAM region. */
#define __fram __attribute__((section(".fram_vars")))

#define DEBUG_RETURN                0x0001 // signals debug main loop to stop
#define DEBUG_REQUESTED_BY_TARGET   0x0002 // the target requested to enter debug mode

#define DEBUG_MODE_REQUEST_WAIT_STATE_BITS      LPM0_bits
#define DEBUG_MODE_EXIT_WAIT_STATE_BITS         LPM0_bits

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
	MSG_STATE_PADDING,      //!< padding in header for alignment
	MSG_STATE_DATA,		    //!< UART data
} msgState_t;

typedef struct {
    uint8_t descriptor;
    uint8_t len;
    uint8_t *data;
} cmd_t;

typedef struct {
    interrupt_type_t type;
    uint16_t id;
    uint8_t features;
} interrupt_context_t;

static state_t state = STATE_OFF;

static uint16_t debug_flags = 0;
static interrupt_context_t interrupt_context;

volatile uint16_t __fram _libdebug_internal_breakpoints = 0x00;

static uint16_t *wisp_sp; // stack pointer on debug entry

// expecting 2-byte messages from the debugger (identifier byte + descriptor byte)
static uint8_t uartRxBuf[CONFIG_DEBUG_UART_BUF_LEN];

static uint8_t cmd_data_buf[WISP_CMD_MAX_LEN];

static void set_state(state_t new_state)
{
#ifdef CONFIG_STATE_PINS
    uint8_t port_value;
#endif

    state = new_state;

#ifdef CONFIG_STATE_PINS
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

static void signal_debugger_with_data(uint8_t data)
{
    unsigned i;
    uint8_t bit;
    uint8_t port_bits[SIG_SERIAL_NUM_BITS];

    // Precompute all port values in order to keep the bit duration constant,
    // i.e. so that it does not vary with the bit index and bit value.
    for (i = 0; i < SIG_SERIAL_NUM_BITS; ++i) {
        bit = (data >> i) & 0x1;
        port_bits[i] = bit << PIN_SIG;
    }

    __disable_interrupt();

    // target signal line starts in high imedence state

    // starting pulse
    GPIO(PORT_SIG, OUT) |= BIT(PIN_SIG);        // output high
    GPIO(PORT_SIG, DIR) |= BIT(PIN_SIG);        // output enable
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);    // output low

    // Need constant and short time between bits, so no loops or conditionals
#define PULSE_BIT(idx) \
    __delay_cycles(SIG_SERIAL_BIT_DURATION_ON_TARGET); \
    GPIO(PORT_SIG, OUT) |= port_bits[idx]; \
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG); \

#if SIG_SERIAL_NUM_BITS > 3
    PULSE_BIT(3);
#endif
#if SIG_SERIAL_NUM_BITS > 2
    PULSE_BIT(2);
#endif
#if SIG_SERIAL_NUM_BITS > 1
    PULSE_BIT(1);
#endif
#if SIG_SERIAL_NUM_BITS > 0
    PULSE_BIT(0);
#endif

    // terminating pulse: must happen after the interval for the last bit elapses
    __delay_cycles(SIG_SERIAL_BIT_DURATION_ON_TARGET); // ignore the few compute instructions
    GPIO(PORT_SIG, OUT) |= BIT(PIN_SIG);        // output high
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);    // output low

    GPIO(PORT_SIG, DIR) &= ~BIT(PIN_SIG);    // back to high impedence state
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear interrupt flag (might have been set by the above)

    __enable_interrupt();
}


static void unmask_debugger_signal()
{
    GPIO(PORT_SIG, IES) &= ~BIT(PIN_SIG); // rising edge
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear the flag that might have been set by IES write
    GPIO(PORT_SIG, IE) |= BIT(PIN_SIG); // enable interrupt
}

static void mask_debugger_signal()
{
    GPIO(PORT_SIG, IE) &= ~BIT(PIN_SIG); // disable interrupt
}

static void UART_teardown()
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
}

static void clear_interrupt_context()
{
    interrupt_context.type = INTERRUPT_TYPE_NONE;
    interrupt_context.id = 0;
    interrupt_context.features = 0;
}

static void enter_debug_mode()
{
    __enable_interrupt();

    if (interrupt_context.features & DEBUG_MODE_WITH_UART)
        UART_init();

    set_state(STATE_DEBUG);
}

void exit_debug_mode()
{
    if (interrupt_context.features & DEBUG_MODE_WITH_UART)
        UART_teardown();

    clear_interrupt_context();
}

void request_debug_mode(interrupt_type_t int_type, unsigned int_id)
{
    // Disable interrupts before unmasking debugger signal to make sure
    // we are asleep (at end of this function) before ISR runs. Otherwise,
    // the race completely derails the sequence to enter-exit debug mode.
    // Furthermore, to prevent a signal from the debugger arriving while
    // we are trying to request debug mode, disable interrupts at the
    // very beginning of this function.
    __disable_interrupt();

    debug_flags |= DEBUG_REQUESTED_BY_TARGET;
    interrupt_context.type = int_type;
    interrupt_context.id = int_id;

    switch (int_type) {
        case INTERRUPT_TYPE_ENERGY_GUARD:
            interrupt_context.features = 0;
            break;
        default:
            interrupt_context.features = DEBUG_MODE_FULL_FEATURES;
            break;
    }

    mask_debugger_signal();

    switch (state) {
        case STATE_DEBUG: // an assert/breakpoint nested in an energy guard
            signal_debugger_with_data(SIG_CMD_INTERRUPT);
            break;
        default: // hot path (hit an assert/bkpt), we want the debugger to take action asap
            signal_debugger();
    }

    unmask_debugger_signal();

    // go to sleep, enable interrupts, and wait for signal from debugger
    __bis_SR_register(DEBUG_MODE_REQUEST_WAIT_STATE_BITS | GIE);
}

void resume_application()
{
    exit_debug_mode();

    set_state(STATE_SUSPENDED); // sleep and wait for debugger to restore energy

    // debugger is in DEBUG state, so our signal needs to contain
    // the information about whether we are exiting the debug mode
    // (as we are here) or whether we are requesting a nested debug
    // mode due to an assert/bkpt.
    signal_debugger_with_data(SIG_CMD_EXIT); // tell debugger we have shutdown UART

    unmask_debugger_signal();
}

uintptr_t mem_addr_from_bytes(uint8_t *buf)
{
    return (uintptr_t)
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
            uintptr_t address = *(wisp_sp + 11); // 22-byte offset to PC

            msg_len = 0;
            tx_buf[msg_len++] = UART_IDENTIFIER_WISP;
            tx_buf[msg_len++] = WISP_RSP_ADDRESS;
            tx_buf[msg_len++] = sizeof(uint32_t);
            tx_buf[msg_len++] = 0; // padding
            tx_buf[msg_len++] = ((uintptr_t)address >> 0) & 0xff;
            tx_buf[msg_len++] = ((uintptr_t)address >> 8) & 0xff;
            tx_buf[msg_len++] = 0; // TODO: 20-bit ptr
            tx_buf[msg_len++] = 0;

            UART_send(tx_buf, msg_len + 1);  // +1 since send sends - 1 bytes (TODO)
            break;
        }
        case WISP_CMD_READ_MEM:
        {
            // TODO: assert(msg->len == 4)
            uint8_t max_len = TX_BUF_SIZE - (UART_MSG_HEADER_SIZE + sizeof(uint32_t)); /* addr */

            offset = 0;
            address = (uint8_t *)mem_addr_from_bytes(&cmd->data[offset]); // TODO: 20-bit ptr
            offset += sizeof(uint32_t);
            len = cmd->data[offset];
            offset += sizeof(uint8_t);

            if (len > max_len)
                len = max_len;

            msg_len = 0;
            tx_buf[msg_len++] = UART_IDENTIFIER_WISP;
            tx_buf[msg_len++] = WISP_RSP_MEMORY;
            tx_buf[msg_len++] = sizeof(uint32_t) + len;
            tx_buf[msg_len++] = 0; // padding
            tx_buf[msg_len++] = ((uintptr_t)address >> 0) & 0xff;
            tx_buf[msg_len++] = ((uintptr_t)address >> 8) & 0xff;
            tx_buf[msg_len++] = 0; // TODO: 20-bit ptr
            tx_buf[msg_len++] = 0;

            for (i = 0; i < len; ++i)
                tx_buf[msg_len++] = *address++;

            UART_send(tx_buf, msg_len + 1); // +1 since send sends - 1 bytes (TODO)
            break;
        }
        case WISP_CMD_WRITE_MEM:
        {
            // TODO: assert(msg->len >= 5)

            offset = 0;
            address = (uint8_t *)mem_addr_from_bytes(&cmd->data[offset]); // TODO: 20-bit ptr
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
            tx_buf[msg_len++] = 0; // padding
            tx_buf[msg_len++] = ((uintptr_t)address >> 0) & 0xff;
            tx_buf[msg_len++] = ((uintptr_t)address >> 8) & 0xff;
            tx_buf[msg_len++] = 0; // TODO: 20-bit ptr
            tx_buf[msg_len++] = 0;
            tx_buf[msg_len++] = *address;

            UART_send(tx_buf, msg_len + 1); // +1 since send sends - 1 bytes (TODO)
            break;
        }
        case WISP_CMD_BREAKPOINT:
        {
            uint8_t index = cmd->data[0];
            bool enable = cmd->data[1];

            if (enable)
                _libdebug_internal_breakpoints |= 1 << index;
            else
                _libdebug_internal_breakpoints &= ~(1 << index);

            msg_len = 0;
            tx_buf[msg_len++] = UART_IDENTIFIER_WISP;
            tx_buf[msg_len++] = WISP_RSP_BREAKPOINT;
            tx_buf[msg_len++] = 0; // length
            tx_buf[msg_len++] = 0; // padding

            UART_send(tx_buf, msg_len + 1); // +1 since send sends -1 bytes (TODO)
            break;
        }
        case WISP_CMD_EXIT_ACTIVE_DEBUG:
            exit_debug_mode();
            debug_flags |= DEBUG_RETURN; // return from debug_main
            break;
        
        case WISP_CMD_GET_INTERRUPT_CONTEXT:
            msg_len = 0;
            tx_buf[msg_len++] = UART_IDENTIFIER_WISP;
            tx_buf[msg_len++] = WISP_RSP_INTERRUPT_CONTEXT;
            tx_buf[msg_len++] = 3 * sizeof(uint8_t);
            tx_buf[msg_len++] = 0; // padding
            tx_buf[msg_len++] = interrupt_context.type;
            tx_buf[msg_len++] = interrupt_context.id;
            tx_buf[msg_len++] = interrupt_context.id >> 8;

            UART_send(tx_buf, msg_len + 1); // +1 since send sends -1 bytes (TODO)
            break;

        case WISP_CMD_SERIAL_ECHO: {
            uint8_t echo_value = cmd->data[0];

            mask_debugger_signal();
            signal_debugger_with_data(echo_value);
            unmask_debugger_signal();

            msg_len = 0;
            tx_buf[msg_len++] = UART_IDENTIFIER_WISP;
            tx_buf[msg_len++] = WISP_RSP_SERIAL_ECHO;
            tx_buf[msg_len++] = 0; // length
            tx_buf[msg_len++] = 0; // padding

            UART_send(tx_buf, msg_len + 1); // +1 since send sends -1 bytes (TODO)
            break;
        }

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
                msg_state = MSG_STATE_DATALEN;
                break;

            case MSG_STATE_DATALEN:
                data_len = msg[i]; // decremented as data bytes are parsed
                msg_state = MSG_STATE_PADDING;
                break;

            case MSG_STATE_PADDING:
                if (data_len) {
                    msg_state = MSG_STATE_DATA;
                } else { // done
                    msg_state = MSG_STATE_IDENTIFIER;
                    return true;
                }
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
        case STATE_DEBUG: // debugger requested to enter a *nested* debug mode

            // If entering debug mode on debugger's initiative (i.e. when we
            // didn't request it), then need to set the features.
            if (interrupt_context.type == INTERRUPT_TYPE_NONE) {
                interrupt_context.type = INTERRUPT_TYPE_DEBUGGER_REQ;
                interrupt_context.features = DEBUG_MODE_FULL_FEATURES;
            }

            enter_debug_mode();
            signal_debugger_with_data(interrupt_context.features);
            unmask_debugger_signal();

            if (interrupt_context.features & DEBUG_MODE_INTERACTIVE) {
                debug_main();
                // debug loop exited (due to UART cmd to exit debugger), release debugger
                set_state(STATE_SUSPENDED); // sleep and wait for debugger to restore energy
                signal_debugger(); // tell debugger we have shutdown UART
            } // else: exit the ISR, let the app continue in tethered mode
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
#ifdef CONFIG_STATE_PINS
    GPIO(PORT_STATE, OUT) &= ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1)); // output low
    GPIO(PORT_STATE, DIR) |= BIT(PIN_STATE_0) | BIT(PIN_STATE_1); // output
#endif

#if defined(CONFIG_ENABLE_PASSIVE_BREAKPOINTS) // codepoint pins are outputs
    GPIO(PORT_CODEPOINT, OUT) &= ~BITS_CODEPOINT;
    GPIO(PORT_CODEPOINT, DIR) |= BITS_CODEPOINT;
#elif !defined(CONFIG_STATE_PINS) // codepoint pins are inputs for target-side breakpoints
    GPIO(PORT_CODEPOINT, DIR) &= ~BITS_CODEPOINT;
#endif

    GPIO(PORT_SIG, DIR) &= ~BIT(PIN_SIG); // input
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear interrupt flag (might have been set by the above)

    __enable_interrupt();

    unmask_debugger_signal();

    set_state(STATE_IDLE);

    // For measuring boot latency
    // GPIO(PORT_STATE, OUT) |= BIT(PIN_STATE_0);
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
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
            wisp_sp = (uint16_t *) __get_SP_register();
#elif defined(__GNUC__)
            wisp_sp = 0x0; // TODO
#else
#error Compiler not supported!
#endif

            mask_debugger_signal();

            handle_debugger_signal();

            /* Power state manipulation is required to be inside the ISR */
            switch (state) {
                case STATE_DEBUG: /* IDLE->DEBUG just happend: entered energy guard */
                    // TODO: we also get here on *nested* assert/bkpt -- what happens
                    // in that case? It seems to work (at least for nested assert),
                    // but comments need to address this case. Also, nested bkpt need
                    // to be tested.

                    // We clear the sleep flags corresponding to the sleep on request
                    // to enter debug mode here, and do not touch them in the DEBUG->SUSPENDED
                    // transition because when upon exiting from the guard we will
                    // not be asleep.
                    if (debug_flags & DEBUG_REQUESTED_BY_TARGET) {
                        debug_flags &= ~DEBUG_REQUESTED_BY_TARGET;
                        __bic_SR_register_on_exit(DEBUG_MODE_REQUEST_WAIT_STATE_BITS);
                    }

                    // Leave the debugger signal masked. The next thing that will
                    // happen in the sequence is us signaling the debugger when
                    // we are exiting the energy guard.

                    // Once we return from this ISR, the application continues with
                    // continuous power on and the debugger in DEBUG state.
                    break;

                case STATE_SUSPENDED: /* DEBUG->SUSPENDED just happened */
                    // Before unmasking the signal interrupt, disable interrupts
                    // globally in order to not let the next signal interrupt happen
                    // until we are asleep. Unmasking won't let the interrupt
                    // call the ISR.
                    __disable_interrupt();
                    unmask_debugger_signal();

                    __bis_SR_register(DEBUG_MODE_EXIT_WAIT_STATE_BITS | GIE); // go to sleep

                    // We will get here after the next ISR (IDLE case) returns
                    // because it would have been invoked while we are asleep
                    // on the line above. That is, the IDLE case ISR is nested
                    // within the SUSPENDED case ISR. If this debug mode exit
                    // sequence is happening because the target had requested
                    // debug mode, then the current ISR was invoked while
                    // asleep in request_debug_mode(). In order to wakeup from
                    // that sleep upon returning from this outer ISR (SUSPENDED
                    // case), we need to clear the sleep bits (otherwise, the
                    // MCU will go to sleep when the SR bits are automatically
                    // restored upon return from interrupt).
                    if (debug_flags & DEBUG_REQUESTED_BY_TARGET) {
                        debug_flags &= ~DEBUG_REQUESTED_BY_TARGET;
                        __bic_SR_register_on_exit(DEBUG_MODE_REQUEST_WAIT_STATE_BITS);
                    }
                    // Once we return from this outer ISR, application execution resumes
                    break;

                case STATE_IDLE: /* SUSPENDED->IDLE just happened */

                    // Before unmasking the signal interrupt, disable
                    // interrupts globally in order to not let the next signal
                    // interrupt happen until either we return from this ISR.
                    // Unmasking won't let the interrupt call the ISR.
                    __disable_interrupt();
                    unmask_debugger_signal();

                    // We were sleeping on the suspend line in the case above when
                    // the current ISR got called, so before returning, clear the
                    // sleep flags (otherwise, we would go back to sleep after
                    // returning from this ISR because the SR flags prior to the ISR
                    // call are automatically restored upon return from ISR).
                    //
                    // Interrupt are current disabled (by the disable call
                    // before unmasking the signal interrupt). The adding the
                    // GIE flag here re-enables the interrupts only after
                    // return from the current ISR, so that the next signal ISR
                    // (unrelated to current enter-exit debug mode sequence)
                    // doesn't get nested within the current one.
                    __bic_SR_register_on_exit(DEBUG_MODE_EXIT_WAIT_STATE_BITS | GIE);
                    // Once we return from this inner ISR we end up in the outer ISR
                    break;
                default: /* nothing to do */
                    break;
            }
            break;
	}
}
