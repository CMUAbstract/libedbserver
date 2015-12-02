#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include <libedb/target_comm.h>

#include "pin_assign.h"
#include "host_comm.h"
#include "host_comm_impl.h"
#include "target_comm_impl.h"
#include "adc.h"
#include "uart.h"
#include "i2c.h"
#include "systick.h"
#include "rfid.h"
#include "rfid_decoder.h"
#include "minmax.h"
#include "config.h"
#include "error.h"
#include "main_loop.h"
#include "params.h"
#include "charge.h"
#include "comparator.h"
#include "codepoint.h"
#include "tether.h"
#include "interrupt.h"
#include "clock.h"
#include "profile.h"
#include "payload.h"

#ifdef CONFIG_PWM_CHARGING
#include "pwm.h"
#endif


/**
 * @defgroup    LOG_DEFINES     Logging flags
 * @brief       Flags to set in a bit mask to check which ADC readings to log
 * @{
 */
#define LOG_VCAP                    0x01 //!< Logging Vcap
#define LOG_VBOOST                  0x02 //!< Logging Vboost
#define LOG_VREG                    0x04 //!< Logging Vreg
#define LOG_VRECT                   0x08 //!< Logging Vrect
#define LOG_VINJ					0x10 //!< Logging Vinj
/** @} End LOG_DEFINES */

#define STREAM_REPLY_MAX_LEN (1 /* num chans */ + ADC_MAX_CHANNELS * sizeof(uint16_t))

volatile uint16_t main_loop_flags = 0; // bit mask containing bit flags to check in the main loop

static uint16_t debug_mode_flags = 0; // TODO: set these by decoding serial bits on signal line
static int sig_serial_bit_index; // debug mode flags are serially encoded on the signal line
static bool target_powered = false; // user requested continuous power

static unsigned sig_serial_echo_value = 0;
static state_t saved_sig_serial_echo_state;

static sig_cmd_t target_sig_cmd;

static interrupt_context_t interrupt_context;

static uint16_t adc_streams_bitmask; // streams from ADC currently streaming
static uint16_t streams_bitmask; // currently enabled streams

static uartPkt_t usbRxPkt = { .processed = 1 };

static void set_state(state_t new_state)
{
    state = new_state;

#ifdef CONFIG_STATE_PINS
    // Encode state onto two indicator pins
    GPIO(PORT_STATE, OUT) = (GPIO(PORT_STATE, OUT) & ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1))) |
                             (new_state << PIN_STATE_0);
#endif
}

#ifdef CONFIG_SCOPE_TRIGGER_SIGNAL
/**
 * @brief       Pulse a designated pin for triggering an oscilloscope
 */
static void trigger_scope()
{
    GPIO(PORT_TRIGGER, OUT) |= BIT(PIN_TRIGGER);
    GPIO(PORT_TRIGGER, DIR) |= BIT(PIN_TRIGGER);
    GPIO(PORT_TRIGGER, OUT) &= ~BIT(PIN_TRIGGER);
}
#endif // CONFIG_SCOPE_TRIGGER_SIGNAL

/**
 * @brief	Send an interrupt to the target device
 */
static void signal_target()
{
    // pulse the signal line

    // target signal line starts in high imedence state
    GPIO(PORT_SIG, OUT) |= BIT(PIN_SIG);		// output high
    GPIO(PORT_SIG, DIR) |= BIT(PIN_SIG);		// output enable
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);    // output low
    GPIO(PORT_SIG, DIR) &= ~BIT(PIN_SIG);    // back to high impedence state
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear interrupt flag (might have been set by the above)
}

/**
 * @brief	Enable interrupt line between the debugger and the target device
 */
static void unmask_target_signal()
{
    GPIO(PORT_SIG, IES) &= ~BIT(PIN_SIG); // rising edge
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear interrupt flag (might have been set by the above)
    GPIO(PORT_SIG, IE) |= BIT(PIN_SIG);   // enable interrupt
}

/**
 * @brief	Disable interrupt line between the debugger and the target device
 */
static void mask_target_signal()
{
    GPIO(PORT_SIG, IE) &= ~BIT(PIN_SIG); // disable interrupt
}

static void continuous_power_on()
{
    // The output level was configured high on boot up
    GPIO(PORT_CONT_POWER, DIR) |= BIT(PIN_CONT_POWER);
}

static void continuous_power_off()
{
    GPIO(PORT_CONT_POWER, DIR) &= ~BIT(PIN_CONT_POWER); // to high-z state
}

static inline void reset_serial_decoder()
{
    sig_serial_bit_index = SIG_SERIAL_NUM_BITS;

    TIMER(TIMER_SIG_SERIAL_DECODE, CCR0) = SIG_SERIAL_BIT_DURATION_ON_DEBUGGER;
    TIMER(TIMER_SIG_SERIAL_DECODE, CTL) |= TACLR | TASSEL__SMCLK;
    TIMER(TIMER_SIG_SERIAL_DECODE, CCTL0) &= ~CCIFG;
    TIMER(TIMER_SIG_SERIAL_DECODE, CCTL0) |= CCIE;
}

static inline void start_serial_decoder()
{
    TIMER(TIMER_SIG_SERIAL_DECODE, CTL) |= MC__UP; // start

#ifdef CONFIG_SIG_SERIAL_DECODE_PINS
    GPIO(PORT_SERIAL_DECODE, OUT) |= BIT(PIN_SERIAL_DECODE_TIMER);
    GPIO(PORT_SERIAL_DECODE, OUT) &= ~BIT(PIN_SERIAL_DECODE_TIMER);
#endif
}

static inline void stop_serial_decoder()
{
    TIMER(TIMER_SIG_SERIAL_DECODE, CTL) = 0;

#ifdef CONFIG_SIG_SERIAL_DECODE_PINS
    GPIO(PORT_SERIAL_DECODE, OUT) |= BIT(PIN_SERIAL_DECODE_TIMER);
    GPIO(PORT_SERIAL_DECODE, OUT) &= ~BIT(PIN_SERIAL_DECODE_TIMER);
#endif
}

static void enter_debug_mode(interrupt_type_t int_type, unsigned flags)
{
    interrupt_context.type = int_type;
    interrupt_context.id = 0;

    set_state(STATE_ENTERING);

    if (!(flags & DEBUG_MODE_NESTED)) {
        interrupt_context.saved_vcap = ADC_read(ADC_CHAN_INDEX_VCAP);
    } else {
        interrupt_context.saved_debug_mode_flags = debug_mode_flags;
    }

    reset_serial_decoder();

    debug_mode_flags = flags;

    mask_target_signal();
    signal_target();
    unmask_target_signal();
}

static void exit_debug_mode()
{
    set_state(STATE_EXITING);

    // interrupt_context cleared after the target acks the exit request

    unmask_target_signal();
    target_comm_send_exit_debug_mode();
}

static void reset_state()
{
    continuous_power_off();
    stop_serial_decoder();
    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_GREEN);
    set_state(STATE_IDLE);
    unmask_target_signal();
}

static void interrupt_target()
{
    uint16_t cur_vreg;

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_vreg = ADC_read(ADC_CHAN_INDEX_VREG);
    } while (cur_vreg < MCU_ON_THRES);

    __delay_cycles(MCU_BOOT_LATENCY_CYCLES);

    enter_debug_mode(INTERRUPT_TYPE_DEBUGGER_REQ, DEBUG_MODE_FULL_FEATURES);
}

static void get_target_interrupt_context(interrupt_context_t *int_context)
{
    // In case target requested the interrupt, ask it for more details
    target_comm_send_get_interrupt_context();
    while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
            (wispRxPkt.descriptor != WISP_RSP_INTERRUPT_CONTEXT)); // wait for response
    int_context->type = (interrupt_type_t)wispRxPkt.data[0];
    int_context->id = ((uint16_t)wispRxPkt.data[2] << 8) | wispRxPkt.data[1];
    wispRxPkt.processed = 1;
}


static void finish_enter_debug_mode()
{
    // WISP has entered debug main loop
    set_state(STATE_DEBUG);
    GPIO(PORT_LED, OUT) |= BIT(PIN_LED_GREEN);

    if (debug_mode_flags & DEBUG_MODE_WITH_UART)
        UART_setup(UART_INTERFACE_WISP);

#ifdef CONFIG_ENABLE_I2C_MONITORING
    if (debug_mode_flags & DEBUG_MODE_WITH_I2C)
        I2C_setup();
#endif

    if (debug_mode_flags & DEBUG_MODE_INTERACTIVE)
          main_loop_flags |= FLAG_INTERRUPTED; // main loop notifies the host

    reset_serial_decoder();

    unmask_target_signal(); // listen because target *may* request to exit active debug mode
}

static void finish_exit_debug_mode()
{
    // WISP has shutdown UART and is asleep waiting for int to resume
#if 0 // TODO: this breaks edb after a few printfs, the only danger of not tearing UART down
      // is energy interference due to the pins being high, but hopefully this is negligible
    if (debug_mode_flags & DEBUG_MODE_WITH_UART)
        UART_teardown(UART_INTERFACE_WISP);
#endif

#ifdef CONFIG_ENABLE_I2C_MONITORING
    if (debug_mode_flags & DEBUG_MODE_WITH_I2C)
        I2C_teardown();
#endif

    if (debug_mode_flags & DEBUG_MODE_INTERACTIVE)
        main_loop_flags |= FLAG_EXITED_DEBUG_MODE;

    if (!(debug_mode_flags & DEBUG_MODE_NESTED)) {
        if (!target_powered) {
            continuous_power_off();
            interrupt_context.restored_vcap = discharge_adc(interrupt_context.saved_vcap);
        }
        set_state(STATE_IDLE);
    } else { // nested: go back to the outer debug mode
        debug_mode_flags = interrupt_context.saved_debug_mode_flags;
        set_state(STATE_DEBUG);
    }

    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_GREEN);

    // Give the target enough time to start waiting for our signal
    __delay_cycles(CONFIG_EXIT_DEBUG_MODE_LATENCY_CYCLES);

    signal_target(); // tell target to continue execution
    unmask_target_signal(); // target may request to enter active debug mode
}

/**
 * @brief	Handle an interrupt from the target device
 */
static void handle_target_signal()
{
    switch (state) {
        case STATE_IDLE: // target-initiated request to enter debug mode
            // NOTE: if the debugger/target speedup is very high, then might need a delay here
            // for the target to start listening for the interrupt
            mask_target_signal(); // TODO: incorporate this cleaner, remember that int flag is set
            enter_debug_mode(INTERRUPT_TYPE_TARGET_REQ,
                             DEBUG_MODE_NO_FLAGS /* feature list rcved from target later */);
            break;

        case STATE_ENTERING:
#ifdef CONFIG_SIG_SERIAL_DECODE_PINS
            GPIO(PORT_SERIAL_DECODE, OUT) |= BIT(PIN_SERIAL_DECODE_PULSE);
            GPIO(PORT_SERIAL_DECODE, OUT) &= ~BIT(PIN_SERIAL_DECODE_PULSE);
#endif
            if (sig_serial_bit_index == SIG_SERIAL_NUM_BITS) {
                --sig_serial_bit_index;
                start_serial_decoder();

                // clear the bits we are about to read from target
                debug_mode_flags &= ~DEBUG_MODE_FULL_FEATURES;
                if (!target_powered && !(debug_mode_flags & DEBUG_MODE_NESTED))
                    continuous_power_on();
            } else if (sig_serial_bit_index >= 0) {
                debug_mode_flags |= 1 << sig_serial_bit_index;
            } else { // bitstream over (there is a terminating edge)
                stop_serial_decoder();

                mask_target_signal(); // TODO: incorporate this cleaner, remember that int flag is set
                finish_enter_debug_mode();
            }
            break;

        case STATE_EXITING: // Targed acknowledged our request to exit debug mode
            mask_target_signal(); // TODO: incorporate this cleaner, remember that int flag is set
            finish_exit_debug_mode();
            break;

        case STATE_DEBUG: // Target requested to exit debug mode OR
                          // target hit an assert or bkpt within an energy guard

#ifdef CONFIG_SIG_SERIAL_DECODE_PINS
            GPIO(PORT_SERIAL_DECODE, OUT) |= BIT(PIN_SERIAL_DECODE_PULSE);
            GPIO(PORT_SERIAL_DECODE, OUT) &= ~BIT(PIN_SERIAL_DECODE_PULSE);
#endif

            if (sig_serial_bit_index == SIG_SERIAL_NUM_BITS) {
                --sig_serial_bit_index;
                start_serial_decoder();

                // clear the bits we are about to read from target
                target_sig_cmd = 0;
            } else if (sig_serial_bit_index >= 0) {
                target_sig_cmd |= 1 << sig_serial_bit_index;
            } else { // bitstream over (there is a terminating edge)
                stop_serial_decoder();

                mask_target_signal(); // TODO: incorporate this cleaner (int flag is set)

                switch (target_sig_cmd) {
                    case SIG_CMD_INTERRUPT: // assert/bkpt nested in an energy guard
                        __delay_cycles(NESTED_DEBUG_MODE_INTERRUPT_LATENCY_CYCLES);
                        enter_debug_mode(INTERRUPT_TYPE_TARGET_REQ, DEBUG_MODE_NESTED);
                        break;
                    case SIG_CMD_EXIT: // exit debug mode (both innner and outer gets here)
                        finish_exit_debug_mode();
                        break;
                    default:
                        ASSERT(ASSERT_INVALID_SIG_CMD, false);
                }
            }
            break;

        case STATE_SERIAL_ECHO: // for testing purposes
#ifdef CONFIG_SIG_SERIAL_DECODE_PINS
            GPIO(PORT_SERIAL_DECODE, OUT) |= BIT(PIN_SERIAL_DECODE_PULSE);
            GPIO(PORT_SERIAL_DECODE, OUT) &= ~BIT(PIN_SERIAL_DECODE_PULSE);
#endif
            if (sig_serial_bit_index == SIG_SERIAL_NUM_BITS) {
                sig_serial_bit_index--;
                start_serial_decoder();
            } else if (sig_serial_bit_index >= 0) {
                sig_serial_echo_value |= 1 << sig_serial_bit_index;
            } else {
                stop_serial_decoder();
                set_state(saved_sig_serial_echo_state);
            }
            break;

        default:
            error(ERROR_UNEXPECTED_INTERRUPT);
            break;
    }

	GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG);
}

/**
 * @brief   Set up all pins.  Default to GPIO output low for unused pins.
 */
static inline void pin_setup()
{
    // Set unconnected pins to output low (note: OUT value is undefined on reset)
    P1DIR |= BIT7;
    P1OUT &= ~(BIT7);
    P2DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P2OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3DIR |= BIT0 | BIT1 | BIT2 | BIT5 | BIT6 | BIT7;
    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT5 | BIT6 | BIT7);
#ifdef BOARD_EDB
    P4DIR |= BIT0 | BIT3 | BIT7;
    P4OUT &= ~(BIT0 | BIT3 | BIT7);
    P5DIR |= BIT0 | BIT1 | BIT6;
    P5OUT &= ~(BIT0 | BIT1 | BIT6);
    P6DIR |= BIT0 | BIT6 | BIT7;
    P6OUT &= ~(BIT0 | BIT6 | BIT7);
#endif
    // PJDIR |= <none>

    // Uncomment this if R3 is not populated since in that case pin is unconnected
    // GPIO(PORT_CONT_POWER, DIR) |= BIT(PIN_CONT_POWER);
    // GPIO(PORT_CONT_POWER, OUT) &= ~BIT(PIN_CONT_POWER);

    GPIO(PORT_LED, OUT) &= ~(BIT(PIN_LED_GREEN) | BIT(PIN_LED_RED));
    GPIO(PORT_LED, DIR) |= BIT(PIN_LED_GREEN) | BIT(PIN_LED_RED);

#ifdef CONFIG_SCOPE_TRIGGER_SIGNAL
    GPIO(PORT_TRIGGER, OUT) &= ~BIT(PIN_TRIGGER);
    GPIO(PORT_TRIGGER, DIR) |= BIT(PIN_TRIGGER);
#endif

#ifdef CONFIG_STATE_PINS
    GPIO(PORT_STATE, OUT) &= ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1));
    GPIO(PORT_STATE, DIR) |= BIT(PIN_STATE_0) | BIT(PIN_STATE_1);
#endif

#ifdef CONFIG_EVENT_PINS
    GPIO(PORT_EVENT, OUT) &= ~(BIT(PIN_EVENT_0) | BIT(PIN_EVENT_1));
    GPIO(PORT_EVENT, DIR) |= BIT(PIN_EVENT_0) | BIT(PIN_EVENT_1);
#endif

#ifdef CONFIG_SIG_SERIAL_DECODE_PINS
    GPIO(PORT_SERIAL_DECODE, OUT) &= ~(BIT(PIN_SERIAL_DECODE_PULSE) | BIT(PIN_SERIAL_DECODE_TIMER));
    GPIO(PORT_SERIAL_DECODE, DIR) |= BIT(PIN_SERIAL_DECODE_PULSE) | BIT(PIN_SERIAL_DECODE_TIMER);
#endif

    // By default codepoint pins are output (bkpt mode), the direction
    // is changed if/when watchpoints are enabled
#ifndef CONFIG_RFID_DECODER_STATE_PINS // pin conflict
    GPIO(PORT_CODEPOINT, DIR) |= BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1);
#endif

#ifdef CONFIG_PULL_DOWN_ON_SIG_LINE
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);
    GPIO(PORT_SIG, REN) |= BIT(PIN_SIG);
#endif

    // Configure the output level for continous power pin ahead of time
    GPIO(PORT_CONT_POWER, OUT) |= BIT(PIN_CONT_POWER);

    // Voltage sense pins as ADC channels
    GPIO(PORT_VSENSE, SEL) = GPIO(PORT_VSENSE, SEL)
#ifdef PIN_VCAP
        | BIT(PIN_VCAP)
#endif
#ifdef PIN_VBOOST
        | BIT(PIN_VBOOST)
#endif
#ifdef PIN_VREG
        | BIT(PIN_VREG)
#endif
#ifdef PIN_VRECT
        | BIT(PIN_VRECT)
#endif
#ifdef PIN_VINJ
        | BIT(PIN_VINJ)
#endif
    ;

#ifdef CONFIG_ROUTE_ACLK_TO_PIN
    P1SEL |= BIT0;
    P1DIR |= BIT0;
#endif

    // In our IDLE state, target might request to enter active debug mode
    unmask_target_signal();
}

/**
 * @brief	Interrupt WISP and enter active debug mode when Vcap reaches the given level
 * @param   level   Vcap level to interrupt at
 * @details Implemented by continuously sampling Vcap using the ADC
 */
static void break_at_vcap_level_adc(uint16_t level)
{
    uint16_t cur_vcap, cur_vreg;

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */

    cur_vreg = ADC_read(ADC_CHAN_INDEX_VREG);
    if (cur_vreg < MCU_ON_THRES) { // MCU is off, wait for MCU to turn on
        do {
            cur_vreg = ADC_read(ADC_CHAN_INDEX_VREG);
        } while (cur_vreg < MCU_ON_THRES);
        // TODO: MCU boot delay
        __delay_cycles(35000);
        __delay_cycles(35000);
    } // else: MCU is already on, go on to check Vcap right away

    do {
        cur_vcap = ADC_read(ADC_CHAN_INDEX_VCAP);
    } while (cur_vcap > level);

    enter_debug_mode(INTERRUPT_TYPE_ENERGY_BREAKPOINT, DEBUG_MODE_FULL_FEATURES);
}

/**
 * @brief	Interrupt WISP and enter active debug mode when Vcap reaches the given level
 * @param   level   Vcap level to interrupt at
 * @param   cmp_ref Voltage reference with resepect to which 'target' voltage is calculated
 * @details Implemented by monitoring Vcap using the analog comparator
 */
static void break_at_vcap_level_cmp(uint16_t level, comparator_ref_t ref)
{
    arm_comparator(CMP_OP_ENERGY_BREAKPOINT, level, ref, CMP_EDGE_RISING);
    // expect comparator interrupt
}

/**
 * @brief       Execute a command received from the computer through the USB port
 * @param       pkt     Packet structure that contains the received message info
 */
static void executeUSBCmd(uartPkt_t *pkt)
{
    uint16_t adc12Result;
    uint16_t target_vcap, actual_vcap;
    uint32_t address;
    unsigned len;

#ifdef CONFIG_SCOPE_TRIGGER_SIGNAL
    trigger_scope();
#endif

    switch(pkt->descriptor)
    {
    case USB_CMD_SENSE:
        {
            adc_chan_index_t chan_idx = (adc_chan_index_t)pkt->data[0];
            adc12Result = ADC_read(chan_idx);
            send_voltage(adc12Result);
            break;
        }
#ifdef CONFIG_PWM_CHARGING
    case USB_CMD_SET_VCAP:
        adc12Target = *((uint16_t *)(pkt->data));
        setWispVoltage_block(ADC_CHAN_INDEX_VCAP, adc12Target);
        break;

    case USB_CMD_SET_VBOOST:
        adc12Target = *((uint16_t *)(pkt->data));
        setWispVoltage_block(ADC_CHAN_INDEX_VBOOST, adc12Target);
        break;
#endif

    case USB_CMD_ENTER_ACTIVE_DEBUG:
    	// todo: turn off all logging?
        enter_debug_mode(INTERRUPT_TYPE_DEBUGGER_REQ, DEBUG_MODE_FULL_FEATURES);
        break;

    case USB_CMD_EXIT_ACTIVE_DEBUG:
        exit_debug_mode();
        break;

    case USB_CMD_INTERRUPT:
        interrupt_target();
        break;

    case USB_CMD_GET_WISP_PC:
        target_comm_send_get_pc();
    	while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
    			(wispRxPkt.descriptor != WISP_RSP_ADDRESS)); // wait for response
        forward_msg_to_host(USB_RSP_ADDRESS, wispRxPkt.data, wispRxPkt.length);
    	wispRxPkt.processed = 1;
    	break;

    case USB_CMD_STREAM_BEGIN: {
        uint16_t streams = pkt->data[0];
        unsigned sampling_period = (pkt->data[2] << 8) | pkt->data[1];

        streams_bitmask = streams;
        adc_streams_bitmask = streams & ADC_STREAMS;

#ifdef CONFIG_SYSTICK
        systick_reset(); // to avoid timestamp wrap-around in middle of stream
#endif

#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
        if (streams & STREAM_RF_EVENTS)
            RFID_start_event_stream();
#endif
        if (streams & STREAM_WATCHPOINTS)
            enable_watchpoints();

#ifdef CONFIG_ENABLE_VOLTAGE_STREAM
        // actions common to all adc streams
        if (streams & ADC_STREAMS) {
            main_loop_flags |= FLAG_LOGGING; // for main loop
            ADC_start(adc_streams_bitmask, sampling_period);
        }
#endif
        break;
    }

    case USB_CMD_STREAM_END: {
        unsigned streams = pkt->data[0];

        adc_streams_bitmask &= ~(streams & ADC_STREAMS);
        streams_bitmask = 0;

#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
        if (streams & STREAM_RF_EVENTS)
            RFID_stop_event_stream();
#endif
        if (streams & STREAM_WATCHPOINTS)
            disable_watchpoints();

#ifdef CONFIG_ENABLE_VOLTAGE_STREAM
        // actions common to all adc streams
        if (streams & ADC_STREAMS) {
            ADC_stop();
            main_loop_flags &= ~FLAG_LOGGING; // for main loop
        }
#endif
        break;
    }

    case USB_CMD_SEND_RF_TX_DATA:
		// not implemented
		break;

    case USB_CMD_ENABLE_PORT_INT_TAG_PWR:
    	// not implemented
    	break;

    case USB_CMD_DISABLE_PORT_INT_TAG_PWR:
    	// not implemented
    	break;

#ifdef CONFIG_PWM_CHARGING
    case USB_CMD_PWM_ON:
    	PWM_start();
    	break;
#endif

    case USB_CMD_CHARGE:
        target_vcap = *((uint16_t *)(&pkt->data[0]));
        actual_vcap = charge_adc(target_vcap);
        send_voltage(actual_vcap);
        break;

    case USB_CMD_DISCHARGE:
        target_vcap = *((uint16_t *)(pkt->data));
        actual_vcap = discharge_adc(target_vcap);
        send_voltage(actual_vcap);
        break;

    case USB_CMD_CHARGE_CMP: {
        target_vcap = *((uint16_t *)(pkt->data));
        comparator_ref_t cmp_ref = (comparator_ref_t)pkt->data[2];
        charge_cmp(target_vcap, cmp_ref);
        break;
    }

    case USB_CMD_DISCHARGE_CMP: {
        target_vcap = *((uint16_t *)(pkt->data));
        comparator_ref_t cmp_ref = (comparator_ref_t)pkt->data[2];
        discharge_cmp(target_vcap, cmp_ref);
        break;
    }

    case USB_CMD_RESET_STATE:
        reset_state();
        send_return_code(RETURN_CODE_SUCCESS);
        break;

#ifdef CONFIG_PWM_CHARGING
    case USB_CMD_RELEASE_POWER:
    case USB_CMD_PWM_OFF:
    case USB_CMD_PWM_LOW:
    	PWM_stop();
    	break;

    case USB_CMD_SET_PWM_FREQUENCY:
        PWM_set_freq((*((uint16_t *)(pkt->data))) - 1);
    	break;

    case USB_CMD_SET_PWM_DUTY_CYCLE:
        PWM_set_duty_cycle(*((uint16_t *)(pkt->data)));
    	break;

    case USB_CMD_PWM_HIGH:
    	PWM_stop();
        GPIO(PORT_CHARGE, OUT) |= BIT(PIN_CHARGE); // output high
    	break;
#endif

    // USB_CMD_PWM_LOW and USB_CMD_PWM_OFF do the same thing

    case USB_CMD_MONITOR_MARKER_BEGIN:
    case USB_CMD_MONITOR_MARKER_END:
        // DEPRECATED
        break;

    case USB_CMD_BREAK_AT_VCAP_LEVEL: {
        target_vcap = *((uint16_t *)(&pkt->data[0]));
        energy_breakpoint_impl_t impl = (energy_breakpoint_impl_t)pkt->data[2];
        switch (impl) {
            case ENERGY_BREAKPOINT_IMPL_ADC:
                break_at_vcap_level_adc(target_vcap);
                break;
            case ENERGY_BREAKPOINT_IMPL_CMP: {
                comparator_ref_t cmp_ref = (comparator_ref_t)pkt->data[3];
                break_at_vcap_level_cmp(target_vcap, cmp_ref);
                break;
            }
        }
        break;
    }

    case USB_CMD_READ_MEM:
        address = *((uint32_t *)(&pkt->data[0]));
        len = pkt->data[4];

        target_comm_send_read_mem(address, len);
        while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                (wispRxPkt.descriptor != WISP_RSP_MEMORY)); // wait for response
        forward_msg_to_host(USB_RSP_WISP_MEMORY, wispRxPkt.data, wispRxPkt.length);
        wispRxPkt.processed = 1;
        break;

    case USB_CMD_WRITE_MEM:
    {
        address = *((uint32_t *)(&pkt->data[0]));
        len = pkt->data[4];
        uint8_t *value = &pkt->data[5];

        if (len > WISP_CMD_MAX_LEN - sizeof(uint32_t) - sizeof(uint8_t)) {
            send_return_code(RETURN_CODE_BUFFER_TOO_SMALL);
            break;
        }

        target_comm_send_write_mem(address, value, len);
        while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                (wispRxPkt.descriptor != WISP_RSP_MEMORY)); // wait for response
        wispRxPkt.processed = 1;

        send_return_code(RETURN_CODE_SUCCESS); // TODO: have WISP return a code
        break;
    }

    case USB_CMD_CONT_POWER:
    {
        bool power_on = (bool)pkt->data[0];
        if (power_on) {
            continuous_power_on();
            target_powered = true;
        } else {
            continuous_power_off();
            target_powered = false;
        }
        send_return_code(RETURN_CODE_SUCCESS);
        break;
    }

    case USB_CMD_BREAKPOINT:
    {
        breakpoint_type_t type = (breakpoint_type_t)pkt->data[0];
        unsigned index = (uint8_t)pkt->data[1];
        uint16_t energy_level = *(uint16_t *)(&pkt->data[2]);
        comparator_ref_t cmp_ref = (comparator_ref_t)pkt->data[4];
        bool enable = (bool)pkt->data[5];
        toggle_breakpoint(type, index, energy_level, cmp_ref, enable);
        break;
    }

    case USB_CMD_WATCHPOINT:
    {
        unsigned index = pkt->data[0];
        bool enable = (bool)(pkt->data[1] & (1 << 0));
        bool vcap_snapshot = (bool)(pkt->data[1] & (1 << 1));
        toggle_watchpoint(index, enable, vcap_snapshot);
        break;
    }

    case USB_CMD_GET_INTERRUPT_CONTEXT: {
        interrupt_context_t target_int_context;
        interrupt_source_t source = (interrupt_source_t)pkt->data[0];

        switch (source) {
            case INTERRUPT_SOURCE_DEBUGGER:
                send_interrupt_context(&interrupt_context);
                break;
            case INTERRUPT_SOURCE_TARGET:
                get_target_interrupt_context(&target_int_context);
                send_interrupt_context(&target_int_context);
                break;
            default:
                send_return_code(RETURN_CODE_INVALID_ARGS);
                break;
        }
        break;
    }

    case USB_CMD_SERIAL_ECHO: {
        unsigned value = pkt->data[0];

        saved_sig_serial_echo_state = state;
        set_state(STATE_SERIAL_ECHO);

        reset_serial_decoder();

        sig_serial_echo_value = 0;
        unmask_target_signal();

        target_comm_send_echo(value);

        // Wait while the ISRs decode the serial bit stream
        volatile uint16_t timeout = 0xffff;
        while (state == STATE_SERIAL_ECHO && --timeout > 0);
        if (state == STATE_SERIAL_ECHO) // timeout
            set_state(saved_sig_serial_echo_state);

        while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                (wispRxPkt.descriptor != WISP_RSP_SERIAL_ECHO)); // wait for response
        wispRxPkt.processed = 1;

        send_echo(value);
        break;
    }

    case USB_CMD_DMA_ECHO: {
        unsigned value = pkt->data[0];
        send_echo(value);
        break;
    }

    case USB_CMD_ENABLE_TARGET_UART: {
        bool enable = pkt->data[0];
        if (enable) {
            UART_setup(UART_INTERFACE_WISP);
        } else {
            UART_teardown(UART_INTERFACE_WISP);
        }
        send_return_code(RETURN_CODE_SUCCESS);
        break;
    }

    case USB_CMD_SET_PARAM: {
        param_t param = (pkt->data[1] << 8) | pkt->data[0];
        set_param(param, &pkt->data[2]);
        send_param(param);
        break;
    }

    case USB_CMD_GET_PARAM: {
        param_t param = pkt->data[0];
        send_param(param);
        break;
    }

    case USB_CMD_PERIODIC_PAYLOAD: {
        bool enable = pkt->data[0];
        if (enable) {
            payload_start_send_timer();
        } else {
            payload_stop_send_timer();
        }
        send_return_code(RETURN_CODE_SUCCESS);
        break;
    }

    default:
        break;
    }

    pkt->processed = 1;
}


int main(void)
{
    uint32_t count = 0;

    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

    pin_setup();

    GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);

    clock_setup(); // set up unified clock system

#ifdef CONFIG_CLOCK_TEST_MODE
    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);
    BLINK_LOOP(PIN_LED_GREEN, 1000000); // to check clock configuration
#endif

#ifdef CONFIG_PWM_CHARGING
    PWM_setup(1024-1, 512); // dummy default values
#endif

    UART_setup(UART_INTERFACE_USB); // USCI_A0 UART

#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
    RFID_init();
#endif

#ifdef CONFIG_ENABLE_WATCHPOINT_STREAM
    init_watchpoint_event_bufs();
#endif

    payload_init();

#ifdef CONFIG_RESET_STATE_ON_BOOT
    arm_comparator(CMP_OP_RESET_STATE_ON_BOOT, MCU_ON_THRES,
                   CMP_REF_VREF_2_5, CMP_EDGE_RISING);
#endif

    reset_state();

#ifdef CONFIG_SYSTICK
    systick_start();
#endif

    __enable_interrupt();                   // enable all interrupts

    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);

    while(1) {

        if (main_loop_flags & FLAG_INTERRUPTED) {
            main_loop_flags &= ~FLAG_INTERRUPTED;

            if (interrupt_context.type == INTERRUPT_TYPE_TARGET_REQ &&
                debug_mode_flags & DEBUG_MODE_WITH_UART)
                get_target_interrupt_context(&interrupt_context);
            // do it here: reply marks completion of enter sequence
            send_interrupt_context(&interrupt_context);
        }

        if (main_loop_flags & FLAG_WATCHPOINT_READY) {
            main_loop_flags &= ~FLAG_WATCHPOINT_READY;
            send_watchpoint_events();
        }

        if (main_loop_flags & FLAG_EXITED_DEBUG_MODE) {
            main_loop_flags &= ~FLAG_EXITED_DEBUG_MODE;

            send_voltage(interrupt_context.restored_vcap);
        }

        if((main_loop_flags & FLAG_ADC_COMPLETE) && (main_loop_flags & FLAG_LOGGING)) {
            // ADC12 has completed conversion on all active channels
            ADC_send_samples_to_host();
            main_loop_flags &= ~FLAG_ADC_COMPLETE;
        }

        if (main_loop_flags & FLAG_CHARGER_COMPLETE) { // comparator triggered after charge/discharge op
            main_loop_flags &= ~FLAG_CHARGER_COMPLETE;
            send_return_code(RETURN_CODE_SUCCESS);
        }

        if(main_loop_flags & FLAG_UART_USB_RX) {
            // we've received a byte from USB
            if(UART_buildRxPkt(UART_INTERFACE_USB, &usbRxPkt) == 0) {
                // packet is complete
                executeUSBCmd(&usbRxPkt);
            }

            // check if we're done for now
            UART_DISABLE_USB_RX; // disable interrupt so new bytes don't come in
            if(UART_RxBufEmpty(UART_INTERFACE_USB)) {
                main_loop_flags &= ~FLAG_UART_USB_RX; // clear USB Rx flag
            }
            UART_ENABLE_USB_RX; // enable interrupt
        }

/*
        if(main_loop_flags & FLAG_UART_USB_TX) {
            // USB UART Tx byte
            main_loop_flags &= ~FLAG_UART_USB_TX;
        }
*/

        if(main_loop_flags & FLAG_UART_WISP_RX) {
            // we've received a byte over UART from the WISP
            if(UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) == 0) {
                switch (wispRxPkt.descriptor) {
                    case WISP_RSP_STDIO:
                        forward_msg_to_host(USB_RSP_STDIO, wispRxPkt.data, wispRxPkt.length);
                        break;
                    case WISP_RSP_APP_OUTPUT:
                        payload_record_app_output(wispRxPkt.data, wispRxPkt.length);
                        break;
                }
            	wispRxPkt.processed = 1;
            }

            if(UART_RxBufEmpty(UART_INTERFACE_WISP)) {
            	main_loop_flags &= ~FLAG_UART_WISP_RX; // clear WISP Rx flag
            }
        }

        if (main_loop_flags & FLAG_SEND_PAYLOAD) {
            payload_send();
            main_loop_flags &= ~FLAG_SEND_PAYLOAD;
        }

/*
        if(main_loop_flags & FLAG_UART_WISP_TX) {
            // WISP UART Tx byte
            main_loop_flags &= ~FLAG_UART_WISP_TX;
        }
*/

#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
        if(main_loop_flags & FLAG_RF_DATA) {
        	main_loop_flags &= ~FLAG_RF_DATA;
            RFID_send_rf_events_to_host();
        }
#endif

        // This LED toggle is unnecessary, and probably a huge waste of processing time.
        // The LED blinking will slow down when the monitor is performing more tasks.
        if (++count == 0xffff) {
            if (state == STATE_IDLE)
                GPIO(PORT_LED, OUT) ^= BIT(PIN_LED_GREEN);
            count = 0;
        }

    }
}

#if PORT_RF == 1 || PORT_SIG == 1 || PORT_CODEPOINT == 1

// Port 1 ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t pin_state = GPIO(PORT_CODEPOINT, IN); // snapshot

    // TODO: clear interrupt for *all* codepoint pins (not just the one that we
    // are handling, since each pin is a bit of the encoded index)

	switch(__even_in_range(P1IV, 16))
	{
#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
    case INTFLAG(PORT_RF, PIN_RF_TX):
#ifdef CONFIG_ENABLE_RF_TX_DECODING
        rfid_decoder_tx_pin_isr();
#endif // CONFIG_ENABLE_RF_TX_DECODING
        GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);
        break;
#endif // CONFIG_ENABLE_RF_PROTOCOL_MONITORING

#if PORT_SIG == 1
	case INTFLAG(PORT_SIG, PIN_SIG):
		handle_target_signal();
		break;
#endif // PORT_SIG

#if PORT_CODEPOINT == 1
    case INTFLAG(PORT_CODEPOINT, PIN_CODEPOINT_0):
    case INTFLAG(PORT_CODEPOINT, PIN_CODEPOINT_1):
    {
#ifdef BOARD_EDB
        /* Workaround the hardware routing that routes AUX1,AUX2 to pins out of order */
        pin_state = (pin_state & BIT(PIN_CODEPOINT_0) ? BIT(PIN_CODEPOINT_1) : 0) |
                    (pin_state & BIT(PIN_CODEPOINT_1) ? BIT(PIN_CODEPOINT_0) : 0);
#endif // BOARD_EDB
        handle_codepoint(pin_state);
        break;
    }
#endif // PORT_CODEPOINT

	default:
		break;
	}
}

#endif // Port 1 ISR users

#if PORT_SIG == 2

// Port 2 ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(P2IV, 16))
    {
#if PORT_SIG == 2
        case INTFLAG(PORT_SIG, PIN_SIG):
            handle_target_signal();
            break;
#endif // PORT_SIG

        default:
            break;
    }
}
#endif // Port 2 ISR users

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=COMP_B_VECTOR
__interrupt void Comp_B_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(COMP_B_VECTOR))) Comp_B_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (comparator_op) {
        case CMP_OP_CHARGE:
            GPIO(PORT_CHARGE, OUT) &= ~BIT(PIN_CHARGE); // cut the power supply
            main_loop_flags |= FLAG_CHARGER_COMPLETE;
            comparator_op = CMP_OP_NONE;
            CBINT &= ~(CBIFG | CBIE);   // clear Interrupt flag and disable interrupt
            break;
        case CMP_OP_DISCHARGE:
            GPIO(PORT_DISCHARGE, DIR) &= ~BIT(PIN_DISCHARGE); // close the discharge "valve"
            main_loop_flags |= FLAG_CHARGER_COMPLETE;
            comparator_op = CMP_OP_NONE;
            CBINT &= ~(CBIFG | CBIE);   // clear Interrupt flag and disable interrupt
            break;
        case CMP_OP_ENERGY_BREAKPOINT:
            enter_debug_mode(INTERRUPT_TYPE_ENERGY_BREAKPOINT, DEBUG_MODE_FULL_FEATURES);
            // TODO: should the interrupt be re-enabled upon exit from debug mode?
            comparator_op = CMP_OP_NONE;
            CBINT &= ~(CBIFG | CBIE);   // clear Interrupt flag and disable interrupt
            break;
        case CMP_OP_CODE_ENERGY_BREAKPOINT:
            if (!code_energy_breakpoints)
                error(ERROR_UNEXPECTED_INTERRUPT);

            // comparator output high means Vcap < cmp ref (activate breakpoint)
            set_external_breakpoint_pin_state(code_energy_breakpoints, CBCTL1 & CBOUT);

            CBCTL1 ^= CBIES; // reverse the edge direction of the interrupt
            CBINT &= ~CBIFG; // clear the flag, leave interrupt enabled
            break;
        case CMP_OP_RESET_STATE_ON_BOOT:
            reset_state();
            CBINT &= ~CBIFG;   // clear Interrupt flag, leave interrupt enabled
            break;
        default:
            error(ERROR_UNEXPECTED_INTERRUPT);
            break;
    }
}

#ifdef CONFIG_ENABLE_DEBUG_MODE
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    // Timed out: at zero we are still waiting for terminator pulse, but if it
    // never comes in one bit slot, then we get here with negative index.
    if (sig_serial_bit_index < 0) {
        reset_state();
    } else {
        --sig_serial_bit_index;
    }

#ifdef CONFIG_SIG_SERIAL_DECODE_PINS
    GPIO(PORT_SERIAL_DECODE, OUT) |= BIT(PIN_SERIAL_DECODE_TIMER);
    GPIO(PORT_SERIAL_DECODE, OUT) &= ~BIT(PIN_SERIAL_DECODE_TIMER);
#endif
    TIMER_CC(TIMER_SIG_SERIAL_DECODE, TMRCC_SIG_SERIAL, CCTL) &= ~CCIFG;
}
#endif // CONFIG_ENABLE_DEBUG_MODE

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=UNMI_VECTOR
__interrupt void unmi_isr(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(UNMI_VECTOR))) unmi_isr(void)
#else
#error Compiler not supported!
#endif
{
    // We do not use ASSERT here because oon oscillator fault, the
    // clock frequency is not the same as nominal, so the assert id
    // encoding onto blink rate would not be correct.
    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_GREEN);
    GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
    if (UCSCTL7 & XT2OFFG)
        GPIO(PORT_LED, OUT) |= BIT(PIN_LED_GREEN);

    while (1);
}

#ifdef DMA_HOST_UART_TX
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(DMA_VECTOR))) DMA_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(DMAIV, 16)) {
#ifdef DMA_HOST_UART_TX
        case DMA_INTFLAG(DMA_HOST_UART_TX):
               host_uart_status &= ~UART_STATUS_TX_BUSY;
            break;
#endif
    }
}
#endif // DMA users
