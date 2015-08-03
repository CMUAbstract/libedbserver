/**
 * @file
 * @author  Graham Harvey
 * @date    13 May 2015
 * @brief   Main function for the WISP monitor.
 * @details The MSP430 on the WISP monitor has UART interrupts enabled to interface
 *          to a computer through USB.  The main loop checks flags that are set in
 *          the interrupt service routines.  This allows execution to continue
 *          without blocking to wait for peripherals.  The 12-bit ADC in use has
 *          several channels that allow four different voltages to be sampled on
 *          the WISP. These are named Vcap, Vboost, Vreg, and Vrect.  The monitor
 *          can get a single sample or log repeated samples of any of those.  It
 *          can also inject power to keep one of those voltages at a level defined
 *          by the user.  However, current is always injected on Vcap.
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <libdebug/target_comm.h>

#include "pmm.h" // must be before pin_assign.h since the latter undefs
#include "pin_assign.h"
#include "host_comm.h"
#include "adc12.h"
#include "uart.h"
#include "i2c.h"
#include "pwm.h"
#include "timeLog.h"
#include "rfid.h"
#include "rfid_decoder.h"
#include "minmax.h"
#include "main.h"
#include "config.h"
#include "error.h"
#include "main_loop.h"


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

#define STREAM_REPLY_MAX_LEN (1 /* num chans */ + ADC12_MAX_CHANNELS * sizeof(uint16_t))
#define HOST_MSG_BUF_SIZE     32 // buffer for UART messages (to host) for main loop
#define TARGET_MSG_BUF_SIZE   16 // buffer for UART messages (to target) for main loop

#if TARGET_MSG_BUF_SIZE < WISP_CMD_MAX_LEN
#error Buffer for UART messages to target is too small: TARGET_MSG_BUF_SIZE < WISP_CMD_MAX_LEN
#endif

// See libdebug/debug.h for description
#define NUM_CODEPOINT_VALUES     ((1 << NUM_CODEPOINT_PINS) - 1)
#define MAX_PASSIVE_BREAKPOINTS  NUM_CODEPOINT_VALUES
#define MAX_INTERNAL_BREAKPOINTS (sizeof(uint16_t) * 8) // _debug_breakpoints_enable in libdebug
#define MAX_EXTERNAL_BREAKPOINTS NUM_CODEPOINT_PINS

#define MAX_WATCHPOINTS          NUM_CODEPOINT_VALUES

#define COMP_NEG_CHAN_INNER(idx) CBIMSEL_ ## idx
#define COMP_NEG_CHAN(idx) COMP_NEG_CHAN_INNER(idx)

#define COMP_PORT_DIS_INNER(idx) CBPD ## idx
#define COMP_PORT_DIS(idx) COMP_PORT_DIS_INNER(idx)

#define DCO_FREQ_RANGE_BITS_INNER(r) DCORSEL_ ## r;
#define DCO_FREQ_RANGE_BITS(r) DCO_FREQ_RANGE_BITS_INNER(r)

#define FLL_D_BITS_INNER(d) FLLD_ ## d
#define FLL_D_BITS(d) FLL_D_BITS_INNER(d)

#define FLL_REF_DIV_BITS_INNER(div) FLLREFDIV__ ## div
#define FLL_REF_DIV_BITS(div) FLL_REF_DIV_BITS_INNER(div)

#define CLK_DIV_BITS_INNER(clk, div) DIV ## clk ## __ ## div
#define CLK_DIV_BITS(clk, div) CLK_DIV_BITS_INNER(clk, div)

/**
 * Debugger state machine states
 */
typedef enum {
    STATE_IDLE = 0,
    STATE_ENTERING,
    STATE_DEBUG,
    STATE_EXITING,
    STATE_SERIAL_ECHO, // for testing purposes only
} state_t;

/**
 * @brief State of async charge/discharge operation
 */
typedef enum {
    CMP_OP_NONE = 0,
    CMP_OP_CHARGE,
    CMP_OP_DISCHARGE,
    CMP_OP_ENERGY_BREAKPOINT,
    CMP_OP_CODE_ENERGY_BREAKPOINT,
} comparator_op_t;

typedef enum {
    CMP_EDGE_ANY,
    CMP_EDGE_FALLING,
    CMP_EDGE_RISING,
} comparator_edge_t;

typedef struct {
    interrupt_type_t type;
    unsigned id;
    uint16_t saved_vcap;
    uint16_t restored_vcap;
    uint16_t saved_debug_mode_flags; // for nested debug mode
} interrupt_context_t;

typedef struct {
    unsigned timestamp;
    unsigned index;
    unsigned vcap;
} watchpoint_event_t;

volatile uint16_t main_loop_flags = 0; // bit mask containing bit flags to check in the main loop

static state_t state = STATE_IDLE;
static comparator_op_t comparator_op = CMP_OP_NONE;
static uint16_t debug_mode_flags = 0; // TODO: set these by decoding serial bits on signal line
static int sig_serial_bit_index; // debug mode flags are serially encoded on the signal line

static unsigned sig_serial_echo_value = 0;
static state_t saved_sig_serial_echo_state;

static sig_cmd_t target_sig_cmd;

static interrupt_context_t interrupt_context;

static uint16_t adc_streams_bitmask; // streams from ADC currently streaming
static uint16_t streams_bitmask; // currently enabled streams

static uartPkt_t usbRxPkt = { .processed = 1 };
static uartPkt_t wispRxPkt = { .processed = 1 };

/**
 * @brief Message payload pointer in a buffer for messages to host
 * @details This buffer is used exclusively by main loop, so it is
 *          shared only in the sense of being multi-plexed in time, i.e. it is
 *          never used concurrently but to threads of control.
 */
static uint8_t host_msg_buf[HOST_MSG_BUF_SIZE];
static uint8_t * const host_msg_payload = &host_msg_buf[UART_MSG_HEADER_SIZE];

/**
 * @brief Message payload pointer in a buffer used exclusively by main loop
 * @details This buffer is used exclusively by main loop, so it is
 *          shared only in the sense of being multi-plexed in time, i.e. it is
 *          never used concurrently but to threads of control.
 */
static uint8_t target_msg_buf[TARGET_MSG_BUF_SIZE];
static uint8_t * const target_msg_payload = &target_msg_buf[UART_MSG_HEADER_SIZE];

// Bitmasks indicate whether a breakpoint (group) of given index is enabled
static uint16_t passive_breakpoints = 0;
static uint16_t external_breakpoints = 0;
static uint16_t internal_breakpoints = 0;
static uint16_t code_energy_breakpoints = 0;

static uint16_t watchpoints = 0;

// TODO:
#define NUM_WATCHPOINT_BUFFERS 2
#define NUM_WATCHPOINT_EVENTS_BUFFERED 16

#define WATCHPOINT_EVENT_BUF_HEADER_SPACE 1 // units of sizeof(watchpoint_event_t)
#define WATCHPOINT_EVENT_BUF_PAYLOAD_SPACE NUM_WATCHPOINT_EVENTS_BUFFERED // units of sizeof(watchpoint_event_t)
#define WATCHPOINT_EVENT_BUF_SIZE \
    (WATCHPOINT_EVENT_BUF_HEADER_SPACE + WATCHPOINT_EVENT_BUF_PAYLOAD_SPACE) // units of sizeof(watchpoint_event_t)

#if STREAM_DATA_MSG_HEADER > WATCHPOINT_EVENT_BUF_HEADER_SPACE
#error Not enough space for header in event buf: add more units of sizeof(struct watchpoint_event_t)
#endif

#define WATCHPOINT_EVENT_BUF_HEADER_OFFSET \
    (WATCHPOINT_EVENT_BUF_HEADER_SPACE * sizeof(watchpoint_event_t) - \
        (UART_MSG_HEADER_SIZE + STREAM_DATA_MSG_HEADER_LEN))

static watchpoint_event_t
watchpoint_events_msg_bufs[NUM_WATCHPOINT_BUFFERS][WATCHPOINT_EVENT_BUF_SIZE];

static watchpoint_event_t *watchpoint_events_bufs[NUM_WATCHPOINT_BUFFERS] = {
    &watchpoint_events_msg_bufs[0][WATCHPOINT_EVENT_BUF_HEADER_SPACE],
    &watchpoint_events_msg_bufs[1][WATCHPOINT_EVENT_BUF_HEADER_SPACE]
};

static unsigned watchpoint_events_count[NUM_WATCHPOINT_BUFFERS];
static watchpoint_event_t *watchpoint_events_buf;
static unsigned watchpoint_events_buf_idx;

static adc12_t adc12 = {
    .config = {
        .channel_masks = { // map permanent software indexes to hardware ADC channels
            ADC_CHAN_VCAP,
            ADC_CHAN_VBOOST,
            ADC_CHAN_VREG,
            ADC_CHAN_VRECT,
            ADC_CHAN_VINJ,
        },
        .num_channels = 0, // maintained at runtime
        .sampling_period = CONFIG_ADC_SAMPLING_PERIOD,
    },
};

static inline void clock_setup()
{
    unsigned rc;

    // Increase Vcore setting to level3 to support high frequency
    rc = PMM_setVCore(0x03);
    ASSERT(ASSERT_SET_CORE_VOLTAGE_FAILED, rc == STATUS_SUCCESS);

#if defined(CONFIG_STARTUP_VOLTAGE_WORKAROUND_DISABLE_PMM)
    PMMCTL0_H = PMMPW_H;
    SVSMHCTL &= ~(SVSHE | SVMHE);
    SVSMLCTL &= ~(SVSLE | SVMLE);
    PMMRIE &= ~(SVMHVLRPE | SVSHPE | SVMLVLRPE | SVSLPE);
    PMMCTL0_H = 0x00;
#elif defined(CONFIG_STARTUP_VOLTAGE_WORKAROUND_DELAY)
    __delay_cycles(65535);
    __delay_cycles(65535);
    __delay_cycles(65535);
    __delay_cycles(65535);

    __delay_cycles(65535);
    __delay_cycles(65535);
    __delay_cycles(65535);
    __delay_cycles(65535);
#else
#error No workaround for startup voltage problem: see CONFIG_STARTUP_VOLTAGE_WORKAROUND_*
#endif

    // NOTE: The MCU starts in a fault condition, because ACLK is set to XT1 LF but
    // XT1 LF takes time to initialize. Its init begins when XT1 pin function
    // is selected. The fault flag for this clock source (and for DCO which
    // depends on it) and the "wildcard" osc fault flag OFIFG are set
    // and cannot be cleared until the init is complete (they bounce back on
    // if cleared before the init is completed).

    SFRIE1 &= OFIE; // ignore oscillator faults while we enable the oscillators

    // Go through each oscillator (REFO, XT1, XT2, DCO) and init each if necessary
    // and choose it as the source for the requested clocks

    // Oscillator: REFO

#if defined(CONFIG_DCO_REF_SOURCE_REFO)
    // already initialized on reset
    UCSCTL3 |= SELREF__REFOCLK;                  // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA__REFOCLK;                   // Set ACLK = REFO
    UCSCTL7 &= ~(XT1LFOFFG | XT2OFFG | DCOFFG); // Errata #UCS11
#endif // CONFIG_DCO_REF_CLOCK_REFO

    // Oscillator: XT1 crystal

    // Need XT1 for both XT1 and XT2 DCO ref configs since ACLK is sourced from XT1
#if defined(CONFIG_DCO_REF_SOURCE_XT1) || defined(CONFIG_DCO_REF_SOURCE_XT2)
    // Enable XT1 by configuring its pins
    UCSCTL6 &= ~(XCAP1 | XCAP0);
    UCSCTL6 |= CONFIG_XT1_CAP_BITS;
    P5SEL |= BIT4 | BIT5;

    // The following are already the default, but include for clarity
    UCSCTL3 |= SELREF__XT1CLK; // select XT1 as the DCO reference
    UCSCTL4 |= SELA__XT1CLK;  // select ST1 as the source for ACLK
    UCSCTL7 &= ~(XT1LFOFFG | XT2OFFG | DCOFFG); // Errata #UCS11

    // wait for XT1 to init and clear the fault flags
    while (UCSCTL7 & XT1LFOFFG)
        UCSCTL7 &= ~XT1LFOFFG;

#else
    // Disable XT1 since it is unused (and we changed the DCO ref and ACLK source above)
    UCSCTL6 |= XT1OFF;
    UCSCTL7 &= ~XT1LFOFFG; // at reset XT1 was selected and faulted (see note at the top)
#endif

    // Oscillator: XT2 crystal

#if defined(CONFIG_DCO_REF_SOURCE_XT2) || defined(CONFIG_CLOCK_SOURCE_XT2)
    // First part of enabling XT2: configure its pins (nothing happens yet)
    P5SEL |= BIT2 | BIT3;

    // Second part of enabling XT2: select it as a source
#if defined(CONFIG_DCO_REF_SOURCE_XT2)
    UCSCTL3 |= SELREF__XT2CLK; // TODO: UNTESTED
#endif
#if defined(CONFIG_CLOCK_SOURCE_XT2)
    // switch master clock (CPU) to XT2 (25 MHz) and clear fault flags
    UCSCTL4 |= SELM__XT2CLK | SELS__XT2CLK | SELA__XT2CLK;
    UCSCTL7 &= ~(XT1LFOFFG | XT2OFFG | DCOFFG); // Errata #UCS11

    // Can't drive the UART with a 25 MHz clock (hang/reset), divide it
    //UCSCTL5 |= DIVS0 | DIVA0; // SMCLK, ACLK = 25 MHz / 2 = 12.5 MHz
#endif

    // Wait for the crystal to initialize by watching for fault flag to go away
    while (UCSCTL7 & XT2OFFG)
        UCSCTL7 &= ~XT2OFFG;
    SFRIFG1 &= ~OFIFG; // clear wildcard fault flag

#endif // CONFIG_DCO_REF_CLOCK_XT2 || CONFIG_CLOCK_SOURCE_XT2

    // Oscillator: DCO

    // DCO is on by default, we change its frequency if requested by config

#if defined(CONFIG_CLOCK_SOURCE_DCO)
    __bis_SR_register(SCG0);                    // Disable the FLL control loop
    UCSCTL3 |= FLL_REF_DIV_BITS(CONFIG_DCO_REF_CLOCK_DIV);
    UCSCTL0 = 0x0000;                           // Set lowest possible DCOx, MODx
    UCSCTL1 = DCO_FREQ_RANGE_BITS(CONFIG_DCO_FREQ_R);    // Select DCO freq range
    UCSCTL2 = FLL_D_BITS(CONFIG_DCO_FREQ_D) | CONFIG_DCO_FREQ_N;
    UCSCTL7 &= ~(XT1LFOFFG | XT2OFFG | DCOFFG); // Errata #UCS11

    __bic_SR_register(SCG0);                    // Enable the FLL control loop

    __delay_cycles(DCO_SETTLING_TIME);
#endif

    // Wait for DCO to stabilize (DCO on by default and we leave it on, so always do this)
    while (UCSCTL7 & DCOFFG)
        UCSCTL7 &= ~DCOFFG;

    // End sequence of oscillators

    SFRIFG1 &= ~OFIFG; // clear wildcard fault flag
    SFRIE1 |= OFIE; // watch for oscillator faults

    UCSCTL5 |= CLK_DIV_BITS(A, CONFIG_CLK_DIV_ACLK) |
               CLK_DIV_BITS(S, CONFIG_CLK_DIV_SMCLK) |
               CLK_DIV_BITS(M, CONFIG_CLK_DIV_MCLK);
}



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

// Uses the main loop host_msg_buf
static inline void send_msg_to_host(unsigned descriptor, unsigned payload_len)
{
    // Out-of-bound writes already happen before we get here, but the payload
    // len should be in a register and this is not a function call (inline), so
    // this check should be robust even if memory got a little corrupted.
    ASSERT(ASSERT_HOST_MSG_BUF_OVERFLOW, payload_len <= HOST_MSG_BUF_SIZE - UART_MSG_HEADER_SIZE);

    UART_send_msg_to_host(descriptor, payload_len, host_msg_buf);
    UART_end_transmission();
}

static void send_voltage(uint16_t voltage)
{
    unsigned payload_len = 0;

    UART_begin_transmission();

    host_msg_payload[payload_len++] = voltage & 0xFF;
    host_msg_payload[payload_len++] = (voltage >> 8) & 0xFF;

    send_msg_to_host(USB_RSP_VOLTAGE, payload_len);
}

static void send_return_code(unsigned code)
{
    unsigned payload_len = 0;
    UART_begin_transmission();
    host_msg_payload[payload_len++] = code;
    send_msg_to_host(USB_RSP_RETURN_CODE, payload_len);
}

static void send_interrupt_context(interrupt_context_t *int_context)
{
    unsigned payload_len = 0;

    UART_begin_transmission();

    host_msg_payload[payload_len++] = int_context->type;
    host_msg_payload[payload_len++] = int_context->id;
    host_msg_payload[payload_len++] = int_context->id >> 8;
    host_msg_payload[payload_len++] = (int_context->saved_vcap >> 0) & 0xff;
    host_msg_payload[payload_len++] = (int_context->saved_vcap >> 8) & 0xff;

    send_msg_to_host(USB_RSP_INTERRUPTED, payload_len);
}

static void forward_msg_to_host(unsigned descriptor, uint8_t *buf, unsigned len)
{
    unsigned payload_len = 0;

    UART_begin_transmission();

    while (len--) {
        host_msg_payload[payload_len] = buf[payload_len];
        payload_len++;
    }

    send_msg_to_host(descriptor, payload_len);
}

static void enter_debug_mode(interrupt_type_t int_type, unsigned flags)
{
    interrupt_context.type = int_type;
    interrupt_context.id = 0;

    set_state(STATE_ENTERING);

    if (!(flags & DEBUG_MODE_NESTED)) {
        interrupt_context.saved_vcap = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);
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
    UART_send_msg_to_target(WISP_CMD_EXIT_ACTIVE_DEBUG, 0, target_msg_buf);
}

static void reset_state()
{
    continuous_power_off();
    stop_serial_decoder();
    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_GREEN);
    set_state(STATE_IDLE);
    unmask_target_signal();
}

static void arm_comparator(comparator_op_t op, uint16_t target, comparator_ref_t ref,
                           comparator_edge_t edge)
{
    comparator_op = op;

    // ref0 = ref1 = target = Vref / 2^32 * target_volts
    switch (ref) {
        case CMP_REF_VCC:
            CBCTL2 = CBRS_1;
            break;
        case CMP_REF_VREF_2_5:
            CBCTL2 = CBREFL_3 | CBRS_2; // Vref = 2.5V and Vref to resistor ladder
            break;
        case CMP_REF_VREF_2_0:
            CBCTL2 = CBREFL_2 | CBRS_2; // Vref = 2.0V and Vref to resistor ladder
            break;
        case CMP_REF_VREF_1_5:
            CBCTL2 = CBREFL_1 | CBRS_2; // Vref = 1.5V and Vref to resistor ladder
            break;
        default:
            error(ERROR_INVALID_VALUE);
            break;
    }
    CBCTL2 |= (target << 8) | target;

    CBCTL0 |= CBIMEN | COMP_NEG_CHAN(COMP_CHAN_VCAP); // route input pin to V-, input channel (pin)
    CBCTL3 |= COMP_PORT_DIS(COMP_CHAN_VCAP); // disable input port buffer on pin
    CBCTL1 |= CBF | CBFDLY_3;

    CBINT &= ~CBIE; // disable CompB interrupt

    CBCTL1 |= CBON;               // turn on ComparatorB

    switch (edge) {
        case CMP_EDGE_FALLING:
            CBCTL1 |= CBIES;
            break;
        case CMP_EDGE_RISING:
            CBCTL1 &= ~CBIES;
            break;
        case CMP_EDGE_ANY: // determine direction that would change current output
            if (CBCTL1 & CBOUT)
                CBCTL1 |= CBIES;
            else
                CBCTL1 &= ~CBIES;
            break;
        default:
            error(ERROR_INVALID_VALUE);
            break;
    }

    CBINT &= ~(CBIFG | CBIIFG);   // clear any errant interrupts
    CBINT |= CBIE;                // enable CompB interrupt
}

static void disarm_comparator()
{
    CBINT &= ~CBIE; // disable interrupt
    CBCTL1 &= ~CBON; // turn off
}

static void init_watchpoint_event_bufs()
{
    unsigned i, offset;
    uint8_t *header;

    for (i = 0; i < NUM_WATCHPOINT_BUFFERS; ++i) {
        watchpoint_events_count[i] = 0;

        header = (uint8_t *)&watchpoint_events_msg_bufs[i][0] +
                    UART_MSG_HEADER_SIZE + WATCHPOINT_EVENT_BUF_HEADER_OFFSET;
        offset = 0;
        header[offset++] = STREAM_WATCHPOINTS;
        header[offset++] = 0; // padding
        ASSERT(ASSERT_INVALID_STREAM_BUF_HEADER, offset == STREAM_DATA_MSG_HEADER_LEN);
    }
    watchpoint_events_buf = watchpoint_events_bufs[0];
}

static inline void append_watchpoint_event(unsigned index)
{
    watchpoint_event_t *watchpoint_event =
        &watchpoint_events_buf[watchpoint_events_count[watchpoint_events_buf_idx]++];

    watchpoint_event->timestamp = TIMELOG_CURRENT_TIME;
    watchpoint_event->index = index;
    watchpoint_event->vcap = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);

    if (watchpoint_events_count[watchpoint_events_buf_idx] == NUM_WATCHPOINT_EVENTS_BUFFERED) {
        // swap to the other buffer in the double-buffer pair
        watchpoint_events_buf_idx ^= 1;
        watchpoint_events_buf = watchpoint_events_bufs[watchpoint_events_buf_idx];
        main_loop_flags |= FLAG_WATCHPOINT_READY;
    }
}

static void send_watchpoint_events()
{
    unsigned ready_events_count;
    unsigned ready_events_buf_idx = watchpoint_events_buf_idx ^ 1; // the other one in the pair

    ready_events_count = watchpoint_events_count[ready_events_buf_idx];

    UART_begin_transmission();

    // Must use a blocking call in order to mark buffer as free once transfer completes
    UART_send_msg_to_host(USB_RSP_STREAM_EVENTS,
            // TODO: the event count is == NUM_BUFFERED_EVENTS, except for the flush case
            STREAM_DATA_MSG_HEADER_LEN + ready_events_count * sizeof(watchpoint_event_t),
            (uint8_t *)&watchpoint_events_msg_bufs[ready_events_buf_idx][0] +
            WATCHPOINT_EVENT_BUF_HEADER_OFFSET);

    UART_end_transmission();

    watchpoint_events_count[ready_events_buf_idx] = 0; // mark buffer as free
}

static void interrupt_target()
{
    uint16_t cur_vreg;

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_vreg = ADC12_read(&adc12, ADC_CHAN_INDEX_VREG);
    } while (cur_vreg < MCU_ON_THRES);

    __delay_cycles(MCU_BOOT_LATENCY_CYCLES);

    enter_debug_mode(INTERRUPT_TYPE_DEBUGGER_REQ, DEBUG_MODE_FULL_FEATURES);
}

static void get_target_interrupt_context(interrupt_context_t *int_context)
{
    // In case target requested the interrupt, ask it for more details
    UART_send_msg_to_target(WISP_CMD_GET_INTERRUPT_CONTEXT, 0, target_msg_buf);
    while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
            (wispRxPkt.descriptor != WISP_RSP_INTERRUPT_CONTEXT)); // wait for response
    int_context->type = (interrupt_type_t)wispRxPkt.data[0];
    int_context->id = ((uint16_t)wispRxPkt.data[2] << 8) | wispRxPkt.data[1];
    wispRxPkt.processed = 1;
}

static void set_external_breakpoint_pin_state(uint16_t bitmask, bool state)
{
#ifdef WORKAROUND_FLIP_CODEPOINT_PINS
    if (bitmask == 0x1)
        bitmask = 0x2;
    else if (bitmask == 0x2)
        bitmask = 0x1;
#endif

    if (state)
        GPIO(PORT_CODEPOINT, OUT) |= bitmask << PIN_CODEPOINT_0;
    else
        GPIO(PORT_CODEPOINT, OUT) &= ~(bitmask << PIN_CODEPOINT_0);
}

static void toggle_breakpoint(breakpoint_type_t type, unsigned index,
                              uint16_t energy_level, comparator_ref_t cmp_ref,
                              bool enable)
{
    unsigned rc = RETURN_CODE_SUCCESS;
    unsigned payload_len;
    uint16_t prev_breakpoints_mask;
    bool breakpoint_active;

    switch (type) {
        case BREAKPOINT_TYPE_PASSIVE:
            if (index >= MAX_PASSIVE_BREAKPOINTS) {
                rc = RETURN_CODE_INVALID_ARGS;
                break;
            }

            if (enable) {
                // passive and external bkpts cannot be used simultaneously since
                // they reuse the codepoint pins in opposite directions
                if (external_breakpoints) {
                    rc = RETURN_CODE_UNSUPPORTED;
                    break;
                }
                prev_breakpoints_mask = passive_breakpoints;
                passive_breakpoints |= 1 << index; // must be before int is enabled
                if (!prev_breakpoints_mask) {
                    // enable rising-edge interrupt on codepoint pins (harmless to do every time)
                    GPIO(PORT_CODEPOINT, DIR) &= BITS_CODEPOINT;
                    GPIO(PORT_CODEPOINT, IES) &= ~BITS_CODEPOINT;
                    GPIO(PORT_CODEPOINT, IE) |= BITS_CODEPOINT;
                }
            } else {
                passive_breakpoints &= ~(1 << index);
                if (!passive_breakpoints) {
                    GPIO(PORT_CODEPOINT, IE) &= ~BITS_CODEPOINT;
                }
            }
            break;

        case BREAKPOINT_TYPE_INTERNAL:
            if (index >= MAX_INTERNAL_BREAKPOINTS) {
                rc = RETURN_CODE_INVALID_ARGS;
                break;
            }
            if (state != STATE_DEBUG) { // debugger (and target) must be in active debug mode
                rc = RETURN_CODE_COMM_ERROR;
                break;
            }

            if (enable)
                internal_breakpoints |= 1 << index;
            else
                internal_breakpoints &= ~(1 << index);

            payload_len = 0;
            target_msg_payload[payload_len++] = index;
            target_msg_payload[payload_len++] = enable ? 0x1 : 0x0;

            UART_send_msg_to_target(WISP_CMD_BREAKPOINT, payload_len, target_msg_buf);
            while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                    (wispRxPkt.descriptor != WISP_RSP_BREAKPOINT)); // wait for response
            wispRxPkt.processed = 1;
            break;

        case BREAKPOINT_TYPE_EXTERNAL:
            if (index >= MAX_EXTERNAL_BREAKPOINTS) {
                rc = RETURN_CODE_INVALID_ARGS;
                break;
            }

            if (enable) {
                // passive and external bkpts cannot be used simultaneously since
                // they reuse the codepoint pins in opposite directions
                if (passive_breakpoints) {
                    rc = RETURN_CODE_UNSUPPORTED;
                    break;
                }

                if (energy_level) {
                    code_energy_breakpoints |= 1 << index;
                    arm_comparator(CMP_OP_CODE_ENERGY_BREAKPOINT, energy_level, cmp_ref, CMP_EDGE_ANY);
                    // comparator output high means Vcap < cmp ref (activate breakpoint)
                    breakpoint_active = CBCTL1 & CBOUT;

                } else {
                    breakpoint_active = true;
                }
                set_external_breakpoint_pin_state((1 << index), breakpoint_active);

                if (!external_breakpoints) {
                    GPIO(PORT_CODEPOINT, DIR) |= BITS_CODEPOINT;
                }
                external_breakpoints |= 1 << index;
            } else {

                if (code_energy_breakpoints & (1 << index)) {
                    code_energy_breakpoints &= ~(1 << index);
                    disarm_comparator();
                }

                external_breakpoints &= ~(1 << index);
                set_external_breakpoint_pin_state((1 << index), false);

                if (!external_breakpoints) {
                    GPIO(PORT_CODEPOINT, DIR) &= ~BITS_CODEPOINT;
                }
            }

            break;
        default:
            error(ERROR_INVALID_VALUE);
            break;
    }
    send_return_code(rc);
}

static void toggle_watchpoint(unsigned index, bool enable)
{
    unsigned rc = RETURN_CODE_SUCCESS;

    if (index > MAX_WATCHPOINTS) { // effectively one-based index
        rc = RETURN_CODE_INVALID_ARGS;
        goto out;
    }

    if (enable) {
        // watchpoints and passive and exteranl bkpts cannot be used simultaneously since
        // they reuse the codepoint pins in opposite directions
        if (external_breakpoints | passive_breakpoints) {
            rc = RETURN_CODE_UNSUPPORTED;
            goto out;
        }
        watchpoints |= 1 << index;
    } else {
        watchpoints &= ~(1 << index);
    }

out:
    send_return_code(rc);
}

static void enable_watchpoints()
{
    // enable rising-edge interrupt on codepoint pins (harmless to do every time)
    GPIO(PORT_CODEPOINT, DIR) &= BITS_CODEPOINT;
    GPIO(PORT_CODEPOINT, IES) &= ~BITS_CODEPOINT;
    GPIO(PORT_CODEPOINT, IE) |= BITS_CODEPOINT;
}

static void disable_watchpoints()
{
    GPIO(PORT_CODEPOINT, IE) &= ~BITS_CODEPOINT;
}

static void finish_enter_debug_mode()
{
    // WISP has entered debug main loop
    set_state(STATE_DEBUG);
    GPIO(PORT_LED, OUT) |= BIT(PIN_LED_GREEN);

    if (debug_mode_flags & DEBUG_MODE_WITH_UART)
        UART_setup(UART_INTERFACE_WISP);
    if (debug_mode_flags & DEBUG_MODE_WITH_I2C)
        I2C_setup();

    if (debug_mode_flags & DEBUG_MODE_INTERACTIVE)
          main_loop_flags |= FLAG_INTERRUPTED; // main loop notifies the host

    reset_serial_decoder();

    unmask_target_signal(); // listen because target *may* request to exit active debug mode
}

static void finish_exit_debug_mode()
{
    // WISP has shutdown UART and is asleep waiting for int to resume
    if (debug_mode_flags & DEBUG_MODE_WITH_UART)
        UART_teardown(UART_INTERFACE_WISP);
    if (debug_mode_flags & DEBUG_MODE_WITH_I2C)
        I2C_teardown();

    if (debug_mode_flags & DEBUG_MODE_INTERACTIVE)
        main_loop_flags |= FLAG_EXITED_DEBUG_MODE;

    if (!(debug_mode_flags & DEBUG_MODE_NESTED)) {
        continuous_power_off();
        interrupt_context.restored_vcap = discharge_adc(interrupt_context.saved_vcap);
        set_state(STATE_IDLE);
    } else { // nested: go back to the outer debug mode
        debug_mode_flags = interrupt_context.saved_debug_mode_flags;
        set_state(STATE_DEBUG);
    }

    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_GREEN);

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
                if (!(debug_mode_flags & DEBUG_MODE_NESTED))
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
    P4DIR |= BIT0 | BIT3 | BIT7;
    P4OUT &= ~(BIT0 | BIT3 | BIT7);
    P5DIR |= BIT0 | BIT1 | BIT6;
    P5OUT &= ~(BIT0 | BIT1 | BIT6);
    P6DIR |= BIT0 | BIT6 | BIT7;
    P6OUT &= ~(BIT0 | BIT6 | BIT7);
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

#ifdef CONFIG_PULL_DOWN_ON_SIG_LINE
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);
    GPIO(PORT_SIG, REN) |= BIT(PIN_SIG);
#endif

    // Configure the output level for continous power pin ahead of time
    GPIO(PORT_CONT_POWER, OUT) |= BIT(PIN_CONT_POWER);

    // Voltage sense pins as ADC channels
    GPIO(PORT_VSENSE, SEL) |=
        BIT(PIN_VCAP) | BIT(PIN_VBOOST) | BIT(PIN_VREG) | BIT(PIN_VRECT) | BIT(PIN_VINJ);

#ifdef CONFIG_ROUTE_ACLK_TO_PIN
    P1SEL |= BIT0;
    P1DIR |= BIT0;
#endif

    // In our IDLE state, target might request to enter active debug mode
    unmask_target_signal();
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

    ADC12_init(&adc12);

    init_watchpoint_event_bufs();

    TimeLog_request(1);

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

        if((main_loop_flags & FLAG_ADC12_COMPLETE) && (main_loop_flags & FLAG_LOGGING)) {
            // ADC12 has completed conversion on all active channels

            ADC12_send_samples_to_host(&adc12);
            main_loop_flags &= ~FLAG_ADC12_COMPLETE;
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
                    default:
                        ASSERT(ASSERT_UNEXPECTED_TARGET_PKT, false);
                }
            	wispRxPkt.processed = 1;
            }

            if(UART_RxBufEmpty(UART_INTERFACE_WISP)) {
            	main_loop_flags &= ~FLAG_UART_WISP_RX; // clear WISP Rx flag
            }
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
    unsigned payload_len;
    unsigned i;

#ifdef CONFIG_SCOPE_TRIGGER_SIGNAL
    trigger_scope();
#endif

    switch(pkt->descriptor)
    {
    case USB_CMD_SENSE:
        {
            adc_chan_index_t chan_idx = (adc_chan_index_t)pkt->data[0];
            adc12Result = ADC12_read(&adc12, chan_idx);
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
        UART_send_msg_to_target(WISP_CMD_GET_PC, 0, target_msg_buf);
    	while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
    			(wispRxPkt.descriptor != WISP_RSP_ADDRESS)); // wait for response
        forward_msg_to_host(USB_RSP_ADDRESS, wispRxPkt.data, wispRxPkt.length);
    	wispRxPkt.processed = 1;
    	break;

    case USB_CMD_STREAM_BEGIN: {
        uint16_t streams = pkt->data[0];
        adc12.config.sampling_period = (pkt->data[2] << 8) | pkt->data[1];

        streams_bitmask = streams;
        adc_streams_bitmask = streams & ADC_STREAMS;

        if (streams & STREAM_VCAP)
            ADC12_addChannel(&adc12, ADC_CHAN_INDEX_VCAP);
        if (streams & STREAM_VBOOST)
            ADC12_addChannel(&adc12, ADC_CHAN_INDEX_VBOOST);
        if (streams & STREAM_VREG)
            ADC12_addChannel(&adc12, ADC_CHAN_INDEX_VREG);
        if (streams & STREAM_VRECT)
            ADC12_addChannel(&adc12, ADC_CHAN_INDEX_VRECT);
        if (streams & STREAM_VINJ)
            ADC12_addChannel(&adc12, ADC_CHAN_INDEX_VINJ);
#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
        if (streams & STREAM_RF_EVENTS)
            RFID_start_event_stream();
#endif
        if (streams & STREAM_WATCHPOINTS)
            enable_watchpoints();

        // actions common to all adc streams
        if (adc12.config.num_channels > 0) {
            main_loop_flags |= FLAG_LOGGING; // for main loop
            ADC12_setup(&adc12, adc_streams_bitmask);
        }
        break;
    }

    case USB_CMD_STREAM_END: {
        unsigned streams = pkt->data[0];

        adc_streams_bitmask &= ~(streams & ADC_STREAMS);
        streams_bitmask = 0;

        if (streams & STREAM_VCAP)
            ADC12_removeChannel(&adc12, ADC_CHAN_INDEX_VCAP);
        if (streams & STREAM_VBOOST)
            ADC12_removeChannel(&adc12, ADC_CHAN_INDEX_VBOOST);
        if (streams & STREAM_VREG)
            ADC12_removeChannel(&adc12, ADC_CHAN_INDEX_VREG);
        if (streams & STREAM_VRECT)
            ADC12_removeChannel(&adc12, ADC_CHAN_INDEX_VRECT);
        if (streams & STREAM_VINJ)
            ADC12_removeChannel(&adc12, ADC_CHAN_INDEX_VINJ);
#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
        if (streams & STREAM_RF_EVENTS)
            RFID_stop_event_stream();
#endif
        if (streams & STREAM_WATCHPOINTS)
            disable_watchpoints();

        // actions common to all adc streams
        if (adc12.config.num_channels == 0) {
            ADC12_stop();
            main_loop_flags &= ~FLAG_LOGGING; // for main loop
        }
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
#endif

    case USB_CMD_SET_PWM_FREQUENCY:
    	TB0CCR0 = (*((uint16_t *)(pkt->data))) - 1;
    	break;

    case USB_CMD_SET_PWM_DUTY_CYCLE:
    	TB0CCR1 = *((uint16_t *)(pkt->data));
    	break;

#ifdef CONFIG_PWM_CHARGING
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

        payload_len = 0;
        target_msg_payload[payload_len++] = (address >> 0) & 0xff;
        target_msg_payload[payload_len++] = (address >> 8) & 0xff;
        target_msg_payload[payload_len++] = (address >> 16) & 0xff;
        target_msg_payload[payload_len++] = (address >> 24) & 0xff;
        target_msg_payload[payload_len++] = len;

        UART_send_msg_to_target(WISP_CMD_READ_MEM, payload_len, target_msg_buf);
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

        payload_len = 0;
        target_msg_payload[payload_len++] = (address >> 0) & 0xff;
        target_msg_payload[payload_len++] = (address >> 8) & 0xff;
        target_msg_payload[payload_len++] = (address >> 16) & 0xff;
        target_msg_payload[payload_len++] = (address >> 24) & 0xff;
        target_msg_payload[payload_len++] = len;

        for (i = 0; i < len; ++i) {
            target_msg_payload[payload_len++] = *value;
            value++;
        }

        UART_send_msg_to_target(WISP_CMD_WRITE_MEM, payload_len, target_msg_buf);
        while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                (wispRxPkt.descriptor != WISP_RSP_MEMORY)); // wait for response
        wispRxPkt.processed = 1;

        send_return_code(RETURN_CODE_SUCCESS); // TODO: have WISP return a code
        break;
    }

    case USB_CMD_CONT_POWER:
    {
        bool power_on = (bool)pkt->data[0];
        if (power_on)
            continuous_power_on();
        else
            continuous_power_off();
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
        bool enable = (bool)pkt->data[1];
        toggle_watchpoint(index, enable);
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

        payload_len = 0;
        target_msg_payload[payload_len++] = value;
        UART_send_msg_to_target(WISP_CMD_SERIAL_ECHO, payload_len, target_msg_buf);

        // Wait while the ISRs decode the serial bit stream
        volatile uint16_t timeout = 0xffff;
        while (state == STATE_SERIAL_ECHO && --timeout > 0);
        if (state == STATE_SERIAL_ECHO) // timeout
            set_state(saved_sig_serial_echo_state);

        while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                (wispRxPkt.descriptor != WISP_RSP_SERIAL_ECHO)); // wait for response
        wispRxPkt.processed = 1;

        payload_len = 0;
        host_msg_payload[payload_len++] = value;
        send_msg_to_host(USB_RSP_ECHO, payload_len);
        break;
    }

    case USB_CMD_DMA_ECHO: {
        unsigned value = pkt->data[0];
        payload_len = 0;
        host_msg_payload[payload_len++] = value;
        send_msg_to_host(USB_RSP_ECHO, payload_len);
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

    default:
        break;
    }

    pkt->processed = 1;
}

static uint16_t charge_adc(uint16_t target)
{
    uint16_t cur_voltage;

    // Output Vcc level to Vcap (through R1) */

    // Configure the pin
    GPIO(PORT_CHARGE, DS) |= BIT(PIN_CHARGE); // full drive strength
    GPIO(PORT_CHARGE, SEL) &= ~BIT(PIN_CHARGE); // I/O function
    GPIO(PORT_CHARGE, DIR) |= BIT(PIN_CHARGE); // I/O function output

    GPIO(PORT_CHARGE, OUT) |= BIT(PIN_CHARGE); // turn on the power supply

    // Wait for the cap to charge to that voltage

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);
    } while (cur_voltage < target);

    GPIO(PORT_CHARGE, OUT) &= ~BIT(PIN_CHARGE); // cut the power supply

    return cur_voltage;
}

static uint16_t discharge_adc(uint16_t target)
{
    uint16_t cur_voltage;

    GPIO(PORT_DISCHARGE, DIR) |= BIT(PIN_DISCHARGE); // open the discharge "valve"

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);
    } while (cur_voltage > target);

    GPIO(PORT_DISCHARGE, DIR) &= ~BIT(PIN_DISCHARGE); // close the discharge "valve"

    return cur_voltage;
}

static void charge_cmp(uint16_t target, comparator_ref_t ref)
{
    arm_comparator(CMP_OP_CHARGE, target, ref, CMP_EDGE_FALLING);

    // Configure the pin
    GPIO(PORT_CHARGE, DS) |= BIT(PIN_CHARGE); // full drive strength
    GPIO(PORT_CHARGE, SEL) &= ~BIT(PIN_CHARGE); // I/O function
    GPIO(PORT_CHARGE, DIR) |= BIT(PIN_CHARGE); // I/O function output

    GPIO(PORT_CHARGE, OUT) |= BIT(PIN_CHARGE); // turn on the power supply

    // expect comparator interrupt
}

static void discharge_cmp(uint16_t target, comparator_ref_t ref)
{
    arm_comparator(CMP_OP_DISCHARGE, target, ref, CMP_EDGE_RISING);

    GPIO(PORT_DISCHARGE, DIR) |= BIT(PIN_DISCHARGE); // open the discharge "valve"

    // expect comparator interrupt
}

#ifdef CONFIG_PWM_CHARGING

/**
 * @brief	Compare two unsigned 16-bit numbers.
 * @param	n1	One number
 * @param	n2	Another number
 * @param	threshold	minimum difference between n1 and n2 to consider them different.
 * @retval	1	n1 > n2 + threshold
 * @retval	0	n1 and n2 are within threshold of one another
 * @retval	-1	n1 < n2 - threshold
 */
static int uint16Compare(uint16_t n1, uint16_t n2, uint16_t threshold) {
	if(n1 < n2 - threshold && n2 >= threshold) {
		return -1;
	} else if(n1 > n2 + threshold && n2 + threshold <= 0xFFFF) {
		return 1;
	}
	return 0;
}

static void setWispVoltage_block(unsigned adc_chan_index, uint16_t target)
{
	uint16_t result;
    int compare;
	unsigned threshold = 1;

	// Here, we want to choose a starting PWM duty cycle close to, but not above,
	// what the correct one will be.  We want it to be close because we will test
	// each duty cycle, increasing one cycle at a time, until we reach the right
	// one.  In my experiment with the oscilloscope, the WISP cap took about 60ms
	// to charge, so I'll give it 80ms for the first charge and 10ms for each
	// charge following.

	// We know the ADC target, so let's give the PWM duty cycle our best guess.
	// target (adc) * PWM_period (SMCLK cycles) / 2^12 (adc) = approx. PWM_duty_cycle (SMCLK cycles)
	// Subtract 40 from this to start.
	uint32_t duty_cycle = (uint32_t) target * (uint32_t) TB0CCR0 / 4096;
	TB0CCR1 = MAX((uint16_t) duty_cycle - 40, 0);
	PWM_start();

	// 70ms wait time
	unsigned i;
	for(i = 0; i < 40; i++) {
		__delay_cycles(21922); // delay for 1ms
	}

	do {
		// 10ms wait time
		for(i = 0; i < 40; i++) {
			__delay_cycles(21922); // delay for 1ms
		}

		result = ADC12_read(&adc12, adc_chan_index);
		compare = uint16Compare(result, target, threshold);
		if(compare < 0) {
			// result < target
			PWM_INCREASE_DUTY_CYCLE;
		} else if(compare > 0) {
			// result > target
			PWM_DECREASE_DUTY_CYCLE;
		}
	} while(compare != 0);
}
#endif

static void break_at_vcap_level_adc(uint16_t level)
{
    uint16_t cur_vcap, cur_vreg;

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */

    cur_vreg = ADC12_read(&adc12, ADC_CHAN_INDEX_VREG);
    if (cur_vreg < MCU_ON_THRES) { // MCU is off, wait for MCU to turn on
        do {
            cur_vreg = ADC12_read(&adc12, ADC_CHAN_INDEX_VREG);
        } while (cur_vreg < MCU_ON_THRES);
        // TODO: MCU boot delay
        __delay_cycles(35000);
        __delay_cycles(35000);
    } // else: MCU is already on, go on to check Vcap right away

    do {
        cur_vcap = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);
    } while (cur_vcap > level);

    enter_debug_mode(INTERRUPT_TYPE_ENERGY_BREAKPOINT, DEBUG_MODE_FULL_FEATURES);
}

static void break_at_vcap_level_cmp(uint16_t level, comparator_ref_t ref)
{
    arm_comparator(CMP_OP_ENERGY_BREAKPOINT, level, ref, CMP_EDGE_RISING);
    // expect comparator interrupt
}

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
    unsigned pin_state = GPIO(PORT_CODEPOINT, IN); // snapshot

	switch(__even_in_range(P1IV, 16))
	{
    case INTFLAG(PORT_RF, PIN_RF_TX):
#ifdef CONFIG_ENABLE_RF_PROTOCOL_MONITORING
#ifdef CONFIG_ENABLE_RF_TX_DECODING
        rfid_decoder_tx_pin_isr();
#endif // CONFIG_ENABLE_RF_TX_DECODING
        GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);
#endif // CONFIG_ENABLE_RF_PROTOCOL_MONITORING
        break;
	case INTFLAG(PORT_SIG, PIN_SIG):
		handle_target_signal();
		GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG);
		break;

    case INTFLAG(PORT_CODEPOINT, PIN_CODEPOINT_0):
    case INTFLAG(PORT_CODEPOINT, PIN_CODEPOINT_1):
    {
#if defined(CONFIG_ENABLE_WATCHPOINTS)
        /* Workaround the hardware routing that routes AUX1,AUX2 to pins out of order */
        pin_state = (pin_state & BIT(PIN_CODEPOINT_0) ? BIT(PIN_CODEPOINT_1) : 0) |
                    (pin_state & BIT(PIN_CODEPOINT_1) ? BIT(PIN_CODEPOINT_0) : 0);

        // NOTE: can't encode a zero-based index, because the pulse must trigger the interrupt
        // -1 to convert from one-based to zero-based index
        unsigned index = ((pin_state & BITS_CODEPOINT) >> PIN_CODEPOINT_0);
        if (watchpoints & (1 << index))
            append_watchpoint_event(index);

#elif defined(CONFIG_ENABLE_PASSIVE_BREAKPOINTS)
        if (!passive_breakpoints) {
            error(ERROR_UNEXPECTED_INTERRUPT);
            break;
        }

        /* Workaround the hardware routing that routes AUX1,AUX2 to pins out of order */
        pin_state = (pin_state & BIT(PIN_CODEPOINT_0) ? BIT(PIN_CODEPOINT_1) : 0) |
                    (pin_state & BIT(PIN_CODEPOINT_1) ? BIT(PIN_CODEPOINT_0) : 0);

        // NOTE: can't encode a zero-based index, because the pulse must trigger the interrupt
        // -1 to convert from one-based to zero-based index
        unsigned index = ((pin_state & BITS_CODEPOINT) >> PIN_CODEPOINT_0) - 1;
        if (passive_breakpoints & (1 << index)) {
            if (state == STATE_DEBUG)
                error(ERROR_UNEXPECTED_CODEPOINT);

            enter_debug_mode(INTERRUPT_TYPE_BREAKPOINT, DEBUG_MODE_FULL_FEATURES);
        }
#endif // CONFIG_ENABLE_PASSIVE_BREAKPOINTS
        break;
    }

	default:
		break;
	}
}

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
        default:
            error(ERROR_UNEXPECTED_INTERRUPT);
            break;
    }
}

#ifdef CONFIG_ENABLE_DEBUG_MODE
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A0_VECTOR))) TIMER2_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    --sig_serial_bit_index;

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
        case DMA_INTFLAG(DMA_HOST_UART_TX):
               host_uart_status &= ~UART_STATUS_TX_BUSY;
            break;
    }
}
