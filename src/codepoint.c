#include <msp430.h>
#include <string.h>

#include <libmsp/periph.h>
#include <libedb/target_comm.h>

#include "config.h"
#include "pin_assign.h"
#include "comparator.h"
#include "host_comm.h"
#include "host_comm_impl.h"
#include "target_comm_impl.h"
#include "error.h"
#include "uart.h"
#include "adc.h"
#include "systick.h"
#include "main_loop.h"
#include "tether.h"
#include "payload.h"

#include "codepoint.h"

typedef struct {
    uint32_t timestamp;
    unsigned index;
    unsigned vcap;
} watchpoint_event_t;

// Bitmasks indicate whether a breakpoint (group) of given index is enabled
uint16_t passive_breakpoints = 0;
uint16_t external_breakpoints = 0;
uint16_t internal_breakpoints = 0;
uint16_t code_energy_breakpoints = 0;
static bool boot_breakpoint = false;

uint16_t watchpoints = 0;
static uint16_t watchpoints_vcap_snapshot = 0;

// See libedb/edb.h for description
#define NUM_CODEPOINT_VALUES     NUM_CODEPOINT_PINS
#define MAX_PASSIVE_BREAKPOINTS  NUM_CODEPOINT_VALUES
#define MAX_INTERNAL_BREAKPOINTS (sizeof(uint16_t) * 8) // _debug_breakpoints_enable in libdebug
#define MAX_EXTERNAL_BREAKPOINTS NUM_CODEPOINT_PINS

#define MAX_WATCHPOINTS          NUM_CODEPOINT_VALUES

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

#ifdef CONFIG_ENABLE_WATCHPOINT_STREAM
static watchpoint_event_t
watchpoint_events_msg_bufs[NUM_WATCHPOINT_BUFFERS][WATCHPOINT_EVENT_BUF_SIZE];

static watchpoint_event_t *watchpoint_events_bufs[NUM_WATCHPOINT_BUFFERS] = {
    &watchpoint_events_msg_bufs[0][WATCHPOINT_EVENT_BUF_HEADER_SPACE],
    &watchpoint_events_msg_bufs[1][WATCHPOINT_EVENT_BUF_HEADER_SPACE]
};

static unsigned watchpoint_events_count[NUM_WATCHPOINT_BUFFERS];
static watchpoint_event_t *watchpoint_events_buf;
static unsigned watchpoint_events_buf_idx;
#endif // CONFIG_ENABLE_WATCHPOINT_STREAM


void set_external_breakpoint_pin_state(uint16_t bitmask, bool state)
{
    if (state)
        GPIO(PORT_CODEPOINT, OUT) |= bitmask << PIN_CODEPOINT_0;
    else
        GPIO(PORT_CODEPOINT, OUT) &= ~(bitmask << PIN_CODEPOINT_0);
}

unsigned toggle_breakpoint(breakpoint_type_t type, unsigned index,
                              uint16_t energy_level, comparator_ref_t cmp_ref,
                              bool enable)
{
    unsigned rc = RETURN_CODE_SUCCESS;
    uint16_t prev_breakpoints_mask;
    bool breakpoint_active;

    LOG("toggle bkpt: type %u idx %u e %u en %u\r\n",
        type, index, energy_level, enable);

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

            target_comm_send_breakpoint(index, enable);
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
                    arm_comparator(CMP_OP_CODE_ENERGY_BREAKPOINT, energy_level,
                                   cmp_ref, CMP_EDGE_ANY, COMP_CHAN_VCAP);
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
                    GPIO(PORT_CODEPOINT, OUT) &= ~BITS_CODEPOINT;
                    GPIO(PORT_CODEPOINT, DIR) &= ~BITS_CODEPOINT;

                    // TODO: there is a problem: once we go into high-Z, level-shifter
                    // thinks it's being driven high, so target detects an enabled breakpoint.
                    // Pull-downs are only enabled on our side. Also tried to set REN
                    // here again (in addition to on boot) -- no difference. Pull-downs
                    // on target side won't do anything because level-shifter drives
                    // target-side high (but did try pull-downs on on target-side anyway).
                    // Perhaps a pull down on EDB side on the board would fix this? But,
                    // then, the built-in pull-down should should also fix it.
                }
            }

            break;
        case BREAKPOINT_TYPE_BOOT:

            if (enable) {
                // Boot breakpoint is identified by all codepoint pins high
                // Note that this is not one of valid values for external bkpts (1 bit only).
                set_external_breakpoint_pin_state(NUM_CODEPOINT_VALUES, true);
                GPIO(PORT_CODEPOINT, DIR) |= BITS_CODEPOINT;
                boot_breakpoint = true;
            } else {
                boot_breakpoint = false;
                set_external_breakpoint_pin_state(NUM_CODEPOINT_VALUES, false);
                GPIO(PORT_CODEPOINT, DIR) &= ~BITS_CODEPOINT;
            }

            break;
        default:
            error(ERROR_INVALID_VALUE);
            break;
    }
    return rc;
}

unsigned toggle_watchpoint(unsigned index, bool enable, bool vcap_snapshot)
{
    unsigned rc = RETURN_CODE_SUCCESS;

    LOG("toggle watchpt: idx %u e %u en %u\r\n",
        index, vcap_snapshot, enable);

    if (index >= MAX_WATCHPOINTS) {
        rc = RETURN_CODE_INVALID_ARGS;
        goto out;
    }

    if (enable) {
        // watchpoints and passive and exteranl bkpts cannot be used simultaneously since
        // they reuse the codepoint pins in opposite directions
        if (boot_breakpoint | external_breakpoints | passive_breakpoints) {
            rc = RETURN_CODE_UNSUPPORTED;
            goto out;
        }
        watchpoints |= 1 << index;
        watchpoints_vcap_snapshot |= ((uint16_t)vcap_snapshot) << index;
    } else {
        watchpoints &= ~(1 << index);
        watchpoints_vcap_snapshot &= ~(((uint16_t)vcap_snapshot) << index);
    }

// TODO: not sure how to make this coexist with enabling on stream start
#ifdef CONFIG_AUTO_ENABLED_WATCHPOINTS
    if (watchpoints)
        enable_watchpoints();
    else
        disable_watchpoints();
#endif

out:
    return rc;
}

static void swap_buffers()
{
    // swap to the other buffer in the double-buffer pair
    watchpoint_events_buf_idx ^= 1;
    watchpoint_events_buf = watchpoint_events_bufs[watchpoint_events_buf_idx];
    main_loop_flags |= FLAG_WATCHPOINT_READY;
}

#ifdef CONFIG_ENABLE_WATCHPOINT_STREAM
void init_watchpoint_event_bufs()
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

        // Just for easier diagnostics of problems in the data stream
        memset(watchpoint_events_bufs[i], 0,
               NUM_WATCHPOINT_EVENTS_BUFFERED * sizeof(watchpoint_event_t));
    }
    watchpoint_events_buf_idx = 0;
    watchpoint_events_buf = watchpoint_events_bufs[watchpoint_events_buf_idx];
}

static void append_watchpoint_event(unsigned index)
{
    if (watchpoint_events_count[watchpoint_events_buf_idx] <
            NUM_WATCHPOINT_EVENTS_BUFFERED) {

        watchpoint_event_t *watchpoint_event =
            &watchpoint_events_buf[watchpoint_events_count[watchpoint_events_buf_idx]++];

        watchpoint_event->timestamp = SYSTICK_CURRENT_TIME;
        watchpoint_event->index = index;
        if (watchpoints_vcap_snapshot & (1 << index))
            watchpoint_event->vcap = ADC_read(ADC_CHAN_INDEX_VCAP);
        else // TODO: don't stream vcap at all if snapshot is not enabled
            watchpoint_event->vcap = 0;

    }

    if (watchpoint_events_count[watchpoint_events_buf_idx] ==
            NUM_WATCHPOINT_EVENTS_BUFFERED) {// buffer full
        if (!(main_loop_flags & FLAG_WATCHPOINT_READY)) { // the other buffer is free
            swap_buffers();
            // clear error indicator
            GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);
        } else { // both buffers are full
            // indicate error on LED
            GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);

            // drop the watchpoints on the floor
        }
    } else {
        // clear error indicator
        GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);
    }
}

void send_watchpoint_events()
{
    unsigned ready_events_count;
    unsigned ready_events_buf_idx = watchpoint_events_buf_idx ^ 1; // the other one in the pair

    ready_events_count = watchpoint_events_count[ready_events_buf_idx];

    LOG("wpts: send buf %u cnt %u\r\n", ready_events_buf_idx, ready_events_count);

    UART_begin_transmission();

    // Must use a blocking call in order to mark buffer as free once transfer completes
    UART_send_msg_to_host(USB_RSP_STREAM_EVENTS,
            STREAM_DATA_MSG_HEADER_LEN + ready_events_count * sizeof(watchpoint_event_t),
            (uint8_t *)&watchpoint_events_msg_bufs[ready_events_buf_idx][0] +
            WATCHPOINT_EVENT_BUF_HEADER_OFFSET);

    UART_end_transmission();

    watchpoint_events_count[ready_events_buf_idx] = 0; // mark buffer as free
}
#endif // CONFIG_ENABLE_WATCHPOINT_STREAM

void enable_watchpoints()
{
    // enable rising-edge interrupt on enabled codepoint pins (harmless to do every time)
    uint8_t enabled_pins = 0;
    for (int i = 0; i < NUM_CODEPOINT_PINS; ++i) {
        enabled_pins |= (watchpoints & (1 << i)) ? ((1 << i) << PIN_CODEPOINT_0) : 0;
    }

    GPIO(PORT_CODEPOINT, DIR) &= ~BITS_CODEPOINT;
    GPIO(PORT_CODEPOINT, IES) &= ~BITS_CODEPOINT;
    GPIO(PORT_CODEPOINT, IFG) &= ~BITS_CODEPOINT;

    GPIO(PORT_CODEPOINT, IE) |= enabled_pins;
}

void disable_watchpoints()
{
    GPIO(PORT_CODEPOINT, IE) &= ~BITS_CODEPOINT;
}

void watchpoints_start_stream()
{
    LOG("wpts: start stream: wpts 0x%04x\r\n", watchpoints);

    init_watchpoint_event_bufs(); // need to clear count
    enable_watchpoints();
}

void watchpoints_stop_stream()
{
    LOG("wpts: stop stream\r\n");

    disable_watchpoints();
    swap_buffers();
}

void handle_codepoint(unsigned index)
{
#if defined(CONFIG_ENABLE_WATCHPOINTS)

        // NOTE: can't encode a zero-based index, because the pulse must
        // trigger the interrupt
        if (watchpoints & (1 << index)) {
#ifdef CONFIG_COLLECT_ENERGY_PROFILE
            // TODO: set and use the flag in watchpoints_vcap_snapshot in sprite-mode too
            uint16_t vcap = ADC_read(ADC_CHAN_INDEX_VCAP);
            payload_record_profile_event(index, vcap);
#endif
#ifdef CONFIG_ENABLE_WATCHPOINT_STREAM
            append_watchpoint_event(index);
#endif
        }

#elif defined(CONFIG_ENABLE_PASSIVE_BREAKPOINTS)

#error NOT IMPLEMENTED: TODO: implement without encoding index onto pins
// The problem is that EDB might not read the pin state in time, because
// it might be busy with another ISR.

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
}
