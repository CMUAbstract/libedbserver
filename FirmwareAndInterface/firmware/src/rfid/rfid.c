#if CONFIG_ENABLE_RF_PROTOCOL_MONITORING

/*******************************************************************************
 * @file
 * @brief           Monitoring of WISP RFID communication and reporting to host
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>

#include "pin_assign.h"
#include "timeLog.h"
#include "uart.h"
#include "host_comm.h"
#include "error.h"
#include "rfid_decoder.h"
#include "main_loop.h"

#include "rfid.h"

/*Buffer structure:
 *  [ empty_space | header | rf_event_t #1 | rf_event_t #2 | ... | rf_event_t #n ]
 * where len(empty_space + header) == sizeof(rf_event_t) because compiler might
 * have alignment rules for structs: can't treat any memory address as pointer to
 * a struct (this is a hypothesis, though).
 *
 * Note that rf_event_t items include padding from the struct.
 *
 * The bytes sent in the UART message start at header.
 */

#define NUM_BUFFERS                                  2 // double-buffer pair
#define NUM_EVENTS_BUFFERED                         48

#define STREAM_DATA_MSG_HEADER_LEN 2 // 'streams bitmask', padding (units of bytes, aligned to 2)

#define RF_EVENT_BUF_HEADER_SPACE 1 // units of sizeof(rf_event_t)
#define RF_EVENT_BUF_PAYLOAD_SPACE NUM_EVENTS_BUFFERED // units of sizeof(rf_event_t)
#define RF_EVENT_BUF_SIZE (RF_EVENT_BUF_HEADER_SPACE + RF_EVENT_BUF_PAYLOAD_SPACE) // units of sizeof(rf_event_t)

#if STREAM_DATA_MSG_HEADER > RF_EVENT_BUF_HEADER_SPACE
#error Not enough space for header in event buf: add more units of sizeof(struct rf_event_t)
#endif

// The header must be aligned because we need pointers *within* the buffer to payload field
#if STREAM_DATA_MSG_HEADER_LEN & 0x1 == 0x1
#error Stream message header size must be aligned to 2
#endif

#define RF_EVENT_BUF_HEADER_OFFSET \
    (RF_EVENT_BUF_HEADER_SPACE * sizeof(rf_event_t) - \
        (UART_MSG_HEADER_SIZE + STREAM_DATA_MSG_HEADER_LEN))

#define STARTING_EVENT_BUF_IDX 0

typedef struct {
    uint32_t timestamp;
    rf_event_type_t id;
} rf_event_t;

/** @brief Memory allocated for the double-buffer pair
 *
 *  @details To avoid an extra copy, the event buffer is contiguous with
 *           memory for the other message fields.
 */
static rf_event_t rf_events_msg_bufs[NUM_BUFFERS][RF_EVENT_BUF_SIZE];

/** @brief Pointers to each event buffer in the double-buffer pair
 *  @details These pointers point to the event data "payload" of the message,
 *           skipping the other message fields.
 */
static rf_event_t * const rf_events_bufs[NUM_BUFFERS] = {
    (rf_event_t *)&rf_events_msg_bufs[0][RF_EVENT_BUF_HEADER_SPACE],
    (rf_event_t *)&rf_events_msg_bufs[1][RF_EVENT_BUF_HEADER_SPACE]
};

/** @brief Pointer and index to current event buffer among the double-buffer pair
 *  @details These are, strictly speaking, redundant, but improve legibility
 *           by allowing to refer to msg buffer when sending the message and to
 *           events buffer when appending events. Also, we do want to store a
 *           pointer to the current buffer in order to avoid recomputing it
 *           from the buffer index in every ISR.
 * */
static unsigned rf_events_buf_idx;
static rf_event_t *rf_events_buf;
/** @brief Number of events in the current buffer so far */
static unsigned rf_events_count[NUM_BUFFERS];


static void append_event(rf_event_type_t id)
{
    rf_event_t *rf_event;

#ifdef CONFIG_ABORT_ON_RFID_EVENT_OVERFLOW
    ASSERT(ASSERT_RF_EVENTS_BUF_OVERFLOW, rf_events_count[rf_events_buf_idx] < NUM_EVENTS_BUFFERED);
#else
    if (rf_events_count < NUM_EVENTS_BUFFERED)
        return;
#endif

    rf_event = &rf_events_buf[rf_events_count[rf_events_buf_idx]];

    // We could take the timestamp a few cycles earlier (in the ISR/callbacks),
    // but it's hardly worth the sacrifice in code conciseness.
    rf_event->timestamp = TIMELOG_CURRENT_TIME();
    rf_event->id = id;

    rf_events_count[rf_events_buf_idx]++;

    // If full, then swap buffers
    if (rf_events_count[rf_events_buf_idx] == NUM_EVENTS_BUFFERED) {
        rf_events_buf_idx = rf_events_buf_idx ^ rf_events_buf_idx;
        rf_events_buf = rf_events_bufs[rf_events_buf_idx];

        ASSERT(ASSERT_RF_EVENTS_BUF_OVERFLOW, rf_events_count[rf_events_buf_idx] == 0);
    }

    main_loop_flags |= FLAG_RF_DATA;
}

static inline void handle_rfid_cmd(rfid_cmd_code_t cmd_code)
{
    append_event(RF_EVENT_TYPE_CMD | cmd_code);
}

static inline void handle_rfid_rsp(rfid_rsp_code_t rsp_code)
{
    // TODO: don't do anything yet
    //append_event(RF_EVENT_TYPE_RSP | rsp_code);
}

/**
 * @brief Send collected RF events to the host via the USB UART
 *
 * @details The approach chosen is to let this send function
 *          switch the double-buffers whenever it is called, instead of having
 *          the ISRs switch them after filling the current buffer and notifying
 *          the main loop to invoke the send function.
 *
 *          The advantage of the former approach is that we will not wait for
 *          the buffer to fill up before sending an event.  The latter approach
 *          would lead to events getting stuck in the debugger when there are
 *          few events.
 *
 *          The disadvantage of the former approach is that we will end up
 *          doing a lot of buffer switches.
 */
static inline void send_rf_events_buf_host(unsigned ready_events_buf_idx)
{
    unsigned next_rf_events_buf_idx;
    unsigned ready_events_count;
    rf_event_t *next_rf_events_buf;

    ready_events_count = rf_events_count[ready_events_buf_idx];

    // Must use a blocking call in order to mark buffer as free once transfer completes
    UART_send_msg_to_host(USB_RSP_STREAM_DATA, 
            STREAM_DATA_MSG_HEADER_LEN + ready_events_count * sizeof(rf_event_t),
            (uint8_t *)&rf_events_msg_bufs[ready_events_buf_idx][0] + RF_EVENT_BUF_HEADER_OFFSET);

    rf_events_count[ready_events_buf_idx] = 0; // mark buffer as free
}

static inline void RFID_flush_rf_events_buf()
{
    send_rf_events_buf_host(rf_events_buf_idx);
}

void RFID_send_ready_rf_events_buf()
{
    send_rf_events_buf_host(rf_events_buf_idx ^ rf_events_buf_idx);
}

void RFID_setup()
{
    unsigned i;
    unsigned offset;
    uint8_t *header;

    // Initialize message header
    for (i = 0; i < NUM_BUFFERS; ++i) {
        header = (uint8_t *)&rf_events_msg_bufs[i][0] + RF_EVENT_BUF_HEADER_OFFSET;
        offset = 0;
        header[offset++] = STREAM_RF_EVENTS;
        header[offset++] = 0; // padding
        ASSERT(ASSERT_INVALID_STREAM_BUF_HEADER, offset == STREAM_DATA_MSG_HEADER_LEN);
    }

    rfid_decoder_init(&handle_rfid_cmd, &handle_rfid_rsp);
}

void RFID_start_event_stream()
{
    // Start filling up the first of the two buffers
    rf_events_buf_idx = 0;
    rf_events_buf = rf_events_bufs[rf_events_buf_idx];
    rf_events_count[0] = rf_events_count[1] = 0;

    rfid_decoder_start();
}

void RFID_stop_event_stream()
{
    rfid_decoder_stop();
    RFID_flush_rf_events_buf();
}

#else
unsigned __dummy_rfid_c; // silence 'empty translation unit' warning
#endif // CONFIG_ENABLE_RF_PROTOCOL_MONITORING
