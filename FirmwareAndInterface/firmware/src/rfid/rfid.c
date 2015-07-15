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

#include "rfid.h"

#define NUM_BUFFERS                                  2 // double-buffer pair
#define NUM_EVENTS_BUFFERED                         32 

#define STREAM_DATA_MSG_HEADER_LEN (sizeof(uint8_t) + 1) // 'streams bitmask' field + padding
#define RF_EVENT_BUF_OFFSET STREAM_DATA_MSG_HEADER_LEN
#define RF_EVENT_BUF_SIZE (sizeof(rf_event_t) * NUM_EVENTS_BUFFERED)

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
static uint8_t rf_events_msg_bufs[NUM_BUFFERS][RF_EVENT_BUF_OFFSET + RF_EVENT_BUF_SIZE];

/** @brief Pointers to each event buffer in the double-buffer pair
 *  @details These pointers point to the event data "payload" of the message,
 *           skipping the other message fields.
 */
static rf_event_t * const rf_events_bufs[NUM_BUFFERS] = {
    (rf_event_t *)&rf_events_msg_bufs[0][RF_EVENT_BUF_OFFSET],
    (rf_event_t *)&rf_events_msg_bufs[1][RF_EVENT_BUF_OFFSET]
};

/** @brief Pointer and index to current event buffer among the double-buffer pair
 *  @details These are, strictly speaking, redundant, but improve legibility
 *           by allowing to refer to msg buffer when sending the message and to
 *           events buffer when appending events. Also, we do want to store a
 *           pointer to the current buffer in order to avoid recomputing it
 *           from the buffer index in every ISR.
 * */
static uint8_t rf_events_buf_idx;
static rf_event_t *rf_events_buf;
/** @brief Number of events in the current buffer so far */
static uint8_t rf_events_count = 0;


// TODO: the flag macros should just be in a shared header
static uint16_t *pFlags;
static uint16_t data_ready_flag;

static void append_event(rf_event_type_t id)
{
    rf_event_t *rf_event;

    ASSERT(ASSERT_RF_EVENTS_BUF_OVERFLOW, rf_events_count < NUM_EVENTS_BUFFERED);

    rf_event = &rf_events_buf[rf_events_count++];

    // We could take the timestamp a few cycles earlier (in the ISR/callbacks),
    // but it's hardly worth the sacrifice in code conciseness.
    rf_event->timestamp = TIMELOG_CURRENT_TIME;
    rf_event->id = id;

	*pFlags |= data_ready_flag;	// alert the main loop that data is ready
}

static inline void handle_rfid_cmd(rfid_cmd_code_t cmd_code)
{
    // TODO: don't do anything yet
    //append_event(RF_EVENT_TYPE_CMD | cmd_code);
}

static inline void handle_rfid_rsp(rfid_rsp_code_t rsp_code)
{
    // TODO: don't do anything yet
    //append_event(RF_EVENT_TYPE_RSP | rsp_code);
}

void RFID_setup(uint16_t *pFlag_bitmask, uint16_t data_ready_flag_arg)
{
    // will set this flag in this bitmask when there is data to transmit to the host
	pFlags = pFlag_bitmask;
	data_ready_flag = data_ready_flag_arg;

    // Start filling up the first of the two buffers
    rf_events_buf_idx = 0;
    rf_events_buf = rf_events_bufs[rf_events_buf_idx];

    // Initialize message header
    rf_events_msg_bufs[0][0] = STREAM_RF_EVENTS;
    rf_events_msg_bufs[1][0] = STREAM_RF_EVENTS;

    rfid_decoder_init(&handle_rfid_cmd, &handle_rfid_rsp);
}

void RFID_start_event_stream()
{
    rfid_decoder_start();
}

void RFID_stop_event_stream()
{
    rfid_decoder_stop();
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
void RFID_send_rf_events_to_host()
{
    uint8_t ready_events_buf_idx;
    uint8_t ready_events_count;

    // Switch double-buffers

    // Updates saving current count and updates to buf and to count must be
    // atomic: appends (from ISRs) must be disallowed
    __disable_interrupt();
    ready_events_buf_idx = rf_events_buf_idx;
    ready_events_count = rf_events_count;

    rf_events_buf_idx = rf_events_buf_idx == 0 ? 1 : 0;
    rf_events_buf = rf_events_bufs[rf_events_buf_idx];
    rf_events_count = 0;
    __enable_interrupt();

	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_STREAM_DATA,
                 rf_events_msg_bufs[ready_events_buf_idx],
                 STREAM_DATA_MSG_HEADER_LEN + ready_events_count * sizeof(rf_event_t),
                 UART_TX_DROP);
}
