#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <msp430.h>

#include <libmsp/periph.h>

#include "config.h"
#include "pin_assign.h"
#include "error.h"
#include "uart.h"
#include "params.h"
#include "host_comm.h"

#include "host_comm_impl.h"

#define HOST_MSG_BUF_SIZE       64 // buffer for UART messages (to host) for main loop

/**
 * @brief Message payload pointer in a buffer for messages to host
 * @details This buffer is used exclusively by main loop, so it is
 *          shared only in the sense of being multi-plexed in time, i.e. it is
 *          never used concurrently but to threads of control.
 */
static uint8_t host_msg_buf[HOST_MSG_BUF_SIZE];
static uint8_t * const host_msg_payload = &host_msg_buf[UART_MSG_HEADER_SIZE];

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

void send_voltage(uint16_t voltage)
{
    unsigned payload_len = 0;

    UART_begin_transmission();

    host_msg_payload[payload_len++] = voltage & 0xFF;
    host_msg_payload[payload_len++] = (voltage >> 8) & 0xFF;

    send_msg_to_host(USB_RSP_VOLTAGE, payload_len);
}

void send_return_code(unsigned code)
{
    unsigned payload_len = 0;
    UART_begin_transmission();
    host_msg_payload[payload_len++] = code;
    send_msg_to_host(USB_RSP_RETURN_CODE, payload_len);
}

void send_interrupt_context(interrupt_context_t *int_context)
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

void send_param(param_t param)
{
    unsigned payload_len = 0;
    UART_begin_transmission();

    host_msg_payload[payload_len++] = param & 0xff;
    host_msg_payload[payload_len++] = (param >> 8) & 0xff;

    payload_len += get_param(param, &host_msg_payload[payload_len]);

    send_msg_to_host(USB_RSP_PARAM, payload_len);
}

void send_echo(uint8_t value)
{
    unsigned payload_len = 0;
    UART_begin_transmission();
    host_msg_payload[payload_len++] = value;
    send_msg_to_host(USB_RSP_ECHO, payload_len);
}

void send_payload(payload_t *payload)
{
    // The '*payload*' variables here refer to the payload of the message that
    // is being sent to host.  The argument 'payload' is just happens to be
    // also called payload.

    unsigned payload_len = 0;
    UART_begin_transmission();

    memcpy(host_msg_payload, payload, sizeof(payload_t));
    payload_len += sizeof(payload_t);

    send_msg_to_host(USB_RSP_ENERGY_PROFILE, payload_len);
}

void forward_msg_to_host(unsigned descriptor, uint8_t *buf, unsigned len)
{
    unsigned payload_len = 0;

    UART_begin_transmission();

    while (len--) {
        host_msg_payload[payload_len] = buf[payload_len];
        payload_len++;
    }

    send_msg_to_host(descriptor, payload_len);
}
