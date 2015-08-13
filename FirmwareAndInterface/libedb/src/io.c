#include <msp430.h>
#include <wisp-base.h>

#include <stdlib.h>

#include "target_comm.h"

#define STDIO_BUF_SIZE 128
#define STDIO_PAYLOAD_SIZE (STDIO_BUF_SIZE - UART_MSG_HEADER_SIZE)

static uint8_t stdio_buf[STDIO_BUF_SIZE];
static uint8_t *stdio_payload = &stdio_buf[UART_MSG_HEADER_SIZE];
static unsigned stdio_payload_len = 0;

int putc(int c, void *stream)
{
    unsigned msg_len = 0;
    
    stdio_payload[stdio_payload_len++] = (uint8_t)c;

    if (stdio_payload_len == STDIO_PAYLOAD_SIZE || c == '\n') { // flush on new line or full

        stdio_buf[msg_len++] = UART_IDENTIFIER_WISP;
        stdio_buf[msg_len++] = WISP_RSP_STDIO;
        stdio_buf[msg_len++] = stdio_payload_len;
        stdio_buf[msg_len++] = 0; // padding

        msg_len += stdio_payload_len;

        UART_send(stdio_buf, msg_len);

        stdio_payload_len = 0;
    }

    return c;
}

int puts(const char *ptr)
{
    unsigned len = 0;
    unsigned msg_len = 0;

    while (*ptr != '\0') {
        stdio_payload[len] = *ptr++;
        len++;
    }

    stdio_buf[msg_len++] = UART_IDENTIFIER_WISP;
    stdio_buf[msg_len++] = WISP_RSP_STDIO;
    stdio_buf[msg_len++] = len;
    stdio_buf[msg_len++] = 0; // padding

    msg_len += len;

    UART_send(stdio_buf, msg_len);

    return len;
}
