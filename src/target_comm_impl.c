#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>

#include <libedb/target_comm.h>

#include "uart.h"

#include "target_comm_impl.h"

#define TARGET_MSG_BUF_SIZE   16 // buffer for UART messages (to target) for main loop

#if TARGET_MSG_BUF_SIZE < WISP_CMD_MAX_LEN
#error Buffer for UART messages to target is too small: TARGET_MSG_BUF_SIZE < WISP_CMD_MAX_LEN
#endif

/**
 * @brief Message payload pointer in a buffer used exclusively by main loop
 * @details This buffer is used exclusively by main loop, so it is
 *          shared only in the sense of being multi-plexed in time, i.e. it is
 *          never used concurrently but to threads of control.
 */
static uint8_t target_msg_buf[TARGET_MSG_BUF_SIZE];
static uint8_t * const target_msg_payload = &target_msg_buf[UART_MSG_HEADER_SIZE];

uartPkt_t wispRxPkt = { .processed = 1 };

void target_comm_send_breakpoint(uint8_t index, bool enable)
{
    unsigned payload_len = 0;
    target_msg_payload[payload_len++] = index;
    target_msg_payload[payload_len++] = enable ? 0x1 : 0x0;

    UART_send_msg_to_target(WISP_CMD_BREAKPOINT, payload_len, target_msg_buf);
}

void target_comm_send_get_pc()
{
    UART_send_msg_to_target(WISP_CMD_GET_PC, 0, target_msg_buf);
}

void target_comm_send_get_interrupt_context()
{
    UART_send_msg_to_target(WISP_CMD_GET_INTERRUPT_CONTEXT, 0, target_msg_buf);
}

void target_comm_send_exit_debug_mode()
{
    UART_send_msg_to_target(WISP_CMD_EXIT_ACTIVE_DEBUG, 0, target_msg_buf);
}

void target_comm_send_read_mem(uint32_t address, unsigned len)
{
    unsigned payload_len = 0;
    target_msg_payload[payload_len++] = (address >> 0) & 0xff;
    target_msg_payload[payload_len++] = (address >> 8) & 0xff;
    target_msg_payload[payload_len++] = (address >> 16) & 0xff;
    target_msg_payload[payload_len++] = (address >> 24) & 0xff;
    target_msg_payload[payload_len++] = len;

    UART_send_msg_to_target(WISP_CMD_READ_MEM, payload_len, target_msg_buf);
}

void target_comm_send_write_mem(uint32_t address, uint8_t *value, unsigned len)
{
    unsigned i;
    unsigned payload_len = 0;

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
}

void target_comm_send_echo(uint8_t value)
{
    unsigned payload_len = 0;
    target_msg_payload[payload_len++] = value;
    UART_send_msg_to_target(WISP_CMD_SERIAL_ECHO, payload_len, target_msg_buf);
}

void target_comm_send_get_app_output()
{
    UART_send_msg_to_target(WISP_CMD_GET_APP_OUTPUT, 0, target_msg_buf);
}
