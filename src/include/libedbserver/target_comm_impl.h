#ifndef TARGET_COMM_IMPL_H
#define TARGET_COMM_IMPL_H

#include <stdint.h>
#include <stdbool.h>

#include "uart.h"

extern uartPkt_t wispRxPkt;

void target_comm_send_breakpoint(uint8_t index, bool enable);
void target_comm_send_get_pc();
void target_comm_send_get_interrupt_context();
void target_comm_send_read_mem(uint32_t address, unsigned len);
void target_comm_send_write_mem(uint32_t address, uint8_t *value, unsigned len);
void target_comm_send_exit_debug_mode();
void target_comm_send_echo(uint8_t value);
void target_comm_send_get_app_output();

#endif // TARGET_COMM_IMPL_H
