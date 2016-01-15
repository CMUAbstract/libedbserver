#ifndef CODEPOINT_H
#define CODEPOINT_H

#include <stdint.h>
#include <stdbool.h>

#include "host_comm.h"

extern uint16_t code_energy_breakpoints; // exposed for comparator ISR

void set_external_breakpoint_pin_state(uint16_t bitmask, bool state);

unsigned toggle_breakpoint(breakpoint_type_t type, unsigned index,
                           uint16_t energy_level, comparator_ref_t cmp_ref,
                           bool enable);
unsigned toggle_watchpoint(unsigned index, bool enable, bool vcap_snapshot);

void enable_watchpoints();
void disable_watchpoints();

void watchpoints_start_stream();
void watchpoints_stop_stream();

void init_watchpoint_event_bufs();
void send_watchpoint_events();

void handle_codepoint(unsigned index);

#endif // CODEPOINT_H
