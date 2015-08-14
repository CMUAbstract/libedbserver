#ifndef CODEPOINT_H
#define CODEPOINT_H

#include <stdint.h>
#include <stdbool.h>

#include "host_comm.h"

// Bitmasks indicate whether a breakpoint (group) of given index is enabled
extern uint16_t passive_breakpoints;
extern uint16_t external_breakpoints;
extern uint16_t internal_breakpoints;
extern uint16_t code_energy_breakpoints;

extern uint16_t watchpoints;

void set_external_breakpoint_pin_state(uint16_t bitmask, bool state);

void toggle_breakpoint(breakpoint_type_t type, unsigned index,
                              uint16_t energy_level, comparator_ref_t cmp_ref,
                              bool enable);
void toggle_watchpoint(unsigned index, bool enable, bool vcap_snapshot);

void enable_watchpoints();
void disable_watchpoints();

void init_watchpoint_event_bufs();
void append_watchpoint_event(unsigned index);
void send_watchpoint_events();

#endif // CODEPOINT_H
