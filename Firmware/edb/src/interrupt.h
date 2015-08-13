#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <libedb/target_comm.h>

typedef struct {
    interrupt_type_t type;
    unsigned id;
    uint16_t saved_vcap;
    uint16_t restored_vcap;
    uint16_t saved_debug_mode_flags; // for nested debug mode
} interrupt_context_t;

#endif // INTERRUPT_H
