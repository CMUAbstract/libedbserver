#ifndef COMPARATOR_H
#define COMPARATOR_H

#include <stdint.h>

#include "host_comm.h"

/**
 * @brief State of async charge/discharge operation for multiplexing
 */
typedef enum {
    CMP_OP_NONE = 0,
    CMP_OP_CHARGE,
    CMP_OP_DISCHARGE,
    CMP_OP_ENERGY_BREAKPOINT,
    CMP_OP_CODE_ENERGY_BREAKPOINT,
    CMP_OP_RESET_STATE_ON_BOOT,
} comparator_op_t;

typedef enum {
    CMP_EDGE_ANY,
    CMP_EDGE_FALLING,
    CMP_EDGE_RISING,
} comparator_edge_t;

/** @brief Async comparator operation currently in progress */
extern comparator_op_t comparator_op;

/**
 * @brief Configure comparator to watch for events
 */
void arm_comparator(comparator_op_t op, uint16_t target,
                    comparator_ref_t ref, comparator_edge_t edge);

/**
 * @brief Disable the comparator
 */
void disarm_comparator();

#endif // COMPARATOR_H
