#ifndef COMPARATOR_H
#define COMPARATOR_H

#include <stdint.h>

#include "host_comm.h"

#define COMP_NEG_CHAN_INNER(idx) CBIMSEL_ ## idx
#define COMP_NEG_CHAN(idx) COMP_NEG_CHAN_INNER(idx)

#define COMP_PORT_DIS_INNER(idx) CBPD ## idx
#define COMP_PORT_DIS(idx) COMP_PORT_DIS_INNER(idx)

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
void arm_comparator_impl(comparator_op_t op, uint16_t target,
                         comparator_ref_t ref, comparator_edge_t edge,
                         uint16_t ctl0_chan_bits, uint16_t ctl3_chan_bits);

#define arm_comparator(op, target, ref, edge, chan) \
  arm_comparator_impl(op, target, ref, edge, \
                      COMP_NEG_CHAN(chan), COMP_PORT_DIS(chan))

/**
 * @brief Disable the comparator
 */
void disarm_comparator();

#endif // COMPARATOR_H
