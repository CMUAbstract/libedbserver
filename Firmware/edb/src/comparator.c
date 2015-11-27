#include <msp430.h>

#include "config.h"
#include "pin_assign.h"
#include "error.h"

#include "comparator.h"

#define COMP_NEG_CHAN_INNER(idx) CBIMSEL_ ## idx
#define COMP_NEG_CHAN(idx) COMP_NEG_CHAN_INNER(idx)

#define COMP_PORT_DIS_INNER(idx) CBPD ## idx
#define COMP_PORT_DIS(idx) COMP_PORT_DIS_INNER(idx)

comparator_op_t comparator_op = CMP_OP_NONE;

void arm_comparator(comparator_op_t op, uint16_t target, comparator_ref_t ref,
                           comparator_edge_t edge)
{
    comparator_op = op;

    // ref0 = ref1 = target = Vref / 2^32 * target_volts
    switch (ref) {
        case CMP_REF_VCC:
            CBCTL2 = CBRS_1;
            break;
        case CMP_REF_VREF_2_5:
            CBCTL2 = CBREFL_3 | CBRS_2; // Vref = 2.5V and Vref to resistor ladder
            break;
        case CMP_REF_VREF_2_0:
            CBCTL2 = CBREFL_2 | CBRS_2; // Vref = 2.0V and Vref to resistor ladder
            break;
        case CMP_REF_VREF_1_5:
            CBCTL2 = CBREFL_1 | CBRS_2; // Vref = 1.5V and Vref to resistor ladder
            break;
        default:
            error(ERROR_INVALID_VALUE);
            break;
    }
    CBCTL2 |= (target << 8) | target;

    CBCTL0 |= CBIMEN | COMP_NEG_CHAN(COMP_CHAN_VCAP); // route input pin to V-, input channel (pin)
    CBCTL3 |= COMP_PORT_DIS(COMP_CHAN_VCAP); // disable input port buffer on pin
    CBCTL1 |= CBF | CBFDLY_3;

    CBINT &= ~CBIE; // disable CompB interrupt

    CBCTL1 |= CBON;               // turn on ComparatorB

    switch (edge) {
        case CMP_EDGE_FALLING:
            CBCTL1 |= CBIES;
            break;
        case CMP_EDGE_RISING:
            CBCTL1 &= ~CBIES;
            break;
        case CMP_EDGE_ANY: // determine direction that would change current output
            if (CBCTL1 & CBOUT)
                CBCTL1 |= CBIES;
            else
                CBCTL1 &= ~CBIES;
            break;
        default:
            error(ERROR_INVALID_VALUE);
            break;
    }

    CBINT &= ~(CBIFG | CBIIFG);   // clear any errant interrupts
    CBINT |= CBIE;                // enable CompB interrupt
}

void disarm_comparator()
{
    CBINT &= ~CBIE; // disable interrupt
    CBCTL1 &= ~CBON; // turn off
}