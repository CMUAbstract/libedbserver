#include <msp430.h>
#include <stdlib.h>

#include <libmsp/periph.h>

#include "config.h"
#include "pin_assign.h"
#include "error.h"

#include "sched.h"

#define TIMER_SCHED CONCAT(TIMER_SCHED_TYPE, TIMER_SCHED_IDX)
#define TMRCC_SCHED TIMER_SCHED_CCR // legacy naming scheme

/** @brief Most-recently scheduled action */
static action_t *sched_action;
static unsigned sched_action_interval;

/** @brief Preempted scheduled action and its interval */
static action_t *preempted_action;
static unsigned preempted_action_interval;

/** @brief Reschedule preempted action if any (internal use only) */
static void reschedule_preempted_action()
{
    if (preempted_action != NULL) {
        action_t *action = preempted_action;
        preempted_action = NULL;
        schedule_action(action, preempted_action_interval);
    }
}

void schedule_action(action_t *action, unsigned interval)
{
    TIMER(TIMER_SCHED, CTL) = 0;

    // Save the event that is currently being scheduled (may be NONE), and is
    // therefore preempted by this call to schedule.
    ASSERT(ASSERT_NESTED_SCHED_ACTION, preempted_action == NULL);
    preempted_action = sched_action;
    preempted_action_interval = sched_action_interval;

    // Schedule this event: when ISR fires, it will switch on the action
    sched_action = action;
    sched_action_interval = interval;

    // TODO: make it one-shot
    TIMER_CC(TIMER_SCHED, TMRCC_SCHED, CCR) = interval;
    TIMER(TIMER_SCHED, CTL) |= TACLR | TASSEL__ACLK | ID__8;
    TIMER_CC(TIMER_SCHED, TMRCC_SCHED, CCTL) &= ~CCIFG;
    TIMER_CC(TIMER_SCHED, TMRCC_SCHED, CCTL) |= CCIE;

    TIMER(TIMER_SCHED, CTL) |= MC__UP; // start
}

void abort_action(action_t *action)
{
    ASSERT(ASSERT_SCHED_ACTION_MISMATCH,
           action == sched_action || action == preempted_action);

    TIMER(TIMER_SCHED, CTL) = 0;
    sched_action = NULL;

    reschedule_preempted_action();
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER_VECTOR(TIMER_SCHED_TYPE, TIMER_SCHED_IDX, TMRCC_SCHED)
__interrupt void TIMER_ISR(TIMER_SCHED_TYPE, TIMER_SCHED_IDX, TIMER_SCHED_CCR)(void)
#elif defined(__GNUC__)
__attribute__ ((interrupt(TIMER_VECTOR(TIMER_SCHED_TYPE, TIMER_SCHED_IDX, TIMER_SCHED_CCR))))
void TIMER_ISR(TIMER_SCHED_TYPE, TIMER_SCHED_IDX, TIMER_SCHED_CCR)(void)
#else
#error Compiler not supported!
#endif
{
    TIMER(TIMER_SCHED, CTL) = MC__STOP;

    if (sched_action != NULL) {

        // At this point, the one-shot timer event has taken place (and is
        // about to be handled). Calls to 'schedule_action' are not
        // "preempting" a prior scheduled event, but are setting the
        // timer as if for the first time -- for this to be the case,
        // we need to reset the current timer action.
        action_t *action = sched_action;
        sched_action = NULL;

        sched_cmd_t cmd = action();
        if (cmd & SCHED_CMD_RESCHEDULE)
            schedule_action(action, sched_action_interval);
        if (cmd & SCHED_CMD_WAKEUP)
            __bic_SR_register_on_exit(LPM3_bits); // LPM3_bits covers all sleep states

        reschedule_preempted_action();
    }

    // TODO: Does this clear IFG mess up any possible timer starts above?
    TIMER_CC(TIMER_SCHED, TMRCC_SCHED, CCTL) &= ~CCIFG;
}
