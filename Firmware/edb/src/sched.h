#ifndef SCHED_H
#define SCHED_H

/** @brief Schedulable actions can return one of these to reschedule self */
typedef enum {
    SCHED_CMD_NONE          = 1 << 0,
    SCHED_CMD_RESCHEDULE    = 1 << 1, // schedule same action again
    SCHED_CMD_WAKEUP        = 1 << 2, // exit from low-power state
} sched_cmd_t;

/** @brief A schedulable action is a void function */
typedef sched_cmd_t (action_t)(void);


/** @brief Schedule an action to run at a later time
 *  @param action   Enum identifying the predefined action
 *  @param interval Clock cycles
 *  @details This is highly approximate.
 *
 *           This supports rudimentary "preemption": an action can be scheduled
 *           even if another one has been scheduled.  In this case, when the
 *           inner action completes, the outer action is rescheduled (from the
 *           beginning). Only one level of "nesting" is supported.
 *
 *           The point of this is to consume only one hardware timer.
 */
void schedule_action(action_t *action, unsigned interval);

/** @brief De-schedule the currently scheduled action, if any
 *  @details Resumes the preempted action, if any. In this sense,
 *           what happens on abort is analogous to what happens on timer
 *           expiration, except that the action is never performed.
 * */
void abort_action(action_t *action);

#endif // SCHED_H
