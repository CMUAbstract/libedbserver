#ifndef TETHER_H
#define TETHER_H

/**
 * Tethering state machine states
 */
typedef enum {
    STATE_IDLE = 0,
    STATE_ENTERING,
    STATE_DEBUG,
    STATE_EXITING,
    STATE_SERIAL_ECHO, // for testing purposes only
} state_t;

/** @brief Current state of the tethering state machine
 *  @ddetails TODO: rename to tether_state
 */
extern volatile state_t state;

#endif // TETHER_H
