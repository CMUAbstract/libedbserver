/*******************************************************************************
 * @file
 * @date        14 April 2015
 * @author      Graham Harvey
 * @brief       Prototypes and definitions for WISP monitor time logging.
 ******************************************************************************/

#ifndef TIMELOG_H
#define TIMELOG_H

#include <msp430.h>
#include <stdint.h>

extern uint16_t overflowCycles;

/**
 * @brief	Get the current timer register value + overflow cycles up to 32 bits.
 * @return	32-bit value representing the number of cycles since the timer was started
 */
#define TIMELOG_CURRENT_TIME (((uint32_t)overflowCycles << 16) | TA2R)

/**
 * @brief	Request a timer for UART logging purposes
 * @param	request	1 to request timer to be turned on,
 * 			0 to indicate timer is no longer needed for this purpose
 */
void TimeLog_request(uint8_t request);

#endif // TIMELOG_H
