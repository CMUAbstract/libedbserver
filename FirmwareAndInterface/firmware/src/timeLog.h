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

/**
 * @brief	Get the current value of the timer counter register (16-bit timestamp)
 */
#define TIMELOG_CURRENT_TIME TA2R

/**
 * @brief	Request a timer for UART logging purposes
 * @param	request	1 to request timer to be turned on,
 * 			0 to indicate timer is no longer needed for this purpose
 */
void TimeLog_request(unsigned request);

#endif // TIMELOG_H
