/*******************************************************************************
 * @file
 * @date        14 April 2015
 * @author      Graham Harvey
 * @brief       Functions for WISP monitor time logging.
 * @details		In order to get useful information from the WISP debugger,
 * 				we will need time data.  Time is recorded in SMCLK cycles.
 * 				The 16-bit timer count register will overflow every few ms,
 * 				depending on clock frequency.  When this happens, those
 * 				cycles are added to a 32-bit overflow variable here.  Using
 * 				the default SMCLK of 21921792 Hz, those 32 bits will overflow
 * 				approximately every 196 seconds.  Beyond this, it is the
 * 				responsibility of the data collector on the other side of
 * 				the serial port to account for these overflows.  This is done
 * 				because serial communication is a major factor limiting speed
 * 				here, so we don't want to have to send more time data.
 * 				The functions in this file allow a request to be made to log
 * 				time data.  When the request is made, Timer2 starts.  The
 * 				macro TIMELOG_CURRENT_TIME can then be called to get the 32-bit time
 * 				in SMCLK cycles.
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>

#include "config.h"
#include "timeLog.h"

void TimeLog_request(unsigned request)
{
	static unsigned time_log_requests = 0;

	if(request) {
		// request for the time

		// prevent 8-bit overflow
		if(time_log_requests != 0xFF) {
			time_log_requests++;
		}

		// configure relative timer
		TA2CTL |= TACLR | CONFIG_TIMELOG_TIMER_SOURCE | CONFIG_TIMELOG_TIMER_DIV_BITS;
		TA2EX0 |= CONFIG_TIMELOG_TIMER_DIV_BITS_EX;

		// continuous mode, clear TAR, enable interrupt
		TA2CTL |= MC__CONTINUOUS; // start
	} else {
		// don't subtract from unsigned 0
		if(time_log_requests != 0) {
			time_log_requests--;
			if(time_log_requests == 0) {
				TA2CTL = 0; // stop timer
			}
		}
	}
}
