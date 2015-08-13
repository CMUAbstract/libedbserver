#include <msp430.h>
#include <stdint.h>

#include "config.h"
#include "pin_assign.h"
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
		TA2CTL |= TACLR | CONFIG_TIMELOG_TIMER_SOURCE | TIMER_DIV_BITS(CONFIG_TIMELOG_TIMER_DIV);
		TA2EX0 |= TIMER_A_DIV_EX_BITS(CONFIG_TIMELOG_TIMER_DIV_EX);

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
