/*******************************************************************************
 * @file
 * @date        14 April 2015
 * @author      Graham Harvey
 * @brief       Functions for WISP monitor time logging.
 * @details		In order to get useful information from the WISP debugger,
 * 				we will need time data.  Time is recorded in SMCLK cycles.
 * 				The 16-bit timer count register will overflow every few ms,
 * 				depending on clock frequency.  By design, it is the
 * 				responsibility of the data collector on the other side of
 * 				the serial port to account for these overflows.  This is done
 * 				because serial communication is the major speed limitation
 * 				here, so we don't want to have to send more time data.
 * 				The functions in this file allow a request to be made to log
 * 				time data.  When the request is made, timer 2 starts, and a
 * 				flag in the main loop is set every time the counter register
 * 				overflows.  In the main loop, time can then be logged somehow,
 * 				so that every overflow is caught, and real time can eventually
 * 				be calculated.
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>

#include "monitor.h"
#include "timeLog.h"

static uint32_t overflowCycles = 0;

void TimeLog_request(uint8_t request)
{
	static uint8_t time_log_requests = 0;

	if(request) {
		// request for the time

		// prevent 8-bit overflow
		if(time_log_requests != 0xFF) {
			time_log_requests++;
		}

		overflowCycles = 0;

		// start relative timer
		// SMCLK, continuous mode, clear TAR, enable interrupt
		TA2CTL = TASSEL__SMCLK + MC__CONTINUOUS + TACLR + TAIE;
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

uint32_t getTime()
{
	return TA2R + overflowCycles;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A1_VECTOR))) TIMER2_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
	switch(__even_in_range(TA2IV, 14))
	{
	case TA2IV_NONE:
	case TA2IV_TACCR1:
	case TA2IV_TACCR2:
	case TA2IV_3:
	case TA2IV_4:
	case TA2IV_5:
	case TA2IV_6:
		break;
	case TA2IV_TAIFG:
		// overflow
		overflowCycles += 0x00010000;
		break;
	default:
		break;
	}
}
