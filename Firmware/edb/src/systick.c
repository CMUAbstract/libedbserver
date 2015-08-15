#include <msp430.h>
#include <stdint.h>

#include "config.h"
#include "pin_assign.h"

#include "systick.h"

void systick_start()
{
    // configure relative timer
    TA2CTL |= TACLR | CONFIG_TIMELOG_TIMER_SOURCE | TIMER_DIV_BITS(CONFIG_TIMELOG_TIMER_DIV);
    TA2EX0 |= TIMER_A_DIV_EX_BITS(CONFIG_TIMELOG_TIMER_DIV_EX);

    // continuous mode, clear TAR, enable interrupt
    TA2CTL |= MC__CONTINUOUS; // start
}

void systick_stop()
{
	TA2CTL = 0; // stop timer
}
