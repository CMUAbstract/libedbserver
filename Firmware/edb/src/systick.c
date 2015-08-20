#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "pin_assign.h"
#include "error.h"

#include "systick.h"

uint32_t ticks = 0;

void systick_start()
{
    // configure relative timer
    TA2CTL |= TACLR | CONFIG_TIMELOG_TIMER_SOURCE | TIMER_DIV_BITS(CONFIG_TIMELOG_TIMER_DIV);
    TA2EX0 |= TIMER_A_DIV_EX_BITS(CONFIG_TIMELOG_TIMER_DIV_EX);

    // continuous mode, clear TAR, enable interrupt
    TA2CTL |= MC__CONTINUOUS | TAIE; // start
}

void systick_stop()
{
	TA2CTL = 0; // stop timer
}

void systick_reset()
{
    TA2CTL |= TACLR;
    ticks = 0;
}

#ifdef CONFIG_SYSTICK_32BIT
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A1_VECTOR
__interrupt void TIMER2_A1_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A1_VECTOR))) TIMER2_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    ticks += 0x10000UL;
    TA2CTL &= ~TAIFG;
}
#endif // CONFIG_SYSTICK_32BIT
