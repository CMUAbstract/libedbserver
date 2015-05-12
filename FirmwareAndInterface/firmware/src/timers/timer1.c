/*******************************************************************************
 * @file
 * @date            22 April 2015
 * @author          Graham Harvey
 * @brief           Functions for using Timer 1.
 * @warning			Make sure to only use this timer to time one event
 * 					at a time.  As of 22 April 2015, the only use is for WISP
 * 					TX event logging to prevent multiple logs for the same TX
 * 					.  If the timer is used to time several
 * 					events at once, only the last call to Timer1_set will
 * 					result in a timer interrupt and call to the callback.
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include "timer1.h"

static void (*pCallbackFn)(void); // function pointer to callback function

void Timer1_set(uint16_t smclk_cycles, void (*callbackFn)(void))
{
	pCallbackFn = callbackFn;

	TA1CCTL0 = CCIE; // enable CCR0 interrupt
	TA1CCR0 = smclk_cycles;
	TA1CTL = TASSEL__SMCLK + MC__UP + TACLR; // SMCLK, up mode, clear TAR
}

// This vector is reserved for CCR0 interrupts
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	(*pCallbackFn)(); // call the callback
	TA1CTL = 0; // stop the timer so the interrupt isn't triggered again
}
