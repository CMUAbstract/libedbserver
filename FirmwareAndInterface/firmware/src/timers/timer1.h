/*******************************************************************************
 * @file
 * @date            15 April 2015
 * @author          Graham Harvey
 * @brief           Prototypes and definitions for using Timer 1.
 ******************************************************************************/

#ifndef TIMER1_H
#define TIMER1_H

#include <msp430.h>
#include <stdint.h>

#define TIMER1_STOP				TA1CCTL0 = 0; \
								TA1CTL = 0	//!< disable timer interrupt and turn timer off

/**
 * @brief	Set Timer1 to expire after a given number of SMCLK cycles.
 * @param	smclk_cycles	The number of SMCLK cycles after which the timer will expire
 * @param	callbackFn		Callback function to call on timer expire.
 */
void Timer1_set(uint16_t smclk_cycles, void (*callbackFn)(void));

#endif // TIMER1_H
