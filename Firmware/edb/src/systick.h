#ifndef SYSTICK_H
#define SYSTICK_H

#include <msp430.h>
#include <stdint.h>

#include "config.h"

#ifdef CONFIG_SYSTICK_32BIT
extern uint32_t ticks;
#endif

/**
 * @brief	Get the current value of the timer counter register (16-bit timestamp)
 */
#ifdef CONFIG_SYSTICK_32BIT
#define SYSTICK_CURRENT_TIME (ticks | TA2R)
#else
#define SYSTICK_CURRENT_TIME (TA2R)
#endif

/**
 * @brief	Start/stop main system timer 
 */
void systick_start();
void systick_stop();
void systick_reset();

#endif // SYSTICK_H
