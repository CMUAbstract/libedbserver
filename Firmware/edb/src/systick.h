#ifndef SYSTICK_H
#define SYSTICK_H

#include <msp430.h>
#include <stdint.h>

/**
 * @brief	Get the current value of the timer counter register (16-bit timestamp)
 */
#define SYSTICK_CURRENT_TIME TA2R

/**
 * @brief	Start/stop main system timer 
 */
void systick_start();
void systick_stop();

#endif // SYSTICK_H
