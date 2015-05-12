/*******************************************************************************
 * @file
 * @date        21 March 2015
 * @author      Graham Harvey
 * @brief       Definitions and prototypes for PWM on the WISP debugger.
 ******************************************************************************/

#ifndef PWM_H
#define PWM_H

#include <msp430.h>
#include <stdint.h>

/**
 * @defgroup    PWM  WISP power manipulation
 * @brief       Functions that can be used to get or set the WISP power level.
 * @{
 */

/**
 * @defgroup	PWM_DUTY_CYCLE		Change PWM duty cycle
 * @brief		Used to indicate how to change the PWM duty cycle
 * @{
 */

#define PWM_INCREASE_DUTY_CYCLE				1 //!< Increase PWM duty cycle
#define PWM_DECREASE_DUTY_CYCLE				0 //!< Decrease PWM duty cycle

/** @} End PWM_DUTY_CYCLE */

/**
 * @brief   Set up PWM for the WISP monitor
 */
void PWM_setup();

/**
 * @brief   Start PWM
 */
void PWM_start();

/**
 * @brief   Stop PWM
 */
void PWM_stop();

/**
 * @brief   If possible, change PWM duty cycle
 * @param   change		Indicates how to change the duty cycle.  See @ref PWM_DUTY_CYCLE.
 */
void PWM_changeDutyCycle(uint8_t change);

/** @} End PWM */

#endif // PWM_H
