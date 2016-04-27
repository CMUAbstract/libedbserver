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
 * @brief		Used to  change the PWM duty cycle
 * @{
 */
#define PWM_INCREASE_DUTY_CYCLE	TB0CCR1 = (TB0CCR1 < TB0CCR0) ? TB0CCR1 + 1 : TB0CCR0 //!< Increase PWM duty cycle by one SMCLK cycle
#define PWM_DECREASE_DUTY_CYCLE	TB0CCR1 = (TB0CCR1 > 0) ? TB0CCR1 - 1 : 0 //!< Decrease PWM duty cycle by one SMCLK cycle
/** @} End PWM_DUTY_CYCLE */

/**
 * @brief   Set up PWM for the WISP monitor
 */
void PWM_setup(uint16_t period, uint16_t duty_cycle);

/**
 * @brief   Start PWM
 */
void PWM_start();

/**
 * @brief   Stop PWM
 */
void PWM_stop();

void PWM_set_freq(uint16_t freq);
void PWM_set_duty_cycle(uint16_t duty_cycle);

/** @} End PWM */

#endif // PWM_H
