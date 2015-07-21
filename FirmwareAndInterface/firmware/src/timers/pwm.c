/*******************************************************************************
 * @file
 * @date        21 March 2015
 * @author      Graham Harvey
 * @brief       Functions for WISP power monitoring.
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include "pin_assign.h"
#include "config.h"

#include "pwm.h"

#ifdef CONFIG_PWM_CHARGING

void PWM_setup(uint16_t period, uint16_t duty_cycle)
{
    TB0CCR0 = period;                   // PWM period
    TB0CCTL1 = OUTMOD_7;                // CCR1 reset/set
    TB0CCR1 = duty_cycle;               // duty cycle
}

void PWM_start()
{
    GPIO(PORT_CHARGE, SEL) |= BIT(PIN_CHARGE);               // PWM option select
    GPIO(PORT_CHARGE, DIR) |= BIT(PIN_CHARGE);               // pin output direction

    TB0CTL = TBSSEL__SMCLK + MC__UP + TBCLR;   // SMCLK, up mode, clear TBR
}

void PWM_stop()
{
    TB0CTL = MC_0;

    GPIO(PORT_CHARGE, OUT) &= ~BIT(PIN_CHARGE);				// output low
    GPIO(PORT_CHARGE, SEL) &= ~BIT(PIN_CHARGE);              // GPIO option select
}

#else
unsigned __dummy_pwm_c; // silence 'empty translation unit' warning
#endif // CONFIG_PWM_CHARGING
