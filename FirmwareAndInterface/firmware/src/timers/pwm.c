/*******************************************************************************
 * @file
 * @date        21 March 2015
 * @author      Graham Harvey
 * @brief       Functions for WISP power monitoring.
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include "pwm.h"
#include "monitor.h"

void PWM_setup(uint16_t period, uint16_t duty_cycle)
{
    TB0CCR0 = period;                   // PWM period
    TB0CCTL1 = OUTMOD_7;                // CCR1 reset/set
    TB0CCR1 = duty_cycle;               // duty cycle
}

void PWM_start()
{
    P5SEL |= WISP_CHARGE;               // PWM option select
    P5DIR |= WISP_CHARGE;               // pin output direction

    TB0CTL = TBSSEL__SMCLK + MC__UP + TBCLR;   // SMCLK, up mode, clear TBR
}

void PWM_stop()
{
    TB0CTL = MC_0;

    P5OUT &= ~WISP_CHARGE;				// output low
    P5SEL &= ~WISP_CHARGE;              // GPIO option select
}
