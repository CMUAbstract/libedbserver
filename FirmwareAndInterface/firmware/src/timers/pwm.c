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

void PWM_setup()
{
    TB0CCR0 = 16-1;                     // PWM period
    TB0CCTL1 = OUTMOD_7;                // CCR1 reset/set
    TB0CCR1 = 8;                        // duty cycle
}

void PWM_start()
{
    P5SEL |= WISP_CHARGE;               // PWM option select
    P5DIR |= WISP_CHARGE;               // pin output direction

    TB0CTL = TASSEL_2 + MC_1 + TBCLR;   // SMCLK, up mode, clear TBR
}

void PWM_stop()
{
    TB0CTL = MC_0;

    P5OUT &= ~WISP_CHARGE;				// output low
    P5SEL &= ~WISP_CHARGE;              // GPIO option select
}

void PWM_changeDutyCycle(uint8_t change)
{
    if(change == PWM_INCREASE_DUTY_CYCLE) {
        if(TB0CCR1 < TB0CCR0) {
            TB0CCR1 += 1;
        }
    } else {
        if(TB0CCR1 > 0) {
            TB0CCR1 -= 1;
        }
    }
}
