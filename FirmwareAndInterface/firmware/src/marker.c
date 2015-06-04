#include <msp430.h>
#include <stdint.h>

#define READ_PERIOD_SMCLK_CYCLES 4

uint8_t marker_port_val = 0;

void marker_monitor_begin()
{
	TA1CCR0 = READ_PERIOD_SMCLK_CYCLES; // set timer compare register
    TA1CCTL0 = CCIE; // enable CCR0 interrupt
    TA1CTL = TASSEL__SMCLK + MC__UP + TACLR; // SMCLK, up mode, clear TAR
}

void marker_monitor_end()
{
    TA1CCR0 &= ~MC__STOP;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER3_A1_VECTOR
__interrupt void TIMER3_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER3_A1_VECTOR))) TIMER3_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    marker_port_val = P1IN;
}
