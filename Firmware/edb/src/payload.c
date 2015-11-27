#include <msp430.h>

#include "config.h"
#include "pin_assign.h"
#include "main_loop.h"

#include "payload.h"

void payload_start_send_timer()
{
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCR) =
        CONFIG_SEND_ENERGY_PROFILE_INTERVAL;
    TIMER(TIMER_SEND_ENERGY_PROFILE, CTL) |= TACLR | TASSEL__ACLK;
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCTL) &= ~CCIFG;
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCTL) |= CCIE;

    TIMER(TIMER_SEND_ENERGY_PROFILE, CTL) |= MC__UP; // start
}

void payload_stop_send_timer()
{
    TIMER(TIMER_SEND_ENERGY_PROFILE, CTL) = 0;
}

#ifdef CONFIG_ENABLE_ENERGY_PROFILE
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
#endif
{
    main_loop_flags |= FLAG_SEND_PAYLOAD;
    // TODO: clear the sleep on exit flag
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCTL) &= ~CCIFG;
}
