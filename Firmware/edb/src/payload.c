#include <msp430.h>
#include <string.h>

#include "config.h"
#include "pin_assign.h"
#include "error.h"
#include "main_loop.h"
#include "host_comm_impl.h"

#include "payload.h"

static payload_t payload; // EDB+App data sent to host/ground

void payload_init()
{
#ifdef CONFIG_ENABLE_ENERGY_PROFILE
    profile_reset(&payload.energy_profile);
#endif
    memset(&payload.app_output, 0, sizeof(payload.app_output));
}

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

void payload_send()
{
#ifdef CONFIG_HOST_UART
    // TODO: for now we send the profile to host, in sprite this would
    // be a call to the radio module
    send_payload(&payload);
#else
    // Well, ... do nothing for now
#endif
}

void payload_record_profile_event(unsigned index, uint16_t vcap)
{
    profile_event(&payload.energy_profile, index, vcap);
}

void payload_record_app_output(const uint8_t *data, unsigned len)
{
    ASSERT(ASSERT_APP_OUTPUT_BUF_OVERFLOW, len < sizeof(payload.app_output));
    memcpy(&payload.app_output, data, len);
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
