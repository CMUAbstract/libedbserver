#include <msp430.h>
#include <string.h>

#include <libio/log.h>

#include "config.h"
#include "pin_assign.h"
#include "error.h"
#include "main_loop.h"
#include "host_comm_impl.h"

#include "payload.h"

static payload_t payload; // EDB+App data sent to host/ground

void payload_init()
{
#ifdef CONFIG_COLLECT_ENERGY_PROFILE
    profile_reset(&payload.energy_profile);
#endif
#ifdef CONFIG_COLLECT_APP_OUTPUT
    memset(&payload.app_output, 0, sizeof(payload.app_output));
#endif
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
#if defined(CONFIG_HOST_UART)
    // TODO: for now we send the profile to host, in sprite this would
    // be a call to the radio module
    send_payload(&payload);
#elif defined(CONFIG_DEV_CONSOLE)
    int i;
    BLOCK_LOG_BEGIN();
    BLOCK_LOG("payload:\r\n");
    for (i = 0; i < sizeof(payload_t); ++i) {
        BLOCK_LOG("%02x ", *((uint8_t *)&payload + i));

        if (((i + 1) & (8 - 1)) == 0)
            BLOCK_LOG("\r\n");
    }
    BLOCK_LOG("\r\n");
    BLOCK_LOG_END();
#else
    // Well, ... do nothing for now
#endif
}

#ifdef CONFIG_COLLECT_ENERGY_PROFILE
void payload_record_profile_event(unsigned index, uint16_t vcap)
{
    profile_event(&payload.energy_profile, index, vcap);
}
#endif // CONFIG_COLLECT_ENERGY_PROFILE

#ifdef CONFIG_COLLECT_APP_OUTPUT
void payload_record_app_output(const uint8_t *data, unsigned len)
{
    ASSERT(ASSERT_APP_OUTPUT_BUF_OVERFLOW, len <= sizeof(payload.app_output));
    memcpy(&payload.app_output, data, len);
}
#endif // CONFIG_COLLECT_APP_OUTPUT

#ifdef CONFIG_ENABLE_PAYLOAD
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) TIMER1_A0_ISR (void)
#else
#error Compiler not supported!
#endif
#endif
{
    main_loop_flags |= FLAG_SEND_PAYLOAD
#ifdef CONFIG_COLLECT_APP_OUTPUT
        | FLAG_APP_OUTPUT
#endif
        ;

    // TODO: clear the sleep on exit flag
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCTL) &= ~CCIFG;
}
