#include <msp430.h>
#include <string.h>

#include "config.h"
#include "pin_assign.h"

#include "profile.h"

void profile_reset(profile_t *profile)
{
    memset(profile, 0, sizeof(profile_t));
}

void profile_start_send_timer()
{
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCR) =
        CONFIG_SEND_ENERGY_PROFILE_INTERVAL;
    TIMER(TIMER_SEND_ENERGY_PROFILE, CTL) |= TACLR | TASSEL__ACLK;
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCTL) &= ~CCIFG;
    TIMER_CC(TIMER_SEND_ENERGY_PROFILE, TMRCC_SEND_ENERGY_PROFILE, CCTL) |= CCIE;

    TIMER(TIMER_SEND_ENERGY_PROFILE, CTL) |= MC__UP; // start
}

void profile_stop_send_timer()
{
    TIMER(TIMER_SEND_ENERGY_PROFILE, CTL) = 0;
}

void profile_event(profile_t *profile, unsigned index, uint16_t vcap)
{
    unsigned quantum_idx = vcap / NUM_ENERGY_QUANTA;
    unsigned byte_idx = quantum_idx / NUM_ENERGY_QUANTA_PER_BYTE;
    unsigned slot_idx = quantum_idx % NUM_ENERGY_QUANTA_PER_BYTE;

    unsigned e_byte = profile->events[index].energy[byte_idx];
    unsigned shift = slot_idx * NUM_ENERGY_BITS_PER_QUANTUM;
    unsigned slot_mask = ENERGY_QUANTUM_MASK << shift;
    unsigned e_slot = (e_byte & slot_mask) >> shift;
    e_slot++;
    e_byte = (e_byte & ~slot_mask) | (e_slot << shift);

    profile->events[index].count++;
    profile->events[index].energy[byte_idx] = e_byte;
}
