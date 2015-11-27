#include <string.h>

#include "config.h"

#include "profile.h"

void profile_reset(profile_t *profile)
{
    memset(profile, 0, sizeof(profile_t));
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
