#include <string.h>

#include "config.h"

#include "profile.h"

void profile_reset(profile_t *profile)
{
    memset(profile, 0, sizeof(profile_t));
}

void profile_event(profile_t *profile, unsigned index, uint16_t vcap)
{
    profile->events[index].count++;

#ifdef CONFIG_PROFILE_SUB_BYTE_BUCKET_SIZES
    unsigned quantum_idx = vcap / NUM_ENERGY_QUANTA; // this is wrong (see byte-based version below)
    unsigned byte_idx = quantum_idx / NUM_ENERGY_QUANTA_PER_BYTE;
    unsigned slot_idx = quantum_idx % NUM_ENERGY_QUANTA_PER_BYTE;

    unsigned e_byte = profile->events[index].energy[byte_idx];
    unsigned shift = slot_idx * NUM_ENERGY_BITS_PER_QUANTUM;
    unsigned slot_mask = ENERGY_QUANTUM_MASK << shift;
    unsigned e_slot = (e_byte & slot_mask) >> shift;
    e_slot++;
    e_byte = (e_byte & ~slot_mask) | (e_slot << shift);

    profile->events[index].energy[byte_idx] = e_byte;
#else
    // Split the range [MIN_VOLTAGE, VREF (2.5v)] into NUM_ENERGY_BYTES buckets
    // (one-byte per bucket), and count the values in each bucket.

    unsigned byte_idx = (((float)vcap - CONFIG_ENERGY_PROFILE_MIN_VOLTAGE) /
                        ((1 << 12) - CONFIG_ENERGY_PROFILE_MIN_VOLTAGE)) * NUM_ENERGY_BYTES;
    if (byte_idx == NUM_ENERGY_BYTES)
        --byte_idx;

    uint8_t e = profile->events[index].energy[byte_idx];

    e++;

    if (e > 0) {
        profile->events[index].energy[byte_idx] = e;
    } else { // bucket overflowed, reset all buckets, to keep histogram consistent
        for (int i = 0; i < NUM_ENERGY_BYTES; ++i)
            profile->events[index].energy[i] = 0;
        profile->events[index].count = 0;
    }

#endif // CONFIG_PROFILE_SUB_BYTE_BUCKET_SIZES
}
