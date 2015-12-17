#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <libedb/target_comm.h>

#ifdef CONFIG_COLLECT_ENERGY_PROFILE
#include "profile.h"
#endif

/*
 * @brief Aggregate data packet sent from sprite to host/ground
 */
typedef struct {
#ifdef CONFIG_COLLECT_ENERGY_PROFILE
    profile_t energy_profile;
#endif
#ifdef CONFIG_COLLECT_APP_OUTPUT
    uint8_t app_output[APP_OUTPUT_SIZE];
#endif
} payload_t;

void payload_init();
void payload_send_beacon();
void payload_send_profile();
void payload_send_app_output();
void payload_send(); // do no use, it's too big

#ifdef CONFIG_COLLECT_ENERGY_PROFILE
void payload_record_profile_event(unsigned index, uint16_t vcap);
#endif
#ifdef CONFIG_COLLECT_APP_OUTPUT
void payload_record_app_output(const uint8_t *data, unsigned len);
#endif

#endif // PAYLOAD_H
