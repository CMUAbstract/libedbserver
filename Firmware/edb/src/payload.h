#ifndef PAYLOAD_H
#define PAYLOAD_H

#include <libedb/target_comm.h>

#include "profile.h"

/*
 * @brief Aggregate data packet sent from sprite to host/ground
 */
typedef struct {
    profile_t energy_profile;
    uint8_t app_output[APP_OUTPUT_SIZE];
} payload_t;

void payload_init();
void payload_send();

void payload_record_profile_event(unsigned index, uint16_t vcap);
void payload_record_app_output(const uint8_t *data, unsigned len);

void payload_start_send_timer();
void payload_stop_send_timer();

#endif // PAYLOAD_H
