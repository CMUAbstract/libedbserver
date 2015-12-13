#include <msp430.h>
#include <string.h>

#ifdef CONFIG_DEV_CONSOLE
#include <libio/log.h>
#endif

#ifdef CONFIG_RADIO_TRANSMIT_PAYLOAD
#include <libsprite/SpriteRadio.h>
#endif

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

// TODO: move to main, because this is too major
#ifdef CONFIG_RADIO_TRANSMIT_PAYLOAD
    SpriteRadio_SpriteRadio();
#endif
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

#ifdef CONFIG_RADIO_TRANSMIT_PAYLOAD
    SpriteRadio_txInit();
    SpriteRadio_transmit((char *)&payload, sizeof(payload_t));
    SpriteRadio_sleep();
#endif // CONFIG_RADIO_TRANSMIT_PAYLOAD
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
