#include <msp430.h>
#include <string.h>
#include <stdlib.h>

#include <libmsp/periph.h>

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

typedef enum {
    PKT_TYPE_BEACON             = 1,
    PKT_TYPE_APP_OUTPUT         = 2,
    PKT_TYPE_ENERGY_PROFILE     = 3,
} pkt_type_t;

// TODO: HACK
// From app: must match!
#define NUM_SENSORS 7
#define NUM_WINDOWS 4
typedef struct _edb_info_t{
  int8_t averages[NUM_SENSORS][NUM_WINDOWS];
} edb_info_t;

static payload_t payload; // EDB+App data sent to host/ground

static void log_packet(char type, uint8_t header, uint8_t *pkt, unsigned len)
{
#if defined(CONFIG_DEV_CONSOLE)
    int i;
    BLOCK_LOG_BEGIN();
    BLOCK_LOG("tx: pkt %c:\r\n", type);
    BLOCK_LOG("%02x ", header);
    for (i = 0; i < len; ++i) {
        BLOCK_LOG("%02x ", *((uint8_t *)pkt + i));

        if (((i + 1 + 1) & (8 - 1)) == 0)
            BLOCK_LOG("\r\n");
    }
    BLOCK_LOG("\r\n");
    BLOCK_LOG_END();
#endif
}

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

void payload_send_beacon()
{
    uint8_t pkt = (PKT_TYPE_BEACON << 4) | 0x0E;

    log_packet('B', 0, &pkt, sizeof(pkt));

#ifdef CONFIG_RADIO_TRANSMIT_PAYLOAD
    SpriteRadio_txInit();
    SpriteRadio_transmit((char *)&pkt, sizeof(pkt));
    SpriteRadio_sleep();
#endif
}

#ifdef CONFIG_COLLECT_APP_OUTPUT
void payload_send_app_output()
{
    // randomply pick one sensor and send only that

    uint8_t sensor_idx = rand() % NUM_SENSORS;

    uint8_t *pkt = (uint8_t *)(&payload.app_output) + sensor_idx * NUM_WINDOWS;
    unsigned pkt_len = NUM_WINDOWS * sizeof(int8_t);

    uint8_t header = (PKT_TYPE_APP_OUTPUT << 4) | (sensor_idx & 0x0f);

    log_packet('A', header, pkt, pkt_len);

#ifdef CONFIG_RADIO_TRANSMIT_PAYLOAD
    SpriteRadio_txInit();
    SpriteRadio_transmit((char *)&header, sizeof(header));
    SpriteRadio_transmit((char *)pkt, pkt_len);
    SpriteRadio_sleep();
#endif // CONFIG_RADIO_TRANSMIT_PAYLOAD
}
#endif // CONFIG_COLLECT_APP_OUTPUT

#ifdef CONFIG_COLLECT_ENERGY_PROFILE
void payload_send_profile()
{
    // randomply pick one watchpoint and send only that
    int wp_idx = rand() % (NUM_EVENTS - 1); // 3rd watchpoint unused

    uint8_t *pkt = (uint8_t *)(&payload.energy_profile.events[0] + wp_idx);
    unsigned pkt_len = NUM_ENERGY_BYTES + 1; // 1 is for count; this is sizeof(event_t) without padding

    uint8_t header = (PKT_TYPE_ENERGY_PROFILE << 4) | (wp_idx & 0x0f);

    log_packet('E', header, pkt, pkt_len);

#ifdef CONFIG_RADIO_TRANSMIT_PAYLOAD
    SpriteRadio_txInit();
    SpriteRadio_transmit((char *)&wp_idx, pkt_len);
    SpriteRadio_transmit((char *)&pkt, pkt_len);
    SpriteRadio_sleep();
#endif // CONFIG_RADIO_TRANSMIT_PAYLOAD
}
#endif // COLLECT_ENERGY_PROFILE

void payload_send()
{
    log_packet('P', 0, (uint8_t *)&payload, sizeof(payload_t));

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
