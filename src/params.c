#include "params.h"

uint16_t param_test = 0xbeef;
uint16_t param_target_boot_voltage_dl = 2745; // = 2.0v * (4096 / EDB_VDD)
uint16_t param_target_boot_latency_kcycles = 24; // = 24 MHz * 1ms
uint16_t param_num_watchpoint_events_buffered = 16; // must <= MAX_WATCHPOINT_EVENTS_BUFFERED

static unsigned serialize_uint16(uint8_t *buf, uint16_t value)
{
    unsigned len = 0;

    buf[len++] = value;
    buf[len++] = value >> 8;

    return len;
}

static unsigned deserialize_uint16(uint16_t *value, uint8_t *buf)
{
    *value = (buf[1] << 8) | buf[0];
    return sizeof(unsigned);
}

#if 0 // comment out while unused
static unsigned serialize_frac(uint8_t *buf, float frac)
{
    unsigned denomenator = 10000;
    unsigned numerator = frac * denomenator;
    unsigned len = 0;

    buf[len++] = numerator;
    buf[len++] = numerator >> 8;
    buf[len++] = denomenator;
    buf[len++] = denomenator >> 8;

    return len;
}

static unsigned deserialize_frac(float *frac, uint8_t *buf)
{
    float numerator = (buf[1] << 8) | buf[0];
    float denomenator = (buf[3] << 8) | buf[2];
    unsigned len = sizeof(unsigned) + sizeof(unsigned);
    *frac = numerator / denomenator;
    return len;
}
#endif

return_code_t set_param(param_t param, uint8_t *buf)
{
    switch (param) {
        case PARAM_TEST:
            deserialize_uint16(&param_test, buf);
            break;
        case PARAM_TARGET_BOOT_VOLTAGE_DL:
            deserialize_uint16(&param_target_boot_voltage_dl, buf);
            break;
        case PARAM_TARGET_BOOT_LATENCY_KCYCLES:
            deserialize_uint16(&param_target_boot_latency_kcycles, buf);
            break;
        case PARAM_NUM_WATCHPOINT_EVENTS_BUFFERED:
            if (param_num_watchpoint_events_buffered > MAX_WATCHPOINT_EVENTS_BUFFERED)
                return RETURN_CODE_INVALID_ARGS;
            deserialize_uint16(&param_num_watchpoint_events_buffered, buf);
            break;
        default:
            return RETURN_CODE_INVALID_ARGS;
    }

    return RETURN_CODE_SUCCESS;
}

unsigned get_param(param_t param, uint8_t *buf)
{
    switch (param) {
        case PARAM_TEST:
            return serialize_uint16(buf, param_test);
        case PARAM_TARGET_BOOT_VOLTAGE_DL:
            return serialize_uint16(buf, param_target_boot_voltage_dl);
        case PARAM_TARGET_BOOT_LATENCY_KCYCLES:
            return serialize_uint16(buf, param_target_boot_latency_kcycles);
        case PARAM_NUM_WATCHPOINT_EVENTS_BUFFERED:
            return serialize_uint16(buf, param_num_watchpoint_events_buffered);
        default:
            return 0;
    }
}
