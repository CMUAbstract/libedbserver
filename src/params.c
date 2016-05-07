#include "params.h"

uint16_t param_test = 0xbeef;
uint16_t param_target_boot_voltage_dl = 2745; // = 2.0v * (4096 / EDB_VDD)
uint16_t param_target_boot_latency_kcycles = 24; // = 24 MHz * 1ms

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

unsigned set_param(param_t param, uint8_t *buf)
{
    switch (param) {
        case PARAM_TEST:
            return deserialize_uint16(&param_test, buf);
        case PARAM_TARGET_BOOT_VOLTAGE_DL:
            return deserialize_uint16(&param_target_boot_voltage_dl, buf);
        case PARAM_TARGET_BOOT_LATENCY_KCYCLES:
            return deserialize_uint16(&param_target_boot_latency_kcycles, buf);
        default:
            return 0;
    }
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
        default:
            return 0;
    }
}
