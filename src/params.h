#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

#include "host_comm.h"

extern uint16_t param_test;
extern uint16_t param_target_boot_voltage_dl;
extern uint16_t param_target_boot_latency_kcycles;

unsigned set_param(param_t param, uint8_t *buf);
unsigned get_param(param_t param, uint8_t *buf);

#endif
