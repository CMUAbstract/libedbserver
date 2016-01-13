#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>

#include "config.h"

#include "tether.h"

volatile state_t state = STATE_IDLE;
