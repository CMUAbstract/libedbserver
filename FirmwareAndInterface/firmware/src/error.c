#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include "pin_assign.h"

#include "error.h"

void error(error_t error)
{
    switch (error) {
        case ERROR_NONE:
            /* ignore */
            break;
        default:
            GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
            while (1);
    }
}
