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
        case ERROR_UNEXPECTED_CODEPOINT:
            BLINK_LOOP(PIN_LED_RED, 100000);
        default:
            GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
            while (1);
    }
}
