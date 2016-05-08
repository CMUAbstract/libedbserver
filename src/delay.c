#include "delay.h"

#include <msp430.h>

void delay_kcycles(uint16_t delay_kcycles)
{
    for (unsigned i = 0; i < delay_kcycles; ++i)
        __delay_cycles(1000);
}
