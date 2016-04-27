#include "i2c.h"

#include <msp430.h>

#include <libmsp/periph.h>

#include "pin_assign.h"

void I2C_setup()
{
    GPIO(PORT_I2C_TARGET, SEL) |= BIT(PIN_I2C_TARGET_SCL) | BIT(PIN_I2C_TARGET_SDA);
}

void I2C_teardown()
{
    // Put pins into high-z state
    GPIO(PORT_I2C_TARGET, SEL) &= ~(BIT(PIN_I2C_TARGET_SCL) | BIT(PIN_I2C_TARGET_SDA));
    GPIO(PORT_I2C_TARGET, DIR) &= ~(BIT(PIN_I2C_TARGET_SCL) | BIT(PIN_I2C_TARGET_SDA));
}
