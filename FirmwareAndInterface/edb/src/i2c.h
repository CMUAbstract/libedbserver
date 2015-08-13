#ifndef I2C_H
#define I2C_H

/**
 * @brief       Setup I2C bus interface
 */
void I2C_setup();

/**
 * @brief       Unsetup I2C bus interface (put pins into high-z state)
 */
void I2C_teardown();

#endif
