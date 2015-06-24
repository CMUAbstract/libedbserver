#ifndef ERROR_H
#define ERROR_H

/* @brief A number that identifies an error condition
 * @details The handling of each error is defined in the source file.
 */
typedef enum {
    ERROR_NONE = 0,
} error_t;

/* @brief Blink led at a given rate indefinitely
 * @details This has to be a macro because arg to __delay_cycles intrinsic must be a
 *          constant.
 */
#define BLINK_LOOP(led_pin, delay_cycles) while (1) { \
        GPIO(PORT_LED, OUT) ^= BIT(led_pin); \
        __delay_cycles(delay_cycles); \
    }

/* @brief Report and handle error */
void error(error_t num);

#endif
