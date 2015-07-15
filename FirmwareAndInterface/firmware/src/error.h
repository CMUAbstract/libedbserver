#ifndef ERROR_H
#define ERROR_H

/* @brief A number that identifies an error condition
 * @details The handling of each error is defined in the source file.
 */
typedef enum {
    ERROR_NONE = 0,
    ERROR_UNEXPECTED_CODEPOINT,
    ERROR_UNEXPECTED_INTERRUPT,
    ERROR_INVALID_VALUE,
} error_t;

typedef enum {
    ASSERT_INVALID_STREAM_DATA_LEN                = 0,
    ASSERT_RF_EVENTS_BUF_OVERFLOW                 = 1,
    ASSERT_HOST_MSG_BUF_OVERFLOW                  = 2,
    ASSERT_RF_RX_DECODE_TIMER_CAPTURE_OVERFLOW    = 3,
    ASSERT_INVALID_RFID_DECODER_STATE             = 4,
    ASSERT_INVALID_RFID_CMD_HANDLER               = 5,
} assert_t;

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

#define ASSERT(idx, cond) \
    if (!(cond)) { \
        GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED); \
        BLINK_LOOP(PIN_LED_GREEN, (CONFIG_DCOCLKDIV_FREQ >> 1) >> (idx)); \
    }

#endif
