#ifndef CONFIG_H
#define CONFIG_H

/** @brief Lower boundary in the histogram of watchpoint energies in the profile
 *  @details V to raw ADC value conversion:
 *      Vmin/Vref * 2^12, where Vref = 2.5 (see adc.c), and 12 is for 12-bit ADC
 */
//#define CONFIG_ENERGY_PROFILE_MIN_VOLTAGE 2949 // Vmin = 1.8v
#define CONFIG_ENERGY_PROFILE_MIN_VOLTAGE 3276 // Vmin = 2.0v


/** @brief Energy profile buckets that are smaller than one byte
 *  @details NOTE: does not work!
 */
// #define CONFIG_PROFILE_SUB_BYTE_BUCKET_SIZES


#define CONFIG_MAIN_LOOP_SLEEP_STATE LPM0_bits

/** @brief Watchdog configuration */
#define CONFIG_WDT_BITS (WDTSSEL__ACLK | WDTIS__8192K) // 4 minutes

// TODO: Each config should be on a separate branch (and branches should
// differ only by the value of the config item).
#ifdef BOARD_EDB

#define CONFIG_USB_UART_BAUDRATE 2000000ull
// #define CONFIG_USB_UART_BAUDRATE 1500000ull
// #define CONFIG_USB_UART_BAUDRATE 1000000ull
// #define CONFIG_USB_UART_BAUDRATE 921600ull
// #define CONFIG_USB_UART_BAUDRATE 576000ull
// #define CONFIG_USB_UART_BAUDRATE 500000ull
// #define CONFIG_USB_UART_BAUDRATE 460800ull
// #define CONFIG_USB_UART_BAUDRATE 171264ull
// #define CONFIG_USB_UART_BAUDRATE 115200ull
// #define CONFIG_USB_UART_BAUDRATE 38400ull

//#define CONFIG_TARGET_UART_BAUDRATE 9600ull
#define CONFIG_TARGET_UART_BAUDRATE 115200ull

// #define CONFIG_USB_UART_UCOS16
// #define CONFIG_TARGET_UART_UCOS16


#elif defined(BOARD_SPRITE_EDB_SOCKET_RGZ) || defined(BOARD_SPRITE_EDB)

// no host baud rate
#define CONFIG_TARGET_UART_BAUDRATE 115200ull

#endif // BOARD_*

// #define CONFIG_TIMELOG_TIMER_SOURCE TASSEL__ACLK
#define CONFIG_TIMELOG_TIMER_SOURCE TASSEL__SMCLK

//#define CONFIG_TIMELOG_TIMER_DIV 1
#define CONFIG_TIMELOG_TIMER_DIV 8
#define CONFIG_TIMELOG_TIMER_DIV_EX 1
//#define CONFIG_TIMELOG_TIMER_DIV_EX 8

// #define CONFIG_ADC_TIMER_SOURCE_ACLK
#define CONFIG_ADC_TIMER_SOURCE_SMCLK
// #define CONFIG_ADC_TIMER_SOURCE_MCLK

#define CONFIG_ADC_TIMER_DIV 8

#if defined(CONFIG_ADC_TIMER_SOURCE_ACLK)
#define CONFIG_ADC_TIMER_SOURCE_NAME ACLK
#define CONFIG_ADC_TIMER_CLK_FREQ CONFIG_ACLK_FREQ
#elif defined(CONFIG_ADC_TIMER_SOURCE_SMCLK)
#define CONFIG_ADC_TIMER_SOURCE_NAME SMCLK
#define CONFIG_ADC_TIMER_CLK_FREQ CONFIG_SMCLK_FREQ
#elif defined(CONFIG_ADC_TIMER_SOURCE_SMCLK)
#define CONFIG_ADC_TIMER_SOURCE_NAME MCLK
#define CONFIG_ADC_TIMER_CLK_FREQ CONFIG_MCLK_FREQ
#elif defined(CONFIG_ADC_TIMER_SOURCE_MCLK)
#else
#error No clock source selected for ADC timer: see CONFIG_ADC_TIMER_SOURCE_*
#endif // CONFIG_ADC_TIMER_SOURCE_*

#define CONFIG_ADC_TIMER_FREQ (CONFIG_ADC_TIMER_CLK_FREQ / CONFIG_ADC_TIMER_DIV)

// Intervals for schedulable actions: time source fixed at ACLK
#define CONFIG_ENTER_DEBUG_MODE_TIMEOUT   0xff
#define CONFIG_EXIT_DEBUG_MODE_TIMEOUT    0xff
#define CONFIG_TARGET_COMM_TIMEOUT        0xff

#define CONFIG_WATCHPOINT_COLLECTION_TIME 0x1fff

#endif // CONFIG_H
