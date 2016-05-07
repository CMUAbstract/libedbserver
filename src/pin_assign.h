#ifndef PIN_ASSIGN_H
#define PIN_ASSIGN_H

#ifdef CONFIG_LIBMSPSOFTUART_PRINTF
#include <libmspsoftuart/pins.h>
#endif // CONFIG_LIBMSPSOFTUART_PRINTF

/**
 * @defgroup    PORTS   Pin assignments
 * @{
 */

#ifdef BOARD_EDB

#define PORT_LED                                J //!< GPIO port for LEDs
#define PIN_LED_GREEN                           2
#define PIN_LED_RED                             3

#define PORT_LED_MAIN_LOOP                      J
#define PIN_LED_MAIN_LOOP                       2

#define PORT_LED_DEBUG_MODE                     J
#define PIN_LED_DEBUG_MODE                      3

#define PORT_LED_BOOT							J
#define PIN_LED_BOOT							3

#define PORT_UART_USB                           3  //!< GPIO port with UART connected to FTDI FT232R USB
#define PIN_UART_USB_TX                         3 //!< TX pin for UART to host USB
#define PIN_UART_USB_RX                         4 //!< RX pin for UART to host USB

#define PORT_UART_TARGET                        4  //!< GPIO port with UART connected to target device
#define PIN_UART_TARGET_TX                      4 //!< TX pin for UART to target device
#define PIN_UART_TARGET_RX                      5 //!< RX pin for UART to target device

#define PORT_VSENSE                             6 //!< GPIO port for voltage sense lines
#define PIN_VCAP                                1 //!< P6.1: ADC input Vcap
#define PIN_VBOOST                              2 //!< P6.2: ADC input Vboost
#define PIN_VREG                                3 //!< P6.3: ADC input Vreg
#define PIN_VRECT                               4 //!< P6.4: ADC input Vrect
#define PIN_VINJ                           	    5 //!< P6.5: ADC input PWM LPF

#define PORT_CHARGE                             5 //!< GPIO port for target capacitor charge pin
#define PIN_CHARGE                              7 //!< target capacitor charge pin

#define PORT_DISCHARGE                          J //!< GPIO port for target capacitor discharge pin
#define PIN_DISCHARGE                           0 //!< target capacitor discharge pin

#define PORT_CONT_POWER                         4 //!< GPIO port with PWM bypass power supply line
#define PIN_CONT_POWER                          6 //!< pin for supplying continuous power

#define PORT_LS_ENABLE                          J //!< GPIO port for level shifter enable signal
#define PIN_LS_ENABLE                           1 //!< level shifter enable pin - output low to disable

#define PORT_SIG                                1 //!< GPIO port for signal line to target
#define PIN_SIG                                 3 //!< target signal pin

#define PORT_STATE                              4 //!< GPIO port for debugger state pins
#define PIN_STATE_0                             1 //!< debugger state lsb bit
#define PIN_STATE_1                             2 //!< debugger state msb bit

#define PORT_EVENT                              3 //!< GPIO port for debugger state pins
#define PIN_EVENT_0                             6 //!< debugger state lsb bit
#define PIN_EVENT_1                             7 //!< debugger state msb bit

#define PORT_TRIGGER                            1 //!< GPIO port for scope trigger line
#define PIN_TRIGGER                             1 //!< scope trigger pin

// Code point pins must be on same port and consecutive
// NOTE: When using the same pins as PIN_STATE, must disable CONFIG_STATE_PINS
#define PORT_CODEPOINT                          1
#define PIN_CODEPOINT_0                         4 // lsb
#define PIN_CODEPOINT_1                         5 // msb
#define BITS_CODEPOINT                          (BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1))
#define NUM_CODEPOINT_PINS                      2

#define PORT_SERIAL_DECODE                      4 //!< GPIO port for serial decoder state
#define PIN_SERIAL_DECODE_PULSE                 1 //!< input pulse trigged decoder interrupt
#define PIN_SERIAL_DECODE_TIMER                 2 //!< decoder timer started, rolled over, or stopped

#define PORT_RFID_DEC_STATE                     1
#define PIN_RFID_RX_DEC_STATE_0                   3
#define PIN_RFID_RX_DEC_STATE_1                   4
#define PIN_RFID_RX_DEC_STATE_2                   5

/* RF line taps. Note: two sets of definitions for the same thing (see below) */
#define PORT_RF                                 1 //!< GPIO port for RF RX/TX line taps
#define PIN_RF_RX                               1 //!< RF RX line
#define PIN_RF_TX                               2 //!< RF TX line
#define TMRCC_RF_RX                             0 //!< timer capture-compare register index
#define TMRCC_RF_TX                             0 //!< timer capture-compare register index

/* The same as above, but for asm, which is limited to a simple pre-processor */
#define PIN_RX BIT2
#define PRXSEL P1SEL
#define PRXIN P1IN
#define PRXIE P1IE
#define PRXIES P1IES
#define PRXIFG P1IFG

#define PORT_AUX                                1 //!< GPIO port for AUX lines to target device
#define PIN_AUX_1                               3 //!< AUX1 line
#define PIN_AUX_2                               3 //!< AUX2 line
#define PIN_AUX_3                               4 //!< AUX3 line

#define PORT_I2C_TARGET                         4 //!< GPIO port with taps into target I2C lines
#define PIN_I2C_TARGET_SDA                      1 //!< target I2C SDA line
#define PIN_I2C_TARGET_SCL                      2 //!< target I2C SCL line

#define UART_HOST                               0
#define UART_TARGET                             1

#define DMA_HOST_UART_TX                        0 //!< DMA channel for UART TX to host

// TODO: warning: timer shared with voltage logging code
// NOTE: if changed, the ISR in main.c must also be changed
#define TIMER_SIG_SERIAL_DECODE                 A1
#define TMRCC_SIG_SERIAL                        0

// NOTE: if changed, the ISR definition in rfid_decoder.c must be also changed
#define TIMER_RF_RX_DECODE                      A0
#define TIMER_RF_TX_DECODE                      A1

// !< general-purpose timer for scheduling pre-defined actions
#define TIMER_SCHED_TYPE                        A
#define TIMER_SCHED_IDX                         1
#define TIMER_SCHED_CCR                         1 //!< timer capture-compare register index

#define TMRMOD_ADC_TRIGGER                      B //!< timer module for ADC trigger in stream mode
#define TMRIDX_ADC_TRIGGER                      0 //!< timer index for ADC trigger in stream mode
#define TMRCC_ADC_TRIGGER                       0 //!< timer capture-compare register index

/** @} End PORTS */

// TODO: define only numbers here and use a macro that takes a number (for consistency)
/**
 * @defgroup    ADC_CHANNELS   ADC input channel names to pins
 * @{
 */
#define ADC_CHAN_VCAP                          ADC12INCH_1 //!< ADC input channel select for Vcap
#define ADC_CHAN_VBOOST                        ADC12INCH_2 //!< ADC input channel select for Vboost
#define ADC_CHAN_VREG                          ADC12INCH_3 //!< ADC input channel select for Vreg
#define ADC_CHAN_VRECT                         ADC12INCH_4 //!< ADC input channel select for Vrect
#define ADC_CHAN_VINJ                          ADC12INCH_5 //!< ADC input channel select for VINJ
/** @} End ADC_CHANNELS */

/**
 * @defgroup    COMP_CHAN comparator input channel pins
 * @{
 */
#define COMP_CHAN_VCAP                          1
#define COMP_CHAN_VBOOST                        2
#define COMP_CHAN_VREG                          3
#define COMP_CHAN_VRECT                         4
#define COMP_CHAN_VINJ                          5
/** @} End COMP_CHAN */

#elif defined(BOARD_SPRITE_EDB_SOCKET_RGZ)

#define PORT_LED                                3 //!< GPIO port for LEDs
#define PIN_LED_GREEN                           3
#define PIN_LED_RED                             4 //!< TODO: make optional

#define PORT_LED_MAIN_LOOP                      3
#define PIN_LED_MAIN_LOOP                       3

#define PORT_LED_DEBUG_MODE                     3
#define PIN_LED_DEBUG_MODE                      4

// No second UART on this board
//#define PORT_UART_USB                           3  //!< GPIO port with UART connected to FTDI FT232R USB
//#define PIN_UART_USB_TX                         3 //!< TX pin for UART to host USB
//#define PIN_UART_USB_RX                         4 //!< RX pin for UART to host USB

#define PORT_UART_TARGET                        1  //!< GPIO port with UART connected to target device
#define PIN_UART_TARGET_TX                      6 //!< TX pin for UART to target device
#define PIN_UART_TARGET_RX                      5 //!< RX pin for UART to target device

#define PORT_VSENSE                             2 //!< GPIO port for voltage sense lines
#define PIN_VCAP                                4 //!< P6.1: ADC input Vcap

#define PORT_CHARGE                             3 //!< GPIO port for target capacitor charge pin
#define PIN_CHARGE                              7 //!< target capacitor charge pin

#define PORT_DISCHARGE                          3 //!< GPIO port for target capacitor discharge pin
#define PIN_DISCHARGE                           6 //!< target capacitor discharge pin

#define PORT_CONT_POWER                         3 //!< GPIO port with PWM bypass power supply line
#define PIN_CONT_POWER                          5 //!< pin for supplying continuous power

#define PORT_LS_ENABLE                          J //!< GPIO port for level shifter enable signal
#define PIN_LS_ENABLE                           2 //!< level shifter enable pin - output low to disable

#define PORT_SIG                                1 //!< GPIO port for signal line to target
#define PIN_SIG                                 0 //!< target signal pin

#define PORT_STATE                              3 //!< GPIO port for debugger state pins
#define PIN_STATE_0                             4 //!< debugger state lsb bit
#define PIN_STATE_1                             5 //!< debugger state msb bit

#define PORT_EVENT                              3 //!< GPIO port for debugger state pins
#define PIN_EVENT_0                             0 //!< debugger state lsb bit
#define PIN_EVENT_1                             2 //!< debugger state msb bit

#define PORT_TRIGGER                            3 //!< GPIO port for scope trigger line
#define PIN_TRIGGER                             2 //!< scope trigger pin

// Code point pins must be on same port and consecutive and in order
// NOTE: When using the same pins as PIN_STATE, must disable CONFIG_STATE_PINS
#define PORT_CODEPOINT                          1
#define PIN_CODEPOINT_0                         1 // lsb
#define PIN_CODEPOINT_1                         2
#define PIN_CODEPOINT_2                         3
#define BITS_CODEPOINT                          (BIT(PIN_CODEPOINT_0) | \
                                                 BIT(PIN_CODEPOINT_1) | \
                                                 BIT(PIN_CODEPOINT_2))
#define NUM_CODEPOINT_PINS                      3 // remember to add cases to ISR

#define PORT_SERIAL_DECODE                      3 //!< GPIO port for serial decoder state
#define PIN_SERIAL_DECODE_PULSE                 0 //!< input pulse trigged decoder interrupt
#define PIN_SERIAL_DECODE_TIMER                 1 //!< decoder timer started, rolled over, or stopped

#define PORT_TARGET_PWR_SWITCH                  2
#define PIN_TARGET_PWR_SWITCH                   0

// no host UART on this board
#define UART_TARGET                             0

// TODO: warning: timer shared with voltage logging code
// NOTE: if changed, the ISR in main.c must also be changed
#define TIMER_SIG_SERIAL_DECODE                 A1
#define TMRCC_SIG_SERIAL                        0

// !< general-purpose timer for scheduling pre-defined actions
#define TIMER_SCHED                             A1
#define TMRCC_SCHED                             0 //!< timer capture-compare register index

/** @} End PORTS */

// TODO: define only numbers here and use a macro that takes a number (for consistency)
/**
 * @defgroup    ADC_CHANNELS   ADC input channel names to pins
 * @{
 */
#define ADC_CHAN_VCAP                          ADC12INCH_4 //!< ADC input channel select for Vcap
/** @} End ADC_CHANNELS */

/**
 * @defgroup    COMP_CHAN comparator input channel pins
 * @{
 */
#define COMP_CHAN_VCAP                          4
#define COMP_CHAN_VDD_SELF                      5
/** @} End COMP_CHAN */


#endif // BOARD_

// The register that selects the channel source depsn on the chan
#if DMA_HOST_UART_TX == 0
#define DMA_HOST_UART_TX_CTL 0
#elif DMA_HOST_UART_TX == 1
#define DMA_HOST_UART_TX_CTL 0
#elif DMA_HOST_UART_TX == 2
#define DMA_HOST_UART_TX_CTL 1
#else
#error Invalid DMA channel index: DMA_HOST_UART_TX
#endif


#endif // PIN_ASSIGN_H
