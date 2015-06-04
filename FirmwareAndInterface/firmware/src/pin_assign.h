#ifndef PIN_ASSIGN_H
#define PIN_ASSIGN_H

#define BIT_INNER(idx) BIT ## idx
#define BIT(idx) BIT_INNER(idx)

#define GPIO_INNER(port, reg) P ## port ## reg
#define GPIO(port, reg) GPIO_INNER(port, reg)

#define INTFLAG_INNER(port, pin) P ## port ## IV_ ## P ## port ## IFG ## pin
#define INTFLAG(port, pin) INTFLAG_INNER(port, pin)

// Ugly workaround to make the pretty GPIO macro work for OUT register
// (a control bit for TAxCCTLx uses the name 'OUT')
#undef OUT

/**
 * @defgroup    PORTS   Pin assignments
 * @brief       MSP430F5340 pin assignments by logic function
 * @{
 */

#define PORT_LED                                J //!< GPIO port for LEDs
#define PIN_LED_GREEN                           2
#define PIN_LED_RED                             3

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

#define PORT_PWM_BYPASS                         4 //!< GPIO port with PWM bypass power supply line
#define PIN_PWM_BYPASS                          6 //!< pin for supplying continuous power

#define PORT_LS_ENABLE                          J //!< GPIO port for level shifter enable signal
#define PIN_LS_ENABLE                           1 //!< level shifter enable pin - output low to disable

#define PORT_SIG                                1 //!< GPIO port for signal line to target
#define PIN_SIG                                 1 //!< target signal pin

#define PORT_STATE                              1 //!< GPIO port for debugger state pins
#define PIN_STATE_0                             5 //!< debugger state bit 0
#define PIN_STATE_1                             4 //!< debugger state bit 1

#define PORT_TRIGGER                            1 //!< GPIO port for scope trigger line
#define PIN_TRIGGER                             1 //!< scope trigger pin

/* RF line taps. Note: two sets of definitions for the same thing (see below) */
#define PORT_RF                                 1 //!< GPIO port for RF RX/TX line taps
#define PIN_RF_TX                               1 //!< RF TX line
#define PIN_RF_RX                               2 //!< RF RX line

/* The same as above, but for asm, which is limited to a simple pre-processor */
#define PIN_RX BIT1
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
#define PIN_I2C_TARGET_SCL                      1 //!< target I2C SCL line

/** @} End PORTS */

#endif // PIN_ASSIGN_H
