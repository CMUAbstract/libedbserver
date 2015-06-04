/*******************************************************************************
 * @file
 * @date            28 April 2015
 * @author          Graham Harvey
 * @brief           Definitions and prototypes for the WISP monitor.
 ******************************************************************************/

#ifndef MONITOR_H
#define MONITOR_H

#include <msp430.h>

// type safe MIN, MAX from http://stackoverflow.com/questions/3437404/min-and-max-in-c
#define MIN(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a < _b ? _a : _b; })
#define MAX(a, b) \
    ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
       _a > _b ? _a : _b; })

#ifndef NULL
#define NULL	0
#endif

#define GPIO_INNER(port, reg) port ## reg
#define GPIO(port, reg) GPIO_INNER(port, reg)

// Ugly workaround to make the pretty GPIO macro work for OUT register
// (a control bit for TAxCCTLx uses the name 'OUT')
#undef OUT

/**
 * @defgroup    PORTS   Pin functions
 * @brief       MSP430F5340 pin functions by port
 * @{
 */

/*************************** Port 1 **************************/
/**
 * @defgroup    PORT1_DEFINES   Port 1
 * @{
 */
#define GPIO_VRECT                              BIT0 //!< P1.0: WISP Vrect
#define GPIO_RF_TX                              BIT1 //!< P1.1: WISP RF TX
#define GPIO_RF_RX                              BIT2 //!< P1.2: WISP RF RX
#define GPIO_AUX_3                              BIT3 //!< P1.3: WISP AUX3, reserved during semi-passive debugging for tagging WISP power trace
#define GPIO_AUX_2                              BIT4 //!< P1.4: WISP AUX2, reserved during semi-passive debugging for tagging WISP power trace
#define GPIO_AUX_1                              BIT5 //!< P1.5: WISP AUX1, reserved for entering active debug mode
#define GPIO_VBOOST                             BIT6 //!< P1.6: WISP Vboost

#define PAUXSEL									P1SEL	//!< AUX port selection
#define PAUXDIR									P1DIR	//!< AUX port direction
#define PAUXIN									P1IN	//!< AUX port input
#define PAUXOUT									P1OUT	//!< AUX port output
#define PAUXIE									P1IE	//!< AUX port interrupt enable
#define PAUXIFG									P1IFG	//!< AUX port interrupt flag
#define PAUXIES									P1IES	//!< AUX port interrupt edge select

#define PIN_RX                                  GPIO_RF_RX      //!< RF RX input pin
#define PRXIN                                   P1IN            //!< RF RX port input
#define PDIR_RX                                 P1DIR           //!< RF RX port direction
#define PRXIES                                  P1IES           //!< RF RX port interrupt edge select
#define PRXIE                                   P1IE            //!< RF RX port interrupt enable
#define PRXIFG                                  P1IFG           //!< RF RX port interrupt flag
#define PRXSEL                                  P1SEL           //!< RF RX port selection
#define PRX_VECTOR_DEF                          PORT1_VECTOR    //!< RF RX port vector

#define PIN_TX									GPIO_RF_TX		//!< RF TX input pin
#define PTXIE									P1IE			//!< RF TX port interrupt enable
#define PTXIES									P1IES			//!< RF TX port interrupt edge select
#define PTXIFG									P1IFG			//!< RF TX port interrupt flag
/** @} End PORT1_DEFINES */

/*************************** Port 2 **************************/

/*************************** Port 3 **************************/
/**
 * @defgroup    PORT3_DEFINES   Port 3
 * @{
 */
#define UART_USB_TX                            BIT3 //!< P3.3: UART TX to FTDI FT232R UART to USB chip
#define UART_USB_RX                            BIT4 //!< P3.4: UART RX from FTDI FT232R UART to USB chip
/** @} End PORT3_DEFINES */

/*************************** Port 4 **************************/
/**
 * @defgroup    PORT4_DEFINES   Port 4
 * @{
 */
#define I2C_SDA                                 BIT1 //!< P4.1: WISP I2C SDA, reconfigurable as GPIO
#define I2C_SCL                                 BIT2 //!< P4.2: WISP I2C SCL, reconfigurable as GPIO
#define UART_WISP_TX                            BIT4 //!< P4.4: UART TX from monitor to WISP, reconfigurable as GPIO
#define UART_WISP_RX                            BIT5 //!< P4.5: UART RX from WISP to monitor, reconfigurable as GPIO
#define GPIO_PWM_BYPASS                         BIT6 //!< P4.6: GPIO bypass for PWM, preventing WISP from charging
/** @} End PORT4_DEFINES */

/*************************** Port 5 **************************/
/**
 * @defgroup    PORT5_DEFINES   Port 5
 * @{
 */
#define PCHGDIR                                 P5DIR //!< WISP charge pin direction
#define PCHGOUT                                 P5OUT //!< WISP charge pin output value

#define XT2IN                                   BIT2 //!< P5.2: XT2 in
#define XT2OUT                                  BIT3 //!< P5.3: XT2 out
#define XT1IN                                   BIT4 //!< P5.4: XT1 in
#define XT1OUT                                  BIT5 //!< P5.5: XT1 out
#define WISP_CHARGE                             BIT7 //!< P5.7: PWM output to charge the WISP
/** @} End PORT5_DEFINES */

/*************************** Port 6 **************************/
/**
 * @defgroup    PORT6_DEFINES   Port 6
 * @{
 */
#define ADC12_VCAP                              BIT1 //!< P6.1: ADC input Vcap
#define ADC12_VBOOST                            BIT2 //!< P6.2: ADC input Vboost
#define ADC12_VREG                              BIT3 //!< P6.3: ADC input Vreg
#define ADC12_VRECT                             BIT4 //!< P6.4: ADC input Vrect
#define ADC12_VINJ                           	BIT5 //!< P6.5: ADC input PWM LPF
/** @} End PORT6_DEFINES */

/** @} End PORTS */

#define PORT_LED                                PJ //<! GPIO port for LEDs
#define PIN_LED_GREEN                           BIT2
#define PIN_LED_RED                             BIT3

#define PORT_CHARGE                             P5 //!< GPIO port for target capacitor charge pin
#define PIN_CHARGE                              BIT7 //!< target capacitor charge pin

#define PORT_DISCHARGE                          PJ //!< GPIO port for target capacitor discharge pin
#define PIN_DISCHARGE                           BIT0 //!< target capacitor discharge pin

#define PORT_LS_ENABLE                          PJ //!< GPIO port for level shifter enable signal
#define PIN_LS_ENABLE                           BIT1 //!< level shifter enable pin - output low to disable

#define PORT_SIG                                P1 //!< GPIO port for signal line to target
#define PIN_SIG                                 BIT5 //!< target signal pin

#define PORT_STATE                              P1 //!< GPIO port for debugger state pins
#define PIN_STATE_0                             BIT1 //!< debugger state bit 0
#define PIN_STATE_1                             BIT2 //!< debugger state bit 1

#define PORT_TRIGGER                            P1 //!< GPIO port for scope trigger line
#define PIN_TRIGGER                             BIT3 //!< scope trigger pin

#endif // MONITOR_H
