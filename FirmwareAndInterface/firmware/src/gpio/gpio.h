/*******************************************************************************
 * @file
 * @date        30 March 2015
 * @author      Graham Harvey
 * @brief       Definitions and prototypes for WISP monitor GPIO.
 ******************************************************************************/

#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

/**
 * @defgroup    GPIO    GPIO control
 * @brief       Manipulate GPIO pins on the MSP430
 * @{
 */

/**
 * @defgroup	P1IFG_DEFINES	Port 1 interrupt flags
 * @brief		Port 1 pins and corresponding interrupt flags
 * @{
 */
#define IFG_VRECT_BUF					P1IV_P1IFG0 //!< Vrect interrupt flag
#define IFG_RF_TX						P1IV_P1IFG1 //!< RF TX interrupt flag
#define IFG_RF_RX						P1IV_P1IFG2 //!< RF RX interrupt flag
#define IFG_AUX_3						P1IV_P1IFG3 //!< AUX 3 interrupt flag
#define IFG_AUX_2						P1IV_P1IFG4 //!< AUX 2 interrupt flag
#define IFG_AUX_1						P1IV_P1IFG5 //!< AUX 1 interrupt flag
#define IFG_SIG  						P1IV_P1IFG5 //!< target signal interrupt flag
#define IFG_VBOOST_BUF					P1IV_P1IFG6 //!< Vboost interrupt flag
/** @} End P1IFG_DEFINES */

/**
 * @brief   Set up all pins.  Default to GPIO output low.
 */
void pin_init();

/** @} End GPIO */

#endif // GPIO_H
