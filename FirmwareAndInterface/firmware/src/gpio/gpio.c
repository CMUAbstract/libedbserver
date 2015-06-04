/*******************************************************************************
 * @file
 * @date        13 April 2015
 * @author      Graham Harvey
 * @brief       Functions for WISP monitor GPIO control.
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include "monitor.h"
#include "gpio.h"

#include "timeLog.h"
#include "rfid.h"

void pin_init()
{
    P1SEL = 0x00;                           // I/O function
    P1DIR = GPIO_AUX_1 | BIT7;              // output direction
    P1OUT = 0x00;                           // output low or pulldown enabled

    P2SEL = 0x00;                           // I/O function
    P2DIR = 0xFF;                           // output direction
    P2OUT = 0x00;                           // output low

    P3SEL = BIT3 | BIT4;                    // alternate function
    P3DIR = BIT0 | BIT1 | BIT2 | BIT5 | BIT6 | BIT7; // output direction
    P3OUT = 0x00;                           // output low

    P4SEL = BIT4 | BIT5;                    // alternate function
    P4DIR = BIT0 | BIT3 | BIT6 | BIT7;      // output direction
    P4OUT = 0x00;                           // output low

    P5SEL = 0x00;                           // I/O function
    P5DIR = 0xFF;                           // output direction
    P5OUT = 0x00;                           // output low

    // Port 6 has ADC channels - don't mess with them here.
    P6SEL = 0x00;                           // I/O function
    P6DIR = BIT0 | BIT6 | BIT7;             // output direction for P6.0,P6.6, P6.7
    P6OUT = 0x00;                           // output low

    PJDIR = ~(GPIO_DISCHARGE | GPIO_LS_ENABLE);  // input for these pins
    PJOUT = 0x00;                           // output low
}

// Port 1 ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
	switch(__even_in_range(P1IV, 16))
	{
	case IFG_VRECT_BUF:
		P1IFG &= ~GPIO_VRECT;
		break;
	case IFG_RF_TX:
		RFID_TxHandler(getTime());
		PTXIFG &= ~PIN_TX;
		break;
	case IFG_RF_RX:
		RFID_RxHandler();
		break;
	case IFG_AUX_3:
		P1IFG &= ~GPIO_AUX_3;
		break;
	case IFG_AUX_2:
		P1IFG &= ~GPIO_AUX_2;
		break;
	case IFG_AUX_1:
		P1IFG &= ~GPIO_AUX_1;
		break;
	case IFG_VBOOST_BUF:
		P1IFG &= ~GPIO_VBOOST;
		break;
	default:
		break;
	}
}
