/**
 * @file
 * @author  Graham Harvey
 * @date    13 May 2015
 * @brief   Prototypes and declarations for main.c.
 */

#ifndef MAIN_H
#define MAIN_H

#include "uart.h"

/**
 * @brief   Execute a command received over USB
 * @param   pkt     UART packet containing message information
 */
static void executeUSBCmd(uartPkt_t *pkt);

/**
 * @brief	Charge WISP capacitor to the specified voltage level
 * @param	target			Target voltage level to charge to (in ADC units)
 * @return  final actual measured voltage level (ADC units)
 */
static uint16_t charge_block(uint16_t target);

/**
 * @brief	Discharge WISP capacitor to the specified voltage level
 * @param	target			Target voltage level to discharge to (in ADC units)
 * @return  final actual measured voltage level (ADC units)
 */
static uint16_t discharge_block(uint16_t target);

/**
 * @brief	Block until setting the voltage read at channel to the ADC reading target.
 * @param	adc_chan_index  Permanent index statically assigned to the ADC channel
 * @param	target			Target ADC reading when the voltage is set, from 0 to 4095
 */
static void setWispVoltage_block(uint8_t adc_chan_index, uint16_t target);

/**
 * @brief	Compare two unsigned 16-bit numbers.
 * @param	n1	One number
 * @param	n2	Another number
 * @param	threshold	minimum difference between n1 and n2 to consider them different.
 * @retval	1	n1 > n2 + threshold
 * @retval	0	n1 and n2 are within threshold of one another
 * @retval	-1	n1 < n2 - threshold
 */
static int8_t uint16Compare(uint16_t n1, uint16_t n2, uint16_t threshold);

#endif // MAIN_H
