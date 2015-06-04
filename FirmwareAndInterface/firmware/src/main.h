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
 * @brief   Add an ADC channel to the adc12 configuration structure
 * @param   pResults_index  Pointer to any of vcap_index, vboost_index, vreg_index, or vrect_index.
 *                          Must correspond to the ADC measurement that will be added
 * @param   channel         Channel to add.  See @ref ADC12_CHANNELS
 */
static void addAdcChannel(uint16_t channel, int8_t *pResults_index);

/**
 * @brief   Remove an ADC channel from the adc12 configuration structure
 * @param   results_index   Any of vcap_index, vboost_index, vreg_index, or vrect_index.
 *                          Must correspond to the ADC measurement that will be removed.
 */
static void removeAdcChannel(int8_t *pResults_index);

/**
 * @brief   Reconfigure and restart ADC defined by the adc12 configuration structure
 */
static void restartAdc();

/**
 * @brief   Blocking read of an ADC channel
 * @param   channel Channel to read
 * @return  ADC12 conversion result
 * @details This function reconfigures the ADC to read only the channel
 *          requested, and returns the result.  It then restores the ADC
 *          to the configuration defined by the adc12 structure.
 */
static uint16_t adc12Read_block(uint16_t channel);

/**
 * @brief	Charge WISP capacitor to the specified voltage level
 * @param	target			Target voltage level to charge to (in ADC units)
 */
static void charge_block(uint16_t target);

/**
 * @brief	Discharge WISP capacitor to the specified voltage level
 * @param	target			Target voltage level to discharge to
 */
static void discharge_block(uint16_t target);

/**
 * @brief	Block until setting the voltage read at channel to the ADC reading target.
 * @param	channel	ADC channel measuring the voltage at the node that is being set.
 * @param	pResults_index	Pointer to any of vcap_index, vboost_index, vreg_index,
 * 							or vrect_index.  Must correspond to the ADC channel
 * 							represented by channel.
 * @param	target			Target ADC reading when the voltage is set, from 0 to 4095
 */
static void setWispVoltage_block(uint16_t channel, int8_t *pResults_index, uint16_t target);

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
