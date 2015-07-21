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
 * @brief	Charge WISP capacitor to the specified voltage level using ADC
 * @param	target			Target voltage level to charge to (in ADC units)
 * @return  final actual measured voltage level (ADC units)
 */
static uint16_t charge_adc(uint16_t target);

/**
 * @brief	Discharge WISP capacitor to the specified voltage level using ADC
 * @param	target			Target voltage level to discharge to (in ADC units)
 * @return  final actual measured voltage level (ADC units)
 */
static uint16_t discharge_adc(uint16_t target);

/**
 * @brief	Charge WISP capacitor to the specified voltage level using comparator
 * @param	target			Target voltage level to charge to (as comparator ref value)
 * @param   cmp_ref Voltage reference with resepect to which 'target' voltage is calculated
 * @details The 5-bit reference value is calculated as: target = 2.5 / 2^32 * target_volts
 */
static void charge_cmp(uint16_t target, comparator_ref_t cmp_ref);

/**
 * @brief	Discharge WISP capacitor to the specified voltage level using comparator
 * @param	target			Target voltage level to discharge to (as comparator ref value)
 * @param   cmp_ref Voltage reference with resepect to which 'target' voltage is calculated
 * @details The 5-bit reference value is calculated as: target = 2.5 / 2^32 * target_volts
 */
static void discharge_cmp(uint16_t target, comparator_ref_t cmp_ref);

#ifdef CONFIG_PWM_CHARGING
/**
 * @brief	Block until setting the voltage read at channel to the ADC reading target.
 * @param	adc_chan_index  Permanent index statically assigned to the ADC channel
 * @param	target			Target ADC reading when the voltage is set, from 0 to 4095
 */
static void setWispVoltage_block(unsigned adc_chan_index, uint16_t target);
#endif

/**
 * @brief	Interrupt WISP and enter active debug mode when Vcap reaches the given level
 * @param   level   Vcap level to interrupt at
 * @details Implemented by continuously sampling Vcap using the ADC
 */
static void break_at_vcap_level_adc(uint16_t level);

/**
 * @brief	Interrupt WISP and enter active debug mode when Vcap reaches the given level
 * @param   level   Vcap level to interrupt at
 * @param   cmp_ref Voltage reference with resepect to which 'target' voltage is calculated
 * @details Implemented by monitoring Vcap using the analog comparator
 */
static void break_at_vcap_level_cmp(uint16_t level, comparator_ref_t cmp_ref);

#endif // MAIN_H
