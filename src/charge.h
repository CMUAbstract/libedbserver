#ifndef CHARGE_H
#define CHARGE_H

#include <stdint.h>
#include <stdbool.h>

#include "host_comm.h"

/**
 * @brief	Charge WISP capacitor to the specified voltage level using ADC
 * @param	target			Target voltage level to charge to (in ADC units)
 * @return  final actual measured voltage level (ADC units)
 */
uint16_t charge_adc(uint16_t target);

/**
 * @brief	Discharge WISP capacitor to the specified voltage level using ADC
 * @param	target			Target voltage level to discharge to (in ADC units)
 * @return  final actual measured voltage level (ADC units)
 */
uint16_t discharge_adc(uint16_t target);

/**
 * @brief	Charge WISP capacitor to the specified voltage level using comparator
 * @param	target			Target voltage level to charge to (as comparator ref value)
 * @param   cmp_ref Voltage reference with resepect to which 'target' voltage is calculated
 * @details The 5-bit reference value is calculated as: target = 2.5 / 2^32 * target_volts
 */
void charge_cmp(uint16_t target, comparator_ref_t ref);

/**
 * @brief	Discharge WISP capacitor to the specified voltage level using comparator
 * @param	target			Target voltage level to discharge to (as comparator ref value)
 * @param   cmp_ref Voltage reference with resepect to which 'target' voltage is calculated
 * @details The 5-bit reference value is calculated as: target = 2.5 / 2^32 * target_volts
 */
void discharge_cmp(uint16_t target, comparator_ref_t ref);

/**
 * @brief	Block until setting the voltage read at channel to the ADC reading target.
 * @param	adc_chan_index  Permanent index statically assigned to the ADC channel
 * @param	target			Target ADC reading when the voltage is set, from 0 to 4095
 */
void setWispVoltage_block(unsigned adc_chan_index, uint16_t target);

#endif // CHARGE_H
