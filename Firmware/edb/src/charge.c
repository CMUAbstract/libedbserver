
#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>

#include <libmsp/periph.h>

#include "config.h"
#include "pin_assign.h"
#include "adc.h"
#include "comparator.h"

#include "charge.h"

uint16_t charge_adc(uint16_t target)
{
    uint16_t cur_voltage;

    // Output Vcc level to Vcap (through R1) */

    // Configure the pin
    GPIO(PORT_CHARGE, DS) |= BIT(PIN_CHARGE); // full drive strength
    GPIO(PORT_CHARGE, SEL) &= ~BIT(PIN_CHARGE); // I/O function
    GPIO(PORT_CHARGE, DIR) |= BIT(PIN_CHARGE); // I/O function output

    GPIO(PORT_CHARGE, OUT) |= BIT(PIN_CHARGE); // turn on the power supply

    // Wait for the cap to charge to that voltage

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = ADC_read(ADC_CHAN_INDEX_VCAP);
    } while (cur_voltage < target);

    GPIO(PORT_CHARGE, OUT) &= ~BIT(PIN_CHARGE); // cut the power supply

    return cur_voltage;
}

uint16_t discharge_adc(uint16_t target)
{
    uint16_t cur_voltage;

    GPIO(PORT_DISCHARGE, DIR) |= BIT(PIN_DISCHARGE); // open the discharge "valve"

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = ADC_read(ADC_CHAN_INDEX_VCAP);
    } while (cur_voltage > target);

    GPIO(PORT_DISCHARGE, DIR) &= ~BIT(PIN_DISCHARGE); // close the discharge "valve"

    return cur_voltage;
}

void charge_cmp(uint16_t target, comparator_ref_t ref)
{
    arm_comparator(CMP_OP_CHARGE, target, ref, CMP_EDGE_FALLING, COMP_CHAN_VCAP);

    // Configure the pin
    GPIO(PORT_CHARGE, DS) |= BIT(PIN_CHARGE); // full drive strength
    GPIO(PORT_CHARGE, SEL) &= ~BIT(PIN_CHARGE); // I/O function
    GPIO(PORT_CHARGE, DIR) |= BIT(PIN_CHARGE); // I/O function output

    GPIO(PORT_CHARGE, OUT) |= BIT(PIN_CHARGE); // turn on the power supply

    // expect comparator interrupt
}

void discharge_cmp(uint16_t target, comparator_ref_t ref)
{
    arm_comparator(CMP_OP_DISCHARGE, target, ref, CMP_EDGE_RISING, COMP_CHAN_VCAP);

    GPIO(PORT_DISCHARGE, DIR) |= BIT(PIN_DISCHARGE); // open the discharge "valve"

    // expect comparator interrupt
}

#ifdef CONFIG_PWM_CHARGING

/**
 * @brief	Compare two unsigned 16-bit numbers.
 * @param	n1	One number
 * @param	n2	Another number
 * @param	threshold	minimum difference between n1 and n2 to consider them different.
 * @retval	1	n1 > n2 + threshold
 * @retval	0	n1 and n2 are within threshold of one another
 * @retval	-1	n1 < n2 - threshold
 */
static int uint16Compare(uint16_t n1, uint16_t n2, uint16_t threshold) {
	if(n1 < n2 - threshold && n2 >= threshold) {
		return -1;
	} else if(n1 > n2 + threshold && n2 + threshold <= 0xFFFF) {
		return 1;
	}
	return 0;
}

void setWispVoltage_block(unsigned adc_chan_index, uint16_t target)
{
	uint16_t result;
    int compare;
	unsigned threshold = 1;

	// Here, we want to choose a starting PWM duty cycle close to, but not above,
	// what the correct one will be.  We want it to be close because we will test
	// each duty cycle, increasing one cycle at a time, until we reach the right
	// one.  In my experiment with the oscilloscope, the WISP cap took about 60ms
	// to charge, so I'll give it 80ms for the first charge and 10ms for each
	// charge following.

	// We know the ADC target, so let's give the PWM duty cycle our best guess.
	// target (adc) * PWM_period (SMCLK cycles) / 2^12 (adc) = approx. PWM_duty_cycle (SMCLK cycles)
	// Subtract 40 from this to start.
	uint32_t duty_cycle = (uint32_t) target * (uint32_t) TB0CCR0 / 4096;
	TB0CCR1 = MAX((uint16_t) duty_cycle - 40, 0);
	PWM_start();

	// 70ms wait time
	unsigned i;
	for(i = 0; i < 40; i++) {
		__delay_cycles(21922); // delay for 1ms
	}

	do {
		// 10ms wait time
		for(i = 0; i < 40; i++) {
			__delay_cycles(21922); // delay for 1ms
		}

		result = ADC_read(adc_chan_index);
		compare = uint16Compare(result, target, threshold);
		if(compare < 0) {
			// result < target
			PWM_INCREASE_DUTY_CYCLE;
		} else if(compare > 0) {
			// result > target
			PWM_DECREASE_DUTY_CYCLE;
		}
	} while(compare != 0);
}
#endif
