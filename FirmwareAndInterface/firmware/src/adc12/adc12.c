/*******************************************************************************
 * @file
 * @date            28 April 2015
 * @author          Graham Harvey
 * @brief           Functions for using the 12-bit ADC with the WISP monitor.
 ******************************************************************************/

#include <stdint.h>
#include <msp430.h>
#include "adc12.h"
#include "timeLog.h"
#include "pin_assign.h"

static adc12_t *_pAdc12;
static adc12_t adc12_single;

void ADC12_init(adc12_t *adc12)
{
    uint16_t i;

    // init indices in the adc12.config.channels and adc12.results arrays
    for (i = 0; i < ADC12_MAX_CHANNELS; ++i)
        adc12->indexes[i] = -1;
}

void ADC12_configure(adc12_t *adc12, adc12Mode_t mode)
{
    _pAdc12 = adc12;

    ADC12CTL0 &= ~ADC12ENC; // disable conversion so we can set control bits
    ADC12IE = 0; // disable interrupt

    uint8_t num_channels = _pAdc12->config.num_channels;
    if(num_channels == 1) {
        // single channel, single conversion
        uint16_t chan_index = adc12->config.channels[0];
        ADC12CTL0 = ADC12SHT0_2 + ADC12ON; // sampling time, ADC12 on
        ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0; // use sampling timer, single-channel, single-conversion
        ADC12MCTL0 = adc12->config.channel_masks[chan_index]; // set ADC memory control register
        if (mode == ADC12_MODE_INTERRUPT)
            ADC12IE = ADC12IE0; // enable interrupt
    } else if(num_channels > 1) {
        // sequence of channels, single conversion
        ADC12CTL0 = ADC12SHT0_2 + ADC12ON + ADC12MSC; // sampling time, ADC12 on, multiple sample conversion
        ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1; // use sampling timer, sequence of channels, single-conversion

        // set ADC memory control registers
        volatile uint8_t *adc12mctl_registers[5] = { &ADC12MCTL0,
                                                     &ADC12MCTL1,
                                                     &ADC12MCTL2,
                                                     &ADC12MCTL3,
                                                     &ADC12MCTL4 };
        uint8_t i;
        for(i = 0; i < num_channels; i++) {
            uint16_t chan_index = adc12->config.channels[i];
            *(adc12mctl_registers[i]) = adc12->config.channel_masks[chan_index];
        }
        uint8_t last_channel_index = num_channels - 1;
        *(adc12mctl_registers[last_channel_index]) |= ADC12EOS;

        // set the correct interrupt
        if (mode == ADC12_MODE_INTERRUPT) {
            ADC12IE = (0x0001 << last_channel_index);
        }
    }
    ADC12CTL0 |= ADC12ENC;
}

void ADC12_addChannel(adc12_t *adc12, uint8_t chan_index)
{
    // note that the results index corresponds to the
    // index in the channels array

    // first check if the channel is already present
	if(adc12->indexes[chan_index] == -1) {
        // channel isn't present in the configuration, so add it
        adc12->config.channels[adc12->config.num_channels] = chan_index;
        adc12->indexes[chan_index] = adc12->config.num_channels++;
    }
}

void ADC12_removeChannel(adc12_t *adc12, uint8_t chan_index)
{
    // note that chan_index is also the corresponding
    // index in the channels array

	if(adc12->indexes[chan_index] != -1) {
		// the channel is present in the configuration
		adc12->config.num_channels--;

		if(adc12->indexes[chan_index] != adc12->config.num_channels) {
			// We're removing this channel, but it wasn't the last channel
			// configured.  We need to move the other channels in the channels
			// array so there are no missing channels in the array.
			uint8_t i;
			for(i = adc12->indexes[chan_index] + 1; i < adc12->config.num_channels + 1; i++) {
				adc12->config.channels[i - 1] = adc12->config.channels[i];
			}

			// update the results array indices
            for (i = 0; i < ADC12_MAX_CHANNELS; ++i) {
                if(adc12->indexes[i] >= adc12->config.num_channels)
                    adc12->indexes[i]--;
			}
		}

		adc12->indexes[chan_index] = -1; // remove the channel
	}
}

void ADC12_start()
{
    ADC12CTL0 |= ADC12SC | ADC12ENC; // enable ADC and start conversion
}

void ADC12_stop()
{
    ADC12CTL0 &= ~(ADC12SC | ADC12ENC);  // stop conversion and disable ADC
}

void ADC12_wait()
{
    while (ADC12CTL1 & ADC12BUSY);
}

void ADC12_restart(adc12_t *adc12)
{
    // don't need to worry about restarting if there are no active channels
    if(adc12->config.num_channels > 0) {
        ADC12_stop();
        ADC12_wait(); // need to complete conversion if multiple channels are active
        ADC12_configure(adc12, ADC12_MODE_INTERRUPT); // reconfigure
        ADC12_start();
    }
}

uint16_t ADC12_read(adc12_t *adc12, uint16_t chan_index)
{
    uint16_t adc12Result;

    adc12_single.pFlags = 0; /* interrupt disabled, so never accessed */
    adc12_single.config.channels[0] = chan_index;
    adc12_single.indexes[chan_index] = 0;
    adc12_single.config.num_channels = 1;
    adc12_single.config.channel_masks[chan_index] = adc12->config.channel_masks[chan_index];

    ADC12_stop(); // stop any active conversion
    ADC12_wait();  // wait for conversion to complete so ADC is stopped

    ADC12_configure(&adc12_single, ADC12_MODE_POLLING); // reconfigure ADC
    ADC12_start(); // start conversion
    ADC12_wait(); // wait for conversion to complete
    adc12Result = ADC12MEM0;

    ADC12_configure(adc12, ADC12_MODE_INTERRUPT); // restore previous configuration, enable interrupt

    return adc12Result;
}

uint16_t ADC12_getSample(adc12_t *adc12, uint16_t chan_index)
{
    return adc12->results[adc12->indexes[chan_index]];
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
	_pAdc12->timeComplete = TIMELOG_CURRENT_TIME;

    switch(__even_in_range(ADC12IV,34))
    {
    case ADC12IV_NONE: return;                        // Vector  0:  No interrupt
    case ADC12IV_ADC12OVIFG: return;                  // Vector  2:  ADC overflow
    case ADC12IV_ADC12TOVIFG: return;                 // Vector  4:  ADC timing overflow
    case ADC12IV_ADC12IFG0:                           // Vector  6:  ADC12IFG0
        *(_pAdc12->pFlags) |= _pAdc12->flag_adc12Complete; // alert main loop
        _pAdc12->results[0] = ADC12MEM0;
        return;
    case ADC12IV_ADC12IFG1:                           // Vector  8:  ADC12IFG1
        *(_pAdc12->pFlags) |= _pAdc12->flag_adc12Complete; // alert main loop
        _pAdc12->results[0] = ADC12MEM0;
        _pAdc12->results[1] = ADC12MEM1;
        return;
    case ADC12IV_ADC12IFG2:                           // Vector 10:  ADC12IFG2
        *(_pAdc12->pFlags) |= _pAdc12->flag_adc12Complete; // alert main loop
        _pAdc12->results[0] = ADC12MEM0;
        _pAdc12->results[1] = ADC12MEM1;
        _pAdc12->results[2] = ADC12MEM2;
        return;
    case ADC12IV_ADC12IFG3:                           // Vector 12:  ADC12IFG3
        *(_pAdc12->pFlags) |= _pAdc12->flag_adc12Complete; // alert main loop
        _pAdc12->results[0] = ADC12MEM0;
        _pAdc12->results[1] = ADC12MEM1;
        _pAdc12->results[2] = ADC12MEM2;
        _pAdc12->results[3] = ADC12MEM3;
        return;
    case ADC12IV_ADC12IFG4:                           // Vector 14:  ADC12IFG4
        *(_pAdc12->pFlags) |= _pAdc12->flag_adc12Complete; // alert main loop
        _pAdc12->results[0] = ADC12MEM0;
        _pAdc12->results[1] = ADC12MEM1;
        _pAdc12->results[2] = ADC12MEM2;
        _pAdc12->results[3] = ADC12MEM3;
        _pAdc12->results[4] = ADC12MEM4;
        return;
    case ADC12IV_ADC12IFG5: return;                    // Vector 16:  ADC12IFG5
    case ADC12IV_ADC12IFG6: return;                    // Vector 18:  ADC12IFG6
    case ADC12IV_ADC12IFG7: return;                    // Vector 20:  ADC12IFG7
    case ADC12IV_ADC12IFG8: return;                    // Vector 22:  ADC12IFG8
    case ADC12IV_ADC12IFG9: return;                    // Vector 24:  ADC12IFG9
    case ADC12IV_ADC12IFG10: return;                   // Vector 26:  ADC12IFG10
    case ADC12IV_ADC12IFG11: return;                   // Vector 28:  ADC12IFG11
    case ADC12IV_ADC12IFG12: return;                   // Vector 30:  ADC12IFG12
    case ADC12IV_ADC12IFG13: return;                   // Vector 32:  ADC12IFG13
    case ADC12IV_ADC12IFG14: return;                   // Vector 34:  ADC12IFG14
    default: return;
    }
}
