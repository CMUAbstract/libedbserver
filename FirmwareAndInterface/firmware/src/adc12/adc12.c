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
#include "main_loop.h"
#include "config.h"

static adc12_t *_pAdc12;

void ADC12_init(adc12_t *adc12)
{
    uint16_t i;

    // init indices in the adc12.config.channels and adc12.results arrays
    for (i = 0; i < ADC12_MAX_CHANNELS; ++i)
        adc12->indexes[i] = -1;
}

void ADC12_setup(adc12_t *adc12)
{
    _pAdc12 = adc12;

    ADC12CTL0 &= ~ADC12ENC; // disable conversion so we can set control bits

    // sequence of channels, single conversion
    ADC12CTL0 = ADC12SHT0_2 + ADC12ON + ADC12MSC; // sampling time, ADC12 on, multiple sample conversion

    // use sampling timer, sequence of channels, repeat-conversion, trigger from Timer B CCR1
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1 + ADC12SHS_2;

    // set ADC memory control registers
    volatile uint8_t *adc12mctl_registers[5] = { &ADC12MCTL0,
                                                 &ADC12MCTL1,
                                                 &ADC12MCTL2,
                                                 &ADC12MCTL3,
                                                 &ADC12MCTL4 };
    unsigned i;
    for(i = 0; i < _pAdc12->config.num_channels; i++) {
        uint16_t chan_index = adc12->config.channels[i];
        *(adc12mctl_registers[i]) = adc12->config.channel_masks[chan_index];
    }
    unsigned last_channel_index = _pAdc12->config.num_channels - 1;
    *(adc12mctl_registers[last_channel_index]) |= ADC12EOS;

    ADC12IFG = 0; // clear int flags
    ADC12IE = (0x0001 << last_channel_index); // enable interupt on last sample

    TB0CCR0 = adc12->config.sampling_period / 2; // period (toggle mode doubles the period)
    TB0CCTL0 = OUTMOD_4; // toggle output mode
    TB0CTL = CONFIG_ADC_TIMER_SOURCE | MC__UP | TBCLR;   // ACLK, up mode, clear TBR
}

void ADC12_arm()
{
    ADC12CTL0 &= ~ADC12ENC;
    ADC12CTL0 |= ADC12ENC;
}

uint16_t ADC12_read(adc12_t *adc12, unsigned chan_index)
{
    ADC12CTL0 &= ~ADC12ENC; // disable ADC

    ADC12CTL0 = ADC12SHT0_2 + ADC12ON; // sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0; // use sampling timer, single-channel, single-conversion
    ADC12MCTL0 = adc12->config.channel_masks[chan_index]; // set ADC memory control register
    ADC12IE = 0; // disable interrupt

    ADC12CTL0 |= ADC12ENC; // enable ADC

    ADC12_trigger(); // start conversion
    while (ADC12CTL1 & ADC12BUSY); // wait for conversion to complete
    return ADC12MEM0;
}

void ADC12_addChannel(adc12_t *adc12, unsigned chan_index)
{
    // the results index corresponds to the index in the channels array

    adc12->config.channels[adc12->config.num_channels] = chan_index;
    adc12->indexes[chan_index] = adc12->config.num_channels++;
}

void ADC12_removeChannel(adc12_t *adc12, unsigned chan_index)
{
    // note that chan_index is also the corresponding
    // index in the channels array

    // the channel is present in the configuration
    adc12->config.num_channels--;

    if(adc12->indexes[chan_index] != adc12->config.num_channels) {
        // We're removing this channel, but it wasn't the last channel
        // configured.  We need to move the other channels in the channels
        // array so there are no missing channels in the array.
        unsigned i;
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

void ADC12_trigger()
{
    ADC12CTL0 &= ~ADC12SC;  // 'start conversion' bit must be toggled
    ADC12CTL0 |= ADC12SC; // start conversion
}

void ADC12_stop()
{
    ADC12CTL0 &= ~(ADC12SC | ADC12ENC);  // stop conversion and disable ADC
    while (ADC12CTL1 & ADC12BUSY); // conversion stops at end of sequence
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
	_pAdc12->timeComplete = TIMELOG_CURRENT_TIME();

    switch(__even_in_range(ADC12IV,34))
    {
    case ADC12IV_NONE: return;                        // Vector  0:  No interrupt
    case ADC12IV_ADC12OVIFG: return;                  // Vector  2:  ADC overflow
    case ADC12IV_ADC12TOVIFG: return;                 // Vector  4:  ADC timing overflow
    case ADC12IV_ADC12IFG0:                           // Vector  6:  ADC12IFG0
        main_loop_flags |= FLAG_ADC12_COMPLETE;
        _pAdc12->results[0] = ADC12MEM0;
        return;
    case ADC12IV_ADC12IFG1:                           // Vector  8:  ADC12IFG1
        main_loop_flags |= FLAG_ADC12_COMPLETE;
        _pAdc12->results[0] = ADC12MEM0;
        _pAdc12->results[1] = ADC12MEM1;
        return;
    case ADC12IV_ADC12IFG2:                           // Vector 10:  ADC12IFG2
        main_loop_flags |= FLAG_ADC12_COMPLETE;
        _pAdc12->results[0] = ADC12MEM0;
        _pAdc12->results[1] = ADC12MEM1;
        _pAdc12->results[2] = ADC12MEM2;
        return;
    case ADC12IV_ADC12IFG3:                           // Vector 12:  ADC12IFG3
        main_loop_flags |= FLAG_ADC12_COMPLETE;
        _pAdc12->results[0] = ADC12MEM0;
        _pAdc12->results[1] = ADC12MEM1;
        _pAdc12->results[2] = ADC12MEM2;
        _pAdc12->results[3] = ADC12MEM3;
        return;
    case ADC12IV_ADC12IFG4:                           // Vector 14:  ADC12IFG4
        main_loop_flags |= FLAG_ADC12_COMPLETE;
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
