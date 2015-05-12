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
#include "monitor.h"

static adc12_t *_pAdc12;

void ADC12_configure(adc12_t *adc12)
{
    _pAdc12 = adc12;

    P6SEL = 0x00;
    
    ADC12CTL0 &= ~ADC12ENC; // disable conversion so we can set control bits
    uint8_t num_channels = _pAdc12->config.num_channels;
    if(num_channels == 1) {
        // single channel, single conversion
        uint16_t channel = _pAdc12->config.channels[0];
        ADC12_pinSelect(channel);
        ADC12CTL0 = ADC12SHT0_2 + ADC12ON; // sampling time, ADC12 on
        ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0; // use sampling timer, single-channel, single-conversion
        ADC12MCTL0 = channel; // set ADC memory control register
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
            uint16_t channel = _pAdc12->config.channels[i];
            ADC12_pinSelect(channel);
            *(adc12mctl_registers[i]) = channel;
        }
        uint8_t last_channel_index = num_channels - 1;
        *(adc12mctl_registers[last_channel_index]) |= ADC12EOS;

        // set the correct interrupt
        ADC12IE = (0x0001 << last_channel_index);
    }
    ADC12CTL0 |= ADC12ENC;
}

static void ADC12_pinSelect(uint16_t channel)
{
    switch(channel)
    {
    case ADC12INCH_VCAP:
        P6SEL |= ADC12_VCAP;
        break;
    case ADC12INCH_VBOOST:
        P6SEL |= ADC12_VBOOST;
        break;
    case ADC12INCH_VREG:
        P6SEL |= ADC12_VREG;
        break;
    case ADC12INCH_VRECT:
        P6SEL |= ADC12_VRECT;
        break;
    case ADC12INCH_VINJ:
        P6SEL |= ADC12_VINJ;
        break;
    default:
        break;
    }
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
	_pAdc12->timeComplete = getTime(); // record time

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
