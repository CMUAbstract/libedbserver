/*******************************************************************************
 * @file
 * @date            28 April 2015
 * @author          Graham Harvey
 * @brief           Functions for using the 12-bit ADC with the WISP monitor.
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>
#include "adc12.h"
#include "timeLog.h"
#include "pin_assign.h"
#include "main_loop.h"
#include "host_comm.h"
#include "uart.h"
#include "config.h"
#include "error.h"
#include "dma.h"

#define TIMER_DIV_BITS_INNER(div) ID__ ## div
#define TIMER_DIV_BITS(div) TIMER_DIV_BITS_INNER(div)

#define TIMER_B_CLK_SOURCE_BITS_INNER(name) TBSSEL__ ## name
#define TIMER_B_CLK_SOURCE_BITS(name) TIMER_B_CLK_SOURCE_BITS_INNER(name)

#define NUM_BUFFERS                                  2 // double-buffer pair
#define NUM_BUFFERED_SAMPLES                        16

#define SAMPLE_TIMESTAMPS_SIZE (NUM_BUFFERED_SAMPLES * sizeof(uint16_t))
#define SAMPLE_VOLTAGES_SIZE   (NUM_BUFFERED_SAMPLES * ADC12_MAX_CHANNELS * sizeof(uint16_t))

// Buffer layout:
//
//    [ uart msg header | stream msg header |
//      timestamp 0 | .. | timestamp N |
//      voltage 0 chan 0 | .. | voltage 0 chan K
//        ...
//      voltage N chan 0 | .. | voltage N chan K
//    ]
//
// NOTE: The buffer layout with timestamps grouped together and voltage samples
// groupped together was chosen in preparation for DMA, which would have one
// channel stream timestamps from the timer and another one stream ADC values,
// both triggered on exact same timer event.
#define SAMPLES_MSG_BUF_SIZE \
    (UART_MSG_HEADER_SIZE + STREAM_DATA_MSG_HEADER_LEN + \
     SAMPLE_TIMESTAMPS_SIZE + SAMPLE_VOLTAGES_SIZE)

#define SAMPLE_TIMESTAMPS_OFFSET  (UART_MSG_HEADER_SIZE + STREAM_DATA_MSG_HEADER_LEN)
#define SAMPLE_VOLTAGES_OFFSET  (SAMPLE_TIMESTAMPS_OFFSET + SAMPLE_TIMESTAMPS_SIZE)

static adc12_t *_pAdc12;

static uint8_t sample_msg_bufs[NUM_BUFFERS][SAMPLES_MSG_BUF_SIZE];

// Can't have a struct type because the number of channels per 'sample' (sequence) varies
static uint16_t * const sample_timestamps_bufs[NUM_BUFFERS] = {
    (uint16_t *)&sample_msg_bufs[0][SAMPLE_TIMESTAMPS_OFFSET],
    (uint16_t *)&sample_msg_bufs[1][SAMPLE_TIMESTAMPS_OFFSET],
};
static uint16_t * const sample_voltages_bufs[NUM_BUFFERS] = {
    (uint16_t *)&sample_msg_bufs[0][SAMPLE_VOLTAGES_OFFSET],
    (uint16_t *)&sample_msg_bufs[1][SAMPLE_VOLTAGES_OFFSET],
};

// volatile because main uses it to get the index of the ready buffer
static volatile unsigned sample_buf_idx;

void ADC12_init(adc12_t *adc12)
{
    uint16_t i;

    // init indices in the adc12.config.channels and adc12.results arrays
    for (i = 0; i < ADC12_MAX_CHANNELS; ++i)
        adc12->indexes[i] = -1;
}

void ADC12_setup(adc12_t *adc12, uint16_t streams_bitmask)
{
    unsigned i;
    unsigned offset;
    uint8_t *header;

    _pAdc12 = adc12;

    ADC12CTL0 &= ~ADC12ENC; // disable conversion so we can set control bits

    // sequence of channels, single conversion
    ADC12CTL0 = ADC12SHT0_2 + ADC12ON + ADC12MSC; // sampling time, ADC12 on, multiple sample conversion

    // use sampling timer, sequence of channels, repeat-conversion, trigger from Timer B CCR0
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1 + ADC12SHS_2;

    // set ADC memory control registers
    volatile uint8_t *adc12mctl_registers[5] = { &ADC12MCTL0,
                                                 &ADC12MCTL1,
                                                 &ADC12MCTL2,
                                                 &ADC12MCTL3,
                                                 &ADC12MCTL4 };
    for(i = 0; i < _pAdc12->config.num_channels; i++) {
        uint16_t chan_index = adc12->config.channels[i];
        *(adc12mctl_registers[i]) = adc12->config.channel_masks[chan_index];
    }
    unsigned last_channel_index = _pAdc12->config.num_channels - 1;
    *(adc12mctl_registers[last_channel_index]) |= ADC12EOS;

    ADC12IFG = 0; // clear int flags
    ADC12IE = (0x0001 << last_channel_index); // enable interupt on last sample

    TB0CCR0 = adc12->config.sampling_period;
    TB0CCTL0 = OUTMOD_3; // set/reset output mode
    TB0CTL = TIMER_B_CLK_SOURCE_BITS(CONFIG_ADC_TIMER_SOURCE_NAME) |
             TIMER_DIV_BITS(CONFIG_ADC_TIMER_DIV) |
             MC__UP | TBCLR;

    for (i = 0; i < NUM_BUFFERS; ++i) {
        header = &sample_msg_bufs[i][UART_MSG_HEADER_SIZE];
        offset = 0;
        header[offset++] = streams_bitmask;
        header[offset++] = NUM_BUFFERED_SAMPLES; // TODO: flushing last buffer, prob just throw it away
    }
    sample_buf_idx = 0;

    // Common DMA config
    DMACTL4 = DMARMWDIS; // TODO: move somewhere up the stack?

    // DMA for ADC sample timestamps

    DMA(DMA_ADC_TIMESTAMPS, CTL) &= ~DMAEN;

#if 0
#if DMA_ADC_TIMESTAMPS == 0
    DMACTL0 = DMA0TSEL_24; /* ADC12IFG */
#elif DMA_ADC_TIMESTAMPS == 1
    DMACTL0 = DMA1TSEL_24;
#elif DMA_ADC_TIMESTAMPS == 2
    DMACTL1 = DMA2TSEL_24;
#endif
#endif

    DMA(DMA_ADC_TIMESTAMPS, CTL) =
          DMADT_4 /* repeated-single transfer */ |
          DMADSTINCR_3 /* dest inc */ | DMASRCINCR_0 /* src no inc */ |
          DMAIE; /* swap buffers on this interrupt */

    DMA(DMA_ADC_TIMESTAMPS, SA) = (__DMA_ACCESS_REG__)&TA2R;
    DMA(DMA_ADC_TIMESTAMPS, DA) = (__DMA_ACCESS_REG__)sample_timestamps_bufs[sample_buf_idx];
    DMA(DMA_ADC_TIMESTAMPS, SZ) = NUM_BUFFERED_SAMPLES;

    // DMA for ADC voltage samples

    DMA(DMA_ADC_VOLTAGES, CTL) &= ~DMAEN;

#if 0
#if DMA_ADC_VOLTAGES == 0
    DMACTL0 = DMA0TSEL_24; /* ADC12IFG */
#elif DMA_ADC_VOLTAGES == 1
    DMACTL0 = DMA1TSEL_24;
#elif DMA_ADC_VOLTAGES == 2
    DMACTL1 = DMA2TSEL_24;
#endif
#endif

    DMA(DMA_ADC_VOLTAGES, CTL) =
          DMADT_5 /* repeated-block transfer */ |
          DMADSTINCR_3 /* dest inc */ | DMASRCINCR_3; /* src inc */

    DMA(DMA_ADC_VOLTAGES, SA) = (__DMA_ACCESS_REG__)&ADC12MEM0;
    DMA(DMA_ADC_VOLTAGES, DA) = (__DMA_ACCESS_REG__)sample_voltages_bufs[sample_buf_idx];
    DMA(DMA_ADC_VOLTAGES, SZ) = adc12->config.num_channels; // size of the block

    DMA(DMA_ADC_TIMESTAMPS, CTL) |= DMAEN;
    DMA(DMA_ADC_VOLTAGES, CTL) |= DMAEN;

    ADC12CTL0 |= ADC12ENC; // launch: wait for trigger
}

void ADC12_swap_dma_buffers()
{
    sample_buf_idx ^= 1;

    // Neither DMA channels should be running, since we're in completion ISR for
    // the lowest priority channel of the two, and both share the same trigger. 
    DMA(DMA_ADC_TIMESTAMPS, CTL) &= ~DMAEN;
    //DMA(DMA_ADC_VOLTAGES, CTL) &= ~DMAEN;

    DMA(DMA_ADC_TIMESTAMPS, DA) = (__DMA_ACCESS_REG__)sample_timestamps_bufs[sample_buf_idx];
    //DMA(DMA_ADC_VOLTAGES, DA) = (__DMA_ACCESS_REG__)sample_voltages_bufs[sample_buf_idx];

    DMA(DMA_ADC_TIMESTAMPS, CTL) |= DMAEN;
    //DMA(DMA_ADC_VOLTAGES, CTL) |= DMAEN;

    main_loop_flags |= FLAG_ADC12_COMPLETE;
}

void ADC12_send_samples_to_host(adc12_t *adc12)
{
    unsigned ready_buf_idx = sample_buf_idx ^ 1; // the other one in the double-buffer pair

    UART_begin_transmission();

    // Concatenated timestamps buf and samples buf
    // sample_buf_idx points to the 'other' (i.e. the ready buf)
    UART_send_msg_to_host(USB_RSP_STREAM_VOLTAGES,
            STREAM_DATA_MSG_HEADER_LEN +
            /* always tx full timestamps section even if buf not completely
             * full because the voltage section is always offset by the
             * size of the timestamp section (i.e. timestamps section is fixed-width,
             * and only the (trailing) voltage section is variable-length). */
            SAMPLE_TIMESTAMPS_SIZE +
            NUM_BUFFERED_SAMPLES * sizeof(uint16_t) * adc12->config.num_channels,
            (uint8_t *)&sample_msg_bufs[ready_buf_idx][0]);

    UART_end_transmission();
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
    switch(__even_in_range(ADC12IV,34))
    {
        case ADC12IV_NONE:                         // Vector  0:  No interrupt
        case ADC12IV_ADC12OVIFG:                   // Vector  2:  ADC overflow
        case ADC12IV_ADC12TOVIFG:                  // Vector  4:  ADC timing overflow
            ASSERT(ASSERT_ADC_FAULT, false);

        case ADC12IV_ADC12IFG0:                           // Vector  6:  ADC12IFG0
        case ADC12IV_ADC12IFG1:                           // Vector  8:  ADC12IFG1
        case ADC12IV_ADC12IFG2:                           // Vector 10:  ADC12IFG2
        case ADC12IV_ADC12IFG3:                           // Vector 12:  ADC12IFG3
        case ADC12IV_ADC12IFG4:                           // Vector 14:  ADC12IFG4

            // TODO: why can't this trigger?
            DMA(DMA_ADC_TIMESTAMPS, CTL) |= DMAREQ;
            DMA(DMA_ADC_VOLTAGES, CTL) |= DMAREQ;

            // This whole ISR only exists because the ADC does not seem to support
            // a truly 'repeated' mode as opposed to 'continuous' mode. In the
            // truly 'repeated' mode, it would take the one sequence of back-to-back
            // samples per one trigger event. All we can do without the ISR is
            // a 'continuous' mode where the first trigger just lets the ADC loose
            // to take back-to-back samples forever.
            // See here: https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/439609
            ADC12CTL0 |= ADC12ENC;
            break;

        case ADC12IV_ADC12IFG5:
        case ADC12IV_ADC12IFG6:
        case ADC12IV_ADC12IFG7:
        case ADC12IV_ADC12IFG8:
        case ADC12IV_ADC12IFG9:
        case ADC12IV_ADC12IFG10:
        case ADC12IV_ADC12IFG11:
        case ADC12IV_ADC12IFG12:
        case ADC12IV_ADC12IFG13:
        case ADC12IV_ADC12IFG14:
        default:
            ASSERT(ASSERT_UNEXPECTED_INTERRUPT, false);
    }
}

#if 0
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt void UNHANDLED_ISR_TIMER0_B1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) UNHANDLED_ISR_TIMER0_B1(void)
#else
#error Compiler not supported!
#endif
{
    _pAdc12->results[0] = 0xbeef;
    main_loop_flags |= FLAG_ADC12_COMPLETE;
    TB0CCTL1 &= ~CCIFG;
}
#endif
