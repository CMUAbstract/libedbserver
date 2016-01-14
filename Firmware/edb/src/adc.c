#include <stdint.h>
#include <stdbool.h>
#include <msp430.h>

#include <libmsp/periph.h>
#include <libio/log.h>

#include "adc.h"
#include "pin_assign.h"
#include "main_loop.h"
#include "host_comm.h"
#include "uart.h"
#include "config.h"
#include "error.h"

#ifdef CONFIG_SYSTICK
#include "systick.h"
#endif

typedef struct {
    unsigned stream; // stream bitmask value
    uint8_t chan; // hw channel id
} adc_stream_t;

/* @brief Map between index, stream bit, and hw chan id
 * @details NOTE: order must match adc_chan_index_t in host_comm.h
 *          NOTE: size must match ADC_MAX_CHANNELS
 */
static adc_stream_t stream_info[] = {
    { STREAM_VCAP,   ADC_CHAN_VCAP   },
#ifdef ADC_CHAN_VBOOST
    { STREAM_VBOOST, ADC_CHAN_VBOOST },
#endif
#ifdef ADC_CHAN_VREG
    { STREAM_VREG,   ADC_CHAN_VREG   },
#endif
#ifdef ADC_CHAN_VRECT
    { STREAM_VRECT,  ADC_CHAN_VRECT  },
#endif
#ifdef ADC_CHAN_VINJ
    { STREAM_VINJ,   ADC_CHAN_VINJ   },
#endif
};

#ifdef CONFIG_ENABLE_VOLTAGE_STREAM

#define TIMER_ADC_TRIGGER CONCAT(TMRMOD_ADC_TRIGGER, TMRIDX_ADC_TRIGGER)

#define ADC_MAX_CHANNELS  5

#define NUM_BUFFERS                                  2 // double-buffer pair
#define NUM_BUFFERED_SAMPLES                        32

#define SAMPLE_TIMESTAMPS_SIZE (NUM_BUFFERED_SAMPLES * sizeof(uint32_t))
#define SAMPLE_VOLTAGES_SIZE   (NUM_BUFFERED_SAMPLES * ADC_MAX_CHANNELS * sizeof(uint16_t))

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

static unsigned num_channels;

static uint8_t sample_msg_bufs[NUM_BUFFERS][SAMPLES_MSG_BUF_SIZE];

// Can't have a struct type because the number of channels per 'sample' (sequence) varies
static uint32_t * const sample_timestamps_bufs[NUM_BUFFERS] = {
    (uint32_t *)&sample_msg_bufs[0][SAMPLE_TIMESTAMPS_OFFSET],
    (uint32_t *)&sample_msg_bufs[1][SAMPLE_TIMESTAMPS_OFFSET],
};
static uint16_t * const sample_voltages_bufs[NUM_BUFFERS] = {
    (uint16_t *)&sample_msg_bufs[0][SAMPLE_VOLTAGES_OFFSET],
    (uint16_t *)&sample_msg_bufs[1][SAMPLE_VOLTAGES_OFFSET],
};

static unsigned num_samples[NUM_BUFFERS];
static unsigned voltage_sample_offset;
// volatile because main uses it to get the index of the ready buffer
static volatile unsigned sample_buf_idx;
static uint32_t *sample_timestamps_buf;
static uint16_t *sample_voltages_buf;

void ADC_start(uint16_t streams, unsigned sampling_period)
{
    unsigned i;
    unsigned offset;
    uint8_t *header;
    volatile uint8_t *ctl_reg;

    LOG("adc: start: streams 0x%04x period %u\r\n", streams, sampling_period);

    ADC12CTL0 &= ~ADC12ENC; // disable conversion so we can set control bits

    // sequence of channels, single conversion
    ADC12CTL0 = ADC12SHT0_2 + ADC12ON + ADC12MSC; // sampling time, ADC12 on, multiple sample conversion

    // use sampling timer, sequence of channels, repeat-conversion, trigger from Timer B CCR0
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_1 + ADC12SHS_2;

    // set ADC memory control registers and count channels
    num_channels = 0;
    ctl_reg = &ADC12MCTL0;
    for (i = 0; i < ADC_MAX_CHANNELS; ++i) {
        if (streams & stream_info[i].stream) {
            *(ctl_reg++) = stream_info[i].chan;
            num_channels++;
        }
    }
    *(--ctl_reg) |= ADC12EOS;

    ADC12IFG = 0; // clear int flags
    ADC12IE = (0x0001 << (num_channels - 1)); // enable interupt on last sample

    TIMER_CC(TIMER_ADC_TRIGGER, TMRCC_ADC_TRIGGER, CCR) = sampling_period;
    TIMER_CC(TIMER_ADC_TRIGGER, TMRCC_ADC_TRIGGER, CCTL) = OUTMOD_3; // set/reset output mode
    TIMER(TIMER_ADC_TRIGGER, CTL) =
         TIMER_CLK_SOURCE_BITS(TMRMOD_ADC_TRIGGER, CONFIG_ADC_TIMER_SOURCE_NAME) |
         TIMER_DIV_BITS(CONFIG_ADC_TIMER_DIV) |
         MC__UP | TIMER_CLR(TMRMOD_ADC_TRIGGER);

    for (i = 0; i < NUM_BUFFERS; ++i) {
        header = &sample_msg_bufs[i][UART_MSG_HEADER_SIZE];
        offset = 0;
        header[offset++] = streams;
        header[offset++] = 0; // filled in num events once buffer is ready

        num_samples[i] = 0;
    }
    voltage_sample_offset = 0;
    sample_buf_idx = 0;
    sample_timestamps_buf = sample_timestamps_bufs[sample_buf_idx];
    sample_voltages_buf = sample_voltages_bufs[sample_buf_idx];

    ADC12CTL0 |= ADC12ENC; // launch: wait for trigger
}

void ADC_send_samples_to_host()
{
    unsigned ready_buf_idx = sample_buf_idx ^ 1; // the other one in the double-buffer pair

    sample_msg_bufs[ready_buf_idx][UART_MSG_HEADER_SIZE + STREAM_DATA_STREAMS_BITMASK_LEN] =
        num_samples[ready_buf_idx];

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
            num_samples[ready_buf_idx] * sizeof(uint16_t) * num_channels,
            (uint8_t *)&sample_msg_bufs[ready_buf_idx][0]);

    UART_end_transmission();

    num_samples[ready_buf_idx] = 0; // mark buffer as free
}

void ADC_stop()
{
    LOG("adc: stop\r\n");

    ADC12CTL0 &= ~(ADC12SC | ADC12ENC);  // stop conversion and disable ADC
    while (ADC12CTL1 & ADC12BUSY); // conversion stops at end of sequence
}

#endif // CONFIG_ENABLE_VOLTAGE_STREAM

uint16_t ADC_read(unsigned chan_index)
{
    ADC12CTL0 &= ~ADC12ENC; // disable ADC

    ADC12CTL0 = ADC12SHT0_2 + ADC12ON + ADC12REF2_5V + ADC12REFON; // sampling time, ADC12 on
    ADC12CTL1 = ADC12SHP + ADC12CONSEQ_0; // use sampling timer, single-channel, single-conversion
    ADC12MCTL0 = stream_info[chan_index].chan; // set ADC memory control register
    ADC12IE = 0; // disable interrupt

    ADC12CTL0 |= ADC12ENC; // enable ADC

    // Trigger
    ADC12CTL0 &= ~ADC12SC;  // 'start conversion' bit must be toggled
    ADC12CTL0 |= ADC12SC; // start conversion

    while (ADC12CTL1 & ADC12BUSY); // wait for conversion to complete
    uint16_t reading = ADC12MEM0;

    ADC12CTL0 &= ~ADC12ON; // turn ADC off
    return reading;
}

#ifdef CONFIG_ENABLE_VOLTAGE_STREAM

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint32_t timestamp;

    unsigned current_num_samples = num_samples[sample_buf_idx];

    uint16_t iv = ADC12IV;
    ADC12IFG = 0; // clear interrupt flags, since ASSERT enables nesting

    ASSERT(ASSERT_ADC_BUFFER_OVERFLOW, current_num_samples < NUM_BUFFERED_SAMPLES);

#ifdef CONFIG_SYSTICK
    timestamp = SYSTICK_CURRENT_TIME;
#else // !CONFIG_SYSTICK
    timestamp = 0;
#endif // !CONFIG_SYSTICK

    sample_timestamps_buf[current_num_samples] = timestamp;

    switch(__even_in_range(iv,34))
    {
        case ADC12IV_NONE:                         // Vector  0:  No interrupt
        case ADC12IV_ADC12OVIFG:                   // Vector  2:  ADC overflow
        case ADC12IV_ADC12TOVIFG:                  // Vector  4:  ADC timing overflow
            ASSERT(ASSERT_ADC_FAULT, false);

        case ADC12IV_ADC12IFG0:                           // Vector  6:  ADC12IFG0
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM0;
            break;
        case ADC12IV_ADC12IFG1:                           // Vector  8:  ADC12IFG1
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM0;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM1;
            break;
        case ADC12IV_ADC12IFG2:                           // Vector 10:  ADC12IFG2
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM0;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM1;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM2;
            break;
        case ADC12IV_ADC12IFG3:                           // Vector 12:  ADC12IFG3
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM0;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM1;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM2;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM3;
            break;
        case ADC12IV_ADC12IFG4:                           // Vector 14:  ADC12IFG4
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM0;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM1;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM2;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM3;
            sample_voltages_buf[voltage_sample_offset++] = ADC12MEM4;
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

    // If buffer is full, then swap to the other buffer in the double-buffer pair
    if (++(num_samples[sample_buf_idx]) == NUM_BUFFERED_SAMPLES) {

        sample_buf_idx ^= 1;
        sample_timestamps_buf = sample_timestamps_bufs[sample_buf_idx];
        sample_voltages_buf = sample_voltages_bufs[sample_buf_idx];

        voltage_sample_offset = 0;

        main_loop_flags |= FLAG_ADC_COMPLETE;
    }

    ADC12CTL0 |= ADC12ENC;
}
#endif // CONFIG_ENABLE_VOLTAGE_STREAM

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
    main_loop_flags |= FLAG_ADC_COMPLETE;
    TB0CCTL1 &= ~CCIFG;
}
#endif
