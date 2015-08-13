#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <msp430.h>

/**
 * @defgroup    ADC12   ADC12
 * @brief       Usage of the 12-bit ADC
 * @{
 */

/**
 * @brief       Configure the 12-bit ADC
 * @param       streams Bitmask of which channels to sample (see stream_t in host_comm.h)
 */
void ADC_start(uint16_t streams, unsigned sampling_period);

/**
 * @brief       Stop the ADC conversion and disable the ADC
 */
void ADC_stop();

/**
 * @brief   Blocking read of an ADC channel
 * @param   chan_index   Permanent index assigned to the ADC channel
 * @return  ADC12 conversion result
 * @details This function reconfigures the ADC to read only the channel
 *          requested, and returns the result.
 */
uint16_t ADC_read(unsigned chan_index);

/**
 * @brief   Send buffered samples to host via UART
 * @details Called by main when the ADC module notifies it that a buffer in the
 *          double-buffer pair has been filled and is ready for transmission.
 */
void ADC_send_samples_to_host();

/** @} end ADC12 */

#endif // ADC_H
