/*******************************************************************************
 * @file
 * @date            21 March 2015
 * @author          Graham Harvey
 * @brief           Definitions and prototypes for using the 12-bit ADC
 *                  with the WISP debugger.
 ******************************************************************************/

#ifndef ADC12_H
#define ADC12_H

#include <stdint.h>
#include <msp430.h>

/**
 * @defgroup    ADC12   ADC12
 * @brief       Usage of the 12-bit ADC
 * @{
 */

#define ADC12_MAX_CHANNELS  5

/**
 * @brief   Static info about ADC channels
 */
typedef struct {
    uint8_t chan_mask;
    volatile uint8_t *port;
    uint8_t pin;
} adc12Chan_t;

typedef enum {
    ADC12_MODE_INTERRUPT,
    ADC12_MODE_POLLING,
} adc12Mode_t;

/**
 * @brief   ADC12 channel configuration
 */
typedef struct {
    adc12Chan_t chan_assigns[ADC12_MAX_CHANNELS]; //<! maps a permanent index to a pin designation (must be filled out by the user statically or before first call to ADC12_addChannel)
    uint16_t channels[ADC12_MAX_CHANNELS];           //!< ADC channels that will be sampled
    uint8_t num_channels;           //!< number of channels in the channels array to sample in order
} adc12Cfg_t;

/**
 * @brief   Structure for storing ADC12 configuration and accessing results
 */
typedef struct {
	uint32_t timeComplete;			//!< time ADC completed conversion, updated in ADC ISR
    adc12Cfg_t config;              //!< channel configuration
    uint16_t results[ADC12_MAX_CHANNELS];            //!< ADC conversion results
    int8_t indexes[ADC12_MAX_CHANNELS];            //!< channel index (never changes) -> index in 'results' array
    uint16_t *pFlags;               //!< pointer to a bit mask that will be set in the ISR - should be checked in main loop
    uint16_t flag_adc12Complete;    //!< flag that will be set in pFlags when conversion completes
} adc12_t;

/**
 * @brief       Initialize the ADC device instance object
 * @param       adc12   Structure containing ADC configuration, flags, and results array
 */
void ADC12_init(adc12_t *adc12);

/**
 * @brief       Configure the 12-bit ADC
 * @param       adc12   Structure containing ADC configuration, flags, and results array
 *
 * @details     An interrupt is triggered when conversion is completed on all
 *              configured channels.  The interrupt sets the flag bit mask
 *              present in the adc12_t data structure.
 */
void ADC12_configure(adc12_t *adc12, adc12Mode_t mode);

/**
 * @brief   Add an ADC channel to the adc12 configuration structure
 * @param   chan_index Permanent index assigned to the ADC channel
 */
void ADC12_addChannel(adc12_t *adc12, uint8_t chan_index);

/**
 * @brief   Remove an ADC channel from the adc12 configuration structure
 * @param  chan_index   Permanent index assigned to the ADC channel
 */
void ADC12_removeChannel(adc12_t *adc12, uint8_t chan_index);

/**
 * @brief       Start an ADC conversion in the mode configured previously
 */
void ADC12_start();

/**
 * @brief       Stop the ADC conversion and disable the ADC
 */
void ADC12_stop();

/**
 * @brief       Wait for the ADC to complete conversion
 */
void ADC12_wait();

/**
 * @brief   Reconfigure and restart ADC defined by the adc12 configuration structure
 */
void ADC12_restart(adc12_t *adc12);

/**
 * @brief   Blocking read of an ADC channel
 * @param   channel Channel to read
 * @return  ADC12 conversion result
 * @details This function reconfigures the ADC to read only the channel
 *          requested, and returns the result.  It then restores the ADC
 *          to the configuration defined by the adc12 structure.
 */
uint16_t ADC12_read(adc12_t *adc12, uint16_t channel);

/**
 * @brief   Retrieve a recorded sample from memory
 *
 * @details Call this to get the data after an interrupt notified that
 *          conversion has completed.
 */
uint16_t ADC12_getSample(adc12_t *adc12, uint16_t chan_index);

/** @} end ADC12 */

#endif // ADC12_H
