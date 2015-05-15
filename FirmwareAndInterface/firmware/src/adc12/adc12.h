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

/**
 * @defgroup    ADC12_MACROS    ADC macros
 * @brief       Macros for controlling the ADC
 * @{
 */
#define ADC12_START                             ADC12CTL0 |= ADC12SC | ADC12ENC     //!< enable ADC and start conversion
#define ADC12_STOP                              ADC12CTL0 &= ~(ADC12SC | ADC12ENC)  //!< stop conversion and disable ADC
#define ADC12_WAIT                              while(ADC12CTL1 & ADC12BUSY)        //!< wait for ADC to stop
#define ADC12_ENABLE_INTERRUPT(n)               ADC12IE = (0x0001 << ((n) - 1))     //!< enable ADC interrupt -- n is the number of active channels
#define ADC12_DISABLE_INTERRUPT                 ADC12IE = 0                         //!< disable ADC interrupt
/** @} End ADC12_MACROS */

/**
 * @defgroup    ADC12_CHANNELS   ADC input channels
 * @{
 */
#define ADC12INCH_VCAP                          ADC12INCH_1 //!< ADC input channel select for Vcap
#define ADC12INCH_VBOOST                        ADC12INCH_2 //!< ADC input channel select for Vboost
#define ADC12INCH_VREG                          ADC12INCH_3 //!< ADC input channel select for Vreg
#define ADC12INCH_VRECT                         ADC12INCH_4 //!< ADC input channel select for Vrect
#define ADC12INCH_VINJ                       	ADC12INCH_5 //!< ADC input channel select for VINJ
/** @} End ADC12_CHANNELS */

/**
 * @brief   ADC12 channel configuration
 */
typedef struct {
    uint16_t channels[5];           //!< ADC channels that will be sampled
    uint8_t num_channels;           //!< number of channels in the channels array to sample in order
} adc12Cfg_t;

/**
 * @brief   Structure for storing ADC12 configuration and accessing results
 */
typedef struct {
	uint32_t timeComplete;			//!< time ADC completed conversion, updated in ADC ISR
    adc12Cfg_t config;              //!< channel configuration
    uint16_t results[5];            //!< ADC conversion results
    uint16_t *pFlags;               //!< pointer to a bit mask that will be set in the ISR - should be checked in main loop
    uint16_t flag_adc12Complete;    //!< flag that will be set in pFlags when conversion completes
} adc12_t;

/**
 * @brief       Configure the 12-bit ADC
 * @param       adc12   Structure containing ADC configuration, flags, and results array
 *
 * @details     An interrupt is triggered when conversion is completed on all
 *              configured channels.  The interrupt sets the flag bit mask
 *              present in the adc12_t data structure.
 */
void ADC12_configure(adc12_t *adc12);

/**
 * @brief       Select ADC12 option for the pin being sampled.
 * @param       channel     ADC12 channel being used for conversion
 *                          See @ref ADC12_CHANNELS
 */
static void ADC12_pinSelect(uint16_t channel);

/** @} end ADC12 */

#endif // ADC12_H
