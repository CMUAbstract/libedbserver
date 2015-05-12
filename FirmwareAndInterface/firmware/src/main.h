/**
 * @file
 * @author  Graham Harvey
 * @date    14 April 2015
 * @brief   Prototypes and declarations for main.c.
 */

#ifndef MAIN_H
#define MAIN_H

/**
 * @brief   Execute a command received over USB
 * @param   pkt     UART packet containing message information
 */
static void executeUSBCmd(uartPkt_t *pkt);

/**
 * @brief   Adjust the PWM duty cycle according to the ADC result
 *          to try to reach the target power level.
 * @param   adc12Result     ADC12 conversion result to use for comparison to the target
 */
static void adjust_power(uint16_t adc12Result);

/**
 * @brief   Release the hold on WISP power, stopping PWM.
 */
static void release_power();

/**
 * @brief   Add an ADC channel to the adc12 configuration structure
 * @param   pResults_index  Pointer to any of vcap_index, vboost_index, vreg_index, or vrect_index.
 *                          Must correspond to the ADC measurement that will be added
 * @param   channel         Channel to add.  See @ref ADC12_CHANNELS
 */
static void addAdcChannel(uint16_t channel, int8_t *pResults_index);

/**
 * @brief   Remove an ADC channel from the adc12 configuration structure
 * @param   results_index   Any of vcap_index, vboost_index, vreg_index, or vrect_index.
 *                          Must correspond to the ADC measurement that will be removed.
 */
static void removeAdcChannel(int8_t *pResults_index);

/**
 * @brief   Reconfigure and restart ADC defined by the adc12 configuration structure
 */
static void restartAdc();

/**
 * @brief   Blocking read of an ADC channel
 * @param   channel Channel to read
 * @return  ADC12 conversion result
 * @details This function reconfigures the ADC to read only the channel
 *          requested, and returns the result.  It then restores the ADC
 *          to the configuration defined by the adc12 structure.
 */
static uint16_t adc12Read_block(uint16_t channel);

static void complete_active_debug_exit();

#endif // MAIN_H
