/*******************************************************************************
 * @file
 * @date            26 March 2015
 * @author          Graham Harvey
 * @brief           Unified Clock System definitions and prototypes
 ******************************************************************************/

#ifndef UCS_H
#define UCS_H

/**
 * @defgroup    UCS     UCS
 * @brief       Usage of the Unified Clock System
 * @{
 */

/**
 * @brief	Set up UCS for ~22MHz operation
 */
void UCS_setup(void);

/**
 * @brief	Set MCLK, SMCLK to (668 + 1) * 32768 Hz = 21921792 Hz (average)
 */
void UCS_set21Mhz();

/**
 * @brief   Set MCLK, SMCLK to 16MHz (average)
 * @note    Source: adapted from 12MHz example file MSP430F534x_UCS_03.c
 */
void UCS_set16MHz();

/**
 * @brief   Set MCLK, SMCLK to 12MHz (average)
 * @note    Source: MSP430 example file MSP430F534x_UCS_03.c
 */
void UCS_set12MHz();

/**
 * @brief   Set MCLK, SMCLK to 8MHz (average)
 * @note    Source: MSP430 example file MSP430534x_UCS_02.c
 */
void UCS_set8MHz();

/**
 * @brief       Set up Vcore to a new level
 *
 * This function comes directly from the sample code for the MSP430F534x
 * provided by TI.  The sample code is available for download at
 * http://www.ti.com/product/MSP430F5340/toolssoftware.
 */
static void SetVcoreUp (unsigned int level);

/** @} End UCS */

#endif
