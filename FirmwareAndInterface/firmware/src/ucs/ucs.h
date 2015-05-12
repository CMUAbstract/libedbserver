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
 * @brief       Set up UCS for 25MHz operation with ACLK
 *              sourced from XT2CLK (12MHz crystal oscillator)
 */
void UCS_setup(void);

void UCS_setMainFreq();

/**
 * @brief   Initialize MCLK, SMCLK to 16MHz
 * @note    Source: adapted from 12MHz example file MSP430F534x_UCS_03.c
 */
void UCS_set16MHz();

/**
 * @brief   Initialize MCLK, SMCLK to 12MHz
 * @note    Source: MSP430 example file MSP430F534x_UCS_03.c
 */
void UCS_set12MHz();

/**
 * @brief   Initialize MCLK, SMCLK to 8MHz
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
