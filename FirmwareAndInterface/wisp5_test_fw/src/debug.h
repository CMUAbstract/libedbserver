/**
 * @file
 * @author	Graham Harvey
 * @date	1 May 2015
 * @brief	Prototypes and definitions for debugging functionality on the WISP.
 */

#ifndef DEBUG_H
#define DEBUG_H

#include "wisp-base.h"

#define DEBUG_UART_BUF_LEN				2

#define UART_IDENTIFIER_WISP			0xF1

/**
 * @defgroup	WISP_MSG_DESCRIPTORS	WISP UART message descriptors
 * @brief		Descriptors for UART communcation between the WISP and
 * 				debugger
 * @{
 */

/**
 * @defgroup	WISP_CMD				WISP command descriptors
 * @brief		Command descriptors sent from the debugger to the WISP.
 * @{
 */
#define WISP_CMD_GET_PC					0x00 //!< get WISP program counter
#define WISP_CMD_EXAMINE_MEMORY			0x01 //!< examine WISP memory
#define WISP_CMD_EXIT_ACTIVE_DEBUG		0x02 //!< prepare to exit active debug mode
/** @} End WISP_CMD */

/**
 * @defgroup	WISP_RSP				WISP response descriptors
 * @brief		Response descriptors sent from the WISP to the debugger.
 * @{
 */
#define WISP_RSP_PC						0x00 //!< message containing program counter
#define WISP_RSP_MEMORY					0x01 //!< message containing requested memory content
/** @} End WISP_RSP */

/** @} End WISP_MSG_DESCRIPTORS */

/* Set up debug interrupt:
 * 	- AUX 1:
 *		- input direction
 *		- rising edge
 *		- enable interrupt
 *	- AUX 2:
 *		- output direction
 *		- output low
 *	- AUX 3:
 *		- output direction
 *		- output low
 */
#define DEBUG_SETUP		P3DIR = P3DIR & ~PIN_AUX1 | PIN_AUX2; \
						P3IES &= ~PIN_AUX1; \
						P3IE |= PIN_AUX1; \
						P3OUT &= ~PIN_AUX2; \
						P1DIR |= PIN_AUX3; \
						P1OUT &= ~PIN_AUX3

typedef struct {
	uint16_t CSCTL0;
	uint16_t CSCTL1;
	uint16_t CSCTL2;
	uint16_t CSCTL3;
	uint16_t CSCTL4;
	uint16_t CSCTL5;
	uint16_t CSCTL6;
} clkInfo_t;

typedef enum {
	MSG_STATE_IDENTIFIER,	//!< UART identifier byte
	MSG_STATE_DESCRIPTOR,	//!< UART descriptor byte
	MSG_STATE_DATALEN,		//!< data length byte
	MSG_STATE_DATA			//!< UART data
} msgState_t;

/**
 * @brief	Debug mode main loop.  This executes when the WISP enters debug mode,
 * 			and should allow debugging functionality.
 */
void debug_main();

/**
 * @brief	Parse and execute a UART message.
 * @param	msg		UART message
 * @param	len		length of msg
 */
void debug_parseAndExecute(uint8_t *msg, uint8_t len);

/**
 * @brief	Prepare to exit debug mode.
 */
void debug_pre_exit();

#endif
