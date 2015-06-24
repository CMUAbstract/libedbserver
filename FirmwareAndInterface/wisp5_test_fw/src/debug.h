/**
 * @file
 * @author	Graham Harvey
 * @date	1 May 2015
 * @brief	Prototypes and definitions for debugging functionality on the WISP.
 */

#ifndef DEBUG_H
#define DEBUG_H

#include "wisp-base.h"
#include "pin_assign.h"

// Encode debugger state machine state onto pins
// #define CONFIG_STATE_PINS

#define DEBUG_UART_BUF_LEN				2
#define DEBUG_CMD_MAX_LEN               16

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
#define WISP_CMD_READ_MEM               0x03 //!< read memory contents at an address
#define WISP_CMD_WRITE_MEM              0x04 //!< read memory contents at an address
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

/**
 * @brief	Initialize pins used by the debugger board
 */
void debug_setup();

#endif
