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

// #define CONFIG_BREAKPOINT_IMPL_C
#define CONFIG_BREAKPOINT_IMPL_ASM

#define CONFIG_BREAKPOINT_TEST

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

// The index argument to breakpoint macros identifies a breakpoint *group*.
// All breakpionts in a group can be enabled/disabled together (not separately).

#if defined(CONFIG_BREAKPOINT_IMPL_C)

#define CODEPOINT(idx) \
        GPIO(PORT_CODEPOINT, OUT) = \
            (GPIO(PORT_CODEPOINT, OUT) & ~(BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1))) | \
            (idx << PIN_CODEPOINT_0);
#define BREAKPOINT(idx) CODEPOINT(idx)

#elif defined(CONFIG_BREAKPOINT_IMPL_ASM)

// NOTE: The preprocessor is not available inside the inline assembly,
// hence the code duplication and the magic numbers. Port address comes from
// msp430fr5969.h and msp430fr5969.cmd. See this post:
// https://e2e.ti.com/support/development_tools/compiler/f/343/t/433587
//
// What we would like ideally would be:
//    #define CODEPOINT(idx) do { \
//        asm ( " BIS.B (idx << #PIN_CODEPOINT_0), &PIN_CODEPOINT_REG " ); \
//        asm ( " BIC.B #PIN_CODEPOINT_0 | #PIN_CODEPOINT_1, &PIN_CODEPOINT_REG " ); \
//      } while (0)
// but we are stuck with approximating the above by the following:

// Start counting at 1 to correspond to bit values.
#define CODEPOINT_1 do { \
        asm ( " BIS.B #0x20, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
    } while (0)
#define CODEPOINT_2 do { \
        asm ( " BIS.B #0x10, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
    } while (0)
#define CODEPOINT_3 do { \
        asm ( " BIS.B #0x30, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
    } while (0)

#define BREAKPOINT_INNER(idx) CODEPOINT_ ## idx
#define BREAKPOINT(idx) BREAKPOINT_INNER(idx)

#endif // CONFIG_BREAKPOINT_IMPL_*

/**
 * @brief	Initialize pins used by the debugger board
 */
void debug_setup();

#endif
