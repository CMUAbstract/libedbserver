/**
 * @file
 * @author	Graham Harvey
 * @date	1 May 2015
 * @brief	Prototypes and definitions for debugging functionality on the WISP.
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

#include "pin_assign.h"

// Encode debugger state machine state onto pins
// #define CONFIG_STATE_PINS

// Breakpoint implementation selection (see docs in eval/interactive-debug)
// Must match the same option in firmware/src/config.h
// #define CONFIG_BREAKPOINTS_DEBUGGER_SIDE
#define CONFIG_BREAKPOINTS_TARGET_SIDE

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
#define WISP_CMD_BREAKPOINT             0x05 //!< enable/disable target-side breakpoint
/** @} End WISP_CMD */

/**
 * @defgroup	WISP_RSP				WISP response descriptors
 * @brief		Response descriptors sent from the WISP to the debugger.
 * @{
 */
#define WISP_RSP_ADDRESS                0x00 //!< message containing program counter
#define WISP_RSP_MEMORY					0x01 //!< message containing requested memory content
#define WISP_RSP_BREAKPOINT             0x02 //!< message acknowledging breakpoint cmd
/** @} End WISP_RSP */

/** @} End WISP_MSG_DESCRIPTORS */

#if defined(CONFIG_BREAKPOINTS_DEBUGGER_SIDE)

/** @brief Latency between a code point marker GPIO setting and the signal to enter debug mode 
 *  @details The execution on the target continues for this long after the codepoint.
 */
#define ENTER_DEBUG_MODE_LATENCY_CYCLES 100

// The index argument to breakpoint macros identifies a breakpoint *group*.
// All breakpionts in a group can be enabled/disabled together (not separately).

#if defined(CONFIG_BREAKPOINT_IMPL_C)

#define CODEPOINT(idx) do { \
        GPIO(PORT_CODEPOINT, OUT) = \
            (GPIO(PORT_CODEPOINT, OUT) & ~(BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1))) | \
            (idx << PIN_CODEPOINT_0); \
            __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
        } while (0)
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
        asm ( " BIS.B #0x10, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
        __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
    } while (0)
#define CODEPOINT_2 do { \
        asm ( " BIS.B #0x20, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
        __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
    } while (0)
#define CODEPOINT_3 do { \
        asm ( " BIS.B #0x30, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
        __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
    } while (0)

#define BREAKPOINT_INNER(idx) CODEPOINT_ ## idx
#define BREAKPOINT(idx) BREAKPOINT_INNER(idx)

#endif // CONFIG_BREAKPOINT_IMPL_*

#elif defined(CONFIG_BREAKPOINTS_TARGET_SIDE)

extern volatile uint16_t _debug_breakpoint_enabled;

void request_debug_mode();

#define BREAKPOINT(idx) \
    if (_debug_breakpoint_enabled & (1 << idx)) request_debug_mode()

#else // CONFIG_BREAKPOINTS_*
#error Invalid selection for breakpoint implementation: see CONFIG_BREAKPOINTS_*
#endif // CONFIG_BREAKPOINTS_*

/**
 * @brief	Initialize pins used by the debugger board
 */
void debug_setup();

#endif
