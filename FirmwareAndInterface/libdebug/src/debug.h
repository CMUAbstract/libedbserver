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

// Passive breakpoints and external breakpoints are exclusive since they share
// the codepoint pins
// #define CONFIG_ENABLE_PASSIVE_BREAKPOINTS

// #define CONFIG_PASSIVE_BREAKPOINT_IMPL_C
#define CONFIG_PASSIVE_BREAKPOINT_IMPL_ASM

#define CONFIG_BREAKPOINT_TEST

typedef enum {
    INTERRUPT_TYPE_NONE = 0,
    INTERRUPT_TYPE_DEBUGGER_REQ,
    INTERRUPT_TYPE_TARGET_REQ,
    INTERRUPT_TYPE_BREAKPOINT,
    INTERRUPT_TYPE_ENERGY_BREAKPOINT,
    INTERRUPT_TYPE_ASSERT,
    INTERRUPT_TYPE_ENERGY_GUARD, // not a true interrupt: execution continues immediately
} interrupt_type_t;

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
#define WISP_CMD_GET_INTERRUPT_CONTEXT  0x06 //!< get reason execution was interrupted
/** @} End WISP_CMD */

/**
 * @defgroup	WISP_RSP				WISP response descriptors
 * @brief		Response descriptors sent from the WISP to the debugger.
 * @{
 */
#define WISP_RSP_ADDRESS                0x00 //!< message containing program counter
#define WISP_RSP_MEMORY					0x01 //!< message containing requested memory content
#define WISP_RSP_BREAKPOINT             0x02 //!< message acknowledging breakpoint cmd
#define WISP_RSP_INTERRUPT_CONTEXT      0x03 //!< reason execution was interrupted
/** @} End WISP_RSP */

/** @} End WISP_MSG_DESCRIPTORS */

#ifdef CONFIG_ENABLE_PASSIVE_BREAKPOINTS

/** @brief Latency between a code point marker GPIO setting and the signal to enter debug mode 
 *  @details The execution on the target continues for this long after the codepoint.
 */
#define ENTER_DEBUG_MODE_LATENCY_CYCLES 100

// The index argument to breakpoint macros identifies a breakpoint *group*.
// All breakpionts in a group can be enabled/disabled together (not separately).

#if defined(CONFIG_PASSIVE_BREAKPOINT_IMPL_C)

#define CODEPOINT(idx) do { \
        GPIO(PORT_CODEPOINT, OUT) = \
            (GPIO(PORT_CODEPOINT, OUT) & ~(BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1))) | \
            (idx << PIN_CODEPOINT_0); \
            __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
        } while (0)
#define PASSIVE_BREAKPOINT_IMPL(idx) CODEPOINT(idx)

#elif defined(CONFIG_PASSIVE_BREAKPOINT_IMPL_ASM)

// NOTE: The preprocessor is not available inside the inline assembly,
// hence the code duplication and the magic numbers. Port address comes from
// msp430fr5969.h and msp430fr5969.cmd. See this post:
// https://e2e.ti.com/support/development_tools/compiler/f/343/t/433587
//
// What we would like ideally would be:
//    #define CODEPOINT(idx) do { \
//        asm ( " BIS.B ((idx + 1) << #PIN_CODEPOINT_0), &PIN_CODEPOINT_REG " ); \
//        asm ( " BIC.B #PIN_CODEPOINT_0 | #PIN_CODEPOINT_1, &PIN_CODEPOINT_REG " ); \
//      } while (0)
//    // NOTE: the +1 is because the encoding of all lines low is not allowed, since
//    //       the debugger needs a pulse that would generate an interrupt.
// but we are stuck with approximating the above by the following:

#define CODEPOINT_0 do { \
        asm ( " BIS.B #0x10, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
        __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
    } while (0)
#define CODEPOINT_1 do { \
        asm ( " BIS.B #0x20, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
        __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
    } while (0)
#define CODEPOINT_2 do { \
        asm ( " BIS.B #0x30, &0x0222 " ); \
        asm ( " BIC.B #0x30, &0x0222 "); \
        __delay_cycles(ENTER_DEBUG_MODE_LATENCY_CYCLES); \
    } while (0)

#define PASSIVE_BREAKPOINT_IMPL_INNER(idx) CODEPOINT_ ## idx
#define PASSIVE_BREAKPOINT_IMPL(idx) PASSIVE_BREAKPOINT_IMPL_INNER(idx)

#endif // CONFIG_PASSIVE_BREAKPOINT_IMPL_*

#endif // CONFIG_ENABLE_PASSIVE_BREAKPOINTS

/**
 * @brief Bitmask that stores the enabled/disabled state of internal breakpoints
 */
extern volatile uint16_t _libdebug_internal_breakpoints;

void request_debug_mode(interrupt_type_t int_type, uint8_t id);

#ifdef CONFIG_ENABLE_PASSIVE_BREAKPOINTS
/**
 * @brief Breakpoint which works by having debugger interrupt the target
 * @details The target encodes the breakpoint index as a pulse on dedicated
 *          CODEPOINT GPIO lines. This pulse triggers and interrupt on the
 *          debugger-side. The debugger checks whether breakpoint is enabled
 *          and if so interrupts the target to enter the active debug mode.
 *
 *          The main drawback of this method is the delay necessary after
 *          the breakpoint that covers the latency between the target hitting
 *          the breakpoint and receiving the interrupt to enter active
 *          debug mode. This is on the order of about 100 (target) cycles.
 *          This overhead is incurred even if the breakpoint is disabled.
 */
#define PASSIVE_BREAKPOINT(idx) PASSIVE_BREAKPOINT_IMPL(idx)
#endif // CONFIG_ENABLE_PASSIVE_BREAKPOINTS

/**
 * @brief Breakpoint whose enable/disable state is stored on the target
 * @details The enable/disable state of an internal breakpoint is stored
 *          in target's non-volatile memory. To enable or disable an internal
 *          breakpoint, the target must be in active debug mode and the
 *          debugger must send the respective command over UART. Up to 16 such
 *          breakpoints (with distinct indexes) can exit, limited only by the
 *          bit-width of the mask, which could easily be increased.
 */
#define INTERNAL_BREAKPOINT(idx) \
    if (_libdebug_internal_breakpoints & (1 << idx)) \
        request_debug_mode(INTERRUPT_TYPE_BREAKPOINT, idx)

#ifndef CONFIG_ENABLE_PASSIVE_BREAKPOINTS
/**
 * @brief Breakpoint whose enable/disable state is controlled by the debugger
 * @details Whether an external breakpoint is enabled or disabled is indicated by the
 *          state of a GPIO line from debugger board to target dedicated to
 *          that one breakpoint index. This line is driven by the debugger.
 *
 *          The advantages over the internal breakpoint are that
 *              (1) the debugger does not need to communicate with the target
 *                  over the UART command interface to enable/disable external
 *                  breakpoints, and
 *              (2) the debugger may condition the breakpoint on the current
 *                  energy level.
 *
 *          The disadvantage is that only as many external breakpoint (with
 *          distinct indexes) as available GPIO lines are supported: currently,
 *          only two (AUX1 and AUX2).
 */
#define EXTERNAL_BREAKPOINT(idx) \
    if (GPIO(PORT_CODEPOINT, IN) & (1 << idx << PIN_CODEPOINT_0)) \
        request_debug_mode(INTERRUPT_TYPE_BREAKPOINT, idx)
#endif // !CONFIG_ENABLE_PASSIVE_BREAKPOINTS

#define ASSERT(idx, cond) \
    if (!(cond)) request_debug_mode(INTERRUPT_TYPE_ASSERT, idx)

#define ENERGY_GUARD_BEGIN() request_debug_mode(INTERRUPT_TYPE_ENERGY_GUARD, 0)
#define ENERGY_GUARD_END() resume_application()

/**
 * @brief	Initialize pins used by the debugger board
 */
void debug_setup();

/**
 * @brief Initiate disconnection from debugger to eventually resume application
 */
void resume_application();

#endif
