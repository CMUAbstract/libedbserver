#ifndef TARGET_COMM_H
#define TARGET_COMM_H

/**
 * @defgroup    UART_PROTOCOL  UART interface
 * @brief       UART usage
 * @details     The UART message structure looks like this:\n
 *              | Byte                 | Name                  | Description                            |
 *              | -------------------- | --------------------- | -------------------------------------- |
 *              | 0                    | UART identifier       | Identifies the source of the message   |
 *              | 1                    | Message descriptor    | Identifies the message                 |
 *              | 2                    | Length                | Length of the upcoming data            |
 *              | 3 to (2 + length)    | Data                  | Message data (optional)                |
 *
 * @{
 */

/*
 * @brief Size of generic header for all UART messages
 * @details WARNING: duplicated from firmware/uart.h
 */
#define UART_MSG_HEADER_SIZE           4

/**
 * @brief A magic value prefix in every message comming from the target
 */
#define UART_IDENTIFIER_WISP			0xF1

/**
 * @brief		Command descriptors sent from the debugger to the WISP.
 */
typedef enum {
    WISP_CMD_GET_PC					= 0x00, //!< get WISP program counter
    WISP_CMD_EXAMINE_MEMORY			= 0x01, //!< examine WISP memory
    WISP_CMD_EXIT_ACTIVE_DEBUG		= 0x02, //!< prepare to exit active debug mode
    WISP_CMD_READ_MEM               = 0x03, //!< read memory contents at an address
    WISP_CMD_WRITE_MEM              = 0x04, //!< read memory contents at an address
    WISP_CMD_BREAKPOINT             = 0x05, //!< enable/disable target-side breakpoint
    WISP_CMD_GET_INTERRUPT_CONTEXT  = 0x06, //!< get reason execution was interrupted
    WISP_CMD_SERIAL_ECHO            = 0x07, //!< send serially encoded data over signal line
} wisp_cmd_t;

/**
 * @brief		Response descriptors sent from the WISP to the debugger.
 * @{
 */
typedef enum {
    WISP_RSP_ADDRESS                = 0x00, //!< message containing program counter
    WISP_RSP_MEMORY					= 0x01, //!< message containing requested memory content
    WISP_RSP_BREAKPOINT             = 0x02, //!< message acknowledging breakpoint cmd
    WISP_RSP_INTERRUPT_CONTEXT      = 0x03, //!< reason execution was interrupted
    WISP_RSP_SERIAL_ECHO            = 0x04, //!< response to the serial echo request
} wisp_rsp_t;

/** @} End UART_PROTOCOL */

#define WISP_CMD_MAX_LEN 16

/**
 * @brief Message length for serial communication on the signal line
 */
#define SIG_SERIAL_NUM_BITS 3

/**
 * @defgroup DEBUG_MODE_FLAGS   Debug mode flags
 * @brief Flags that define functionality in debug mode
 * @details NOTE: must update CONFIG_SIG_SERIAL_NUM_BITS when this list changes
 * @{
 */
#define DEBUG_MODE_INTERACTIVE      0x01
#define DEBUG_MODE_WITH_UART        0x02
#define DEBUG_MODE_WITH_I2C         0x04
/** @} End DEBUG_MODE_FLAGS */

#define DEBUG_MODE_FULL_FEATURES    (DEBUG_MODE_INTERACTIVE | DEBUG_MODE_WITH_UART)

/**
 * @defgroup    SIG_SERIAL_BIT_DURATION Signal-line serial protocol bit duration param
 * @brief Interval between bit pulses in serial protocol on the signal line
 *
 * @details The interval is defined on both the debugger and the target: in
 *          terms of cycles of the respective clock.
 *
 *          The encoding code on the target itself takes 4 instructions (excluding the
 *          set+clear pair), but the interrupt latency on the debugger is much
 *          greater and highly variable (because of blocking from other ISRs). This
 *          latency is the bottleneck.
 *
 *          NOTE: Must keep this in sync with the clock choice.
 *
 *          TODO: values for both default clock and the fast (debug mode) clock
 * @{
 */
#define SIG_SERIAL_BIT_DURATION_ON_TARGET       64 // MCLK clock cycles
#define SIG_SERIAL_BIT_DURATION_ON_DEBUGGER     480 // SMCLK cycles
/** @} End SIG_SERIAL_BIT_DURATION */


/** @brief Latency between a code point marker edge and the signal to enter debug mode 
 *
 *  @details This is only relevant for passive breakpoints. The execution on
 *           the target continues after the program counter passes the codepoint
 *           marker. If a breakpoint at this code marker is enabled, then
 *           execution must not continue past the code marker. But, the debugger
 *           cannot read the code marker and react by request debug mode to
 *           be entered imediately (i.e. in less than one target clock cycle).
 *           To prevent the target from executing code after the breakpoint,
 *           each code marker is followed by a delay that exceeds debugger's
 *           reaction latency.
 */
#define ENTER_DEBUG_MODE_LATENCY_CYCLES 100

/**
 * @brief Reason target execution is interrupted
 */
typedef enum {
    INTERRUPT_TYPE_NONE                     = 0,
    INTERRUPT_TYPE_DEBUGGER_REQ             = 1,
    INTERRUPT_TYPE_TARGET_REQ               = 2,
    INTERRUPT_TYPE_BREAKPOINT               = 3,
    INTERRUPT_TYPE_ENERGY_BREAKPOINT        = 4,
    INTERRUPT_TYPE_ENERGY_GUARD             = 5, // not a true interrupt: execution continues immediately
    INTERRUPT_TYPE_ASSERT                   = 6,
} interrupt_type_t;

#endif // TARGET_COMM_H
