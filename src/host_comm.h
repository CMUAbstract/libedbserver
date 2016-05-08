#ifndef HOST_COMM_H
#define HOST_COMM_H

#include <stdint.h>

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

/**
 * @brief Indentifies the source of message to be the debugger
 */
#define UART_IDENTIFIER_USB                     0xF0

/**
 * @brief       Message descriptors sent from the computer to
 *              the WISP monitor over the USB interface.
 */
typedef enum {
    USB_CMD_SENSE                           = 0x01, //!< Get ADC12 reading of Vcap
    USB_CMD_STREAM_BEGIN                    = 0x02, //!< Start streaming ADC12 readings continuously
    USB_CMD_STREAM_END                      = 0x03, //!< Stop stream ADC12 readings continuously
    USB_CMD_SET_VCAP                        = 0x04, //!< Inject charge until Vcap has this ADC12 reading, length should be 2
    USB_CMD_SET_VBOOST                      = 0x05, //!< Inject charge until Vboost has this ADC12 reading, length should be 2
    USB_CMD_SET_VREG                        = 0x06, //!< Inject charge until Vreg has this ADC12 reading, length should be 2
    USB_CMD_SET_VRECT                       = 0x07, //!< Inject charge until Vrect has this ADC12 reading, length should be 2
    USB_CMD_RELEASE_POWER                   = 0x08, //!< Release the hold on power state now
    USB_CMD_ENTER_ACTIVE_DEBUG              = 0x09, //!< Enter active debug mode by triggering the WISP's port interrupt
    USB_CMD_EXIT_ACTIVE_DEBUG               = 0x0A, //!< Exit active debug mode
    USB_CMD_GET_WISP_PC                     = 0x0B, //!< Get current program counter
    USB_CMD_SEND_RF_TX_DATA                 = 0x19, //!< Have the monitor send RF TX data
    USB_CMD_LOG_WISP_UART_BEGIN             = 0x1A, //!< Start streaming UART activity between the WISP and the monitor
    USB_CMD_LOG_WISP_UART_END               = 0x1B, //!< Stop streaming UART activity
    USB_CMD_ENABLE_PORT_INT_TAG_PWR         = 0x1C, //!< Enable port interrupts on AUX_2 and AUX_3 to tag the power trace for WISP execution
    USB_CMD_DISABLE_PORT_INT_TAG_PWR        = 0x1D, //!< Disable port interrupts on AUX_2 and AUX_3
    USB_CMD_PWM_ON                          = 0x1E, //!< Turn charging PWM on
    USB_CMD_PWM_OFF                         = 0x1F, //!< Turn charging PWM off
    USB_CMD_SET_PWM_FREQUENCY               = 0x20, //!< Set PWM frequency in SMCLK cycles, length should be 2, default is 16
    USB_CMD_SET_PWM_DUTY_CYCLE              = 0x21, //!< Set PWM duty cycle in SMCLK cycles, length should be 2, default is 8
    USB_CMD_PWM_HIGH                        = 0x24, //!< Set PWM pin to GPIO and output high
    USB_CMD_PWM_LOW                         = 0x25, //!< Set PWM pin to GPIO and output low
    USB_CMD_MONITOR_MARKER_BEGIN            = 0x26, //!< Start periodically reading a code marker GPIO pin
    USB_CMD_MONITOR_MARKER_END              = 0x27, //!< Stop periodically reading a code marker GPIO pin
    USB_CMD_RESET_STATE                     = 0x28, //!< Reset the state machine
    USB_CMD_CHARGE                          = 0x29, //!< Charge WISP capacitor to a given level
    USB_CMD_DISCHARGE                       = 0x30, //!< Discharge WISP capacitor to a given level
    USB_CMD_BREAK_AT_VCAP_LEVEL             = 0x31, //!< Interrupt execution when Vcap reaches a given level
    USB_CMD_READ_MEM                        = 0x32, //!< Read memory contents at an address
    USB_CMD_WRITE_MEM                       = 0x33, //!< Write memory contents at an address
    USB_CMD_CONT_POWER                      = 0x34, //!< Turn on a continuous power supply to the target
    USB_CMD_BREAKPOINT                      = 0x35, //!< Enable/disable breakpoint
    USB_CMD_INTERRUPT                       = 0x36, //!< wait for target to be on and enter debug mode
    USB_CMD_CHARGE_CMP                      = 0x37, //!< charge Vcap to given level using comparator
    USB_CMD_DISCHARGE_CMP                   = 0x38, //!< discharge Vcap to given level using comparator
    USB_CMD_GET_INTERRUPT_CONTEXT           = 0x39, //!< ask why target interrupted execution
    USB_CMD_SERIAL_ECHO                     = 0x40, //!< test communication with WISP via serial encoding over the signal line
    USB_CMD_DMA_ECHO                        = 0x41, //!< send message to USB UART using DMA
    USB_CMD_ENABLE_TARGET_UART              = 0x42, //!< prepare UART to accept messages from target
    USB_CMD_WATCHPOINT                      = 0x43, //!< enable/disable a watchpoint
    USB_CMD_SET_PARAM                       = 0x44, //!< set a parameter value
    USB_CMD_GET_PARAM                       = 0x45, //!< get a parameter value
    USB_CMD_PERIODIC_PAYLOAD                = 0x46, //!< enable periodic sending of EDB+App data
} usb_cmd_t;

/**
 * @brief       Message descriptors sent from the WISP monitor
 *              to the computer over the USB interface.
 */
typedef enum {
    USB_RSP_VOLTAGE                         = 0x01, //!< message containing an ADC12 voltage reading
    USB_RSP_VOLTAGES                        = 0x02, //!< message containing an list of ADC12 voltage reading
    USB_RSP_SET_POWER_COMPLETE              = 0x04, //!< message signaling that the power level has been set, data = last measured voltage
    USB_RSP_RELEASE_POWER_COMPLETE          = 0x05, //!< message signaling that the power state has been released
    USB_RSP_ADDRESS                         = 0x06, //!< message containing the WISP program counter
    USB_RSP_WISP_MEMORY                     = 0x07, //!< message containing a WISP memory address and contents at that address
    USB_RSP_STREAM_EVENTS                   = 0x08, //!< message containing data from one or more streams
    USB_RSP_STREAM_VOLTAGES                 = 0x09, //!< message containing data a voltage stream
    USB_RSP_UART_WISP_TO_MONITOR            = 0x0A, //!< message containing UART message sent from the WISP to the monitor
    USB_RSP_UART_MONITOR_TO_WISP            = 0x0B, //!< message containing UART message sent from the monitor to the WISP
    USB_RSP_TAG_PWR                         = 0x0C, //!< message signaling that the power trace has been tagged here
    USB_RSP_TIME                            = 0x0D, //!< message containing a relative time in current execution (careful: timer will overflow!)
    USB_RSP_VINJ                            = 0x0E, //!< message containing Vinj ADC12 reading
    USB_RSP_RETURN_CODE                     = 0x0F, //!< message containing a return code indicating success or failure
    USB_RSP_INTERRUPTED                     = 0x10, //!< message sent upon entering debug mode (includes saved Vcap level)
    USB_RSP_ECHO                            = 0x11, //!< response to test commands
    USB_RSP_STDIO                           = 0x12, //!< printf data from target
    USB_RSP_WATCHPOINT                      = 0x13, //!< watchpoint event info
    USB_RSP_PARAM                           = 0x14, //!< configurable parameter value
    USB_RSP_ENERGY_PROFILE                  = 0x15, //!< collected energy profile
} usb_rsp_t;


/**
 * @brief Return codes for return code message
 */
typedef enum {
    RETURN_CODE_SUCCESS                     = 0,
    RETURN_CODE_INVALID_ARGS                = 1,
    RETURN_CODE_BUFFER_TOO_SMALL            = 2,
    RETURN_CODE_COMM_ERROR                  = 3,
    RETURN_CODE_UNSUPPORTED                 = 4,
} return_code_t;

/** @} End UART_PROTOCOL */

typedef enum {
    PARAM_TEST                              = 0,
    PARAM_TARGET_BOOT_VOLTAGE_DL            = 1, //!< regulated voltage threshold for determinining target is on
    PARAM_TARGET_BOOT_LATENCY_KCYCLES       = 2, //!< time for target to start listening for EDB signals after voltage reaches on threshold
} param_t;

/**
 * @brief Specifies the type of breakpoint among ones supported
 *
 * @defails Several distict breakpoint mechanisms are supported
 *          each with its own advantages and disadvantages. See
 *          libdebug/debug.h for details.
 */
typedef enum {
    BREAKPOINT_TYPE_PASSIVE                 = 0,
    BREAKPOINT_TYPE_INTERNAL                = 1,
    BREAKPOINT_TYPE_EXTERNAL                = 2,
    BREAKPOINT_TYPE_BOOT                    = 3,
} breakpoint_type_t;

/**
 * @brief Select which implementation to use for energy breakpoints
 */
typedef enum {
    ENERGY_BREAKPOINT_IMPL_ADC              = 0,
    ENERGY_BREAKPOINT_IMPL_CMP              = 1,
} energy_breakpoint_impl_t;

/**
 * @brief Specify the initiator who caused target execution to be interrupted
 */
typedef enum {
    INTERRUPT_SOURCE_DEBUGGER               = 0,
    INTERRUPT_SOURCE_TARGET                 = 1,
} interrupt_source_t;

/**
 * @brief   Assigns a permanent index to each ADC channels
 *
 * @details This maps a application's name for an ADC channel to an index
 *          understandable to the ADC driver in adc.c.
 *
 *          NOTE: Must match order of stream_info in adc.c.
 */
typedef enum {
    ADC_CHAN_INDEX_VCAP                     = 0,
    ADC_CHAN_INDEX_VBOOST                   = 1,
    ADC_CHAN_INDEX_VREG                     = 2,
    ADC_CHAN_INDEX_VRECT                    = 3,
    ADC_CHAN_INDEX_VINJ                     = 4,
} adc_chan_index_t;

/**
 * @brief Identifies the data stream to collect and deliver over USB UART
 */
typedef enum {
    STREAM_VCAP                             = 0x0001,
    STREAM_VBOOST                           = 0x0002,
    STREAM_VREG                             = 0x0004,
    STREAM_VRECT                            = 0x0008,
    STREAM_VINJ                             = 0x0010,
    STREAM_RF_EVENTS                        = 0x0020,
    STREAM_WATCHPOINTS                      = 0x0040,
} stream_t;

/**
 * @brief Bitmask that covers all streams that come from the ADC (for convenience)
 */
#define ADC_STREAMS \
    (STREAM_VCAP | STREAM_VBOOST | STREAM_VREG | STREAM_VRECT | STREAM_VINJ)

typedef enum {
    CMP_REF_VCC                             = 0,
    CMP_REF_VREF_2_5                        = 1,
    CMP_REF_VREF_2_0                        = 2,
    CMP_REF_VREF_1_5                        = 3,
} comparator_ref_t;


/**
 * @brief Identifiers for RFID events that happen on the target
 * @details Commands are sent from the reader and decoded by the target,
 *          responses are sent by the target to the reader.
 *
 *          Currently, decoding of target transmissions is not supported,
 *          so there is only one response event: a bit that indicates that a
 *          transmission was attempted.
 *
 *          These identifiers in the LSB match the codes in the RFID protocol,
 *          the MSB distinguishes between event types.
 */
typedef enum {
    RF_EVENT_INVALID                    = 0x0000,

    RF_EVENT_CMD_QUERYREP               = 0x0100,
    RF_EVENT_CMD_ACK                    = 0x0140,
    RF_EVENT_CMD_QUERY                  = 0x0180,
    RF_EVENT_CMD_QUERYADJUST            = 0x0190,
    RF_EVENT_CMD_SELECT                 = 0x01A0,
    RF_EVENT_CMD_NAK                    = 0x01C0,
    RF_EVENT_CMD_REQRN                  = 0x01C1,
    RF_EVENT_CMD_READ                   = 0x01C2,
    RF_EVENT_CMD_WRITE                  = 0x01C3,
    RF_EVENT_CMD_KILL                   = 0x01C4,
    RF_EVENT_CMD_LOCK                   = 0x01C5,
    RF_EVENT_CMD_ACCESS                 = 0x01C6,
    RF_EVENT_CMD_BLOCKWRITE             = 0x01C7,
    RF_EVENT_CMD_BLOCKERASE             = 0x01C8,
    RF_EVENT_CMD_BLOCKPERMALOCK         = 0x01C9,
    RF_EVENT_CMD_READBUFFER             = 0x01D2,
    RF_EVENT_CMD_FILEOPEN               = 0x01D3,
    RF_EVENT_CMD_CHALLENGE              = 0x01D4,
    RF_EVENT_CMD_AUTHENTICATE           = 0x01D5,
    RF_EVENT_CMD_SECURECOMM             = 0x01D6,
    RF_EVENT_CMD_AUTHCOMM               = 0x01D7,

    RF_EVENT_RSP_GENERIC                = 0x0200,

    RF_EVENT_ERR_BAD_DELIM              = 0x0E01,
    RF_EVENT_ERR_RT_CAL                 = 0x0E02,
    RF_EVENT_ERR_TR_CAL                 = 0x0E03,
} rf_event_type_t;

/**
 * @defgroup RF_EVENT_TYPE
 * @brief Byte that identifies the RFID event type
 *
 * @details Must match the MSB in rf_event_type_t.
 *
 *          This value repitition could be avoided by using function-macros, but this
 *          header is parsed by Python on the host which can't handle function-macros.
 */
#define RF_EVENT_TYPE_INVALID           0x0000
#define RF_EVENT_TYPE_CMD               0x0100
#define RF_EVENT_TYPE_RSP               0x0200
#define RF_EVENT_TYPE_ERROR             0x0E00
/* @} End RF_EVENT_TYPE */

#define STREAM_DATA_STREAMS_BITMASK_LEN     1
#define STREAM_DATA_PADDING_LEN             1
#define STREAM_DATA_MSG_HEADER_LEN  (STREAM_DATA_STREAMS_BITMASK_LEN + STREAM_DATA_PADDING_LEN)

// The header must be aligned because we need pointers *within* the buffer to payload field
#if STREAM_DATA_MSG_HEADER_LEN & 0x1 == 0x1
#error Stream message header size must be aligned to 2
#endif

#endif // HOST_COMM_H
