#ifndef HOST_COMM_H
#define HOST_COMM_H

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
    USB_CMD_LOG_RF_RX_BEGIN                 = 0x15, //!< Start streaming RF RX data over USB
    USB_CMD_LOG_RF_RX_END                   = 0x16, //!< Stop streaming RF RX data
    USB_CMD_LOG_RF_TX_BEGIN                 = 0x17, //!< Start streaming RF TX activity on the WISP
    USB_CMD_LOG_RF_TX_END                   = 0x18, //!< Stop streaming RF TX activity on the WISP
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
    USB_RSP_RF_RX                           = 0x08, //!< message containing RF RX data
    USB_RSP_RF_TX                           = 0x09, //!< message containing RF TX data sent by the WISP
    USB_RSP_UART_WISP_TO_MONITOR            = 0x0A, //!< message containing UART message sent from the WISP to the monitor
    USB_RSP_UART_MONITOR_TO_WISP            = 0x0B, //!< message containing UART message sent from the monitor to the WISP
    USB_RSP_TAG_PWR                         = 0x0C, //!< message signaling that the power trace has been tagged here
    USB_RSP_TIME                            = 0x0D, //!< message containing a relative time in current execution (careful: timer will overflow!)
    USB_RSP_VINJ                            = 0x0E, //!< message containing Vinj ADC12 reading
    USB_RSP_RETURN_CODE                     = 0x0F, //!< message containing a return code indicating success or failure
    USB_RSP_INTERRUPTED                     = 0x10, //!< message sent upon entering debug mode (includes saved Vcap level)
    USB_RSP_SERIAL_ECHO                     = 0x11, //!< response to serial communication test
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

#endif // HOST_COMM_H
