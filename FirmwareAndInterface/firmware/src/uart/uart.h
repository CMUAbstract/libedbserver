/******************************************************************************
 * @file
 * @date    28 April 2015
 * @author  Graham Harvey
 * @brief   Definitions and prototypes for using UARTs with the WISP monitor.
 *****************************************************************************/

#ifndef UART_H
#define UART_H

/**
 * @defgroup    UART_USAGE  UART
 * @brief       UART usage
 * @details     The UART message structure looks like this:\n
 *              | Byte                 | Name                  | Description                                                           |
 *              | -------------------- | --------------------- | --------------------------------------------------------------------- |
 *              | 0                    | @ref UART_IDENTIFIERS | Identifies this as a UART message                                     |
 *              | 1                    | Message descriptor    | See @ref UART_MSG_DESCRIPTORS                                             |
 *              | 2                    | Length                | Length of the upcoming data (optional)                                |
 *              | 3 to (2 + length)    | Data                  | Message data (optional)                                               |
 *
 * @{
 */

/**
 * @defgroup    UART_INTERFACES UART interfaces
 * @brief       Available UART interfaces
 * @{
 */
#define UART_INTERFACE_USB                      0x00 //!< identifier for UART that interfaces with USB
#define UART_INTERFACE_WISP                     0x01 //!< identifier for UART that interfaces with WISP
/** @} End UART_INTERFACES */

/**
 * @defgroup    UART_MACROS     UART macros
 * @brief       Macros for use with UART
 * @{
 */
#define UART_DISABLE_USB_RX                     UCA0IE &= ~UCRXIE	//!< Disable RX interrupt for USB
#define UART_ENABLE_USB_RX                      UCA0IE |= UCRXIE	//!< Enable RX interrupt for USB
#define UART_DISABLE_WISP_RX                    UCA1IE &= ~UCRXIE	//!< Disable RX interrupt for WISP UART
#define UART_ENABLE_WISP_RX                     UCA1IE |= UCRXIE	//!< Enable RX interrupt for WISP UART
/** @} End UART_MACROS */

/**
 * @defgroup    UART_IDENTIFIERS    UART identifiers
 * @brief       Identify a UART message as valid
 * @{
 */
#define UART_USB_IDENTIFIER                     0xF0 //!< First byte of each valid USB message
#define UART_WISP_IDENTIFIER                    0xF1 //!< First byte of each valid UART message with the WISP
/** @} End UART_IDENTIFIERS */

/**
 * @defgroup	UART_TX_FORCING		UART TX blocking
 * @brief		Used to indicate whether to block while sending
 * @{
 */

#define UART_TX_FORCE							1 //!< Wait until there's space in the TX buffer for these bytes
#define UART_TX_DROP							0 //!< If there's no space in the TX buffer, drop this packet

/** @} End UART_TX_FORCING */

/**
 * @defgroup    UART_MSG_DESCRIPTORS    UART message descriptors
 * @brief       Message descriptors defining the type of message being sent.
 * @{
 */

/**
 * @defgroup    USB_MSG_DESCRIPTORS     USB message descriptors
 * @brief       Message descriptors specifically for use with the USB interface.
 * @{
 */

/**
 * @defgroup    USB_CMD    USB command descriptors
 * @brief       Message descriptors sent from the computer to
 *              the WISP monitor over the USB interface.
 * @{
 */
#define USB_CMD_SENSE                 			0x01 //!< Get ADC12 reading of Vcap
#define USB_CMD_STREAM_BEGIN          			0x02 //!< Start streaming ADC12 readings continuously
#define USB_CMD_STREAM_END            			0x03 //!< Stop stream ADC12 readings continuously
#define USB_CMD_SET_VCAP                    	0x04 //!< Inject charge until Vcap has this ADC12 reading, length should be 2
#define USB_CMD_SET_VBOOST                     	0x05 //!< Inject charge until Vboost has this ADC12 reading, length should be 2
#define USB_CMD_SET_VREG                      	0x06 //!< Inject charge until Vreg has this ADC12 reading, length should be 2
#define USB_CMD_SET_VRECT                     	0x07 //!< Inject charge until Vrect has this ADC12 reading, length should be 2
#define USB_CMD_RELEASE_POWER                	0x08 //!< Release the hold on power state now
#define USB_CMD_ENTER_ACTIVE_DEBUG             	0x09 //!< Enter active debug mode by triggering the WISP's port interrupt
#define USB_CMD_EXIT_ACTIVE_DEBUG             	0x0A //!< Exit active debug mode
#define USB_CMD_GET_WISP_PC                     0x0B //!< Get current program counter
#define USB_CMD_LOG_RF_RX_BEGIN                 0x15 //!< Start streaming RF RX data over USB
#define USB_CMD_LOG_RF_RX_END                   0x16 //!< Stop streaming RF RX data
#define USB_CMD_LOG_RF_TX_BEGIN            		0x17 //!< Start streaming RF TX activity on the WISP
#define USB_CMD_LOG_RF_TX_END              		0x18 //!< Stop streaming RF TX activity on the WISP
#define USB_CMD_SEND_RF_TX_DATA                 0x19 //!< Have the monitor send RF TX data
#define USB_CMD_LOG_WISP_UART_BEGIN             0x1A //!< Start streaming UART activity between the WISP and the monitor
#define USB_CMD_LOG_WISP_UART_END               0x1B //!< Stop streaming UART activity
#define USB_CMD_ENABLE_PORT_INT_TAG_PWR         0x1C //!< Enable port interrupts on AUX_2 and AUX_3 to tag the power trace for WISP execution
#define USB_CMD_DISABLE_PORT_INT_TAG_PWR        0x1D //!< Disable port interrupts on AUX_2 and AUX_3
#define USB_CMD_PWM_ON							0x1E //!< Turn charging PWM on
#define USB_CMD_PWM_OFF							0x1F //!< Turn charging PWM off
#define USB_CMD_SET_PWM_FREQUENCY				0x20 //!< Set PWM frequency in SMCLK cycles, length should be 2, default is 16
#define USB_CMD_SET_PWM_DUTY_CYCLE				0x21 //!< Set PWM duty cycle in SMCLK cycles, length should be 2, default is 8
#define USB_CMD_PWM_HIGH						0x24 //!< Set PWM pin to GPIO and output high
#define USB_CMD_PWM_LOW							0x25 //!< Set PWM pin to GPIO and output low
#define USB_CMD_MONITOR_MARKER_BEGIN            0x26 //!< Start periodically reading a code marker GPIO pin
#define USB_CMD_MONITOR_MARKER_END              0x27 //!< Stop periodically reading a code marker GPIO pin
#define USB_CMD_RESET_STATE                     0x28 //!< Reset the state machine
#define USB_CMD_CHARGE                          0x29 //!< Charge WISP capacitor to a given level
#define USB_CMD_DISCHARGE                       0x30 //!< Discharge WISP capacitor to a given level
#define USB_CMD_BREAK_AT_VCAP_LEVEL             0x31 //!< Interrupt execution when Vcap reaches a given level
#define USB_CMD_READ_MEM                        0x32 //!< Read memory contents at an address
#define USB_CMD_WRITE_MEM                       0x33 //!< Write memory contents at an address
#define USB_CMD_CONT_POWER                      0x34 //!< Turn on a continuous power supply to the target
#define USB_CMD_BREAKPOINT                      0x35 //!< Enable/disable breakpoint
#define USB_CMD_INTERRUPT                       0x36 //!< wait for target to be on and enter debug mode
#define USB_CMD_CHARGE_CMP                      0x37 //!< charge Vcap to given level using comparator
#define USB_CMD_DISCHARGE_CMP                   0x38 //!< discharge Vcap to given level using comparator
#define USB_CMD_GET_INTERRUPT_CONTEXT           0x39 //!< ask why target interrupted execution
#define USB_CMD_SERIAL_ECHO                     0x40 //!< test communication with WISP via serial encoding over the signal line
/** @} End USB_CMD */

/**
 * @defgroup    USB_RSP  USB response descriptors
 * @brief       Message descriptors sent from the WISP monitor
 *              to the computer over the USB interface.
 * @{
 */
#define USB_RSP_VOLTAGE                     	0x01 //!< message containing an ADC12 voltage reading
#define USB_RSP_VOLTAGES                    	0x02 //!< message containing an list of ADC12 voltage reading
#define USB_RSP_SET_POWER_COMPLETE              0x04 //!< message signaling that the power level has been set, data = last measured voltage
#define USB_RSP_RELEASE_POWER_COMPLETE          0x05 //!< message signaling that the power state has been released
#define USB_RSP_ADDRESS                         0x06 //!< message containing the WISP program counter
#define USB_RSP_WISP_MEMORY                     0x07 //!< message containing a WISP memory address and contents at that address
#define USB_RSP_RF_RX                           0x08 //!< message containing RF RX data
#define USB_RSP_RF_TX                      		0x09 //!< message containing RF TX data sent by the WISP
#define USB_RSP_UART_WISP_TO_MONITOR            0x0A //!< message containing UART message sent from the WISP to the monitor
#define USB_RSP_UART_MONITOR_TO_WISP            0x0B //!< message containing UART message sent from the monitor to the WISP
#define USB_RSP_TAG_PWR                         0x0C //!< message signaling that the power trace has been tagged here
#define USB_RSP_TIME							0x0D //!< message containing a relative time in current execution (careful: timer will overflow!)
#define USB_RSP_VINJ							0x0E //!< message containing Vinj ADC12 reading
#define USB_RSP_RETURN_CODE                     0x0F //!< message containing a return code indicating success or failure
#define USB_RSP_INTERRUPTED                     0x10 //!< message sent upon entering debug mode (includes saved Vcap level)
#define USB_RSP_SERIAL_ECHO                     0x11 //!< response to serial communication test

/** @} End USB_RSP */
/** @} End USB_MSG_DESCRIPTORS */

/**
 * @defgroup    RETURN_CODE Return codes for return code message
 * @{
 */
#define RETURN_CODE_SUCCESS 0
#define RETURN_CODE_INVALID_ARGS 1
#define RETURN_CODE_BUFFER_TOO_SMALL 2
#define RETURN_CODE_COMM_ERROR 3
#define RETURN_CODE_UNSUPPORTED 4
/** @} End RETURN_CODE */

/**
 * @defgroup    WISP_MSG_DESCRIPTORS    WISP message descriptors
 * @brief       Message descriptors for use with the WISP interface.
 * @{
 */

/**
 * @defgroup    WISP_CMD    WISP command descriptors
 * @brief       Message descriptors sent from the WISP monitor to
 * 				the WISP through UART.
 * @{
 */
#define WISP_CMD_GET_PC							0x00 //!< get program counter
#define WISP_CMD_EXAMINE_MEMORY					0x01 //!< examine WISP memory
#define WISP_CMD_EXIT_ACTIVE_DEBUG				0x02 //!< prepare to exit active debug mode
#define WISP_CMD_READ_MEM         				0x03 //!< read memory contents at an address
#define WISP_CMD_WRITE_MEM         				0x04 //!< write memory contents at an address
#define WISP_CMD_BREAKPOINT        				0x05 //!< enable/disable target-side breakpoint
#define WISP_CMD_GET_INTERRUPT_CONTEXT          0x06 //!< get the reason execution was interrupted
#define WISP_CMD_SERIAL_ECHO                    0x07 //!< send serially encoded data over signal line
/** @} End WISP_CMD */

/**
 * @defgroup    WISP_RSP   WISP response descriptors
 * @brief       Message descriptors sent from the WISP to the
 * 				WISP monitor through UART.
 * @{
 */
#define WISP_RSP_ADDRESS						0x00 //!< message containing an address
#define WISP_RSP_MEMORY							0x01 //!< message containing requested memory content
#define WISP_RSP_BREAKPOINT			            0x02 //!< message acknowledging breakpoint cmd
#define WISP_RSP_INTERRUPT_CONTEXT              0x03 //!< reason execution was interrupted
#define WISP_RSP_SERIAL_ECHO                    0x04 //!< response to the serial test request
/** @} End WISP_RSP */

/** @} End WISP_MSG_DESCRIPTORS */

/** @} End UART_MSG_DESCRIPTORS */

#define UART_BUF_MAX_LEN                            64 //!< Maximum length of the UART buffers
#define UART_BUF_MAX_LEN_WITH_TAIL					(UART_BUF_MAX_LEN + 1) //!< Add a byte because the tail should never point to a full byte
#define UART_PKT_MAX_DATA_LEN                       (UART_BUF_MAX_LEN - 3) //!< Maximum length of the data field of a UART message

/**
 * @brief       Enumeration used to place a UART message into a variable of type uartPkt_t.
 */
typedef enum {
    CONSTRUCT_STATE_IDENTIFIER,     //!< Currently parsing the UART identifier from the software buffer
    CONSTRUCT_STATE_DESCRIPTOR,     //!< Currently parsing the message descriptor from the software buffer
    CONSTRUCT_STATE_DATA_LEN,       //!< Currently parsing length of the data from the software buffer
    CONSTRUCT_STATE_DATA            //!< Currently parsing the message data from the software buffer
} pktConstructState_t;

/**
 * @brief       UART message packet structure
 */
typedef struct {
    uint8_t data[UART_PKT_MAX_DATA_LEN];    //!< Data field of the UART message
    uint8_t identifier;                     //!< UART message identifier
    uint8_t descriptor;                     //!< Message descriptor
    uint8_t length;                         //!< Message data length
    uint8_t processed;                      //!< Indicates whether the packet structure is free to be overwritten
} uartPkt_t;

/**
 * @brief       Circular buffer type for UART communication
 */
typedef struct {
    uint8_t buf[UART_BUF_MAX_LEN_WITH_TAIL];  //!< Circular buffer to be used for UART interfaces
    uint8_t head;                   //!< Relative buffer head
    uint8_t tail;                   //!< Relative buffer tail
    // the tail should never point to byte that contains data
} uartBuf_t;

/**
 * @brief       Set up UART
 * @param       interface       UART interface to set up.  See @ref UART_INTERFACES
 * @param       flag_bitmask    Bitmask where flags will be set on UART Rx and Tx
 * @param       rxFlag          Flag to set in flag_bitmask on UART Rx
 * @param       txFlag          Flag to set in flag_bitmask on UART Tx
 * @details     The FTDI FT232R USB to UART chip is connected to the
 *              MSP430 on the USCI_A0 UART.  The WISP is connected
 *              on the USCI_A1 UART.  This function sets
 *              registers to configure either interface.
 */
void UART_setup(uint8_t interface, uint16_t *flag_bitmask, uint16_t rxFlag, uint16_t txFlag);

/**
 * @brief       Unsetup UART (put pins into high-z state)
 * @param		interface	UART interface to use.  See @ref UART_INTERFACES
 */
void UART_teardown(uint8_t interface);

/**
 * @brief       Block until all bytes are queued in the UART TX buffer
 * @param		interface	UART interface to use.  See @ref UART_INTERFACES
 * @param       buf         Pointer to a buffer to send
 * @param       len         Number of bytes to send
 */
void UART_blockBufferBytes(uint8_t interface, uint8_t *buf, uint8_t len);

/**
 * @brief		Queue bytes in the UART TX buffer, but drop the bytes if there
 * 				isn't enough space
 * @param		interface	UART interface to use.  See @ref UART_INTERFACES
 * @param		buf			Pointer to a buffer to send
 * @param		len			Number of bytes to send
 */
void UART_dropBufferBytes(uint8_t interface, uint8_t *buf, uint8_t len);

/**
 * @brief       Construct a UART packet from the UART buffer
 * @param       interface   UART interface to use.  See @ref UART_INTERFACES
 * @param       pkt     Pointer to a uartPkt_t structure in which to store the message
 * @retval      0       Packet construction succeeded
 * @retval      1       Packet construction failure
 * @retval      2       More data is needed to finish constructing the packet
 */
uint8_t UART_buildRxPkt(uint8_t interface, uartPkt_t *pkt);

/**
 * @brief       Queue a UART message to be sent over the specified interface
 * @param       interface   UART interface on which to send the message.  See @ref UART_INTERFACES
 * @param       descriptor  Message descriptor.  See @ref UART_MSG_DESCRIPTORS
 * @param       data        Pointer to the data to send
 * @param       data_len    Length of the data to send
 * @param		force		Whether to block to buffer or drop packets.  See @ref UART_TX_FORCING
 * @warning     As it is currently implemented, this function will block until
 *              the entire message is written to the software buffer.
 */
void UART_sendMsg(uint8_t interface, uint8_t descriptor, uint8_t *data,
				  uint8_t data_len, uint8_t force);

/**
 * @brief       Determine whether a software UART RX buffer is empty
 * @param       interface   UART interface to check
 * @retval      1           Software buffer is empty
 * @retval      0           Software buffer is not empty or unavailable
 */
uint8_t UART_RxBufEmpty(uint8_t interface);

/**
 * @brief       Determine the length of the circular buffer
 * @param       buf        Pointer to the circular buffer
 * @return      Length of the buffer
 */
inline uint8_t uartBuf_len(uartBuf_t *buf) {
    int16_t diff = buf->tail - buf->head;
    if(diff >= 0) {
        return (uint8_t) diff;
    } else {
        return (uint8_t)(diff + UART_BUF_MAX_LEN_WITH_TAIL);
    }
}

/**
 * @brief       Copy an array of bytes into a uartBuf_t circular buffer
 * @param       bufInto     Pointer to a uartBuf_t structure to which bytes will be copied
 * @param       bufFrom     Pointer to a byte array from which bytes will be copied
 * @param       len         Number of bytes to copy
 * @warning     This function does not check if there is enough space to copy the bytes.
 *              It will overwrite anything in its way.
 */
static void uartBuf_copyTo(uartBuf_t *bufInto, uint8_t *bufFrom, uint8_t len);

/**
 * @brief       Copy bytes from a uartBuf_t circular buffer into an array of bytes
 * @param       bufFrom     Pointer to a uartBuf_t structure from which bytes will be copied
 * @param       bufInto     Pointer to a byte array to which bytes will be copied
 * @param       len         Number of bytes to copy
 * @warning     This function does not check if there is enough space to copy the bytes.
 *              It will overwrite anything in its way.
 */
static void uartBuf_copyFrom(uartBuf_t *bufFrom, uint8_t *bufInto, uint8_t len);

/** @} End UART_USAGE */

#endif // UART_H
