/******************************************************************************
 * @file
 * @date    28 April 2015
 * @author  Graham Harvey
 * @brief   Definitions and prototypes for using UARTs with the WISP monitor.
 *****************************************************************************/

#ifndef UART_H
#define UART_H

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
 * @defgroup	UART_TX_FORCING		UART TX blocking
 * @brief		Used to indicate whether to block while sending
 * @{
 */

#define UART_TX_FORCE							1 //!< Wait until there's space in the TX buffer for these bytes
#define UART_TX_DROP							0 //!< If there's no space in the TX buffer, drop this packet

/** @} End UART_TX_FORCING */

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
    unsigned identifier;                     //!< UART message identifier
    unsigned descriptor;                     //!< Message descriptor
    unsigned length;                         //!< Message data length
    unsigned processed;                      //!< Indicates whether the packet structure is free to be overwritten
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
 * @brief Buffer and len available to all modules that may send messages to host
 */
extern uint8_t host_msg_buf[UART_PKT_MAX_DATA_LEN];
extern uint16_t host_msg_len;

/**
 * @brief       Set up UART
 * @param       interface       UART interface to set up.  See @ref UART_INTERFACES
 * @details     The FTDI FT232R USB to UART chip is connected to the
 *              MSP430 on the USCI_A0 UART.  The WISP is connected
 *              on the USCI_A1 UART.  This function sets
 *              registers to configure either interface.
 */
void UART_setup(unsigned interface);

/**
 * @brief       Unsetup UART (put pins into high-z state)
 * @param		interface	UART interface to use.  See @ref UART_INTERFACES
 */
void UART_teardown(unsigned interface);

/**
 * @brief       Block until all bytes are queued in the UART TX buffer
 * @param		interface	UART interface to use.  See @ref UART_INTERFACES
 * @param       buf         Pointer to a buffer to send
 * @param       len         Number of bytes to send
 */
void UART_blockBufferBytes(unsigned interface, uint8_t *buf, unsigned len);

/**
 * @brief		Queue bytes in the UART TX buffer, but drop the bytes if there
 * 				isn't enough space
 * @param		interface	UART interface to use.  See @ref UART_INTERFACES
 * @param		buf			Pointer to a buffer to send
 * @param		len			Number of bytes to send
 */
void UART_dropBufferBytes(unsigned interface, uint8_t *buf, unsigned len);

/**
 * @brief       Construct a UART packet from the UART buffer
 * @param       interface   UART interface to use.  See @ref UART_INTERFACES
 * @param       pkt     Pointer to a uartPkt_t structure in which to store the message
 * @retval      0       Packet construction succeeded
 * @retval      1       Packet construction failure
 * @retval      2       More data is needed to finish constructing the packet
 */
unsigned UART_buildRxPkt(unsigned interface, uartPkt_t *pkt);

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
unsigned UART_RxBufEmpty(unsigned interface);

/**
 * @brief       Copy an array of bytes into a uartBuf_t circular buffer
 * @param       bufInto     Pointer to a uartBuf_t structure to which bytes will be copied
 * @param       bufFrom     Pointer to a byte array from which bytes will be copied
 * @param       len         Number of bytes to copy
 * @warning     This function does not check if there is enough space to copy the bytes.
 *              It will overwrite anything in its way.
 */
static void uartBuf_copyTo(uartBuf_t *bufInto, uint8_t *bufFrom, unsigned len);

/**
 * @brief       Copy bytes from a uartBuf_t circular buffer into an array of bytes
 * @param       bufFrom     Pointer to a uartBuf_t structure from which bytes will be copied
 * @param       bufInto     Pointer to a byte array to which bytes will be copied
 * @param       len         Number of bytes to copy
 * @warning     This function does not check if there is enough space to copy the bytes.
 *              It will overwrite anything in its way.
 */
static void uartBuf_copyFrom(uartBuf_t *bufFrom, uint8_t *bufInto, unsigned len);

#endif // UART_H
