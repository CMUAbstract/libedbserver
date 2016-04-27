#ifndef UART_H
#define UART_H

#include <libedb/target_comm.h>
#include <libmsp/clock.h>

#define CONFIG_UART_CLOCK_FREQ CONFIG_SMCLK_FREQ

// UART baudrate specification:
//
// Not UCOS16:
//
// N = SMCLK / BAUD
// BR0 = LSB(floor(N))
// BR1 = MSB(floor(N))
// BRS = (floor(N) - N) * 8
//
// UCOS16:
//
// N = SMCLK / BAUD
// BR0 = LSB(floor(N/16))
// BR1 = MSB(floor(N/16))
// BRF = (floor(N/16) - N/16) * 16
// UCOS16 = 1

#if CONFIG_UART_CLOCK_FREQ == 24576000

#if CONFIG_USB_UART_BAUDRATE == 1000000

#ifdef CONFIG_USB_UART_UCOS16

// N/16 = 24.576
#define CONFIG_USB_UART_BAUDRATE_BR0 0x01
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 9
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 921600

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 1.66666...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x01
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 10
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 576000

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 2.66666...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x02
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 10
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 500000

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 3.072
#define CONFIG_USB_UART_BAUDRATE_BR0 0x03
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 1
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 115200

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 13.3333...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x0d
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 5
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 38400
// N = 640
#define CONFIG_USB_UART_BAUDRATE_BR0 0x80
#define CONFIG_USB_UART_BAUDRATE_BR1 0x02
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 2560
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0x00
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x0a
#define CONFIG_TARGET_UART_BAUDRATE_BRS 0

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 21921792

#if CONFIG_USB_UART_BAUDRATE == 921600
// N = 23.786666...
#define CONFIG_USB_UART_BAUDRATE_BR0 23
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 6

#elif CONFIG_USB_UART_BAUDRATE == 171264

// N = 128
#define CONFIG_USB_UART_BAUDRATE_BR0 128
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 115200

// N = 190.293333...
#define CONFIG_USB_UART_BAUDRATE_BR0 0xBE
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 2

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 2283.52
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0xEB
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x08
#define CONFIG_TARGET_UART_BAUDRATE_BRS 4

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 12500000

#if CONFIG_USB_UART_BAUDRATE == 2000000
#define CONFIG_USB_UART_BAUDRATE_BR0 6
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 2

#elif CONFIG_USB_UART_BAUDRATE == 1000000
#define CONFIG_USB_UART_BAUDRATE_BR0 12
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 4

#elif CONFIG_USB_UART_BAUDRATE == 115200
#define CONFIG_USB_UART_BAUDRATE_BR0 108
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 4

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0x16
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x05
#define CONFIG_TARGET_UART_BAUDRATE_BRS 1

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 6250000

#if CONFIG_USB_UART_BAUDRATE == 115200
#define CONFIG_USB_UART_BAUDRATE_BR0 54
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 2

#endif

#elif CONFIG_UART_CLOCK_FREQ == 24000000

#if CONFIG_USB_UART_BAUDRATE == 2000000
// N = 12
#define CONFIG_USB_UART_BAUDRATE_BR0 12
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 1000000

#ifdef CONFIG_USB_UART_UCOS16
#define CONFIG_USB_UART_BAUDRATE_BR0 1
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRF 8
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#else // !CONFIG_USB_UART_UCOS16
// N = 24
#define CONFIG_USB_UART_BAUDRATE_BR0 24
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0
#endif // !CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 1500000

#ifdef CONFIG_USB_UART_UCOS16
// N = 1
#define CONFIG_USB_UART_BAUDRATE_BR0 1
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRF 0
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 500000
// N = 48
#define CONFIG_USB_UART_BAUDRATE_BR0 48
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 115200

#ifdef CONFIG_USB_UART_UCOS16
// N = 13.0208333...
#define CONFIG_USB_UART_BAUDRATE_BR0 13
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRF 1
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#else // !CONFIG_USB_UART_UCOS16
// N = 208.33333..
#define CONFIG_USB_UART_BAUDRATE_BR0 0xD0
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 3
#endif // !CONFIG_USB_UART_UCOS16

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 2500
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0xC4
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x09
#define CONFIG_TARGET_UART_BAUDRATE_BRS 0

#elif CONFIG_TARGET_UART_BAUDRATE == 115200

// N = 13.0208333...
#define CONFIG_TARGET_UART_BAUDRATE_BR0 13
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0
#define CONFIG_TARGET_UART_BAUDRATE_BRF 0
#define CONFIG_TARGET_UART_BAUDRATE_UCOS16 1

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 12000000

#if CONFIG_USB_UART_BAUDRATE == 2000000
// N = 6
#define CONFIG_USB_UART_BAUDRATE_BR0 6
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 460800
// N = 26.041666... 
#define CONFIG_USB_UART_BAUDRATE_BR0 26
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 115200

// N = 104.16666...
#define CONFIG_USB_UART_BAUDRATE_BR0 104
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 1

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 1250
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0xE2
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x04
#define CONFIG_TARGET_UART_BAUDRATE_BRS 0

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 8192000

#if CONFIG_TARGET_UART_BAUDRATE == 115200
// N = 71.1111...
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0x47
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x00
#define CONFIG_TARGET_UART_BAUDRATE_BRS 1

#endif // CONFIG_TARGET_UART_BAUDRATE

#endif // CONFIG_UART_CLOCK_FREQ_*

#if defined(CONFIG_USB_UART_BAUDRATE) && !defined(CONFIG_USB_UART_BAUDRATE_BR0)
#error Host UART configuration error: no config for selected CONFIG_UART_CLOCK_FREQ
#endif

#if defined(CONFIG_TARGET_UART_BAUDRATE) && !defined(CONFIG_TARGET_UART_BAUDRATE_BR0)
#error Target UART configuration error: no config for selected CONFIG_UART_CLOCK_FREQ
#endif


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

#define UART_MSG_HEADER_SIZE                    4 // marker, msg id, size, padding (must be aligned to 2)

#define UART_BUF_MAX_LEN                        64
#define UART_BUF_MAX_LEN_WITH_TAIL				(UART_BUF_MAX_LEN + 1) //!< Add a byte because the tail should never point to a full byte
#define UART_PKT_MAX_DATA_LEN                   (UART_BUF_MAX_LEN - UART_MSG_HEADER_SIZE)

// TODO: factor out a uart protocol header (even a whole library)
#if UART_PKT_MAX_DATA_LEN < STDIO_PAYLOAD_SIZE
#error UART buffer too small for std io messages from target
#endif

// Must be aligned because we want pointers *within* the buffer to payload field
#if UART_MSG_HEADER_SIZE & 0x1 == 0x1
#error UART message header size must be aligned to 2
#endif

/**
 * @brief       Enumeration used to place a UART message into a variable of type uartPkt_t.
 */
typedef enum {
    CONSTRUCT_STATE_IDENTIFIER,     //!< Currently parsing the UART identifier from the software buffer
    CONSTRUCT_STATE_DESCRIPTOR,     //!< Currently parsing the message descriptor from the software buffer
    CONSTRUCT_STATE_DATA_LEN,       //!< Currently parsing length of the data from the software buffer
    CONSTRUCT_STATE_PADDING,        //!< Padding in msg header for alignment
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
    unsigned head;                   //!< Relative buffer head
    unsigned tail;                   //!< Relative buffer tail
    // the tail should never point to byte that contains data
} uartBuf_t;

typedef enum {
    UART_STATUS_TX_BUSY = 0x01,
    UART_STATUS_RX_BUSY = 0x02,
} uart_status_t;

extern volatile unsigned host_uart_status;

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
 * @brief       Queue a UART message to be sent to the target
 * @param       descriptor  Message descriptor.  See @ref target_comm.h
 * @param       data        Pointer to the data to send
 * @param       data_len    Length of the data to send
 * @warning     As it is currently implemented, this function will block until
 *              the entire message is written to the software buffer.
 */
void UART_send_msg_to_target(unsigned descriptor, unsigned data_len, uint8_t *data);

/**
 * @brief   Send message to host via UART
 * @param   buf             Complete msg buffer (including space for header
 * @param   payload_len     Number of bytes in payload data (excludes msg header)
 * @details This function will fill in the payload length field in the header.
 *          Waits for UART to become available, and returns after setting up
 *          the DMA request.
 */
void UART_send_msg_to_host(unsigned descriptor, unsigned payload_len, uint8_t *buf);

/**
 * @brief   Wraps transmissions (incl. buffer filling) to serialize them
 * @details Waits for DMA to complete and therefore "release" the buffer
 *          in use by the previous transmission.
 */
void UART_begin_transmission();
void UART_end_transmission();

/**
 * @brief       Determine whether a software UART RX buffer is empty
 * @param       interface   UART interface to check
 * @retval      1           Software buffer is empty
 * @retval      0           Software buffer is not empty or unavailable
 */
unsigned UART_RxBufEmpty(unsigned interface);

#endif // UART_H
