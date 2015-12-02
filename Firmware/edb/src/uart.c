#include <stdint.h>
#include <msp430.h>

#include <libedb/target_comm.h>

#include "host_comm.h"
#include "minmax.h"
#include "pin_assign.h"
#include "config.h"
#include "error.h"
#include "main_loop.h"
#include "dma.h"

#include "uart.h"

// TODO: rename "usb" to "host"

volatile unsigned host_uart_status = 0;

#ifdef UART_HOST
static uartBuf_t usbRx = { .head = 0, .tail = 0 };
#endif // UART_HOST

#ifdef UART_TARGET
static uartBuf_t wispRx = { .head = 0, .tail = 0 };
static uartBuf_t wispTx = { .head = 0, .tail = 0 };
#endif // UART_TARGET

/**
 * @brief       Determine the length of the circular buffer
 * @param       buf        Pointer to the circular buffer
 * @return      Length of the buffer
 */
static inline unsigned uartBuf_len(uartBuf_t *buf) {
    int diff = buf->tail - buf->head;
    if(diff >= 0) {
        return diff;
    } else {
        return (diff + UART_BUF_MAX_LEN_WITH_TAIL);
    }
}

void UART_setup(unsigned interface)
{
    switch(interface)
    {
#ifdef UART_HOST
    case UART_INTERFACE_USB:
        GPIO(PORT_UART_USB, SEL) |= BIT(PIN_UART_USB_TX) | BIT(PIN_UART_USB_RX);

        UART(UART_HOST, CTL1) |= UCSWRST; // put state machine in reset
        UART(UART_HOST, CTL1) |= UCSSEL__SMCLK;
#ifdef CONFIG_ABORT_ON_USB_UART_ERROR
        UART(UART_HOST, CTL1) |= UCRXEIE;
#endif

        UART(UART_HOST, BR0) = CONFIG_USB_UART_BAUDRATE_BR0;
        UART(UART_HOST, BR1) = CONFIG_USB_UART_BAUDRATE_BR1;
        UART(UART_HOST, MCTL) |= 0
#ifdef CONFIG_USB_UART_BAUDRATE_UCOS16
            | UCOS16
#endif
#ifdef CONFIG_USB_UART_BAUDRATE_BRS
            | BRS_BITS(CONFIG_USB_UART_BAUDRATE_BRS)
#endif
#ifdef CONFIG_USB_UART_BAUDRATE_BRF
            | BRF_BITS(CONFIG_USB_UART_BAUDRATE_BRF)
#endif
       ;

        // TX DMA

        DMA(DMA_HOST_UART_TX, CTL) &= ~DMAEN;

        DMA_CTL(DMA_HOST_UART_TX_CTL) =
            DMA_TRIG(DMA_HOST_UART_TX, DMA_TRIG_UART(UART_HOST, TX))

        DMACTL4 = DMARMWDIS;

        DMA(DMA_HOST_UART_TX, CTL) =
              DMADT_0 /* single */ |
              DMADSTINCR_0 /* dest no inc */ | DMASRCINCR_3 /* src inc */ |
              DMADSTBYTE | DMASRCBYTE | DMALEVEL | DMAIE;

        // DMA(DMA_HOST_UART_TX, SA) = set on each transfer
        DMA(DMA_HOST_UART_TX, DA) = (__DMA_ACCESS_REG__)(&UART(UART_HOST, TXBUF));
        // DMA(DMA_HOST_UART_TX, SZ) = set on each transfer

        UART(UART_HOST, CTL1) &= ~UCSWRST; // initialize USCI state machine
        UART(UART_HOST, IE) |= UCRXIE;     // enable Tx + Rx interrupts
        break;
#endif // PORT_PORT_UART_USB

#ifdef UART_TARGET
    case UART_INTERFACE_WISP:
        GPIO(PORT_UART_TARGET, SEL) |=
            BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX);

        UART(UART_TARGET, CTL1) |= UCSWRST; // put state machine in reset
        UART(UART_TARGET, CTL1) |= UCSSEL__SMCLK;

        UART(UART_TARGET, BR0) = CONFIG_TARGET_UART_BAUDRATE_BR0;
        UART(UART_TARGET, BR1) = CONFIG_TARGET_UART_BAUDRATE_BR1;
        UART(UART_TARGET, MCTL) |= 0
#ifdef CONFIG_TARGET_UART_BAUDRATE_UCOS16
            | UCOS16
#endif
#ifdef CONFIG_TARGET_UART_BAUDRATE_BRS
            | BRS_BITS(CONFIG_TARGET_UART_BAUDRATE_BRS)
#endif
#ifdef CONFIG_TARGET_UART_BAUDRATE_BRF
            | BRF_BITS(CONFIG_TARGET_UART_BAUDRATE_BRF)
#endif
       ;

        UART(UART_TARGET, CTL1) &= ~UCSWRST; // initialize USCI state machine
        UART(UART_TARGET, IE) |= UCRXIE;     // enable Tx + Rx interrupts
        break;

    default:
        break;
    }
#endif // PORT_UART_TARGET
} // UART_setup

void UART_teardown(unsigned interface)
{
    // Put pins into High-Z state
    switch(interface)
    {
#ifdef UART_HOST
        case UART_INTERFACE_USB:
            UART(UART_HOST, IE) &= ~UCRXIE;   // disable Tx + Rx interrupts
            UART(UART_HOST, CTL1) |= UCSWRST; // put state machine in reset
            GPIO(PORT_UART_USB, SEL) &=
                ~(BIT(PIN_UART_USB_TX) | BIT(PIN_UART_USB_RX));
            GPIO(PORT_UART_USB, DIR) &=
                ~(BIT(PIN_UART_USB_TX) | BIT(PIN_UART_USB_RX));
            break;
#endif // PORT_UART_USB
#ifdef UART_TARGET
        case UART_INTERFACE_WISP:
            UART(UART_TARGET, IE) &= ~UCRXIE;   // disable Tx + Rx interrupts
            UART(UART_TARGET, CTL1) |= UCSWRST; // put state machine in reset
            GPIO(PORT_UART_TARGET, SEL) &=
                ~(BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX));
            GPIO(PORT_UART_TARGET, DIR) &=
                ~(BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX));
            break;
#endif // PORT_UART_TARGET
    }
}

unsigned UART_RxBufEmpty(unsigned interface)
{
    switch(interface)
    {
#ifdef UART_HOST
    case UART_INTERFACE_USB:
        return usbRx.head == usbRx.tail;
#endif // PORT_UART_USB
#ifdef UART_TARGET
    case UART_INTERFACE_WISP:
        return wispRx.head == wispRx.tail;
#endif
    default:
        return 0;
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
static void uartBuf_copyTo(uartBuf_t *bufInto, uint8_t *bufFrom, unsigned len)
{
    while(len--) {
        ASSERT(ASSERT_UART_ERROR_CIRC_BUF_OVERFLOW, bufInto->tail < UART_BUF_MAX_LEN_WITH_TAIL);
        bufInto->buf[bufInto->tail] = *bufFrom++; // copy byte
        bufInto->tail = (bufInto->tail + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL;   // set tail of circular buffer
    }
}

/**
 * @brief       Copy bytes from a uartBuf_t circular buffer into an array of bytes
 * @param       bufFrom     Pointer to a uartBuf_t structure from which bytes will be copied
 * @param       bufInto     Pointer to a byte array to which bytes will be copied
 * @param       len         Number of bytes to copy
 * @warning     This function does not check if there is enough space to copy the bytes.
 *              It will overwrite anything in its way.
 */
static void uartBuf_copyFrom(uartBuf_t *bufFrom, uint8_t *bufInto, unsigned len)
{
    while(len--) {
        ASSERT(ASSERT_UART_ERROR_CIRC_BUF_OVERFLOW, bufFrom->head < UART_BUF_MAX_LEN_WITH_TAIL);
        *bufInto++ = bufFrom->buf[bufFrom->head]; // copy byte
        bufFrom->head = (bufFrom->head + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL;   // set head of circular buffer
    }
}

unsigned UART_buildRxPkt(unsigned interface, uartPkt_t *pkt)
{
    static pktConstructState_t state = CONSTRUCT_STATE_IDENTIFIER;
    uartBuf_t *uartBuf;
    unsigned minUartBufLen; // the buffer length may change if bytes are received while
                           // this function is executing, but there are at least this
                           // many bytes
    uint8_t pkt_byte;

    if(!(pkt->processed)) {
        // don't overwrite the existing RX packet
        return 1;
    }

    switch(interface)
    {
#ifdef UART_HOST
    case UART_INTERFACE_USB:
        uartBuf = &usbRx;
        break;
#endif // PORT_UART_USB
#ifdef UART_TARGET
    case UART_INTERFACE_WISP:
        uartBuf = &wispRx;
        break;
#endif // PORT_UART_TARGET
    default:
        // unknown interface
        state = CONSTRUCT_STATE_IDENTIFIER;
        pkt->processed = 1;
        return 1;
    }

    minUartBufLen = uartBuf_len(uartBuf);
    while(minUartBufLen > 0) {
        switch(state)
        {
        case CONSTRUCT_STATE_IDENTIFIER:
            // copy identifier to packet structure
            uartBuf_copyFrom(uartBuf, &pkt_byte, sizeof(uint8_t));
            pkt->identifier = pkt_byte;
            if(pkt->identifier != UART_IDENTIFIER_USB &&
                                    pkt->identifier != UART_IDENTIFIER_WISP) {
                // unknown identifier
                pkt->processed = 1; // reset packet processed state
                state = CONSTRUCT_STATE_IDENTIFIER;
                return 1;
            }
            minUartBufLen -= sizeof(uint8_t);
            state = CONSTRUCT_STATE_DESCRIPTOR;
            break;
        case CONSTRUCT_STATE_DESCRIPTOR:
            // copy descriptor
            uartBuf_copyFrom(uartBuf, &pkt_byte, sizeof(uint8_t));
            pkt->descriptor = pkt_byte;
            minUartBufLen -= sizeof(uint8_t);
            state = CONSTRUCT_STATE_DATA_LEN;
            break;
        case CONSTRUCT_STATE_DATA_LEN:
            uartBuf_copyFrom(uartBuf, &pkt_byte, sizeof(uint8_t));
            pkt->length = pkt_byte;
            minUartBufLen -= sizeof(uint8_t);
            state = CONSTRUCT_STATE_PADDING;
            break;
        case CONSTRUCT_STATE_PADDING:
        {
            uartBuf_copyFrom(uartBuf, &pkt_byte, sizeof(uint8_t));
            minUartBufLen -= sizeof(uint8_t);
            if (pkt->length > 0) {
                state = CONSTRUCT_STATE_DATA;
            } else { // done
                pkt->processed = 0;
                state = CONSTRUCT_STATE_IDENTIFIER;
                return 0;   // packet construction succeeded
            }
            break;
        }
        case CONSTRUCT_STATE_DATA:
            if(minUartBufLen >= pkt->length) {
                // copy the data
                if (pkt->length > UART_PKT_MAX_DATA_LEN) {
                    pkt->processed = 1; // reset packet processed state
                    state = CONSTRUCT_STATE_IDENTIFIER;
                    return 1;
                }
                uartBuf_copyFrom(uartBuf, pkt->data, pkt->length);
                // no need to update the minUartBufLen, since packet construction is complete
                // mark this packet as unprocessed
				pkt->processed = 0;
                state = CONSTRUCT_STATE_IDENTIFIER;
                return 0; // packet construction succeeded
            } else {
                // not enough data
                return 2; // packet construction will resume the next time this function is called
            }
        default:
            // unknown state, so reset the packet construction state
            pkt->processed = 1;
            state = CONSTRUCT_STATE_IDENTIFIER;
            return 1;       // packet construction failed
        }
    }

    // ran out of data
    return 2;   // packet construction will resume the next time this function is called
}

static inline unsigned write_header(uint8_t *buf,
                                    unsigned identifier, unsigned descriptor,
                                    unsigned payload_len)
{
    unsigned len = 0;

    buf[len++] = identifier;
    buf[len++] = descriptor;
    buf[len++] = payload_len;
    buf[len++] = 0; // padding

    len += payload_len;

    return len;
}

// TODO: make target version also copy-free
void UART_send_msg_to_target(unsigned descriptor, unsigned payload_len, uint8_t *buf)
{
    unsigned len;
    unsigned uartBufLen, copyLen;
    uint8_t *byte_ptr;

    len = write_header(buf, UART_IDENTIFIER_WISP, descriptor, payload_len);

	// loop until we have copied all of buf into the UART TX buffer
    byte_ptr = buf;
	while(len > 0) {
		uartBufLen = uartBuf_len(&wispTx);
		copyLen = MIN(UART_BUF_MAX_LEN - uartBufLen, len);
		uartBuf_copyTo(&wispTx, byte_ptr, copyLen);
		byte_ptr += copyLen;
		len -= copyLen;
	}

    // enable the correct interrupt to start sending data
    UART(UART_TARGET, IE) |= UCTXIE;
}

#ifdef UART_HOST

void UART_send_msg_to_host(unsigned descriptor, unsigned payload_len, uint8_t *buf)
{
    unsigned len;

    while (host_uart_status & UART_STATUS_TX_BUSY);
    host_uart_status |= UART_STATUS_TX_BUSY;

    DMA(DMA_HOST_UART_TX, CTL) &= ~DMAEN; // should already be disabled, but just in case

    len = write_header(buf, UART_IDENTIFIER_USB, descriptor, payload_len);

    DMA(DMA_HOST_UART_TX, SA) = (__DMA_ACCESS_REG__)buf;
    DMA(DMA_HOST_UART_TX, SZ) = len;

    DMA(DMA_HOST_UART_TX, CTL) |= DMAEN;
}

static inline void UART_wait_for_tx_dma()
{
    while (host_uart_status & UART_STATUS_TX_BUSY) {
        __delay_cycles(10);
    }
}

void UART_begin_transmission()
{
    UART_wait_for_tx_dma();
}

void UART_end_transmission()
{
    UART_wait_for_tx_dma();
}

#endif // UART_HOST

// TODO: factor these two into on_rx_int(...):

#ifdef UART_HOST
static inline void on_host_rx_int()
{
    usbRx.buf[usbRx.tail] = UART(UART_HOST, RXBUF); // copy the new byte

    // update circular buffer tail
    usbRx.tail = (usbRx.tail + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL;

    main_loop_flags |= FLAG_UART_USB_RX;
}
#endif // UART_HOST

#ifdef UART_TARGET
static inline void on_target_rx_int()
{
    wispRx.buf[wispRx.tail] = UART(UART_TARGET, RXBUF); // copy the new byte

    // update circular buffer tail
    wispRx.tail = (wispRx.tail + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL;

    main_loop_flags |= FLAG_UART_WISP_RX;
}

static inline void on_target_tx_int()
{
    UART(UART_TARGET, TXBUF) = wispTx.buf[wispTx.head];

    // update circular buffer headd
    wispTx.head = (wispTx.head + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL;

    main_loop_flags |= FLAG_UART_WISP_TX;

    if (wispTx.head == wispTx.tail) {
        // the software buffer is empty
        UART(UART_TARGET, IE) &= ~UCTXIE; // disable TX interrupt
    }
}
#endif // UART_TARGET

#if (defined(UART_HOST) && UART_HOST == 0) || (defined(UART_TARGET) && UART_TARGET == 0)

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA0IV,USCI_UCTXIFG))
    {
    case USCI_NONE:break;                   // Vector 0 - no interrupt

    case USCI_UCRXIFG:                      // Vector 2 - RXIFG
    {
#if defined(UART_HOST) && UART_HOST == 0

#ifdef CONFIG_ABORT_ON_USB_UART_ERROR
        ASSERT(ASSERT_UART_FAULT,  !(UCA0STAT & UCRXERR));
#endif

        on_host_rx_int();
#elif defined(UART_TARGET) && UART_TARGET == 0
        on_target_rx_int();
#endif
        break;
    }

    default: break;
    }
}
#endif // UART A0 users

#if (defined(UART_HOST) && UART_HOST == 1) || (defined(UART_TARGET) && UART_TARGET == 1)

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(UCA1IV,USCI_UCTXIFG))
    {
    case USCI_NONE:break;                     // Vector 0 - no interrupt

    case USCI_UCRXIFG:                        // Vector 2 - RXIFG
    {
#if defined(UART_HOST) && UART_HOST == 1

#ifdef CONFIG_ABORT_ON_USB_UART_ERROR
        ASSERT(ASSERT_UART_FAULT,  !(UCA1STAT & UCRXERR));
#endif

        on_host_rx_int();
#elif defined(UART_TARGET) && UART_TARGET == 1
        on_target_rx_int();
#endif
        break;
    }

    case USCI_UCTXIFG:                        // Vector 4 - TXIFG
    {
#if defined(UART_TARGET) && UART_TARGET == 1
        on_target_tx_int();
#endif
        break;
    }

    default: break;
    }
}
#endif // UART A1 users
