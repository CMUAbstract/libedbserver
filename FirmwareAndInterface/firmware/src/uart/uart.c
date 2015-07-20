/******************************************************************************
 * @file
 * @date    2 March 2015
 * @author  Graham Harvey
 * @brief   Functions for using UARTs with the WISP monitor.
 *****************************************************************************/

#include <stdint.h>
#include <msp430.h>

#include <libdebug/target_comm.h>

#include "host_comm.h"
#include "minmax.h"
#include "pin_assign.h"
#include "config.h"
#include "error.h"
#include "main_loop.h"

#include "uart.h"

#define BRS_BITS_INNER(brs) UCBRS_ ## brs
#define BRS_BITS(brs) BRS_BITS_INNER(brs)

#define BRF_BITS_INNER(brf) UCBRF_ ## brf
#define BRF_BITS(brf) BRF_BITS_INNER(brf)

typedef enum {
    UART_STATUS_TX_BUSY = 0x01,
    UART_STATUS_RX_BUSY = 0x02,
} uart_status_t;

volatile unsigned host_uart_status = 0;

static uartBuf_t usbRx = { .head = 0, .tail = 0 };
static uartBuf_t wispRx = { .head = 0, .tail = 0 };
static uartBuf_t wispTx = { .head = 0, .tail = 0 };


static uint8_t msg[UART_BUF_MAX_LEN];

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
    case UART_INTERFACE_USB:
        // USCI_A0 option select
        GPIO(PORT_UART_USB, SEL) |= BIT(PIN_UART_USB_TX) | BIT(PIN_UART_USB_RX);

        UCA0CTL1 |= UCSWRST;                    // put state machine in reset
        UCA0CTL1 |= UCSSEL__SMCLK;
#ifdef CONFIG_ABORT_ON_USB_UART_ERROR
        UCA0CTL1 |= UCRXEIE;
#endif

        UCA0BR0 = CONFIG_USB_UART_BAUDRATE_BR0;
        UCA0BR1 = CONFIG_USB_UART_BAUDRATE_BR1;
        UCA0MCTL |= 0
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

#if DMA_HOST_UART_TX == 0
        DMACTL0 = DMA0TSEL_17; /* USCA0 tx */
#elif DMA_HOST_UART_TX == 1
        DMACTL0 = DMA1TSEL_17; /* USCA0 tx */
#elif DMA_HOST_UART_TX == 2
        DMACTL1 = DMA2TSEL_17; /* USCA0 tx */
#endif

        DMACTL4 = DMARMWDIS;

        DMA(DMA_HOST_UART_TX, CTL) =
              DMADT_0 /* single */ |
              DMADSTINCR_0 /* dest no inc */ | DMASRCINCR_3 /* src inc */ |
              DMADSTBYTE | DMASRCBYTE | DMALEVEL | DMAIE;

        // DMA(DMA_HOST_UART_TX, SA) = set on each transfer
        DMA(DMA_HOST_UART_TX, DA) = (uint16_t)&UCA0TXBUF;
        // DMA(DMA_HOST_UART_TX, SZ) = set on each transfer

        UCA0CTL1 &= ~UCSWRST;                   // initialize USCI state machine
        UCA0IE |= UCRXIE;                       // enable USCI_A0 Tx + Rx interrupts
        break;

    case UART_INTERFACE_WISP:
        // USCI_A1 option select
        GPIO(PORT_UART_TARGET, SEL) |= BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX);

        UCA1CTL1 |= UCSWRST;                    // put state machine in reset
        UCA1CTL1 |= UCSSEL__SMCLK;              // use SMCLK

        UCA1BR0 = CONFIG_TARGET_UART_BAUDRATE_BR0;
        UCA1BR1 = CONFIG_TARGET_UART_BAUDRATE_BR1;
        UCA1MCTL |= BRS_BITS(CONFIG_TARGET_UART_BAUDRATE_BRS);

        UCA1CTL1 &= ~UCSWRST;                   // initialize USCI state machine
        UCA1IE |= UCRXIE;                       // enable USCI_A1 Tx + Rx interrupts
        break;

    default:
        break;
    }
} // UART_setup

void UART_teardown(unsigned interface)
{
    // Put pins into High-Z state
    switch(interface)
    {
        case UART_INTERFACE_USB:
            UCA0IE &= ~UCRXIE;                      // disable USCI_A1 Tx + Rx interrupts
            UCA0CTL1 |= UCSWRST;                    // put state machine in reset
            GPIO(PORT_UART_USB, SEL) &= ~(BIT(PIN_UART_USB_TX) | BIT(PIN_UART_USB_RX));
            GPIO(PORT_UART_USB, DIR) &= ~(BIT(PIN_UART_USB_TX) | BIT(PIN_UART_USB_RX));
            break;
        case UART_INTERFACE_WISP:
            UCA1IE &= ~UCRXIE;                      // disable USCI_A1 Tx + Rx interrupts
            UCA1CTL1 |= UCSWRST;                    // put state machine in reset
            GPIO(PORT_UART_TARGET, SEL) &= ~(BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX));

            // Briefly pull low to "discharge". WISP is already in High-Z state, so this
            // will not cause energy interference. This is necessary because otherwise,
            // current continues to flow into the WISP after the UART has been disabled (into
            // high-Z). "Discharging" manually by shorting to ground stopped the flow, hence
            // this workaround here. Maybe this is effectively a MOSFET remaining open due
            // to charge at the gate that can't drain anywhere. Might have to do with
            // the level shifter remaining in the drive state instead of detecting high-Z
            // and releasing the output.
            GPIO(PORT_UART_TARGET, OUT) &= ~(BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX));
            GPIO(PORT_UART_TARGET, DIR) |= BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX);

            GPIO(PORT_UART_TARGET, DIR) &= ~(BIT(PIN_UART_TARGET_TX) | BIT(PIN_UART_TARGET_RX));
            break;
    }
}

unsigned UART_RxBufEmpty(unsigned interface)
{
    switch(interface)
    {
    case UART_INTERFACE_USB:
        return usbRx.head == usbRx.tail;
    case UART_INTERFACE_WISP:
        return wispRx.head == wispRx.tail;
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
        ASSERT(ASSERT_UART_ERROR_CIRC_BUF_TAIL, bufInto->tail < UART_BUF_MAX_LEN_WITH_TAIL);
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
        ASSERT(ASSERT_UART_ERROR_CIRC_BUF_HEAD, bufFrom->head < UART_BUF_MAX_LEN_WITH_TAIL);
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
    case UART_INTERFACE_USB:
        uartBuf = &usbRx;
        break;
    case UART_INTERFACE_WISP:
        uartBuf = &wispRx;
        break;
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
        {
            // copy length
            uartBuf_copyFrom(uartBuf, &pkt_byte, sizeof(uint8_t));
            pkt->length = pkt_byte;
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
                ASSERT(ASSERT_UART_ERROR_RX_PKT_LEN, pkt->length < UART_PKT_MAX_DATA_LEN);
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

// TODO: make target version also copy-free
void UART_send_msg_to_target(unsigned descriptor, unsigned data_len, uint8_t *data)
{
    unsigned msg_len = 0;
    unsigned uartBufLen, copyLen;
    uint8_t *buf = msg;

    msg[msg_len++] = UART_IDENTIFIER_WISP;
    msg[msg_len++] = descriptor;
    msg[msg_len++] = data_len;
    msg[msg_len++] = 0; // padding

    while(data_len > 0) {
        msg[msg_len++] = *data++;
        data_len--;
    }

	// loop until we have copied all of buf into the UART TX buffer
    buf = msg;
	while(msg_len > 0) {
		uartBufLen = uartBuf_len(&wispTx);
		copyLen = MIN(UART_BUF_MAX_LEN - uartBufLen, msg_len);
		uartBuf_copyTo(&wispTx, buf, copyLen);
		buf += copyLen;
		msg_len -= copyLen;
	}

    // enable the correct interrupt to start sending data
    UCA1IE |= UCTXIE;
}

void UART_send_msg_to_host(unsigned descriptor, unsigned payload_len, uint8_t *buf)
{
    unsigned len = 0;

    ASSERT(ASSERT_INVALID_PAYLOAD, (host_uart_status & UART_STATUS_TX_BUSY) == 0x0);

    host_uart_status |= UART_STATUS_TX_BUSY;

    DMA(DMA_HOST_UART_TX, CTL) &= ~DMAEN; // should already be disabled, but just in case

    buf[len++] = UART_IDENTIFIER_USB;
    buf[len++] = descriptor;
    buf[len++] = payload_len;
    buf[len++] = 0; // padding

    len += payload_len;

    DMA(DMA_HOST_UART_TX, SA) = (uint16_t)buf;
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

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=DMA_VECTOR
__interrupt void DMA_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(DMA_VECTOR))) DMA_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch (__even_in_range(DMAIV, 16)) {
        case DMA_INTFLAG(DMA_HOST_UART_TX):
            host_uart_status &= ~UART_STATUS_TX_BUSY;
            break;
    }
}

/*
 * USB message ISR
 */
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

#ifdef CONFIG_ABORT_ON_USB_UART_ERROR
        ASSERT(ASSERT_UART_ERROR_OVERFLOW, !(UCA0STAT & UCRXERR & UCOE));
        ASSERT(ASSERT_UART_ERROR_GENERIC,  !(UCA0STAT & UCRXERR));
#endif
        usbRx.buf[usbRx.tail] = UCA0RXBUF; // copy the new byte
        usbRx.tail = (usbRx.tail + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL; // update circular buffer tail

        main_loop_flags |= FLAG_UART_USB_RX;
        break;
    }

    default: break;
    }
}

/*
 * WISP message ISR
 */
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
        wispRx.buf[wispRx.tail] = UCA1RXBUF; // copy the new byte
        wispRx.tail = (wispRx.tail + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL; // update circular buffer tail

        main_loop_flags |= FLAG_UART_WISP_RX;
        break;
    }

    case USCI_UCTXIFG:                        // Vector 4 - TXIFG
    {
        UCA1TXBUF = wispTx.buf[wispTx.head]; // place data in the Tx register
        wispTx.head = (wispTx.head + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL; // update circular buffer headd

        main_loop_flags |= FLAG_UART_WISP_TX;

        if(wispTx.head == wispTx.tail) {
            // the software buffer is empty
            UCA1IE &= ~UCTXIE; // disable TX interrupt
        }
        break;
    }

    default: break;
    }
}
