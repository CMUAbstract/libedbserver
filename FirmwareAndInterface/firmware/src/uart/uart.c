/******************************************************************************
 * @file
 * @date    2 March 2015
 * @author  Graham Harvey
 * @brief   Functions for using UARTs with the WISP monitor.
 *****************************************************************************/

#include <stdint.h>
#include <msp430.h>
#include "uart.h"
#include "minmax.h"
#include "pin_assign.h"
#include "config.h"

#define BRS_BITS_INNER(brs) UCBRS_ ## brs
#define BRS_BITS(brs) BRS_BITS_INNER(brs)

static uint16_t *pUSBFlags;
static uint16_t *pWISPFlags;
static uint16_t USBRxFlag, USBTxFlag;
static uint16_t WISPRxFlag, WISPTxFlag;

static uartBuf_t usbRx = { .head = 0, .tail = 0 };
static uartBuf_t usbTx = { .head = 0, .tail = 0 };
static uartBuf_t wispRx = { .head = 0, .tail = 0 };
static uartBuf_t wispTx = { .head = 0, .tail = 0 };

static uint8_t msg[UART_BUF_MAX_LEN];

extern inline uint8_t uartBuf_len(uartBuf_t *buf);

void UART_setup(uint8_t interface, uint16_t *flag_bitmask, uint16_t rxFlag, uint16_t txFlag)
{
    switch(interface)
    {
    case UART_INTERFACE_USB:
        pUSBFlags = flag_bitmask;
        USBRxFlag = rxFlag;
        USBTxFlag = txFlag;

        // USCI_A0 option select
        GPIO(PORT_UART_USB, SEL) |= BIT(PIN_UART_USB_TX) | BIT(PIN_UART_USB_RX);

        UCA0CTL1 |= UCSWRST;                    // put state machine in reset
        UCA0CTL1 |= UCSSEL__SMCLK;
#ifdef CONFIG_ABORT_ON_USB_UART_ERROR
        UCA0CTL1 |= UCRXEIE;
#endif

        UCA0BR0 = CONFIG_USB_UART_BAUDRATE_BR0;
        UCA0BR1 = CONFIG_USB_UART_BAUDRATE_BR1;
        UCA0MCTL |= BRS_BITS(CONFIG_USB_UART_BAUDRATE_BRS);

        UCA0CTL1 &= ~UCSWRST;                   // initialize USCI state machine
        UCA0IE |= UCRXIE;                       // enable USCI_A0 Tx + Rx interrupts
        break;

    case UART_INTERFACE_WISP:
        pWISPFlags = flag_bitmask;
        WISPRxFlag = rxFlag;
        WISPTxFlag = txFlag;

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

void UART_teardown(uint8_t interface)
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

void UART_blockBufferBytes(uint8_t interface, uint8_t *buf, uint8_t len)
{
	volatile uint8_t *pUCAXIE;
	uartBuf_t *uartBuf;
    uint8_t uartBufLen, copyLen;

    if(interface == UART_INTERFACE_USB) {
    	uartBuf = &usbTx;
    	pUCAXIE = &UCA0IE;
    } else if(interface == UART_INTERFACE_WISP) {
    	uartBuf = &wispTx;
    	pUCAXIE = &UCA1IE;
    }

	// loop until we have copied all of buf into the UART TX buffer
	while(len > 0) {
		uartBufLen = uartBuf_len(uartBuf);
		copyLen = MIN(UART_BUF_MAX_LEN - uartBufLen, len);
		uartBuf_copyTo(uartBuf, buf, copyLen);
		buf += copyLen;
		len -= copyLen;
	}

    // enable the correct interrupt to start sending data
    *pUCAXIE |= UCTXIE;
}

void UART_dropBufferBytes(uint8_t interface, uint8_t *buf, uint8_t len)
{
	volatile uint8_t *pUCAXIE;
	uartBuf_t *uartBuf;

	if(interface == UART_INTERFACE_USB) {
		uartBuf = &usbTx;
		pUCAXIE = &UCA0IE;
	} else if(interface == UART_INTERFACE_WISP) {
		uartBuf = &wispTx;
		pUCAXIE = &UCA1IE;
	}

	// if there isn't enough space in the software UART buffer, drop these bytes
	if(len <= (UART_BUF_MAX_LEN - uartBuf_len(uartBuf))) {
		// we have enough space in the software UART buffer
		uartBuf_copyTo(uartBuf, buf, len);

		*pUCAXIE |= UCTXIE;
	}
}

uint8_t UART_RxBufEmpty(uint8_t interface)
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

static void uartBuf_copyTo(uartBuf_t *bufInto, uint8_t *bufFrom, uint8_t len)
{
    while(len--) {
        bufInto->buf[bufInto->tail] = *bufFrom++; // copy byte
        bufInto->tail = (bufInto->tail + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL;   // set tail of circular buffer
    }
}

static void uartBuf_copyFrom(uartBuf_t *bufFrom, uint8_t *bufInto, uint8_t len)
{
    while(len--) {
        *bufInto++ = bufFrom->buf[bufFrom->head]; // copy byte
        bufFrom->head = (bufFrom->head + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL;   // set head of circular buffer
    }
}

uint8_t UART_buildRxPkt(uint8_t interface, uartPkt_t *pkt)
{
    static pktConstructState_t state = CONSTRUCT_STATE_IDENTIFIER;
    uartBuf_t *uartBuf;
    uint8_t minUartBufLen; // the buffer length may change if bytes are received while
                           // this function is executing, but there are at least this
                           // many bytes

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
            uartBuf_copyFrom(uartBuf, &(pkt->identifier), sizeof(uint8_t));
            if(pkt->identifier != UART_USB_IDENTIFIER &&
                                    pkt->identifier != UART_WISP_IDENTIFIER) {
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
            uartBuf_copyFrom(uartBuf, &(pkt->descriptor), sizeof(uint8_t));
            minUartBufLen -= sizeof(uint8_t);

            // TODO: parser should not be aware of specific commands
            // check if additional data is needed for this message
            if(interface == UART_INTERFACE_USB) {
				switch(pkt->descriptor)
				{
				case USB_CMD_RELEASE_POWER:
				case USB_CMD_ENTER_ACTIVE_DEBUG:
				case USB_CMD_EXIT_ACTIVE_DEBUG:
				case USB_CMD_GET_WISP_PC:
				case USB_CMD_LOG_RF_RX_BEGIN:
				case USB_CMD_LOG_RF_RX_END:
				case USB_CMD_LOG_RF_TX_BEGIN:
				case USB_CMD_LOG_RF_TX_END:
				case USB_CMD_LOG_WISP_UART_BEGIN:
				case USB_CMD_LOG_WISP_UART_END:
				case USB_CMD_ENABLE_PORT_INT_TAG_PWR:
				case USB_CMD_DISABLE_PORT_INT_TAG_PWR:
				case USB_CMD_PWM_ON:
				case USB_CMD_PWM_OFF:
				case USB_CMD_PWM_HIGH:
				case USB_CMD_PWM_LOW:
				case USB_CMD_MONITOR_MARKER_BEGIN:
				case USB_CMD_MONITOR_MARKER_END:
				case USB_CMD_RESET_STATE:
				case USB_CMD_INTERRUPT:
					// no additional data is needed
					// mark this packet as unprocessed
					pkt->processed = 0;
					state = CONSTRUCT_STATE_IDENTIFIER;
					return 0;   // packet construction succeeded
				case USB_CMD_SENSE:                 // expectin channel index
				case USB_CMD_STREAM_BEGIN:          // expecting channel list
				case USB_CMD_STREAM_END:            // expecting channel list
				case USB_CMD_SET_VCAP:				// expecting 2 data bytes (ADC reading)
				case USB_CMD_SET_VBOOST:			// expecting 2 data bytes (ADC reading)
				case USB_CMD_SET_VREG:				// expecting 2 data bytes (ADC reading)
				case USB_CMD_SET_VRECT:				// expecting 2 data bytes (ADC reading)
				case USB_CMD_CHARGE:                // expecting 2 data bytes (target voltage)
				case USB_CMD_DISCHARGE:             // expecting 2 data bytes (target voltage)
				case USB_CMD_SEND_RF_TX_DATA:
				case USB_CMD_SET_PWM_FREQUENCY:		// expecting 2 data bytes (TB0CCR0 register)
				case USB_CMD_SET_PWM_DUTY_CYCLE:	// expecting 2 data bytes (TB0CCR1 register)
				case USB_CMD_BREAK_AT_VCAP_LEVEL:	// expecting 2 data bytes (voltage level)
				case USB_CMD_READ_MEM:	            // expecting 2 data bytes (mem address)
				case USB_CMD_WRITE_MEM:	            // expecting 3 data bytes (mem address, value)
				case USB_CMD_CONT_POWER:            // expecting 1 data byte  (on/off value)
				case USB_CMD_BREAKPOINT:            // expecting 2 data byte  (index, en/dis value)
				case USB_CMD_CHARGE_CMP:            // expecting 2 data bytes (target voltage)
				case USB_CMD_DISCHARGE_CMP:         // expecting 2 data bytes (target voltage)
				case USB_CMD_GET_INTERRUPT_CONTEXT: // expecting 1 data byte  (ask target)
					// additional data is needed
					state = CONSTRUCT_STATE_DATA_LEN;
					break;
				default:
					// unknown message descriptor
					pkt->processed = 1; // reset packet processed state
					state = CONSTRUCT_STATE_IDENTIFIER;
					return 1;   // packet construction failed
				}
            } else if(interface == UART_INTERFACE_WISP) {
            	switch(pkt->descriptor)
            	{
            	case WISP_RSP_BREAKPOINT:
					// no additional data is needed
					// mark this packet as unprocessed
					pkt->processed = 0;
					state = CONSTRUCT_STATE_IDENTIFIER;
                    return 0; // packet construction succeeded
            	case WISP_RSP_ADDRESS:
            	case WISP_RSP_MEMORY:
            	case WISP_RSP_INTERRUPT_CONTEXT:
            		// additional data is needed
            		state = CONSTRUCT_STATE_DATA_LEN;
            		break;
            	default:
            		// unknown message descriptor
            		pkt->processed = 1; // reset packet processed state
            		state = CONSTRUCT_STATE_IDENTIFIER;
            		return 1; // packet construction failed
            	}
            }
            break;
        case CONSTRUCT_STATE_DATA_LEN:
        {
            // copy length
            uartBuf_copyFrom(uartBuf, &(pkt->length), sizeof(uint8_t));
            minUartBufLen -= sizeof(uint8_t);
            state = CONSTRUCT_STATE_DATA;
            break;
        }
        case CONSTRUCT_STATE_DATA:
            if(minUartBufLen >= pkt->length) {
                // copy the data
                uartBuf_copyFrom(uartBuf, (uint8_t *)(&(pkt->data)), pkt->length);
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

void UART_sendMsg(uint8_t interface, uint8_t descriptor, uint8_t *data,
				  uint8_t data_len, uint8_t force)
{
    uint8_t msg_len = 0;

    switch(interface)
    {
    case UART_INTERFACE_USB:
        msg[msg_len++] = UART_USB_IDENTIFIER;
        break;

    case UART_INTERFACE_WISP:
        msg[msg_len++] = UART_WISP_IDENTIFIER;
        break;

    default:
        break;
    }

    msg[msg_len++] = descriptor;

    // if there's no data, we don't need to send the length of data
    if(data_len != 0) {
        msg[msg_len++] = data_len;
    }

    while(data_len > 0) {
        msg[msg_len++] = *data++;
        data_len--;
    }

    if(force == UART_TX_FORCE) {
    	UART_blockBufferBytes(interface, msg, msg_len);
    } else {
    	UART_dropBufferBytes(interface, msg, msg_len);
    }

    return;
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
        if (UCA0STAT & UCRXERR) {
                GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
                GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_GREEN);
                while(1) {
                    if (UCA0STAT & UCOE)
                        GPIO(PORT_LED, OUT) ^= BIT(PIN_LED_GREEN);
                    __delay_cycles(10000);
                }
        }
#endif
        usbRx.buf[usbRx.tail] = UCA0RXBUF; // copy the new byte
        usbRx.tail = (usbRx.tail + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL; // update circular buffer tail
        *pUSBFlags |= USBRxFlag;        // alert the main loop
        break;
    }

    case USCI_UCTXIFG:                      // Vector 4 - TXIFG
    {
        UCA0TXBUF = usbTx.buf[usbTx.head]; // place data in the TX register to send
        usbTx.head = (usbTx.head + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL; // update circular buffer head
        *pUSBFlags |= USBTxFlag; // alert the main loop

        if(usbTx.head == usbTx.tail) {
            // the software buffer is empty
            UCA0IE &= ~UCTXIE; // disable TX interrupt
        }
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
        *pWISPFlags |= WISPRxFlag; // alert the main loop
        break;
    }

    case USCI_UCTXIFG:                        // Vector 4 - TXIFG
    {
        UCA1TXBUF = wispTx.buf[wispTx.head]; // place data in the Tx register
        wispTx.head = (wispTx.head + sizeof(uint8_t)) % UART_BUF_MAX_LEN_WITH_TAIL; // update circular buffer headd
        *pWISPFlags |= WISPTxFlag; // alert the main loop

        if(wispTx.head == wispTx.tail) {
            // the software buffer is empty
            UCA1IE &= ~UCTXIE; // disable TX interrupt
        }
        break;
    }

    default: break;
    }
}
