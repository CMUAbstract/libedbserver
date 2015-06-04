/*******************************************************************************
 * @file
 * @date            10 April 2015
 * @author          Graham Harvey
 * @brief           WISP RFID communication using the WISP monitor.
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include "rfid.h"
#include "pin_assign.h"

#include "timeLog.h"
#include "timer1.h"
#include "uart.h"

#define DATA_BUF_MAX_LEN							30

RFIDstruct rfid;   // inventory state
uint8_t cmd[CMD_BUFSIZE];

uint16_t R_dest_saved;
uint16_t R_bits_saved;
uint16_t R_bitCt_saved;
uint16_t R_newCt_saved;
uint16_t R_pivot_saved;
uint16_t R_scratch0_saved;
uint16_t R_scratch2_saved;
uint16_t R_wakeupBits_saved;

static uint8_t rxDataBuf[DATA_BUF_MAX_LEN];
static uint32_t rxTimeBuf[DATA_BUF_MAX_LEN];
static uint8_t rxDataBufLen = 0;

static uint32_t txTimeBuf[DATA_BUF_MAX_LEN];
static uint8_t txBufLen = 0;

static uint16_t *pFlags;
static uint16_t rxActivityFlag;
static uint16_t txActivityFlag;

void RFID_setup(uint16_t *pFlag_bitmask, uint16_t rxFlag, uint16_t txFlag)
{
	pFlags = pFlag_bitmask;
	rxActivityFlag = rxFlag; // will set this flag in *pFlags when Rx data is ready
	txActivityFlag = txFlag;

    GPIO(PORT_RF, SEL) &= ~BIT(PIN_RF_RX);   // set GPIO function
    GPIO(PORT_RF, DIR) &= ~BIT(PIN_RF_RX);   // set input direction
    GPIO(PORT_RF, IES) |= BIT(PIN_RF_RX);    // interrupt on falling edge
    GPIO(PORT_RF, IFG) &= ~(BIT(PIN_RF_RX) | BIT(PIN_RF_TX));           // clear interrupt flags
    GPIO(PORT_RF, IE) |= BIT(PIN_RF_RX);     // enable port interrupt
}

void RFID_UARTSendRxData()
{
	uint8_t i;
	for(i = 0; i < rxDataBufLen; i++) {
		// send time and Rx data
		UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
				(uint8_t *)(&(rxTimeBuf[i])), sizeof(uint32_t), UART_TX_DROP);
		UART_sendMsg(UART_INTERFACE_USB, USB_RSP_RF_RX,
				(uint8_t *)(&(rxDataBuf[i])), sizeof(uint8_t), UART_TX_DROP);
	}

	rxDataBufLen = 0;
}

void RFID_UARTSendTxData()
{
	uint8_t i;
	for(i = 0; i < txBufLen; i++) {
		// send time and Tx activity
		UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
				(uint8_t *)(&(txTimeBuf[i])), sizeof(uint32_t), UART_TX_DROP);
		UART_sendMsg(UART_INTERFACE_USB, USB_RSP_RF_TX, 0, 0, UART_TX_DROP);
	}

	txBufLen = 0;
}

void RFID_startTxLog()
{
	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);			// clear Tx interrupt flag
	GPIO(PORT_RF, IE) |= BIT(PIN_RF_TX);			// enable Tx interrupt
}

void RFID_stopTxLog()
{
	GPIO(PORT_RF, IE) &= ~BIT(PIN_RF_TX);			// disable interrupt
	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);			// clear interrupt flag

	TIMER1_STOP;
}

void RFID_TxHandler(uint32_t curTime)
{
	txTimeBuf[txBufLen++] = curTime;		// record relative time
	*pFlags |= txActivityFlag;				// alert the main loop to Tx activity

	// Disable the Tx interrupt and set a timer to enable it again.
	// This allows us to only log activity once when a single Tx event occurs,
	// since the Tx line makes several transitions when a transmission occurs.
	GPIO(PORT_RF, IE) &= ~BIT(PIN_RF_TX;)						// disable Tx interrupt

	Timer1_set(FORCE_SKIP_TX_ACTIVITY, &RFID_startTxLog);
}

void RFID_stopRxLog()
{
	TA0CTL = 0;				// disable timer
	TA0CCTL1 = 0;			// reset timer control register
	GPIO(PORT_RF, IE) &= ~BIT(PIN_RF_RX);		// disable Rx port interrupt
	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_RX);		// clear Rx port interrupt flag
}

void handleQR(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();				// record relative time
	rxDataBuf[rxDataBufLen++] = RFID_CMD_QUERYREP;		// record decoded command
	*pFlags |= rxActivityFlag;							// alert the main loop
}

void handleAck(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_ACK;
	*pFlags |= rxActivityFlag;
}

void handleQuery(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_QUERY;
	*pFlags |= rxActivityFlag;
}

void handleQA(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_QUERYADJUST;
	*pFlags |= rxActivityFlag;
}

void handleSelect(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_SELECT;
	*pFlags |= rxActivityFlag;
}

void handleNak(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_NAK;
	*pFlags |= rxActivityFlag;
}

void handleReqRN(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_REQRN;
	*pFlags |= rxActivityFlag;
}

void handleRead(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_READ;
	*pFlags |= rxActivityFlag;
}

void handleWrite(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_WRITE;
	*pFlags |= rxActivityFlag;
}

void handleKill(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_KILL;
	*pFlags |= rxActivityFlag;
}

void handleLock(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_LOCK;
	*pFlags |= rxActivityFlag;
}

void handleAccess(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_ACCESS;
	*pFlags |= rxActivityFlag;
}

void handleBlockWrite(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_BLOCKWRITE;
	*pFlags |= rxActivityFlag;
}

void handleBlockErase(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_BLOCKERASE;
	*pFlags |= rxActivityFlag;
}

void handleBlockPermalock(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_BLOCKPERMALOCK;
	*pFlags |= rxActivityFlag;
}

void handleReadBuffer(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_READBUFFER;
	*pFlags |= rxActivityFlag;
}

void handleFileOpen(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_FILEOPEN;
	*pFlags |= rxActivityFlag;
}

void handleChallenge(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_CHALLENGE;
	*pFlags |= rxActivityFlag;
}

void handleAuthenticate(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_AUTHENTICATE;
	*pFlags |= rxActivityFlag;
}

void handleSecureComm(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_SECURECOMM;
	*pFlags |= rxActivityFlag;
}

void handleAuthComm(void)
{
	rxTimeBuf[rxDataBufLen] = getTime();
	rxDataBuf[rxDataBufLen++] = RFID_CMD_AUTHCOMM;
	*pFlags |= rxActivityFlag;
}
