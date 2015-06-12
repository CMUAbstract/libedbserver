/*******************************************************************************
 * @file
 * @date            9 April 2015
 * @author          Graham Harvey
 * @brief           Definitions and prototypes for WISP RFID communication
 *                  using the WISP monitor.
 ******************************************************************************/

#ifndef RFID_H
#define RFID_H

#include <stdint.h>

#include <msp430.h>

#include "pin_assign.h"

// from wisp-base project -- globals.h
// ====================================================
#define TRUE                                1
#define FALSE                               0
#define CMD_BUFSIZE                         30

#define CMD_PARSE_AS_QUERY_REP          (0x00)                          /** @todo   describe these myst vals!                   */
#define CMD_PARSE_AS_OVF                (0xFF)
#define ENOUGH_BITS_TO_FORCE_EXECUTION  (200)

#define RESET_BITS_VAL                      -1        /* this is the value which will reset the TA1_SM if found in 'bits (R5)' by rfid_sm         */

//RFID TIMING DEFS                      /*4.08MHz                                                                                   */                                                                  */
//Impinj Uses RTCal = 31.4us = 2.5*TARI (min possible.   RTCal goes between 2.5-3.0 TARI Per Spec)
#define RTCAL_MIN       (2*116)//(2*85)
#define RTCAL_NOM       (2*171)//(2*125)     /* (really only saw spread from 102 at high power to 96 at low power)                       */
#define RTCAL_MAX       (2*206)//(2*150)     /*(this accounts for readers who use up to 3TARI, plus a little wiggle room)                */
#define RTCAL_OFFS      (2*16)//(2*12)      /* see documentation for notes.                                                             */

//Impinj Uses TRCal = 50.2us = 1.6*RTCAL(middle of road. TRCal goes between 1.1-3 RTCAL per spec. also said, 2.75-9.00 TARI)
#define TRCAL_MIN       (2*191)//(2*140)
#define TRCAL_NOM       (2*363)//(2*265)     /* (really only saw spread from 193 at high power to 192 at low power)                      */
#define TRCAL_MAX       (2*618)//(2*451)     /* (this accounts for readers who use up to 9TARI)                                          */

#define FORCE_SKIP_INTO_RTCAL   (42)//(24)                    /* after delim, wait till data0 passes before starting TA1_SM. note     */
                                                            /* changing this will affect timing criteria on RTCal measurement       */
// ====================================================

#define FORCE_SKIP_TX_ACTIVITY	(1096) //!< SMCLK cycles to skip Tx activity after it starts (~36us, but this gives us ~50us for safety)


// descriptors to define RFID commands
#define RFID_CMD_QUERYREP				0x00
#define RFID_CMD_ACK					0x40
#define RFID_CMD_QUERY					0x80
#define RFID_CMD_QUERYADJUST			0x90
#define RFID_CMD_SELECT					0xA0
#define RFID_CMD_NAK					0xC0
#define RFID_CMD_REQRN					0xC1
#define RFID_CMD_READ					0xC2
#define RFID_CMD_WRITE					0xC3
#define RFID_CMD_KILL					0xC4
#define RFID_CMD_LOCK					0xC5
#define RFID_CMD_ACCESS					0xC6
#define RFID_CMD_BLOCKWRITE				0xC7
#define RFID_CMD_BLOCKERASE				0xC8
#define RFID_CMD_BLOCKPERMALOCK			0xC9
#define RFID_CMD_READBUFFER				0xD2
#define RFID_CMD_FILEOPEN				0xD3
#define RFID_CMD_CHALLENGE				0xD4
#define RFID_CMD_AUTHENTICATE			0xD5
#define RFID_CMD_SECURECOMM				0xD6
#define RFID_CMD_AUTHCOMM				0xD7

void RFID_setup(uint16_t *pFlags, uint16_t rxFlag, uint16_t txFlag);
void RFID_startRxLog();
void RFID_stopRxLog();
void RFID_startTxLog();
void RFID_stopTxLog();
void RFID_UARTSendRxData();
void RFID_UARTSendTxData();

void RFID_RxHandler();
void RFID_TxHandler(uint32_t curTime);

// RFID command handles
void handleQR(void);
void handleAck(void);
void handleQuery(void);
void handleQA(void);
void handleSelect(void);
void handleNak(void);
void handleReqRN(void);
void handleRead(void);
void handleWrite(void);
void handleKill(void);
void handleLock(void);
void handleAccess(void);
void handleBlockWrite(void);
void handleBlockErase(void);
void handleBlockPermalock(void);
void handleReadBuffer(void);
void handleFileOpen(void);
void handleChallenge(void);
void handleAuthenticate(void);
void handleSecureComm(void);
void handleAuthComm(void);

extern void decodeCmd(void);

// from wisp-base project -- globals.h
// ===================================================
//THE RFID STRUCT FOR INVENTORY STATE VARS
typedef struct {
    uint8_t     TRext;
    uint16_t    handle;
    uint16_t    slotCount;
    uint8_t     Q;

    uint8_t     mode;
    uint8_t     abortOn;                    /*  List of command responses which cause the main RFID loop to return              */
    uint8_t     abortFlag;
    uint8_t     isSelected;                 /* state of being selected via the select command. Zero if not selected             */

    uint8_t     rn8_ind;                    /* using our RN values in INFO_MEM, this points to the current one to use next      */

    uint16_t    edge_capture_prev_ccr;      /* Previous value of CCR register, used to compute delta in edge capture ISRs       */


}RFIDstruct;                                /* in MODE_USES_SEL!!                                                               */

extern RFIDstruct rfid;
extern uint8_t cmd[CMD_BUFSIZE];

// ===================================================

extern uint16_t R_dest_saved;
extern uint16_t R_bits_saved;
extern uint16_t R_bitCt_saved;
extern uint16_t R_newCt_saved;
extern uint16_t R_pivot_saved;
extern uint16_t R_scratch0_saved;
extern uint16_t R_scratch2_saved;
extern uint16_t R_wakeupBits_saved;

#endif
