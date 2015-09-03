#ifndef RFID_PROTOCOL_H
#define RFID_PROTOCOL_H

#include "config.h"

// Constant values in this file come from:
// Gen2 Protocol Standard, Section 6.3.1.2 Interrogator-To-Tag (R=>T) communications

/**
 * @defgroup RFID_SYMBOL_TIMING
 * @brief Symbol lengths for RFID preamble and data bits
 * @details NOTE: suffixes 'ull' are important: if not specified,
 *          the msp430-gcc gives a warning about overflow during constant
 *          folding, but if specified as 'ul' then there is still overflow (we
 *          multiply by the frequency) but no warning! We overflow 32-bit
 *          because we have to multiply by freq in Hz. Not that it matters at
 *          all (relevant to compile-time only), but if we could use fractions
 *          (us) then we could fit into 32-bit, but we can't because msp430-gcc
 *          does not fold non-integer constant expressions.
 * @{
 */
#define RFID_PREAMBLE_DELIM_MIN (12500ull - 625ull) // ns
#define RFID_PREAMBLE_DELIM_MAX (12500ull + 625ull) // ns

#define RFID_PREAMBLE_TARI_MIN  6250ull // ns
#define RFID_PREAMBLE_TARI_MAX 25000ull // ns

#define RFID_PREAMBLE_RT_CAL_MIN_INTERNAL(tari) (25ull * tari / 10ull)
#define RFID_PREAMBLE_RT_CAL_MAX_INTERNAL(tari) (3ull * tari)

#define RFID_PREAMBLE_TR_CAL_MIN_INTERNAL(rt_cal) (11ull * rt_cal / 10ull)
#define RFID_PREAMBLE_TR_CAL_MAX_INTERNAL(rt_cal) (3ull * rt_cal)

#ifdef CONFIG_RFID_DECODER_RUNTIME_BOUNDS

#define RFID_PREAMBLE_RT_CAL_MIN(tari) RFID_PREAMBLE_RT_CAL_MIN_INTERNAL(tari) 
#define RFID_PREAMBLE_RT_CAL_MAX(tari) RFID_PREAMBLE_RT_CAL_MAX_INTERNAL(tari)

#define RFID_PREAMBLE_TR_CAL_MIN(rt_cal) RFID_PREAMBLE_TR_CAL_MIN_INTERNAL(rt_cal)
#define RFID_PREAMBLE_TR_CAL_MAX(rt_cal) RFID_PREAMBLE_TR_CAL_MAX_INTERNAL(rt_cal)

#define RFID_DATA_MIN(tari) (tari)
#define RFID_DATA_MAX(tari) (2ull * tari)

#else // compile-time only calculations

#define RFID_CONST_TARI 5750ull // measured value (real value available only at runtime)

#define RFID_PREAMBLE_RT_CAL_MIN(tari) RFID_PREAMBLE_RT_CAL_MIN_INTERNAL(RFID_CONST_TARI) 
#define RFID_PREAMBLE_RT_CAL_MAX(tari) RFID_PREAMBLE_RT_CAL_MAX_INTERNAL(RFID_CONST_TARI)

#define RFID_CONST_RTCAL RFID_PREAMBLE_RT_CAL_MIN(RFID_CONST_TARI) // pick a value: min bound

#define RFID_PREAMBLE_TR_CAL_MIN(rt_cal) RFID_PREAMBLE_TR_CAL_MIN_INTERNAL(RFID_CONST_RTCAL)
#define RFID_PREAMBLE_TR_CAL_MAX(rt_cal) RFID_PREAMBLE_TR_CAL_MAX_INTERNAL(RFID_CONST_RTCAL)

#define RFID_DATA_MIN(tari) (RFID_CONST_TARI)
#define RFID_DATA_MAX(tari) (2ull * RFID_CONST_TARI)

#endif // CONFIG_RFID_DECODER_RUNTIME_BOUNDS

/* @} End RFID_SYMBOL_TIMING */

typedef enum {
    RFID_CMD_QUERYREP				= 0x00,
    RFID_CMD_ACK					= 0x40,
    RFID_CMD_QUERY					= 0x80,
    RFID_CMD_QUERYADJUST			= 0x90,
    RFID_CMD_SELECT					= 0xA0,
    RFID_CMD_NAK					= 0xC0,
    RFID_CMD_REQRN					= 0xC1,
    RFID_CMD_READ					= 0xC2,
    RFID_CMD_WRITE					= 0xC3,
    RFID_CMD_KILL					= 0xC4,
    RFID_CMD_LOCK					= 0xC5,
    RFID_CMD_ACCESS					= 0xC6,
    RFID_CMD_BLOCKWRITE				= 0xC7,
    RFID_CMD_BLOCKERASE				= 0xC8,
    RFID_CMD_BLOCKPERMALOCK			= 0xC9,
    RFID_CMD_READBUFFER				= 0xD2,
    RFID_CMD_FILEOPEN				= 0xD3,
    RFID_CMD_CHALLENGE				= 0xD4,
    RFID_CMD_AUTHENTICATE			= 0xD5,
    RFID_CMD_SECURECOMM				= 0xD6,
    RFID_CMD_AUTHCOMM				= 0xD7,
} rfid_cmd_code_t;

/** @brief Types of replies the tag sends in response to commands from the reader
 *
 *  @details NOTE: TX decoding is not implemented. A generic wildcard code is
 *           reported whenever activity is detected on the TX pin.
 */
typedef enum {
    RFID_RSP_GENERIC                = 0x00,
} rfid_rsp_code_t;

/** @brief Transmission time of an RFID reply from tag to the reader
 *  @details Units: SMCLK cycles
 *           Measured on scope to be about 40us (Simple ACK demo), but
 *           am not certain that this applies to all possible reply types.
 *
 *           This is a shortcut in place while parsing of the TX bit stream
 *           is not implemented. We simply report a reply event whenever
 *           an activity is detected and then ignore the renaming edges for
 *           this long (since they are part of the same message).
 */
#define RFID_TX_MSG_DURATION        1200 // cycles @ 24 Mhz = 50 us

#if 0 // these come from ASM implementation from (wisp-base)
      // TODO: remove once C impl is working
// from wisp-base project -- globals.h
// ====================================================
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
#endif

#endif // RFID_PROTOCOL
