#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include "pin_assign.h"
#include "error.h"
#include "rfid_protocol.h"

#include "rfid_decoder.h"

#define NS_TO_CYCLES(t) (t * CONFIG_DCOCLKDIV_FREQ / 1000000000UL)

/** @brief States of RFID protocol decoder state machine */
typedef enum {
    RX_DEC_STATE_IDLE,
    RX_DEC_STATE_PREAMBLE_DELIM,
    RX_DEC_STATE_PREAMBLE_TARI,
    RX_DEC_STATE_PREAMBLE_RT_CAL,
    RX_DEC_STATE_PREAMBLE_TR_CAL_OR_DATA_0,
    RX_DEC_STATE_DATA,
} rx_dec_state_t;

static rfid_cmd_handler_t *rfid_cmd_handler;
static rfid_rsp_handler_t *rfid_rsp_handler;

static uint16_t prev_rx_edge_timestamp;

/** @brief Current state of RFID protocol decoder state machine */
static rx_dec_state_t rx_dec_state;

/** @brief TARI: a unit of time in the RX protocol (duration of a data-0 bit) */
static uint16_t rx_tari;

/** @brief Router-to-tag calibration pulse duration */
static uint16_t rx_rt_cal;

/** @brief Pulse duration that defines the data bit value
 *  @details If pulse duration < rx_pivot then data = 0
 *                                        else data = 1
 *
 *           Set by the RFID reader in preamble: RTcal duration / 2
 */
static uint16_t rx_pivot;

/** @brief Data rate for tag replies (TODO: units: bit duration?)
 *  @details Set by reader in preamble + payload: TRcal duration / divider
 *           The divider is in payload.
 */
static uint16_t tx_rate;

/** @brief Index of the next bit of the command code to be decode */
static uint8_t rx_cmd_code_bit_idx;

/** @brief Code of the RFID command that is being decoded */
static rfid_cmd_code_t rx_cmd_code;

static inline void set_rx_decoder_state(rx_dec_state_t state)
{
    rx_dec_state = state;

#ifdef CONFIG_RFID_DECODER_STATE_PINS
    GPIO(PORT_RFID_DEC_STATE, OUT) = (GPIO(PORT_RFID_DEC_STATE, OUT) &
        ~(BIT(PORT_RFID_DEC_STATE_0) | BIT(PORT_RFID_DEC_STATE_1) | BIT(PORT_RFID_DEC_STATE_2))) |
        (state << PORT_RFID_DEC_STATE_0);
#endif
}

void rfid_decoder_init(rfid_cmd_handler_t *rfid_cmd_handler_cb,
                       rfid_rsp_handler_t *rfid_rsp_handler_cb)
{
    rfid_cmd_handler = rfid_cmd_handler_cb;
    rfid_rsp_handler = rfid_rsp_handler_cb;

    ASSERT(ASSERT_INVALID_RFID_CMD_HANDLER, rfid_cmd_handler);

    GPIO(PORT_RF, SEL) &= ~BIT(PIN_RF_TX); // tx pin in GPIO function
    GPIO(PORT_RF, DIR) &= ~BIT(PIN_RF_TX); // input direction

    GPIO(PORT_RF, SEL) |= BIT(PIN_RF_RX); // route rx pin to timer

    // Configure RX timer (it does capture on rx edges):
    // capture on falling edge, input CCIxA, sync
    TIMER(TIMER_RF_RX_DECODE, CTL) = 0;  // disable timer to configure and clear cfg
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) = CAP | CCIS_0 | SCS | CM_2;

    // Init TX timer (it does ignores interrupts for msg duration after the first edge)
    TIMER(TIMER_RF_TX_DECODE, CTL) = 0;  // disable timer to configure and clear cfg
	TIMER_CC(TIMER_RF_TX_DECODE, TMRCC_RF_TX, CCR) = RFID_TX_MSG_DURATION; // period

    // Expose the state on pins for debugging purposes
#ifdef CONFIG_RFID_DECODER_STATE_PINS
    GPIO(PORT_RFID_DEC_STATE, OUT) &=
        ~(BIT(PORT_RFID_DEC_STATE_0) | BIT(PORT_RFID_DEC_STATE_1) | BIT(PORT_RFID_DEC_STATE_2));
    GPIO(PORT_RFID_DEC_STATE, DIR) |=
        BIT(PORT_RFID_DEC_STATE_0) | BIT(PORT_RFID_DEC_STATE_1) | BIT(PORT_RFID_DEC_STATE_2);
#endif
}

void rfid_decoder_start()
{
    set_rx_decoder_state(RX_DEC_STATE_IDLE);
    prev_rx_edge_timestamp = 0;

    // Start RX timer in capture-on-falling-edge mode
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) &= ~CCIFG;
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) |= CCIE;
    TIMER(TIMER_RF_RX_DECODE, CTL) |= MC__CONTINUOUS | TACLR;

	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);			// clear Tx interrupt flag
	GPIO(PORT_RF, IE) |= BIT(PIN_RF_TX);			// enable Tx interrupt
}

void rfid_decoder_stop()
{
    TIMER(TIMER_RF_RX_DECODE, CTL) = MC__STOP;
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) &= ~CCIE;

    TIMER(TIMER_RF_TX_DECODE, CTL) = MC__STOP; // stop and disable rollover interrupt

	GPIO(PORT_RF, IE) &= ~BIT(PIN_RF_TX);			// disable interrupt
	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);			// clear interrupt flag
}


static inline void receive_cmd_bit(bool data_bit)
{
#if 0 // TODO: field decoding code would go something like this:
    cmd_desc = &rx_cmd_desc_query_rep;
    // TODO: allocate space in event buf and fill it directly
    switch (cmd_desc->fields[rx_cmd_field_idx].type) {
        case RX_CMD_FIELD_TYPE_FIXED_LEN:
            if (rx_cmd_field_len < rx_cmd_desc_query_rep.field_len[rx_cmd_field_idx]) {
                rx_cmd.fields[rx_cmd_field_idx] |= data_bit << rx_cmd_field_len++;
            } else {
                rx_cmd_field_idx++;
            }
            break;
        case RX_CMD_FIELD_TYPE_EVB:
            error(ERROR_NOT_IMPLEMENTED);
            // TODO: parse the variable length field
            break;
    }
    if (rx_cmd_field_idx < rx_cmd_desc_query_rep.num_fields) {
        rx_cmd_field_idx++;
    } else { // command decoding done
        // TODO: do we check the CRC here or on the host? probably on the host
        rfid_cmd_handler(rx_cmd); // TODO: might involve copying the data into event buf,
                            // but could be done copy-free if alloced in event buf
    }
#else // TODO: as a first cut, decode only command codes
    rfid_cmd_handler(rx_cmd_code);
#endif
}

static inline bool receive_data_bit(bool data_bit)
{
    // Command code is a prefix-code encoding, so we just shift bits
    // until a valid value matches.
    //
    // NOTE: An optimization might be to avoid the unoptimizable switch on all
    // the payload bits (after the cmd code had been decoded). This could be
    // done by setting a 'cmd code decoded' flag and coverting the raw comand
    // code into an 'even only' bitmask once the cmd code is decoded, then use
    // an 'even only' optimized switch (that translates into one jump
    // instruction).
    switch (rx_cmd_code) {
        case RFID_CMD_QUERYREP:
        case RFID_CMD_ACK:
        case RFID_CMD_QUERY:
        case RFID_CMD_QUERYADJUST:
        case RFID_CMD_SELECT:
        case RFID_CMD_NAK:
        case RFID_CMD_REQRN:
        case RFID_CMD_READ:
        case RFID_CMD_WRITE:
        case RFID_CMD_KILL:
        case RFID_CMD_LOCK:
        case RFID_CMD_ACCESS:
        case RFID_CMD_BLOCKWRITE:
        case RFID_CMD_BLOCKERASE:
        case RFID_CMD_BLOCKPERMALOCK:
        case RFID_CMD_READBUFFER:
        case RFID_CMD_FILEOPEN:
        case RFID_CMD_CHALLENGE:
        case RFID_CMD_AUTHENTICATE:
        case RFID_CMD_SECURECOMM:
        case RFID_CMD_AUTHCOMM:
            receive_cmd_bit(data_bit);
            rx_cmd_code_bit_idx = 8; // reset
            break;
        default: // cmd code not yet complete, keep reading its bits
            if (!rx_cmd_code_bit_idx) {
                // reset state
                rx_cmd_code = (rfid_cmd_code_t)0x0;
                rx_cmd_code_bit_idx = 8;
                return true; // fail: invalid cmd code
            }
            rx_cmd_code |= data_bit << --rx_cmd_code_bit_idx;
            break;
    }
    return false; // no failure = succcess
}

static inline void handle_rf_rx_edge(uint16_t rx_edge_timestamp)
{
    bool fail = false;
    bool data_bit;
    uint16_t interval;

    interval = rx_edge_timestamp - prev_rx_edge_timestamp;
    prev_rx_edge_timestamp = rx_edge_timestamp;

    switch (rx_dec_state) { // TODO: are there tricks to optimize into one jump?
        case RX_DEC_STATE_IDLE:
            // Wait for *rising* edge of end of preamble delimeter
            TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) =
                (TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) & (CM0 | CM1)) | CM_1;
            set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_DELIM);
            break;
        case RX_DEC_STATE_PREAMBLE_DELIM:
            if (NS_TO_CYCLES(RFID_PREAMBLE_DELIM_MIN) <= interval &&
                interval < NS_TO_CYCLES(RFID_PREAMBLE_DELIM_MAX)) {
                set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_TARI);
            } else {
                fail = true;
            }
            break;
        case RX_DEC_STATE_PREAMBLE_TARI:
            if (NS_TO_CYCLES(RFID_PREAMBLE_TARI_MIN) <= interval &&
                interval < NS_TO_CYCLES(RFID_PREAMBLE_TARI_MAX)) {
                rx_tari = interval;
                set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_RT_CAL);
            } else {
                fail = true;
            }
            break;
        case RX_DEC_STATE_PREAMBLE_RT_CAL:
            if (NS_TO_CYCLES(RFID_PREAMBLE_RT_CAL_MIN(rx_tari)) <= interval &&
                interval < NS_TO_CYCLES(RFID_PREAMBLE_RT_CAL_MAX(tari))) {
                rx_rt_cal = interval;
                rx_pivot = rx_rt_cal / 2;
                set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_TR_CAL_OR_DATA_0);
            } else {
                fail = true;
            }
            break;
        case RX_DEC_STATE_PREAMBLE_TR_CAL_OR_DATA_0:
            if (interval >= rx_rt_cal) { // TR cal
                if (NS_TO_CYCLES(RFID_PREAMBLE_TR_CAL_MIN(rx_rt_cal)) <= interval &&
                    interval < NS_TO_CYCLES(RFID_PREAMBLE_TR_CAL_MAX(rx_rt_cal))) {
                    // Technically, we don't need this info, but save it for now
                    tx_rate = interval; // TODO: divide by divider once we get the divider
                    (void)tx_rate; // TODO: silence unused
                    set_rx_decoder_state(RX_DEC_STATE_DATA);
                } else {
                    fail = true;
                }
                break;
            } else { // data bit 0
                // fall through to next case
            }
            // no break here: watch the fall through logic above
        case RX_DEC_STATE_DATA:
            if (NS_TO_CYCLES(RFID_DATA_MIN(rx_tari)) <= interval &&
                interval < NS_TO_CYCLES(RFID_DATA_MAX(rx_tari))) {
                data_bit = interval > rx_pivot;
                fail = receive_data_bit(data_bit);
            } else {
                fail = true;
            }
            break;
        default:
            ASSERT(ASSERT_INVALID_RFID_DECODER_STATE, false); // but (unhandled case) or corrupt memory
    }

    // TODO: record specific failure event in event buf; although, maybe not, because
    // in case parsing fails (e.g. bit error, or unimplemented cmd), then we need to
    // somehow abort parsing and wait for next command -- the only way to do that
    // is to wait for the delimiter, which would drop many times in a row into this
    // fail clause, while the remaining bits for the aborted command are received.
    if (fail) {
        // Reset: wait for *falling* edge of the delimiter
        TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) =
            (TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) & (CM0 | CM1)) | CM_2;
        set_rx_decoder_state(RX_DEC_STATE_IDLE);
    }
}

/** @brief RX capture compare timer ISR */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) TIMER0_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint16_t rx_edge_timestamp;

	switch(__even_in_range(TIMER(TIMER_RF_RX_DECODE, IV), 16)) {
        case TIMERA_INTFLAG(TIMER_RF_RX_DECODE, TMRCC_RF_RX):

            // The budget for this ISR is about 100 cycles: 6us / (1.0/20Mhz),
            // where 6us is the minimum bit duration in RFID Gen2 and 20Mhz is
            // our clock. This is at best about 50 instructions.

            ASSERT(ASSERT_RF_RX_DECODE_TIMER_CAPTURE_OVERFLOW,
                   !(TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) & COV));
            rx_edge_timestamp = TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCR);
            handle_rf_rx_edge(rx_edge_timestamp);
        default:
            error(ERROR_UNEXPECTED_INTERRUPT);
    }

    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) &= ~CCIFG;
}

void rfid_decoder_tx_pin_isr()
{
	// Disable the Tx interrupt and set a timer to enable it again.
	// This allows us to only log activity once when a single Tx event occurs,
	// since the Tx line makes several transitions when a transmission occurs.
	GPIO(PORT_RF, IE) &= ~BIT(PIN_RF_TX);			// disable interrupt

    // SMCLK, up mode with period set earlier, clear, interrupt on rollover
    TIMER(TIMER_RF_TX_DECODE, CTL) = TASSEL__SMCLK | MC__UP | TACLR | TAIE;

    // Decoding is not implemented, so report a generic response ID
    rfid_rsp_handler(RFID_RSP_GENERIC);
}

/** @brief TX "ignore" timer ISR */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) TIMER1_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    // stop the timer, we'll set it again on first edge of the next message
	TIMER(TIMER_RF_TX_DECODE, CTL) = MC__STOP;

    // Message transmission is over: re-enable the edge interrupt on the TX pin
	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);
	GPIO(PORT_RF, IE) |= BIT(PIN_RF_TX);
}

