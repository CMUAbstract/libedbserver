#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include "pin_assign.h"
#include "error.h"
#include "rfid_protocol.h"

#include "rfid_decoder.h"

#define NS_TO_CYCLES(t) (t * CONFIG_DCOCLKDIV_FREQ / 1000000000UL)

/**
 * @brief Match the bits in state variable to pins for quick encoding
 * @details Run-time shift is very expensive: in the worst case it
 *          compiles into a bunch of software emulation code, in the
 *          best case it is an many-cycle repeated shift instruction.
 */
#define RX_DEC_STATE_OFFSET PIN_RFID_RX_DEC_STATE_0

/** @brief States of RFID protocol decoder state machine
 *  @details Even values in order to use __even_in_range optimizatino
 *           which turns the switch statement into one jump.
*/
typedef enum {
    RX_DEC_STATE_IDLE                               = 0x00 << RX_DEC_STATE_OFFSET,
    RX_DEC_STATE_PREAMBLE_DELIM                     = 0x01 << RX_DEC_STATE_OFFSET,
    RX_DEC_STATE_PREAMBLE_TARI                      = 0x02 << RX_DEC_STATE_OFFSET,
    RX_DEC_STATE_PREAMBLE_RT_CAL                    = 0x03 << RX_DEC_STATE_OFFSET,
    RX_DEC_STATE_PREAMBLE_TR_CAL_OR_DATA_0          = 0x04 << RX_DEC_STATE_OFFSET,
    RX_DEC_STATE_DATA                               = 0x05 << RX_DEC_STATE_OFFSET,
} rx_dec_state_t;

/** @brief Decoder sub-state for phases of decoding command data bits */
typedef enum {
    RX_CMD_STATE_CODE,
    RX_CMD_STATE_PAYLOAD,
} rx_cmd_state_t;

static rfid_cmd_handler_t *rfid_cmd_handler;
static rfid_rsp_handler_t *rfid_rsp_handler;

static uint16_t prev_rx_edge_timestamp;

/** @brief Current state of RFID protocol decoder state machine */
static rx_dec_state_t rx_dec_state;
static rx_cmd_state_t rx_cmd_state;

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

/** @brief Index of the next bit of the command code to be decode */
static uint8_t rx_cmd_code_bit_idx;

/** @brief Code of the RFID command that is being decoded */
static rfid_cmd_code_t rx_cmd_code;

static inline void set_rx_decoder_state(rx_dec_state_t state)
{
    rx_dec_state = state;
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

#ifdef CONFIG_ENABLE_RF_TX_DECODING
    // Init TX timer (it does ignores interrupts for msg duration after the first edge)
    TIMER(TIMER_RF_TX_DECODE, CTL) = 0;  // disable timer to configure and clear cfg
	TIMER_CC(TIMER_RF_TX_DECODE, TMRCC_RF_TX, CCR) = RFID_TX_MSG_DURATION; // period
#endif

    // Expose the state on pins for debugging purposes
#ifdef CONFIG_RFID_DECODER_STATE_PINS
    GPIO(PORT_RFID_DEC_STATE, OUT) &=
        ~(BIT(PIN_RFID_RX_DEC_STATE_0) | BIT(PIN_RFID_RX_DEC_STATE_1) | BIT(PIN_RFID_RX_DEC_STATE_2));
    GPIO(PORT_RFID_DEC_STATE, DIR) |=
        BIT(PIN_RFID_RX_DEC_STATE_0) | BIT(PIN_RFID_RX_DEC_STATE_1) | BIT(PIN_RFID_RX_DEC_STATE_2);
#endif
}

void rfid_decoder_start()
{
    set_rx_decoder_state(RX_DEC_STATE_IDLE);
    prev_rx_edge_timestamp = 0;

    // Start RX timer in capture-on-falling-edge mode
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) &= ~CCIFG;
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) |= CCIE;
    TIMER(TIMER_RF_RX_DECODE, CTL) |= TASSEL__SMCLK | MC__CONTINUOUS | TACLR;


#ifdef CONFIG_ENABLE_RF_TX_DECODING
	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);			// clear Tx interrupt flag
	GPIO(PORT_RF, IE) |= BIT(PIN_RF_TX);			// enable Tx interrupt
#endif
}

void rfid_decoder_stop()
{
    TIMER(TIMER_RF_RX_DECODE, CTL) = MC__STOP;
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) &= ~CCIE;

    TIMER(TIMER_RF_TX_DECODE, CTL) = MC__STOP; // stop and disable rollover interrupt

	GPIO(PORT_RF, IE) &= ~BIT(PIN_RF_TX);			// disable interrupt
	GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);			// clear interrupt flag
}

static inline void reset_cmd_dec_state()
{
    rx_cmd_code = (rfid_cmd_code_t)0x0;
    rx_cmd_state = RX_CMD_STATE_CODE;
    rx_cmd_code_bit_idx = 0;
}

static inline void reset_rx_dec_state()
{
    reset_cmd_dec_state();
    prev_rx_edge_timestamp = 0;

    // Reset: wait for *falling* edge of the delimiter
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) =
        (TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) & ~(CM0 | CM1)) | CM_2;

    // Send an all-high pulse just for info
#ifdef CONFIG_RFID_DECODER_STATE_PINS
    GPIO(PORT_RFID_DEC_STATE, OUT) &=
        ~(BIT(PIN_RFID_RX_DEC_STATE_0) | BIT(PIN_RFID_RX_DEC_STATE_1) | BIT(PIN_RFID_RX_DEC_STATE_2));
    GPIO(PORT_RFID_DEC_STATE, OUT) |=
        BIT(PIN_RFID_RX_DEC_STATE_0) | BIT(PIN_RFID_RX_DEC_STATE_1) | BIT(PIN_RFID_RX_DEC_STATE_2);
    GPIO(PORT_RFID_DEC_STATE, OUT) &=
        ~(BIT(PIN_RFID_RX_DEC_STATE_0) | BIT(PIN_RFID_RX_DEC_STATE_1) | BIT(PIN_RFID_RX_DEC_STATE_2));
#endif

    set_rx_decoder_state(RX_DEC_STATE_IDLE);
}


static inline void receive_cmd_bit(bool data_bit)
{
// TODO: there's absolutely no way this can happen on the debugger board,
// the most we can do is collect bytes and ship them out. Even parsing
// the command is optional, but seems to be efficient, because if we
// don't parse the command code on the debuger then we wouldn't be able
// to enqueue a packet to the host until way later (next command?)
// since there's no delimiter for end-of-command. With parsing on-board,
// we are able to report the command as soon as it is ready.

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

static inline void receive_data_bit(uint8_t data_bit)
{
    // Command code is a prefix-code encoding, so we just shift bits
    // until a valid value matches.
    //
    // This is written with efficiency in mind: the nested switch statements
    // that fork by length reduce the number of comparisons; the rx cmd
    // decoding state eliminates the switch over all cmd codes on every payload
    // bit.

    switch(rx_cmd_state) {
        case RX_CMD_STATE_CODE: {

#if C_COMPILER_WERE_NICE_TO_US
            rx_cmd_code |= data_bit >> rx_cmd_code_bit_idx++;
#else
            uint8_t shifted_bit = data_bit;
            if (rx_cmd_code_bit_idx == 1) { // can't rpt 0 or 1 times
                shifted_bit >>= 1;
            } else if (rx_cmd_code_bit_idx > 1) {
                __asm__ (
                    " clrc\n "
                    " rpt %[bit_idx]\n "
                    " rrcx.b %[data_bit]\n "
                    : [data_bit] "=r" (shifted_bit)
                    : [bit_idx]  "r" (rx_cmd_code_bit_idx), "[data_bit]" (shifted_bit)
                );
            }
            rx_cmd_code |= shifted_bit;
            rx_cmd_code_bit_idx++;
#endif

            switch (rx_cmd_code_bit_idx) { // num bits decoded
                case 2:
                    switch (rx_cmd_code) {
                        case RFID_CMD_QUERYREP:
                        case RFID_CMD_ACK:
                            rx_cmd_state = RX_CMD_STATE_PAYLOAD;
                            break;
                    }
                    break;
                case 4:
                    switch (rx_cmd_code) {
                        case RFID_CMD_QUERY:
                        case RFID_CMD_QUERYADJUST:
                        case RFID_CMD_SELECT:
                            GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
                            while(1);
                            rx_cmd_state = RX_CMD_STATE_PAYLOAD;
                            break;
                    }
                    break;
                case 8:
                    switch (rx_cmd_code) {
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
                            rx_cmd_state = RX_CMD_STATE_PAYLOAD;
                            break;
                        default: // we don't support commands of >8 bits
                            reset_cmd_dec_state();
                            break;
                    }
                    break;
            }
            break;
        }
        case RX_CMD_STATE_PAYLOAD:
            receive_cmd_bit(data_bit);
    }
}

static inline void handle_rf_rx_edge(uint16_t rx_edge_timestamp)
{
    uint8_t data_bit;
    uint16_t interval;

    interval = rx_edge_timestamp - prev_rx_edge_timestamp;
    prev_rx_edge_timestamp = rx_edge_timestamp;

    switch (rx_dec_state) { // TODO: are there tricks to optimize into one jump?
        case RX_DEC_STATE_IDLE:
            // Wait for *rising* edge of end of preamble delimeter
            TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) =
                (TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) & ~(CM0 | CM1)) | CM_1;
            set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_DELIM);
            break;
        case RX_DEC_STATE_PREAMBLE_DELIM:
            if (NS_TO_CYCLES(RFID_PREAMBLE_DELIM_MIN) <= interval &&
                interval < NS_TO_CYCLES(RFID_PREAMBLE_DELIM_MAX)) {
                set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_TARI);
            } else {
                goto fail;
            }
            break;
        case RX_DEC_STATE_PREAMBLE_TARI:
            if (NS_TO_CYCLES(RFID_PREAMBLE_TARI_MIN) <= interval &&
                interval < NS_TO_CYCLES(RFID_PREAMBLE_TARI_MAX)) {
                rx_tari = interval; // needed for symbol bounds checking later
                set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_RT_CAL);
            } else {
                goto fail;
            }
            break;
        case RX_DEC_STATE_PREAMBLE_RT_CAL:
            if (NS_TO_CYCLES(RFID_PREAMBLE_RT_CAL_MIN(rx_tari)) <= interval &&
                interval < NS_TO_CYCLES(RFID_PREAMBLE_RT_CAL_MAX(rx_tari))) {
                rx_rt_cal = interval;
#if C_COMPILER_WERE_NICE_TO_US
                rx_pivot = rx_rt_cal / 2;
#else
                rx_pivot = rx_rt_cal;
                __asm__ (
                    " rrum.w    #1, %[pivot] "
                    : [pivot] "=r" (rx_pivot)
                    : "[pivot]" (rx_pivot)
                );
#endif
                set_rx_decoder_state(RX_DEC_STATE_PREAMBLE_TR_CAL_OR_DATA_0);
            } else {
                goto fail;
            }
            break;
        case RX_DEC_STATE_PREAMBLE_TR_CAL_OR_DATA_0:
            if (interval >= rx_rt_cal) { // TR cal
                if (NS_TO_CYCLES(RFID_PREAMBLE_TR_CAL_MIN(rx_rt_cal)) <= interval &&
                    interval < NS_TO_CYCLES(RFID_PREAMBLE_TR_CAL_MAX(rx_rt_cal))) {
                    // This interval communicates the tag reply rate, we ignore this info
                    reset_cmd_dec_state();
                    set_rx_decoder_state(RX_DEC_STATE_DATA);
                } else {
                    goto fail;
                }
                break;
            } else { // data bit 0
                reset_cmd_dec_state();
                set_rx_decoder_state(RX_DEC_STATE_DATA);
                // fall through to next case
            }
            // no break here: watch the fall through logic above
        case RX_DEC_STATE_DATA:
            // This is on very hot path, so skip the check
            //if (NS_TO_CYCLES(RFID_DATA_MIN(rx_tari)) <= interval &&
            //    interval < NS_TO_CYCLES(RFID_DATA_MAX(rx_tari))) {

                // shift the bit from left to right, because this means fewer
                // shifts on average because most frequent comands have short codes
                data_bit = interval > rx_pivot ? 0x80 : 0x00;
                receive_data_bit(data_bit);

            //} else {
            //    goto fail;
            //}
            break;
    }
    return;

    // TODO: record specific failure event in event buf; although, maybe not, because
    // in case parsing fails (e.g. bit error, or unimplemented cmd), then we need to
    // somehow abort parsing and wait for next command -- the only way to do that
    // is to wait for the delimiter, which would drop many times in a row into this
    // fail clause, while the remaining bits for the aborted command are received.
    // UPDATE: cannot afford to report preamble failures: we are not fast enough
fail:
    reset_rx_dec_state();
}

/** @brief RX capture compare timer ISR */
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
    uint16_t rx_edge_timestamp;

    // The budget for this ISR is the duration of the shortest RFID Gen2
    // symbol, which is a TARI and is about 6us. The latency to enter the ISR
    // was measured to be 1us. This leaves about 5us which is about 120 cycles
    // at 24.576 MHz, which is about 50 instructions. Not much at all!

#ifdef CONFIG_RFID_DECODER_STATE_PINS
    // Pull all low to signal enterance into ISR
    GPIO(PORT_RFID_DEC_STATE, OUT) &=
        ~(BIT(PIN_RFID_RX_DEC_STATE_0) | BIT(PIN_RFID_RX_DEC_STATE_1) | BIT(PIN_RFID_RX_DEC_STATE_2));
#endif

    // Clear the interrupt flag. Even if we are not out of this ISR before the
    // next edge, the interrupt would "queue up" and we would handle it back-to-back.
    // If a back-to-back ISR is quicker to enter (is it?), then we might catch up.
    // Regardless of whether we clear this interrupt flag upon entrance or not,
    // if we can't keep up, we will get the overflow event and detect that.
    TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) &= ~CCIFG;

    if (!(TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) & COV)) {
        rx_edge_timestamp = TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCR);
        handle_rf_rx_edge(rx_edge_timestamp);
    } else { // capture overflow: either spurious edge or processing took too long
        TIMER_CC(TIMER_RF_RX_DECODE, TMRCC_RF_RX, CCTL) &= ~COV;
        reset_rx_dec_state();
    }

#ifdef CONFIG_RFID_DECODER_STATE_PINS
    // Encode state onto pins: shows both ISR exit event and state
    GPIO(PORT_RFID_DEC_STATE, OUT) = (GPIO(PORT_RFID_DEC_STATE, OUT) &
        ~(BIT(PIN_RFID_RX_DEC_STATE_0) | BIT(PIN_RFID_RX_DEC_STATE_1) | BIT(PIN_RFID_RX_DEC_STATE_2))) |
        rx_dec_state;
#endif
}

void rfid_decoder_tx_pin_isr()
{
    // TODO: set GIE flag to enable nesting, so that the high-priority RX
    // interrupt can interrupt this ISR (does not happen by default),
    // however this causes any ISRs to interrupt (even lower priority ones),
    // which requires some thought.

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
#pragma vector=TIMER1_A1_VECTOR
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

