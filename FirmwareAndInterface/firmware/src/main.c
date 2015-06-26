/**
 * @file
 * @author  Graham Harvey
 * @date    13 May 2015
 * @brief   Main function for the WISP monitor.
 * @details The MSP430 on the WISP monitor has UART interrupts enabled to interface
 *          to a computer through USB.  The main loop checks flags that are set in
 *          the interrupt service routines.  This allows execution to continue
 *          without blocking to wait for peripherals.  The 12-bit ADC in use has
 *          several channels that allow four different voltages to be sampled on
 *          the WISP. These are named Vcap, Vboost, Vreg, and Vrect.  The monitor
 *          can get a single sample or log repeated samples of any of those.  It
 *          can also inject power to keep one of those voltages at a level defined
 *          by the user.  However, current is always injected on Vcap.
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#include "pin_assign.h"
#include "ucs.h"
#include "adc12.h"
#include "uart.h"
#include "i2c.h"
#include "pwm.h"
#include "timeLog.h"
#include "timer1.h"
#include "rfid.h"
#include "marker.h"
#include "minmax.h"
#include "main.h"
#include "config.h"
#include "error.h"

/**
 * @defgroup    MAIN_FLAG_DEFINES   Main loop flags
 * @brief       Flags to set in a bit mask to check in the main loop
 * @{
 */
#define FLAG_ADC12_COMPLETE         0x0001 //!< ADC12 has completed conversion
#define FLAG_UART_USB_RX            0x0002 //!< Bytes received on the USB UART
#define FLAG_UART_USB_TX            0x0004 //!< Bytes transmitted on the USB UART
#define FLAG_UART_WISP_RX           0x0008 //!< Bytes received on the WISP UART
#define FLAG_UART_WISP_TX           0x0010 //!< Bytes transmitted on the WISP UART
#define FLAG_LOGGING                0x0020 //!< Logging ADC conversion results to USB
#define FLAG_RF_DATA				0x0040 //!< RF Rx activity ready to be logged
/** @} End MAIN_FLAG_DEFINES */

/**
 * @defgroup    LOG_DEFINES     Logging flags
 * @brief       Flags to set in a bit mask to check which ADC readings to log
 * @{
 */
#define LOG_VCAP                    0x01 //!< Logging Vcap
#define LOG_VBOOST                  0x02 //!< Logging Vboost
#define LOG_VREG                    0x04 //!< Logging Vreg
#define LOG_VRECT                   0x08 //!< Logging Vrect
#define LOG_VINJ					0x10 //!< Logging Vinj
/** @} End LOG_DEFINES */

#define WISP_CMD_MAX_LEN 16
#define STREAM_REPLY_MAX_LEN (1 /* num chans */ + ADC12_MAX_CHANNELS * sizeof(uint16_t))

#define NUM_CODEPOINTS (1 << NUM_CODEPOINT_PINS)

/**
 * @brief   Assigns a permanent index to each ADC channels
 *
 * @details This maps a application's name for an ADC channel to an index
 *          understandable to the ADC12 driver in adc12.c.
 */
typedef enum {
    ADC_CHAN_INDEX_VCAP = 0,
    ADC_CHAN_INDEX_VBOOST,
    ADC_CHAN_INDEX_VREG,
    ADC_CHAN_INDEX_VRECT,
    ADC_CHAN_INDEX_VINJ,
} adc_chan_index_t;

/**
 * Debugger state machine states
 */
typedef enum {
    STATE_IDLE = 0,
    STATE_ENTERING,
    STATE_DEBUG,
    STATE_EXITING,
} state_t;

static uint16_t flags = 0; // bit mask containing bit flags to check in the main loop
static uint8_t log_flags = 0; // bit mask containing active log values to send via USB

static state_t state = STATE_IDLE;

static uint16_t adc12Target; // target ADC reading
static uint16_t saved_vcap; // energy level before entering active debug mode

static uartPkt_t wispRxPkt = { .processed = 1 };

static uint8_t wisp_cmd_buf[WISP_CMD_MAX_LEN];
static uint8_t stream_buf[STREAM_REPLY_MAX_LEN];

static bool bkpt_group_enable[NUM_CODEPOINTS] = {0}; // group index -> is group enabled

static adc12_t adc12 = {
    .config = {
        .channel_masks = { // map permanent software indexes to hardware ADC channels
            ADC_CHAN_VCAP,
            ADC_CHAN_VBOOST,
            ADC_CHAN_VREG,
            ADC_CHAN_VRECT,
            ADC_CHAN_VINJ,
        },
        .num_channels = 0, // maintained at runtime
    },

    .pFlags = &flags,
    .flag_adc12Complete = FLAG_ADC12_COMPLETE,
};

static void set_state(state_t new_state)
{
    state = new_state;

#ifdef CONFIG_STATE_PINS
    // Encode state onto two indicator pins
    GPIO(PORT_STATE, OUT) &= ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1));
    GPIO(PORT_STATE, OUT) |= (new_state & 0x1 ? BIT(PIN_STATE_0) : 0) |
                             (new_state & 0x2 ? BIT(PIN_STATE_1) : 0);
#endif
}

/**
 * @brief       Pulse a designated pin for triggering an oscilloscope
 */
static void trigger_scope()
{
    GPIO(PORT_TRIGGER, OUT) |= BIT(PIN_TRIGGER);
    GPIO(PORT_TRIGGER, DIR) |= BIT(PIN_TRIGGER);
    GPIO(PORT_TRIGGER, OUT) &= ~BIT(PIN_TRIGGER);
}


/**
 * @brief	Send an interrupt to the target device
 */
static void signal_target()
{
    // pulse the signal line

    // target signal line starts in high imedence state
    GPIO(PORT_SIG, OUT) |= BIT(PIN_SIG);		// output high
    GPIO(PORT_SIG, DIR) |= BIT(PIN_SIG);		// output enable
    GPIO(PORT_SIG, OUT) &= ~BIT(PIN_SIG);    // output low
    GPIO(PORT_SIG, DIR) &= ~BIT(PIN_SIG);    // back to high impedence state
    GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG); // clear interrupt flag (might have been set by the above)
}

/**
 * @brief	Enable interrupt line between the debugger and the target device
 */
static void unmask_target_signal()
{
    GPIO(PORT_SIG, IE) |= BIT(PIN_SIG);   // enable interrupt
    GPIO(PORT_SIG, IES) &= ~BIT(PIN_SIG); // rising edge
}

/**
 * @brief	Disable interrupt line between the debugger and the target device
 */
static void mask_target_signal()
{
    GPIO(PORT_SIG, IE) &= ~BIT(PIN_SIG); // disable interrupt
}

static void continuous_power_on()
{
    // The output level was configured high on boot up
    GPIO(PORT_CONT_POWER, DIR) |= BIT(PIN_CONT_POWER);
}

static void continuous_power_off()
{
    GPIO(PORT_CONT_POWER, DIR) &= ~BIT(PIN_CONT_POWER); // to high-z state
}

static void send_vcap(uint16_t vcap)
{
    UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VOLTAGE,
                 (uint8_t *)(&vcap), sizeof(uint16_t), UART_TX_FORCE);
}

static void send_return_code(uint8_t code)
{
    UART_sendMsg(UART_INTERFACE_USB, USB_RSP_RETURN_CODE,
                 (uint8_t *)(&code), sizeof(uint8_t), UART_TX_FORCE);
}

static void send_interrupted(uint16_t saved_vcap)
{
    UART_sendMsg(UART_INTERFACE_USB, USB_RSP_INTERRUPTED,
                 (uint8_t *)(&saved_vcap), sizeof(uint16_t), UART_TX_FORCE);
}

/**
 * @brief	Handle an interrupt from the target device
 */
static void handle_target_signal()
{
    uint16_t restored_vcap;

    switch (state) {
        case STATE_ENTERING:
            // WISP has entered debug main loop
            set_state(STATE_DEBUG);
            GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
            continuous_power_on();
            UART_setup(UART_INTERFACE_WISP, &flags, FLAG_UART_WISP_RX, FLAG_UART_WISP_TX);
            I2C_setup();
            send_interrupted(saved_vcap); // do it here: reply marks completion
            break;
        case STATE_EXITING:
            // WISP has shutdown UART and is asleep waiting for int to resume
            UART_teardown(UART_INTERFACE_WISP);
            I2C_teardown();
            continuous_power_off();
            restored_vcap = discharge_block(saved_vcap); // restore energy level
            signal_target();
            send_vcap(restored_vcap);
            GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);
            set_state(STATE_IDLE);
            break;
        default:
            // received an unexpected signal
            break;
    }
}

static void enter_debug_mode()
{
    set_state(STATE_ENTERING);

    saved_vcap = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP); // read Vcap and set as the target for exit
    signal_target();
    unmask_target_signal();
}

static void exit_debug_mode()
{
    set_state(STATE_EXITING);
    unmask_target_signal();
    UART_sendMsg(UART_INTERFACE_WISP, WISP_CMD_EXIT_ACTIVE_DEBUG, 0, 0, UART_TX_FORCE);
}

static void reset_state()
{
    continuous_power_off();
    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);
    set_state(STATE_IDLE);
    mask_target_signal();
}

/**
 * @brief   Set up all pins.  Default to GPIO output low for unused pins.
 */
static void pin_setup()
{
    // Set unconnected pins to output low (note: OUT value is undefined on reset)
    P1DIR |= BIT7;
    P1OUT &= ~(BIT7);
    P2DIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P2OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3DIR |= BIT0 | BIT1 | BIT2 | BIT5 | BIT6 | BIT7;
    P3OUT &= ~(BIT0 | BIT1 | BIT2 | BIT5 | BIT6 | BIT7);
    P4DIR |= BIT0 | BIT3 | BIT7;
    P4OUT &= ~(BIT0 | BIT3 | BIT7);
    P5DIR |= BIT0 | BIT1 | BIT6;
    P5OUT &= ~(BIT0 | BIT1 | BIT6);
    P6DIR |= BIT0 | BIT6 | BIT7;
    P6OUT &= ~(BIT0 | BIT6 | BIT7);
    // PJDIR |= <none>

    // Uncomment this if R3 is not populated since in that case pin is unconnected
    // GPIO(PORT_CONT_POWER, DIR) |= BIT(PIN_CONT_POWER);
    // GPIO(PORT_CONT_POWER, OUT) &= ~BIT(PIN_CONT_POWER);

    GPIO(PORT_LED, OUT) &= ~(BIT(PIN_LED_GREEN) | BIT(PIN_LED_RED));
    GPIO(PORT_LED, DIR) |= BIT(PIN_LED_GREEN) | BIT(PIN_LED_RED);
    GPIO(PORT_TRIGGER, OUT) &= ~BIT(PIN_TRIGGER);
    GPIO(PORT_TRIGGER, DIR) |= BIT(PIN_TRIGGER);

#ifdef CONFIG_STATE_PINS
    GPIO(PORT_STATE, OUT) &= ~(BIT(PIN_STATE_0) | BIT(PIN_STATE_1));
    GPIO(PORT_STATE, DIR) |= BIT(PIN_STATE_0) | BIT(PIN_STATE_1);
#endif

#ifdef CONFIG_ENABLE_CODEPOINTS
    // Monitor code points TODO: enable only when a breakpoint is set
    GPIO(PORT_CODEPOINT, IE) |= BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1);   // enable interrupt
    GPIO(PORT_CODEPOINT, IES) &= ~BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1); // rising edge
#endif

    // Configure the output level for continous power pin ahead of time
    GPIO(PORT_CONT_POWER, OUT) |= BIT(PIN_CONT_POWER);

    // Voltage sense pins as ADC channels
    GPIO(PORT_VSENSE, SEL) |=
        BIT(PIN_VCAP) | BIT(PIN_VBOOST) | BIT(PIN_VREG) | BIT(PIN_VRECT) | BIT(PIN_VINJ);

#ifdef CONFIG_ROUTE_ACLK_TO_PIN
    P1SEL |= BIT0;
    P1DIR |= BIT0;
#endif

    // For measuring debugger energy interference only: configure interrupt line on boot
    // unmask_target_signal();
}

int main(void)
{
    uartPkt_t usbRxPkt = { .processed = 1 };
    uint32_t count = 0;

    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

    pin_setup();

    GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);

    UCS_setup(); // set up unified clock system
#ifdef CONFIG_CLOCK_TEST_MODE
    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);
    BLINK_LOOP(PIN_LED_GREEN, 1000000); // to check clock configuration
#endif

    PWM_setup(1024-1, 512); // dummy default values
    UART_setup(UART_INTERFACE_USB, &flags, FLAG_UART_USB_RX, FLAG_UART_USB_TX); // USCI_A0 UART

    RFID_setup(&flags, FLAG_RF_DATA, FLAG_RF_DATA); // use the same flag for Rx and Tx so
    												// we only have to check one flag
    ADC12_init(&adc12);

    __enable_interrupt();                   // enable all interrupts

    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_RED);

    while(1) {
        if(flags & FLAG_ADC12_COMPLETE) {
            // ADC12 has completed conversion on all active channels
            flags &= ~FLAG_ADC12_COMPLETE;

            if(flags & FLAG_LOGGING) {
                // send ADC conversion time and conversion results
                UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
                        (uint8_t *)(&(adc12.timeComplete)), sizeof(uint32_t),
                        UART_TX_DROP);

                /* TODO: inefficient: try to get rid of the this copy */
                stream_buf[0] = adc12.config.num_channels;
                memcpy(stream_buf + sizeof(uint8_t), adc12.results,
                       adc12.config.num_channels * sizeof(uint16_t));

                UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VOLTAGES, stream_buf,
                    sizeof(uint8_t) + adc12.config.num_channels * sizeof(uint16_t),
                    UART_TX_DROP);

                ADC12_start();
            }
        }

        if(flags & FLAG_UART_USB_RX) {
            // we've received a byte from USB
            if(UART_buildRxPkt(UART_INTERFACE_USB, &usbRxPkt) == 0) {
                // packet is complete
                executeUSBCmd(&usbRxPkt);
            }

            // check if we're done for now
            UART_DISABLE_USB_RX; // disable interrupt so new bytes don't come in
            if(UART_RxBufEmpty(UART_INTERFACE_USB)) {
                flags &= ~FLAG_UART_USB_RX; // clear USB Rx flag
            }
            UART_ENABLE_USB_RX; // enable interrupt
        }

/*
        if(flags & FLAG_UART_USB_TX) {
            // USB UART Tx byte
            flags &= ~FLAG_UART_USB_TX;
        }
*/
/*
        if(flags & FLAG_UART_WISP_RX) {
            // we've received a byte over UART from the WISP
            if(UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) == 0) {
            	// packet is complete
            	//doStuff();
            	wispRxPkt.processed = 1;
            }

            // check if we're done for now
            UART_DISABLE_WISP_RX; // disable interrupt so new bytes don't come in
            if(UART_RxBufEmpty(UART_INTERFACE_WISP)) {
            	flags &= ~FLAG_UART_WISP_RX; // clear WISP Rx flag
            }
            UART_ENABLE_WISP_RX; // enable interrupt
        }
*/
/*
        if(flags & FLAG_UART_WISP_TX) {
            // WISP UART Tx byte
            flags &= ~FLAG_UART_WISP_TX;
        }
*/

        if(flags & FLAG_RF_DATA) {
        	flags &= ~FLAG_RF_DATA;
        	RFID_UARTSendRxData(); // send any data that we may have collected
        	RFID_UARTSendTxData();
        }

        // This LED toggle is unnecessary, and probably a huge waste of processing time.
        // The LED blinking will slow down when the monitor is performing more tasks.
        if (++count == 0xffff) {
            GPIO(PORT_LED, OUT) ^= BIT(PIN_LED_GREEN);
            count = 0;
        }

    }
}

/**
 * @brief       Execute a command received from the computer through the USB port
 * @param       pkt     Packet structure that contains the received message info
 */
static void executeUSBCmd(uartPkt_t *pkt)
{
    uint16_t adc12Result;
    uint16_t target_vcap, actual_vcap;
    uint32_t address;
    uint8_t cmd_len;
    uint8_t i;

    trigger_scope();

    switch(pkt->descriptor)
    {
    case USB_CMD_SENSE:
        {
            adc_chan_index_t chan_idx = (adc_chan_index_t)pkt->data[0];
            adc12Result = ADC12_read(&adc12, chan_idx);
            UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VOLTAGE,
                            (uint8_t *)(&adc12Result), sizeof(uint16_t),
                            UART_TX_FORCE);
            break;
        }
    case USB_CMD_SET_VCAP:
        adc12Target = *((uint16_t *)(pkt->data));
        setWispVoltage_block(ADC_CHAN_INDEX_VCAP, adc12Target);
        break;

    case USB_CMD_SET_VBOOST:
        adc12Target = *((uint16_t *)(pkt->data));
        setWispVoltage_block(ADC_CHAN_INDEX_VBOOST, adc12Target);
        break;

    case USB_CMD_ENTER_ACTIVE_DEBUG:
    	// todo: turn off all logging?
        enter_debug_mode();
        break;

    case USB_CMD_EXIT_ACTIVE_DEBUG:
        exit_debug_mode();
        break;

    case USB_CMD_GET_WISP_PC:
    	UART_sendMsg(UART_INTERFACE_WISP, WISP_CMD_GET_PC, 0, 0, UART_TX_FORCE); // send request
    	while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
    			(wispRxPkt.descriptor != WISP_RSP_PC)); // wait for response
    	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_WISP_PC, &(wispRxPkt.data[0]),
    					wispRxPkt.length, UART_TX_FORCE); // send PC over USB
    	wispRxPkt.processed = 1;
    	break;

    case USB_CMD_EXAMINE_MEMORY:
    	// not implemented
        break;

    case USB_CMD_STREAM_BEGIN: {
        uint8_t num_chans = pkt->data[0];
        adc_chan_index_t *chan_indexes = (adc_chan_index_t *)&pkt->data[1];
        flags |= FLAG_LOGGING; // for main loop
        TimeLog_request(1); // request timer overflow notifications
        for (i = 0; i < num_chans; ++i)
            ADC12_addChannel(&adc12, chan_indexes[i]);
        ADC12_restart(&adc12);
        break;
    }

    case USB_CMD_STREAM_END: {
        uint8_t num_chans = pkt->data[0];
        adc_chan_index_t *chan_indexes = (adc_chan_index_t *)&pkt->data[1];
		flags &= ~FLAG_LOGGING; // for main loop
        TimeLog_request(0);
        for (i = 0; i < num_chans; ++i)
            ADC12_removeChannel(&adc12, chan_indexes[i]);
        ADC12_restart(&adc12);
        break;
    }

    case USB_CMD_LOG_RF_RX_BEGIN:
    	TimeLog_request(1); // request timer overflow notifications
    	RFID_startRxLog();
    	break;

    case USB_CMD_LOG_RF_RX_END:
    	TimeLog_request(0);
    	RFID_stopRxLog();
    	break;

    case USB_CMD_LOG_RF_TX_BEGIN:
    	TimeLog_request(1); // request timer overflow notifications
    	RFID_startTxLog();
    	break;

    case USB_CMD_LOG_RF_TX_END:
    	TimeLog_request(0);
    	RFID_stopTxLog();
    	break;

    case USB_CMD_SEND_RF_TX_DATA:
		// not implemented
		break;

    case USB_CMD_ENABLE_PORT_INT_TAG_PWR:
    	// not implemented
    	break;

    case USB_CMD_DISABLE_PORT_INT_TAG_PWR:
    	// not implemented
    	break;

    case USB_CMD_PWM_ON:
    	PWM_start();
    	break;

    case USB_CMD_CHARGE:
        target_vcap = *((uint16_t *)(&pkt->data[0]));
        actual_vcap = charge_block(target_vcap);
        send_vcap(actual_vcap);
        break;

    case USB_CMD_DISCHARGE:
        target_vcap = *((uint16_t *)(pkt->data));
        actual_vcap = discharge_block(target_vcap);
        send_vcap(actual_vcap);
        break;

    case USB_CMD_RESET_STATE:
        reset_state();
        break;

    case USB_CMD_RELEASE_POWER:
    case USB_CMD_PWM_OFF:
    case USB_CMD_PWM_LOW:
    	PWM_stop();
    	break;

    case USB_CMD_SET_PWM_FREQUENCY:
    	TB0CCR0 = (*((uint16_t *)(pkt->data))) - 1;
    	break;

    case USB_CMD_SET_PWM_DUTY_CYCLE:
    	TB0CCR1 = *((uint16_t *)(pkt->data));
    	break;

    case USB_CMD_PWM_HIGH:
    	PWM_stop();
        GPIO(PORT_CHARGE, OUT) |= BIT(PIN_CHARGE); // output high
    	break;

    // USB_CMD_PWM_LOW and USB_CMD_PWM_OFF do the same thing

    case USB_CMD_MONITOR_MARKER_BEGIN:
        marker_monitor_begin();
        break;

    case USB_CMD_MONITOR_MARKER_END:
        marker_monitor_end();
        break;

    case USB_CMD_BREAK_AT_VCAP_LEVEL:
        target_vcap = *((uint16_t *)(&pkt->data[0]));
        break_at_vcap_level(target_vcap);
        break;

    case USB_CMD_READ_MEM:
        address = *((uint32_t *)(&pkt->data[0]));

        cmd_len = 0;
        wisp_cmd_buf[cmd_len++] = (address >> 0) & 0xff;
        wisp_cmd_buf[cmd_len++] = (address >> 8) & 0xff;
        wisp_cmd_buf[cmd_len++] = (address >> 16) & 0xff;
        wisp_cmd_buf[cmd_len++] = (address >> 24) & 0xff;

        UART_sendMsg(UART_INTERFACE_WISP, WISP_CMD_READ_MEM,
                     wisp_cmd_buf, cmd_len, UART_TX_FORCE); // send request
        while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                (wispRxPkt.descriptor != WISP_RSP_MEMORY)); // wait for response
        UART_sendMsg(UART_INTERFACE_USB, USB_RSP_WISP_MEMORY, &(wispRxPkt.data[0]),
                     wispRxPkt.length, UART_TX_FORCE); // send PC over USB
        wispRxPkt.processed = 1;
        break;

    case USB_CMD_WRITE_MEM:
    {
        address = *((uint32_t *)(&pkt->data[0]));
        uint8_t value = pkt->data[sizeof(uint32_t)];

        cmd_len = 0;
        wisp_cmd_buf[cmd_len++] = (address >> 0) & 0xff;
        wisp_cmd_buf[cmd_len++] = (address >> 8) & 0xff;
        wisp_cmd_buf[cmd_len++] = (address >> 16) & 0xff;
        wisp_cmd_buf[cmd_len++] = (address >> 24) & 0xff;
        wisp_cmd_buf[cmd_len++] = value;

        UART_sendMsg(UART_INTERFACE_WISP, WISP_CMD_WRITE_MEM,
                     wisp_cmd_buf, cmd_len, UART_TX_FORCE); // send request
        while((UART_buildRxPkt(UART_INTERFACE_WISP, &wispRxPkt) != 0) ||
                (wispRxPkt.descriptor != WISP_RSP_MEMORY)); // wait for response
        UART_sendMsg(UART_INTERFACE_USB, USB_RSP_WISP_MEMORY, &(wispRxPkt.data[0]),
                     wispRxPkt.length, UART_TX_FORCE); // send PC over USB
        wispRxPkt.processed = 1;
        break;
    }

    case USB_CMD_CONT_POWER:
    {
        bool power_on = (bool)pkt->data[0];
        if (power_on)
            continuous_power_on();
        else
            continuous_power_off();
        send_return_code(RETURN_CODE_SUCCESS);
        break;
    }

    case USB_CMD_BREAKPOINT:
    {
        uint8_t index = (uint8_t)pkt->data[0];
        bool enable = (bool)pkt->data[1];
        uint8_t rc = RETURN_CODE_SUCCESS;
        if (index > 0 && index < NUM_CODEPOINTS)
            bkpt_group_enable[index] = enable;
        else
            rc = RETURN_CODE_INVALID_ARGS;
        send_return_code(rc);
        break;
    }

    default:
        break;
    }

    pkt->processed = 1;
}

static uint16_t charge_block(uint16_t target)
{
    uint16_t cur_voltage;

    // Output Vcc level to Vcap (through R1) */

    // Configure the pin
    GPIO(PORT_CHARGE, DS) |= BIT(PIN_CHARGE); // full drive strength
    GPIO(PORT_CHARGE, SEL) &= ~BIT(PIN_CHARGE); // I/O function
    GPIO(PORT_CHARGE, DIR) |= BIT(PIN_CHARGE); // I/O function output

    GPIO(PORT_CHARGE, OUT) |= BIT(PIN_CHARGE); // turn on the power supply

    // Wait for the cap to charge to that voltage

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);
    } while (cur_voltage < target);

    GPIO(PORT_CHARGE, OUT) &= ~BIT(PIN_CHARGE); // cut the power supply

    return cur_voltage;
}

static uint16_t discharge_block(uint16_t target)
{
    uint16_t cur_voltage;

    GPIO(PORT_DISCHARGE, DIR) |= BIT(PIN_DISCHARGE); // open the discharge "valve"

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);
    } while (cur_voltage > target);

    GPIO(PORT_DISCHARGE, DIR) &= ~BIT(PIN_DISCHARGE); // close the discharge "valve"

    return cur_voltage;
}

static void setWispVoltage_block(uint8_t adc_chan_index, uint16_t target)
{
	uint16_t result;
	int8_t compare;
	uint8_t threshold = 1;
	ADC12_addChannel(&adc12, adc_chan_index);
	ADC12_restart(&adc12);

	// Here, we want to choose a starting PWM duty cycle close to, but not above,
	// what the correct one will be.  We want it to be close because we will test
	// each duty cycle, increasing one cycle at a time, until we reach the right
	// one.  In my experiment with the oscilloscope, the WISP cap took about 60ms
	// to charge, so I'll give it 80ms for the first charge and 10ms for each
	// charge following.

	// We know the ADC target, so let's give the PWM duty cycle our best guess.
	// target (adc) * PWM_period (SMCLK cycles) / 2^12 (adc) = approx. PWM_duty_cycle (SMCLK cycles)
	// Subtract 40 from this to start.
	uint32_t duty_cycle = (uint32_t) target * (uint32_t) TB0CCR0 / 4096;
	TB0CCR1 = MAX((uint16_t) duty_cycle - 40, 0);
	PWM_start();

	// 70ms wait time
	uint8_t i;
	for(i = 0; i < 40; i++) {
		__delay_cycles(21922); // delay for 1ms
	}

	do {
		// 10ms wait time
		for(i = 0; i < 40; i++) {
			__delay_cycles(21922); // delay for 1ms
		}

		result = ADC12_read(&adc12, adc_chan_index);
		compare = uint16Compare(result, target, threshold);
		if(compare < 0) {
			// result < target
			PWM_INCREASE_DUTY_CYCLE;
		} else if(compare > 0) {
			// result > target
			PWM_DECREASE_DUTY_CYCLE;
		}
	} while(compare != 0);

	// We've found the correct PWM duty cycle for the target voltage.
	// Leave PWM on, but remove this channel from the ADC configuration
	// if necessary.
	if((adc_chan_index == ADC_CHAN_INDEX_VCAP && !(log_flags & LOG_VCAP)) ||
				(adc_chan_index == ADC_CHAN_INDEX_VBOOST && !(log_flags & LOG_VBOOST))) {
		// We don't need this ADC channel anymore, since we're done here and
		// we're not logging this voltage right now.
		ADC12_removeChannel(&adc12, adc_chan_index);
		ADC12_restart(&adc12);
	}
}

static void break_at_vcap_level(uint16_t level)
{
    uint16_t cur_voltage;

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = ADC12_read(&adc12, ADC_CHAN_INDEX_VCAP);
    } while (cur_voltage > level);

    enter_debug_mode();
}

static int8_t uint16Compare(uint16_t n1, uint16_t n2, uint16_t threshold) {
	if(n1 < n2 - threshold && n2 >= threshold) {
		return -1;
	} else if(n1 > n2 + threshold && n2 + threshold <= 0xFFFF) {
		return 1;
	}
	return 0;
}

// Port 1 ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{
    uint8_t pin_state = GPIO(PORT_CODEPOINT, IN); // snapshot

	switch(__even_in_range(P1IV, 16))
	{
	case INTFLAG(PORT_RF, PIN_RF_TX):
		RFID_TxHandler(TIMELOG_CURRENT_TIME);
		GPIO(PORT_RF, IFG) &= ~BIT(PIN_RF_TX);
		break;
	case INTFLAG(PORT_RF, PIN_RF_RX):
		RFID_RxHandler();
		break;
	case INTFLAG(PORT_SIG, PIN_SIG):
		mask_target_signal();
		handle_target_signal();
		GPIO(PORT_SIG, IFG) &= ~BIT(PIN_SIG);
		break;

#ifdef CONFIG_ENABLE_CODEPOINTS
    case INTFLAG(PORT_CODEPOINT, PIN_CODEPOINT_0):
    case INTFLAG(PORT_CODEPOINT, PIN_CODEPOINT_1):
    {
        /* Workaround the hardware routing that routes AUX1,AUX2 to pins out of order */
        pin_state = (pin_state & BIT(PIN_CODEPOINT_0) ? BIT(PIN_CODEPOINT_1) : 0) |
                    (pin_state & BIT(PIN_CODEPOINT_1) ? BIT(PIN_CODEPOINT_0) : 0);
        uint8_t codept_idx = (pin_state & (BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1)))
            >> PIN_CODEPOINT_0;
        if (bkpt_group_enable[codept_idx]) {
            if (state == STATE_DEBUG)
                error(ERROR_UNEXPECTED_CODEPOINT);

            enter_debug_mode();
        }
        break;
    }
#endif

	default:
		break;
	}
}
