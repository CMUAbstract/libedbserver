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

#include "monitor.h"
#include "ucs.h"
#include "adc12.h"
#include "uart.h"
#include "gpio.h"
#include "pwm.h"
#include "timeLog.h"
#include "timer1.h"
#include "rfid.h"
#include "marker.h"
#include "main.h"

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

static adc12_t adc12;
// indices in the adc12.config.channels and adc12.results arrays
static int8_t vcap_index = -1;
static int8_t vboost_index = -1;
static int8_t vreg_index = -1;
static int8_t vrect_index = -1;
static int8_t vinj_index = -1;

static uint16_t flags = 0; // bit mask containing bit flags to check in the main loop
static uint8_t log_flags = 0; // bit mask containing active log values to send via USB

static uint16_t adc12Target; // target ADC reading

static uartPkt_t wispRxPkt = { .processed = 1 };

int main(void)
{
    uartPkt_t usbRxPkt = { .processed = 1 };

    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;

    UCS_setup(); // set up unified clock system
    pin_init();
    PWM_setup();
    UART_setup(UART_INTERFACE_USB, &flags, FLAG_UART_USB_RX, FLAG_UART_USB_TX); // USCI_A0 UART
    UART_setup(UART_INTERFACE_WISP, &flags, FLAG_UART_WISP_RX, FLAG_UART_WISP_TX); // USCI_A1 UART

    RFID_setup(&flags, FLAG_RF_DATA, FLAG_RF_DATA); // use the same flag for Rx and Tx so
    												// we only have to check one flag

    // initialize ADC12 configuration structure
    adc12.pFlags = &flags;
    adc12.flag_adc12Complete = FLAG_ADC12_COMPLETE;
    adc12.config.num_channels = 0;

    __enable_interrupt();                   // enable all interrupts

    long count = 0;
    while(1) {
        if(flags & FLAG_ADC12_COMPLETE) {
            // ADC12 has completed conversion on all active channels
            flags &= ~FLAG_ADC12_COMPLETE;

            if(flags & FLAG_LOGGING) {
                // check if we need to send Vcap
                if(log_flags & LOG_VCAP) {
                	// send ADC conversion time and conversion result
                	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
                			(uint8_t *)(&(adc12.timeComplete)), sizeof(uint32_t),
							UART_TX_DROP);
                    UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VCAP,
                    	(uint8_t *)(&(adc12.results[vcap_index])), sizeof(uint16_t),
						UART_TX_DROP);
                }

                // check if we need to send Vboost
                if(log_flags & LOG_VBOOST) {
                	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
                	             (uint8_t *)(&(adc12.timeComplete)), sizeof(uint32_t),
								 UART_TX_DROP);
                    UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VBOOST,
                    	(uint8_t *)(&(adc12.results[vboost_index])), sizeof(uint16_t),
						UART_TX_DROP);
                }

                // check if we need to send Vreg
                if(log_flags & LOG_VREG) {
                	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
									(uint8_t *)(&(adc12.timeComplete)), sizeof(uint32_t),
									UART_TX_DROP);
                    UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VREG,
                    		(uint8_t *)(&(adc12.results[vreg_index])), sizeof(uint16_t),
							UART_TX_DROP);
                }

                // check if we need to send Vrect
                if(log_flags & LOG_VRECT) {
                	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
								 (uint8_t *)(&(adc12.timeComplete)), sizeof(uint32_t),
								 UART_TX_DROP);
                    UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VRECT,
                        (uint8_t *)(&(adc12.results[vrect_index])), sizeof(uint16_t),
						UART_TX_DROP);
                }

                // check if we need to send Vinj
                if(log_flags & LOG_VINJ) {
                	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_TIME,
							(uint8_t *)(&(adc12.timeComplete)), sizeof(uint32_t),
							UART_TX_DROP);
                	UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VINJ,
                			(uint8_t *)(&(adc12.results[vinj_index])), sizeof(uint16_t),
							UART_TX_DROP);
                }

                ADC12_START;
                ADC12_START; // I don't know why, but for some reason this seems to
                			 // only work if the ADC12SC bit is set twice.  I hope
                			 // this doesn't indicate that something is horribly,
                			 // horribly wrong.
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
        if(count++ > 500000) {
        	PLEDOUT ^= LED3;
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

    switch(pkt->descriptor)
    {
    case USB_CMD_GET_VCAP:
        adc12Result = adc12Read_block(ADC12INCH_VCAP);
        UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VCAP,
                        (uint8_t *)(&adc12Result), sizeof(uint16_t),
						UART_TX_FORCE);
        break;

    case USB_CMD_GET_VBOOST:
        adc12Result = adc12Read_block(ADC12INCH_VBOOST);
        UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VBOOST,
                        (uint8_t *)(&adc12Result), sizeof(uint16_t),
						UART_TX_FORCE);
        break;

    case USB_CMD_GET_VREG:
        adc12Result = adc12Read_block(ADC12INCH_VREG);
        UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VREG,
                        (uint8_t *)(&adc12Result), sizeof(uint16_t),
						UART_TX_FORCE);
        break;

    case USB_CMD_GET_VRECT:
        adc12Result = adc12Read_block(ADC12INCH_VRECT);
        UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VRECT,
                        (uint8_t *)(&adc12Result), sizeof(uint16_t),
						UART_TX_FORCE);
        break;

    case USB_CMD_SET_VCAP:
        adc12Target = *((uint16_t *)(pkt->data));
        setWispVoltage_block(ADC12INCH_VCAP, &vcap_index, adc12Target);
        break;

    case USB_CMD_SET_VBOOST:
        adc12Target = *((uint16_t *)(pkt->data));
        setWispVoltage_block(ADC12INCH_VBOOST, &vboost_index, adc12Target);
        break;

    case USB_CMD_ENTER_ACTIVE_DEBUG:
    {
    	// todo: turn off all logging?

        adc12Target = adc12Read_block(ADC12INCH_VCAP); // read Vcap and set as the target for exit

        // report reading over USB
        UART_sendMsg(UART_INTERFACE_USB, USB_RSP_VCAP,
                        (uint8_t *)(&adc12Target), sizeof(uint16_t),
						UART_TX_FORCE);

        // bring AUX1 high, interrupting the WISP to enter active debug mode
    	PAUXSEL &= ~GPIO_AUX_1;		// GPIO option select
        PAUXOUT |= GPIO_AUX_1;		// output high
        PAUXDIR |= GPIO_AUX_1;		// output direction

        PLEDOUT |= LED4;

        P5SEL &= ~WISP_CHARGE; // option select for PWM charge line
        P5OUT |= WISP_CHARGE; // output high for PWM charge line
        break;
    }

    case USB_CMD_EXIT_ACTIVE_DEBUG:
    {
    	uint16_t pauxie = PAUXIE;
    	PAUXIE &= ~GPIO_AUX_2; // we need to use AUX 2 to exit debug mode, so disable interrupt

    	// let the WISP know so that it can perform pre-exit tasks
    	UART_sendMsg(UART_INTERFACE_WISP, WISP_CMD_EXIT_ACTIVE_DEBUG, 0, 0, UART_TX_FORCE);

    	while(!(PAUXIN & GPIO_AUX_2)); // WISP should raise AUX 2 while performing
    								   // pre-exit tasks, so wait for this

    	// Wait for WISP to indicate that it's ready to exit debug mode
		while(PAUXIN & GPIO_AUX_2); // WISP will pull AUX 2 low when ready

		// Now we need to wait for the WISP cap's voltage to drop below the target,
		// and start PWM until we get there.
		PWM_stop();
		while(adc12Read_block(ADC12INCH_VCAP) > adc12Target); // wait for the voltage to drop
		setWispVoltage_block(ADC12INCH_VCAP, &vcap_index, adc12Target); // set the voltage
		PAUXOUT &= ~GPIO_AUX_1; // signal to the WISP that we're exiting active debug mode
		PWM_stop();
		PAUXIE = pauxie; // restore interrupt enable register

		PLEDOUT &= ~LED4;
    	break;
    }

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

    case USB_CMD_LOG_VCAP_BEGIN:
        flags |= FLAG_LOGGING; // for main loop
        log_flags |= LOG_VCAP; // for main loop
        TimeLog_request(1); // request timer overflow notifications
        addAdcChannel(ADC12INCH_VCAP, &vcap_index);
        restartAdc();
        break;

    case USB_CMD_LOG_VCAP_END:
        log_flags &= ~LOG_VCAP;
        if(!log_flags) {
			flags &= ~FLAG_LOGGING; // for main loop
		}
        TimeLog_request(0);

		removeAdcChannel(&vcap_index);
        restartAdc();
        break;

    case USB_CMD_LOG_VBOOST_BEGIN:
        flags |= FLAG_LOGGING; // for main loop
        log_flags |= LOG_VBOOST; // for main loop
        TimeLog_request(1); // request timer overflow notifications
        addAdcChannel(ADC12INCH_VBOOST, &vboost_index);
        restartAdc();
        break;

    case USB_CMD_LOG_VBOOST_END:
        log_flags &= ~LOG_VBOOST;
        if(!log_flags) {
        	flags &= ~FLAG_LOGGING; // for main loop
        }
        TimeLog_request(0);

		removeAdcChannel(&vboost_index);
        restartAdc();
        break;

    case USB_CMD_LOG_VREG_BEGIN:
        flags |= FLAG_LOGGING; // for main loop
        log_flags |= LOG_VREG; // for main loop
        TimeLog_request(1); // request timer overflow notifications
        addAdcChannel(ADC12INCH_VREG, &vreg_index);
        restartAdc();
        break;

    case USB_CMD_LOG_VREG_END:
        log_flags &= ~LOG_VREG;
        if(!log_flags) {
			flags &= ~FLAG_LOGGING; // for main loop
		}
        TimeLog_request(0);
		removeAdcChannel(&vreg_index);
        restartAdc();
        break;

    case USB_CMD_LOG_VRECT_BEGIN:
        flags |= FLAG_LOGGING; // for main loop
        log_flags |= LOG_VRECT; // for main loop
        TimeLog_request(1); // request timer overflow notifications
        addAdcChannel(ADC12INCH_VRECT, &vrect_index);
        restartAdc();
        break;

    case USB_CMD_LOG_VRECT_END:
        log_flags &= ~LOG_VRECT;
        if(!log_flags) {
			flags &= ~FLAG_LOGGING; // for main loop
		}
        TimeLog_request(0);
        removeAdcChannel(&vrect_index);
        restartAdc();
        break;

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
        // pulse a gpio pin to use as  scope trigger
        P1DIR |= GPIO_AUX_3;
        P1OUT |= GPIO_AUX_3;
        P1OUT &= ~GPIO_AUX_3;

        adc12Target = *((uint16_t *)(&pkt->data[0]));
        charge_block(adc12Target);
        break;

    case USB_CMD_DISCHARGE:
        // pulse a gpio pin to use as  scope trigger
        P1DIR |= GPIO_AUX_3;
        P1OUT |= GPIO_AUX_3;
        P1OUT &= ~GPIO_AUX_3;

        adc12Target = *((uint16_t *)(pkt->data));
        discharge_block(adc12Target);
        break;

    case USB_CMD_PULSE_AUX_3:
        P1DIR |= GPIO_AUX_3;
        P1OUT |= GPIO_AUX_3;
        P1OUT &= ~GPIO_AUX_3;
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

    case USB_CMD_LOG_VINJ_BEGIN:
    	flags |= FLAG_LOGGING; // for main loop
		log_flags |= LOG_VINJ; // for main loop
		TimeLog_request(1);
		addAdcChannel(ADC12INCH_VINJ, &vinj_index);
		restartAdc();
    	break;

    case USB_CMD_LOG_VINJ_END:
		log_flags &= ~LOG_VINJ;
		if(!log_flags) {
			flags &= ~FLAG_LOGGING; // for main loop
		}
		TimeLog_request(0);
		removeAdcChannel(&vinj_index);
		restartAdc();
    	break;

    case USB_CMD_PWM_HIGH:
    	PWM_stop();
    	P5OUT |= WISP_CHARGE; // output high
    	break;

    // USB_CMD_PWM_LOW and USB_CMD_PWM_OFF do the same thing

    case USB_CMD_MONITOR_MARKER_BEGIN:
        marker_monitor_begin();
        break;

    case USB_CMD_MONITOR_MARKER_END:
        marker_monitor_end();
        break;

    default:
        break;
    }

    pkt->processed = 1;
}

static void addAdcChannel(uint16_t channel, int8_t *pResults_index)
{
    // note that the results index corresponds to the
    // index in the channels array

    // first check if the channel is already present
	if(*pResults_index == -1) {
        // channel isn't present in the configuration, so add it
        adc12.config.channels[adc12.config.num_channels] = channel;
        *pResults_index = adc12.config.num_channels++;
    }
}

static void removeAdcChannel(int8_t *pResults_index)
{
    // note that results_index is also the corresponding
    // index in the channels array

	if(*pResults_index != -1) {
		// the channel is present in the configuration
		adc12.config.num_channels--;

		if(*pResults_index != adc12.config.num_channels) {
			// We're removing this channel, but it wasn't the last channel
			// configured.  We need to move the other channels in the channels
			// array so there are no missing channels in the array.
			uint8_t i;
			for(i = *pResults_index + 1; i < adc12.config.num_channels + 1; i++) {
				adc12.config.channels[i - 1] = adc12.config.channels[i];
			}

			// update the results array indices
			if(vcap_index >= adc12.config.num_channels) {
				vcap_index--;
			}
			if(vboost_index >= adc12.config.num_channels) {
				vboost_index--;
			}
			if(vreg_index >= adc12.config.num_channels) {
				vreg_index--;
			}
			if(vrect_index >= adc12.config.num_channels) {
				vrect_index--;
			}
			if(vinj_index >= adc12.config.num_channels) {
				vinj_index--;
			}
		}

		*pResults_index = -1; // remove the channel
	}
}

static void restartAdc()
{
    // don't need to worry about restarting if there are no active channels
    if(adc12.config.num_channels > 0) {
        ADC12_STOP;
        ADC12_WAIT; // wait for ADC to stop (it needs to complete a conversion if multiple channels are active)
        ADC12_configure(&adc12); // reconfigure
        ADC12_START;
    }
}

static uint16_t adc12Read_block(uint16_t channel)
{
    adc12_t adc12_temp = { .pFlags = 0,
    					   .config.channels[0] = channel,
						   .config.num_channels = 1 };
    uint16_t adc12Result;

    ADC12_STOP; // stop any active conversion
    ADC12_WAIT;  // wait for conversion to complete so ADC is stopped
    ADC12_configure(&adc12_temp); // reconfigure ADC
    ADC12_DISABLE_INTERRUPT;
    ADC12_START; // start conversion
    ADC12_WAIT; // wait for conversion to complete
    adc12Result = ADC12MEM0;

    ADC12_configure(&adc12); // restore previous configuration, enable interrupt

    return adc12Result;
}

static void charge_block(uint16_t target)
{
    uint16_t chan = ADC12INCH_VCAP;
    int8_t chan_index;
    uint16_t cur_voltage;

    addAdcChannel(chan, &chan_index);
    restartAdc();

    /* Output Vcc level to Vcap (through R1) */

    /* Configure the pin */
    P5DS |= WISP_CHARGE; /* full drive strength (note that R1 is the bottleneck for current) */
    P5SEL &= ~WISP_CHARGE; /* I/O function */
    P5DIR |= WISP_CHARGE; /* I/O function output */

    P5OUT |= WISP_CHARGE;

    /* Wait for the cap to charge to that voltage */

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = adc12Read_block(chan);
    } while (cur_voltage < target);

    P5OUT &= ~(WISP_CHARGE);

    removeAdcChannel(&chan_index);
}

static void discharge_block(uint16_t target)
{
    uint16_t chan = ADC12INCH_VCAP;
    int8_t chan_index;
    uint16_t cur_voltage;

    addAdcChannel(chan, &chan_index);
    restartAdc();

    PDISCHGDIR |= GPIO_DISCHARGE;

    /* The measured effective period of this loop is roughly 30us ~ 33kHz (out
     * of 200kHz that the ADC can theoretically do). */
    do {
        cur_voltage = adc12Read_block(chan);
    } while (cur_voltage > target);

    PDISCHGDIR &= ~GPIO_DISCHARGE;

    removeAdcChannel(&chan_index);
}

static void setWispVoltage_block(uint16_t channel, int8_t *pResults_index, uint16_t target)
{
	uint16_t result;
	int8_t compare;
	uint8_t threshold = 1;
	addAdcChannel(channel, pResults_index);
	restartAdc();

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

		result = adc12Read_block(channel);
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
	if((channel == ADC12INCH_VCAP && !(log_flags & LOG_VCAP)) ||
				(channel == ADC12INCH_VBOOST && !(log_flags & LOG_VBOOST))) {
		// We don't need this ADC channel anymore, since we're done here and
		// we're not logging this voltage right now.
		removeAdcChannel(pResults_index);
		restartAdc();
	}
}

static int8_t uint16Compare(uint16_t n1, uint16_t n2, uint16_t threshold) {
	if(n1 < n2 - threshold && n2 >= threshold) {
		return -1;
	} else if(n1 > n2 + threshold && n2 + threshold <= 0xFFFF) {
		return 1;
	}
	return 0;
}
