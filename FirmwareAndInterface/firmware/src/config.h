#ifndef CONFIG_H
#define CONFIG_H

// These are fixed constants properties of hardware
#define CONFIG_XT1_FREQ 32768ull
#define CONFIG_XT1_CAP 12 // pF
#define CONFIG_XT2_FREQ 12000000ull
// #define CONFIG_XT2_FREQ 25000000
#define CONFIG_REFO_FREQ 32768ull

#define MCU_ON_THRES 2607ull /* 2.1 V */ // int(math.ceil(voltage * 4096 / self.VDD))
#define MCU_BOOT_LATENCY_MS 3ull // measured: from Vreg = 2.2V to GPIO high at end of debug_setup()

// #define CONFIG_DCO_REF_SOURCE_REFO
// #define CONFIG_DCO_REF_CLOCK_DIV 1ull

// #define CONFIG_DCO_REF_SOURCE_XT1
// #define CONFIG_DCO_REF_CLOCK_DIV 1ull

#define CONFIG_DCO_REF_SOURCE_XT2
#define CONFIG_DCO_REF_CLOCK_DIV 4ull

#define CONFIG_CLOCK_SOURCE_DCO
// #define CONFIG_CLOCK_SOURCE_XT2

// #define CONFIG_DCOCLKDIV_FREQ 24576000ull
#define CONFIG_DCOCLKDIV_FREQ 24000000ull
// #define CONFIG_DCOCLKDIV_FREQ 21921792ull
// #define CONFIG_DCOCLKDIV_FREQ 16384000ull
// #define CONFIG_DCOCLKDIV_FREQ 12288000ull
// #define CONFIG_DCOCLKDIV_FREQ 8192000ull

#define CONFIG_TIMELOG_TIMER_SOURCE TASSEL__ACLK
// #define CONFIG_TIMELOG_TIMER_SOURCE TASSEL__SMCLK

#define CONFIG_TIMELOG_TIMER_DIV_BITS 0
#define CONFIG_TIMELOG_TIMER_DIV_BITS_EX 0
// #define CONFIG_TIMELOG_TIMER_DIV_BITS (ID0 | ID1)
// #define CONFIG_TIMELOG_TIMER_DIV_BITS_EX (TAIDEX_2 | TAIDEX_1 | TAIDEX_0)

#define CONFIG_ADC_TIMER_SOURCE TBSSEL__ACLK
#define CONFIG_ADC_SAMPLING_PERIOD 32768  // ACLK (32768 Hz) cycles

// #define CONFIG_USB_UART_BAUDRATE 2000000ull
// #define CONFIG_USB_UART_BAUDRATE 1500000ull
// #define CONFIG_USB_UART_BAUDRATE 1000000ull
// #define CONFIG_USB_UART_BAUDRATE 921600ull
// #define CONFIG_USB_UART_BAUDRATE 576000ull
// #define CONFIG_USB_UART_BAUDRATE 500000ull
// #define CONFIG_USB_UART_BAUDRATE 171264ull
#define CONFIG_USB_UART_BAUDRATE 115200ull
// #define CONFIG_USB_UART_BAUDRATE 38400ull

#define CONFIG_TARGET_UART_BAUDRATE 9600ull

#define CONFIG_ABORT_ON_USB_UART_ERROR // red led on, and if error is overflow, green led blinking

/**
 * @brief Abort if RFID event buffer overflows or drop the events
 */
#define CONFIG_ABORT_ON_RFID_EVENT_OVERFLOW

// #define CONFIG_CLOCK_TEST_MODE // enter a blinker loop after configuring clocks
// #define CONFIG_ROUTE_ACLK_TO_PIN // must "unplug" op amp buffers by disconnecting JP1

// Encode debugger state machine state onto pins
// #define CONFIG_STATE_PINS

// Encode RFID decoder state onto pins for debugging purposes
#define CONFIG_RFID_DECODER_STATE_PINS

// Use implementation of symbol time validity checks that requires bounds to
// be calculated at runtime. More aligned with the spec without chosen magic
// values, however might be prohibitevely expensive in terms of cycles.
// #define CONFIG_RFID_DECODER_RUNTIME_BOUNDS

/**
 * @brief Enable pull-down on the debugger<->target interrupt line.
 *
 * @details We watch for interrupts the target raises when the target requests
 *          to enter active debug mode, such as upon hitting an internal or
 *          external breakpoint. If the target is off or not present, then both
 *          ends of this line are in high impedence mode (effectively
 *          floating?). This does not seem to cause spurious interrupts, but if
 *          this problem did arise, enabling pull-down resistors should solve
 *          it.
 *
 *          NOTE: The pull-down causes energy interference (current is sourced
 *          from the target when target drives this pin high), but the target
 *          only drives this pin high for at most one cycle (signal
 *          communication is exclusively done using one-cycle-long pulses).
 */
// #define CONFIG_PULL_DOWN_ON_SIG_LINE

/**
 * @brief Enable code for decoding the RF protocol
 *
 * @details Currently, this is disabled because it causes spurious interrupts.
 *         TODO: setup RF pins only upon cmd to monitor RF because otherwise we
 *         get spurious interrupts on RX pin
 */
#define CONFIG_ENABLE_RF_PROTOCOL_MONITORING

/**
 * @brief Enable code to decode RFID transmissions from the target
 */
// #define CONFIG_ENABLE_RF_TX_DECODING

/**
 * @brief Decode the RFID command payload bits (not only the command code)
 * @details NOT IMPLEMENTED
 */
// #define CONFIG_DECODE_RFID_CMD_PAYLOAD

/**
 * @brief Route serial decoder events to external pins
 * @details For testing and tuning decoding of serial protocol over the signal
 *          line. Applies only to the USB_CMD_SERIAL_ECHO, not to regular
 *          debug mode enter sequence.
 */
// #define CONFIG_SIG_SERIAL_DECODE_PINS

/**
 * @brief Enable an auxiliary signal useful for triggering a scope
 * @details A pulse is issued when a host command is ready for execution.
 */
// #define CONFIG_SCOPE_TRIGGER_SIGNAL

// The rest essentially defines the register settings that carry out the above

#define MCU_BOOT_LATENCY_CYCLES (MCU_BOOT_LATENCY_MS * CONFIG_DCOCLKDIV_FREQ / 1000)

// See MSP430F5340 datasheet p44
#if CONFIG_XT1_CAP >= 12
#define CONFIG_XT1_CAP_BITS (XCAP0 | XCAP1)
#elif CONFIG_XT1_CAP >= 8
#define CONFIG_XT1_CAP_BITS (XCAP1)
#elif CONFIG_XT1_CAP >= 5
#define CONFIG_XT1_CAP_BITS (XCAP0)
#else
#define CONFIG_XT1_CAP_BITS 0
#endif

#if defined(CONFIG_DCO_REF_SOURCE_REFO)
#define CONFIG_DCO_REF_CLOCK_FREQ CONFIG_REFO_FREQ
#elif defined(CONFIG_DCO_REF_SOURCE_XT1)
#define CONFIG_DCO_REF_CLOCK_FREQ CONFIG_XT1_FREQ
#elif defined(CONFIG_DCO_REF_SOURCE_XT2)
#define CONFIG_DCO_REF_CLOCK_FREQ CONFIG_XT2_FREQ
#else // CONFIG_DCO_FREQ_SOURCE_*
#error Invalid DCO clock reference: see DCO_REF_SOURCE_*
#endif // CONFIG_DCO_REF_SOURCE_*

#define CONFIG_DCO_REF_FREQ (CONFIG_DCO_REF_CLOCK_FREQ / CONFIG_DCO_REF_CLOCK_DIV)

// DCO config
//
// NOTE: MSP430 crashes if it runs too fast?
// This may be caused by the average frequency from DCO modulation.  If we try to use
// a faster clock, the FLL may adjust the DCO above 25MHz to produce a clock with that
// average frequency.  If this happens, even for an instant, the MSP430 can crash.

// DCOCLK freq = 2^D * (N + 1) * REF_CLOCK_FREQ/REF_CLOCK_DIV
// DCOCLKDIV = DCOCLK / 2^D
//
// R | frequency range (MHz) (datasheet MSP430F5340 page 47)
// 0:  0.20 -  0.70
// 1:  0.36 -  1.47
// 2:  0.75 -  3.17
// 3:  1.51 -  6.07
// 4:  3.20 - 12.30
// 5:  6.00 - 23.70
// 6: 10.70 - 39.00
// 7: 19.60 - 60.00

#if CONFIG_DCOCLKDIV_FREQ == 24576000 && CONFIG_DCO_REF_FREQ == 32768

// 32768 * (668 + 1)
#define CONFIG_DCO_FREQ_R 7
#define CONFIG_DCO_FREQ_N 749
#define CONFIG_DCO_FREQ_D 0

#elif CONFIG_DCOCLKDIV_FREQ == 21921792 && CONFIG_DCO_REF_FREQ == 32768

// 32768 * (668 + 1)
#define CONFIG_DCO_FREQ_R 7
#define CONFIG_DCO_FREQ_N 668
#define CONFIG_DCO_FREQ_D 0

#elif CONFIG_DCOCLKDIV_FREQ == 16384000 && CONFIG_DCO_REF_FREQ == 32768

// 32768 * (499 + 1)
#define CONFIG_DCO_FREQ_R 6
#define CONFIG_DCO_FREQ_N 499
#define CONFIG_DCO_FREQ_D 1

#elif CONFIG_DCOCLKDIV_FREQ == 12288000 && CONFIG_DCO_REF_FREQ == 32768

// 32768 * (374 + 1)
#define CONFIG_DCO_FREQ_R 5
#define CONFIG_DCO_FREQ_N 374
#define CONFIG_DCO_FREQ_D 1

#elif CONFIG_DCOCLKDIV_FREQ == 8192000 && CONFIG_DCO_REF_FREQ == 32768

// 32768 * (249 + 1)
#define CONFIG_DCO_FREQ_R 5
#define CONFIG_DCO_FREQ_N 249
#define CONFIG_DCO_FREQ_D 1

#elif CONFIG_DCOCLKDIV_FREQ == 24000000 && CONFIG_DCO_REF_FREQ == 3000000

// 3000000 * (7 + 1)
#define CONFIG_DCO_FREQ_R 6
#define CONFIG_DCO_FREQ_N 7
#define CONFIG_DCO_FREQ_D 0

#endif // CONFIG_DCOCLKDIV_FREQ && CONFIG_DCO_REF_FREQ

// See MSP430F5340 datasheet (p. 177)
#if CONFIG_DCO_REF_CLOCK_DIV == 1
#define CONFIG_FLL_REF_DIV 0
#elif CONFIG_DCO_REF_CLOCK_DIV == 2
#define CONFIG_FLL_REF_DIV (FLLREFDIV0)
#elif CONFIG_DCO_REF_CLOCK_DIV == 4
#define CONFIG_FLL_REF_DIV (FLLREFDIV1)
#elif CONFIG_DCO_REF_CLOCK_DIV == 8
#define CONFIG_FLL_REF_DIV (FLLREFDIV0 | FLLREFDIV1)
#elif CONFIG_DCO_REF_CLOCK_DIV == 12
#define CONFIG_FLL_REF_DIV (FLLREFDIV2)
#elif CONFIG_DCO_REF_CLOCK_DIV == 16
#define CONFIG_FLL_REF_DIV (FLLREFDIV2 | FLLREFDIV0)
#else
#error Invalid DCO REF clock divider: see CONFIG_DCO_REF_CLOCK_DIV
#endif

// Worst-case settling time for the DCO when the DCO range bits have been changed:
// See MSP430x5xx Family User Manual (p. 165). The last fraction term is
// converting from FLL ref clock cycles to core clock cycles.
#define DCO_SETTLING_TIME \
    (1ull * CONFIG_DCO_REF_CLOCK_DIV * 32ull * 32ull * \
     (CONFIG_DCOCLKDIV_FREQ / CONFIG_DCO_REF_CLOCK_FREQ + 1ull))

#if CONFIG_DCOCLKDIV_FREQ != ((CONFIG_DCO_FREQ_N + 1) * CONFIG_DCO_REF_FREQ)
#error Inconsistent DCO freq config
#endif

// Clock source for MCLK, SMCLK
#if defined(CONFIG_CLOCK_SOURCE_DCO)
#define CONFIG_SMCLK_FREQ CONFIG_DCOCLKDIV_FREQ
#elif defined(CONFIG_CLOCK_SOURCE_XT2)
#define CONFIG_SMCLK_FREQ CONFIG_XT2_FREQ // for now, SMCLK source is not configurable
#else // CONFIG_CLOCK_SOURCE_*
#error Invalid main clock source: see CONFIG_CLOCK_SOURCE_*
#endif // CONFIG_CLOCK_SOURCE_*

#define CONFIG_UART_CLOCK_FREQ CONFIG_SMCLK_FREQ

// UART baudrate specification:
// N = SMCLK / BAUD
// BR0 = LSB(floor(N))
// BR1 = MSB(floor(N))
// BRS = (floor(N) - N) * 8

#if CONFIG_UART_CLOCK_FREQ == 24576000

#if CONFIG_USB_UART_BAUDRATE == 1000000
// N/16 = 24.576
#define CONFIG_USB_UART_BAUDRATE_BR0 0x01
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 9
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#elif CONFIG_USB_UART_BAUDRATE == 921600
// N/16 = 1.66666...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x01
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 10
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#elif CONFIG_USB_UART_BAUDRATE == 576000
// N/16 = 2.66666...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x02
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 10
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#elif CONFIG_USB_UART_BAUDRATE == 500000
// N/16 = 3.072
#define CONFIG_USB_UART_BAUDRATE_BR0 0x03
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 1
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#elif CONFIG_USB_UART_BAUDRATE == 115200
// N/16 = 13.3333...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x0d
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 5
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#elif CONFIG_USB_UART_BAUDRATE == 38400
// N = 640
#define CONFIG_USB_UART_BAUDRATE_BR0 0x80
#define CONFIG_USB_UART_BAUDRATE_BR1 0x02
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 2560
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0x00
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x0a
#define CONFIG_TARGET_UART_BAUDRATE_BRS 0

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 21921792

#if CONFIG_USB_UART_BAUDRATE == 921600
// N = 23.786666...
#define CONFIG_USB_UART_BAUDRATE_BR0 23
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 6

#elif CONFIG_USB_UART_BAUDRATE == 171264

// N = 128
#define CONFIG_USB_UART_BAUDRATE_BR0 128
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 115200

// N = 190.293333...
#define CONFIG_USB_UART_BAUDRATE_BR0 0xBE
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 2

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 2283.52
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0xEB
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x08
#define CONFIG_TARGET_UART_BAUDRATE_BRS 4

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 12500000

#if CONFIG_USB_UART_BAUDRATE == 2000000
#define CONFIG_USB_UART_BAUDRATE_BR0 6
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 2

#elif CONFIG_USB_UART_BAUDRATE == 1000000
#define CONFIG_USB_UART_BAUDRATE_BR0 12
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 4

#elif CONFIG_USB_UART_BAUDRATE == 115200
#define CONFIG_USB_UART_BAUDRATE_BR0 108
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 4

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0x16
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x05
#define CONFIG_TARGET_UART_BAUDRATE_BRS 1

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 6250000

#if CONFIG_USB_UART_BAUDRATE == 115200
#define CONFIG_USB_UART_BAUDRATE_BR0 54
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 2

#endif

#elif CONFIG_UART_CLOCK_FREQ == 24000000

#if CONFIG_USB_UART_BAUDRATE == 2000000
// N = 12
#define CONFIG_USB_UART_BAUDRATE_BR0 12
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 1000000
// N = 24
#define CONFIG_USB_UART_BAUDRATE_BR0 24
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 1500000
// N = 1
#define CONFIG_USB_UART_BAUDRATE_BR0 1
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRF 0
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#elif CONFIG_USB_UART_BAUDRATE == 500000
// N = 48
#define CONFIG_USB_UART_BAUDRATE_BR0 48
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 115200
// N = 13.0208333...
// #define CONFIG_USB_UART_BAUDRATE_BR0 13
// #define CONFIG_USB_UART_BAUDRATE_BR1 0
// #define CONFIG_USB_UART_BAUDRATE_BRF 1
// #define CONFIG_USB_UART_BAUDRATE_UCOS16 1

// N = 208.33333..
#define CONFIG_USB_UART_BAUDRATE_BR0 0xD0
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 3

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 2500
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0xC4
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x09
#define CONFIG_TARGET_UART_BAUDRATE_BRS 0

#endif // CONFIG_TARGET_UART_BAUDRATE

#elif CONFIG_UART_CLOCK_FREQ == 12000000

#if CONFIG_USB_UART_BAUDRATE == 2000000
// N = 6
#define CONFIG_USB_UART_BAUDRATE_BR0 6
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#endif // CONFIG_USB_UART_BAUDRATE

#if CONFIG_TARGET_UART_BAUDRATE == 9600
// N = 1250
#define CONFIG_TARGET_UART_BAUDRATE_BR0 0xE2
#define CONFIG_TARGET_UART_BAUDRATE_BR1 0x04
#define CONFIG_TARGET_UART_BAUDRATE_BRS 0

#endif // CONFIG_TARGET_UART_BAUDRATE

#endif // CONFIG_UART_CLOCK_FREQ_*

#if !defined(CONFIG_USB_UART_BAUDRATE_BR0) || !defined(CONFIG_TARGET_UART_BAUDRATE_BR0)
#error UART configuration error: no config for selected CONFIG_UART_CLOCK_FREQ
#endif

#endif // CONFIG_H
