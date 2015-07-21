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
// #define CONFIG_DCO_REF_CLOCK_DIV 1

//#define CONFIG_DCO_REF_SOURCE_XT1
//#define CONFIG_DCO_REF_CLOCK_DIV 1

#define CONFIG_DCO_REF_SOURCE_XT2
#define CONFIG_DCO_REF_CLOCK_DIV 4

#define CONFIG_CLOCK_SOURCE_DCO
// #define CONFIG_CLOCK_SOURCE_XT2

// #define CONFIG_DCOCLKDIV_FREQ 24576000ull
#define CONFIG_DCOCLKDIV_FREQ 24000000ull
// #define CONFIG_DCOCLKDIV_FREQ 21921792ull
// #define CONFIG_DCOCLKDIV_FREQ 16384000ull
// #define CONFIG_DCOCLKDIV_FREQ 12288000ull
// #define CONFIG_DCOCLKDIV_FREQ 8192000ull

#define CONFIG_CLK_DIV_MCLK         1
#define CONFIG_CLK_DIV_SMCLK        1
#define CONFIG_CLK_DIV_ACLK         1

#define CONFIG_TIMELOG_TIMER_SOURCE TASSEL__ACLK
// #define CONFIG_TIMELOG_TIMER_SOURCE TASSEL__SMCLK

#define CONFIG_TIMELOG_TIMER_DIV 1
#define CONFIG_TIMELOG_TIMER_DIV_BITS_EX 0
// #define CONFIG_TIMELOG_TIMER_DIV_BITS_EX (TAIDEX_2 | TAIDEX_1 | TAIDEX_0)

// #define CONFIG_ADC_TIMER_SOURCE_ACLK
#define CONFIG_ADC_TIMER_SOURCE_SMCLK
// #define CONFIG_ADC_TIMER_SOURCE_MCLK

#define CONFIG_ADC_TIMER_DIV 8
#define CONFIG_ADC_SAMPLING_FREQ_HZ 1000

#if defined(CONFIG_ADC_TIMER_SOURCE_ACLK)
#define CONFIG_ADC_TIMER_SOURCE_NAME ACLK
#define CONFIG_ADC_TIMER_CLK_FREQ CONFIG_ACLK_FREQ
#elif defined(CONFIG_ADC_TIMER_SOURCE_SMCLK)
#define CONFIG_ADC_TIMER_SOURCE_NAME SMCLK
#define CONFIG_ADC_TIMER_CLK_FREQ CONFIG_SMCLK_FREQ
#elif defined(CONFIG_ADC_TIMER_SOURCE_SMCLK)
#define CONFIG_ADC_TIMER_SOURCE_NAME MCLK
#define CONFIG_ADC_TIMER_CLK_FREQ CONFIG_MCLK_FREQ
#elif defined(CONFIG_ADC_TIMER_SOURCE_MCLK)
#else
#error No clock source selected for ADC timer: see CONFIG_ADC_TIMER_SOURCE_*
#endif // CONFIG_ADC_TIMER_SOURCE_*

#define CONFIG_ADC_TIMER_FREQ (CONFIG_ADC_TIMER_CLK_FREQ / CONFIG_ADC_TIMER_DIV)
#define CONFIG_ADC_SAMPLING_PERIOD (CONFIG_ADC_TIMER_FREQ / CONFIG_ADC_SAMPLING_FREQ_HZ)

#define CONFIG_USB_UART_BAUDRATE 2000000ull
// #define CONFIG_USB_UART_BAUDRATE 1500000ull
// #define CONFIG_USB_UART_BAUDRATE 1000000ull
// #define CONFIG_USB_UART_BAUDRATE 921600ull
// #define CONFIG_USB_UART_BAUDRATE 576000ull
// #define CONFIG_USB_UART_BAUDRATE 500000ull
// #define CONFIG_USB_UART_BAUDRATE 460800ull
// #define CONFIG_USB_UART_BAUDRATE 171264ull
// #define CONFIG_USB_UART_BAUDRATE 115200ull
// #define CONFIG_USB_UART_BAUDRATE 38400ull

#define CONFIG_TARGET_UART_BAUDRATE 9600ull

// #define CONFIG_USB_UART_UCOS16
// #define CONFIG_TARGET_UART_UCOS16

#define CONFIG_ABORT_ON_USB_UART_ERROR // red led on, and if error is overflow, green led blinking

/**
 * @brief Abort if RFID event buffer overflows or drop the events
 */
#define CONFIG_ABORT_ON_RFID_EVENT_OVERFLOW

/**
 * @brief Two alternative workarounds for the crashes due to voltage ramp on startup
 * @detials The problem these work around is crashes that occur at arbitrary times
 *          after startup. The crash is either a wild jump or an oscillator
 *          fault.
 *
 *          At first glance crashes are somewhat consistent and seem
 *          correlated to peripheral activity. For example, not doing anything
 *          with the UART might seem to get rid of the crash, or, even
 *          stranger, re-arranging C-code, like removing code that's in an
 *          if-branch that is never taken, or replacing a function call with an
 *          inline invocation, all appear to consistently influence whether a
 *          crash happens or not.
 *
 *          However, the underlying issue is traced to some kind of
 *          non-fail-stop failure during startup that corrupts subsequent
 *          execution. Furthermore, the problem has to do with the Power
 *          Management Module -- it either generates false events (like
 *          overvoltage), or does not generate correct events in time. In
 *          particular, there are multiple errata about it, including a false
 *          overvoltage even if voltage ramps up too quickly.
 *
 *          The delay workaround is recommended over disabling PMM, since PMM
 *          is nice to have to detect brownout -- we do want it if it works
 *          correctly. It is plausible that there is a true fix that involves
 *          configuration of PMM.
 *
 *          NOTE: Unfortunately, these workarounds do not get rid of the same kind
 *                of crash that happens with debugger attached (either CCSv6 or
 *                mspdebug).
 */
// #define CONFIG_STARTUP_VOLTAGE_WORKAROUND_DISABLE_PMM
#define CONFIG_STARTUP_VOLTAGE_WORKAROUND_DELAY

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
#define CONFIG_ENABLE_RF_TX_DECODING

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

/**
 * @brief Capacitor charge implemented by a PWM with a control loop around the duty-cycle
 * @detail The alternative is an "valve" method of raising a GPIO high and watching
 *         the voltage level with ADC or comparator and pulling the GPIO low once
 *         the target threshold is crossed.
 */
// #define CONFIG_PWM_CHARGING

// The rest essentially defines the register settings that carry out the above

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

// Worst-case settling time for the DCO when the DCO range bits have been changed:
// See MSP430x5xx Family User Manual (p. 165). The last fraction term is
// converting from FLL ref clock cycles to core clock cycles.
#define DCO_SETTLING_TIME \
    (1ull * CONFIG_DCO_REF_CLOCK_DIV * 32ull * 32ull * \
     (CONFIG_DCOCLKDIV_FREQ / CONFIG_DCO_REF_CLOCK_FREQ + 1ull))

#if CONFIG_DCOCLKDIV_FREQ != ((CONFIG_DCO_FREQ_N + 1) * CONFIG_DCO_REF_FREQ)
#error Inconsistent DCO freq config
#endif

// TODO: this is not the case for all possible configs
#define CONFIG_ACLK_SRC_FREQ    CONFIG_ACLK_XT1_FREQ

// Clock source for MCLK, SMCLK
#if defined(CONFIG_CLOCK_SOURCE_DCO)
#define CONFIG_MCLK_SRC_FREQ CONFIG_DCOCLKDIV_FREQ
#define CONFIG_SMCLK_SRC_FREQ CONFIG_DCOCLKDIV_FREQ
#elif defined(CONFIG_CLOCK_SOURCE_XT2)
#define CONFIG_MCLK_SRC_FREQ CONFIG_XT2_FREQ // for now, SMCLK source is not configurable
#define CONFIG_SMCLK_SRC_FREQ CONFIG_XT2_FREQ // for now, SMCLK source is not configurable
#else // CONFIG_CLOCK_SOURCE_*
#error Invalid main clock source: see CONFIG_CLOCK_SOURCE_*
#endif // CONFIG_CLOCK_SOURCE_*

#define CONFIG_ACLK_FREQ (CONFIG_ACLK_SRC_FREQ / CONFIG_CLK_DIV_ACLK)
#define CONFIG_SMCLK_FREQ (CONFIG_SMCLK_SRC_FREQ / CONFIG_CLK_DIV_SMCLK)
#define CONFIG_MCLK_FREQ (CONFIG_MCLK_SRC_FREQ / CONFIG_CLK_DIV_MCLK)

#define MCU_BOOT_LATENCY_CYCLES (MCU_BOOT_LATENCY_MS * CONFIG_MCLK_FREQ / 1000)

#define CONFIG_UART_CLOCK_FREQ CONFIG_SMCLK_FREQ

// UART baudrate specification:
//
// Not UCOS16:
//
// N = SMCLK / BAUD
// BR0 = LSB(floor(N))
// BR1 = MSB(floor(N))
// BRS = (floor(N) - N) * 8
//
// UCOS16:
//
// N = SMCLK / BAUD
// BR0 = LSB(floor(N/16))
// BR1 = MSB(floor(N/16))
// BRF = (floor(N/16) - N/16) * 16
// UCOS16 = 1

#if CONFIG_UART_CLOCK_FREQ == 24576000

#if CONFIG_USB_UART_BAUDRATE == 1000000

#ifdef CONFIG_USB_UART_UCOS16

// N/16 = 24.576
#define CONFIG_USB_UART_BAUDRATE_BR0 0x01
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 9
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 921600

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 1.66666...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x01
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 10
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 576000

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 2.66666...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x02
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 10
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 500000

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 3.072
#define CONFIG_USB_UART_BAUDRATE_BR0 0x03
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 1
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 115200

#ifdef CONFIG_USB_UART_UCOS16
// N/16 = 13.3333...
#define CONFIG_USB_UART_BAUDRATE_BR0 0x0d
#define CONFIG_USB_UART_BAUDRATE_BR1 0x00
#define CONFIG_USB_UART_BAUDRATE_BRF 5
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

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

#ifdef CONFIG_USB_UART_UCOS16
#define CONFIG_USB_UART_BAUDRATE_BR0 1
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRF 8
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#else // !CONFIG_USB_UART_UCOS16
// N = 24
#define CONFIG_USB_UART_BAUDRATE_BR0 24
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0
#endif // !CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 1500000

#ifdef CONFIG_USB_UART_UCOS16
// N = 1
#define CONFIG_USB_UART_BAUDRATE_BR0 1
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRF 0
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1
#endif // CONFIG_USB_UART_UCOS16

#elif CONFIG_USB_UART_BAUDRATE == 500000
// N = 48
#define CONFIG_USB_UART_BAUDRATE_BR0 48
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 115200

#ifdef CONFIG_USB_UART_UCOS16
// N = 13.0208333...
#define CONFIG_USB_UART_BAUDRATE_BR0 13
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRF 1
#define CONFIG_USB_UART_BAUDRATE_UCOS16 1

#else // !CONFIG_USB_UART_UCOS16
// N = 208.33333..
#define CONFIG_USB_UART_BAUDRATE_BR0 0xD0
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 3
#endif // !CONFIG_USB_UART_UCOS16

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

#elif CONFIG_USB_UART_BAUDRATE == 460800
// N = 26.041666... 
#define CONFIG_USB_UART_BAUDRATE_BR0 26
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 0

#elif CONFIG_USB_UART_BAUDRATE == 115200

// N = 104.16666...
#define CONFIG_USB_UART_BAUDRATE_BR0 104
#define CONFIG_USB_UART_BAUDRATE_BR1 0
#define CONFIG_USB_UART_BAUDRATE_BRS 1

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
