#ifndef MAIN_LOOP_H
#define MAIN_LOOP_H

#include <stdint.h>

/**
 * @brief       Flags to set in a bit mask to check in the main loop
 * @{
 */
typedef enum {
    FLAG_ADC_COMPLETE           = 0x0001, //!< ADC has completed conversion
    FLAG_UART_USB_RX            = 0x0002, //!< Bytes received on the USB UART
    FLAG_UART_USB_TX            = 0x0004, //!< Bytes transmitted on the USB UART
    FLAG_UART_WISP_RX           = 0x0008, //!< Bytes received on the WISP UART
    FLAG_UART_WISP_TX           = 0x0010, //!< Bytes transmitted on the WISP UART
    FLAG_LOGGING                = 0x0020, //!< Logging ADC conversion results to USB
    FLAG_RF_DATA				= 0x0040, //!< RF Rx activity ready to be logged
    FLAG_CHARGER_COMPLETE		= 0x0080, //!< Charge or discharge operation completed
    FLAG_INTERRUPTED     		= 0x0100, //!< target is in active debug mode
    FLAG_EXITED_DEBUG_MODE      = 0x0200, //!< debugger has restored energy level
    FLAG_WATCHPOINT_READY       = 0x0400, //!< watchpoint event ready for transmission to host
    FLAG_ENERGY_PROFILE_READY   = 0x0800, //!< send payload with data from EDB and app to host/ground
    FLAG_APP_OUTPUT             = 0x1000, //!< interrupt target and get app data packet
    FLAG_COLLECT_WATCHPOINTS    = 0x2000, //!< start collecting energy profile (watchpoints)
    FLAG_SEND_BEACON            = 0x4000, //!< transmit a beacon to ground
} main_loop_flag_t;

extern volatile uint16_t main_loop_flags; // bit mask containing bit flags to check in the main loop

#endif // MAIN_LOOP_H
