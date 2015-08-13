#ifndef LIBDEBUG_PIN_ASSIGN_H
#define LIBDEBUG_PIN_ASSIGN_H

#define BIT_INNER(idx) BIT ## idx
#define BIT(idx) BIT_INNER(idx)

#define GPIO_INNER(port, reg) P ## port ## reg
#define GPIO(port, reg) GPIO_INNER(port, reg)

#define INTFLAG_INNER(port, pin) P ## port ## IV_ ## P ## port ## IFG ## pin
#define INTFLAG(port, pin) INTFLAG_INNER(port, pin)

// Ugly workaround to make the pretty GPIO macro work for OUT register
// (a control bit for TAxCCTLx uses the name 'OUT')
#undef OUT

// NOTE: underscores LED_# iin order to not confict with wisp-base lib
//
// MCU-side LED
#define PORT_LED_1   4
#define PIN_LED_1    0
//
// Non-MCU-side LED
#define PORT_LED_2   J
#define PIN_LED_2    6

#define PORT_STATE  3
#define PIN_STATE_0 4 // lsb
#define PIN_STATE_1 5 // msb

#define PORT_SIG   1
#define PIN_SIG    4

// Code point pins must be on same port and consecutive
// NOTE: When using the same pins as PIN_STATE, must disable CONFIG_STATE_PINS
// NOTE: Cannot use macros in inline assembly, so debug.h has these hardcoded!
#define PORT_CODEPOINT  3
#define PIN_CODEPOINT_0 4 // lsb
#define PIN_CODEPOINT_1 5 // msb
#define BITS_CODEPOINT  (BIT(PIN_CODEPOINT_0) | BIT(PIN_CODEPOINT_1))

#endif // LIBDEBUG_PIN_ASSIGN_H
