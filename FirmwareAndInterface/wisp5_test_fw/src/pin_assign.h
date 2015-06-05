#ifndef PIN_ASSIGN_H
#define PIN_ASSIGN_H

#define BIT_INNER(idx) BIT ## idx
#define BIT(idx) BIT_INNER(idx)

#define GPIO_INNER(port, reg) P ## port ## reg
#define GPIO(port, reg) GPIO_INNER(port, reg)

#define INTFLAG_INNER(port, pin) P ## port ## IV_ ## P ## port ## IFG ## pin
#define INTFLAG(port, pin) INTFLAG_INNER(port, pin)

// Ugly workaround to make the pretty GPIO macro work for OUT register
// (a control bit for TAxCCTLx uses the name 'OUT')
#undef OUT


#define PORT_STATE  3
#define PIN_STATE_0 4 // lsb
#define PIN_STATE_1 5 // msb

#define PORT_SIG   1
#define PIN_SIG    4

#endif // PIN_ASSIGN_H
