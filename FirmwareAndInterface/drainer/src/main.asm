    .cdecls C,LIST, "msp430fr5969.h"

    .text
    .def main

main:
    ; watchdog is active on reset, so disable it, to have rogue resets
    MOV #WDTPW+WDTHOLD, &WDTCTL

loop:
    ; States: LPM0 - LPM4 (change the obvious macro below)
    NOP
    BIS #LPM2, SR
    NOP

    ; State: LPM5
    ; NOP
    ; BIC #GIE, SR
    ; NOP
    ; MOV.B #PMMPW_H, &PMMCTL0_H
    ; BIS.B #PMMREGOFF, &PMMCTL0_L
    ; MOV.B #000h, &PMMCTL0_H
    ; BIS #CPUOFF+OSCOFF+SCG0+SCG1, SR

    JMP loop

    .end


