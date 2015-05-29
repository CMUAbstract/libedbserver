    .cdecls C,LIST, "msp430fr5969.h"

    .text
    .def main

main:
    ; watchdog is active on reset, so disable it, to have rogue resets
    MOV #WDTPW+WDTHOLD, &WDTCTL

    ; Initialize all pins to outputs

    MOV.B #0xff, &P1DIR
    MOV.B #0x0, &P1OUT

    MOV.B #0xff, &P2DIR
    MOV.B #0x0, &P2OUT

    MOV.B #0xff, &P3DIR
    MOV.B #0x0, &P3OUT

    MOV.B #0xff, &P4DIR
    MOV.B #0x0, &P4OUT

    MOV.B #0xff, &PJDIR
    MOV.B #0x0, &PJOUT

    BIC.B #LOCKLPM5, &PM5CTL0 ; "commit" pin config

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


