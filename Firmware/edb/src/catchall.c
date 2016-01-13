/*
 * @file catchall.c
 *
 * @author Aaron Parks
 */


#include <msp430.h>

#include <libmsp/periph.h>

#include "config.h"
#include "pin_assign.h"
#include "error.h"

// TODO: This is board-specific: maybe make a module_assign.h like pin_assig.h
//       For now, we say that an interrupt is used if there is at least one
//       build config in which it is used.

// #define INT_HANDLED_RTC
#define INT_HANDLED_PORT2
#define INT_HANDLED_TIMER2_A1
// #define INT_HANDLED_TIMER2_A0
// #define INT_HANDLED_USCI_B1
#define INT_HANDLED_USCI_A1
#define INT_HANDLED_PORT1
#define INT_HANDLED_TIMER1_A1
#define INT_HANDLED_TIMER1_A0
#define INT_HANDLED_DMA
// #define INT_HANDLED_TIMER0_A1
#define INT_HANDLED_TIMER0_A0
#define INT_HANDLED_ADC12
// #define INT_HANDLED_USCI_B0
#define INT_HANDLED_USCI_A0
#define INT_HANDLED_WDT
// #define INT_HANDLED_TIMER0_B1
// #define INT_HANDLED_TIMER0_B0
#define INT_HANDLED_COMP_B
#define INT_HANDLED_UNMI
// #define INT_HANDLED_SYSNMI

/**
 * If interrupt vectors are left unassigned and are called, the CPU will reset.
 *
 * This function catches un-handled interrupts to reduce confusing resets
 * during debugging. If your application handles certain interrupts, comment
 * them out here to solve linker placement errors.
 */
#ifndef INT_HANDLED_RTC
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt void UNHANDLED_ISR_RTC(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) UNHANDLED_ISR_RTC(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_PORT2
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void UNHANDLED_ISR_PORT2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) UNHANDLED_ISR_PORT2(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_TIMER2_A1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A1_VECTOR
__interrupt void UNHANDLED_ISR_TIMER2_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A1_VECTOR))) UNHANDLED_ISR_TIMER2_A1(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifdef BOARD_EDB
#ifndef INT_HANDLED_TIMER2_A0
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER2_A0_VECTOR
__interrupt void UNHANDLED_ISR_TIMER2_A0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER2_A0_VECTOR))) UNHANDLED_ISR_TIMER2_A0(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif
#endif // BOARD_EDB

#ifdef BOARD_EDB
#ifndef INT_HANDLED_USCI_B1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B1_VECTOR
__interrupt void UNHANDLED_ISR_USCI_B1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B1_VECTOR))) UNHANDLED_ISR_USCI_B1(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif
#endif // BOARD_EDB

#ifndef INT_HANDLED_USCI_A1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void UNHANDLED_ISR_USCI_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) UNHANDLED_ISR_USCI_A1(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_PORT1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void UNHANDLED_ISR_PORT1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) UNHANDLED_ISR_PORT1(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_TIMER1_A1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt void UNHANDLED_ISR_TIMER1_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A1_VECTOR))) UNHANDLED_ISR_TIMER1_A1(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_TIMER1_A0
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void UNHANDLED_ISR_TIMER1_A0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) UNHANDLED_ISR_TIMER1_A0(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_DMA
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=DMA_VECTOR
__interrupt void UNHANDLED_ISR_DMA(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(DMA_VECTOR))) UNHANDLED_ISR_DMA(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_TIMER0_A1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void UNHANDLED_ISR_TIMER0_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) UNHANDLED_ISR_TIMER0_A1(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_TIMER0_A0
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void UNHANDLED_ISR_TIMER0_A0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) UNHANDLED_ISR_TIMER0_A0(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_ADC12
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt void UNHANDLED_ISR_ADC12(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) UNHANDLED_ISR_ADC12(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_USCI_B0
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt void UNHANDLED_ISR_USCI_B0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) UNHANDLED_ISR_USCI_B0(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_USCI_A0
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void UNHANDLED_ISR_USCI_A0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) UNHANDLED_ISR_USCI_A0(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_WDT
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void UNHANDLED_ISR_WDT(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) UNHANDLED_ISR_WDT(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifdef BOARD_EDB
#ifndef INT_HANDLED_TIMER0_B1
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt void UNHANDLED_ISR_TIMER0_B1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) UNHANDLED_ISR_TIMER0_B1(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif
#endif // BOARD_EDB

#ifdef BOARD_EDB
#ifndef INT_HANDLED_TIMER0_B0
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B0_VECTOR
__interrupt void UNHANDLED_ISR_TIMER0_B0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B0_VECTOR))) UNHANDLED_ISR_TIMER0_B0(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif
#endif // BOARD_EDB

#ifndef INT_HANDLED_COMP_B
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=COMP_B_VECTOR
__interrupt void UNHANDLED_ISR_COMP_B(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(COMP_B_VECTOR))) UNHANDLED_ISR_COMP_B(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_UNMI
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=UNMI_VECTOR
__interrupt void UNHANDLED_ISR_UNMI(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(UNMI_VECTOR))) UNHANDLED_ISR_UNMI(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif

#ifndef INT_HANDLED_SYSNMI
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=SYSNMI_VECTOR
__interrupt void UNHANDLED_ISR_SYSNMI(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(SYSNMI_VECTOR))) UNHANDLED_ISR_SYSNMI(void)
#else
#error Compiler not supported!
#endif
{
    ASSERT(ASSERT_UNHANDLED_INTERRUPT, 0);
}
#endif
