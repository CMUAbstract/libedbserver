/*******************************************************************************
 * @file
 * @date            26 March 2015
 * @author          Graham Harvey
 * @brief           Unified Clock System functions
 ******************************************************************************/

#include <msp430.h>
#include <stdint.h>
#include "pin_assign.h"
#include "ucs.h"
#include "config.h"

#define DCO_FREQ_RANGE_BITS_INNER(r) DCORSEL_ ## r;
#define DCO_FREQ_RANGE_BITS(r) DCO_FREQ_RANGE_BITS_INNER(r)

#define FLL_D_BITS_INNER(d) FLLD_ ## d
#define FLL_D_BITS(d) FLL_D_BITS_INNER(d)

void UCS_setup()
{
    // Increase Vcore setting to level3 to support fsystem=25MHz
    // NOTE: Change core voltage one level at a time.
    SetVcoreUp(0x01);
    SetVcoreUp(0x02);
    SetVcoreUp(0x03);

    // NOTE: The MCU starts in a fault condition, because ACLK is set to XT1 LF but
    // XT1 LF takes time to initialize. Its init begins when XT1 pin function
    // is selected. The fault flag for this clock source (and for DCO which
    // depends on it) and the "wildcard" osc fault flag OFIFG are set
    // and cannot be cleared until the init is complete (they bounce back on
    // if cleared before the init is completed).

    SFRIE1 &= OFIE; // ignore oscillator faults while we enable the oscillators

    // Go through each oscillator (REFO, XT1, XT2, DCO) and init each if necessary
    // and choose it as the source for the requested clocks

    // Oscillator: REFO

#if defined(CONFIG_DCO_REF_SOURCE_REFO)
    // already initialized on reset
    UCSCTL3 |= SELREF__REFOCLK;                  // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA__REFOCLK;                   // Set ACLK = REFO
#endif // CONFIG_DCO_REF_CLOCK_REFO

    // Oscillator: XT1 crystal

    // Need XT1 for both XT1 and XT2 DCO ref configs since ACLK is sourced from XT1
#if defined(CONFIG_DCO_REF_SOURCE_XT1) || defined(CONFIG_DCO_REF_SOURCE_XT2)
    // Enable XT1 by configuring its pins
    UCSCTL6 &= ~(XCAP1 | XCAP0);
    UCSCTL6 |= CONFIG_XT1_CAP_BITS;
    P5SEL |= BIT4 | BIT5;

    // The following are already the default, but include for clarity
    UCSCTL3 |= SELREF__XT1CLK; // select XT1 as the DCO reference
    UCSCTL4 |= SELA__XT1CLK;  // select ST1 as the source for ACLK

    // wait for XT1 to init and clear the fault flags
    while (UCSCTL7 & XT1LFOFFG)
        UCSCTL7 &= ~XT1LFOFFG;

#else
    // Disable XT1 since it is unused (and we changed the DCO ref and ACLK source above)
    UCSCTL6 |= XT1OFF;
    UCSCTL7 &= ~XT1LFOFFG; // at reset XT1 was selected and faulted (see note at the top)
#endif

    // Oscillator: XT2 crystal

#if defined(CONFIG_DCO_REF_SOURCE_XT2) || defined(CONFIG_CLOCK_SOURCE_XT2)
    // First part of enabling XT2: configure its pins (nothing happens yet)
    P5SEL |= BIT2 | BIT3;

    // Second part of enabling XT2: select it as a source
#if defined(CONFIG_DCO_REF_SOURCE_XT2)
    UCSCTL3 |= SELREF__XT2CLK; // TODO: UNTESTED
#endif
#if defined(CONFIG_CLOCK_SOURCE_XT2)
    // switch master clock (CPU) to XT2 (25 MHz) and clear fault flags
    UCSCTL4 |= SELM__XT2CLK | SELS__XT2CLK | SELA__XT2CLK;

    // Can't drive the UART with a 25 MHz clock (hang/reset), divide it
    //UCSCTL5 |= DIVS0 | DIVA0; // SMCLK, ACLK = 25 MHz / 2 = 12.5 MHz
#endif

    // Wait for the crystal to initialize by watching for fault flag to go away
    while (UCSCTL7 & XT2OFFG)
        UCSCTL7 &= ~XT2OFFG;
    SFRIFG1 &= ~OFIFG; // clear wildcard fault flag

#endif // CONFIG_DCO_REF_CLOCK_XT2 || CONFIG_CLOCK_SOURCE_XT2

    // Oscillator: DCO

    // DCO is on by default, we change its frequency if requested by config

#if defined(CONFIG_CLOCK_SOURCE_DCO)
    __bis_SR_register(SCG0);                    // Disable the FLL control loop
    UCSCTL3 |= CONFIG_FLL_REF_DIV;
    UCSCTL0 = 0x0000;                           // Set lowest possible DCOx, MODx
    UCSCTL1 = DCO_FREQ_RANGE_BITS(CONFIG_DCO_FREQ_R);    // Select DCO freq range
    UCSCTL2 = FLL_D_BITS(CONFIG_DCO_FREQ_D) | CONFIG_DCO_FREQ_N;
    __bic_SR_register(SCG0);                    // Enable the FLL control loop

    __delay_cycles(DCO_SETTLING_TIME);
#endif

    // Wait for DCO to stabilize (DCO on by default and we leave it on, so always do this)
    while (UCSCTL7 & DCOFFG)
        UCSCTL7 &= ~DCOFFG;

    // End sequence of oscillators

    SFRIFG1 &= ~OFIFG; // clear wildcard fault flag
    SFRIE1 |= OFIE; // watch for oscillator faults
}

static void SetVcoreUp (unsigned int level)
{
    // Open PMM registers for write
    PMMCTL0_H = PMMPW_H;
    // Set SVS/SVM high side new level
    SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
    // Set SVM low side to new level
    SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
    // Wait till SVM is settled
    while ((PMMIFG & SVSMLDLYIFG) == 0);
    // Clear already set flags
    PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
    // Set VCore to new level
    PMMCTL0_L = PMMCOREV0 * level;
    // Wait till new level reached
    if((PMMIFG & SVMLIFG))
        while((PMMIFG & SVMLVLRIFG) == 0);
    // Set SVS/SVM low side to new level
    SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
    // Lock PMM registers for write access
    PMMCTL0_H = 0x00;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=UNMI_VECTOR
__interrupt void unmi_isr(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(UNMI_VECTOR))) unmi_isr(void)
#else
#error Compiler not supported!
#endif
{
    GPIO(PORT_LED, OUT) &= ~BIT(PIN_LED_GREEN);
    GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
    if (UCSCTL7 & XT2OFFG)
        GPIO(PORT_LED, OUT) |= BIT(PIN_LED_GREEN);

    while (1);
}
