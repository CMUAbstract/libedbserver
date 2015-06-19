/*******************************************************************************
 * @file
 * @date            26 March 2015
 * @author          Graham Harvey
 * @brief           Unified Clock System functions
 ******************************************************************************/

#include <msp430.h>
#include "pin_assign.h"
#include "ucs.h"
#include "config.h"

// #define CLOCK_TEST_MODE

#ifdef CLOCK_TEST_MODE
// For debugging clock configurations
static void blink_loop() {
    unsigned i = 0;

    while (1) {
        GPIO(PORT_LED, OUT) ^= BIT(PIN_LED_GREEN);
        for (i = 0; i < 100; ++i)
            __delay_cycles(100000);
    }
}
#endif

#ifdef CONFIG_CLOCK_SOURCE_CRYSTAL
static void setClockSource()
{
    // In pin setup (main.c): P5SEL |= BIT2 | BIT3 | BIT4 | BIT5;

    // The MCU starts in a fault condition, because ACLK is set to XT1 LF but
    // XT1 LF takes time to initialize. Its init begins when XT1 pin function
    // is selected. The fault flag for this clock source (and for DCO which
    // depends on it) and the "wildcard" osc fault flag OFIFG are set
    // and cannot be cleared until the init is complete (they bounce back on
    // if cleared before the init is completed).

    // wait for default clock source to init and clear the fault flags
    while (UCSCTL7 & XT1LFOFFG || UCSCTL7 & DCOFFG)
        UCSCTL7 &= ~(XT1LFOFFG | DCOFFG);
    SFRIFG1 &= ~OFIFG; // clear wildcard fault flag (strangely, does not work without this)

    // At this point we are running in the default config:
    //      ACLK = XT1LFCLK (32.768 kHz); MCLK,SMCLK = DCOCLKDIV (1.048567 MHz)

    // switch master clock (CPU) to XT2 (25 MHz) and clear fault flags
    UCSCTL4 |= SELM__XT2CLK | SELS__XT2CLK | SELA__XT2CLK;
    while (UCSCTL7 & XT2OFFG)
        UCSCTL7 &= ~XT2OFFG;
    SFRIFG1 &= ~OFIFG; // clear wildcard fault flag

    SFRIE1 |= OFIE; // watch for oscillator faults

    // Can't drive the UART with a 25 MHz clock (hang/reset), divide it
    UCSCTL5 |= DIVS0 | DIVA0; // SMCLK, ACLK = 25 MHz / 2 = 12.5 MHz
}
#endif

#ifdef CONFIG_CLOCK_SOURCE_INTERNAL
static void setClockSource()
{
    UCSCTL3 = SELREF__REFOCLK;                  // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA__REFOCLK;                   // Set ACLK = REFO
    UCS_set21Mhz();
}
#endif

void UCS_setup()
{
    // Increase Vcore setting to level3 to support fsystem=25MHz
    // NOTE: Change core voltage one level at a time.
    SetVcoreUp(0x01);
    SetVcoreUp(0x02);
    SetVcoreUp(0x03);

    setClockSource();

#ifdef CLOCK_TEST_MODE
    blink_loop(); // to check clock configuration
#endif
}

void UCS_set21Mhz()
{
    __bis_SR_register(SCG0);                    // Disable the FLL control loop
    UCSCTL0 = 0x0000;                           // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_7;                        // Select DCO range 50MHz operation

    // MSP430 crashes if it runs too fast?
    // This may be caused by the average frequency from DCO modulation.  If we try to use
    // a faster clock, the FLL may adjust the DCO above 25MHz to produce a clock with that
    // average frequency.  If this happens, even for an instant, the MSP430 can crash.
    UCSCTL2 = FLLD_0 + 668;                     // Set DCO Multiplier
                                                // (N + 1) * FLLRef = Fdco
                                                // (668 + 1) * 32768 = 21921792 Hz (average)
                                                // Set FLL Div = fDCOCLK/1

    __bic_SR_register(SCG0);                    // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 25 MHz / 32,768 Hz = ~780k MCLK cycles for DCO to settle
    __delay_cycles(782000);

    // Loop until XT1, XT2 & DCO stabilize
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                 	// Clear fault flags
    } while(SFRIFG1 & OFIFG);             	// Test oscillator fault flag
}

void UCS_set16MHz()
{
    __bis_SR_register(SCG0);                // Disable the FLL control loop
    UCSCTL0 = 0x0000;                       // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_6;                    // Set DCO Multiplier for 32MHz
    UCSCTL2 = FLLD_1 + 499;                 // (N + 1) * FLLRef = Fdco
                                            // (499 + 1) * 32768 = 16MHz
                                            // Set FLL Div = fDCOCLK/2

    __bic_SR_register(SCG0);                // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 16MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
    __delay_cycles(500000);

    // XT1 is by default on as it is used default reference for the FLL - internal load caps?
    // Loop until XT1,XT2 & DCO stabilize
    do {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
                                          	// Clear XT2, XT1, DCO fault flags
        SFRIFG1 &= ~OFIFG;                	// Clear fault flags
    } while(SFRIFG1 & OFIFG);             	// Test oscillator fault flag

}

void UCS_set12MHz()
{
    __bis_SR_register(SCG0);                // Disable the FLL control loop
    UCSCTL0 = 0x0000;                       // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                    // Select DCO range 24MHz operation
    UCSCTL2 = FLLD_1 + 374;                 // Set DCO Multiplier for 12MHz
                                            // (N + 1) * FLLRef = Fdco
                                            // (374 + 1) * 32768 = 12MHz
                                            // Set FLL Div = fDCOCLK/2

    __bic_SR_register(SCG0);                // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 12 MHz / 32,768 Hz = 375000 = MCLK cycles for DCO to settle
    __delay_cycles(375000);

    // Loop until XT1,XT2 & DCO fault flag is cleared
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
        SFRIFG1 &= ~OFIFG;                	// Clear fault flags
    } while (SFRIFG1 & OFIFG);             	// Test oscillator fault flag
}

void UCS_set8MHz()
{
    __bis_SR_register(SCG0);                // Disable the FLL control loop
    UCSCTL0 = 0x0000;                       // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_5;                    // Select DCO range 16MHz operation
    UCSCTL2 = FLLD_1 + 249;                 // Set DCO Multiplier for 8MHz
                                            // (N + 1) * FLLRef = Fdco
                                            // (249 + 1) * 32768 = 8MHz

    __bic_SR_register(SCG0);                // Enable the FLL control loop

    // Worst-case settling time for the DCO when the DCO range bits have been
    // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
    // UG for optimization.
    // 32 x 32 x 8 MHz / 32,768 Hz = 250000 = MCLK cycles for DCO to settle
    __delay_cycles(250000);


    // Loop until XT1,XT2 & DCO stabilize - in this case only DCO has to stabilize
    do
    {
        UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2 ,XT1, DCO fault flags
        SFRIFG1 &= ~OFIFG;                	// Clear fault flags
    }while (SFRIFG1 & OFIFG);				// Test oscillator fault flag
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
    GPIO(PORT_LED, OUT) |= BIT(PIN_LED_RED);
    while (1);
}
