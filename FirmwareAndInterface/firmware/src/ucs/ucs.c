/*******************************************************************************
 * @file
 * @date            26 March 2015
 * @author          Graham Harvey
 * @brief           Unified Clock System functions
 ******************************************************************************/

#include <msp430.h>
#include "monitor.h"
#include "ucs.h"

void UCS_setup()
{
    // Increase Vcore setting to level3 to support fsystem=25MHz
    // NOTE: Change core voltage one level at a time.
    SetVcoreUp(0x01);
    SetVcoreUp(0x02);
    SetVcoreUp(0x03);

    UCSCTL3 = SELREF__REFOCLK;                  // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA__REFOCLK;                   // Set ACLK = REFO

    UCS_setMainFreq();
}

void UCS_setMainFreq()
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
