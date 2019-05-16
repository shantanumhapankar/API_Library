/*
 * WDT.c
 *
 *  Created on: Oct 18, 2018
 *      Author: Shantanu
 */

#include "msp430f5529.h"
#include "WDT.h"

// Should be called at the start of your main to avoid accidentally generating a reset during config.
void wdtHold(void) {
    /* Hold the watchdog */
    WDTCTL = WDTPW + WDTHOLD;
}

void wdtStart(void) {
    WDTCTL = WDTPW + ~(WDTHOLD);
}

void wdtInit(int interval) {
    WDTCTL = WDTHOLD + interval;
}

void wdtResetTimer(void) {
    // Set Counter Clear bit
    WDTCTL = WDTPW + WDTCNTCL;
}
