/*
 * WDT.h
 *
 *  Created on: Oct 18, 2018
 *      Author: Shantanu
 */

#ifndef WDT_H_
#define WDT_H_

/*
 * Function: wdtHold
 * ----------------------------
 *   Holds the watchdog timer. Should be called at the start of your main to avoid generating a wdt reset if you aren't using it.
 */
void wdtHold(void);

/*
 * Function: wdtStart
 * ----------------------------
 *   Starts the watchdog timer by clearing the hold.
 */
void wdtStart(void);

/*
 * Function: wdtInit
 * ----------------------------
 *   Initializes the watchdog timer but doesn't start it. Call wdtStart() to start it.
 *
 *   interval: time interval. The board specific time interval is present in their header file. Example: WDT_ADLY_1000 for a 1 second wdt in msp430f2559
 *
 *   returns: sets time interval; returns nothing.
 */
void wdtInit(int interval);

/*
 * Function: wdtResetTimer
 * ----------------------------
 *	Resets the watchdog counter to prevent reset.
 */
void wdtResetTimer(void);


#endif /* WDT_H_ */
