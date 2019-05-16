/*
 * PWM.h
 *
 *  Created on: Nov 5, 2018
 *      Author: Shantanu
 */

#ifndef PWM_H_
#define PWM_H_

#define PERIOD 1000



/*
 * Function: pinSetPWM
 * ----------------------------
 *   Sets a pin in pwm mode.
 *
 *   port: port number of the pin.
 *   pin: pin number to set as pwm.
 *
 *   returns: sets SEL register to pwm. sets period, mode and clock source, returns nothing.
 */
void pinSetPWM(unsigned int port, int pin);

/*
 * Function: pwmSetDuty
 * ----------------------------
 *   Changes dutycycle.
 *
 *   dutycycle: dutycycle in %.
 *
 *   returns: sets TA0CCR1 register, returns nothing.
 */
void pwmSetDuty(int dutycycle);


#endif /* PWM_H_ */
