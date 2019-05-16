/*
 * DIN.h
 *
 *  Created on: Sep 14, 2018
 *      Author: Shantanu
 */

#ifndef DIN_H_
#define DIN_H_


/*
 * Function: pinSetIP
 * ----------------------------
 *   Sets a pin in input mode.
 *
 *   port: port number of the pin.
 *   pin: pin number to set as input.
 *
 *   returns: sets direction register to input, returns nothing.
 */
void pinSetIP(unsigned int port, int pin);

/*
 * Function: gpioSetIP
 * ----------------------------
 *   Sets a pin in gpio input mode.
 *
 *   port: port number of the pin.
 *   pin: pin number to set as input.
 *
 *   returns: sets direction register to input & select register to GPIO, returns nothing.
 */
void gpioSetIP(unsigned int port, int pin);

/*
 * Function: gpioGetVal
 * ----------------------------
 *   Gets the value of a pin.
 *
 *   port: port number of the pin.
 *   pin: pin number to set as input.
 *
 *   returns: state of the input pin(High or Low)
 */
int gpioGetVal(unsigned int port, int pin);

#endif /* DIN_H_ */
