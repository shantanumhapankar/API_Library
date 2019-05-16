/*
 * DOUT.h
 *
 *  Created on: Sep 14, 2018
 *      Author: Shantanu
 */

#ifndef DOUT_H_
#define DOUT_H_

/*
 * Function: pinSetOP
 * ----------------------------
 *   Sets a pin in output mode.
 *
 *   port: port number of the pin.
 *   pin: pin number to set as output.
 *
 *   returns: sets direction register to output, returns nothing.
 */
void pinSetOP(unsigned int port, int pin);

/*
 * Function: gpioSetOP
 * ----------------------------
 *   Sets a pin in output GPIO mode.
 *
 *   port: port number of the pin.
 *   pin: pin number to set as output.
 *
 *   returns: sets direction register to output & select register to GPIO, returns nothing.
 */
void gpioSetOP(unsigned int port, int pin);

/*
 * Function: gpioSetVal
 * ----------------------------
 *   Sets an output pin as high or low depending on the value.
 *
 *   port: port number of the pin.
 *   pin: pin number to set high.
 *   val: value to set at the pin.
 *
 *   returns: sets output register depending on the val argument, returns nothing.
 */
void gpioSetVal(unsigned int port, int pin, int val);


#endif /* DOUT_H_ */
