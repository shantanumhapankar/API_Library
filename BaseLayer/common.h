/*
 * common.h
 *
 *  Created on: Sep 26, 2018
 *      Author: Shantanu
 */

#ifndef COMMON_H_
#define COMMON_H_

/*
 * Function: pinClearSEL
 * ----------------------------
 *   Clears the pin's SEL register.
 *
 *   port: port number of the pin.
 *   pin: pin number whose SEL register is to be cleared.
 *
 *   returns: clears SEL register, returns nothing.
 */
void pinClearSEL(unsigned int port, int pin);

/*
 * Function: pinSetSEL
 * ----------------------------
 *   Sets the pin's SEL register.
 *
 *   port: port number of the pin.
 *   pin: pin number whose SEL register is to be set.
 *
 *   returns: sets SEL register, returns nothing.
 */
void pinSetSEL(unsigned int port, int pin);

/*
 * Function: pinSetREN
 * ----------------------------
 *   Sets the pin's REN register.
 *
 *   port: port number of the pin.
 *   pin: pin number whose REN register is to be set.
 *
 *   returns: sets REN register, returns nothing.
 */
void pinSetREN(unsigned int port, int pin);

/*
 * Function: pinClearREN
 * ----------------------------
 *   Clears the pin's REN register.
 *
 *   port: port number of the pin.
 *   pin: pin number whose REN register is to be cleared.
 *
 *   returns: clears REN register, returns nothing.
 */
void pinClearREN(unsigned int port, int pin);

/*
 * Function: pinSetPull
 * ----------------------------
 *   Sets the pin as a pullup or pulldown.
 *
 *   port: port number of the pin.
 *   pin: pin number whose pin is to be set as pullup or pulldown.
 *   mode: pullup or pulldown mode.
 *
 *   returns: set's pin pull based on the mode passed, returns nothing.
 */
void pinSetPull(unsigned int port, int pin, int mode);


/*
 * Function: pinSetOffsetSet
 * ----------------------------
 *   Sets the register offset for a particular port's pin.
 *
 *   port: port number of the pin.
 *   pin: pin number.
 *   offset: register offset to set.
 *
 *   returns: set's pin register based on the offset passed, returns nothing.
 */
void pinSetOffsetSet(unsigned int port, int pin, int offset);

/*
 * Function: pinSetOffsetClear
 * ----------------------------
 *   Clears the register offset for a particular port's pin.
 *
 *   port: port number of the pin.
 *   pin: pin number.
 *   offset: register offset to clear.
 *
 *   returns: set's pin register based on the offset passed, returns nothing.
 */
void pinSetOffsetClear(unsigned int port, int pin, int offset);

/*
 * Function: pinSetOffsetGet
 * ----------------------------
 *   Gets the register offset value for a particular port's pin.
 *
 *   port: port number of the pin.
 *   pin: pin number.
 *   offset: register offset to read the value of.
 *
 *   returns: The value of the register offset for the particular pin.
 */
unsigned int pinSetOffsetGet(unsigned int port, int pin, int offset);

#endif /* COMMON_H_ */
