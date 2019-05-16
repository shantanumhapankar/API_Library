/*
 * I2C.h
 *
 *  Created on: Feb 13, 2019
 *      Author: Shantanu
 */

#ifndef I2C_H_
#define I2C_H_

#include "stdint.h"

/*
 * Function: i2cConfigureCR
 * ----------------------------
 *   Configure the control registers to function as I2C as master and set the clock.
 *
 *   add: address of the slave to communicate with.
 */
void i2cConfigureCR(uint8_t add);

/*
 * Function: i2cInit
 * ----------------------------
 *   Initialize the pins and control registers
 *
 *   add: address of the slave to communicate with.
 */
void i2cInit(uint8_t add);

/*
 * Function: i2cRead
 * ----------------------------
 *   Reads the RXBUF to get data sent from slave.
 *
 *   returns: value of RXBUF read.
 */
uint8_t i2cRead(void);

/*
 * Function: i2cWrite
 * ----------------------------
 *   Write to the TXBUF
 *
 *   data: data to write to the slave
 */
void i2cWrite(uint8_t data);

/*
 * Function: i2cSendStartTx
 * ----------------------------
 *   Put the master in Tx mode and send start condition.
 */
void i2cSendStartTx(void);

/*
 * Function: i2cSendStartRx
 * ----------------------------
 *   Put the master in Rx mode and send start condition.
 */
void i2cSendStartRx(void);

/*
 * Function: i2cSendStop
 * ----------------------------
 *   Sends stop condition to stop I2C communication
 */
void i2cSendStop(void);

#endif /* I2C_H_ */
