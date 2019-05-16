/*
 * SPI.h
 *
 *  Created on: Sep 16, 2018
 *      Author: Shantanu
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

/*
 * Function: spiMasterInit
 * ----------------------------
 *  Calls spiMasterPinInit and spiMasterConfigureCR for initialization.
 *  Resets SS pin.
 *
 *   returns: void.
 */
void spiMasterInit(void);

/*
 * Function: spiSlaveInit
 * ----------------------------
 *  Calls spiSlavePinInit and spiSlaveConfigureCR for initialization.
 *
 *   returns: void.
 */
void spiSlaveInit(void);

/*
 * Function: spiMasterPinInit
 * ----------------------------
 *  Sets SEL & SEL2 to USCI mode for MISO, MOSI & CLK. Sets SS pin to low.
 *
 *   returns: void.
 */
void spiMasterPinInit(void);

/*
 * Function: spiSlavePinInit
 * ----------------------------
 *  Sets SEL & SEL2 to USCI mode for MISO, MOSI & CLK.
 *
 *   returns: void.
 */
void spiSlavePinInit(void);

/*
 * Function: spiMasterConfigureCR
 * ----------------------------
 *  Configures the SPI control registers to 3-pin, 8-bit SPI master, sync by clock; enable interrupts.
 *
 *   returns: void.
 */
void spiMasterConfigureCR(void);

/*
 * Function: spiSlaveConfigureCR
 * ----------------------------
 *  Configures the SPI control registers to 3-pin, 8-bit SPI slave, sync by clock; enable interrupts.
 *
 *   returns: void.
 */
void spiSlaveConfigureCR(void);
/*
 * Function: spiSelectSlave
 * ----------------------------
 *  Sets the SS pin to low to select the slave.
 *
 *   returns: void.
 */
void spiSelectSlave(void);

/*
 * Function: spiDeselectSlave
 * ----------------------------
 *  Sets the SS pin to high to deselect the slave.
 *
 *   returns: void.
 */
void spiDeselectSlave(void);

/*
 * Function: spiTransmitData
 * ----------------------------
 *   Checks if TX buffer is ready and then sets the UCA0TXBUF register with the data to send.
 *
 *   data: character to send via spi.
 *
 *   returns: void.
 */
uint8_t spiTransmitData(char data);

/*
 * Function: spiCheckTxReady
 * ----------------------------
 *   Polls the UCA0TXIFG flag for interrupt.
 *
 *   returns: void.
 */
void spiCheckTxReady(void);

/*
 * Function: spiReceiveData
 * ----------------------------
 *   Gets the received character from UCA0RXBUF.
 *
 *   returns: data received.
 */
uint8_t spiReceiveData(void);

/*
 * Function: spiCheckUSCI
 * ----------------------------
 *   Checks the UCA0STAT status register to make sure TX is complete.
 *
 *   returns: void
 */
void spiCheckUSCI(void);

/*
 * Function: spiCheckRxReady
 * ----------------------------
 *   Polls the UCA0RXIFG flag for interrupt.
 *
 *   returns: void.
 */
void spiCheckRxReady(void);

#endif /* SPI_H_ */
