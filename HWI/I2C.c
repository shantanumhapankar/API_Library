/******************************************************************************/
/*               University of North Carolina at Charlotte                    */
/*                           ECE Department                                   */
/*                                                                            */
/*                                                                            */
/* File Name:   I2C.c                                                         */
/* Module Name: Basic Software                                                */
/* Project: M.Sc. Thesis: Autonomous ATV Control                              */
/* File Version:  1.0       Created by: Shantanu M                            */
/* Base_line:     Initial_Release                                             */
/* Created:       Feb 13, 2018                                                */
/*                                                                            */
/*                                                                            */
/* Description:                                                               */
/*    Version 1.0:         Created by: Shantanu M                             */
/*     - Configure Control Registers.                                         */
/*     - Configure Pins for I2C.                                              */
/*     - Read and Write function.                                             */
/*     - Sending start and stop bits of the I2C frame                         */
/*    Version 2.0:         Created by: Shantanu M                             */
/*                                                                            */
/* Reference: www.ti.com/lit/an/slaa382a/slaa382a.pdf                         */
/******************************************************************************/

#include "msp430.h"
#include "../BaseLayer/common.h"
#include "../BaseLayer/pin_common.h"
#include "I2C.h"

const struct PinDescriptor SDA = {GPIO_PORT_P4, GPIO_PIN1};
const struct PinDescriptor SCL  = {GPIO_PORT_P4, GPIO_PIN2};


void i2cInit(uint8_t add) {
    UCB1CTL1 &= ~UCSWRST;
    i2cConfigureCR(add);
}

void i2cPinInit(void) {
    // Set SEL to 1 for USCI mode.
    pinSetSEL(SDA.port, SDA.pin | SCL.pin);
}

void i2cConfigureCR(uint8_t add) {

    // Make sure there's no I2C stop flag set.
    while (UCB1CTL1 & UCTXSTT);

    // Configure pins to use USCI
    i2cPinInit();

    //USCI Configuration
    UCB1CTL1 |= UCSWRST;                           // Enable SW reset
    UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;          // I2C Master, synchronous mode
    UCB1CTL1 = UCSSEL_2 + UCSWRST;                 // Use SMCLK, keep SW reset
    UCB1I2CSA = add;                               // Setting slave address.

    // Configure the baud rate registers
    UCB1BR0 = 12;                                  // fSCL = SMCLK/10 = ~100kHz
    UCB1BR1 = 0;

    UCB1CTL1 &= ~UCSWRST;                          // Clear SW reset, resume operation
}

void i2cSendStartTx(void) {

    // Send the start condition in transmit mode.
    UCB1CTL1 |= UCTR | UCTXSTT;
    // Wait for the start condition to be sent and ready to transmit interrupt.
    while ((UCB1CTL1 & UCTXSTT) && (!(UCB1IFG & UCTXIFG)));
}

void i2cSendStartRx(void) {
    // Send the start condition in receive mode.
    UCB1CTL1 &= ~UCTR;
    UCB1CTL1 |= UCTXSTT;
    // Wait for the start condition to be sent.
    while (UCB1CTL1 & UCTXSTT);
}

void i2cSendStop(void) {
    // Send stop condition
    UCB1CTL1 |= UCTXSTP;
    while (UCB0CTL1 & UCTXSTP);
}

void i2cWrite(uint8_t data) {
    // Write to the TX buf.
    UCB1TXBUF = data;
    // Poll for transmit interrupt flag.
    while(!(UCB1IFG & UCTXIFG));
}


uint8_t i2cRead(void) {
    // Wait till there's something to read.
    while (!(UCB1IFG & UCRXIFG));
    return UCB1RXBUF;
}
