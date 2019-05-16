/******************************************************************************/
/*               University of North Carolina at Charlotte                    */
/*                           ECE Department                                   */
/*                                                                            */
/*                                                                            */
/* File Name:   SPI.c                                                         */
/* Module Name: Basic Software                                                */
/* Project: M.Sc. Thesis: Autonomous ATV Control                              */
/* File Version:  2.1       Created by: Shantanu M                            */
/* Base_line:     Initial_Release                                             */
/* Created:       Sep 15, 2018                                                */
/*                                                                            */
/*                                                                            */
/* Description:                                                               */
/*    Version 1.0:         Created by: Shantanu M                             */
/*     - Configure Control Registers.                                         */
/*     - Configure Pins for SPI.                                              */
/*     - Receive and Transmit function.                                       */
/*     - Check if buffers are ready functions.                                */
/*    Version 2.0:         Created by: Shantanu M                             */
/*     - Separate configuration functions for Master and Slave.               */
/*    Version 2.1:         Created by: Karim H Erian                          */
/*    - Changed the Clock to 1.1 MHz by changing  UCA0BR0 to 1                */
/*    - Changed the TX function to read the RX register to avoid overrun error*/
/*    Version 2.2:         Created by: Shantanu M                             */
/*    - Changed SPI USCI to B as UART runs on A.                              */
/*                                                                            */
/* Ref:http://www.argenox.com/library/msp430/msp430-spi-peripheral-chapter-9/ */
/*     http://www.electrodummies.net/de/msp430-spi-tutorial/                  */
/******************************************************************************/

#include "SPI.h"
#include "../BaseLayer/DOUT.h"
#include "../BaseLayer/common.h"
#include "../BaseLayer/pin_common.h"
#include "msp430f.h"

/*
 *  P3.1 – UCA0SOMI
 *  P3.0 – UCA0SIMO
 *  P3.2 – UCA0CLK
 *  P2.7 - UCB0SS
 */
const struct PinDescriptor MISO = {GPIO_PORT_P3, GPIO_PIN1};
const struct PinDescriptor MOSI = {GPIO_PORT_P3, GPIO_PIN0};
const struct PinDescriptor CLK  = {GPIO_PORT_P3, GPIO_PIN2};
const struct PinDescriptor SS  = {GPIO_PORT_P2, GPIO_PIN7};


void spiMasterInit(void) {
    spiMasterPinInit();
    spiMasterConfigureCR();
    // Reset SS. Is this needed?
    spiSelectSlave();
    spiDeselectSlave();
}

void spiSlaveInit(void) {
    spiSlavePinInit();
    spiSlaveConfigureCR();
}

uint8_t spiTransmitData(char data) {
    spiCheckTxReady();
    UCB0TXBUF = data;
    while(UCB0STAT & UCBUSY);
    return UCB0RXBUF;
}

uint8_t spiReceiveData(void) {
    while(UCB0STAT & UCBUSY); // added to wait for busy flag
    return UCB0RXBUF;
}

void spiSelectSlave(void) {
    gpioSetVal(SS.port, SS.pin, PIN_LOW);
}

void spiDeselectSlave(void) {
    gpioSetVal(SS.port, SS.pin, PIN_HIGH);
}

void spiCheckTxReady(void) {
    while (!(UCB0IFG & UCTXIFG)); // USCI_A0 TX buffer ready?
}

void spiCheckRxReady(void) {
    while (!(UCB0IFG & UCRXIFG)); // USCI_A0 RX Received?
}

void spiMasterPinInit(void) {
    gpioSetVal(SS.port, SS.pin, PIN_LOW);
    pinSetOP(SS.port, SS.pin);
    // Set SEL 1 for USCI mode.
    pinSetSEL(MISO.port, MISO.pin | MOSI.pin | CLK.pin);
}

void spiSlavePinInit(void) {
    // Set SEL to 1 for USCI mode.
    pinSetSEL(MISO.port, MISO.pin | MOSI.pin | CLK.pin);
}

void spiMasterConfigureCR(void) {
    //Configuring the SPI control registers:
    UCB0CTL1 |= UCSWRST;                            // Disable USCI, reset mode.
    UCB0CTL0 |= UCCKPL + UCMSB + UCMST + UCSYNC;    // 3-pin, 8-bit SPI master, sync by clock
    UCB0CTL1 |= UCSSEL_2;                           // SMCLK
    UCB0BR0 |= 0x01;                                // Clock prescalar, Low Byte. Clock/1 (was 2)
    UCB0BR1 = 0;                                    // Higher Byte
//    UCB0MCTL = 0;                                   // No modulation
    UCB0CTL1 &= ~UCSWRST;                           // Initialize USCI state machine

    // Uncomment the the following line and call __enable_interrupt() in main if you want it interrupt based.
    //    IE2 |= UCA0RXIE;                                // Enable USCIA0 RX interrupt will be received on USCIAB0RX_VECTOR
}

void spiSlaveConfigureCR(void) {
    //Configuring the SPI control registers:
    UCB0CTL1 = UCSWRST;                            // Disable USCI, reset mode.
    UCB0CTL0 |= UCCKPL + UCMSB + UCSYNC;            // 3-pin, 8-bit SPI slave, sync by clock
    UCB0CTL1 &= ~UCSWRST;                           // **Initialize USCI state machine**

    // Uncomment the the following line and call __enable_interrupt() in main if you want it interrupt based.
    //    IE2 |= UCA0RXIE;                                // Enable USCIA0 RX interrupt will be received on USCIAB0RX_VECTOR
}
