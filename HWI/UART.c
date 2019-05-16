/*
 * UART.c
 *
 *  Created on: Feb 4, 2019
 *      Author: Shantanu
 */

#include "msp430.h"
#include "../BaseLayer/common.h"
#include "../BaseLayer/pin_common.h"
#include "UART.h"

const struct PinDescriptor TX  = {GPIO_PORT_P3, GPIO_PIN3};
const struct PinDescriptor RX = {GPIO_PORT_P3, GPIO_PIN4};

void uartInit(void) {
    uartPinInit();
    uartConfigureCR();
}

void uartPinInit(void) {
    pinSetSEL(RX.port, RX.pin | TX.pin);
}

void uartConfigureCR(void) {
    UCA0CTL1 |= UCSWRST;         // Disable USCI, reset mode
    UCA0CTL1 |= UCSSEL_2;        // SMCLK
    UCA0BR0 = 6;                              // 1MHz 9600 (see User's Guide)
    UCA0BR1 = 0;                              // 1MHz 9600
    UCA0MCTL = UCBRS_0 + UCBRF_13 + UCOS16;   // Modln UCBRSx=0, UCBRFx=0,
    UCA0CTL1 &= ~UCSWRST;        // Initialize USCI state machine
}

void uartCheckTxReady(void) {
    while (!(UCA0IFG & UCTXIFG)); // USCI_A0 TX buffer ready?
}

void uartCheckRxReady(void) {
    while (!(UCA0IFG & UCRXIFG)); // USCI_A0 RX Received?
}

uint8_t uartReceiveChar(void) {
    uartCheckRxReady();
    return UCA0RXBUF;
}

void uartTransmitChar(uint8_t data) {
    uartCheckTxReady();
    UCA0TXBUF = data;
}