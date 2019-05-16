/******************************************************************************/
/*               University of North Carolina at Charlotte                    */
/*                           ECE Department                                   */
/*                                                                            */
/*                                                                            */
/* File Name:   common.c                                                      */
/* Module Name: Basic Software                                                */
/* Project: M.Sc. Thesis: Autonomous ATV Control                              */
/* File Version:  1.0       Created by: Shantanu M                            */
/* Base_line:     Initial_Release                                             */
/* Created:       Sep 26, 2018                                                */
/*                                                                            */
/*                                                                            */
/* Description:                                                               */
/*    Version 1.0:                                                            */
/*     - Clear and Set functions for SEL, SEL2 and REN registers              */
/*     - Set pin pullup or pulldown function                                  */
/*                                                                            */
/* References: http://www.argenox.com/library/msp430/                         */
/*                  general-purpose-input-output-gpio-chapter-5/              */
/******************************************************************************/

/*
 * common.c
 *
 *  Created on: Sep 26, 2018
 *      Author: Shantanu
 */

#include "pin_common.h"
#include "DIN.h"
#include "DOUT.h"
#include "common.h"

void pinClearSEL(unsigned int port, int pin) {
    pinSetOffsetClear(port, pin, GPIO_SEL_REG_OFFSET);
}

void pinSetSEL(unsigned int port, int pin) {
    pinSetOffsetSet(port, pin, GPIO_SEL_REG_OFFSET);
}

void pinSetOffsetSet(unsigned int port, int pin, int offset) {
    int portAddress = GPIO_PORT_ADD_TABLE[port];
    if (!(port & 1)) (*((volatile int *)(portAddress + offset))) |= pin;
    else (*((volatile int *)(portAddress + offset))) |= (pin << 8);
}

void pinSetOffsetClear(unsigned int port, int pin, int offset) {
    int portAddress = GPIO_PORT_ADD_TABLE[port];
    if (!(port & 1)) (*((volatile int *)(portAddress + offset)))  &= ~pin;
    else (*((volatile int *)(portAddress + offset))) &= ~(pin << 8);
}

unsigned int pinSetOffsetGet(unsigned int port, int pin, int offset) {
    int portAddress = GPIO_PORT_ADD_TABLE[port];
    if (!(port & 1)) return (*((volatile int *)(portAddress + offset))) & pin;
    else return (*((volatile int *)(portAddress + offset))) & (pin << 8);
}


void pinSetREN(unsigned int port, int pin) {
    pinSetOffsetSet(port, pin, GPIO_REN_REG_OFFSET);
}

void pinClearREN(unsigned int port, int pin) {
    pinSetOffsetClear(port, pin, GPIO_REN_REG_OFFSET);
}

//When the pin is in input mode and REN is enabled, P1OUT register selects whether the resistors is a pull-up (1) or pull-down (0)
void pinSetPull(unsigned int port, int pin, int mode) {
    pinSetIP(port, pin);
//    pinSetSEL2(port, pin);
    if (mode) gpioSetVal(port, pin, PIN_HIGH);
    else gpioSetVal(port, pin, PIN_LOW);
}
