/******************************************************************************/
/*               University of North Carolina at Charlotte                    */
/*                           ECE Department                                   */
/*                                                                            */
/*                                                                            */
/* File Name:   DOUT.c                                                        */
/* Module Name: Basic Software                                                */
/* Project: M.Sc. Thesis: Autonomous ATV Control                              */
/* File Version:  1.0       Created by: Shantanu M                            */
/* Base_line:     Initial_Release                                             */
/* Created:       Feb 25, 2019                                                */
/*                                                                            */
/*                                                                            */
/* Description:                                                               */
/*    Version 1.0:                                                            */
/*     - GPIO set output function                                             */
/*     - GPIO set value function                                              */
/*     - Pin set output function                                              */
/*                                                                            */
/* References:http://www.ocfreaks.com/msp430-gpio-programming-tutorial/       */
/******************************************************************************/

/*
 * DOUT.c
 *
 *  Created on: Sep 14, 2018
 *      Author: Shantanu
 */

#include "pin_common.h"
#include "DOUT.h"
#include "common.h"

void pinSetOP(unsigned int port, int pin) {
    pinSetOffsetSet(port, pin, GPIO_DIR_REG_OFFSET);
}

void gpioSetOP(unsigned int port, int pin) {
    pinSetOP(port, pin);
    pinClearSEL(port, pin);
}

void gpioSetVal(unsigned int port, int pin, int val) {
    if (val) pinSetOffsetSet(port, pin, GPIO_OUT_REG_OFFSET);
    else pinSetOffsetClear(port, pin, GPIO_OUT_REG_OFFSET);
}
