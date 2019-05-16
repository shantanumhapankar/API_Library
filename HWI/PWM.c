/******************************************************************************/
/*               University of North Carolina at Charlotte                    */
/*                           ECE Department                                   */
/*                                                                            */
/*                                                                            */
/* File Name:   PWM.c                                                         */
/* Module Name: Basic Software                                                */
/* Project: M.Sc. Thesis: Autonomous ATV Control                              */
/* File Version:  1.0       Created by: Shantanu M                            */
/* Base_line:     Initial_Release                                             */
/* Created:       Nov 5, 2018                                                 */
/*                                                                            */
/*                                                                            */
/* Description:                                                               */
/*    Version 1.0:                                                            */
/*     - Pwm set duty cycle function                                          */
/*     - Pin set pwm function                                                 */
/*    Version 1.1:                                                            */
/*     - Changed timer to TA1                                                 */
/*                                                                            */
/* References:https://www.kompulsa.com/example-code-msp430-pwm/               */
/******************************************************************************/

/*
 * PWM.c
 *
 *  Created on: Nov 5, 2018
 *      Author: Shantanu
 */
#include "PWM.h"
#include "../BaseLayer/DOUT.h"
#include "msp430.h"
#include "../BaseLayer/common.h"


// Built for TA0. Supports PWM on pins P1.1-P1.5
void pinSetPWM(unsigned int port, int pin) {
    pinSetOP(port, pin);
    // set the P1SEL as 1 and P1SEL2 as 0 selecting the Primary peripheral module function.
    pinSetSEL(port, pin);
    //Set the period in the Timer A0 Capture/Compare 0 register to 1000 us.
    TA0CCR0 = PERIOD;
    TA0CCTL1 = OUTMOD_7;
    TA0CCTL1 = OUTMOD_7;
    // Start with 0% duty cycle.
    TA0CCR1 = 0;
    //TASSEL_2 selects SMCLK as the clock source, and MC_1 tells it to count up to the value in TA0CCR0.
    TA0CTL = TASSEL_2 + MC_1;
}

void pwmSetDuty(unsigned int dutycycle) {
    //dutycycle = (CCR1/CCR0)*100
    TA0CCR1 = (dutycycle * PERIOD)/100;
}
