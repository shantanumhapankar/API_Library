/*
 * pin_common.h
 *
 *  Created on: Feb 25, 2019
 *      Author: Shantanu
 *      References:http://www.ti.com/lit/ds/symlink/msp430f5529.pdf
 */

#ifndef PINCOMMON_H
#define PINCOMMON_H

/************************************************************
* GPIO VALUES
************************************************************/
#define GPIO_HIGH                                                     (1)
#define GPIO_LOW                                                      (0)

/************************************************************
* PIN VALUES
************************************************************/
#define PIN_HIGH                                                      (1)
#define PIN_LOW                                                       (0)

/************************************************************
* GPIO PORTS
************************************************************/
#define GPIO_PORT_P1                                                   (0)
#define GPIO_PORT_P2                                                   (1)
#define GPIO_PORT_P3                                                   (2)
#define GPIO_PORT_P4                                                   (3)
#define GPIO_PORT_P5                                                   (4)
#define GPIO_PORT_P6                                                   (5)
#define GPIO_PORT_P7                                                   (6)
#define GPIO_PORT_P8                                                   (7)

/************************************************************
* GPIO PINS
************************************************************/
#define GPIO_PIN0                                                      (0x0001)
#define GPIO_PIN1                                                      (0x0002)
#define GPIO_PIN2                                                      (0x0004)
#define GPIO_PIN3                                                      (0x0008)
#define GPIO_PIN4                                                      (0x0010)
#define GPIO_PIN5                                                      (0x0020)
#define GPIO_PIN6                                                      (0x0040)
#define GPIO_PIN7                                                      (0x0080)

/************************************************************
* GPIO DIGITAL I/O REGISTERS
************************************************************/
#define GPIO_IN_REG_OFFSET                                             (0x00)
#define GPIO_OUT_REG_OFFSET                                            (0x02)
#define GPIO_DIR_REG_OFFSET                                            (0x04)
#define GPIO_REN_REG_OFFSET                                            (0x06)
#define GPIO_DS_REG_OFFSET                                             (0x08)
#define GPIO_SEL_REG_OFFSET                                            (0x0A)
#define GPIO_IV_REG_OFFSET                                             (0x0E)
#define GPIO_IES_REG_OFFSET                                            (0x18)
#define GPIO_IE_REG_OFFSET                                             (0x1A)
#define GPIO_IFG_REG_OFFSET                                            (0x1C)

/************************************************************
* msp430f5529 PORT BASE ADDRESS
************************************************************/
#define GPIO_BASE_PORT1                                                (0x200)
#define GPIO_BASE_PORT2                                                (0x200)
#define GPIO_BASE_PORT3                                                (0x220)
#define GPIO_BASE_PORT4                                                (0x220)
#define GPIO_BASE_PORT5                                                (0x240)
#define GPIO_BASE_PORT6                                                (0x240)
#define GPIO_BASE_PORT7                                                (0x260)
#define GPIO_BASE_PORT8                                                (0x260)

/************************************************************
* msp430f5529 PORT ADDRESS TABLE MAPPING TABLE
************************************************************/
static const int GPIO_PORT_ADD_TABLE[] = { GPIO_BASE_PORT1,
                                           GPIO_BASE_PORT2,
                                           GPIO_BASE_PORT3,
                                           GPIO_BASE_PORT4,
                                           GPIO_BASE_PORT5,
                                           GPIO_BASE_PORT6,
                                           GPIO_BASE_PORT7,
                                           GPIO_BASE_PORT8 };

/************************************************************
* Structure to define Pin
************************************************************/
struct PinDescriptor {
    unsigned int port;
    unsigned int pin;
};

#endif
