/****************************************************************
 *  Header File: MCP23S17.h
 *  Description: MCP23S17 expander with HERCULES TMS570ls0732 library header source
 *  Date       : 18.02.2021
 *  Author     : MOUAIAD SALAAS & ALI YOLCU
 *     ---------- ---------- ----------------------------
 *
 * Copyright MOUAIAD SALAAS & ALI YOLCU , 2021
 *
 * This unpublished material is proprietary to MOUAIAD SALAAS.
 * All rights reserved. The methods and
 * techniques described herein are considered trade secrets
 * and/or confidential. Reproduction or distribution, in whole
 * or in part, is forbidden except by express written permission
 * of MOUAIAD SALAAS.
 ****************************************************************/
/**************************************************************
 * Notes: in this library CS (slave select pin has been set as GIOA3 you can change it like you want from Global Definitions section)
 *
 **************************************************************/
#ifndef __MCP23S17_H
#define __MCP23S17_H
/**************************************************************
 * Include c libraries
 **************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/**************************************************************
 * Include TMS570ls4032 libraries
 **************************************************************/
#include "spi.h"
#include "gio.h"

spiDAT1_t dataconfig1_t;

/**************************************************************
 * Global Definitions
 **************************************************************/
#define HIGH 1
#define LOW 0

#define BTT2_IN1_PORT   gioPORTA
#define BTT2_IN1_PIN    2U

#define BTT1_IN1_PORT   gioPORTA
#define BTT1_IN1_PIN    3U

#define SPI_CS_PORT     spiPORT2
#define SPI_CS_PIN      0U

#define NUM_SSI_DATA    3

#define    IODIRA    (0x00)      // MCP23x17 I/O Direction Register
#define    IODIRB    (0x01)      // 1 = Input (default), 0 = Output

#define    IPOLA     (0x02)      // MCP23x17 Input Polarity Register
#define    IPOLB     (0x03)      // 0 = Normal (default)(low reads as 0), 1 = Inverted (low reads as 1)

#define    GPINTENA  (0x04)      // MCP23x17 Interrupt on Change Pin Assignements
#define    GPINTENB  (0x05)      // 0 = No Interrupt on Change (default), 1 = Interrupt on Change

#define    DEFVALA   (0x06)      // MCP23x17 Default Compare Register for Interrupt on Change
#define    DEFVALB   (0x07)      // Opposite of what is here will trigger an interrupt (default = 0)

#define    INTCONA   (0x08)      // MCP23x17 Interrupt on Change Control Register
#define    INTCONB   (0x09)      // 1 = pin is compared to DEFVAL, 0 = pin is compared to previous state (default)

#define    IOCON     (0x0A)      // MCP23x17 Configuration Register
//                   (0x0B)      //     Also Configuration Register

#define    GPPUA     (0x0C)      // MCP23x17 Weak Pull-Up Resistor Register
#define    GPPUB     (0x0D)      // INPUT ONLY: 0 = No Internal 100k Pull-Up (default) 1 = Internal 100k Pull-Up

#define    INTFA     (0x0E)      // MCP23x17 Interrupt Flag Register
#define    INTFB     (0x0F)      // READ ONLY: 1 = This Pin Triggered the Interrupt

#define    INTCAPA   (0x10)      // MCP23x17 Interrupt Captured Value for Port Register
#define    INTCAPB   (0x11)      // READ ONLY: State of the Pin at the Time the Interrupt Occurred

#define    GPIOA     (0x12)      // MCP23x17 GPIO Port Register 0x12
#define    GPIOB     (0x13)      // Value on the Port - Writing Sets Bits in the Output Latch

#define    OLATA     (0x14)      // MCP23x17 Output Latch Register
#define    OLATB     (0x15)      // 1 = Latch High, 0 = Latch Low (default) Reading Returns Latch State, Not Port Value!


#define    OPCODEW       (0b01000000)  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3
#define    ADDR_ENABLE   (0b00001000)  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing

#define    address        0

#define    ADDR_OPCODEW  (OPCODEW | (address << 1))
#define    ADDR_OPCODER  (OPCODER | (address << 1))

#define    ADDR_reg       reg
#define    ADDR_word      word

#define    MCP_GPA0         1
#define    MCP_GPA1         2
#define    MCP_GPA2         3
#define    MCP_GPA3         4
#define    MCP_GPA4         5
#define    MCP_GPA5         6
#define    MCP_GPA6         7
#define    MCP_GPA7         8


#define    MCP_GPB0         9
#define    MCP_GPB1         10
#define    MCP_GPB2         11
#define    MCP_GPB3         12
#define    MCP_GPB4         13
#define    MCP_GPB5         14
#define    MCP_GPB6         15
#define    MCP_GPB7         16
/**************************************************************
 *Function Prototypes
 **************************************************************/
void spiWrite( uint16_t data );
uint8_t spiWriteRead( uint16_t txdata);

void wordWrite(uint8_t reg, unsigned int word);
void byteWrite(uint8_t reg, uint8_t value);

void MCP_Init(uint8_t SPI_PORT_NO, uint8_t CS);

void PortMode(unsigned int mode);
void MCP_PortpullupMode(unsigned int mode);
void MCP_PortinputInvert(unsigned int mode);
unsigned int MCP_PortRead(void);

void MCP_PinMode(uint8_t pin, uint8_t mode);
void MCP_PinOutput(uint8_t pin, uint8_t value);
uint8_t MCP_pinRead(uint8_t pin);



#endif  /* __MCP23S17_H */
