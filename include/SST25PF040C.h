/****************************************************************
 *  Header File: SST25PF040C.h
 *  Description: SST25PF040C FLash with HERCULES TMS570ls07 library header
 *  Date       : 31.03.2021
 *  Author     : MOUAIAD SALAAS
 *     ---------- ---------- ----------------------------
 *
 * Copyright MOUAIAD SALAAS , 2021
 *
 * This unpublished material is proprietary to MOUAIAD SALAAS.
 * All rights reserved. The methods and
 * techniques described herein are considered trade secrets
 * and/or confidential. Reproduction or distribution, in whole
 * or in part, is forbidden except by express written permission
 * of MOUAIAD SALAAS.
 ****************************************************************/
/**************************************************************
 * Notes: in this library CS (slave select pin has been set as SPIPORT! cs1)
 *
 **************************************************************/
#ifndef __SST25PF040C_H_
#define __SST25PF040C_H_
/**************************************************************
 * Include c libraries
 **************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <stddef.h>
#include <strings.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/**************************************************************
 * Include TMS570ls4032 libraries
 **************************************************************/
#include "spi.h"
#include "gio.h"

/**************************************************************
 * Global Definitions
 **************************************************************/
#define DUMMY           0X00
#define WRITE           0x02
#define READ            0x03
#define READ_JEDEK      0x9F
#define READ_ID         0xAB
#define CHIP_ERASE      0xC7
#define WRITE_ENABLE    0x06
#define WRITE_DISABLE   0x04
#define READ_STATUS     0x05

spiDAT1_t dataconfig4_t;



/**************************************************************
 *Function Prototypes
 **************************************************************/
void Read_JEDEK();
void Read_Id();
uint16 ReadFlashStatus(void);
uint16 checkBusy(void);
void WriteEnable();
void WriteDisable();
void ChipErase(void);
void Write(uint16_t Adress1, uint16_t Adress2, uint16_t Adress3, uint16_t value);
uint16 Read(uint16_t Adress1, uint16_t Adress2, uint16_t Adress3);


#endif /* __SST25PF040C_H_ */
