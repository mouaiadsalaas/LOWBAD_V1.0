/****************************************************************
 *  Header File: PCA2129.h
 *  Description: PCA2129 RTC with HERCULES TMS570ls07 library header 
 *  Date       : 26.03.2021
 *  Author     : MOUAIAD SALAAS & ALI YOLCU
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
 * Notes: in this library CS (slave select pin has been set as GIOA3 you can change it like you want from Global Definitions section)
 *
 **************************************************************/
#ifndef __PCA2129_H
#define __PCA2129_H
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

spiDAT1_t dataconfig3_t;

/**************************************************************
 * Global Definitions
 **************************************************************/

#define PCA2129_CONTROL_REGISTERS_READ  0xA0

#define PCA2129_SECONDS_READ            0xA3
#define PCA2129_MINUTES_READ            0xA4
#define PCA2129_HOURS_READ              0xA5
#define PCA2129_DAYS_READ               0xA6
#define PCA2129_WEEKDAYS_READ           0xA7
#define PCA2129_MONTHS_READ             0xA8
#define PCA2129_YEARS_READ              0xA9

#define PCA2129_SECONDS_WRITE           0x23
#define PCA2129_MINUTES_WRITE           0x24
#define PCA2129_HOURS_WRITE             0x25
#define PCA2129_DAYS_WRITE              0x26
#define PCA2129_WEEKDAYS_WRITE          0x27
#define PCA2129_MONTHS_WRITE            0x28
#define PCA2129_YEARS_WRITE             0x29

spiDAT1_t dataconfig1_t;
uint16 TX_DATA[100];
uint16 RX_DATA[100];

uint16 second_value;
uint16 minutes_value;
uint16 hours_value;
uint16 day_value;
uint16 weekdays_value;
uint16 months_value;
uint16 years_value;




/**************************************************************
 *Function Prototypes
 **************************************************************/
unsigned set_bit(unsigned x, unsigned offset, bool value);
uint16_t decToBcd(uint16_t value);
uint16_t bcdToDec(uint16_t value);
void SetRegister(uint16 registeradress,uint16 Value);
uint16 GetRegister(uint16 registeradress);
uint16 Getseconds();
void Setseconds(uint16 setvalue);
uint16 Getminutes();
void Setminutes(uint16 setvalue);
uint16 Gethours();
void Sethours(uint16 setvalue);
uint16 Getdays();
void Setdays(uint16 setvalue);
uint16 Getweekdays();
void Setweekdays(uint16 setvalue);
uint16 Getmonths();
void Setmonths(uint16 setvalue);
uint16 Getyears();
void Setyears(uint16 setvalue);
uint16* GetDate();
void SetDate(uint16 second,uint16 minute,uint16 hour,uint16 day,uint16 weekday,uint16 month,uint16 year);

#endif  /* __PCA2129_H */
