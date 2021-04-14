/****************************************************************
 *  Header File: ADS1018.h
 *  Description: ADS1018 Analog to digital convetrter with HERCULES TMS570ls0732 library header source
 *  Date       : 14.04.2021
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
 * Notes:
 *
 **************************************************************/
#ifndef __ADS1018_H
#define __ADS1018_H
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

spiDAT1_t dataconfig2_t;

/**************************************************************
 * Global Definitions
 **************************************************************/
#define ADS_AIN0      0xC28B
#define ADS_AIN1      0xD28B
#define ADS_AIN2      0xE28B
#define ADS_AIN3      0xF28B

extern uint16 global_counter;
extern uint32 ADS1018Data[4];

enum substeps{

    STEP_0 = 0,
    STEP_1,
    STEP_2,
    STEP_3,
    STEP_WAIT,
    STEP_GPRS_CONNECTED
};



/**************************************************************
 *Function Prototypes
 **************************************************************/
void setStep( int sub, int current, int success, uint32_t time, uint32_t begin );
void Register2write(void);
void ADS1018Adcread( void );


#endif  /* __ADS1018_H */
