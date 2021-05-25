/****************************************************************
 *  Header File: PINOUT.h
 *  Description: our controlunit pinout header file
 *  Date       : 1.04.2021
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
 * Notes: in this header file pins name have been set according to our board and IC places in it
 *
 **************************************************************/

#ifndef INCLUDE_PINOUT_H_
#define INCLUDE_PINOUT_H_

/**************************************************************
 * Global Definitions
 **************************************************************/

#define HIGH        1
#define LOW         0

/********************************************************4 channel POWER SWITCHE(LEFT)***************************************************************/
#define VNQ5050KTRE_LEFT_INT1_PIN               10
#define VNQ5050KTRE_LEFT_INT1_PORT              hetPORT1

#define VNQ5050KTRE_LEFT_INT2_PIN               12
#define VNQ5050KTRE_LEFT_INT2_PORT              hetPORT1

#define VNQ5050KTRE_LEFT_INT3_PIN               14
#define VNQ5050KTRE_LEFT_INT3_PORT              hetPORT1

#define VNQ5050KTRE_LEFT_INT4_PIN               16
#define VNQ5050KTRE_LEFT_INT4_PORT              hetPORT1

#define VNQ5050KTRE_LEFT_STAT_DIS_PIN           18
#define VNQ5050KTRE_LEFT_STAT_DIS_PORT          hetPORT1

uint8_t VNQ5050KTRE_LEFT_1STAT1_VAL;             //ADIN[11]
uint8_t VNQ5050KTRE_LEFT_1STAT2_VAL;             //AIDN[3]
uint8_t VNQ5050KTRE_LEFT_1STAT3_VAL;             //ADIN[2]
uint8_t VNQ5050KTRE_LEFT_1STAT4_VAL;             //ADIN[10]

/********************************************************4 channel POWER SWITCHE(RIGHT)***************************************************************/
#define VNQ5050KTRE_RIGHT_INT1_PIN              1
#define VNQ5050KTRE_RIGHT_INT1_PORT             gioPORTA

#define VNQ5050KTRE_RIGHT_INT2_PIN              0
#define VNQ5050KTRE_RIGHT_INT2_PORT             gioPORTA

#define VNQ5050KTRE_RIGHT_INT3_PIN              24
#define VNQ5050KTRE_RIGHT_INT3_PORT             hetPORT1

#define VNQ5050KTRE_RIGHT_INT4_PIN              8
#define VNQ5050KTRE_RIGHT_INT4_PORT             hetPORT1

#define VNQ5050KTRE_RIGHT_STAT_DIS_PIN          18
#define VNQ5050KTRE_RIGHT_STAT_DIS_PORT         hetPORT1

uint8_t VNQ5050KTRE_RIGHT_2STAT1_VAL;           //ADIN[8]
uint8_t VNQ5050KTRE_RIGHT_2STAT2_VAL;           //ADIN[6]
uint8_t VNQ5050KTRE_RIGHT_2STAT3_VAL;           //ADIN[5]
uint8_t VNQ5050KTRE_RIGHT_2STAT4_VAL;           //ADIN[4]

/********************************************************2 channel POWER SWITCHE(LEFT)***************************************************************/
#define VND5T100AJTRE_LEFT_INT1_PIN            2
#define VND5T100AJTRE_LEFT_INT1_PORT           hetPORT1

#define VND5T100AJTRE_LEFT_INT2_PIN            5
#define VND5T100AJTRE_LEFT_INT2_PORT           gioPORTA

#define VND5T100AJTRE_LEFT_FRSTBY_PIN          22           //PIN TO DO FAULT RESET
#define VND5T100AJTRE_LEFT_FRSTBY_PORT         hetPORT1

uint8_t VNQ5050KTRE_LEFT_1CS1_VAL;             //ADIN[9]
uint8_t VNQ5050KTRE_LEFT_1CS2_VAL;             //ADIN[1]

/********************************************************2 channel POWER SWITCHE(RIGHT)***************************************************************/
#define VND5T100AJTRE_RIGHT_INT1_PIN            6
#define VND5T100AJTRE_RIGHT_INT1_PORT           gioPORTA

#define VND5T100AJTRE_RIGHT_INT2_PIN            7
#define VND5T100AJTRE_RIGHT_INT2_PORT           gioPORTA

#define VND5T100AJTRE_RIGHT_FRSTBY_PIN          22           //PIN TO DO FAULT RESET
#define VND5T100AJTRE_RIGHT_FRSTBY_PORT         hetPORT1

uint8_t VNQ5050KTRE_RIGHT_2CS1_VAL;             //ADIN[0]
uint8_t VNQ5050KTRE_RIGHT_2CS2_VAL;             //ADIN[7]

/********************************************************1 channel POWER SWITCHE(BOTTOM CENTER)***************************************************************/
#define VN5T006ASPTRE_BOTTOM_INT1_PIN            0
#define VN5T006ASPTRE_BOTTOM_INT1_PORT           hetPORT1

#define VN5T006ASPTRE_BOTTOM_FRSTBY_PIN          22           //PIN TO DO FAULT RESET
#define VN5T006ASPTRE_BOTTOM_FRSTBY_PORT         hetPORT1

uint8_t VN5T006ASPTRE_BOTTOM_CS1_VAL;            //ADIN[21]

/********************************************************1 channel POWER SWITCHE(BOTTOM RIGHT)***************************************************************/
#define NCV8403ADTRKG_RIGHT_INT1_PIN            4
#define NCV8403ADTRKG_RIGHT_INT1_PORT           hetPORT1

/********************************************************1 channel POWER SWITCHE(BOTTOM LEFT)***************************************************************/
#define NCV8403ADTRKG_LEFT_INT1_PIN            6
#define NCV8403ADTRKG_LEFT_INT1_PORT           hetPORT1

#endif /* INCLUDE_PINOUT_H_ */
