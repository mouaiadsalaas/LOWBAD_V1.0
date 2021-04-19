#include <ADS1018.h>

/****************************************************************
 *  Header File: ADS1018.h
 *  Description: ADS1018 Analog to digital convetrter with HERCULES TMS570ls0732 library header source
 *  Date       : 14.04.2021
 *  Author     : MOUAIAD SALAAS
 *     ---------- ---------- ----------------------------
 *
 * Copyright MOUAIAD SALAAS, 2021
 *
 * This unpublished material is proprietary to MOUAIAD SALAAS.
 * All rights reserved. The methods and
 * techniques described herein are considered trade secrets
 * and/or confidential. Reproduction or distribution, in whole
 * or in part, is forbidden except by express written permission
 * of MOUAIAD SALAAS.
 **************************************************************/
/**************************************************************
 * Notes:
 *
 **************************************************************/
/**************************************************************
  * Global Variables
 **************************************************************/

volatile uint32_t beginTime;
volatile uint32_t timeOut;

uint32_t step=0;
uint32_t functiontodo =0;
volatile int8_t subStep  = 0;
volatile int8_t failStep = 0;

volatile int8_t currentStep   = 0;
volatile int8_t successStep   = 0;

uint16_t rcvId[] = { 0xC58B, 0xD58B, 0xE58B, 0xF58B };

uint16 global_counter;
uint16 ADS1018Data[4];
/*************************************************************
 * Function Definitions

**************************************************************/

void setStep( int sub, int current, int success, uint32_t time, uint32_t begin ){
    subStep = sub;
    currentStep = current;
    successStep = success;
    timeOut = time;
    beginTime = begin;
}

 void Register2write(void){
     switch (functiontodo) {
        case 0:
            spiTransmitData(spiREG3, &dataconfig2_t, 1, (uint16_t*)&rcvId[0]);
            break;

        case 1:
            spiTransmitData(spiREG3, &dataconfig2_t, 1, (uint16_t*)&rcvId[1]);
            break;

        case 2:
            spiTransmitData(spiREG3, &dataconfig2_t, 1, (uint16_t*)&rcvId[2]);
            break;

        case 3:
            spiTransmitData(spiREG3, &dataconfig2_t, 1, (uint16_t*)&rcvId[3]);
            break;

        default:
            break;
    }

 }

void ADS1018Adcread( void ){

    switch( subStep ){

        case STEP_0:
            setStep(STEP_WAIT, STEP_0, STEP_1, 1, global_counter);
            functiontodo = 0;
            break;

        case STEP_1:
            setStep(STEP_WAIT, STEP_1, STEP_2, 1, global_counter);
            functiontodo = 1;
            break;

        case STEP_2:
           setStep(STEP_WAIT, STEP_2, STEP_3,  1, global_counter);
           functiontodo = 2;
           break;

        case STEP_3:
            setStep(STEP_WAIT, STEP_3, STEP_GPRS_CONNECTED, 1, global_counter);
            functiontodo = 3;
            break;


        case STEP_GPRS_CONNECTED:
            subStep = STEP_0;
            break;


        case STEP_WAIT:

            if( global_counter - beginTime > timeOut ){
                    spiReceiveData(spiREG3,  &dataconfig2_t, 1, (uint16_t*)&ADS1018Data[currentStep]);
                    spiReceiveData(spiREG3,  &dataconfig2_t, 1, (uint16_t*)&ADS1018Data[currentStep]);
                    ADS1018Data[currentStep] = (ADS1018Data[currentStep] /8);
                    subStep = successStep;
            }else{
                    Register2write();
            }

    }
}

