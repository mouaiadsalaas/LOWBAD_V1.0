#include <PCA2129.h>

/****************************************************************
 *  Header File: PCA129.h
 *  Description: PCA129 RTC with HERCULES TMS570LS07 library source
 *  Date       : 26.03.2021
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
spiDAT1_t dataconfig3_t;

/*************************************************************
 * Function Definitions

**************************************************************/

/*
    January 0 0 0 0 1
    February 0 0 0 1 0
    March 0 0 0 1 1
    April 0 0 1 0 0
    May 0 0 1 0 1
    June 0 0 1 1 0
    July 0 0 1 1 1
    August 0 1 0 0 0
    September 0 1 0 0 1
    October 1 0 0 0 0
    November 1 0 0 0 1
    December 1 0 0 1 0
 */

/*
    Sunday       0 0 0
    Monday       0 0 1
    Tuesday      0 1 0
    Wednesday    0 1 1
    Thursday     1 0 0
    Friday       1 0 1
    Saturday     1 1 0
 */

//to use each function alone for set one thing
//        second_value  = Getseconds();
//        minutes_value = Getminutes();
//        hours_value   = Gethours();
//        day_value     = Getdays();
//        weekdays_value= Getweekdays();
//        months_value  = Getmonths();
//        years_value   = Getyears();

//incase using oneshot getdate
//        for ( i = 0; i < 7; i++ ) {
//           printf( "*(p + %d) : %d\n", i, bcdToDec(*(p + i)));
//        }

unsigned set_bit(unsigned x, unsigned offset, bool value)
{
    //dizi[5] = set_bit(dizi[5], 3, 1);

    return (value)
        ? x | (1 << offset)
        : x & ~(1 << offset);
}



uint16_t decToBcd(uint16_t value) {
  return (uint8_t) ( (value/10*16) + (value%10) );
}

uint16_t bcdToDec(uint16_t value) {
  return (uint8_t) ( (value/16*10) + (value%16) );
}

void SetRegister(uint16 registeradress,uint16 Value){
    uint16 Adress2Write = registeradress;
    uint16 AdressValue =Value;
    //read seconds
    gioSetBit(spiPORT1, 0U, 0);
    spiTransmitData(spiREG1, &dataconfig3_t, 1, (uint16_t*)&Adress2Write);
    spiTransmitData(spiREG1, &dataconfig3_t, 1, (uint16_t*)&AdressValue);
    gioSetBit(spiPORT1, 0U, 1);
}

uint16 GetRegister(uint16 registeradress){
    uint16 Adress2Read=registeradress;
    uint16 AdressValue;
    //read seconds
    gioSetBit(spiPORT1, 0U, 0);
    spiTransmitData(spiREG1, &dataconfig3_t, 1, (uint16_t*)&Adress2Read);
    spiReceiveData(spiREG1, &dataconfig3_t, 1, (uint16_t*)&AdressValue);
    RX_DATA[0] = set_bit(RX_DATA[0], 7, 0);//set osf bit as 0 always
    gioSetBit(spiPORT1, 0U, 1);
    return AdressValue;
}

uint16 Getseconds(){
    return bcdToDec(GetRegister(PCA2129_SECONDS_READ));
}

void Setseconds(uint16 setvalue){
    SetRegister(PCA2129_SECONDS_WRITE,(decToBcd(setvalue)+0x80));
}

uint16 Getminutes(){
    return bcdToDec(GetRegister(PCA2129_MINUTES_READ));
}

void Setminutes(uint16 setvalue){
    SetRegister(PCA2129_MINUTES_WRITE,decToBcd(setvalue));
}

uint16 Gethours(){
    return bcdToDec(GetRegister(PCA2129_HOURS_READ));
}

void Sethours(uint16 setvalue){
    SetRegister(PCA2129_HOURS_WRITE,decToBcd(setvalue));
}

uint16 Getdays(){
    return bcdToDec(GetRegister(PCA2129_DAYS_READ));
}

void Setdays(uint16 setvalue){
    SetRegister(PCA2129_DAYS_WRITE,decToBcd(setvalue));
}

uint16 Getweekdays(){
    return bcdToDec(GetRegister(PCA2129_WEEKDAYS_READ));
}

void Setweekdays(uint16 setvalue){
    SetRegister(PCA2129_WEEKDAYS_WRITE,decToBcd(setvalue));
}

uint16 Getmonths(){
    return bcdToDec(GetRegister(PCA2129_MONTHS_READ));
}

void Setmonths(uint16 setvalue){
    SetRegister(PCA2129_MONTHS_WRITE,decToBcd(setvalue));
}

uint16 Getyears(){
    return bcdToDec(GetRegister(PCA2129_YEARS_READ));
}

void Setyears(uint16 setvalue){
    SetRegister(PCA2129_YEARS_WRITE,decToBcd(setvalue));
}

uint16* GetDate(){
    static uint16 Date[7];
    Date[0] = Getseconds();
    Date[1] = Getminutes();
    Date[2] = Gethours();
    Date[3] = Getdays();
    Date[4] = Getweekdays();
    Date[5] = Getmonths();
    Date[6] = Getyears();
    return Date;
}

//uint16* GetDateoneshot(){
//    static uint16 Date[7];
//    uint16 Adress2Read = PCA2129_SECONDS_READ;
//    TX_DATA[0]=0xA3;
//    gioSetBit(spiPORT1, 0U, 0);
//    spiTransmitData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Adress2Read);
//    spiReceiveData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Date[0]);
//    spiReceiveData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Date[1]);
//    spiReceiveData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Date[2]);
//    spiReceiveData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Date[3]);
//    spiReceiveData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Date[4]);
//    spiReceiveData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Date[5]);
//    spiReceiveData(spiREG1, &dataconfig1_t, 1, (uint16_t*)&Date[6]);
//    Date[0] = set_bit(RX_DATA[0], 7, 0);//set osf bit as 0 always
//    gioSetBit(spiPORT1, 0U, 1);
//    return Date;
//}

void SetDate(uint16 second,uint16 minute,uint16 hour,uint16 day,uint16 weekday,uint16 month,uint16 year){
    Setseconds(second);
    Setminutes(minute);
    Sethours(hour);
    Setdays(day);
    Setweekdays(weekday);
    Setmonths(month);
    Setyears(year);
}

