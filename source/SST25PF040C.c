#include <SST25PF040C.h>

/****************************************************************
 *  Header File: SST25PF040C.h
 *  Description: SST25PF040C flash with HERCULES TMS570LS07 library source
 *  Date       : 31.03.2021
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
  *
 **************************************************************/
spiDAT1_t dataconfig4_t;
uint16 TX_DATA[100];
uint16 RX_DATA[100];

uint16 busyMask = 0;

uint16 mfg_id = 0;
uint16 dev_type;
uint16 dev_id;
uint16_t dev_identification;

/*************************************************************
 * Function Definitions

**************************************************************/

void Read_JEDEK(){
    uint16_t Command=READ_JEDEK;

    TX_DATA[0]=0x9F;
    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    gioSetBit(spiPORT1, 1U, 1);
    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    spiReceiveData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&mfg_id);
    spiReceiveData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&dev_type);
    spiReceiveData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&dev_id);
    gioSetBit(spiPORT1, 1U, 1);
}

void Read_Id(){
    uint16_t Command=READ_ID;
    uint16_t dummy=DUMMY;

    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    TX_DATA[0]=0x00;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&dummy);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&dummy);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&dummy);
    spiReceiveData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&dev_identification);
    gioSetBit(spiPORT1, 1U, 1);
}


uint16 ReadFlashStatus(void) {
    uint16_t StatusRegister;

    uint16_t Command=READ_STATUS;
    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    spiReceiveData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&StatusRegister);
    gioSetBit(spiPORT1, 1U, 1);

    return StatusRegister;
}

uint16 checkBusy(void) {
    uint16_t StatusRegister[8];

    uint16_t Command=READ_STATUS;
    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    spiReceiveData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&StatusRegister);
    gioSetBit(spiPORT1, 1U, 1);
    return StatusRegister[0];
}

void WriteEnable(){
    uint16_t Command=WRITE_ENABLE;

    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    gioSetBit(spiPORT1, 1U, 1);
    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    gioSetBit(spiPORT1, 1U, 1);
}

void WriteDisable(){
    uint16_t Command=WRITE_DISABLE;

    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    gioSetBit(spiPORT1, 1U, 1);
    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    gioSetBit(spiPORT1, 1U, 1);
}

void ChipErase(void) {
    WriteEnable();
    uint16_t Command=CHIP_ERASE;

    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    gioSetBit(spiPORT1, 1U, 1);
    while (checkBusy() | busyMask);
    //or
    //while (checkBusy() | busyMask);


}

void Write(uint32_t Adress, uint16_t value){
    uint16_t Adress1= Adress>>16;
    uint16_t Adress2= (Adress>>8)& 0x00FF;
    uint16_t Adress3= Adress & 0x00FF;

    uint16_t Command=WRITE;

    WriteEnable();
    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    Command=Adress1;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    Command=Adress2;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    Command=Adress3;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    Command=value;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    gioSetBit(spiPORT1, 1U, 1);
    while (checkBusy() | busyMask);

    WriteDisable();
    while (checkBusy() | busyMask);
    //or
    //while (checkBusy() | busyMask);

}

uint16 Read(uint32_t Adress){
    uint16_t Adress1= Adress>>16;
    uint16_t Adress2= (Adress>>8)& 0x00FF;
    uint16_t Adress3= Adress & 0x00FF;

    uint16_t Command = READ;
    uint16_t value ;

    gioSetBit(spiPORT1, 1U, 0);
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    Command=Adress1;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    Command=Adress2;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);
    Command=Adress3;
    spiTransmitData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&Command);

    spiReceiveData(spiREG1, &dataconfig4_t, 1, (uint16_t*)&value);
    gioSetBit(spiPORT1, 1U, 1);

    while (checkBusy() | busyMask);
    //or
    //while (checkBusy() | busyMask);

    return value;
}
