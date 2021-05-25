#include "sys_common.h"
#include "system.h"

#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <stddef.h>
#include <strings.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "rti.h"
#include "esm.h"
#include "gio.h"
#include "het.h"
#include "adc.h"
#include "spi.h"
#include "can.h"
#include "ti_fee.h"

#include "PINOUT.h"
#include "LOWBAD_IO_PIN.h"
#include "MCP23S17.h"
#include "ADS1018.h"
#include "PCA2129.h"
#include "SST25PF040C.h"
//NOTES:
/*1.OUR USED ADIN PINS ARE:
 *                        ADIN[0],ADIN[1],
 *                        ADIN[3],ADIN[4],
 *                        ADIN[5],ADIN[6],
 *                        ADIN[7],ADIN[8],
 *                        ADIN[9],ADIN[10],
 *                        ADIN[11],ADIN[21]
 *2.OUR NOT USED ADIN PINS ARE:
 *                        ADIN[16],ADIN[17]
 *                        ADIN[20]
 *
 *3.OUR USED SPI DRIVERS ARE  :
 *                        SPI1,SPI2,SPI3
 *4.SPI1 CONNECTED DEVICE IS  : PCA2129 Real Time Clock and SST25PF040C Flash Memory
 *5.SPI2 CONNECTED DEVICE IS  : MCP23S18  GIO  EXPANDER
 *6.SPI3 CONNECTED DEVICE IS  : ADS1018 ANALOG EXPANDER
 *
 *
 */

uint8 PumpSlaveBtn = 0;
bool semi_Automatic_AutoAlignment = false;

bool ISSActiveState      = false;
bool AutoAligmentState   = false;
bool AlignmentRightState = false;
bool AlignmentLeftState  = false;
bool AlignmentAutoState  = false;

bool AutoHizalama   = false;
bool AcilStopON     = false;
bool SysActive      = false;
bool AlignmentRight = false;
bool AlignmentLeft  = false;
bool AlignmentAuto  = false;
bool fullPumpState  = false;


//FULLAUTOMATIC
uint16_t Amount;
uint16_t Average;
uint16_t adc_samplecounter;

uint8_t aligment_buf;
uint16_t setcounter = 0;

bool SetMode  = false;
bool StopMode = false;
bool ISS      = false;

bool emergency_stop = false;
bool auto_aligment  = false;
bool left_aligment  = false;
bool right_aligment = false;
bool set_aligment   = false;
bool StopModeState  = false;

bool StatusReady2Evaluate  = false;

bool full_Automatic_AutoAlignment = false;
bool full_Automatic_Set_Aligment  = false;

bool CalibrationDone    = false;
bool SystemActiveCaliber = true;

bool tick  = false;
bool tick2 = false;

bool Fullautomaticmode = false;
bool Semiautomaticmode = false;

int CamelneckHomeTemporary;
int FifthWheelHomeTemporary;

int CamelneckRightAngleTemporary;
int FifthWheelRightAngleTemporary;

int CamelneckLeftAngleTemporary;
int FifthWheelLeftAngleTemporary;

float CamelneckNumberTemporary;
float FifthWheelNumberTemporary;

int FifthWheelNumberValue =0;
int CamelneckNumberValue =0;

typedef struct timer{
    uint16_t set;
    uint16_t count;
    bool state;
    uint16_t counter;
}Timers;

Timers Fullautomaticled;
Timers Semiautomaticled;

Timers ModeSelectDelay;
Timers Calibrationbegin;
Timers CalibrationFirstTemprorySettings;
Timers CalibrationSecondTemprorySettings;
Timers CalibrationFinalSettings;
Timers CalibrationCancel;
Timers CalibrationFailure;


Timers FifthWheelAnglePositioneLeft;
Timers FifthWheelAnglePositioneRight;
Timers VehicleInRoute;

Timers AutomaticAligmentDone;
Timers ISSSingalActive;
Timers FrontSensorInoperative;
Timers BackSensorInoperative;

Timers EmergencyStopActive;

Timers StatusValueGet;

typedef enum delays{

    DELAY_250MS = 250,
    DELAY_500MS = 500,
    DELAY_1S = 1000

}Delays;

typedef enum botuns{

    NOTHING_PRESSED        = 0,
    AlignmentRight_PRESSED = 1,
    AlignmentLeft_PRESSED  = 2,
    AlignmentAuto_PRESSED  = 4,
    AlignmentSet_PRESSED   = 8,
    AcilStopON_PRESSED     = 16,
    AlignmentSave_PRESSED   = 12

}Botuns;

typedef struct E2PROM_DATA
{
    int CamelneckHome;
    int CamelneckRightAngle;
    int CamelneckLeftAngle;
    int FifthWheelHome;
    int FifthWheelRightAngle;
    int FifthWheelLeftAngle;

    float CamelneckNumber;
    float FifthWheelNumber;
}eepromdata;

//struct E2PROM_KAYIT e2prom_write_value = {1822,2424,1220,1953,2714,1192,6.69,20.2};
eepromdata eeprom_read_data = {1,1,1,1,1,1,1,1};

uint16_t AttractiveRightSensor; // Sensor 1
uint16_t AttractiveLeftSensorAndCamelNeck; // Sensor 2

/**************************************************************************/
uint16_t canSensorData;
uint8_t aligment_set_level = 0;
bool PumpState = false;

//adc
adcData_t adc_data[16];
uint64 ch_count=0;
uint32 id    =0;
uint64 value =0;

//canbus
#define  D_SIZE 8
uint8  can_tx_data[D_SIZE];
uint8  can_rx_data[D_SIZE] = {0};
uint32 error = 0;

uint8 sensorRight;
uint8 sensorLeft;


//PCA129
uint16 *Dateinfo;

//SST25PF040C
uint16_t adress_value[100] = {0};


//eeprom
uint16 u16JobResult,Status;
Std_ReturnType oResult=E_OK;

unsigned int BlockNumber;
uint8 SpecialRamBlock[16];
uint8 eeprom_write_data[16];

unsigned int BlockOffset, Length;
unsigned char read_data[100]={0};


unsigned char pattern;
uint16 u16writecounter;

unsigned int  FeeVirtualSectorNumber;
unsigned char VsState, u8EEPIndex;
unsigned char u8VirtualSector;
uint8 Test_Recovery;
uint8 Test_Cancel;

int AciSonuc =0;
float DegerSonuc =0;
float AngleValue =0;

int mode;
uint8_t GIO_OIL_PUMP_PIN_STATUS = 0 ;
uint8_t GIO_STATUS_VALUE = 0 ;
uint8_t mycounter = 0;
uint8_t pump_mycounter = 0;
uint32_t log_start_adress   = 0x000002;
bool keep_log_flag = false;
bool keep_log_open_counter = false;
bool keep_log_normal_counter = false;
bool keep_log_short_counter = false;

bool pump_keep_log_flag = false;
bool pump_keep_log_open_counter = false;
bool pump_keep_log_normal_counter = false;
bool pump_keep_log_short_counter = false;
bool check_flag = false;
bool pump_check_again = true;
void delay(void)
{
    unsigned int dummycnt=0x0000FFU;
    do
    {
        dummycnt--;
    }
    while(dummycnt>0);
}


uint32 checkPackets(uint8 *src_packet,uint8 *dst_packet,uint32 psize);

/*--------------------------------------------------------------------------RTIInit---------------------------------------------------------*/
void RTIInit(){

    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    _enable_IRQ();
    rtiStartCounter(rtiCOUNTER_BLOCK0);

    rtiEnableNotification(rtiNOTIFICATION_COMPARE1);
    _enable_IRQ();
    rtiStartCounter(rtiCOUNTER_BLOCK0);
}
/*------------------------------------------------------------------------HardwareInit------------------------------------------------------*/
void HardwareInit(){
    rtiInit();
    gioInit();

    //gioSetDirection(hetPORT1, 0xFFFFFFFF);

    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    rtiStartCounter(rtiCOUNTER_BLOCK0);
    //_enable_IRQ();


    rtiEnableNotification(rtiNOTIFICATION_COMPARE1);
    rtiStartCounter(rtiCOUNTER_BLOCK1);
    _enable_IRQ();


    hetInit();
    adcInit();
    spiInit();
    canInit();
}
/*------------------------------------------------------------------------SPIMCPInit--------------------------------------------------------*/
void SPIMCPInit(){
    dataconfig1_t.CS_HOLD = TRUE;
    dataconfig1_t.WDEL    = FALSE;
    dataconfig1_t.DFSEL   = SPI_FMT_0;
    dataconfig1_t.CSNR    = 0xFE;

    _enable_IRQ();
}
/*-----------------------------------------------------------------------SPIADS1018Init------------------------------------------------------*/
void SPIADS1018Init(){
    dataconfig2_t.CS_HOLD = TRUE;
    dataconfig2_t.WDEL    = TRUE;
    dataconfig2_t.DFSEL   = SPI_FMT_3;//8 bit  fmt2
    dataconfig2_t.CSNR    = 0xFE;
}
/*-----------------------------------------------------------------------SPIPCA2129Init------------------------------------------------------*/
void SPIPCA2129Init(){
    dataconfig3_t.CS_HOLD = TRUE;
    dataconfig3_t.WDEL    = TRUE;
    dataconfig3_t.DFSEL   = SPI_FMT_1;
    dataconfig3_t.CSNR    = 0xFE;
}
/*----------------------------------------------------------------------SPISST25PF040CInit---------------------------------------------------*/
void SPISST25PF040CInit(){
    dataconfig4_t.CS_HOLD = TRUE;
    dataconfig4_t.WDEL    = TRUE;
    dataconfig4_t.DFSEL   = SPI_FMT_2;
    dataconfig4_t.CSNR    = 0xFE;
}
/*-----------------------------------------------------------------------SPIdevicesinit------------------------------------------------------*/
void SPIdevicesinit(){
    SPIMCPInit();
    SPIADS1018Init();
    SPIPCA2129Init();
    SPISST25PF040CInit();
}
/*-------------------------------------------------------------------------OutputInit--------------------------------------------------------*/
void OutputInit(){
    //4 channel POWER SWITCHE(LEFT) LOW
    gioSetBit(VNQ5050KTRE_LEFT_INT1_PORT, VNQ5050KTRE_LEFT_INT1_PIN, LOW);
    gioSetBit(VNQ5050KTRE_LEFT_INT2_PORT, VNQ5050KTRE_LEFT_INT2_PIN, LOW);
    gioSetBit(VNQ5050KTRE_LEFT_INT3_PORT, VNQ5050KTRE_LEFT_INT3_PIN, LOW);
    gioSetBit(VNQ5050KTRE_LEFT_INT4_PORT, VNQ5050KTRE_LEFT_INT4_PIN, LOW);
    gioSetBit(VNQ5050KTRE_LEFT_STAT_DIS_PORT, VNQ5050KTRE_LEFT_STAT_DIS_PIN, LOW);        //1 to disable status
    //4 channel POWER SWITCHE(RIGHT) LOW
    gioSetBit(VNQ5050KTRE_RIGHT_INT1_PORT, VNQ5050KTRE_RIGHT_INT1_PIN, LOW);
    gioSetBit(VNQ5050KTRE_RIGHT_INT2_PORT, VNQ5050KTRE_RIGHT_INT2_PIN, LOW);
    gioSetBit(VNQ5050KTRE_RIGHT_INT3_PORT, VNQ5050KTRE_RIGHT_INT3_PIN, LOW);
    gioSetBit(VNQ5050KTRE_RIGHT_INT4_PORT, VNQ5050KTRE_RIGHT_INT4_PIN, LOW);
    gioSetBit(VNQ5050KTRE_RIGHT_STAT_DIS_PORT, VNQ5050KTRE_RIGHT_STAT_DIS_PIN, LOW);        //1 to disable status

    //2 channel POWER SWITCHE(LEFT) LOW
    gioSetBit(VND5T100AJTRE_LEFT_INT1_PORT, VND5T100AJTRE_LEFT_INT1_PIN, LOW);
    gioSetBit(VND5T100AJTRE_LEFT_INT2_PORT, VND5T100AJTRE_LEFT_INT2_PIN, LOW);
    //2 channel POWER SWITCHE(RIGHT) LOW
    gioSetBit(VND5T100AJTRE_RIGHT_INT1_PORT, VND5T100AJTRE_RIGHT_INT1_PIN, LOW);
    gioSetBit(VND5T100AJTRE_RIGHT_INT2_PORT, VND5T100AJTRE_RIGHT_INT2_PIN, LOW);

    //1 channel POWER SWITCHE(BOTTOM LEFT) LOW
    gioSetBit(NCV8403ADTRKG_LEFT_INT1_PORT, NCV8403ADTRKG_LEFT_INT1_PIN, LOW);
    //1 channel POWER SWITCHE(BOTTOM RIGHT) LOW
    gioSetBit(NCV8403ADTRKG_RIGHT_INT1_PORT, NCV8403ADTRKG_RIGHT_INT1_PIN, LOW);

    //1 channel POWER SWITCHE(BOTTOM CENTER) LOW
    gioSetBit(VN5T006ASPTRE_BOTTOM_INT1_PORT, VN5T006ASPTRE_BOTTOM_INT1_PIN, LOW);
}
/*----------------------------------------------------------------Left4PowerSwitchStatusGet--------------------------------------------------*/
void Left4PowerSwitchStatusGet(){
    //4 channel POWER SWITCHE(LEFT) STATUS
    VNQ5050KTRE_LEFT_1STAT1_VAL = adc_data[11].value;
    VNQ5050KTRE_LEFT_1STAT2_VAL = adc_data[3].value;
    VNQ5050KTRE_LEFT_1STAT3_VAL = adc_data[2].value;
    VNQ5050KTRE_LEFT_1STAT4_VAL = adc_data[10].value;
}
/*----------------------------------------------------------------Right4PowerSwitchStatusGet-------------------------------------------------*/
void Right4PowerSwitchStatusGet(){
    //4 channel POWER SWITCHE(RIGHT) STATUS
    VNQ5050KTRE_RIGHT_2STAT1_VAL = adc_data[8].value;
    VNQ5050KTRE_RIGHT_2STAT2_VAL = adc_data[6].value;
    VNQ5050KTRE_RIGHT_2STAT3_VAL = adc_data[5].value;
    VNQ5050KTRE_RIGHT_2STAT4_VAL = adc_data[4].value;
}
/*----------------------------------------------------------------Left2PowerSwitchStatusGet--------------------------------------------------*/
void Left2PowerSwitchStatusGet(){
    //2 channel POWER SWITCHE(LEFT)
    VNQ5050KTRE_LEFT_1CS1_VAL = adc_data[9].value;
    VNQ5050KTRE_LEFT_1CS2_VAL = adc_data[1].value;
}
/*----------------------------------------------------------------Right2PowerSwitchStatusGet-------------------------------------------------*/
void Right2PowerSwitchStatusGet(){
    //2 channel POWER SWITCHE(RIGHT)
    VNQ5050KTRE_RIGHT_2CS1_VAL = adc_data[0].value;
    VNQ5050KTRE_RIGHT_2CS2_VAL = adc_data[7].value;
}
/*----------------------------------------------------------------Bottom1PowerSwitchStatusGet-------------------------------------------------*/
void Bottom1PowerSwitchStatusGet(){
    VN5T006ASPTRE_BOTTOM_CS1_VAL = adc_data[12].value;
}
/*------------------------------------------------------------------AllPowerSwitchSTatusGet-----------------------------------------------------*/
void AllPowerSwitchSTatusGet(){
    Left4PowerSwitchStatusGet();
    Right4PowerSwitchStatusGet();
    Left2PowerSwitchStatusGet();
    Right2PowerSwitchStatusGet();
    Bottom1PowerSwitchStatusGet();
}
/*-----------------------------------------------------------------------MCUADCread-------------------------------------------------------------*/
void MCUADCread(){
    adcStartConversion(adcREG1,adcGROUP1);
    //while((adcIsConversionComplete(adcREG1,adcGROUP1))==0);
    if((adcIsConversionComplete(adcREG1,adcGROUP1))!=0){
        ch_count = adcGetData(adcREG1, adcGROUP1,&adc_data[0]);
        ch_count = ch_count;
        adcStopConversion(adcREG1,adcGROUP1);
    }

}
/*------------------------------------------------------------------------InputInit--------------------------------------------------------------*/
void InputInit(){
    MCP_Init(2, 0);
    for(int i=1 ; i<=16 ; i++){
        MCP_PinMode(i, 0);      //set second pin to be input
    }
}

/*--------------------------------------------------------------------functions 2 calculate--------------------------------------------------------*/
void AngleToValue ( int Home , int KatSayi , int Aci ){
    AciSonuc = Home + ( KatSayi * (Aci) );
}

void ValueToAngle ( int Home , int KatSayi , int Sensor ){
    DegerSonuc =  ( Sensor - Home ) / KatSayi;
}

void HedefAci ( float Aci){
    AngleValue = 41 * sin(Aci/68);
}

void logKeep(uint16 output_channel ,uint16 error_code){
    Write(log_start_adress,output_channel);
    log_start_adress++;
    Write(log_start_adress,error_code);
    log_start_adress++;

}
//we can just use this function after pressing the button of its output other wise
//if there is no input there is no output and status can be undefined
void AlignmentValfLeftStatusGet(){


}

/*-------------------------------------------------------------------------epromWrite--------------------------------------------------------------*/
void epromWrite(unsigned int Block_NO,uint8 eeprom_Data[16] ){

    //unsigned int loop;
    /* Initialize RAM array.*/
    for(int i = 0;i<=sizeof(SpecialRamBlock);i++ ){
        SpecialRamBlock[i] = eeprom_Data[i];
    }

    TI_Fee_Init();
    do
    {
        TI_Fee_MainFunction();
        delay();
        Status=TI_Fee_GetStatus(0);
    }
    while(Status!= IDLE);

  TI_Fee_Format(0xA5A5A5A5U);
    do
    {
        TI_Fee_MainFunction();
        delay();
        Status=TI_Fee_GetStatus(0 );
    }
    while(Status!= IDLE);

    BlockOffset = 0;
    Length = 0xFFFF;
    TI_Fee_Init();
    do
    {
        TI_Fee_MainFunction();
        delay();
        Status=TI_Fee_GetStatus(0 );
    }
    while(Status!= IDLE);

    TI_Fee_WriteAsync(Block_NO, &SpecialRamBlock[0]);
    do
    {
        TI_Fee_MainFunction();
        delay();
        Status=TI_Fee_GetStatus(0);
    }
    while(Status!=IDLE);

}
/*-------------------------------------------------------------------------eepromRead--------------------------------------------------------------*/
void eepromRead(unsigned int Block_NO,uint8 eeprom_Data_RX[16]){
        /* Read the block with unknown length */
        unsigned char *Read_Ptr=eeprom_Data_RX;

        TI_Fee_Init();
        do
        {
            TI_Fee_MainFunction();
            delay();
            Status=TI_Fee_GetStatus(0 );
        }
        while(Status!= IDLE);

         BlockOffset = 0;
         Length = 0xFFFF;
         oResult=TI_Fee_Read(Block_NO,BlockOffset,Read_Ptr,Length);
         do
         {
             TI_Fee_MainFunction();
             delay();
             Status=TI_Fee_GetStatus(0);
         }
         while(Status!=IDLE);

}
/*-------------------------------------------------------------------------CanYellowHighLow--------------------------------------------------------*/
void CanYellowHighLow ( bool state ){

    if( state ){
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//yellow
        can_tx_data[0] = (255) & 0xFF;

    }else{
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//yellow
        can_tx_data[0] = (0) & 0xFF;

    }
}
/*--------------------------------------------------------------------------CanRedHighLow----------------------------------------------------------*/
void CanRedHighLow ( bool state ){

    if( state ){
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);
        can_tx_data[1] = (255) & 0xFF;
    }else{
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);
        can_tx_data[1] = (0) & 0xFF;
    }
}
/*--------------------------------------------------------------------------canMessageCheck----------------------------------------------------------*/
void canMessageCheck(){

    if(!canIsRxMessageArrived(canREG1, canMESSAGE_BOX1)){
        canGetData(canREG1, canMESSAGE_BOX1,can_rx_data);  /* receive on can2  */ //id 1
    }

    if( Semiautomaticmode ){

        sensorRight = can_rx_data[0];
        sensorLeft =  can_rx_data[1];
        PumpSlaveBtn =can_rx_data[2];

        if( !emergency_stop ){

            if( sensorRight == true && sensorLeft == true ){
                CanRedHighLow(true);
            }else{
                CanRedHighLow(false);
            }

            if( AttractiveLeftSensorAndCamelNeck >= 2500 ){
                CanYellowHighLow(true);
                AutoAligmentState = true;

            }else if(AttractiveLeftSensorAndCamelNeck <= 150 ){
                CanYellowHighLow(false);
                AutoAligmentState = false;
                semi_Automatic_AutoAlignment = false;
            }

        }
    }


    if( Fullautomaticmode ){

//        ValueToAngle(1800,6.5, 500);
//        HedefAci(DegerSonuc);
//        AngleToValue(1, eeprom_read_data.FifthWheelNumber, AngleValue);

        //do not forgot to put init values if you want every thing to work because in real life vlue when you
        //save it while home position is different from the value while second saving alse canrxdata need to keep change so
        // our if( canSensorData >= (AciSonuc - 5)  && canSensorData <= (AciSonuc + 5) ){ and others can work
        ValueToAngle(eeprom_read_data.CamelneckHome, eeprom_read_data.CamelneckNumber, AttractiveLeftSensorAndCamelNeck);
        HedefAci(DegerSonuc);
        AngleToValue(eeprom_read_data.FifthWheelHome, eeprom_read_data.FifthWheelNumber, AngleValue);

//        canSensorData = can_rx_data[0];
//        canSensorData <<= 8;
//        canSensorData |= can_rx_data[1];
        canSensorData = (ADS1018Data[2]-1000);
        PumpSlaveBtn =  can_rx_data[2];

        if( CalibrationDone == true){

            if( canSensorData >= (AciSonuc - 5)  && canSensorData <= (AciSonuc + 5) ){

                AutoAligmentState = false;

                VehicleInRoute.set = DELAY_250MS;
                FifthWheelAnglePositioneRight.set =0;
                FifthWheelAnglePositioneLeft.set =0;
                CalibrationFailure.set = 0;
                FrontSensorInoperative.set =0;
                BackSensorInoperative.set =0;

           }else if( canSensorData <  AciSonuc  && canSensorData > eeprom_read_data.FifthWheelLeftAngle){

                AutoAligmentState = true;

                VehicleInRoute.set =0;
                FifthWheelAnglePositioneRight.set =0;
                FifthWheelAnglePositioneLeft.set =DELAY_500MS;
                CalibrationFailure.set = 0;
                FrontSensorInoperative.set =0;
                BackSensorInoperative.set =0;

           }else if( canSensorData > AciSonuc  && canSensorData < eeprom_read_data.FifthWheelRightAngle ){
                AutoAligmentState = true;

                VehicleInRoute.set =0;
                FifthWheelAnglePositioneRight.set =DELAY_500MS;
                FifthWheelAnglePositioneLeft.set =0;
                CalibrationFailure.set = 0;
                FrontSensorInoperative.set =0;
                BackSensorInoperative.set =0;

            }else if( canSensorData  < (eeprom_read_data.FifthWheelLeftAngle - 300)){
                if(canSensorData  < (eeprom_read_data.FifthWheelLeftAngle - 350)){
                    CalibrationFailure.set = DELAY_250MS;

                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    FrontSensorInoperative.set =0;
                    BackSensorInoperative.set = 0;
                }else{
                    CalibrationFailure.set = 0;
                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    FrontSensorInoperative.set =0;
                    BackSensorInoperative.set = DELAY_250MS;
                }
            }else if( canSensorData > (eeprom_read_data.FifthWheelRightAngle + 300)){
                if(canSensorData  > (eeprom_read_data.FifthWheelRightAngle + 350)){
                    CalibrationFailure.set = DELAY_250MS;

                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    FrontSensorInoperative.set =0;
                    BackSensorInoperative.set = 0;
                }else{
                    CalibrationFailure.set = 0;
                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    FrontSensorInoperative.set =0;
                    BackSensorInoperative.set = DELAY_250MS;
                }
            }else if( AttractiveLeftSensorAndCamelNeck < (eeprom_read_data.CamelneckLeftAngle - 300)){
                if( AttractiveLeftSensorAndCamelNeck < (eeprom_read_data.CamelneckLeftAngle - 350) && AttractiveLeftSensorAndCamelNeck > eeprom_read_data.CamelneckLeftAngle){
                    CalibrationFailure.set = DELAY_250MS;

                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    BackSensorInoperative.set =0;
                    FrontSensorInoperative.set = 0;
                }else{
                    CalibrationFailure.set = 0;
                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    BackSensorInoperative.set =0;
                    FrontSensorInoperative.set = DELAY_250MS;
                }
            }else if( AttractiveLeftSensorAndCamelNeck > (eeprom_read_data.CamelneckRightAngle + 300)){
                if( AttractiveLeftSensorAndCamelNeck > (eeprom_read_data.CamelneckRightAngle + 350) && AttractiveLeftSensorAndCamelNeck < eeprom_read_data.CamelneckRightAngle){
                    CalibrationFailure.set = DELAY_250MS;

                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    BackSensorInoperative.set =0;
                    FrontSensorInoperative.set = 0;
                }else{
                    CalibrationFailure.set = 0;
                    VehicleInRoute.set =0;
                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    BackSensorInoperative.set =0;
                    FrontSensorInoperative.set = DELAY_250MS;
                }
            }
        }
    }

}
/*--------------------------------------------------------------------------systemActiveCheck------------------------------------------------------------------------------*/
bool systemActiveCheck(){

    SysActive =  MCP_pinRead(GPO_SYSTEM_ACTIVE_PIN);
    return SysActive;
}
/*------------------------------------------------------------------------------ISSCheck-----------------------------------------------------------------------------------*/
bool ISSCheck(){

    ISSActiveState = MCP_pinRead(MCP_GPB2);
    return ISSActiveState;
}
/*---------------------------------------------------------------------getSensorAndCamelNeckValue--------------------------------------------------------------------------*/
void getSensorAndCamelNeckValue(){
    ADS1018Adcread();

    if( Semiautomaticmode ){
        if(ADS1018Data[2] != AttractiveLeftSensorAndCamelNeck){
            AttractiveLeftSensorAndCamelNeck = ADS1018Data[2];
        }
    }

    if( Fullautomaticmode ){
        if(ADS1018Data[2] != AttractiveLeftSensorAndCamelNeck){
            if( adc_samplecounter <= 20)
                Amount += ADS1018Data[2];

            if( adc_samplecounter == 20 )
                Average = Amount/20;

            if( adc_samplecounter > 20 )
                Average = ( Average * 19 + ADS1018Data[2] ) / 20;

            if( adc_samplecounter < 21)
                adc_samplecounter++;

            AttractiveLeftSensorAndCamelNeck = Average;
            //AttractiveRightSensor = ui32ADC0Value[0];
        }
     }
}


/*--------------------------------------------------------------------fullAutomaticPumpCheck--------------------------------------------------------------------------------*/
void fullAutomaticPumpCheck(){

    if( full_Automatic_AutoAlignment == false ){

        if ( PumpSlaveBtn ){ //Button Push UP
             gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN , HIGH);
             PumpState = true;

         }else{
             if( PumpState == true){
                 gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN , LOW);
                 PumpState = false;
             }
         }//PumpSlaveBtn

    }//full_Automatic_AutoAlignment

}
/*------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void setAligmentProcess(){

if(aligment_set_level == 1){
    if( AttractiveLeftSensorAndCamelNeck > CamelneckHomeTemporary){

              CamelneckRightAngleTemporary = AttractiveLeftSensorAndCamelNeck;
              CamelneckNumberValue = -1 *(CamelneckHomeTemporary - CamelneckRightAngleTemporary);
              CamelneckLeftAngleTemporary = CamelneckHomeTemporary - CamelneckNumberValue;
              CamelneckNumberTemporary = CamelneckNumberValue / 90;

              FifthWheelRightAngleTemporary = canSensorData;
              FifthWheelNumberValue = -1 * (FifthWheelHomeTemporary - FifthWheelRightAngleTemporary);
              FifthWheelLeftAngleTemporary = FifthWheelHomeTemporary - FifthWheelNumberValue;
              FifthWheelNumberTemporary = FifthWheelNumberValue / 38;
          }else{
              CamelneckLeftAngleTemporary = AttractiveLeftSensorAndCamelNeck;
              CamelneckNumberValue = CamelneckHomeTemporary - CamelneckLeftAngleTemporary;
              CamelneckRightAngleTemporary = CamelneckHomeTemporary + CamelneckNumberValue;
              CamelneckNumberTemporary = CamelneckNumberValue / 90;

              FifthWheelLeftAngleTemporary = canSensorData;
              FifthWheelNumberValue = FifthWheelHomeTemporary - FifthWheelLeftAngleTemporary;
              FifthWheelRightAngleTemporary = FifthWheelHomeTemporary + FifthWheelNumberValue;
              FifthWheelNumberTemporary = FifthWheelNumberValue / 38;
          }

        aligment_set_level = 2;

    }
}

/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void commandButtonscheck(){

/*********************************************************************** AlignmentRight button bottun pressed***/
    //here we need to try enum and create one function
    if ( AlignmentRight != MCP_pinRead(GIO_ALIGNMENT_RIGHT_BTN_PIN)){
        if (AlignmentRight = MCP_pinRead(GIO_ALIGNMENT_RIGHT_BTN_PIN)){ //Button Push UP

            aligment_buf = set_bit(aligment_buf, 0, 1);
            right_aligment = true;
        }else{
            aligment_buf = set_bit(aligment_buf, 0, 0);
            right_aligment = false;
        }
    }//AlignmentRight
/*********************************************************************** AlignmentLeft button bottun pressed***/
    if ( AlignmentLeft != MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN) ){
        if ((AlignmentLeft = MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN)) ){

            aligment_buf = set_bit(aligment_buf, 1, 1);
            left_aligment = true;
            check_flag = true;
        }else{
            aligment_buf = set_bit(aligment_buf, 1, 0);
            left_aligment = false;
            check_flag = false ;

        }
    }//AlignmentLeft
/************************************************************************* Auto mode button bottun pressed***/
    if ( AlignmentAuto != MCP_pinRead(GIO_ALIGNMENT_AUTO_BTN_PIN) ){
        if ((AlignmentAuto = MCP_pinRead(GIO_ALIGNMENT_AUTO_BTN_PIN)) ){

            aligment_buf = set_bit(aligment_buf, 2, 1);
            auto_aligment = true;
        }else{
            aligment_buf = set_bit(aligment_buf, 2, 0);
            auto_aligment =false;
        }
    }//AlignmentAuto
/************************************************************************ Set mode button bottun pressed***/
    if(!Semiautomaticmode){
        if ( SetMode = MCP_pinRead(GIO_ALIGNMENT_SET_BTN_PIN) ){
            if(tick){
                tick = false;
                setcounter++;
                if (++setcounter == 150 ){
                        set_aligment = !set_aligment;
                        aligment_buf = set_bit(aligment_buf, 3, 1);
                }
            }
        }else{
            setcounter = 0;
            aligment_buf = set_bit(aligment_buf, 3, 0);

        }//SetMode
    }//Semiautomaticmode
/************************************************************************ Emergency stop bottun pressed***/
    if ( StopMode != MCP_pinRead(GIO_STOP_BTN_PIN) ){
        if ((StopMode = MCP_pinRead(GIO_STOP_BTN_PIN)) ){
            aligment_buf = set_bit(aligment_buf, 4, 1);
            emergency_stop = true;

            gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT,GIO_ALIGNMENT_VALF_LEFT_PIN ,LOW);
            gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT,GIO_ALIGNMENT_VALF_RIGHT_PIN ,LOW);
            gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN ,LOW);
        }else{
            aligment_buf = set_bit(aligment_buf, 4, 0);
            emergency_stop = false;
            EmergencyStopActive.set = 0;
            CalibrationFailure.set = DELAY_250MS;
            gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
            gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow

        }
    }//StopMode

}

/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
void fullAutomaticAligmentCommandsCheck(){

    commandButtonscheck();


    switch (aligment_buf) {
/******************************************************************************** No bottun pressed***/
        case NOTHING_PRESSED:
            if(full_Automatic_AutoAlignment == false){
                gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, LOW);
                gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, LOW);
                gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, LOW);
            }
            break;
/************************************************************************Right_valf bottun process***/
        case AlignmentRight_PRESSED:
            if(emergency_stop == false && right_aligment == true){
                if(full_Automatic_Set_Aligment){
                    //printf("right save\n");
                    setAligmentProcess();
                    //CalibrationSecondTemprorySettings.set = DELAY_1S;//need to edit accord the documents
                    CalibrationFirstTemprorySettings.set = 0;
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                }else{
                    if(full_Automatic_AutoAlignment == false){
                        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, HIGH);
                        gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
                    }

                }
            }
            break;
/************************************************************************left_valf bottun process***/
        case AlignmentLeft_PRESSED:
            if(emergency_stop == false && left_aligment == true){
                if(full_Automatic_Set_Aligment){
                    //printf("left save\n");
                    setAligmentProcess();
                    //CalibrationSecondTemprorySettings.set = DELAY_1S;//need to edit accord the documents

                    CalibrationFirstTemprorySettings.set = 0;
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red

                }else{
                    if(full_Automatic_AutoAlignment == false){
                        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, HIGH);
                        gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
                    }

                }
            }

            break;
/***********************************************************************Calibrration Auto process***/
        case AlignmentAuto_PRESSED:
            if(emergency_stop == false && auto_aligment == true){
                if(full_Automatic_Set_Aligment){
                    if(aligment_set_level == 0){
                        Calibrationbegin.set = 0;
                        Calibrationbegin.counter = 0;
                        //printf("home save\n");
                        //take neckangle as  neck angle Home
                        //take canSensorData as FifthWheel Home
                        CamelneckHomeTemporary = AttractiveLeftSensorAndCamelNeck;  //from ads1018[2]
                        FifthWheelHomeTemporary = canSensorData;   //from canbus
                        //printf("yellow lamp on keep\n");
                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                        CalibrationFirstTemprorySettings.set = DELAY_500MS;
                        aligment_set_level = 1;
                    }
                }else{
                    if(full_Automatic_AutoAlignment == false && CalibrationDone == true){
                        full_Automatic_AutoAlignment = true;
                    }else if (full_Automatic_AutoAlignment == false && Semiautomaticmode){
                        semi_Automatic_AutoAlignment =true;
                    }
                }
            }
            break;
/***************************************************************************Calibration Set process***/
        case AlignmentSet_PRESSED:
            if(emergency_stop == false){
                full_Automatic_Set_Aligment = set_aligment;
                if(full_Automatic_Set_Aligment == true){
                    if(full_Automatic_AutoAlignment == false){
                        CalibrationFailure.set = 0;
                        Calibrationbegin.set = DELAY_500MS;
                    }
                }else{
                    if(full_Automatic_AutoAlignment == false){
                        CalibrationFailure.set = DELAY_250MS;
                        Calibrationbegin.set = 0;
                    }
                }

            }
            break;
/****************************************************************Set and Auto at same time process***/
        case AlignmentSave_PRESSED:
            if(emergency_stop == false){
                if(aligment_set_level == 2){
                    full_Automatic_Set_Aligment = false;
                    //printf("saved\n");

                    eeprom_write_data[0]  = CamelneckHomeTemporary >> 8;
                    eeprom_write_data[1]  = CamelneckHomeTemporary & 0x00FF;

                    eeprom_write_data[2]  = CamelneckRightAngleTemporary >> 8;
                    eeprom_write_data[3]  = CamelneckRightAngleTemporary & 0x00FF;

                    eeprom_write_data[4]  = CamelneckLeftAngleTemporary >> 8;
                    eeprom_write_data[5]  = CamelneckLeftAngleTemporary & 0x00FF;

                    eeprom_write_data[6]  = FifthWheelHomeTemporary >> 8;
                    eeprom_write_data[7]  = FifthWheelHomeTemporary & 0x00FF;

                    eeprom_write_data[8]  = FifthWheelRightAngleTemporary >> 8;
                    eeprom_write_data[9]  = FifthWheelRightAngleTemporary & 0x00FF;

                    eeprom_write_data[10] = FifthWheelLeftAngleTemporary >> 8;
                    eeprom_write_data[11] = FifthWheelLeftAngleTemporary & 0x00FF;

                    CamelneckNumberTemporary =(uint16)(CamelneckNumberTemporary*100);
                    eeprom_write_data[12] = (uint16)CamelneckNumberTemporary >> 8;
                    eeprom_write_data[13] = (uint16)CamelneckNumberTemporary & 0x00FF;

                    FifthWheelNumberTemporary = FifthWheelNumberTemporary*100;
                    eeprom_write_data[14] = (uint16)FifthWheelNumberTemporary >> 8;
                    eeprom_write_data[15] = (uint16)FifthWheelNumberTemporary & 0x00FF;

                    //printf(" eeprom saved\n");

                    epromWrite(0x1, eeprom_write_data);
                    eepromRead(0x1, read_data);

                    //use it for reading
                    eeprom_read_data.CamelneckHome      = (read_data[0] << 8)|read_data[1];
                    eeprom_read_data.CamelneckRightAngle= (read_data[2] << 8)|read_data[3];
                    eeprom_read_data.CamelneckLeftAngle = (read_data[4] << 8)|read_data[5];
                    eeprom_read_data.FifthWheelHome     = (read_data[6] << 8)|read_data[7];
                    eeprom_read_data.FifthWheelRightAngle  = (read_data[8] << 8)|read_data[9];
                    eeprom_read_data.FifthWheelLeftAngle   = (read_data[10] << 8)|read_data[11];
                    eeprom_read_data.CamelneckNumber  = (read_data[12] << 8)|read_data[13];
                    eeprom_read_data.CamelneckNumber  = CamelneckNumberTemporary/100;
                    eeprom_read_data.FifthWheelNumber = (read_data[14] << 8)|read_data[15];
                    eeprom_read_data.FifthWheelNumber = FifthWheelNumberTemporary/100;

                    aligment_set_level = 0;
                    //printf("yellow and red lamp off\n");
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                    CalibrationFinalSettings.set = DELAY_250MS;
                }

            }else{

            }
            break;
/***********************************************************************emergency_stop process***/
        case AcilStopON_PRESSED:
            gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT,GIO_ALIGNMENT_VALF_LEFT_PIN ,LOW);//red
            gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT,GIO_ALIGNMENT_VALF_RIGHT_PIN ,LOW);//red
            gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN ,LOW);//red
            //gioSetBit(GIO_OIL_PUMP_PORT,GIO_OIL_PUMP_PIN ,LOW);//red

            emergency_stop = true;

            full_Automatic_AutoAlignment = false;
            full_Automatic_Set_Aligment  = false;

            Calibrationbegin.set = 0;
            CalibrationFirstTemprorySettings.set = 0;
            CalibrationSecondTemprorySettings.set = 0;
            CalibrationFinalSettings.set = 0;
            CalibrationCancel.set = 0;

            set_aligment = false;
            aligment_set_level = 0 ;

            FrontSensorInoperative.set = 0;
            BackSensorInoperative.set = 0;
            FifthWheelAnglePositioneLeft.set = 0;
            FifthWheelAnglePositioneRight.set = 0;
            VehicleInRoute.set = 0;

            AutomaticAligmentDone.set = 0;
            ISSSingalActive.set = 0;

            aligment_set_level = 0;
            EmergencyStopActive.set = DELAY_250MS;
            break;
/***********************************************************************Others process***/
        default:
            if(full_Automatic_AutoAlignment == false){
                gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, LOW);
                gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, LOW);
                gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, LOW);
                EmergencyStopActive.set = 0;
                Calibrationbegin.set = 0;
                CalibrationFirstTemprorySettings.set = 0;
                CalibrationSecondTemprorySettings.set = 0;
                CalibrationFinalSettings.set = 0;
                CalibrationCancel.set = 0;

                FifthWheelAnglePositioneLeft.set = 0;
                FifthWheelAnglePositioneRight.set = 0;
                VehicleInRoute.set = 0;

                AutomaticAligmentDone.set = 0;
                FrontSensorInoperative.set = 0;
                BackSensorInoperative.set = 0;
            }
            break;

    }
}

/*--------------------------------------------------------------------ControlLeds-------------------------------------------------------------------------------------------*/
void ControlLeds(){
    if(tick){
        tick=false;
/***********************************************************************incommon leds****************************************************************************************/
/***********************************************************************EmergencyStopActive****/
        if(EmergencyStopActive.set != 0){
            if(++EmergencyStopActive.count >= EmergencyStopActive.set){

                EmergencyStopActive.state=!EmergencyStopActive.state;
                EmergencyStopActive.count = 0;

               if(EmergencyStopActive.state){
                   //printf("yellow and red lamp on x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red
                   EmergencyStopActive.counter++;
               }else{
                   //printf("yellow and red lamp off x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red

                   if(EmergencyStopActive.counter>=1){
                       EmergencyStopActive.set = 0;
                       EmergencyStopActive.counter = 0;
                       CalibrationFailure.set = DELAY_250MS;
                   }
               }
           }//EmergencyStopActive
        }//EmergencyStopActive
/***************************************************************************AutomaticAligmentDone*****/
        if(AutomaticAligmentDone.set != 0){
            if(++AutomaticAligmentDone.count >= AutomaticAligmentDone.set){

                AutomaticAligmentDone.state=!AutomaticAligmentDone.state;
                AutomaticAligmentDone.count = 0;

               if(AutomaticAligmentDone.state){
                   //printf("yellow and red lamp on 250 x2\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//yellow
                   AutomaticAligmentDone.counter++;
               }else{
                   //printf("yellow and red lamp off 250 x2\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow
                   if(AutomaticAligmentDone.counter>=2){
                       AutomaticAligmentDone.set = 0;
                       AutomaticAligmentDone.counter = 0;

                   }
               }
            }
        }//AutomaticAligmentDone
/***********************************************************************just full auto leds**********************************************************************************/
        if(Fullautomaticmode && !Semiautomaticmode){
/*******************************************************************FrontSensorInoperative*****/
            if( FrontSensorInoperative.set != 0){
                if( ++FrontSensorInoperative.count >= FrontSensorInoperative.set){

                    FrontSensorInoperative.count =0;
                    FrontSensorInoperative.state = !FrontSensorInoperative.state;

                    Calibrationbegin.set = 0;
                    CalibrationCancel.set = 0;
                    CalibrationFirstTemprorySettings.set =0;
                    CalibrationSecondTemprorySettings.set =0;
                    CalibrationFinalSettings.set =0;
                    EmergencyStopActive.set =0;

                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    VehicleInRoute.set =0;
                    CalibrationFailure.set =0;

                    full_Automatic_AutoAlignment = false;

                    if( FrontSensorInoperative.state ){
                        CanYellowHighLow(true);
                        CanRedHighLow(false);
                    }else{
                        CanYellowHighLow(false);
                        CanRedHighLow(false);
                    }
                }
            }//FrontSensorInoperative
/*******************************************************************BackSensorInoperative*****/
            if( BackSensorInoperative.set != 0){
                if( ++BackSensorInoperative.count >= BackSensorInoperative.set){

                    BackSensorInoperative.count =0;
                    BackSensorInoperative.state = !BackSensorInoperative.state;

                    Calibrationbegin.set = 0;
                    CalibrationCancel.set = 0;
                    CalibrationFirstTemprorySettings.set =0;
                    CalibrationSecondTemprorySettings.set =0;
                    CalibrationFinalSettings.set =0;
                    EmergencyStopActive.set =0;

                    FifthWheelAnglePositioneRight.set =0;
                    FifthWheelAnglePositioneLeft.set =0;
                    VehicleInRoute.set =0;
                    CalibrationFailure.set =0;

                    full_Automatic_AutoAlignment = false;

                    if( BackSensorInoperative.state ){
                        CanYellowHighLow(false);
                        CanRedHighLow(true);
                    }else{
                        CanYellowHighLow(false);
                        CanRedHighLow(false);
                    }
                }
            }//BackSensorInoperative
/*************************************************************************Calibrationbegin*****/
            if(Calibrationbegin.set != 0){
                if(++Calibrationbegin.count >= Calibrationbegin.set){

                    Calibrationbegin.state=!Calibrationbegin.state;
                    Calibrationbegin.count = 0;

                   if(Calibrationbegin.state){
                       //printf("yellow and red lamp on x1 for 500ms\n");
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//yellow
                       Calibrationbegin.counter++;
                   }else{
                       //printf("yellow and red lamp off x1 for 500ms\n");
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow

                   }
               }
           }//Calibrationbegin
/***********************************************************CalibrationFirstTemprorySettings****/
            if(CalibrationFirstTemprorySettings.set != 0){
                if(++CalibrationFirstTemprorySettings.count >= CalibrationFirstTemprorySettings.set){

                    CalibrationFirstTemprorySettings.state=!CalibrationFirstTemprorySettings.state;
                    CalibrationFirstTemprorySettings.count = 0;

                   if(CalibrationFirstTemprorySettings.state){
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   }else{
                       //red lamp off x1 for 250ms
                       //printf("red lamp off x1 for 500ms and yellow keep\n");
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
    //                   if(CalibrationFirstTemprorySettings.counter>=1){
    //                       CalibrationFirstTemprorySettings.set = 0;
    //                       CalibrationFirstTemprorySettings.counter = 0;
    //                   }
                   }
               }
           }//CalibrationFirstTemprorySettings
/***********************************************************CalibrationSecondTemprorySettings****/
            if(CalibrationSecondTemprorySettings.set != 0){
                if(++CalibrationSecondTemprorySettings.count >= CalibrationSecondTemprorySettings.set){

                    CalibrationSecondTemprorySettings.state=!CalibrationSecondTemprorySettings.state;
                    CalibrationSecondTemprorySettings.count = 0;

                   if(CalibrationSecondTemprorySettings.state){
                       //printf("yellow and red lamp on keep\n");
                       CalibrationSecondTemprorySettings.counter++;
                   }else{
                       if(CalibrationSecondTemprorySettings.counter>=1){
                           CalibrationSecondTemprorySettings.set = 0;
                           CalibrationSecondTemprorySettings.counter = 0;
                       }
                   }
               }
           }//CalibrationSecondTemprorySettings
/*******************************************************************CalibrationFinalSettings*****/

            if(CalibrationFinalSettings.set != 0){
                if(++CalibrationFinalSettings.count >= CalibrationFinalSettings.set){

                    CalibrationFinalSettings.state=!CalibrationFinalSettings.state;
                    CalibrationFinalSettings.count = 0;

                   if(CalibrationFinalSettings.state){
                       //printf("yellow and red lamp on 250 x3\n");
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//yellow

                       CalibrationFinalSettings.counter++;
                   }else{
                       //printf("yellow and red lamp off 250 x3\n");
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow
                       if(CalibrationFinalSettings.counter>=3){
                           CalibrationFinalSettings.set = 0;
                           CalibrationFinalSettings.counter = 0;
                           CalibrationDone = true;

                       }
                   }
               }
           }//CalibrationFinalSettings
/*************************************************************************CalibrationCancel******/

            if(CalibrationCancel.set != 0){
                if(++CalibrationCancel.count >= CalibrationCancel.set){

                    CalibrationCancel.state=!CalibrationCancel.state;
                    CalibrationCancel.count = 0;

                   if(CalibrationCancel.state){
                       //printf("yellow lamp on 500 x2  //red lamp on 2saniye x1 \n");
                       CalibrationCancel.counter++;
                   }else{
                       //printf("yellow lamp off 500 x2  //red lamp on 2saniye x1 \n");
                       if(CalibrationCancel.counter>=3){
                           //printf("yellow lamp off 500 x2  //red lamp off 2saniye x1 \n");
                           CalibrationCancel.set = 0;
                           CalibrationCancel.counter = 0;
                       }
                   }
               }
           }//CalibrationCancel
/************************************************************************CalibrationFailure*****/

            if(CalibrationFailure.set != 0){
                if(++CalibrationFailure.count >= CalibrationFailure.set){

                    CalibrationFailure.state=!CalibrationFailure.state;
                    CalibrationFailure.count = 0;

                   if(CalibrationFailure.state){
                       //printf("yellow and red lamp on x1 for 250ms\n");
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                       CalibrationFailure.counter++;
                   }else{
                      //printf("yellow and red lamp off x1 for 250ms\n");
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                       gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red

                       if(CalibrationFailure.counter>=1){
                           CalibrationFailure.counter = 0;
                       }
                   }
               }//CalibrationFailure
            }//CalibrationFailure
/************************************************************************VehicleInRoute*********/
            if ( VehicleInRoute.set !=0){
                   if( ++VehicleInRoute.count >= VehicleInRoute.set){

                       VehicleInRoute.count =0;
                       VehicleInRoute.state = !VehicleInRoute.state;

                       CanYellowHighLow(true);
                       CanRedHighLow(true);

                   }
               }//VehicleInRoute
/*********************************************************FifthWheelAnglePositioneRight*********/
               if ( FifthWheelAnglePositioneRight.set !=0 ){
                   if( ++FifthWheelAnglePositioneRight.count >= FifthWheelAnglePositioneRight.set){

                       FifthWheelAnglePositioneRight.count =0;
                       FifthWheelAnglePositioneRight.state = !FifthWheelAnglePositioneRight.state;

                       CanRedHighLow(true);
                       if( FifthWheelAnglePositioneRight.state){
                           CanYellowHighLow(true);
                       }else{
                           CanYellowHighLow(false);
                       }
                   }
               }//FifthWheelAnglePositioneRight

/*********************************************************FifthWheelAnglePositioneLeft*********/
               if ( FifthWheelAnglePositioneLeft.set !=0 ){
                   if( ++FifthWheelAnglePositioneLeft.count >= FifthWheelAnglePositioneLeft.set){

                       FifthWheelAnglePositioneLeft.count =0;
                       FifthWheelAnglePositioneLeft.state = !FifthWheelAnglePositioneLeft.state;

                       CanYellowHighLow(true);
                       if( FifthWheelAnglePositioneLeft.state){
                           CanRedHighLow(true);
                       }else{
                           CanRedHighLow(false);
                       }
                   }
               }//FifthWheelAnglePositioneLeft
        }

    }//tick

}
/*--------------------------------------------------------------------CommandCheck-------------------------------------------------------------------------------------------*/
void CommandCheck(){

        fullAutomaticPumpCheck();
        fullAutomaticAligmentCommandsCheck();

}
/*--------------------------------------------------------------------modeWarning-------------------------------------------------------------------------------------------*/
void modeWarning(){
    if(tick && Fullautomaticmode == false && Semiautomaticmode == false){
        tick = false;

        if( Semiautomaticled.set != 0){
            if( ++Semiautomaticled.count >= Semiautomaticled.set){

                Semiautomaticled.count =0;
                Semiautomaticled.state = !Semiautomaticled.state;

                if( Semiautomaticled.counter < 2){
                    if( Semiautomaticled.state){

                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                    }else{
                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                        Semiautomaticled.counter++;
                    }
                    canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
                }else{
                    Fullautomaticmode = false;
                    Semiautomaticmode = true;
                    Semiautomaticled.state =0;
                    Semiautomaticled.set =0;
                }
            }
        }//semiautonmaticwarning


        if( Fullautomaticled.set != 0){
            if( ++Fullautomaticled.count >= Fullautomaticled.set){

                Fullautomaticled.count =0;
                Fullautomaticled.state = !Fullautomaticled.state;

                if( Fullautomaticled.counter < 2){
                    if( Fullautomaticled.state){

                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                    }else{
                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red
                        Fullautomaticled.counter++;
                    }
                    canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
                }else{
                    Fullautomaticmode = true;
                    Semiautomaticmode = false;
                    Fullautomaticled.state =0;
                    Fullautomaticled.set =0;

                }
            }
        }//fullautonmaticwarning


    }//tick
}
/*-----------------------------------------------------------semiAutomaticAutoHizalamaProcess---------------------------------------------------------------------------------*/
void semiAutomaticAutoHizalamaProcess(){
    if( sensorLeft == 1 && sensorRight == 1){
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,LOW);

        AutomaticAligmentDone.set = 5;
        AutomaticAligmentDone.counter =0;
        semi_Automatic_AutoAlignment = false;

    }else if( sensorRight == 0 && AttractiveLeftSensorAndCamelNeck >= 2500){
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,HIGH);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,HIGH);

    }else if( sensorLeft == 0 && AttractiveLeftSensorAndCamelNeck >= 2500){
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,HIGH);
        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,HIGH);

    }else{
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,LOW);

        semi_Automatic_AutoAlignment = false;
    }
}
/*-----------------------------------------------------------semiAutomaticStandByMode---------------------------------------------------------------------------------*/
void semiAutomaticStandByMode(){

    emergency_stop = true ;
    gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
    gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
    gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,LOW);
    gioSetBit(GIO_OIL_PUMP_PORT, GIO_OIL_PUMP_PIN,LOW);

    CanYellowHighLow(false);
    CanRedHighLow(false);
    can_tx_data[2] = 0;
    canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
}
/*-----------------------------------------------------------fullAutomaticStandBymode---------------------------------------------------------------------------------*/
void fullAutomaticStandBymode(){

    emergency_stop = true;
    full_Automatic_AutoAlignment = false;
    full_Automatic_Set_Aligment  = false;

    ModeSelectDelay.set = 0;
    Calibrationbegin.set = 0;
    CalibrationFirstTemprorySettings.set = 0;
    CalibrationSecondTemprorySettings.set = 0;
    CalibrationFinalSettings.set = 0;
    CalibrationCancel.set = 0;
    CalibrationFailure.set = 0;

    set_aligment = false;
    aligment_set_level = 0 ;

    FifthWheelAnglePositioneLeft.set = 0;
    FifthWheelAnglePositioneRight.set = 0;

    VehicleInRoute.set = 0;

    AutomaticAligmentDone.set = 0;
    ISSSingalActive.set = 0;
    FrontSensorInoperative.set = 0;
    BackSensorInoperative.set = 0;
    aligment_set_level = 0;
    EmergencyStopActive.set = DELAY_250MS;

    gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, LOW);
    gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, LOW);
    gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN,LOW);
    gioSetBit(GIO_OIL_PUMP_PORT, GIO_OIL_PUMP_PIN,LOW);
    CanYellowHighLow(false);
    CanRedHighLow(false);
}

void fullAutomaticAutoAlignmentProcess(){
    if( AciSonuc == canSensorData){

        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,LOW);


        AutomaticAligmentDone.set = DELAY_250MS;
        AutomaticAligmentDone.counter =0;


        full_Automatic_AutoAlignment = false;
        //AutoAligmentState = true;

    }else if( AciSonuc > canSensorData ){

        AutoAligmentState = false;
        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,HIGH);
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,HIGH);

    }else if( AciSonuc < canSensorData ){

        AutoAligmentState = false;
        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,HIGH);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,HIGH);
    }

}
///ADS not working check global counter maybe its timer is not working

void delaymode(){
    if( ModeSelectDelay.set !=0 ){
        for(int i = 0; i <= 10000000; i++){
        }
        ModeSelectDelay.set = 0;
    }
}

void pinStatusCheck(uint8_t pin2check){

    if(check_flag == true){
        if(pin2check == GIO_ALIGNMENT_VALF_LEFT_PIN_STATUS){
            GIO_STATUS_VALUE =VN5T006ASPTRE_BOTTOM_CS1_VAL;
        }

        if((GIO_STATUS_VALUE >8 && GIO_STATUS_VALUE <250)){
            keep_log_short_counter  = false;
            keep_log_open_counter   = false;
            keep_log_normal_counter = true;
        }else if((GIO_STATUS_VALUE <8)){
              keep_log_open_counter   = true;
              keep_log_normal_counter = false;
              keep_log_short_counter  = false;
          }else if((GIO_STATUS_VALUE >250)){
              keep_log_short_counter  = true;
              keep_log_open_counter   = false;
              keep_log_normal_counter = false;
          }
    }else{
        if(keep_log_open_counter || keep_log_short_counter){
            keep_log_flag = true;
            if(keep_log_flag){
                mycounter = mycounter + 1;
                logKeep(GIO_ALIGNMENT_VALF_LEFT_PIN_STATUS, GIO_OPEN_ERROR);
                keep_log_flag = false;
                keep_log_open_counter  = false;
                keep_log_short_counter = false;
            }
        }
    }
}

void oilPumpStatusCheck(uint8_t pin2check){

        if(pin2check == GIO_OIL_PUMP_PIN_STATUS){
            GIO_STATUS_VALUE =VNQ5050KTRE_LEFT_1CS2_VAL;
        }

        if((GIO_STATUS_VALUE >2 && GIO_STATUS_VALUE <250)){
            pump_keep_log_short_counter  = false;
            pump_keep_log_open_counter   = false;
            pump_keep_log_normal_counter = true;
            pump_check_again = true;
        }else if((GIO_STATUS_VALUE <8)){
              pump_keep_log_open_counter   = true;
              pump_keep_log_normal_counter = false;
              pump_keep_log_short_counter  = false;

          }else if((GIO_STATUS_VALUE >250)){
              pump_keep_log_short_counter  = true;
              pump_keep_log_open_counter   = false;
              pump_keep_log_normal_counter = false;
          }

        if(pump_check_again){
            if(pump_keep_log_open_counter || pump_keep_log_short_counter){
                pump_keep_log_flag = true;
                if(pump_keep_log_flag){
                    pump_mycounter = pump_mycounter + 1;
                    logKeep(GIO_PUMP_PIN_STATUS, GIO_OPEN_ERROR);
                    pump_keep_log_flag = false;
                    pump_keep_log_open_counter  = false;
                    pump_keep_log_short_counter = false;
                    pump_check_again = false;

                }
            }
        }

}

void main(void)
{
//    HardwareInit();
//    SPIdevicesinit();
//    InputInit();
//    OutputInit();
//    ModeSelectDelay.set =DELAY_250MS;
//    delaymode();
//    mode = MCP_pinRead(GIO_FUNCTION_SELECT_PIN);
//    if(mode > 0){
//        Fullautomaticled.set = DELAY_250MS;
//        //fullAutomatic = true;
//    }else{
//        Semiautomaticled.set = DELAY_250MS;
//        //semiAutomatic = false;
//    }
//
//
//    Calibrationbegin.set = 0;
//    CalibrationFirstTemprorySettings.set = 0;
//    CalibrationSecondTemprorySettings.set = 0;
//    CalibrationFinalSettings.set = 0;
//    EmergencyStopActive.set =0;
//    CalibrationFailure.set = DELAY_250MS;
//    ChipErase();
//
//
//    adress_value[0] =  Read(0x000001);
//    adress_value[1] =  Read(0x000002);
//    adress_value[2] =  Read(0x000003);
//    adress_value[3] =  Read(0x000004);
//    adress_value[4] =  Read(0x000005);
//    adress_value[5] =  Read(0x000006);
//    adress_value[6] =  Read(0x000007);
//    adress_value[7] =  Read(0x000008);
//    adress_value[8] =  Read(0x000009);
//    adress_value[9] =  Read(0x00000A);
//    adress_value[10] =  Read(0x00000B);
//    adress_value[11] =  Read(0x00000C);
//    adress_value[12] =  Read(0x00000D);


    canInit();

    while(1) /* ... continue forever */
    {
        if(canIsRxMessageArrived(canREG1, canMESSAGE_BOX1)){
            canGetData(canREG1, canMESSAGE_BOX1,can_rx_data);  /* receive on can2  */ //id 1
        }
//        modeWarning();
//        canMessageCheck();
//        MCUADCread();
/////*****************************************************************************************************************************************************/
/////******************************************************************************fullAutomaticmode*****************************************************/
//
//        if( Fullautomaticmode ){
//            if(systemActiveCheck()&& ISSCheck()!=true){
//                gioSetBit(GIO_OIL_PUMP_PORT, GIO_OIL_PUMP_PIN, HIGH);
//
//                if(full_Automatic_AutoAlignment){
//                    fullAutomaticAutoAlignmentProcess();
//                }
//                ControlLeds();
//                CommandCheck();
//                AllPowerSwitchSTatusGet();
//                pinStatusCheck(GIO_ALIGNMENT_VALF_LEFT_PIN_STATUS);     //left valf status
//                oilPumpStatusCheck(GIO_OIL_PUMP_PIN_STATUS);            //pump status
//            }else{
//                fullAutomaticStandBymode();
//            }//systemActiveCheck
//
//        }//Fullautomaticmode
////
///////****************************************************************************************************************************************************/
///////******************************************************************************Semiautomaticmode*****************************************************/
//
//        if(Semiautomaticmode){
//            if(systemActiveCheck()&& ISSCheck()!=true){
//                gioSetBit(GIO_OIL_PUMP_PORT, GIO_OIL_PUMP_PIN, HIGH);
//
//                if( semi_Automatic_AutoAlignment ){
//                    semiAutomaticAutoHizalamaProcess();
//                }
//                ControlLeds();
//                CommandCheck();
//                pinStatusCheck(GIO_ALIGNMENT_VALF_LEFT_PIN_STATUS);     //left valf status
//                oilPumpStatusCheck(GIO_OIL_PUMP_PIN_STATUS);            //pump status
//            }else{
//                semiAutomaticStandByMode();
//            }//systemActiveCheck
//        }//Semiautomaticmode

    }//while(1)
}//main

/****************************************************************************************************************************************************/
/*************************************************************************rtiNotification************************************************************/
/****************************************************************************************************************************************************/
void rtiNotification(uint32 notification)
{
    if(notification ==rtiNOTIFICATION_COMPARE0){
        tick=true;
    }
    if(notification ==rtiNOTIFICATION_COMPARE1){
        global_counter++;
        //tick=true;

        getSensorAndCamelNeckValue();
    }
}



uint32 checkPackets(uint8 *src_packet,uint8 *dst_packet,uint32 psize)
{
   uint32 err=0;
   uint32 cnt=psize;

   while(cnt--)
   {
     if((*src_packet++) != (*dst_packet++))
     {
        err++;           /*  data error  */
     }
   }
   return (err);
}
