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
//#include "ADS1018.h"
#include "PCA2129.h"
#include "SST25PF040C.h"
//NOTES:
/*1.OUR USEED ADIN PINS ARE:
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
 *4.SPI1 CONNECTED DEVICE IS  : ADC2129 Real Time Clock and SST25PF040C Flash Memory
 *5.SPI2 CONNECTED DEVICE IS  : MCP23S18  GIO  EXPANDER
 *6.SPI3 CONNECTED DEVICE IS  : ADS1018 ANALOG EXPANDER
 *
 *
 */
uint16 myinput[16];
uint8_t result;

//ADS1018
/**************************************************************
 * Global Definitions
 **************************************************************/
#define ADS_AIN0      0xC28B
#define ADS_AIN1      0xD28B
#define ADS_AIN2      0xE28B
#define ADS_AIN3      0xF28B
spiDAT1_t dataconfig2_t;
uint16_t global_counter=0;

volatile uint32_t beginTime;
volatile uint32_t timeOut;

uint32_t step=0;
uint32_t functiontodo =0;
volatile int8_t subStep  = 0;
volatile int8_t failStep = 0;

volatile int8_t currentStep   = 0;
volatile int8_t successStep   = 0;



enum substeps{

    STEP_0 = 0,
    STEP_1,
    STEP_2,
    STEP_3,
    STEP_WAIT,
    STEP_GPRS_CONNECTED
};


uint16_t rcvId[] = { 0xC28B, 0xD28B, 0xE28B, 0xF28B };
uint16_t ADS1018Data[4];
uint16_t Amount;
uint16_t Average;
uint16_t adc_samplecounter;

uint8 PumpSlaveBtn = 0;
bool fullPumpState = false;

bool AcilStopON = false;
bool ISSAktive = true;
bool SysActive = false;
bool ISSActiveState = false;

bool AutoHizalamaState = false;
bool AutoHizalama = false;

bool AlignmentRight = false;
bool AlignmentLeft = false;
bool AlignmentAuto = false;

bool AlignmentRightState = false;
bool AlignmentLeftState = false;
bool AlignmentAutoState = false;

uint8_t aligment_buf;
bool SetMode = false;
bool StopMode = false;
bool emergency_stop = false;
bool ISS = false;
bool auto_aligment =false;
bool left_aligment =false;
bool right_aligment =false;
bool set_aligment =false;

bool SetModeState = false;
bool StopModeState = false;
bool StopModeStateTwo = false;

bool Caliberundefined = false;
bool SystemActiveCaliber = true;

int CamelneckHomeTemporary;
int CamelneckRightAngleTemporary;
int CamelneckLeftAngleTemporary;
float CamelneckNumberTemporary;
int CamelneckNumberTemporaryinteger;

int FifthWheelHomeTemporary;
int FifthWheelRightAngleTemporary;
int FifthWheelLeftAngleTemporary;
float FifthWheelNumberTemporary;

int FifthWheelNumberValue =0;
int CamelneckNumberValue =0;

typedef struct leds{

    bool state;
    uint8_t count;
    uint8_t set;
    uint8_t counter;

}Led;

Led Fullautomaticled;
Led Semiautomaticled;
Led CaliberAcilStopLed;
Led CaliberStartLed;
Led CaliberStopLed;
Led CaliberBirinciKayitLed;
Led CaliberikinciKayitLed;
Led CaliberKesinKayitLed;
Led TekerlekSagLed;
Led TekerlekSolLed;
Led TekerlekDuzLed;
Led AutoHizalamaBitti;
Led CaliberBozuk;
Led OnSensorAriza;
Led ArkaSensorAriza;

typedef struct timer{
    uint16_t set;
    uint16_t count;
    bool state;
    uint16_t counter;
}Timers;

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


typedef struct buttons{

    bool state;
    uint8_t count;
    uint8_t set;

}Button;


Button KalibrasyonStart;
Button KalibrasyonStop;
Button KalibrasyonKesinKayit;
Button OtomatikhizalamaEx;

typedef enum delays{

    DELAY_250MS = 25,
    DELAY_500MS = 50,
    DELAY_1S = 100

}Delays;

bool full_Automatic_AutoAlignment = false;
bool full_Automatic_Set_Aligment = false;

bool tick=false;
bool tick2=false;
bool Fullautomaticmode = false;
bool Semiautomaticmode = false;
uint16_t AttractiveRightSensor; // Sensor 1
uint16_t AttractiveLeftSensorAndCamelNeck; // Sensor 2


/**************************************************************************/

uint16_t canSensorData;
uint8_t aligment_set_level = 0;
bool PumpState = false;

bool CaliberMod = true;
bool AutoControl = true; //true ali
//ADS1018 spi config

//adc
adcData_t adc_data[16];
void wait(uint32 time);

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
uint16_t adress_value = 0;

//eeprom
uint16 u16JobResult,Status;
Std_ReturnType oResult=E_OK;

unsigned int BlockNumber;
uint8 SpecialRamBlock[16];
uint8 EepromData[16];

unsigned int BlockOffset, Length;
unsigned char read_data[100]={0};


unsigned char pattern;
uint16 u16writecounter;

unsigned int  FeeVirtualSectorNumber;
unsigned char VsState, u8EEPIndex;
unsigned char u8VirtualSector;
uint8 Test_Recovery;
uint8 Test_Cancel;

uint16_t AciSonuc ;
uint16_t DegerSonuc ;

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


void SPIPCA2129Init(){
    dataconfig3_t.CS_HOLD = TRUE;
    dataconfig3_t.WDEL    = TRUE;
    dataconfig3_t.DFSEL   = SPI_FMT_1;
    dataconfig3_t.CSNR    = 0xFE;
}

void SPISST25PF040CInit(){
    dataconfig4_t.CS_HOLD = TRUE;
    dataconfig4_t.WDEL    = TRUE;
    dataconfig4_t.DFSEL   = SPI_FMT_2;
    dataconfig4_t.CSNR    = 0xFE;
}
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
                    subStep = successStep;
            }else{
                    Register2write();
            }

    }
}

void RTIInit(){
    rtiInit();

    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);
    //
    //    rtiEnableNotification(rtiNOTIFICATION_COMPARE1);

    _enable_IRQ();
    rtiStartCounter(rtiCOUNTER_BLOCK0);
    //    rtiStartCounter(rtiCOUNTER_BLOCK0);

}

void HardwareInit(){
    hetInit();
    gioInit();
    adcInit();
    spiInit();
    canInit();
    RTIInit();
}

void SPIMCPInit(){
    dataconfig1_t.CS_HOLD = TRUE;
    dataconfig1_t.WDEL    = FALSE;
    dataconfig1_t.DFSEL   = SPI_FMT_0;
    dataconfig1_t.CSNR    = 0xFE;

    _enable_IRQ();
}

void SPIADS1018Init(){
    dataconfig2_t.CS_HOLD = TRUE;
    dataconfig2_t.WDEL    = TRUE;
    dataconfig2_t.DFSEL   = SPI_FMT_3;//8 bit  fmt2
    dataconfig2_t.CSNR    = 0xFE;
}

void SPIdevicesinit(){
    SPIMCPInit();
    SPIADS1018Init();
    SPIPCA2129Init();
    SPISST25PF040CInit();
}

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

void Left4PowerSwitchStatusGet(){
    //4 channel POWER SWITCHE(LEFT) STATUS
    VNQ5050KTRE_LEFT_1STAT1_VAL = adc_data[11].value;
    VNQ5050KTRE_LEFT_1STAT2_VAL = adc_data[3].value;
    VNQ5050KTRE_LEFT_1STAT3_VAL = adc_data[2].value;
    VNQ5050KTRE_LEFT_1STAT4_VAL = adc_data[10].value;
}

void Right4PowerSwitchStatusGet(){
    //4 channel POWER SWITCHE(RIGHT) STATUS
    VNQ5050KTRE_RIGHT_2STAT1_VAL = adc_data[8].value;
    VNQ5050KTRE_RIGHT_2STAT2_VAL = adc_data[6].value;
    VNQ5050KTRE_RIGHT_2STAT3_VAL = adc_data[5].value;
    VNQ5050KTRE_RIGHT_2STAT4_VAL = adc_data[4].value;
}

void Left2PowerSwitchStatusGet(){
    //2 channel POWER SWITCHE(LEFT)
    VNQ5050KTRE_LEFT_1CS1_VAL = adc_data[9].value;
    VNQ5050KTRE_LEFT_1CS2_VAL = adc_data[1].value;
}

void Right2PowerSwitchStatusGet(){
    //2 channel POWER SWITCHE(RIGHT)
    VNQ5050KTRE_RIGHT_2CS1_VAL = adc_data[0].value;
    VNQ5050KTRE_RIGHT_2CS2_VAL = adc_data[7].value;
}

void Bottom1PowerSwitchStatusGet(){
    VN5T006ASPTRE_BOTTOM_CS1_VAL = adc_data[15].value;
}

void AllPowerSwitchSTatusGet(){
    Left4PowerSwitchStatusGet();
    Right4PowerSwitchStatusGet();
    Left2PowerSwitchStatusGet();
    Right2PowerSwitchStatusGet();
}

void CanYellowHighLow ( bool state ){

    if( state ){
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//yellow
        can_tx_data[0] = (255) & 0xFF;

    }else{
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//yellow
        can_tx_data[0] = (0) & 0xFF;

    }
}

void CanRedHighLow ( bool state ){

    if( state ){
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);
        can_tx_data[1] = (255) & 0xFF;
    }else{
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);
        can_tx_data[1] = (0) & 0xFF;
    }
}

void MCUADCread(){
    adcStartConversion(adcREG1,adcGROUP1);

    if((adcIsConversionComplete(adcREG1,adcGROUP1))!=0){
        ch_count = adcGetData(adcREG1, adcGROUP1,&adc_data[0]);
        ch_count = ch_count;
        AllPowerSwitchSTatusGet();
        adcStopConversion(adcREG1,adcGROUP1);
    }

}


void InputInit(){
    MCP_Init(2, 0);
    for(int i=1 ; i<=16 ; i++){
        MCP_PinMode(i, 0);      //set second pin to be input
    }
}

void AngleToValue ( int Home , int KatSayi , int Aci ){
    AciSonuc = Home + ( KatSayi * (Aci) );
}

void ValueToAngle ( int Home , int KatSayi , int Sensor ){
    DegerSonuc =  ( Sensor - Home ) / KatSayi;
}

void HedefAci ( float Aci){
   uint16_t AngleValue = 41 * sin(Aci/68);
}

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
//    TI_Fee_EraseImmediateBlock(0x1);
//        do
//        {
//            TI_Fee_MainFunction();
//            delay();
//            Status=TI_Fee_GetStatus(0 );
//        }
//        while(Status!= IDLE);
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

    BlockNumber=0x1;
    TI_Fee_WriteAsync(Block_NO, &SpecialRamBlock[0]);
    do
    {
        TI_Fee_MainFunction();
        delay();
        Status=TI_Fee_GetStatus(0);
    }
    while(Status!=IDLE);

}

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
        /* Invalidate a written block  */
//        TI_Fee_InvalidateBlock(BlockNumber);
//        do
//        {
//            TI_Fee_MainFunction();
//            delay();
//            Status=TI_Fee_GetStatus(0);
//        }
//        while(Status!=IDLE);

        /* Format bank 7 */
        //TI_Fee_Format(0xA5A5A5A5U);
}
void canMessageCheck(){

    if(!canIsRxMessageArrived(canREG1, canMESSAGE_BOX1)){
        canGetData(canREG1, canMESSAGE_BOX1,can_rx_data);  /* receive on can2  */ //id 1
    }

    if( Semiautomaticmode ){

        sensorRight = can_rx_data[0];
        sensorLeft =  can_rx_data[1];
        PumpSlaveBtn =can_rx_data[2];

        if( !AcilStopON ){

            if( sensorRight == true && sensorLeft == true ){
                CanRedHighLow(true);
            }else{
                CanRedHighLow(false);
            }

            if( AttractiveLeftSensorAndCamelNeck >= 2500 ){
                CanYellowHighLow(true);
                AutoHizalamaState = true;

            }else if(AttractiveLeftSensorAndCamelNeck <= 150 ){
                CanYellowHighLow(false);
                AutoHizalamaState = false;
                AutoHizalama = false;
            }

        }
    }


    if( Fullautomaticmode ){

        //ValueToAngle(e2prom_read_value.CamelneckHome, e2prom_read_value.CamelneckNumber, AttractiveLeftSensorAndCamelNeck);
        //HedefAci(DegerSonuc);
        //AngleToValue(e2prom_read_value.FifthWheelHome, e2prom_read_value.FifthWheelNumber, AngleValue);

        canSensorData = can_rx_data[0];
        canSensorData <<= 8;
        canSensorData |= can_rx_data[1];

        PumpSlaveBtn =  can_rx_data[2];
//
//        if( Caliberundefined == true){
//
//            if( canSensorData >= (AciSonuc - 5)  && canSensorData <= (AciSonuc + 5) ){
//
//                AutoHizalamaState = false;
//
//                TekerlekDuzLed.set = DELAY_250MS;
//                TekerlekSagLed.set =0;
//                TekerlekSolLed.set =0;
//                CaliberBozuk.set = 0;
//                OnSensorAriza.set =0;
//                ArkaSensorAriza.set =0;
//
// //           }else if( canSensorData <  AciSonuc  && canSensorData > e2prom_read_value.FifthWheelLeftAngle){
//
//                AutoHizalamaState = true;
//
//                TekerlekDuzLed.set =0;
//                TekerlekSagLed.set =0;
//                TekerlekSolLed.set =DELAY_500MS;
//                CaliberBozuk.set = 0;
//                OnSensorAriza.set =0;
//                ArkaSensorAriza.set =0;
//
// //           }else if( canSensorData > AciSonuc  && canSensorData < e2prom_read_value.FifthWheelRightAngle ){
//                AutoHizalamaState = true;
//
//                TekerlekDuzLed.set =0;
//                TekerlekSagLed.set =DELAY_500MS;
//                TekerlekSolLed.set =0;
//                CaliberBozuk.set = 0;
//                OnSensorAriza.set =0;
//                ArkaSensorAriza.set =0;
//
////            }else if( canSensorData  < (e2prom_read_value.FifthWheelLeftAngle - 300)){
////                if(canSensorData  < (e2prom_read_value.FifthWheelLeftAngle - 350)){
//                    CaliberBozuk.set = DELAY_250MS;
//
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    OnSensorAriza.set =0;
//                    ArkaSensorAriza.set = 0;
//                //}else{
//                    CaliberBozuk.set = 0;
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    OnSensorAriza.set =0;
//                    ArkaSensorAriza.set = DELAY_250MS;
//                //}
////            }else if( canSensorData > (e2prom_read_value.FifthWheelRightAngle + 300)){
////                if(canSensorData  > (e2prom_read_value.FifthWheelRightAngle + 350)){
//                    CaliberBozuk.set = DELAY_250MS;
//
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    OnSensorAriza.set =0;
//                    ArkaSensorAriza.set = 0;
//                //}else{
//                    CaliberBozuk.set = 0;
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    OnSensorAriza.set =0;
//                    ArkaSensorAriza.set = DELAY_250MS;
//                //}
////            }else if( AttractiveLeftSensorAndCamelNeck < (e2prom_read_value.CamelneckLeftAngle - 300)){
////                if( AttractiveLeftSensorAndCamelNeck < (e2prom_read_value.CamelneckLeftAngle - 350) && AttractiveLeftSensorAndCamelNeck > e2prom_read_value.CamelneckLeftAngle){
//                    CaliberBozuk.set = DELAY_250MS;
//
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    ArkaSensorAriza.set =0;
//                    OnSensorAriza.set = 0;
//               // }else{
//                    CaliberBozuk.set = 0;
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    ArkaSensorAriza.set =0;
//                    OnSensorAriza.set = DELAY_250MS;
//               // }
////            }else if( AttractiveLeftSensorAndCamelNeck > (e2prom_read_value.CamelneckRightAngle + 300)){
////                if( AttractiveLeftSensorAndCamelNeck > (e2prom_read_value.CamelneckRightAngle + 350) && AttractiveLeftSensorAndCamelNeck < e2prom_read_value.CamelneckRightAngle){
//                    CaliberBozuk.set = DELAY_250MS;
//
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    ArkaSensorAriza.set =0;
//                    OnSensorAriza.set = 0;
//               // }else{
//                    CaliberBozuk.set = 0;
//                    TekerlekDuzLed.set =0;
//                    TekerlekSagLed.set =0;
//                    TekerlekSolLed.set =0;
//                    ArkaSensorAriza.set =0;
//                    OnSensorAriza.set = DELAY_250MS;
//               // }
//            }
//        }
    }

}

bool systemActiveCheck(){

        SysActive =  MCP_pinRead(GPO_SYSTEM_ACTIVE_PIN);

    return SysActive;
}

bool ISSCheck(){

        ISSActiveState = MCP_pinRead(MCP_GPB2);
        if(ISSActiveState){
           ISSSingalActive.set = DELAY_1S; //need to edit according to documents
        }
    return ISSActiveState;
}

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



void semiAutomaticPumpCheck(){
    if ( PumpSlaveBtn ){ //Button Push UP

        gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
        /* Can ikili dogrulama */
        //dikkat ali kodunda id 2 ama burada 4
        can_tx_data[4] = 1;
        canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
        /* Can ikili dogrulama */
        fullPumpState = true;
    }else{
        if( fullPumpState == true){

            gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, LOW);
            /* Can ikili dogrulama */
            can_tx_data[4] = 0;
            canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
            /* Can ikili dogrulama */
            fullPumpState = false;
        }
    }



}


void semiAutomaticAligmentCommandsCheck(){
    if( !AutoHizalama ){

          if ( AlignmentRight != MCP_pinRead(GIO_ALIGNMENT_RIGHT_BTN_PIN) ){
              if (AlignmentRight = MCP_pinRead(GIO_ALIGNMENT_RIGHT_BTN_PIN)){ //Button Push UP

                  gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, HIGH);
                  gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
                  AlignmentRightState = true;
              }else{
                  if( AlignmentRightState == true){

                      gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, LOW);
                      gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, LOW);
                      AlignmentRightState = false;
                  }
              }
          }//AlignmentRightBTN

          if ( AlignmentLeft != MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN) ){
              if (AlignmentLeft = MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN)){ //Button Push UP

                  gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, HIGH);
                  gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
                  AlignmentLeftState = true;
              }else{
                  if( AlignmentRightState == true){//

                      gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, LOW);
                      gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, LOW);
                      AlignmentLeftState = false;
                  }
              }
          }//AlignmentLeftBTN
      }// !AutoHizalama

      if( AutoHizalamaState ){

          if ( AlignmentAuto != MCP_pinRead(GIO_ALIGNMENT_AUTO_BTN_PIN)){
            if (AlignmentAuto = MCP_pinRead(GIO_ALIGNMENT_AUTO_BTN_PIN)){

                AutoHizalama = true;
                AlignmentAutoState = true;
            }else{
                if( AlignmentAutoState == true){
                    AlignmentAutoState = false;
                }
            }
          }
      }//AutoHizalama

      //Acil Stop Butonu
      if ( StopMode != MCP_pinRead(GIO_STOP_BTN_PIN) ){
        if (StopMode = MCP_pinRead(GIO_STOP_BTN_PIN)){

            CaliberAcilStopLed.set = DELAY_250MS;
            AcilStopON = true;
            StopModeState = true;
        }else{
            if( StopModeState == true){

                CaliberAcilStopLed.set = 0;
                Caliberundefined = true;
                AcilStopON = false;
                StopModeState = false;
            }
        }
      }//StopMode

}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void fullAutomaticPumpCheck(){

    if ( PumpSlaveBtn ){ //Button Push UP
         gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN , HIGH);

//         DataDizi[4] = 1;
//         CANMessageSet(CAN0_BASE, 2, &Can_TX_Message, MSG_OBJ_TYPE_TX);

         PumpState = true;
     }else{
         if( PumpState == true){
             gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN , LOW);

//             DataDizi[4] = 0;
//             CANMessageSet(CAN0_BASE, 2, &Can_TX_Message, MSG_OBJ_TYPE_TX);

             PumpState = false;
         }
     }
}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
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

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
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
    }
/*********************************************************************** AlignmentLeft button bottun pressed***/
    if ( AlignmentLeft != MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN) ){
        if ((AlignmentLeft = MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN)) ){

            aligment_buf = set_bit(aligment_buf, 1, 1);
            left_aligment = true;
        }else{
            aligment_buf = set_bit(aligment_buf, 1, 0);
            left_aligment = false;
        }
    }

/************************************************************************* Auto mode button bottun pressed***/
    if ( AlignmentAuto != MCP_pinRead(GIO_ALIGNMENT_AUTO_BTN_PIN) ){
        if ((AlignmentAuto = MCP_pinRead(GIO_ALIGNMENT_AUTO_BTN_PIN)) ){

            aligment_buf = set_bit(aligment_buf, 2, 1);
            auto_aligment = true;
        }else{
            aligment_buf = set_bit(aligment_buf, 2, 0);
            auto_aligment =false;
        }
    }
/************************************************************************ Set mode button bottun pressed***/
    if ( SetMode != MCP_pinRead(GIO_ALIGNMENT_SET_BTN_PIN) ){
        if ((SetMode = MCP_pinRead(GIO_ALIGNMENT_SET_BTN_PIN)) ){

            aligment_buf = set_bit(aligment_buf, 3, 1);
            set_aligment = true;
        }else{
            aligment_buf = set_bit(aligment_buf, 3, 0);
            set_aligment = false;
        }
    }
/************************************************************************ Emergency stop bottun pressed***/
    if ( StopMode != MCP_pinRead(GIO_STOP_BTN_PIN) ){
        if ((StopMode = MCP_pinRead(GIO_STOP_BTN_PIN)) ){

            aligment_buf = set_bit(aligment_buf, 4, 1);
            emergency_stop = true;
        }else{
            aligment_buf = set_bit(aligment_buf, 4, 0);
            emergency_stop = false;
            EmergencyStopActive.set = 0;
            gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
            gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red


        }
    }

}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void fullAutomaticAligmentCommandsCheck(){

    commandButtonscheck();


    switch (aligment_buf) {
/******************************************************************************** No bottun pressed***/
        case 0:
            gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, LOW);
            gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, LOW);
            gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, LOW);
            if(full_Automatic_Set_Aligment == false){
                //gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT, GIO_ALIGNMENT_WARNING_LAMP_RED_PIN, LOW);
                //gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT, GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN, LOW);
            }

            break;
/************************************************************************Right_valf bottun process***/
        case 1:
            if(emergency_stop == false && right_aligment == true){
                if(full_Automatic_Set_Aligment){
                    printf("right save\n");
                    setAligmentProcess();
                    //CalibrationSecondTemprorySettings.set = DELAY_1S;//need to edit accord the documents
                    CalibrationFirstTemprorySettings.set = 0;
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                }else{
                    gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, HIGH);
                    gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
                }
            }
            break;
/************************************************************************left_valf bottun process***/
        case 2:
            if(emergency_stop == false && left_aligment == true){
                if(full_Automatic_Set_Aligment){
                    printf("left save\n");
                    setAligmentProcess();
                    //CalibrationSecondTemprorySettings.set = DELAY_1S;//need to edit accord the documents

                    CalibrationFirstTemprorySettings.set = 0;
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red

                }else{
                    gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, HIGH);
                    gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
                }
            }

            break;
/***********************************************************************Calibrration Auto process***/
        case 4:
            if(emergency_stop == false && auto_aligment == true){
                if(full_Automatic_Set_Aligment){
                    if(aligment_set_level == 0){
                        printf("home save\n");
                        //take neckangle as  neck angle Home
                        //take canSensorData as FifthWheel Home
                        CamelneckHomeTemporary = AttractiveLeftSensorAndCamelNeck;  //from ads1018[2]
                        FifthWheelHomeTemporary = canSensorData;   //from canbus
                        printf("yellow lamp on keep\n");
                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                        CalibrationFirstTemprorySettings.set = DELAY_500MS;
                        aligment_set_level = 1;
                    }
                }else{
                    full_Automatic_AutoAlignment=true;
                }
            }
            break;
/***************************************************************************Calibration Set process***/
        case 8:
            if(emergency_stop == false){
                full_Automatic_Set_Aligment = true;
                CalibrationFailure.set = 0;
                Calibrationbegin.set = DELAY_500MS;

            }
            break;
/****************************************************************Set and Auto at same time process***/
        case 12:
            if(emergency_stop == false){
                if(aligment_set_level == 2){
                    full_Automatic_Set_Aligment = false;
                    printf("saved\n");

//                    CamelneckHomeTemporary = 1022;
//                    CamelneckRightAngleTemporary = 1000;
//                    CamelneckLeftAngleTemporary  = 1001;
//                    FifthWheelHomeTemporary      = 1002;
//                    FifthWheelRightAngleTemporary= 1018;
//                    FifthWheelLeftAngleTemporary = 1017;
//                    CamelneckNumberTemporary     = 10.56;
//                    FifthWheelNumberTemporary    = 10.30;
//
//                    EepromData[0]  = CamelneckHomeTemporary >> 8;
//                    EepromData[1]  = CamelneckHomeTemporary & 0x00FF;
//
//                    EepromData[2]  = CamelneckRightAngleTemporary >> 8;
//                    EepromData[3]  = CamelneckRightAngleTemporary & 0x00FF;
//
//                    EepromData[4]  = CamelneckLeftAngleTemporary >> 8;
//                    EepromData[5]  = CamelneckLeftAngleTemporary & 0x00FF;
//
//                    EepromData[6]  = FifthWheelHomeTemporary >> 8;
//                    EepromData[7]  = FifthWheelHomeTemporary & 0x00FF;
//
//                    EepromData[8]  = FifthWheelRightAngleTemporary >> 8;
//                    EepromData[9]  = FifthWheelRightAngleTemporary & 0x00FF;
//
//                    EepromData[10] = FifthWheelLeftAngleTemporary >> 8;
//                    EepromData[11] = FifthWheelLeftAngleTemporary & 0x00FF;
//
//                    CamelneckNumberTemporary =(uint16)(CamelneckNumberTemporary*100);
//                    EepromData[12] = (uint16)CamelneckNumberTemporary >> 8;
//                    EepromData[13] = (uint16)CamelneckNumberTemporary & 0x00FF;
//
//                    FifthWheelNumberTemporary = FifthWheelNumberTemporary*100;
//                    EepromData[14] = (uint16)FifthWheelNumberTemporary >> 8;
//                    EepromData[15] = (uint16)FifthWheelNumberTemporary & 0x00FF;
                    printf(" eeprom saved\n");


                    //epromWrite(0x1, EepromData);
                    eepromRead(0x1, read_data);

                    //use it for reading
                    CamelneckHomeTemporary      = (read_data[0] << 8)|read_data[1];
                    CamelneckRightAngleTemporary= (read_data[2] << 8)|read_data[3];
                    CamelneckLeftAngleTemporary = (read_data[4] << 8)|read_data[5];
                    FifthWheelHomeTemporary     = (read_data[6] << 8)|read_data[7];
                    FifthWheelRightAngleTemporary  = (read_data[8] << 8)|read_data[9];
                    FifthWheelLeftAngleTemporary   = (read_data[10] << 8)|read_data[11];
                    CamelneckNumberTemporary  = (read_data[12] << 8)|read_data[13];
                    CamelneckNumberTemporary  = CamelneckNumberTemporary/100;
                    FifthWheelNumberTemporary = (read_data[14] << 8)|read_data[15];
                    FifthWheelNumberTemporary = FifthWheelNumberTemporary/100;

                    aligment_set_level = 0;
                    printf("yellow and red lamp off\n");
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red
                    gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                    CalibrationFinalSettings.set = DELAY_250MS;
                }

            }else{

            }
            break;
/***********************************************************************emergency_stop process***/
        case 16:
            emergency_stop = true;
            Calibrationbegin.set = 0;
            CalibrationFirstTemprorySettings.set = 0;
            CalibrationSecondTemprorySettings.set = 0;
            CalibrationFinalSettings.set = 0;
            CalibrationCancel.set = 0;
            CalibrationFailure.set = 0;

            FifthWheelAnglePositioneLeft.set = 0;
            FifthWheelAnglePositioneRight.set = 0;
            VehicleInRoute.set = 0;

            AutomaticAligmentDone.set = 0;
            ISSSingalActive.set = 0;
            FrontSensorInoperative.set = 0;
            BackSensorInoperative.set = 0;
            aligment_set_level = 0;
            EmergencyStopActive.set = DELAY_250MS;
            break;
/***********************************************************************Others process***/
        default:
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
            ISSSingalActive.set = 0;
            FrontSensorInoperative.set = 0;
            BackSensorInoperative.set = 0;

            break;

    }
}

/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void fullAutomaticControlLeds(){
    if(tick){
        tick=false;
/***********************************************************************EmergencyStopActive****/
        if(EmergencyStopActive.set != 0){
            if(++EmergencyStopActive.count >= EmergencyStopActive.set){

                EmergencyStopActive.state=!EmergencyStopActive.state;
                EmergencyStopActive.count = 0;

               if(EmergencyStopActive.state){
                   printf("yellow and red lamp on x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red
                   EmergencyStopActive.counter++;
               }else{
                   printf("yellow and red lamp off x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red

                   if(Calibrationbegin.counter>=1){
                       EmergencyStopActive.set = 0;
                       EmergencyStopActive.counter = 0;
                   }
               }
           }//EmergencyStopActive
        }//EmergencyStopActive
/*******************************************************************FrontSensorInoperative*****/
        if(FrontSensorInoperative.set != 0){
            if(++FrontSensorInoperative.count >= FrontSensorInoperative.set){

                FrontSensorInoperative.state=!FrontSensorInoperative.state;
                FrontSensorInoperative.count = 0;

               if(FrontSensorInoperative.state){
                   printf("yellow lamp on x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                   FrontSensorInoperative.counter++;
               }else{
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red
                   printf("yellow lamp off x1 for 250ms\n");
                   if(FrontSensorInoperative.counter>=1){
                       FrontSensorInoperative.set = 0;
                       FrontSensorInoperative.counter = 0;
                   }
               }
           }
       }//FrontSensorInoperative
/********************************************************************BackSensorInoperative*****/
        if(BackSensorInoperative.set != 0){
            if(++BackSensorInoperative.count >= BackSensorInoperative.set){

                BackSensorInoperative.state=!BackSensorInoperative.state;
                BackSensorInoperative.count = 0;

               if(BackSensorInoperative.state){
                   printf("red lamp on x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   BackSensorInoperative.counter++;
               }else{
                   printf("red lamp off x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   if(BackSensorInoperative.counter>=1){
                       BackSensorInoperative.set = 0;
                       BackSensorInoperative.counter = 0;
                   }
               }
           }
       }//BackSensorInoperative
/*************************************************************************Calibrationbegin*****/

        if(Calibrationbegin.set != 0){
            if(++Calibrationbegin.count >= Calibrationbegin.set){

                Calibrationbegin.state=!Calibrationbegin.state;
                Calibrationbegin.count = 0;

               if(Calibrationbegin.state){
                   printf("yellow and red lamp on x1 for 500ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//yellow
                   Calibrationbegin.counter++;
               }else{
                   printf("yellow and red lamp off x1 for 500ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow
                   if(Calibrationbegin.counter>=1){
                       Calibrationbegin.set = 0;
                       Calibrationbegin.counter = 0;
                   }
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
                   printf("red lamp off x1 for 500ms and yellow keep\n");
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
                   printf("yellow and red lamp on keep\n");
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
                   printf("yellow and red lamp on 250 x3\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//yellow

                   CalibrationFinalSettings.counter++;
               }else{
                   printf("yellow and red lamp off 250 x3\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow
                   if(CalibrationFinalSettings.counter>=3){
                       printf("red lamp off x1 for 500ms and yellow off after finish calibration\n");
                       printf("yellow and red lamp off  after finish calibration\n");
                       CalibrationFinalSettings.set = 0;
                       CalibrationFinalSettings.counter = 0;
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
                   printf("yellow lamp on 500 x2  //red lamp on 2saniye x1 \n");
                   CalibrationCancel.counter++;
               }else{
                   printf("yellow lamp off 500 x2  //red lamp on 2saniye x1 \n");
                   if(CalibrationCancel.counter>=3){
                       printf("yellow lamp off 500 x2  //red lamp off 2saniye x1 \n");
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
                   printf("yellow and red lamp on x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//red
                   CalibrationFailure.counter++;
               }else{
                   printf("yellow and red lamp off x1 for 250ms\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//red

                   if(CalibrationFailure.counter>=1){
                       CalibrationFailure.counter = 0;
                   }
               }
           }//CalibrationFailure
        }//CalibrationFailure
/***************************************************************************ISSSingalActive*****/

        if(ISSSingalActive.set != 0){
            if(++ISSSingalActive.count >= ISSSingalActive.set){

                ISSSingalActive.state=!ISSSingalActive.state;
                ISSSingalActive.count = 0;
                gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//yellow
               if(ISSSingalActive.state){
                   printf("yellow and red lamp on 250 x1\n");

                   ISSSingalActive.counter++;
               }else{
                   printf("yellow and red lamp off 250 x1\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow
                   if(ISSSingalActive.counter>=1){
                       ISSSingalActive.set = 0;
                       ISSSingalActive.counter = 0;
                   }
               }
           }
       }//ISSSingalActive
/***********************************************************************AutomaticAligmentDone****/

        if(AutomaticAligmentDone.set != 0){
            if(++AutomaticAligmentDone.count >= AutomaticAligmentDone.set){

                AutomaticAligmentDone.state=!AutomaticAligmentDone.state;
                AutomaticAligmentDone.count = 0;

               if(AutomaticAligmentDone.state){
                   printf("yellow and red lamp on 250 x2\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,HIGH);//yellow
                   AutomaticAligmentDone.counter++;
               }else{
                   printf("yellow and red lamp off 250 x2\n");
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                   gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_YELLOW_PIN ,LOW);//yellow
                   if(AutomaticAligmentDone.counter>=2){
                       AutomaticAligmentDone.set = 0;
                       AutomaticAligmentDone.counter = 0;
                   }
               }
           }
       }//AutomaticAligmentDone
/***********************************************************************AutomaticAligmentDone****/

        if(VehicleInRoute.set != 0){
            if(++VehicleInRoute.count >= VehicleInRoute.set){

                VehicleInRoute.state=!VehicleInRoute.state;
                VehicleInRoute.count = 0;

               if(VehicleInRoute.state){
                   //printf("yellow and red lamp on keep\n");
                   VehicleInRoute.counter++;
               }else{
                   //printf("yellow and red lamp on keep\n");
                   if(VehicleInRoute.counter>=1){
                       VehicleInRoute.set = 0;
                       VehicleInRoute.counter = 0;
                   }
               }
           }
       }//VehicleInRoute

    }//tick

}
/*-----------------------------------------------------------------------------------------------------------------------------------------*/
void semiAutomaticControlLeds( void ){

    if( AutoHizalamaBitti.set !=0 ){
        if( ++AutoHizalamaBitti.count >= AutoHizalamaBitti.set ){

            AcilStopON = true;
            AutoHizalamaBitti.count =0;
            AutoHizalamaBitti.state = !AutoHizalamaBitti.state;

            if( AutoHizalamaBitti.counter <=1 ){
                if( AutoHizalamaBitti.state ){
                    CanYellowHighLow(false);
                    CanRedHighLow(false);
                }else{
                    CanYellowHighLow(true);
                    CanRedHighLow(true);
                    AutoHizalamaBitti.counter++;
                }

                canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
            }else{
                AutoHizalama = false;
                AcilStopON = false;
                canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4

                AutoHizalamaBitti.state =0;
                AutoHizalamaBitti.set =0;
            }
        }
    }//AutoHizalamaBitti.set

    if( CaliberAcilStopLed.set != 0){
        if( ++CaliberAcilStopLed.count >= CaliberAcilStopLed.set){

            CaliberAcilStopLed.count =0;
            CaliberAcilStopLed.state = !CaliberAcilStopLed.state;

            CaliberStartLed.set = 0;
            CaliberStopLed.set = 0;
            CaliberBirinciKayitLed.set =0;
            CaliberikinciKayitLed.set =0;
            CaliberKesinKayitLed.set =0;
            CaliberAcilStopLed.set =0;

            TekerlekSagLed.set =0;
            TekerlekSolLed.set =0;
            TekerlekDuzLed.set =0;
            CaliberBozuk.set =0;
            AutoHizalamaBitti.set =0;

            AutoHizalama = false;
            Caliberundefined = false;

            gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
            gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
            gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN,LOW);

            CanYellowHighLow(false);
            CanRedHighLow(false);

            if( CaliberAcilStopLed.state){
                CanRedHighLow(true);
                CanYellowHighLow(false);
            }else{
                CanRedHighLow(false);
                CanYellowHighLow(true);
            }

            canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
        }//CaliberAcilStopLed.count
    }//CaliberAcilStopLed.set
}



void semiAutomaticCommandsCheck(){
    semiAutomaticPumpCheck();
    semiAutomaticAligmentCommandsCheck();
}

void fullAutomaticCommandCheck(){
    if( full_Automatic_AutoAlignment == false ){ // Otomatik Hizalama
        fullAutomaticPumpCheck();
        fullAutomaticAligmentCommandsCheck();
        }
}
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

                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);//red
                    }else{
                        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//red
                        Fullautomaticled.counter++;
                    }
                    canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
                }else{
                    Fullautomaticmode = true;
                    Semiautomaticmode = false;
                    KalibrasyonStart.set = 55;

                    KalibrasyonStop.set = 55;
                    KalibrasyonKesinKayit.set = 50;

                    CaliberStartLed.set = 0;
                    CaliberStopLed.set = 0;
                    CaliberBirinciKayitLed.set =0;
                    CaliberikinciKayitLed.set =0;
                    CaliberKesinKayitLed.set =0;
                    CaliberAcilStopLed.set =0;


                    TekerlekSagLed.set =0;
                    TekerlekSolLed.set =0;
                    TekerlekDuzLed.set =0;
                    CaliberBozuk.set =0;
                    SystemActiveCaliber = true;

                    AutoHizalamaBitti.set =0;
                    AutoHizalamaBitti.counter =0;

                    Fullautomaticled.state =0;
                    Fullautomaticled.set =0;
                }
            }
        }//fullautonmaticwarning


    }//tick



}

void semiAutomaticAutoHizalamaProcess(){
    if( sensorLeft == 1 && sensorRight == 1){
        gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
        gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
        gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,LOW);

        AutoHizalamaBitti.set = 5;
        AutoHizalamaBitti.counter =0;
        AutoHizalama = false;

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

        AutoHizalama = false;
    }
}

void semiAutomaticStandByMode(){

    ISSAktive = true;
    AcilStopON = true;

    gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN,LOW);
    gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN,LOW);
    gioSetBit(GIO_PUMP_PORT,GIO_PUMP_PIN,LOW);
    gioSetBit(GIO_OIL_PUMP_PORT, GIO_OIL_PUMP_PIN,LOW);

    CanYellowHighLow(false);
    CanRedHighLow(false);
    can_tx_data[2] = 0;
    canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
}

void fullAutomaticStandBymode(){
//    CaliberStartLed.set = 0;
//    CaliberStopLed.set = 0;
//    CaliberBirinciKayitLed.set =0;
//    CaliberikinciKayitLed.set =0;
//    CaliberKesinKayitLed.set =0;
//    CaliberAcilStopLed.set =0;
//    CaliberBozuk.set =0;
//
//    TekerlekSagLed.set =0;
//    TekerlekSolLed.set =0;
//    TekerlekDuzLed.set =0;
//    CaliberBozuk.set =0;
//
//    BigAngle = true;
//    SmallAngle = true;
//    AutoHizalama = false;
//    SystemActiveCaliber = true;
//
//    gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, LOW);
//    gioSetBit(GIO_ALIGNMENT_VALF_RIGHT_PORT, GIO_ALIGNMENT_VALF_RIGHT_PIN, LOW);
//    gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN,LOW);
//    CanYellowHighLow(false);
//    CanRedHighLow(false);
}

void fullAutomaticAutoAlignmentProcess(){

    full_Automatic_AutoAlignment = false;
    printf(" red and yellow lamp on");

}
///ADS not working check global counter maybe its timer is not working
void main(void)
{
    HardwareInit();
    SPIdevicesinit();
    InputInit();
    OutputInit();

    if( MCP_pinRead(GIO_FUNCTION_SELECT_PIN) ){
        Fullautomaticled.set = DELAY_250MS;
        //fullAutomatic = true;
    }else{
        Semiautomaticled.set = DELAY_250MS;
        //semiAutomatic = false;
    }
    Calibrationbegin.set = 0;
    CalibrationFirstTemprorySettings.set = 0;
    CalibrationSecondTemprorySettings.set = 0;
    CalibrationFinalSettings.set = 0;
    EmergencyStopActive.set =0;
    CalibrationFailure.set = DELAY_250MS;
//
//    /*******************************************************/
//    //eeprom
//    /*******************************************************/
//
//


     /***********************************************************/


    while(1) /* ... continue forever */
    {
//        myinput[0]=MCP_pinRead(MCP_GPA0);
//        myinput[1]=MCP_pinRead(MCP_GPA1);
//        myinput[2]=MCP_pinRead(MCP_GPA2);
//        myinput[3]=MCP_pinRead(MCP_GPA3);
//        myinput[4]=MCP_pinRead(MCP_GPA4);
//        myinput[5]=MCP_pinRead(MCP_GPA5);
//        myinput[6]=MCP_pinRead(MCP_GPA6);
//        myinput[7]=MCP_pinRead(MCP_GPA7);
//        myinput[8]=MCP_pinRead(MCP_GPB0);
//        myinput[9]=MCP_pinRead(MCP_GPB2);
//        myinput[10]=MCP_pinRead(MCP_GPB3);
//        myinput[11]=MCP_pinRead(MCP_GPB4);
//        myinput[12]=MCP_pinRead(MCP_GPB5);
//        myinput[13]=MCP_pinRead(MCP_GPB6);
//        myinput[14]=MCP_pinRead(MCP_GPB7);

        //ISSActiveState = MCP_pinRead(MCP_GPB2);

        modeWarning();

 /************************************************************************************************************************************/
 /******************************************************************************fullAutomaticmode*****************************************************/


        if( Fullautomaticmode ){
            if(systemActiveCheck()&& ISSCheck()!=true){

                if(full_Automatic_AutoAlignment){
                    fullAutomaticAutoAlignmentProcess();
                }else{
                    fullAutomaticCommandCheck();
                    fullAutomaticControlLeds();
                }//full_Automatic_AutoAlignment

            }else{
                fullAutomaticStandBymode();
            }//systemActiveCheck

        }//Fullautomaticmode

 /************************************************************************************************************************************/
 /******************************************************************************Semiautomaticmode*****************************************************/

        if(Semiautomaticmode){
            can_tx_data[3] = 0;
            canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
            if(systemActiveCheck()&& ISSCheck()!=true){

                gioSetBit(GIO_OIL_PUMP_PORT, GIO_OIL_PUMP_PIN, HIGH);
                can_tx_data[2] = 255;
                canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4

                if( AutoHizalama ){
                    semiAutomaticAutoHizalamaProcess();
                }

                if(tick){
                    tick = false;
                    semiAutomaticCommandsCheck();
                    semiAutomaticControlLeds();
                }

            }else{
                semiAutomaticStandByMode();
            }

        }//Semiautomaticmode

    }//while(1)
}//main


void rtiNotification(uint32 notification)
{
//    if(notification ==rtiNOTIFICATION_COMPARE0){
        global_counter++;
        tick=true;
        canMessageCheck();
        getSensorAndCamelNeckValue();
        MCUADCread();
//        printf("%d \n",1);
//    }

//    if(notification ==rtiNOTIFICATION_COMPARE1){
//        tick=true;
//        printf("%d \n",2);
//
//    }
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
