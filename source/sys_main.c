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
uint16 myinput[8];
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

bool SetMode = false;
bool StopMode = false;
bool ISS = false;

bool SetModeState = false;
bool StopModeState = false;
bool StopModeStateTwo = false;

bool Caliberundefined = false;
bool SystemActiveCaliber = true;


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

bool tick=false;
bool tick2=false;
bool Fullautomaticmode = false;
bool Semiautomaticmode = false;
uint16_t AttractiveRightSensor; // Sensor 1
uint16_t AttractiveLeftSensorAndCamelNeck; // Sensor 2
//ADS1018 spi config

//adc
adcData_t adc_data[16];
void wait(uint32 time);

    uint64 ch_count=0;
    uint32 id    =0;
    uint64 value =0;

//canbus
#define  D_SIZE 9

uint8  can_tx_data[D_SIZE];
uint8  can_rx_data[D_SIZE] = {0};
uint32 error = 0;

uint8 sensorRight;
uint8 sensorLeft;


//PCA129
uint16 *Dateinfo;

//SST25PF040C
uint16_t adress_value = 0;

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
    }else{
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);//yellow
    }
}

void CanRedHighLow ( bool state ){

    if( state ){
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,HIGH);
        //DataDizi[1] = (255) & 0xFF;
    }else{
        gioSetBit(GIO_ALIGNMENT_WARNING_LAMP_RED_PORT,GIO_ALIGNMENT_WARNING_LAMP_RED_PIN ,LOW);
       // DataDizi[1] = (0) & 0xFF;
    }
}

void MCUADCread(){
    adcStartConversion(adcREG1,adcGROUP1);

    while((adcIsConversionComplete(adcREG1,adcGROUP1))==0);
    ch_count = adcGetData(adcREG1, adcGROUP1,&adc_data[0]);
    ch_count = ch_count;
    AllPowerSwitchSTatusGet();

    adcStopConversion(adcREG1,adcGROUP1);
}


void InputInit(){
    MCP_Init(2, 0);
    for(int i=1 ; i<=16 ; i++){
        MCP_PinMode(i, 0);      //set second pin to be input
    }

}

void canMessageCheck(){


    if(!canIsRxMessageArrived(canREG1, canMESSAGE_BOX1)){
        canGetData(canREG1, canMESSAGE_BOX1, &sensorRight);  /* receive on can2  */
    }
    if(!canIsRxMessageArrived(canREG1, canMESSAGE_BOX2)){
        canGetData(canREG1, canMESSAGE_BOX2, &sensorLeft);  /* receive on can2  */
    }
    if(!canIsRxMessageArrived(canREG1, canMESSAGE_BOX3)){
        canGetData(canREG1, canMESSAGE_BOX3, &PumpSlaveBtn);  /* receive on can2  */
    }
        //error = checkPackets(&can_tx_data[0],&can_rx_data[0],D_SIZE);

        if( Semiautomaticmode ){
            if( !AcilStopON ){
                       if( sensorRight == true && sensorLeft == true ){

                           CanRedHighLow(true);
                       }else{
                           CanRedHighLow(false);
                       }


                       //aliye sor 2500 ve 150 ne demek
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

}

bool systemActiveCheck(){
    if(MCP_pinRead(GPO_SYSTEM_ACTIVE_PIN)){
        SysActive = true;
    }
    return SysActive;
}

bool ISSCheck(){
    if( MCP_pinRead(GIO_ISS_PIN) ){
        ISSActiveState = true;
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
}



void pumpCheck(){
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

void AligmentCommandsCheck(){
    if( !AutoHizalama ){

          if ( AlignmentRight != MCP_pinRead(GIO_ALIGNMENT_RIGHT_BTN_PIN)){

              if ((AlignmentRight = MCP_pinRead(GIO_ALIGNMENT_RIGHT_BTN_PIN)) ){ //Button Push UP

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
          }

          if ( AlignmentLeft != MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN)){

              if ((AlignmentLeft = MCP_pinRead(GIO_ALIGNMENT_LEFT_BTN_PIN)) ){ //Button Push UP

                  gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, HIGH);
                  gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, HIGH);
                  AlignmentLeftState = true;
              }else{
                  if( AlignmentRightState == true){

                      gioSetBit(GIO_ALIGNMENT_VALF_LEFT_PORT, GIO_ALIGNMENT_VALF_LEFT_PIN, LOW);
                      gioSetBit(GIO_PUMP_PORT, GIO_PUMP_PIN, LOW);
                      AlignmentLeftState = false;
                  }
              }
          }
      }
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
      }

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
      }

}


void controlLeds_Semi( void ){

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
                //ali burada ne gonderecegini belirlemedi
                //can_tx_data[4] = 0;
                canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
            }else{
                AutoHizalama = false;
                AcilStopON = false;
                //ali burada ne gonderecegini belirlemedi
                //can_tx_data[4] = 0;
                canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4

                AutoHizalamaBitti.state =0;
                AutoHizalamaBitti.set =0;
            }
        }
    }

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
        }
    }
}

void commandsCheck(){
    pumpCheck();
    AligmentCommandsCheck();
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

void AutoHizalamaProcess(){
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

void standByMode(){

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

    while(1) /* ... continue forever */
    {
        modeWarning();

        if(Semiautomaticmode){
            can_tx_data[3] = 0;
            canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4
            if(systemActiveCheck()&& !ISSCheck()){

                gioSetBit(GIO_OIL_PUMP_PORT, GIO_OIL_PUMP_PIN, HIGH);
                can_tx_data[2] = 255;
                canTransmit(canREG1, canMESSAGE_BOX4, can_tx_data);//id = 4

                if( AutoHizalama ){
                    AutoHizalamaProcess();
                }

                if(tick){
                    tick = false;
                    commandsCheck();
                    controlLeds_Semi();
                }


            }else{
                standByMode();
            }

        }//Semiautomaticmode

    }//while(1)
}//main



void wait(uint32 time)
{
    while(time){time--;};
}

void rtiNotification(uint32 notification)
{
//    if(notification ==rtiNOTIFICATION_COMPARE0){
        global_counter++;
        tick=true;
        canMessageCheck();
        getSensorAndCamelNeckValue();
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
