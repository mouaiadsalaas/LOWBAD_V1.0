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

#include "MCP23S17.h"
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
 *4.SPI1 CONNECTED DEVICE IS  : ADC2129 Real Time Clock
 *5.SPI2 CONNECTED DEVICE IS  : MCP23S18  GIO  EXPANDER
 *6.SPI3 CONNECTED DEVICE IS  : ADS1018 ANALOG EXPANDER
 *
 *
 */

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

uint16 VNQ5050KTRE_LEFT_1STAT1_VAL;             //ADIN[11]
uint16 VNQ5050KTRE_LEFT_1STAT2_VAL;             //AIDN[3]
uint16 VNQ5050KTRE_LEFT_1STAT3_VAL;             //ADIN[2]
uint16 VNQ5050KTRE_LEFT_1STAT4_VAL;             //ADIN[10]

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

uint16 VNQ5050KTRE_RIGHT_2STAT1_VAL;           //ADIN[8]
uint16 VNQ5050KTRE_RIGHT_2STAT2_VAL;           //ADIN[6]
uint16 VNQ5050KTRE_RIGHT_2STAT3_VAL;           //ADIN[5]
uint16 VNQ5050KTRE_RIGHT_2STAT4_VAL;           //ADIN[4]

/********************************************************2 channel POWER SWITCHE(LEFT)***************************************************************/
#define VND5T100AJTRE_LEFT_INT1_PIN            2
#define VND5T100AJTRE_LEFT_INT1_PORT           hetPORT1

#define VND5T100AJTRE_LEFT_INT2_PIN            5
#define VND5T100AJTRE_LEFT_INT2_PORT           gioPORTA

#define VND5T100AJTRE_LEFT_FRSTBY_PIN          22           //PIN TO DO FAULT RESET
#define VND5T100AJTRE_LEFT_FRSTBY_PORT         hetPORT1

uint16 VNQ5050KTRE_LEFT_1CS1_VAL;             //ADIN[9]
uint16 VNQ5050KTRE_LEFT_1CS2_VAL;             //ADIN[1]

/********************************************************2 channel POWER SWITCHE(RIGHT)***************************************************************/
#define VND5T100AJTRE_RIGHT_INT1_PIN            6
#define VND5T100AJTRE_RIGHT_INT1_PORT           gioPORTA

#define VND5T100AJTRE_RIGHT_INT2_PIN            7
#define VND5T100AJTRE_RIGHT_INT2_PORT           gioPORTA

#define VND5T100AJTRE_RIGHT_FRSTBY_PIN          22           //PIN TO DO FAULT RESET
#define VND5T100AJTRE_RIGHT_FRSTBY_PORT         hetPORT1

uint16 VNQ5050KTRE_RIGHT_2CS1_VAL;             //ADIN[0]
uint16 VNQ5050KTRE_RIGHT_2CS2_VAL;             //ADIN[7]

/********************************************************1 channel POWER SWITCHE(BOTTOM CENTER)***************************************************************/
#define VN5T006ASPTRE_BOTTOM_INT1_PIN            0
#define VN5T006ASPTRE_BOTTOM_INT1_PORT           hetPORT1

#define VN5T006ASPTRE_BOTTOM_FRSTBY_PIN          22           //PIN TO DO FAULT RESET
#define VN5T006ASPTRE_BOTTOM_FRSTBY_PORT         hetPORT1

uint16 VN5T006ASPTRE_BOTTOM_CS1_VAL;            //ADIN[21]

/********************************************************1 channel POWER SWITCHE(BOTTOM RIGHT)***************************************************************/
#define NCV8403ADTRKG_RIGHT_INT1_PIN            4
#define NCV8403ADTRKG_RIGHT_INT1_PORT           hetPORT1

/********************************************************1 channel POWER SWITCHE(BOTTOM LEFT)***************************************************************/
#define NCV8403ADTRKG_LEFT_INT1_PIN            6
#define NCV8403ADTRKG_LEFT_INT1_PORT           hetPORT1

//ADS1018 spi config
spiDAT1_t dataconfig2_t;


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
uint16_t rcvData[4];

uint16_t global_counter=0;


//adc
adcData_t adc_data[16];
void wait(uint32 time);

    uint64 ch_count=0;
    uint32 id    =0;
    uint64 value =0;

//canbus
#define  D_SIZE 9

uint8  tx_data[D_SIZE]  = {'H','E','R','C','U','L','E','S','\0'};
uint8  rx_data[D_SIZE] = {0};
uint32 error = 0;


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

void SST25PF040CInit(){
    dataconfig4_t.CS_HOLD = TRUE;
    dataconfig4_t.WDEL    = TRUE;
    dataconfig4_t.DFSEL   = SPI_FMT_2;
    dataconfig4_t.CSNR    = 0xFE;
}
/*******************************************************************************************/
//functions
void HardwareInit(){
    hetInit();
    gioInit();
    adcInit();
    spiInit();

}

void RTIInit(){
    rtiInit();
    /* Set high end timer GIO port hetPort pin direction to all output */

    /* Enable RTI Compare 0 interrupt notification */
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0);

    /* Enable IRQ - Clear I flag in CPS register */
    /* Note: This is usually done by the OS or in an svc dispatcher */
    _enable_IRQ();

    /* Start RTI Counter Block 0 */
    rtiStartCounter(rtiCOUNTER_BLOCK0);
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
    dataconfig2_t.DFSEL   = SPI_FMT_3;//8 bit icin fmt2
    dataconfig2_t.CSNR    = 0xFE;
}



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
                    spiReceiveData(spiREG3,  &dataconfig2_t, 1, (uint16_t*)&rcvData[currentStep]);
                    spiReceiveData(spiREG3,  &dataconfig2_t, 1, (uint16_t*)&rcvData[currentStep]);
                    subStep = successStep;
            }else{
                    Register2write();
            }

    }
}

void PowerSwitchesTest(){
    //4 channel POWER SWITCHE(LEFT) HIGHT
    gioSetBit(VNQ5050KTRE_LEFT_INT1_PORT, VNQ5050KTRE_LEFT_INT1_PIN, HIGH);
    gioSetBit(VNQ5050KTRE_LEFT_INT2_PORT, VNQ5050KTRE_LEFT_INT2_PIN, HIGH);
    gioSetBit(VNQ5050KTRE_LEFT_INT3_PORT, VNQ5050KTRE_LEFT_INT3_PIN, HIGH);
    gioSetBit(VNQ5050KTRE_LEFT_INT4_PORT, VNQ5050KTRE_LEFT_INT4_PIN, HIGH);
    gioSetBit(VNQ5050KTRE_LEFT_STAT_DIS_PORT, VNQ5050KTRE_LEFT_STAT_DIS_PIN, 0);        //1 to disable status

    //2 channel POWER SWITCHE(LEFT) HIGHT
    gioSetBit(VND5T100AJTRE_LEFT_INT1_PORT, VND5T100AJTRE_LEFT_INT1_PIN, HIGH);
    gioSetBit(VND5T100AJTRE_LEFT_INT2_PORT, VND5T100AJTRE_LEFT_INT2_PIN, HIGH);

    //1 channel POWER SWITCHE(BOTTOM LEFT) HIGHT
    gioSetBit(NCV8403ADTRKG_LEFT_INT1_PORT, NCV8403ADTRKG_LEFT_INT1_PIN, HIGH);

    //1 channel POWER SWITCHE(BOTTOM CENTER) HIGHT
    gioSetBit(VN5T006ASPTRE_BOTTOM_INT1_PORT, VN5T006ASPTRE_BOTTOM_INT1_PIN, HIGH);
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

void MCUADCread(){
    adcStartConversion(adcREG1,adcGROUP1);

    while((adcIsConversionComplete(adcREG1,adcGROUP1))==0);
    ch_count = adcGetData(adcREG1, adcGROUP1,&adc_data[0]);
    ch_count = ch_count;
    AllPowerSwitchSTatusGet();

    adcStopConversion(adcREG1,adcGROUP1);
}


void MCP23S18SetAllInput(){

    for(int i=1 ; i<=16 ; i++){
        MCP_PinMode(i, 0);      //set second pin to be input
    }

}

uint16 myinput[8];
uint8_t result;

void main(void)
{
    HardwareInit();

    SPIMCPInit();
    SPIADS1018Init();
    SPIPCA2129Init();
    SST25PF040CInit();
    MCP_Init(2, 0);
    MCP23S18SetAllInput();
    PowerSwitchesTest();

    /* initialize can 1    */
    canInit(); /* can1 */

    SetDate(50, 59, 23, 31, 6, 12, 21);
    //dikkat
    //dikkat
    //dikkat flash fonctionsss do not put it inside while loop
    //dikkat flash fonctionsss do not put it inside while loop
    //dikkat flash fonctionsss do not put it inside while loop
    //dikkat flash fonctionsss do not put it inside while loop
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    Read_JEDEK();
    Read_Id();
    ChipErase();
    Write(0x11, 0x00, 0x00,0x13);
    adress_value = Read(0x11, 0x00, 0x00);
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////
    //dikkat flash fonctionsss do not put it inside while loop
    //dikkat flash fonctionsss do not put it inside while loop
    //dikkat flash fonctionsss do not put it inside while loop
    //dikkat flash fonctionsss do not put it inside while loop
    //dikkat flash fonctionsss do not put it inside while loop




    while(1) /* ... continue forever */
    {          MCUADCread();
               ADS1018Adcread();
               myinput[0]=MCP_pinRead(1);
               myinput[1]=MCP_pinRead(2);
               myinput[2]=MCP_pinRead(3);
               myinput[3]=MCP_pinRead(4);
               myinput[4]=MCP_pinRead(5);

               canTransmit(canREG1, canMESSAGE_BOX1, tx_data);

               if(!canIsRxMessageArrived(canREG1, canMESSAGE_BOX2)){
                   canGetData(canREG1, canMESSAGE_BOX2, rx_data);  /* receive on can2  */

                   error = checkPackets(&tx_data[0],&rx_data[0],D_SIZE);
               }
               Dateinfo      = GetDate();


    }
}



void wait(uint32 time)
{
    while(time){time--;};
}

void rtiNotification(uint32 notification)
{
    global_counter++;
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
