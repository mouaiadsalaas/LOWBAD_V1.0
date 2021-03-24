#include <MCP23S17.h>

/****************************************************************
 *  Header File: MCP23S17.h
 *  Description: MCP23S17 expander with HERCULES TMS570ls0432 library header source
 *  Date       : 18.02.2021
 *  Author     : MOUAIAD SALAAS & ALI YOLCU
 *     ---------- ---------- ----------------------------
 *
 * Copyright MOUAIAD SALAAS & ALI YOLCU, 2021
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
uint32_t _modeCache   = 0xFFFF;                // Default I/O mode is all input, 0xFFFF
uint32_t _outputCache = 0x0000;                // Default output state is all off, 0x0000
uint32_t _pullupCache = 0x0000;                // Default pull-up state is all off, 0x0000
uint32_t _invertCache = 0x0000;                // Default input inversion state is not inverted, 0x0000


uint8_t CS_PIN=0;
uint8_t SPI_PORT_NUMBER=0;
/*************************************************************
 * Function Definitions

**************************************************************/
void spiWrite( uint16_t data ){
    switch (SPI_PORT_NUMBER) {
        case 1:
            spiTransmitData(spiREG1, &dataconfig1_t, 1, &data);
            break;

        case 2:
            spiTransmitData(spiREG2, &dataconfig1_t, 1, &data);
            break;

        case 3:
            spiTransmitData(spiREG3, &dataconfig1_t, 1, &data);
            break;

        default:
            break;
    }
}

uint8_t spiWriteRead( uint16_t txdata){
    uint16_t rxdata;
    switch (SPI_PORT_NUMBER) {
        case 1:
            spiTransmitAndReceiveData(spiREG1, &dataconfig1_t, 1,&txdata, &rxdata);
            break;

        case 2:
            spiTransmitAndReceiveData(spiREG2, &dataconfig1_t, 1,&txdata, &rxdata);
            break;

        case 3:
            spiTransmitAndReceiveData(spiREG3, &dataconfig1_t, 1,&txdata, &rxdata);
            break;

        default:
            break;
    }
    return (uint8_t)rxdata;
}

// GENERIC WORD WRITE - will write a word to a register pair, LSB to first register, MSB to next higher value register
void wordWrite(uint8_t reg, unsigned int word) {  // Accept the start register and word

    gioSetBit(SPI_CS_PORT, SPI_CS_PIN, HIGH);
    gioSetBit(SPI_CS_PORT, SPI_CS_PIN, LOW);

    spiWrite(ADDR_OPCODEW);

    spiWrite(reg);

    spiWrite((uint8_t) (word));
    spiWrite((uint8_t) (word >> 8));

    gioSetBit(SPI_CS_PORT, SPI_CS_PIN, HIGH);
}

// GENERIC BYTE WRITE - will write a byte to a register, arguments are register address and the value to write
void byteWrite(uint8_t reg, uint8_t value) {

    gioSetBit(SPI_CS_PORT, 0, HIGH);
    gioSetBit(SPI_CS_PORT, 0, LOW);

    spiWrite(ADDR_OPCODEW);
    spiWrite(reg);
    spiWrite(value);

    gioSetBit(SPI_CS_PORT, 0, HIGH);
}

// MODE SETTING FUNCTION - BY Port
void PortMode(unsigned int mode) {
  wordWrite(IODIRA, mode);
  _modeCache = mode;
}

// MODE SETTING FUNCTION - BY PIN
void MCP_PinMode(uint8_t pin, uint8_t mode) {  // Accept the pin # and I/O mode
  if (pin < 1 | pin > 16) return;               // If the pin value is not valid (1-16) return, do nothing and return
  if (mode == 0) {                          // Determine the mode before changing the bit state in the mode cache
    _modeCache |= 1 << (pin - 1);               // Since input = "HIGH", OR in a 1 in the appropriate place
  } else {
    _modeCache &= ~(1 << (pin - 1));            // If not, the mode must be output, so and in a 0 in the appropriate place
  }
  wordWrite(IODIRA, _modeCache);                // Call the generic word writer with start register and the mode cache
}

// WRITE FUNCTION - BY PIN
void MCP_PinOutput(uint8_t pin, uint8_t value) {
  if (pin < 1 | pin > 16) return;
  if (pin < 1 | pin > 16) return;
  if (value) {
    _outputCache |= 1 << (pin - 1);
  } else {
    _outputCache &= ~(1 << (pin - 1));
  }
  wordWrite(GPIOA, _outputCache);
}

// WEAK PULL-UP SETTING FUNCTIONS - BY WORD AND BY PIN
void MCP_PortpullupMode(unsigned int mode) {
  wordWrite(GPPUA, mode);
  _pullupCache = mode;
}

// INPUT INVERSION SETTING FUNCTIONS - BY WORD AND BY PIN
void MCP_PortinputInvert(unsigned int mode) {
  wordWrite(IPOLA, mode);
  _invertCache = mode;
}

// This function will read all 16 bits of I/O, and return them as a word in the format 0x(portB)(portA)
unsigned int MCP_PortRead(void) {
  unsigned int value = 0;                   // Initialize a variable to hold the read values to be returned

  gioSetBit(SPI_CS_PORT, SPI_CS_PIN, LOW);
  spiWrite(ADDR_OPCODER);
  spiWrite(GPIOA);
  value=spiWriteRead(0x00);
  value |= (spiWriteRead(0x00) << 8);

  gioSetBit(SPI_CS_PORT, SPI_CS_PIN, HIGH);
  return value;
}

uint8_t MCP_pinRead(uint8_t pin) {                    // Return a single bit value, supply the necessary bit (1-16)
    if (pin < 1 | pin > 16) return 0x0;                    // if the pin value is not valid (1-16) return, do nothing and return
    return MCP_PortRead() & (1 << (pin - 1)) ? HIGH : LOW;  // Call the word reading function, extract HIGH/LOW information from the requested pin
}

void MCP_Init(uint8_t SPI_PORT_NO, uint8_t CS) {                           // function to set Configurations to MCP23x17 Configuration Register

    byteWrite(IOCON, ADDR_ENABLE);                  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing
    CS_PIN=CS;
    SPI_PORT_NUMBER=SPI_PORT_NO;

}

