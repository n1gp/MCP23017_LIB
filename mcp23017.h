/*************************************************** 
 This is a library for the MCP23017 i2c port expander
 Rick Koch / N1GP December 14, 2016

 I grabbed this from an Arduino github project and converted
 it from C++ to C and from Adruino to linux.
 
 Originally from: https://github.com/pkourany/Adafruit_MCP23017_IDE
   These displays use I2C to communicate, 2 pins are required to
   interface
   Adafruit invests time and resources providing this open source code,
   please support Adafruit and open-source hardware by purchasing
   products from Adafruit!

   Written by Limor Fried/Ladyada for Adafruit Industries.
   BSD license, all text above must be included in any redistribution
 
   Adapted for Spark Core by Paul Kourany, Sept 3, 2014
 ****************************************************/

#ifndef _MCP23017_H_
#define _MCP23017_H_

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define CHANGE 0
#define FALLING 1
#define RISING 2

void mcp_begin(uint8_t addr);

void mcp_pinMode(uint8_t p, uint8_t d);
void mcp_digitalWrite(uint8_t p, uint8_t d);
void mcp_pullUp(uint8_t p, uint8_t d);
uint8_t mcp_digitalRead(uint8_t p);

void mcp_writeGPIOAB(uint16_t);
uint16_t mcp_readGPIOAB();
uint8_t mcp_readGPIO(uint8_t b);

void mcp_setupInterrupts(uint8_t mirroring, uint8_t open, uint8_t polarity);
void mcp_setupInterruptPin(uint8_t p, uint8_t mode);
uint8_t mcp_getLastInterruptPin();
uint8_t mcp_getLastInterruptPinValue();

#define MCP23017_ADDRESS 0x20

// registers
#define MCP23017_IODIRA 0x00
#define MCP23017_IPOLA 0x02
#define MCP23017_GPINTENA 0x04
#define MCP23017_DEFVALA 0x06
#define MCP23017_INTCONA 0x08
#define MCP23017_IOCONA 0x0A
#define MCP23017_GPPUA 0x0C
#define MCP23017_INTFA 0x0E
#define MCP23017_INTCAPA 0x10
#define MCP23017_GPIOA 0x12
#define MCP23017_OLATA 0x14


#define MCP23017_IODIRB 0x01
#define MCP23017_IPOLB 0x03
#define MCP23017_GPINTENB 0x05
#define MCP23017_DEFVALB 0x07
#define MCP23017_INTCONB 0x09
#define MCP23017_IOCONB 0x0B
#define MCP23017_GPPUB 0x0D
#define MCP23017_INTFB 0x0F
#define MCP23017_INTCAPB 0x11
#define MCP23017_GPIOB 0x13
#define MCP23017_OLATB 0x15

#define MCP23017_INT_ERR 255

#endif
