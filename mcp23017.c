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

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "mcp23017.h"

// LOCAL routines and vars
static int deviceFd;

/**
 * Bit number associated to a give Pin
 */
static uint8_t bitForPin(uint8_t pin){
	return pin%8;
}

/**
 * Register address, port dependent, for a given PIN
 */
static uint8_t regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr){
	return(pin<8) ?portAaddr:portBaddr;
}

/**
 * Reads a given register
 */
static uint8_t readRegister(uint8_t regAddr){
	// Read the register
	return wiringPiI2CReadReg8(deviceFd, regAddr);
}


/**
 * Writes a given register
 */
static void writeRegister(uint8_t regAddr, uint8_t regValue){
	// Write the register
	wiringPiI2CWriteReg8(deviceFd, regAddr, regValue);
}


/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
static void updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
	uint8_t regValue;
	uint8_t regAddr=regForPin(pin,portAaddr,portBaddr);
	uint8_t bit=bitForPin(pin);
	regValue = readRegister(regAddr);

	// set the value for the particular bit
	bitWrite(regValue,bit,pValue);

	writeRegister(regAddr,regValue);
}

////////////////////////////////////////////////////////////////////////////////
// PUBLIC routines

/**
 * > 0  Initializes the MCP23017 given its HW selected address, see datasheet for Address selection.
 * == 0 Initializes the default MCP23017, with 000 for the configurable part of the address
 */
void mcp_begin(uint8_t addr) {
//?	addr = (addr > 7) ? 7 : addr;

	// Begin I2C
        deviceFd = wiringPiI2CSetup (addr);

	// set defaults!
	// all inputs on port A and B
	writeRegister(MCP23017_IODIRA,0xff);
	writeRegister(MCP23017_IODIRB,0xff);
}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void mcp_pinMode(uint8_t p, uint8_t d) {
	updateRegisterBit(p,(d==INPUT),MCP23017_IODIRA,MCP23017_IODIRB);
}

/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 */
uint16_t mcp_readGPIOAB() {
	uint16_t readValue = 0;

	// read the current GPIO output latches
	readValue = wiringPiI2CReadReg16(deviceFd, MCP23017_GPIOA);
	readValue = (readValue & 0x00FF) << 8 | ((readValue & 0xFF00) >> 8);

	return readValue;
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * Parameter b should be 0 for GPIOA, and 1 for GPIOB.
 */
uint8_t mcp_readGPIO(uint8_t b) {
	// read the current GPIO output latches
	return wiringPiI2CReadReg8(deviceFd, (b == 0)?MCP23017_GPIOA:MCP23017_GPIOB);
}

/**
 * Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void mcp_writeGPIOAB(uint16_t writeValue) {
        uint16_t writeWord;

        writeWord = (writeValue & 0x00FF) << 8;
        writeWord |= ((writeValue & 0xFF00) >> 8);
        wiringPiI2CWriteReg16(deviceFd, MCP23017_GPIOA, writeWord);
}

void mcp_digitalWrite(uint8_t pin, uint8_t d) {
	uint8_t gpio;
	uint8_t bit=bitForPin(pin);


	// read the current GPIO output latches
	uint8_t regAddr=regForPin(pin,MCP23017_OLATA,MCP23017_OLATB);
	gpio = readRegister(regAddr);

	// set the pin and direction
	bitWrite(gpio,bit,d);

	// write the new GPIO
	regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	writeRegister(regAddr,gpio);
}

void mcp_pullUp(uint8_t p, uint8_t d) {
	updateRegisterBit(p,d,MCP23017_GPPUA,MCP23017_GPPUB);
}

uint8_t mcp_digitalRead(uint8_t pin) {
	uint8_t bit=bitForPin(pin);
	uint8_t regAddr=regForPin(pin,MCP23017_GPIOA,MCP23017_GPIOB);
	return (readRegister(regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false,flase, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 */
void mcp_setupInterrupts(uint8_t mirroring, uint8_t openDrain, uint8_t polarity){
	// configure the port A
	uint8_t ioconfValue=readRegister(MCP23017_IOCONA);
	bitWrite(ioconfValue,6,mirroring);
	bitWrite(ioconfValue,2,openDrain);
	bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONA,ioconfValue);

	// Configure the port B
	ioconfValue=readRegister(MCP23017_IOCONB);
	bitWrite(ioconfValue,6,mirroring);
	bitWrite(ioconfValue,2,openDrain);
	bitWrite(ioconfValue,1,polarity);
	writeRegister(MCP23017_IOCONB,ioconfValue);
}

/**
 * Set's up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
 *
 */
void mcp_setupInterruptPin(uint8_t pin, uint8_t mode) {

	// set the pin interrupt control (0 means change, 1 means compare against given value);
	updateRegisterBit(pin,(mode!=CHANGE),MCP23017_INTCONA,MCP23017_INTCONB);
	// if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

	// In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
	// In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
	updateRegisterBit(pin,(mode==FALLING),MCP23017_DEFVALA,MCP23017_DEFVALB);

	// enable the pin for interrupt
	updateRegisterBit(pin,HIGH,MCP23017_GPINTENA,MCP23017_GPINTENB);

}

uint8_t mcp_getLastInterruptPin(){
	uint8_t intf, i;

	// try port A
	intf=readRegister(MCP23017_INTFA);
	for(i=0;i<8;i++) if (bitRead(intf,i)) return i;

	// try port B
	intf=readRegister(MCP23017_INTFB);
	for(i=0;i<8;i++) if (bitRead(intf,i)) return i+8;

	return MCP23017_INT_ERR;

}
uint8_t mcp_getLastInterruptPinValue(){
	uint8_t intPin=getLastInterruptPin();
	if(intPin!=MCP23017_INT_ERR){
		uint8_t intcapreg=regForPin(intPin,MCP23017_INTCAPA,MCP23017_INTCAPB);
		uint8_t bit=bitForPin(intPin);
		return (readRegister(intcapreg)>>bit) & (0x01);
	}

	return MCP23017_INT_ERR;
}
