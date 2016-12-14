/* part of MCP23017_LIB */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include "mcp23017.h"

const uint8_t MCP_ADDRESS = 0x20;
#define ODROID_INT_PIN 0 // External interrupt pin

// Two pins at the MCP (Ports A/B where some buttons have been setup.)
// Buttons connect the pin to grond, and pins are pulled up.
uint8_t mcpPinA=7;
uint8_t mcpPinB=15;

void setup(){

  printf("MCP23017 Interrupt Test\n");

  pinMode(ODROID_INT_PIN, INPUT);
  pullUpDnControl(ODROID_INT_PIN, PUD_UP);

  mcp_begin(MCP_ADDRESS);
  
  // We mirror INTA and INTB, so that only one line is required between MCP and Arduino for int reporting
  // The INTA/B will not be Floating 
  // INTs will be signaled with a LOW
  mcp_setupInterrupts(true,false,LOW);

  // configuration for a button on port A
  // interrupt will triger when the pin is taken to ground by a pushbutton
  mcp_pinMode(mcpPinA, INPUT);
  mcp_pullUp(mcpPinA, HIGH);  // turn on a 100K pullup internally
  mcp_setupInterruptPin(mcpPinA,FALLING); 

  // similar, but on port B.
  mcp_pinMode(mcpPinB, INPUT);
  mcp_pullUp(mcpPinB, HIGH);  // turn on a 100K pullup internall
  mcp_setupInterruptPin(mcpPinB,FALLING);
}

void handleInterrupt(){
  
  // Get more information from the MCP from the INT
  uint8_t pin=mcp_getLastInterruptPin();
  uint8_t val=mcp_getLastInterruptPinValue();

  printf("handleInterrupt(), ping = %d val = %x\n", pin, val);
  
  // we have to wait for the interrupt condition to finish.
  // An action is required to clear the INT flag, and allow it to trigger again.
  // see datasheet for datails.
  while( ! (mcp_digitalRead(mcpPinB) && mcp_digitalRead(mcpPinA) ));
}

int main()
{
    if (wiringPiSetup () < 0) {
       printf ("Unable to setup wiringPi: %s\n", strerror (errno));
       return -1;
    }

#if 0 // need?
    fp = popen("echo 87 > /sys/class/gpio/export\n", "r");
    pclose(fp);
    fp = popen("echo \"in\" > /sys/class/gpio/gpio87/direction\n", "r");
    pclose(fp);
    fp = popen("chmod 0666 /sys/class/gpio/gpio87/value\n", "r");
    pclose(fp);
#endif

    if (wiringPiISR (ODROID_INT_PIN, INT_EDGE_BOTH, &handleInterrupt) < 0 ) {
       printf ("Unable to setup ISR: %s\n", strerror (errno));
       return -1;
    }

    while(1) {
       usleep(1000);
    }

    return 0;
}

