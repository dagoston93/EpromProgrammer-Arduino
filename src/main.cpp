#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include <Wire.h>
#include "SerialComm.h"

Adafruit_MCP23017 mcp;

/**
 * At the moment we only support the TMS27C010A-12
 */
#define EPROM_A16 7
#define MEM_SIZE 131072

/**
 * Global variables
 */
bool isPcConnected = false;

void Eprom_SetAddress(uint32_t address);

void setup() {
  // Init I2C and Serial communication
  mcp.begin();
  Serial.begin(500000);

  // Set all the pins of the MCP23017 to OUTPUT (theese are the address pins 0-15)
  for(uint8_t i = 0; i<16; i++){
    mcp.pinMode(i, OUTPUT);
    mcp.digitalWrite(i, LOW);
  }

  // Set the last address pin (A16) to output
  pinMode(EPROM_A16, OUTPUT);
  digitalWrite(EPROM_A16, LOW);

  // Set all pins of PORTB as inputs
  DDRB = 0x00;
}

void loop() {
  // Check if we have data available on the serial port
  if(Serial.available() > 0){
    // Read the OpCode first
    uint8_t opCode;
    Serial.readBytes(&opCode, 1);

    if(isPcConnected){
      // Read request
      if(opCode == READ_REQUEST){
        uint8_t dataSize[4];
        Serial.readBytes(dataSize, 4);
        uint32_t numOfBytes = 0;

        numOfBytes |= dataSize[0] << 24;
        numOfBytes |= dataSize[1] << 16;
        numOfBytes |= dataSize[2] <<  8;
        numOfBytes |= dataSize[3];

        // Check if requested num of bytes is within bounds
        if(numOfBytes <= MEM_SIZE){
          // We are good to go
          Serial.write(OK);

          uint8_t data = 0;

          // Now read the requested number of bytes
          for(uint32_t address = 0; address < numOfBytes; address++){
            Eprom_SetAddress(address);
            data = PINB;
            Serial.write(data);
          }
        }else{
          // Requested data too long, send error
          Serial.write(DATA_SIZE_ERROR);
        }
      }

    }else{
      // Pc is not connected so we are waiting for connection request
      if(opCode == CONNECT_REQUEST){
        // On request we connect to PC
        Serial.write(CONNECT_ACCEPT);
        isPcConnected = true;
      }
    }
  }
}

/**
 * @brief Sets the A0-A16 pins of the EPROM
 * @param address The address to set.
 */
void Eprom_SetAddress(uint32_t address){
  // Separate the lower and the upper part of the addresss
  uint16_t addressLower16 = (uint16_t)(address & 0xFFFF);
  uint8_t addressLastBit = (uint8_t) (address >> 16);

  // Set the address bits of the EPROM chip
  mcp.writeGPIOAB(addressLower16);
  digitalWrite(EPROM_A16, (addressLastBit > 0) ? HIGH : LOW);
}
