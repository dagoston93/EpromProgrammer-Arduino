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
      // Disconnect request
      if(opCode == DISCONNECT_REQUEST){
        Serial.write(OK);
        isPcConnected = false;
      }
      // Read request
      else if(opCode == READ_REQUEST){
        uint8_t initialData[8];
        Serial.readBytes(initialData, 8);

        uint32_t numOfBytes = 0;
        uint32_t startAddress = 0;

        numOfBytes |= (uint32_t)initialData[0] << 24;
        numOfBytes |= (uint32_t)initialData[1] << 16;
        numOfBytes |= (uint32_t)initialData[2] <<  8;
        numOfBytes |= (uint32_t)initialData[3];

        startAddress |= (uint32_t)initialData[4] << 24;
        startAddress |= (uint32_t)initialData[5] << 16;
        startAddress |= (uint32_t)initialData[6] <<  8;
        startAddress |= (uint32_t)initialData[7];

        // Check if requested num of bytes is within bounds
        if((startAddress + numOfBytes) <= MEM_SIZE){
          // We are good to go
          Serial.write(OK);

          uint8_t data = 0;

          delay(1000);
          // Now read the requested number of bytes
          uint32_t lastAddress = (startAddress + numOfBytes) - 1;
          for(uint32_t address = startAddress; address <= lastAddress; address++){
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
