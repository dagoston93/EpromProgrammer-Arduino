#include <Arduino.h>
#include <Adafruit_MCP23017.h>
#include <Wire.h>
#include "SerialComm.h"

Adafruit_MCP23017 mcp;

/**
 * At the moment we only support the TMS27C010A-12
 */
#define EPROM_A16 30
#define EPROM_PGM 31
#define EPROM_CHIP_ENABLE 32
#define EPROM_OUTPUT_ENABLE 33
#define EPROM_PIN_30 34
#define PIN_LED1 35
#define PIN_LED2 36
#define PIN_LED3 37
#define BUZZER_PIN 44

#define ANALOG_SENS1 A15
#define ANALOG_SENS2 A14

#define MEM_SIZE 131072

/**
 * Global variables
 */
bool isPcConnected = false;

void Eprom_SetAddress(uint32_t address);
uint16_t ReorderAddress(uint16_t address);

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

  // Set all pins of PORTA as inputs
  DDRA = 0x00;

  // For now only reading is implemented -> Set the Chip Enable and Output Enable pins low,
  // PGM to high
  pinMode(EPROM_CHIP_ENABLE, OUTPUT);
  pinMode(EPROM_OUTPUT_ENABLE, OUTPUT);
  pinMode(EPROM_PGM, OUTPUT);
  pinMode(EPROM_PIN_30, OUTPUT);

  digitalWrite(EPROM_CHIP_ENABLE, LOW);
  digitalWrite(EPROM_OUTPUT_ENABLE, LOW);
  digitalWrite(EPROM_PGM, HIGH);
  digitalWrite(EPROM_PIN_30, LOW);

  // Turn off LEDs
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);

  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, LOW);
  digitalWrite(PIN_LED3, LOW);
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
            delayMicroseconds(1000);

            data = PINA;

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
  digitalWrite(EPROM_A16, (addressLastBit > 0) ? HIGH : LOW);

  addressLower16 = ReorderAddress(addressLower16);

  mcp.writeGPIOAB(addressLower16);
}

/**
 * @brief Reorders the address bits according to schematics. (Bottom 16 bits)
 * @param address The address to reorder.
 */
uint16_t ReorderAddress(uint16_t address){

  uint8_t addrMap[] = {
    15, 12, 10, 14, 11,  9,  8, 13,
     7,  6,  5,  4,  3,  2,  1,  0
  };

  uint16_t reordered = 0;

  for(int i = 0; i < 16; i++){
    uint16_t mask = 1 << addrMap[i];
    uint16_t bit = address & mask;

    reordered |= bit;
  }

  return reordered;
}
