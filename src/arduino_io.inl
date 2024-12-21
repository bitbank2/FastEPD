//
// bb_epaper I/O wrapper functions for Arduino
// Copyright (c) 2024 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 9/11/2024
//
// Use of this software is governed by the Business Source License
// included in the file ./LICENSE.
//
// As of the Change Date specified in that file, in accordance with
// the Business Source License, use of this software will be governed
// by the Apache License, Version 2.0, included in the file
// ./APL.txt.
//
// Adapt these functions to whatever target platform you're using
// and the rest of the code can remain unchanged
//
#ifndef __BB_EP_IO__
#define __BB_EP_IO__

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
// foreward references
void bbepWakeUp(BBEPDISP *pBBEP);

//
// Initialize the GPIO pins and SPI for use by bb_eink
//
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS, uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed)
{
    pBBEP->iDCPin = u8DC;
    pBBEP->iCSPin = u8CS;
    pBBEP->iMOSIPin = u8MOSI;
    pBBEP->iCLKPin = u8SCK;
    pBBEP->iRSTPin = u8RST;
    pBBEP->iBUSYPin = u8BUSY;

    pinMode(pBBEP->iDCPin, OUTPUT);
    pinMode(pBBEP->iRSTPin, OUTPUT);
    digitalWrite(pBBEP->iRSTPin, LOW);
    delay(100);
    digitalWrite(pBBEP->iRSTPin, HIGH);
    delay(100);
    if (pBBEP->iBUSYPin != 0xff) {
        pinMode(pBBEP->iBUSYPin, INPUT);
    }
    pBBEP->iSpeed = u32Speed;
#ifdef ARDUINO_ARCH_ESP32
    SPI.begin(pBBEP->iCLKPin, -1, pBBEP->iMOSIPin, pBBEP->iCSPin);
#else
    pinMode(pBBEP->iCSPin, OUTPUT);
    digitalWrite(pBBEP->iCSPin, HIGH); // we have to manually control the CS pin
    SPI.begin(); // other architectures have fixed SPI pins
#endif
    SPI.beginTransaction(SPISettings(u32Speed, MSBFIRST, SPI_MODE0));
    SPI.endTransaction(); // N.B. - if you call beginTransaction() again without a matching endTransaction(), it will hang on ESP32
} /* bbepInitIO() */
//
// Convenience function to write a command byte along with a data
// byte (it's single parameter)
//
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2)
{
    if (!pBBEP->is_awake) {
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    digitalWrite(pBBEP->iDCPin, LOW);
#ifndef ARDUINO_ARCH_ESP32
    digitalWrite(pBBEP->iCSPin, LOW);
#endif
    SPI.transfer(cmd1);
    digitalWrite(pBBEP->iDCPin, HIGH);
    SPI.transfer(cmd2); // second byte is data
#ifndef ARDUINO_ARCH_ESP32
    digitalWrite(pBBEP->iCSPin, HIGH);
#endif
//    digitalWrite(pBBEP->iDCPin, HIGH); // leave data mode as the default
} /* bbepCMD2() */
//
// Write a single byte as a COMMAND (D/C set low)
//
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd)
{
    if (!pBBEP->is_awake) {
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    digitalWrite(pBBEP->iDCPin, LOW);
#ifndef ARDUINO_ARCH_ESP32
    digitalWrite(pBBEP->iCSPin, LOW);
#endif
    SPI.transfer(cmd);
#ifndef ARDUINO_ARCH_ESP32
    digitalWrite(pBBEP->iCSPin, HIGH);
#endif
    digitalWrite(pBBEP->iDCPin, HIGH); // leave data mode as the default
} /* bbepWriteCmd() */
//
// Write 1 or more bytes as DATA (D/C set high)
//
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
//    digitalWrite(pBBEP->iDCPin, HIGH);
#ifdef ARDUINO_ARCH_ESP32
    SPI.transferBytes(pData, NULL, iLen);
#else
    if (pBBEP->iFlags & BBEP_CS_EVERY_BYTE) {
        for (int i=0; i<iLen; i++) { // Arduino clobbers the data (duplex)
            digitalWrite(pBBEP->iCSPin, LOW);
            SPI.transfer(pData[i]);
            digitalWrite(pBBEP->iCSPin, HIGH);
        }
    } else {
        digitalWrite(pBBEP->iCSPin, LOW);
        for (int i=0; i<iLen; i++) { // Arduino clobbers the data (duplex)
            SPI.transfer(pData[i]);
        }
        digitalWrite(pBBEP->iCSPin, HIGH);
    }
#endif
} /* bbepWriteData() */
//
// Initialize the Wire library on the given SDA/SCL GPIOs
//
int bbepI2CInit(BBEPDISP *pBBEP)
{
    Wire.end();
    Wire.begin(pBBEP->io_config.u8SDA, pBBEP->io_config.u8SCL);
    Wire.setClock(400000);
    Wire.setTimeout(100);
} /* bbepI2CInit() */

int bbepI2CWrite(unsigned char iAddr, unsigned char *pData, int iLen)
{
    int rc = 0;
    Wire.beginTransmission(iAddr);
    Wire.write(pData, (unsigned char)iLen);
    rc = !Wire.endTransmission();
    return rc;
}

int bbepI2CRead(unsigned char iAddr, unsigned char *pData, int iLen)
{
int i = 0;

    Wire.requestFrom(iAddr, (unsigned char)iLen);
    while (i < iLen && Wire.available()) {
        pData[i++] = Wire.read();
    }
    return i;
}
int bbepI2CReadRegister(unsigned char iAddr, unsigned char u8Register, unsigned char *pData, int iLen)
{
  int i;

      Wire.beginTransmission(iAddr);
      Wire.write(u8Register);
      Wire.endTransmission();
      Wire.requestFrom(iAddr, (unsigned char)iLen);
      while (i < iLen && Wire.available()) {
          pData[i++] = Wire.read();
      }
    return i;
}
//
// Test if a device responds at the given address
// returns 1 if present, 0 if not
//
int bbepI2CTest(uint8_t addr)
{
    int response;
  // We use the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(addr);
  response = !Wire.endTransmission();
  return response;
}

uint8_t TPS65186PowerGood(void)
{
uint8_t ucTemp[4];

    bbepI2CReadRegister(0x48, 0x0f, ucTemp, 1);
    return ucTemp[0];
}

void TPS65186Init(void)
{
    uint8_t ucTemp[8];
    ucTemp[0] = 0x9; // power up sequence register
    ucTemp[1] = 0x1b; // power up sequence
    ucTemp[2] = 0; // power up delay (3ms per rail)
    ucTemp[3] = 0x1b; // power down seq
    ucTemp[4] = 0; // power down delay (6ms per rail);
    bbepI2CWrite(0x48, ucTemp, 5);
}

#endif // __BB_EP_IO__
