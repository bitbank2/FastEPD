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

// GPIO modes
#ifndef ARDUINO
#define DISABLED 0
#define INPUT 1
#define INPUT_PULLUP 2
#define OUTPUT 3
#endif

void bbepPinMode(int iPin, int iMode)
{
    gpio_config_t io_conf = {};

    gpio_reset_pin((gpio_num_t)iPin);
    if (iMode == DISABLED) return;
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << iPin);
    io_conf.pull_down_en = (gpio_pulldown_t)0; //disable pull-down mode
    io_conf.pull_up_en = (gpio_pullup_t)(iMode == INPUT_PULLUP); // pull-up mode
    if (iMode == INPUT || iMode == INPUT_PULLUP) {
        io_conf.mode = GPIO_MODE_INPUT;
    } else { // must be output
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    gpio_config(&io_conf); //configure GPIO with the given settings
} /* bbepPinMode() */

//
// Initialize the Wire library on the given SDA/SCL GPIOs
//
int bbepI2CInit(uint8_t sda, uint8_t scl)
{
    Wire.end();
    Wire.begin(sda, scl);
    Wire.setClock(400000);
    Wire.setTimeout(100);
    return BBEP_SUCCESS;
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

#endif // __BB_EP_IO__
