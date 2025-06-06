//
// bb_epdiy I/O wrapper functions for Arduino
// Copyright (c) 2024 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================
//

// Adapt these functions to whatever target platform you're using
// and the rest of the code can remain unchanged
//
#ifndef __BB_EP_IO__
#define __BB_EP_IO__

// Since the Espressif I2C driver seems to corrupt memory with it's frequent allocs and frees, use bit banging
#define BIT_BANG

#ifdef BIT_BANG
static uint8_t u8SDA_Pin, u8SCL_Pin;
static int iDelay = 1;
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "rom/ets_sys.h"
#ifndef ARDUINO
#include "driver/gpio.h"
#include "esp_timer.h"

// GPIO modes
#define memcpy_P memcpy
#define pgm_read_byte(a) (*(uint8_t *)a)
#define pgm_read_word(a) (*(uint16_t *)a)
#define pgm_read_dword(a) (*(uint32_t *)a)
#define PROGMEM 
#define HIGH 1
#define LOW 0
#define DISABLED 0
#define INPUT 1
#define INPUT_PULLUP 2
#define OUTPUT 3
#define INPUT_PULLDOWN 4

unsigned long micros(void)
{
    return (unsigned long)(esp_timer_get_time());
}
unsigned long millis(void)
{
    return micros() / 1000;
}

void delayMicroseconds(uint32_t us)
{
    ets_delay_us(us);
}

void delay(uint32_t ms)
{
    if (ms >= 10) {
        vTaskDelay(ms/10);
    }
    delayMicroseconds((ms % 10) * 1000);
}
#endif // !ARDUINO

void bbepPinMode(int iPin, int iMode)
{
    gpio_config_t io_conf = {};

    gpio_reset_pin((gpio_num_t)iPin);
    if (iMode == DISABLED) return;
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL << iPin);
    io_conf.pull_down_en = (gpio_pulldown_t)(iMode == INPUT_PULLDOWN); // pull-down mode
    io_conf.pull_up_en = (gpio_pullup_t)(iMode == INPUT_PULLUP); // pull-up mode
    if (iMode == OUTPUT) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    } else { // must be input
        io_conf.mode = GPIO_MODE_INPUT;
    }
    gpio_config(&io_conf); //configure GPIO with the given settings
} /* bbepPinMode() */

#ifdef BIT_BANG
uint8_t SDA_READ(void)
{
    return gpio_get_level((gpio_num_t)u8SDA_Pin);
}
void SDA_HIGH(void)
{
    bbepPinMode(u8SDA_Pin, INPUT_PULLDOWN);
}
void SDA_LOW(void)
{
    bbepPinMode(u8SDA_Pin, OUTPUT);
    gpio_set_level((gpio_num_t)u8SDA_Pin, 0);
}
void SCL_HIGH(void)
{
    bbepPinMode(u8SCL_Pin, INPUT_PULLDOWN);
}
void SCL_LOW(void)
{
    bbepPinMode(u8SCL_Pin, OUTPUT);
    gpio_set_level((gpio_num_t)u8SCL_Pin, 0);
}
void I2CSetSpeed(int iSpeed)
{
    if (iSpeed >= 400000) iDelay = 1;
    else if (iSpeed >= 100000) iDelay = 10;
    else iDelay = 20;
}

// Transmit a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//

int i2cByteOut(uint8_t b)
{
uint8_t i, ack;

for (i=0; i<8; i++)
{
//    delayMicroseconds(iDelay);
    if (b & 0x80)
      SDA_HIGH(); // set data line to 1
    else
      SDA_LOW(); // set data line to 0
    b <<= 1;
//    delayMicroseconds(iDelay);
    SCL_HIGH(); // clock high (slave latches data)
    delayMicroseconds(iDelay);
    SCL_LOW(); // clock low
    delayMicroseconds(iDelay);
} // for i
//delayMicroseconds(iDelay);
// read ack bit
SDA_HIGH(); // set data line for reading
//delayMicroseconds(iDelay);
SCL_HIGH(); // clock line high
delayMicroseconds(iDelay); // DEBUG - delay/2
ack = SDA_READ();
//delayMicroseconds(iDelay);
SCL_LOW(); // clock low
delayMicroseconds(iDelay); // DEBUG - delay/2
SDA_LOW(); // data low
return (ack == 0); // a low ACK bit means success
} /* i2cByteOut() */
//
// Receive a byte and read the ack bit
// if we get a NACK (negative acknowledge) return 0
// otherwise return 1 for success
//
uint8_t i2cByteIn(uint8_t bLast)
{
uint8_t i;
uint8_t b = 0;

     SDA_HIGH(); // set data line as input
     for (i=0; i<8; i++)
     {
         delayMicroseconds(iDelay); // wait for data to settle
         SCL_HIGH(); // clock high (slave latches data)
         delayMicroseconds(iDelay);
         b <<= 1;
         if (SDA_READ() != 0) // read the data bit
           b |= 1; // set data bit
         SCL_LOW(); // clock low
     } // for i
     if (bLast)
        SDA_HIGH(); // last byte sends a NACK
     else
     SDA_LOW();
     //     delayMicroseconds(iDelay);
          SCL_HIGH(); // clock high
          delayMicroseconds(iDelay);
          SCL_LOW(); // clock low to send ack
          delayMicroseconds(iDelay);
     //     SDA_HIGH();
          SDA_LOW(); // data low
       return b;
     } /* i2cByteIn() */
     //
     // Send I2C STOP condition
     //
     void i2cEnd(void)
     {
        SDA_LOW(); // data line low
        delayMicroseconds(iDelay);
        SCL_HIGH(); // clock high
        delayMicroseconds(iDelay);
        SDA_HIGH(); // data high
        delayMicroseconds(iDelay);
     } /* i2cEnd() */
     int i2cBegin(uint8_t addr, uint8_t bRead)
     {
        int rc;
     //   SCL_HIGH();
     //   delayMicroseconds(iDelay);
        SDA_LOW(); // data line low first
        delayMicroseconds(iDelay);
        SCL_LOW(); // then clock line low is a START signal
        addr <<= 1;
        if (bRead)
           addr++; // set read bit
        rc = i2cByteOut(addr); // send the slave address and R/W bit
        return rc;
     } /* i2cBegin() */
     void I2CWrite(uint8_t addr, uint8_t *pData, int iLen)
     {
     uint8_t b;
     int rc;
     
        i2cBegin(addr, 0);
        rc = 1;
        while (iLen && rc == 1)
        {
           b = *pData++;
           rc = i2cByteOut(b);
           if (rc == 1) // success
           {
              iLen--;
           }
        } // for each byte
        i2cEnd();
     //return (rc == 1) ? (iOldLen - iLen) : 0; // 0 indicates bad ack from sending a byte
     } /* I2CWrite() */
     int I2CRead(uint8_t addr, uint8_t *pData, int iLen)
     {
        i2cBegin(addr, 1);
        while (iLen--)
        {
           *pData++ = i2cByteIn(iLen == 0);
        } // for each byte
        i2cEnd();
        return 1;
     } /* I2CRead() */
     int I2CTest(uint8_t addr)
     {
     int response = 0;
     
        if (i2cBegin(addr, 0)) // try to write to the given address
        {
           response = 1;
        }
        i2cEnd();
     return response;
     } /* I2CTest() */
#endif // BIT_BANG                       
//
// Initialize the Wire library on the given SDA/SCL GPIOs
//
int bbepI2CInit(uint8_t sda, uint8_t scl)
{
#ifdef BIT_BANG
    u8SDA_Pin = sda;
    u8SCL_Pin = scl;
//    if (iSpeed >= 400000) iDelay = 1;
//    else if (iSpeed >= 100000) iDelay = 10;
//    else iDelay = 20;
#else
#ifdef ARDUINO
    Wire.end();
    Wire.begin(sda, scl);
    Wire.setClock(400000);
    Wire.setTimeout(100);
#else
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0; 
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
#endif
#endif // BIT_BANG
    return BBEP_SUCCESS;
} /* bbepI2CInit() */

int bbepI2CWrite(unsigned char iAddr, unsigned char *pData, int iLen)
{
#ifdef BIT_BANG
    I2CWrite(iAddr, pData, iLen);
    return 1;
#else
#ifdef ARDUINO
    int rc = 0;
    Wire.beginTransmission(iAddr);
    Wire.write(pData, (unsigned char)iLen);
    rc = !Wire.endTransmission();
    return rc;
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
       // ESP_LOGE("bb_epdiy", "insufficient memory for I2C transaction");
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (iAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pData, iLen, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK);
#endif
#endif // BIT_BANG
}

int bbepI2CRead(unsigned char iAddr, unsigned char *pData, int iLen)
{
int i = 0;
#ifdef BIT_BANG
    i = I2CRead(iAddr, pData, iLen);
#else
#ifdef ARDUINO
    Wire.requestFrom(iAddr, (unsigned char)iLen);
    while (i < iLen && Wire.available()) {
        pData[i++] = Wire.read();
    }
#else
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
       // ESP_LOGE("epdiy", "insufficient memory for I2C transaction");
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (iAddr << 1) | I2C_MASTER_READ, true);
    if (iLen > 1) {
        i2c_master_read(cmd, pData, iLen - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, pData + iLen - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        i = iLen;
    }
    i2c_cmd_link_delete(cmd);
#endif
#endif // BIT_BANG
    return i;
}
int bbepI2CReadRegister(unsigned char iAddr, unsigned char u8Register, unsigned char *pData, int iLen)
{
    bbepI2CWrite(iAddr, &u8Register, 1);
    bbepI2CRead(iAddr, pData, iLen);
    return iLen;
}
#endif // __BB_EP_IO__
