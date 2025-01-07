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

// GPIO modes
#ifndef ARDUINO
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
unsigned long IRAM_ATTR micros(void)
{
    return (unsigned long)(esp_timer_get_time());
}
unsigned long millis(void)
{
    return micros() / 1000;
}
void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if (us)
    {
        uint32_t e = (m + us);
        if (m > e)
        { //overflow
            while (micros() > e)
            {
                __asm__ __volatile__("nop\n");
            }
        }
        while (micros() < e)
        {
            __asm__ __volatile__("nop\n");
        }
    }
} /* delayMicroseconds() */

void delay(uint32_t ms)
{
    if (ms >= 10) {
        vTaskDelay(ms/10);
    }
    delayMicroseconds((ms % 10) * 1000);
}
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
    return BBEP_SUCCESS;
} /* bbepI2CInit() */

int bbepI2CWrite(unsigned char iAddr, unsigned char *pData, int iLen)
{
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
}

int bbepI2CRead(unsigned char iAddr, unsigned char *pData, int iLen)
{
int i = 0;

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
#endif
    return i;
}
int bbepI2CReadRegister(unsigned char iAddr, unsigned char u8Register, unsigned char *pData, int iLen)
{
    bbepI2CWrite(iAddr, &u8Register, 1);
    bbepI2CRead(iAddr, pData, iLen);
    return iLen;
}
#endif // __BB_EP_IO__
