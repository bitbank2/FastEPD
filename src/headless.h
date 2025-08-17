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
#ifndef __BB_EP_HEADLESS__
#define __BB_EP_HEADLESS__

#include <FastEPD.h>
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


unsigned long micros(void);
unsigned long millis(void);

void delayMicroseconds(uint32_t us);

void delay(uint32_t ms);

#define MALLOC_CAP_SPIRAM 0
#define MALLOC_CAP_DMA 0
#define LCD_CLK_SRC_PLL160M 0
#define ESP_OK 1


void* heap_caps_aligned_alloc(int alignment, int size, int caps);
void* heap_caps_malloc(int size, int caps);

typedef int gpio_num_t;

void gpio_set_level(gpio_num_t pin, int level);

int bbepI2CWrite(unsigned char iAddr, unsigned char *pData, int iLen);
int bbepI2CRead(unsigned char iAddr, unsigned char *pData, int iLen);
void bbepPinMode(int pim, int mode);
int bbepI2CReadRegister(unsigned char iAddr, unsigned char u8Register, unsigned char *pData, int iLen);
void vTaskDelay(int ms);
int bbepI2CInit(uint8_t sda, uint8_t scl);
typedef int esp_err_t;
typedef void* esp_lcd_panel_io_handle_t;

int HeadlessEinkPower(void *pBBEP, int bOn);
int HeadlessIOInit(void *pBBEP);
void HeadlessRowControl(void *pBBEP, int iMode);

void setup();
#endif