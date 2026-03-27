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
#include "headless.h"

unsigned long micros(void) { return 10000000; }
unsigned long millis(void) { return micros() / 1000; }

void delayMicroseconds(uint32_t us) {}

void delay(uint32_t ms) {}

void* heap_caps_aligned_alloc(int alignment, int size, int caps) {
    printf("alocating aligned memory %d\n", size);
       return malloc(size);
}
void* heap_caps_malloc(int size, int caps) {
    printf("alocating memory %d\n", size);
    return malloc(size);
}


void gpio_set_level(gpio_num_t pin, int level) {}


int HeadlessEinkPower(void *pBBEP, int bOn) { return BBEP_SUCCESS; }
int HeadlessIOInit(void *pBBEP) { return BBEP_SUCCESS; }
void HeadlessRowControl(void *pBBEP, int iMode) {}

int bbepI2CWrite(unsigned char iAddr, unsigned char *pData, int iLen) {
    return 1;
}
int bbepI2CRead(unsigned char iAddr, unsigned char *pData, int iLen) {
    return 1;
}

void bbepPinMode(int pim, int mode) {}

int bbepI2CReadRegister(unsigned char iAddr, unsigned char u8Register, unsigned char *pData, int iLen)
{
    return 1;
}

void vTaskDelay(int ms) {}

int bbepI2CInit(uint8_t sda, uint8_t scl) { return 1; }

int main(int argc, char*argv[]) {
    setup();
    return 0;
}