//
// FastEPD I/O wrapper functions for Linux
// Copyright (c) 2025 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 11/6/2025
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
//
// Adapt these functions to whatever target platform you're using
// and the rest of the code can remain unchanged
//
#ifndef __FASTEPD_IO__
#define __FASTEPD_IO__

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h> 
#include <string.h>
#include <fcntl.h>
#include <sys/sysinfo.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <math.h>
#include <time.h>
#ifndef CONSUMER
#define CONSUMER "Consumer"
#endif
#define pgm_read_byte(a) (*(uint8_t *)a)
#define pgm_read_word(a) (*(uint16_t *)a)
#define pgm_read_dword(a) (*(uint32_t *)a)
#define memcpy_P memcpy
#define gpio_num_t uint8_t
#define gpio_set_level bbepDigitalWrite
#define PROGMEM
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define HIGH 1
#define LOW 0

// for parallel GPIO
#define GPIO_BASE  (0xfe000000+0x00200000)
//#define GPIO_BASE  (0x3F000000+0x00200000)
#define GPIO_BLOCK_SIZE 4*1024
//#define GPFSEL0         (MMIO_BASE+0x00200000)
//#define GPSET0          (MMIO_BASE+0x0020001C)
//#define GPCLR0          (MMIO_BASE+0x00200028)
volatile uint32_t *gpio;
volatile uint32_t *set_reg, *clr_reg, *sel_reg;
//#define AUX_ENABLE      ((volatile uint32_t*)(MMIO_BASE+0x00215004))
int iMinDelay; // minimum inter-byte delay 
struct gpiod_chip *chip = NULL;
#ifdef GPIOD_API
struct gpiod_line *lines[64];
#else
struct gpiod_line_request *lines[64];
#endif
static int file_i2c = -1; // I2C handle
static uint32_t u32RowDelay;

// Initialize the I2C bus on Linux
int bbepI2CInit(uint8_t sda, uint8_t scl, uint8_t bBitBang)
{
char filename[32];
int iChannel = sda;
(void) scl;
(void) bBitBang;

// Only try to initialize the handle if it hasn't already been done
    if (file_i2c == -1) {
        sprintf(filename, "/dev/i2c-%d", iChannel);
        if ((file_i2c = open(filename, O_RDWR)) < 0)
        {
                fprintf(stderr, "Failed to open the i2c bus\n");
                return BBEP_IO_ERROR;
        }
    }
    return BBEP_SUCCESS;
} /* bbepI2CInit() */
//
// Read n bytes from the given I2C address
//
int bbepI2CRead(unsigned char iAddr, unsigned char *pData, int iLen)
{
int rc;
        ioctl(file_i2c, I2C_SLAVE, iAddr);
        rc = read(file_i2c, pData, iLen);
        return rc;
} /* bbepI2CRead() */
//
// Write n bytes to the given address
//
int bbepI2CWrite(unsigned char iAddr, unsigned char *pData, int iLen)
{
int rc;
        ioctl(file_i2c, I2C_SLAVE, iAddr);
        rc = write(file_i2c, pData, iLen);
        return rc;
} /* I2CWrite() */

int bbepI2CReadRegister(unsigned char iAddr, unsigned char u8Register, unsigned char *pData, int iLen)
{
    bbepI2CWrite(iAddr, &u8Register, 1);
    bbepI2CRead(iAddr, pData, iLen);
    return iLen;
}

int digitalRead(int iPin)
{
	if (lines[iPin] == 0) return 0;
#ifdef GPIOD_API // 1.x (old) API
  return gpiod_line_get_value(lines[iPin]);
#else // 2.x (new)
  return gpiod_line_request_get_value(lines[iPin], iPin) == GPIOD_LINE_VALUE_ACTIVE;
#endif
} /* digitalRead() */

static void bbepDigitalWrite(int iPin, int iState)
{
#ifdef SLOW_WAY
	if (lines[iPin] == 0) return;
#ifdef GPIOD_API // old 1.6 API
   gpiod_line_set_value(lines[iPin], iState);
#else // new 2.x API
   gpiod_line_request_set_value(lines[iPin], iPin, (iState) ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
#endif
#else // fast way
    if (iState) {
        *set_reg = (1 << iPin);
    } else {
        *clr_reg = (1 << iPin);
    }
#endif
} /* bbepDigitalWrite() */

void bbepPinMode(int iPin, int iMode)
{
//printf("bbepPinMode %d, %d\n", iPin, iMode);

#ifdef GPIOD_API // old 1.6 API
   if (chip == NULL) {
       chip = gpiod_chip_open_by_name("gpiochip0");
   }
   lines[iPin] = gpiod_chip_get_line(chip, iPin);
   if (iMode == OUTPUT) {
       gpiod_line_request_output(lines[iPin], CONSUMER, 0);
   } else if (iMode == INPUT_PULLUP) {
       gpiod_line_request_input_flags(lines[iPin], CONSUMER, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
   } else { // plain input
       gpiod_line_request_input(lines[iPin], CONSUMER);
   }
#else // new 2.x API
   struct gpiod_line_settings *settings;
   struct gpiod_line_config *line_cfg;
   struct gpiod_request_config *req_cfg;
   chip = gpiod_chip_open("/dev/gpiochip0");
   if (!chip) {
	printf("chip open failed\n");
	   return;
   }
   settings = gpiod_line_settings_new();
   if (!settings) {
	printf("line_settings_new failed\n");
	   return;
   }
   gpiod_line_settings_set_direction(settings, (iMode == OUTPUT) ? GPIOD_LINE_DIRECTION_OUTPUT : GPIOD_LINE_DIRECTION_INPUT);
   line_cfg = gpiod_line_config_new();
   if (!line_cfg) return;
   gpiod_line_config_add_line_settings(line_cfg, (const unsigned int *)&iPin, 1, settings);
   req_cfg = gpiod_request_config_new();
   gpiod_request_config_set_consumer(req_cfg, CONSUMER);
   lines[iPin] = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
   gpiod_request_config_free(req_cfg);
   gpiod_line_config_free(line_cfg);
   gpiod_line_settings_free(settings);
   gpiod_chip_close(chip);
#endif
} /* bbepPinMode() */

int millis(void)
{
int iTime;
struct timespec res;

    clock_gettime(CLOCK_MONOTONIC, &res);
    iTime = 1000*res.tv_sec + res.tv_nsec/1000000;

    return iTime;
} /* millis() */

void delay(int iMS)
{
  usleep(iMS * 1000);
} /* delay() */

void vTaskDelay(int iDelay)
{
    delay(iDelay * 10);
}

/**
 * Wait N CPU cycles (ARM CPU only)
 * On the RPI Zero 2W, 1 microsecond ˜= 463 wait loops
 */
static void wait_cycles(unsigned int n)
{
    if(n) while(n--) { asm volatile("nop"); }
}
static void delayMicroseconds(int iMS)
{
  //usleep(iMS);
  wait_cycles(iMS * (400 + iMinDelay*63));
} /* delayMicroseconds() */

void RPIRowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(pState->panelDef.ioShiftMask, 1); // EP_MODE on
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(7);
        gpio_set_level(spv, 0); // SPV off
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(2);
        gpio_set_level(ckv, 1);
        delayMicroseconds(8);
        gpio_set_level(spv, 1); // SPV on
        delayMicroseconds(10);
        gpio_set_level(ckv, 0);
        delayMicroseconds(1);
        for (int i=0; i<2; i++) {
            gpio_set_level(ckv, 1); // CKV on
            delayMicroseconds(18);
            gpio_set_level(ckv, 0); // CKV off
            delayMicroseconds(1);
        }
        gpio_set_level(ckv, 1);
        bbepDigitalWrite(pState->panelDef.ioOE, 1); // OE on
    } else if (iType == ROW_STEP || iType == ROW_STEP_FAST) {
        gpio_set_level(le, 1); // LE toggle
        wait_cycles(iMinDelay*2);
        gpio_set_level(le, 0);
        gpio_set_level(ckv, 1); // CKV off
        delayMicroseconds(1);
        gpio_set_level(ckv, 0); // CKV on
        wait_cycles((iType == ROW_STEP) ? u32RowDelay : u32RowDelay/2);
    } else if (iType == ROW_END) {
        bbepDigitalWrite(pState->panelDef.ioShiftMask, 0); // EP_MODE off
        bbepDigitalWrite(pState->panelDef.ioOE, 0); // OE off
        delayMicroseconds(230);
    }
}

void bbepWriteRow(FASTEPDSTATE *pState, uint8_t *pData, int iLen, int iRowStep)
{
    (*(pState->pfnRowControl))(pState, iRowStep);
    bbepDigitalWrite(pState->panelDef.ioCKV, 1); // CKV on
// Test with slow method first
    bbepDigitalWrite(pState->panelDef.ioSPH, 0); // CS low to start data
    wait_cycles(iMinDelay*2);
    iLen += pState->panelDef.iLinePadding;
#ifdef SLOW_WAY
    for (int i=0; i<iLen; i++) { // for each byte
        uint8_t u8Data = *pData++;
        bbepDigitalWrite(pState->panelDef.ioCL, 0); // CLK low
        for (int j=0; j<8; j++) {
            bbepDigitalWrite(pState->panelDef.data[j], u8Data & 1);
            u8Data >>= 1;
        } // for j
        bbepDigitalWrite(pState->panelDef.ioCL, 1); // CLK high to latch
    } // for i
#else // Fast Way :)
        const uint32_t DATA_BIT_0 = pState->panelDef.data[0];
        uint32_t c, e, d=0xffff;
        const uint32_t u32WR = 1 << pState->panelDef.ioCL;
        const uint32_t xor_mask = (0xff << DATA_BIT_0);
        const uint32_t xor_mask2 = (xor_mask | u32WR);

        for (int i=0; i<iLen; i++) {
            c = *pData++;
          // The GPIO writes add additional latency. This extra branch
          // speeds up the average data write by 20+%
            if (c != d) { // different data?
                e = c << DATA_BIT_0;
                *clr_reg = (e ^ xor_mask2); // set 0 bits
                *set_reg = e; // set 1 bits
                d = c;
            } else {
                *clr_reg = u32WR;
            }
            wait_cycles(iMinDelay);
            *set_reg = u32WR; // clock high
            wait_cycles(iMinDelay);
        } // for i
#endif
    bbepDigitalWrite(pState->panelDef.ioSPH, 1); // CS high to end data
} /* bbepWriteRow() */
void SetupGPIO(void)
{
   int mem_fd;
   void *gpio_map;
   uint32_t u32GPIO_BASE;

   // Determine if we're on a RPI 2/3 or 4 based on the RAM size
   struct sysinfo info;
   sysinfo(&info);
   if (info.totalram < 950000000) { // must be Zero2W or 3B
       //printf("RPI 2/3\n");
       u32GPIO_BASE = 0x3f000000+0x00200000;
       u32RowDelay = 9500;
       iMinDelay = 1;
   } else { // RPI 4B
       //printf("RPI 4B\n");
       u32RowDelay = 15000;
       u32GPIO_BASE = 0xfe000000+0x00200000;
       iMinDelay = 9;
   }
    if ((mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
        perror("Failed to open /dev/mem, try running as root");
        exit(EXIT_FAILURE);
    }

    // Map GPIO memory to our address space
    gpio_map = mmap(
        NULL,                 // Any address in our space will do
        21506*4,  // need to include the AUXILLARY function selector
//        GPIO_BLOCK_SIZE,      // Map length = 4K
        PROT_READ | PROT_WRITE, // Enable reading & writing to mapped memory
        MAP_SHARED,           // Shared with other processes
        mem_fd,               // File descriptor for /dev/mem
        u32GPIO_BASE          // Offset to GPIO peripheral
    );
    close(mem_fd);
    if (gpio_map == MAP_FAILED) {
        perror("mmap error");
        exit(EXIT_FAILURE);
    }

    // volatile pointer to prevent compiler optimizations
    gpio = (volatile uint32_t *)gpio_map;

    sel_reg = gpio;
    set_reg = &gpio[7];
    clr_reg = &gpio[10];
    gpio[21505] = 0; // disable UART/SPI1/SPI2

} /* SetupGPIO() */

//
// Board control
//
int RPIIOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    bbepPinMode(pState->panelDef.ioPWR_Good, INPUT);
    bbepPinMode(pState->panelDef.ioShiftSTR, OUTPUT); // TPS Wakeup
    bbepPinMode(pState->panelDef.ioShiftMask, OUTPUT); // EP_MODE
    bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT); 
    bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL, 0);
    // initialize the data lines
    for (int i=0; i<pState->panelDef.bus_width; i++) {
        bbepPinMode(pState->panelDef.data[i], OUTPUT);
    }
    pState->pCurrent = NULL; // make sure the memory gets allocated
// Prepare parallel GPIO
    SetupGPIO();

// Timing test of wait_cycles()
//    int l = millis();
//    wait_cycles(100000000);
//    l = millis() - l;
//    printf("100M wait_cycles = %d milliseconds\n", l);
    return BBEP_SUCCESS;
} /* RPIIOInit() */

int RPIEinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t u8Value, ucTemp[4];
int vcom;

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        bbepDigitalWrite(pState->panelDef.ioOE, 0); // OE off
        bbepDigitalWrite(pState->panelDef.ioShiftMask, 0); // EP_MODE off
        bbepDigitalWrite(pState->panelDef.ioPWR, 1); // TPS_PWRUP
        bbepDigitalWrite(pState->panelDef.ioShiftSTR, 1); // TPS_WAKEUP
        vTaskDelay(1); // allow time to power up
        while (!(digitalRead(pState->panelDef.ioPWR_Good))) { } // TPS_PWRGOOD
        ucTemp[0] = 0x01; //TPS_REG_ENABLE;
        ucTemp[1] = 0x3f; // enable output
        bbepI2CWrite(0x68, ucTemp, 2);
        // set VCOM (usually -1.6V = -1600mV = 160 value used in registers
        vcom = pState->iVCOM / -10; // convert to TPS format
        ucTemp[0] = 3; // vcom voltage register 3+4 = L + H
        ucTemp[1] = (uint8_t)vcom;
        ucTemp[2] = (uint8_t)(vcom >> 8);
        bbepI2CWrite(0x68, ucTemp, 3);

        int iTimeout = 0;
        u8Value = 0;
        while (iTimeout < 400 && ((u8Value & 0xfa) != 0xfa)) {
            bbepI2CReadRegister(0x68, 0x0f /* TPS_REG_PG */, &u8Value, 1); // read power good
            iTimeout++;
            vTaskDelay(1);
        }
        if (iTimeout >= 400) {
            printf("The power_good signal never arrived!\n");
            return BBEP_IO_ERROR;
        }
        pState->pwr_on = 1;
    } else { // power off
        bbepDigitalWrite(pState->panelDef.ioOE, 0); // OE off
        bbepDigitalWrite(pState->panelDef.ioShiftMask, 0); // EP_MODE off
        bbepDigitalWrite(pState->panelDef.ioPWR, 0); // TPS_PWRUP off
        vTaskDelay(1);
        bbepDigitalWrite(pState->panelDef.ioShiftSTR, 0); // TPS_WAKEUP off
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* RPIEinkPower() */

#endif // __FASTEPD_IO__
