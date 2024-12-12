#ifndef __BB_EPDIY_H__
#define __BB_EPDIY_H__

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif
// Display panels
enum {
    BB_INVALID = 0,
    BB_ED047TC1, // 960x540 4.7"
    BB_ED052TC4, // 1280x720 5.2"
    BB_ED060SCT, // 800x600 6"
    BB_ED060XC3, // 1024x768 6"
    BB_ED097OC4, // 1200x825 9.7"
    BB_ED097TC2, // 1200x825 9.7"
    BB_ED133UT2, // 1600x1200 13.3"
    BB_ED078KC1, // 1872x1404 7.8" 16-bit
    BB_PANEL_COUNT
};

enum {
    BBEP_SUCCESS = 0,
    BBEP_IO_ERROR,
    BBEP_MEM_ERROR,
    BBEP_INVALID_PARAM,
    BBEP_ERROR_COUNT
};

typedef struct tag_bb_display {
    int iWidth; // pixels
    int iHeight;
    uint8_t u8BusWidth; // bits
    int iBusSpeed; // MHz    
} BB_DISPLAY_T;

typedef struct tag_bb_io_config {
    uint8_t u8DataPins[16];
    uint8_t u8SDA; // I2C bus for power and I/O control
    uint8_t u8SCL;
    uint8_t u8CHK;
    uint8_t u8STH;
    uint8_t u8LEH;
    uint8_t u8STV;
    int iVCOM; // VCOM in 100th's of volts (e.g. 1600 = 16V)
} BB_IO_CONFIG_T;

typedef struct tag_bbepdiystate
{
    int iPanelType;
    BB_DISPLAY_T display;
    BB_IO_CONFIG_T io_config;
    // different member variables depending on the target system
#ifdef ARDUINO_ESP32S3_DEV
#else // old ESP32
#endif // old ESP32 vs S3
} BBEPDIYSTATE;

#ifdef __cplusplus
class BBEPDIY
{
  public:
    int setPanelType(int iPanel);
    int initIO(BB_IO_CONFIG_T *pIO);
    int powerOn(void);
    void powerOff(void);
  protected:
    BBEPDIYSTATE _state;

}; // class BBEPDIY
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
    int bbepIOInit(BBEPDIYSTATE *pState);
#ifdef __cplusplus
};
#endif // __cplusplus

#endif // __BB_EPDIY_H__
