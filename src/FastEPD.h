//
// FastEPD
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
#ifndef __FASTEPD_H__
#define __FASTEPD_H__

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

#define BB_PANEL_FLAG_NONE     0x00
#define BB_PANEL_FLAG_MIRROR_X 0x01
#define BB_PANEL_FLAG_MIRROR_Y 0x02
#define BB_PANEL_FLAG_SLOW_SPH 0x04

#define BB_NOT_USED 0xff
#define BBEP_TRANSPARENT 255

// 5 possible clearing options before an update
enum {
   CLEAR_NONE = 0, // don't clear
   CLEAR_FAST, // 8 passes black/white
   CLEAR_SLOW, // 10 passes black/white/black/white
   CLEAR_WHITE, // 5 passes to white (may not be sufficient to clear old pixels)
   CLEAR_BLACK, // 5 passes to black (not recommended unless you know what you're doing)
};
// 5 possible font sizes: 8x8, 16x32, 6x8, 12x16 (stretched from 6x8 with smoothing), 16x16 (stretched from 8x8) 
enum {
   FONT_6x8 = 0,
   FONT_8x8,
   FONT_12x16,
   FONT_16x16,
   FONT_COUNT
};
// Stretch+smoothing options
#define BBEP_SMOOTH_NONE  0
#define BBEP_SMOOTH_HEAVY 1
#define BBEP_SMOOTH_LIGHT 2
// Centering coordinates to pass to the character drawing functions
#define CENTER_X 9998
#define CENTER_Y 9999

// device names
enum {
    BB_PANEL_NONE=0,
    BB_PANEL_M5PAPERS3,
    BB_PANEL_EPDIY_V7,
    BB_PANEL_INKPLATE6PLUS,
    BB_PANEL_INKPLATE5V2,
    BB_PANEL_EPDIY_V7_16,
    BB_PANEL_V7_RAW,
    BB_PANEL_V7_103,
    BB_PANEL_CUSTOM,
    BB_PANEL_COUNT
};

// Pre-configured displays
enum {
    BBEP_DISPLAY_EC060TC1,
    BBEP_DISPLAY_EC060KD1,
    BBEP_DISPLAY_ED0970TC1,
    BBEP_DISPLAY_ED103TC2,
    BBEP_DISPLAY_ED052TC4,
    BBEP_DISPLAY_COUNT
};

// A complete description of an EPD panel
typedef struct _paneldef {
    uint16_t width;
    uint16_t height;
    uint32_t bus_speed;
    uint32_t flags;
    uint8_t data[16];
    uint8_t bus_width;
    uint8_t ioPWR;
    uint8_t ioSPV;
    uint8_t ioCKV;
    uint8_t ioSPH; // XSTL
    uint8_t ioOE; // XOE
    uint8_t ioLE; // XLE
    uint8_t ioCL; // XCL
    uint8_t ioPWR_Good;
    uint8_t ioSDA;
    uint8_t ioSCL;
    uint8_t ioShiftSTR; // shift store register
    uint8_t ioShiftMask; // shift bits that can be left permanently in this state
    uint8_t ioDCDummy; // unused GPIO for the LCD library to needlessly toggle
    const uint8_t *pGrayMatrix; // pointer to matrix of values (waveform) for 16 gray levels
    int iMatrixSize; // size of matrix in bytes
    int iLinePadding; // extra bytes needed for each transmission
} BBPANELDEF;

typedef struct bbepr {
    int x;
    int y;
    int w;
    int h;
} BBEPRECT;

// To access external IO through a single function pointer
// This enum defines the operation
enum {
    BB_EXTIO_SET_MODE=0,
    BB_EXTIO_WRITE,
    BB_EXTIO_READ
};

// Display clearing pass type
enum {
    BB_CLEAR_LIGHTEN = 0,
    BB_CLEAR_DARKEN,
    BB_CLEAR_NEUTRAL,
    BB_CLEAR_SKIP
};

// Graphics modes
enum {
    BB_MODE_NONE = 0,
    BB_MODE_1BPP, // 1 bit per pixel
    BB_MODE_4BPP, // 4 bits per pixel
};
#define BBEP_BLACK 0
#define BBEP_WHITE 1
// Row step options
enum {
    ROW_START = 0,
    ROW_STEP,
    ROW_END
};

// error messages
enum {
    BBEP_SUCCESS,
    BBEP_ERROR_BAD_PARAMETER,
    BBEP_ERROR_BAD_DATA,
    BBEP_ERROR_NOT_SUPPORTED,
    BBEP_ERROR_NO_MEMORY,
    BBEP_ERROR_OUT_OF_BOUNDS,
    BBEP_IO_ERROR,
    BBEP_ERROR_COUNT
};

// Normal pixel drawing function pointer
typedef int (BB_SET_PIXEL)(void *pBBEP, int x, int y, unsigned char color);
// Fast pixel drawing function pointer (no boundary checking)
typedef void (BB_SET_PIXEL_FAST)(void *pBBEP, int x, int y, unsigned char color);
// Callback function for turning on and off the eink DC/DC power
typedef int (BB_EINK_POWER)(void *pBBEP, int bOn);
// Callback function for initializing all of the I/O devices
typedef int (BB_IO_INIT)(void *pBBEP);
// Callback function to shut down the extra I/O (e.g. extenders)
typedef void (BB_IO_DEINIT)(void *pBBEP);
// Callback function for controlling the row start/step
typedef void (BB_ROW_CONTROL)(void *pBBEP, int iMode);
// Callback function to access external IO expanders
typedef uint8_t (BB_EXT_IO)(uint8_t iOP, uint8_t iPin, uint8_t iValue);

typedef struct tag_bbeppanelprocs
{
    BB_EINK_POWER *pfnEinkPower;
    BB_IO_INIT *pfnIOInit;
    BB_ROW_CONTROL *pfnRowControl;
    BB_IO_DEINIT *pfnIODeInit;
    BB_EXT_IO *pfnExtIO;
} BBPANELPROCS;

typedef struct tag_fastepdstate
{
    int iPanelType;
    uint8_t wrap, last_error, pwr_on, mode;
    uint8_t shift_data, anti_alias;
    uint8_t u8LED1, u8LED2;
    int iCursorX, iCursorY;
    int width, height, native_width, native_height;
    int rotation;
    int iPartialPasses, iFullPasses;
    int iScreenOffset, iOrientation;
    int iFG, iBG; //current color
    int iFont, iFlags;
    void *pFont;
    uint8_t *dma_buf;
    uint8_t *pCurrent; // current pixels
    uint8_t *pPrevious; // comparison with previous buffer
    uint8_t *pTemp; // temporary buffer for the patterns sent to the eink controller
    BBPANELDEF panelDef;
    BB_SET_PIXEL *pfnSetPixel;
    BB_SET_PIXEL_FAST *pfnSetPixelFast;
    BB_EINK_POWER *pfnEinkPower;
    BB_EXT_IO *pfnExtIO;
    BB_IO_INIT *pfnIOInit;
    BB_IO_DEINIT *pfnIODeInit;
    BB_ROW_CONTROL *pfnRowControl;
} FASTEPDSTATE;

#ifdef __cplusplus
#ifdef ARDUINO
class FASTEPD : public Print
#else
class FASTEPD
#endif
{
  public:
    FASTEPD() {memset(&_state, 0, sizeof(_state)); _state.iFont = FONT_8x8; _state.iFG = BBEP_BLACK;}
    int initPanel(int iPanelType, uint32_t u32Speed = 0);
    void initLights(uint8_t led1, uint8_t led2 = 0xff);
    void setBrightness(uint8_t led1, uint8_t led2 = 0);
    int initCustomPanel(BBPANELDEF *pPanel, BBPANELPROCS *pProcs);
    int setPanelSize(int iPanel);
    int setCustomMatrix(const uint8_t *pMatrix, size_t matrix_size);
    int setPanelSize(int width, int height, int flags = BB_PANEL_FLAG_NONE);
    int getStringBox(const char *text, BBEPRECT *pRect);
    int setMode(int iMode); // set graphics mode
    void ioPinMode(uint8_t u8Pin, uint8_t iMode);
    void ioWrite(uint8_t u8Pin, uint8_t iValue);
    uint8_t ioRead(uint8_t u8Pin);
    int getMode(void) {return _state.mode;}
    uint8_t *previousBuffer(void) { return _state.pPrevious;}
    uint8_t *currentBuffer(void) { return _state.pCurrent;}
    int einkPower(int bOn);
    void deInit(void) {if (_state.pfnIODeInit) (*_state.pfnIODeInit)(&_state);}
    int fullUpdate(int iClearMode = CLEAR_SLOW, bool bKeepOn = false, BBEPRECT *pRect = NULL);
    int partialUpdate(bool bKeepOn, int iStartRow = 0, int iEndRow = 4095);
    int smoothUpdate(bool bKeepOn, uint8_t u8Color);
    void setPasses(uint8_t iPartialPasses, uint8_t iFullPasses = 5);
    int setRotation(int iAngle);
    int getRotation(void) { return _state.rotation;}
    void backupPlane(void);
    void invertRect(int x, int y, int w, int h);
    void drawRoundRect(int x, int y, int w, int h, int r, uint8_t color);
    void fillRoundRect(int x, int y, int w, int h, int r, uint8_t color);
    void fillScreen(uint8_t iColor);
    int clearWhite(bool bKeepOn = false);
    int clearBlack(bool bKeepOn = false);
    void drawRect(int x, int y, int w, int h, uint8_t color);
    void fillRect(int x, int y, int w, int h, uint8_t color);
    void setTextWrap(bool bWrap);
    void setTextColor(int iFG, int iBG = BBEP_TRANSPARENT);
    void setCursor(int x, int y) {_state.iCursorX = x; _state.iCursorY = y;}
    int loadBMP(const uint8_t *pBMP, int x, int y, int iFG, int iBG);
    int loadG5Image(const uint8_t *pG5, int x, int y, int iFG, int iBG, float fScale = 1.0f);
    void setFont(int iFont);
    void setFont(const void *pFont, bool bAntiAliased = false);
    void drawLine(int x1, int y1, int x2, int y2, int iColor);
    void drawPixel(int x, int y, uint8_t color);
    void drawPixelFast(int x, int y, uint8_t color);
    int16_t height(void) { return _state.height;}
    int16_t width(void) {return _state.width;}
    void drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color);
    void fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color);
    void drawEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
    void fillEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
    void drawString(const char *pText, int x, int y);
    void drawSprite(const uint8_t *pSprite, int cx, int cy, int iPitch, int x, int y, uint8_t iColor);
#ifdef ARDUINO
    using Print::write;
    virtual size_t write(uint8_t);
#else
    size_t write(uint8_t);
#endif

  protected:
    FASTEPDSTATE _state;
}; // class FASTEPD
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
// put C functions here
#ifdef __cplusplus
};
#else
// Include all of the library code inline for C targets
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "arduino_io.inl"
#include "FastEPD.inl"
#include "bb_ep_gfx.inl"
#endif // __cplusplus

#endif // __FASTEPD_H__
