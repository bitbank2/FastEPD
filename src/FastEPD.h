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
#ifdef __cplusplus
#include <string>
#endif // __cplusplus
// for Print support
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#endif

#define BB_PANEL_FLAG_NONE     0x00
#define BB_PANEL_FLAG_MIRROR_X 0x01
#define BB_PANEL_FLAG_MIRROR_Y 0x02
#define BB_PANEL_FLAG_SLOW_SPH 0x04
#define BB_PANEL_FLAG_DARK     0x08

#define BB_NOT_USED 0xff
#define BBEP_TRANSPARENT 255

// 5 possible clearing options before an update
enum {
   CLEAR_NONE = 0, // don't clear
   CLEAR_FAST, // 8 passes black/white
   CLEAR_SLOW, // 10 passes black/white/black/white
   CLEAR_WHITE, // 8 passes to white
   CLEAR_BLACK, // 8 passes to black
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
    BB_PANEL_INKPLATE10,
    BB_PANEL_EPDIY_V7_16,
    BB_PANEL_V7_RAW,
    BB_PANEL_LILYGO_T5PRO,
    BB_PANEL_LILYGO_T5P4,
    BB_PANEL_TRMNL_X,
    BB_PANEL_EPDINKY_P4,
    BB_PANEL_EPDINKY_P4_16,
    BB_PANEL_RPI,
    BB_PANEL_IT8951,
    BB_PANEL_SENSORIA_C5,
    BB_PANEL_CUSTOM,
    BB_PANEL_VIRTUAL,
    BB_PANEL_COUNT
};

// Pre-configured displays
enum {
    BBEP_DISPLAY_EC060TC1,
    BBEP_DISPLAY_EC060KD1,
    BBEP_DISPLAY_ED0970TC1,
    BBEP_DISPLAY_ED103TC2,
    BBEP_DISPLAY_ED052TC4,
    BBEP_DISPLAY_ED1150C1,
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
    int iVCOM; // VCOM in millivolts (e.g. -1600 = -1.6V)
} BBPANELDEF;

#ifndef __BB_RECT__
#define __BB_RECT__
typedef struct bbepr {
    int x;
    int y;
    int w;
    int h;
} BB_RECT;
#endif // __BB_RECT__

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
    BB_MODE_2BPP, // 2 bits per pixel
    BB_MODE_4BPP, // 4 bits per pixel
};
#define BBEP_BLACK 0
#define BBEP_WHITE 1
// Row step options
enum {
    ROW_START = 0,
    ROW_STEP,
    ROW_STEP_FAST,
    ROW_NOP,
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
    uint8_t u8CS, u8RST, u8Busy, u8EN, u8ITE_EN; // IT8951 support
    uint32_t spi_frequency, img_buf_addr, vcom_write_selector;
    uint8_t wrap, last_error, pwr_on, prev_mode, mode;
    uint8_t shift_data, anti_alias, italic, bit_bang;
    volatile int bVideo;
    uint8_t u8LED1, u8LED2;
    int iCursorX, iCursorY;
    int width, height, native_width, native_height;
    int rotation;
    int iPartialPasses, iFullPasses;
    int iScreenOffset, iOrientation;
    int iFG, iBG; //current color
    int iFont, iFlags, iVCOM;
    void *pFont;
    uint8_t *dma_buf;
    uint8_t *pCurrent; // current pixels
    uint8_t *pPrevious; // comparison with previous buffer
    uint8_t *pTemp; // temporary buffer for the patterns sent to the eink controller
#ifdef __LINUX__
    uint8_t *pCounts; // pointer to pixel counts buffer
#endif // __LINUX__
    BBPANELDEF panelDef;
    BB_SET_PIXEL *pfnSetPixel;
    BB_SET_PIXEL_FAST *pfnSetPixelFast;
    BB_EINK_POWER *pfnEinkPower;
    BB_EXT_IO *pfnExtIO;
    BB_IO_INIT *pfnIOInit;
    BB_IO_DEINIT *pfnIODeInit;
    BB_ROW_CONTROL *pfnRowControl;
} FASTEPDSTATE;

// IT8951 Support
// IT8951 command codes
#define IT8951_TCON_SYS_RUN      0x0001
#define IT8951_TCON_STANDBY      0x0002
#define IT8951_TCON_SLEEP        0x0003
#define IT8951_TCON_REG_RD       0x0010
#define IT8951_TCON_REG_WR       0x0011
#define IT8951_TCON_MEM_BST_RD_T 0x0012
#define IT8951_TCON_MEM_BST_RD_S 0x0013
#define IT8951_TCON_MEM_BST_WR   0x0014
#define IT8951_TCON_MEM_BST_END  0x0015
#define IT8951_TCON_LD_IMG       0x0020
#define IT8951_TCON_LD_IMG_AREA  0x0021
#define IT8951_TCON_LD_IMG_END   0x0022

// User-defined I80 command codes
#define USDEF_I80_CMD_DPY_AREA       0x0034
#define USDEF_I80_CMD_GET_DEV_INFO   0x0302
#define USDEF_I80_CMD_VCOM           0x0039
#define USDEF_I80_CMD_TEMP           0x0040
#define USDEF_I80_CMD_LD_IMG_1BPP    0x0095
// Rotation modes
#define IT8951_ROTATE_0   0
#define IT8951_ROTATE_90  1
#define IT8951_ROTATE_180 2
#define IT8951_ROTATE_270 3
// Waveform modes
#define IT8951_MODE_0  0
#define IT8951_MODE_1  1
#define IT8951_MODE_2  2
#define IT8951_MODE_3  3
#define IT8951_MODE_4  4
// Pixel format constants
#define IT8951_2BPP 0
#define IT8951_3BPP 1
#define IT8951_4BPP 2
#define IT8951_8BPP 3
#define IT8951_LDIMG_L_ENDIAN 0
#define IT8951_LDIMG_B_ENDIAN 1

// Register addresses
#define IT8951_REG_I80CPCR  0x0004
#define IT8951_REG_LISAR    0x0208
#define IT8951_REG_LUTAFSR  0x1224
#define IT8951_REG_UP1SR    0x1138
#define IT8951_REG_BGVR     0x1250

// SPI frequencies
#define IT8951_SPI_PROBE_FREQUENCY 1000000
#define IT8951_SPI_RUN_FREQUENCY   4000000

// IT8951 device info structure
typedef struct {
    uint16_t panel_width;
    uint16_t panel_height;
    uint16_t img_buf_addr_l;
    uint16_t img_buf_addr_h;
    uint16_t fw_version[8];
    uint16_t lut_version[8];
} IT8951DevInfo;

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
    int initIT8951(uint8_t u8MOSI, uint8_t u8MISO, uint8_t u8CLK, uint8_t u8CS, uint8_t u8Busy, uint8_t u8RST, uint8_t u8EN, uint8_t u8ITE_EN);
    void initLights(uint8_t led1, uint8_t led2 = 0xff);
    void setBrightness(uint8_t led1, uint8_t led2 = 0);
    int initCustomPanel(BBPANELDEF *pPanel, BBPANELPROCS *pProcs);
    int initSprite(int iWidth, int iHeight);
    int drawSprite(FASTEPD *pSprite, int x, int y, int iTransparent = -1);
    void freeSprite(void);
#ifdef __LINUX__
    void startVideo(void);
    void stopVideo(void);
    void videoUpdate(void);
#endif
    int setPanelSize(int iPanel);
    int setCustomMatrix(const uint8_t *pMatrix, size_t matrix_size);
    int setPanelSize(int width, int height, int flags = BB_PANEL_FLAG_NONE, int iVCOM = -1600);
    int getStringBox(const char *text, BB_RECT *pRect);
#ifdef ARDUINO
    void getStringBox(const String &str, BB_RECT *pRect);
#endif
    int setMode(int iMode); // set graphics mode
    int getPreviousMode(void) { return _state.prev_mode;}
    void setPreviousMode(uint8_t prev_mode) { _state.prev_mode = prev_mode;}
    void ioPinMode(uint8_t u8Pin, uint8_t iMode);
    void ioWrite(uint8_t u8Pin, uint8_t iValue);
    uint8_t ioRead(uint8_t u8Pin);
    int getMode(void) {return _state.mode;}
    uint8_t *previousBuffer(void) { return _state.pPrevious;}
    uint8_t *currentBuffer(void) { return _state.pCurrent;}
    uint8_t *tempBuffer(void) { return _state.pTemp;}
    int einkPower(int bOn);
    void deInit(void) {if (_state.pfnIODeInit) (*_state.pfnIODeInit)(&_state);}
    int fullUpdate(int iClearMode = CLEAR_SLOW, bool bKeepOn = false, BB_RECT *pRect = NULL);
    int partialUpdate(bool bKeepOn, int iStartRow = 0, int iEndRow = 4095);
    int smoothUpdate(bool bKeepOn, uint8_t u8Color);
    int fastUpdate(bool bKeepOn = false);
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
    void setCursor(int x, int y);
    int loadBMP(const uint8_t *pBMP, int x, int y, int iFG, int iBG);
    int loadG5Image(const uint8_t *pG5, int x, int y, int iFG, int iBG, float fScale = 1.0f);
    void setFont(int iFont);
    void setItalic(bool bItalic);
    void setBitBang(bool bBitBang);
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
    void print(const char *pString);
    void println(const char *pString);
    void print(int, int);
    void println(int, int);
    void print(const std::string &);
    void println(const std::string &);
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
