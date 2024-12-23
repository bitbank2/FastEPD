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

#define BB_PANEL_FLAG_NONE     0x00
#define BB_PANEL_FLAG_MIRROR_X 0x01
#define BB_PANEL_FLAG_MIRROR_Y 0x02
#define BB_PANEL_FLAG_TPS65185 0x04
#define BB_PANEL_FLAG_TPS65186 0x08

// Flags indicating the connection type and behavior of the EPD signals
#define BB_IO_FLAG_GPIO      0x0000
#define BB_IO_FLAG_INVERTED  0x8000
#define BB_IO_FLAG_MCP23017  0x4000
#define BB_IO_FLAG_PCA9535   0x2000
#define BB_IO_FLAG_SHIFTREG  0x1000
#define BB_NOT_USED 0xffff
#define BBEP_TRANSPARENT 255

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
    BB_PANEL_T5EPAPERS3,
    BB_PANEL_EPDIY_V7,
    BB_PANEL_INKPLATE6PLUS,
    BB_PANEL_CUSTOM,
    BB_PANEL_COUNT
};
// A complete description of an EPD panel
typedef struct _paneldef {
    uint16_t width;
    uint16_t height;
    uint32_t bus_speed;
    uint32_t flags;
    uint8_t data[8];
    uint16_t ioPWR;
    uint16_t ioSPV;
    uint16_t ioCKV;
    uint16_t ioSPH; // XSTL
    uint16_t ioOE; // XOE
    uint16_t ioLE; // XLE
    uint16_t ioCL; // XCL
    uint16_t ioPWR_Good;
    uint16_t ioSDA;
    uint16_t ioSCL;
    uint16_t ioShiftSTR; // shift store register
    uint16_t ioShiftMask; // shift bits that can be left permanently in this state
} BBPANELDEF;
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

typedef struct tag_bbepdiystate
{
    int iPanelType;
    uint8_t wrap, last_error, pwr_on, mode, shift_data;
    int iCursorX, iCursorY;
    int width, height, native_width, native_height;
    int rotation;
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
} BBEPDIYSTATE;

#ifdef __cplusplus
#ifdef ARDUINO
class BBEPDIY : public Print
#else
class BBEPDIY
#endif
{
  public:
    BBEPDIY() {memset(&_state, 0, sizeof(_state)); _state.iFont = FONT_8x8;}
     int initPanel(int iPanelType);
    int initCustomPanel(BBPANELDEF *pPanel);
    int setPanelSize(int width, int height);
    void shutdown(void);
    int setMode(int iMode); // set graphics mode
    uint8_t *previousBuffer(void);
    uint8_t *currentBuffer(void);
    int einkPower(int bOn);
    int fullUpdate(bool bFast = false, bool bKeepOn = false);
    int partialUpdate(bool bKeepOn, int iStartRow = 0, int iEndRow = 2047);
    int setRotation(int iAngle);
    int getRotation(void) { return _state.rotation;}
    void backupPlane(void);
    void drawRoundRect(int x, int y, int w, int h,
                       int r, uint8_t color);
    void fillRoundRect(int x, int y, int w, int h,
                       int r, uint8_t color);
    void fillScreen(uint8_t iColor);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void fillRect(int x, int y, int w, int h, uint8_t color);
    void setTextWrap(bool bWrap);
    void setTextColor(int iFG, int iBG = BBEP_TRANSPARENT);
    void setCursor(int x, int y) {_state.iCursorX = x; _state.iCursorY = y;}
    int loadBMP(const uint8_t *pBMP, int x, int y, int iFG, int iBG);
    int loadBMP3(const uint8_t *pBMP, int x, int y);
    int loadG5Image(const uint8_t *pG5, int x, int y, int iFG, int iBG);
    void setFont(int iFont);
    void setFont(const void *pFont);
    void drawLine(int x1, int y1, int x2, int y2, int iColor);
    void drawPixel(int16_t x, int16_t y, uint8_t color);
    void drawPixelFast(int16_t x, int16_t y, uint8_t color);
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
#endif

  protected:
    void rowControl(int iMode);
    void writeRow(uint8_t *pData, int iLen);
    void clear(uint8_t val, uint8_t count);
    BBEPDIYSTATE _state;
}; // class BBEPDIY
#endif // __cplusplus

#ifdef __cplusplus
extern "C" {
#endif
// put C functions here
#ifdef __cplusplus
};
#endif // __cplusplus

#endif // __BB_EPDIY_H__
