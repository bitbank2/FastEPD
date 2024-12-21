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

// Flags indicating the connection type and behavior of the EPD signals
#define BB_IO_FLAG_GPIO      0x0000
#define BB_IO_FLAG_INVERTED  0x8000
#define BB_IO_FLAG_MCP23017  0x4000
#define BB_IO_FLAG_TPS65186  0x2000

#define BBEP_TRANSPARENT 255

// device names
enum {
    BB_PANEL_NONE=0,
    BB_PANEL_M5PAPERS3,
    BB_PANEL_T5EPAPERS3_PRO,
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
    uint16_t ioGMODE; // MODE
    uint16_t ioSDA;
    uint16_t ioSCL;
} BBPANELDEF;
// Graphics modes
enum {
    BB_MODE_NONE = 0,
    BB_MODE_1BPP, // 1 bit per pixel
    BB_MODE_2BPP, // 2 bits per pixel
    BB_MODE_4BPP, // 4 bits per pixel
};

// Row step options
enum {
    ROW_START = 0,
    ROW_STEP,
    ROW_END
};

enum {
    BBEP_SUCCESS = 0,
    BBEP_IO_ERROR,
    BBEP_MEM_ERROR,
    BBEP_INVALID_PARAM,
    BBEP_ERROR_COUNT
};

// Normal pixel drawing function pointer
typedef int (BB_SET_PIXEL)(void *pBBEP, int x, int y, unsigned char color);
// Fast pixel drawing function pointer (no boundary checking)
typedef void (BB_SET_PIXEL_FAST)(void *pBBEP, int x, int y, unsigned char color);

typedef struct tag_bbepdiystate
{
    int iPanelType;
    uint8_t wrap, last_error, pwr_on, mode;
    int iCursorX, iCursorY;
    int width, height, native_width, native_height;
    int iScreenOffset, iOrientation;
    int iFG, iBG; //current color
    int iFont, iFlags;
    void *pFont;
    uint8_t *dma_buf;
    BBPANELDEF panelDef;
    BB_SET_PIXEL *pfnSetPixel;
    BB_SET_PIXEL_FAST *pfnSetPixelFast;
} BBEPDIYSTATE;

#ifdef __cplusplus
class BBEPDIY
{
  public:
    int initPanel(int iPanelType);
    int initCustomPanel(BBPANELDEF *pPanel);
    void shutdown(void);
    int setMode(int iMode); // set graphics mode
    uint8_t *previousBuffer(void);
    uint8_t *currentBuffer(void);
    int einkPower(int bOn);
    int fullUpdate(bool bFast, bool bKeepOn);
    int partialUpdate(bool bKeepOn, int iStartRow, int iEndRow);
    void setRotation(int iAngle);
    int getRotation(void);
    void backupPlane(void);
    void drawRoundRect(int x, int y, int w, int h,
                       int r, uint8_t color);
    void fillRoundRect(int x, int y, int w, int h,
                       int r, uint8_t color);
    void fillScreen(int iColor);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void setTextWrap(bool bWrap);
    void setTextColor(int iFG, int iBG = BBEP_TRANSPARENT);
    void setCursor(int x, int y);
    int loadBMP(const uint8_t *pBMP, int x, int y, int iFG, int iBG);
    int loadBMP3(const uint8_t *pBMP, int x, int y);
    int loadG5Image(const uint8_t *pG5, int x, int y, int iFG, int iBG);
    void setFont(int iFont);
    void setFont(const void *pFont);
    void drawLine(int x1, int y1, int x2, int y2, int iColor);
    void drawPixel(int16_t x, int16_t y, uint8_t color);
    int16_t height(void);
    int16_t width(void);
    void drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color);
    void fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color);
    void drawEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
    void fillEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
    void drawString(const char *pText, int x, int y);
    void drawSprite(const uint8_t *pSprite, int cx, int cy, int iPitch, int x, int y, uint8_t iColor);
  //  using Print::write;
  //  virtual size_t write(uint8_t);

  protected:
    void rowControl(int iMode);
    void writeRow(uint8_t *pData, int iLen);
    void clear(uint8_t val, uint8_t count);
    uint8_t *_pCurrent; // current pixels
    uint8_t *_pPrevious; // comparison with previous buffer
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
