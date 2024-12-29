//
// bb_epdiy
// Copyright (c) 2024 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
//
// This file contains the C++ wrapper functions
// which call the C code doing the actual work.
// This allows for both C++ and C code to make
// use of all of the library features
//
#include <Wire.h>
#include "bb_epdiy.h"
#include "arduino_io.inl"
#include "bb_epdiy.inl"
#include "bb_ep_gfx.inl"

#if !(defined(CONFIG_ESP32_SPIRAM_SUPPORT) || defined(CONFIG_ESP32S3_SPIRAM_SUPPORT))
#error "Please enable PSRAM support"
#endif
//#pragma GCC optimize("O2")
// Display how much time each operation takes on the serial monitor
#define SHOW_TIME

int BBEPDIY::getStringBox(const char *text, BBEPRECT *pRect)
{
    return bbepGetStringBox(&_state, text, pRect);
}
//
// Copy the current pixels to the previous for partial updates after powerup
//
void BBEPDIY::backupPlane(void)
{
    bbepBackupPlane(&_state);
}

int BBEPDIY::setRotation(int iAngle)
{
    return bbepSetRotation(&_state, iAngle);
}
void BBEPDIY::drawPixel(int16_t x, int16_t y, uint8_t color)
{
    (*_state.pfnSetPixel)(&_state, x, y, color);
}
void BBEPDIY::drawPixelFast(int16_t x, int16_t y, uint8_t color)
{
    (*_state.pfnSetPixelFast)(&_state, x, y, color);
}

void BBEPDIY::drawRoundRect(int x, int y, int w, int h,
                   int r, uint8_t color)
{
    bbepRoundRect(&_state, x, y, w, h, r, color, 0);
}
void BBEPDIY::fillRoundRect(int x, int y, int w, int h,
                   int r, uint8_t color)
{
    bbepRoundRect(&_state, x, y, w, h, r, color, 1);
}

void BBEPDIY::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    bbepRectangle(&_state, x, y, x+w-1, y+h-1, color, 0);
}

void BBEPDIY::fillRect(int x, int y, int w, int h, uint8_t color)
{
    bbepRectangle(&_state, x, y, x+w-1, y+h-1, color, 1);
}

void BBEPDIY::drawLine(int x1, int y1, int x2, int y2, int iColor)
{
    bbepDrawLine(&_state, x1, y1, x2, y2, iColor);
} /* drawLine() */ 

int BBEPDIY::setMode(int iMode)
{
    return bbepSetMode(&_state, iMode);
  /* setMode() */
}
void BBEPDIY::setFont(int iFont)
{
    _state.iFont = iFont;
    _state.pFont = NULL;
} /* setFont() */

void BBEPDIY::setFont(const void *pFont)
{
    _state.iFont = -1;
    _state.pFont = (void *)pFont;
} /* setFont() */

void BBEPDIY::setTextColor(int iFG, int iBG)
{
    _state.iFG = iFG;
    _state.iBG = (iBG == -1) ? iFG : iBG;
} /* setTextColor() */

void BBEPDIY::drawString(const char *pText, int x, int y)
{
    if (_state.pFont) {
        bbepWriteStringCustom(&_state, (BB_FONT *)_state.pFont, x, y, (char *)pText, _state.iFG);
    } else if (_state.iFont >= FONT_6x8 && _state.iFont < FONT_COUNT) {
        bbepWriteString(&_state, x, y, (char *)pText, _state.iFont, _state.iFG);
    }
} /* drawString() */

size_t BBEPDIY::write(uint8_t c) {
char szTemp[2]; // used to draw 1 character at a time to the C methods
int w=8, h=8;

  szTemp[0] = c; szTemp[1] = 0;
   if (_state.pFont == NULL) { // use built-in fonts
      if (_state.iFont == FONT_8x8 || _state.iFont == FONT_6x8) {
        h = 8;
        w = (_state.iFont == FONT_8x8) ? 8 : 6;
      } else if (_state.iFont == FONT_12x16 || _state.iFont == FONT_16x16) {
        h = 16;
        w = (_state.iFont == FONT_12x16) ? 12:16;
      }

    if (c == '\n') {              // Newline?
      _state.iCursorX = 0;          // Reset x to zero,
      _state.iCursorY += h; // advance y one line
    } else if (c != '\r') {       // Ignore carriage returns
        if (_state.wrap && ((_state.iCursorX + w) > _state.width)) { // Off right?
            _state.iCursorX = 0;               // Reset x to zero,
            _state.iCursorY += h; // advance y one line
        }
        bbepWriteString(&_state, -1, -1, szTemp, _state.iFont, _state.iFG);
    }
  } else { // Custom font
      BB_FONT *pBBF = (BB_FONT *)_state.pFont;
    if (c == '\n') {
      _state.iCursorX = 0;
      _state.iCursorY += pBBF->height;
    } else if (c != '\r') {
      if (c >= pBBF->first && c <= pBBF->last) {
          BB_GLYPH *pBBG = &pBBF->glyphs[c - pBBF->first];
        w = pBBG->width;
        h = pBBG->height;
        if (w > 0 && h > 0) { // Is there an associated bitmap?
          w += pBBG->xOffset;
          if (_state.wrap && (_state.iCursorX + w) > _state.width) {
            _state.iCursorX = 0;
            _state.iCursorY += h;
          }
          bbepWriteStringCustom(&_state, (BB_FONT *)_state.pFont, -1, -1, szTemp, _state.iFG);
        }
      }
    }
  }
  return 1;
} /* write() */

int BBEPDIY::initCustomPanel(BBPANELDEF *pPanel)
{
    _state.iPanelType = BB_PANEL_CUSTOM;
    memcpy(&_state.panelDef, pPanel, sizeof(BBPANELDEF));
    return bbepIOInit(&_state);
} /* setPanelType() */

int BBEPDIY::setPanelSize(int width, int height) {
    return bbepSetPanelSize(&_state, width, height);
} /* setPanelSize() */

int BBEPDIY::initPanel(int iPanel)
{
    return bbepInitPanel(&_state, iPanel);
} /* initIO() */

int BBEPDIY::einkPower(int bOn)
{
    return bbepEinkPower(&_state, bOn);
} /* einkPower() */

void BBEPDIY::fillScreen(uint8_t u8Color)
{
    bbepFillScreen(&_state, u8Color);
} /* fillScreen() */

int BBEPDIY::fullUpdate(bool bFast, bool bKeepOn, BBEPRECT *pRect)
{
    return bbepFullUpdate(&_state, bFast, bKeepOn, pRect);
} /* fullUpdate() */

int BBEPDIY::partialUpdate(bool bKeepOn, int iStartLine, int iEndLine)
{
    return bbepPartialUpdate(&_state, bKeepOn, iStartLine, iEndLine);
} /* partialUpdate() */
