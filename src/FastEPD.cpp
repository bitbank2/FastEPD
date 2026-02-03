//
// FastEPD
// Copyright (c) 2024 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
//
// This file contains the C++ wrapper functions
// which call the C code doing the actual work.
// This allows for both C++ and C code to make
// use of all of the library features
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
#ifdef ARDUINO
#include <Wire.h>
#endif
#include "FastEPD.h"
#include "arduino_io.inl"
#include "FastEPD.inl"
#include "bb_ep_gfx.inl"

//#pragma GCC optimize("O2")
// Display how much time each operation takes on the serial monitor
#define SHOW_TIME

int FASTEPD::getStringBox(const char *text, BB_RECT *pRect)
{
    return bbepGetStringBox(&_state, text, pRect);
}
#ifdef ARDUINO
void FASTEPD::getStringBox(const String &str, BB_RECT *pRect)
{
    bbepGetStringBox(&_state, str.c_str(), pRect);
}
#endif
void FASTEPD::setCursor(int x, int y)
{
    if (x >= 0) {
        _state.iCursorX = x;
    }
    if (y >= 0) {
        _state.iCursorY = y;
    }
}
//
// Copy the current pixels to the previous for partial updates after powerup
//
void FASTEPD::backupPlane(void)
{
    bbepBackupPlane(&_state);
}

int FASTEPD::setCustomMatrix(const uint8_t *pMatrix, size_t matrix_size)
{
    return bbepSetCustomMatrix(&_state, pMatrix, matrix_size);
} /* setCustomMatrix() */

int FASTEPD::loadBMP(const uint8_t *pBMP, int x, int y, int iFG, int iBG)
{
    return bbepLoadBMP(&_state, pBMP, x, y, iFG, iBG);
} /* loadBMP() */

int FASTEPD::loadG5Image(const uint8_t *pG5, int x, int y, int iFG, int iBG, float fScale)
{
    return bbepLoadG5(&_state, pG5, x, y, iFG, iBG, fScale);
}

void FASTEPD::setPasses(uint8_t iPartialPasses, uint8_t iFullPasses)
{
    if (iPartialPasses > 0 && iPartialPasses < 15) { // reasonable numbers
        _state.iPartialPasses = iPartialPasses;
    }
    if (iFullPasses > 0 && iFullPasses < 15) { // reasonable numbers
        _state.iFullPasses = iFullPasses;
    }
} /* setPasses() */

int FASTEPD::setRotation(int iAngle)
{
    return bbepSetRotation(&_state, iAngle);
}
void FASTEPD::drawPixel(int x, int y, uint8_t color)
{
    (*_state.pfnSetPixel)(&_state, x, y, color);
}
void FASTEPD::drawPixelFast(int x, int y, uint8_t color)
{
    (*_state.pfnSetPixelFast)(&_state, x, y, color);
}

void FASTEPD::drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color)
{
    bbepEllipse(&_state, x, y, r, r, 0xf, color, 0);
}
void FASTEPD::fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color)
{
    bbepEllipse(&_state, x, y, r, r, 0xf, color, 1);
}
void FASTEPD::drawRoundRect(int x, int y, int w, int h,
                   int r, uint8_t color)
{
    bbepRoundRect(&_state, x, y, w, h, r, color, 0);
}
void FASTEPD::fillRoundRect(int x, int y, int w, int h,
                   int r, uint8_t color)
{
    bbepRoundRect(&_state, x, y, w, h, r, color, 1);
}

void FASTEPD::freeSprite(void)
{
    if (_state.pCurrent) {
        free(_state.pCurrent);
    }
    memset(&_state, 0, sizeof(FASTEPD));
} /* freeSprite() */

int FASTEPD::initSprite(int iWidth, int iHeight)
{
int rc;
    rc = bbepInitPanel(&_state, BB_PANEL_VIRTUAL, 0);
    if (rc == BBEP_SUCCESS) {
        rc = bbepSetPanelSize(&_state, iWidth, iHeight, 0, 0);
    }
    return rc;
} /* initSprite() */

int FASTEPD::drawSprite(FASTEPD *pSprite, int x, int y, int iTransparent)
{
    return bbepDrawSprite(&pSprite->_state, &_state, x, y, iTransparent);
} /* drawSprite() */

void FASTEPD::drawRect(int x, int y, int w, int h, uint8_t color)
{
    bbepRectangle(&_state, x, y, x+w-1, y+h-1, color, 0);
}

void FASTEPD::invertRect(int x, int y, int w, int h)
{
    bbepInvertRect(&_state, x, y, w, h);
}

void FASTEPD::fillRect(int x, int y, int w, int h, uint8_t color)
{
    bbepRectangle(&_state, x, y, x+w-1, y+h-1, color, 1);
}

void FASTEPD::drawLine(int x1, int y1, int x2, int y2, int iColor)
{
    bbepDrawLine(&_state, x1, y1, x2, y2, iColor);
} /* drawLine() */ 

int FASTEPD::setMode(int iMode)
{
    return bbepSetMode(&_state, iMode);
  /* setMode() */
}
void FASTEPD::ioPinMode(uint8_t u8Pin, uint8_t iMode)
{
    if (_state.pfnExtIO) {
        (*_state.pfnExtIO)(BB_EXTIO_SET_MODE, u8Pin, iMode);
    }
}
void FASTEPD::ioWrite(uint8_t u8Pin, uint8_t iValue)
{
    if (_state.pfnExtIO) {
        (*_state.pfnExtIO)(BB_EXTIO_WRITE, u8Pin, iValue);
    }
}
uint8_t FASTEPD::ioRead(uint8_t u8Pin)
{
    uint8_t val = 0;
    if (_state.pfnExtIO) {
        val = (*_state.pfnExtIO)(BB_EXTIO_READ, u8Pin, 0);
    }
    return val;
}

void FASTEPD::setBitBang(bool bBitBang)
{
    _state.bit_bang = (uint8_t)bBitBang;
} /* setBitBang() */

void FASTEPD::setItalic(bool bItalic)
{
   _state.italic = (uint8_t)bItalic;
} /* setItalic() */

void FASTEPD::setFont(int iFont)
{
    _state.iFont = iFont;
    _state.pFont = NULL;
    _state.anti_alias = 0;
} /* setFont() */

void FASTEPD::setFont(const void *pFont, bool bAntiAlias)
{
    _state.iFont = -1;
    _state.pFont = (void *)pFont;
    if (_state.mode != BB_MODE_4BPP) bAntiAlias = false; // only works in grayscale mode
    _state.anti_alias = (uint8_t)bAntiAlias;
} /* setFont() */

void FASTEPD::setTextWrap(bool bWrap)
{
    bbepSetTextWrap(&_state, (int)bWrap); 
} /* setTextWrap() */

void FASTEPD::setTextColor(int iFG, int iBG)
{
    _state.iFG = iFG;
    _state.iBG = (iBG == -1) ? iFG : iBG;
} /* setTextColor() */

void FASTEPD::drawString(const char *pText, int x, int y)
{
    if (_state.pFont) {
        bbepWriteStringCustom(&_state, (BB_FONT *)_state.pFont, x, y, (char *)pText, _state.iFG);
    } else if (_state.iFont >= FONT_6x8 && _state.iFont < FONT_COUNT) {
        bbepWriteString(&_state, x, y, (char *)pText, _state.iFont, _state.iFG);
    }
} /* drawString() */

size_t FASTEPD::write(uint8_t c) {
char szTemp[2]; // used to draw 1 character at a time to the C methods
int w=8, h=8;
static int iUnicodeCount = 0;
static uint8_t u8Unicode0, u8Unicode1;

   if (iUnicodeCount == 0) {
       if (c >= 0x80) { // start of a multi-byte character
           iUnicodeCount++;
           u8Unicode0 = c;
           return 1;
       }
   } else { // middle/end of a multi-byte character
       uint16_t u16Code;
       if (u8Unicode0 < 0xe0) { // 2 byte char, 0-0x7ff
           u16Code = (u8Unicode0 & 0x3f) << 6;
           u16Code += (c & 0x3f);
           c = bbepUnicodeTo1252(u16Code);
           iUnicodeCount = 0;
       } else { // 3 byte character 0x800 and above
           if (iUnicodeCount == 1) {
               iUnicodeCount++; // save for next byte to arrive
               u8Unicode1 = c;
               return 1;
           }
           u16Code = (u8Unicode0 & 0x3f) << 12;
           u16Code += (u8Unicode1 & 0x3f) << 6;
           u16Code += (c & 0x3f);
           c = bbepUnicodeTo1252(u16Code);
           iUnicodeCount = 0;
       }
   }
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
      BB_FONT *pBBF;
      BB_FONT_SMALL *pBBFS;
      BB_GLYPH *pGlyph;
      BB_GLYPH_SMALL *pGlyphSmall;
      int first, last;

      if (*(uint16_t *)_state.pFont == BB_FONT_MARKER) {
        pBBF = (BB_FONT *)_state.pFont; pBBFS = NULL;
        first = pBBF->first;
        last = pBBF->last;
        pGlyph = &pBBF->glyphs[c - first]; pGlyphSmall = NULL;
      } else {
        pBBFS = (BB_FONT_SMALL *)_state.pFont; pBBF = NULL;
        first = pBBFS->first;
        last = pBBFS->last;
        pGlyphSmall = &pBBFS->glyphs[c - first]; pGlyph = NULL;
      }
    if (c == '\n') {
      _state.iCursorX = 0;
      _state.iCursorY += (pBBF) ? pBBF->height : pBBFS->height;
    } else if (c != '\r') {
      if (c >= first && c <= last) {
        if (pBBF) {
            w = pGlyph->width;
            h = pGlyph->height;
        } else {
            w = pGlyphSmall->width;
            h = pGlyphSmall->height;
        }
        if (w > 0 && h > 0) { // Is there an associated bitmap?
          w += (pBBF) ? pGlyph->xOffset : pGlyphSmall->xOffset;
          if (_state.wrap && (_state.iCursorX + w) > _state.width) {
            _state.iCursorX = 0;
            _state.iCursorY += h;
          }
          bbepWriteStringCustom(&_state, _state.pFont, -1, -1, szTemp, _state.iFG);
        }
      }
    }
  }
  return 1;
} /* write() */

void FASTEPD::setBrightness(uint8_t led1, uint8_t led2)
{
    bbepSetBrightness(&_state, led1, led2);
}
void FASTEPD::initLights(uint8_t led1, uint8_t led2)
{
    bbepInitLights(&_state, led1, led2);
} /* initLights() */

int FASTEPD::initCustomPanel(BBPANELDEF *pPanel, BBPANELPROCS *pProcs)
{
    _state.iPanelType = BB_PANEL_CUSTOM;
    memcpy(&_state.panelDef, pPanel, sizeof(BBPANELDEF));
    _state.pfnEinkPower = pProcs->pfnEinkPower;
    _state.pfnIOInit = pProcs->pfnIOInit;
    _state.pfnRowControl = pProcs->pfnRowControl;
    return (*(_state.pfnIOInit))(&_state);
} /* initCustomPanel() */

int FASTEPD::setPanelSize(int iPanel)
{
    return bbepSetDefinedPanel(&_state, iPanel);
}

int FASTEPD::setPanelSize(int width, int height, int flags, int iVCOM) {
    return bbepSetPanelSize(&_state, width, height, flags, iVCOM);
} /* setPanelSize() */

int FASTEPD::initPanel(int iPanel, uint32_t u32Speed)
{
    return bbepInitPanel(&_state, iPanel, u32Speed);
} /* initIO() */

int FASTEPD::einkPower(int bOn)
{
    return bbepEinkPower(&_state, bOn);
} /* einkPower() */

int FASTEPD::clearWhite(bool bKeepOn)
{
    if (bbepEinkPower(&_state, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    fillScreen((_state.mode == BB_MODE_1BPP) ? BBEP_WHITE : 0xf);
    backupPlane(); // previous buffer set to the same color
    // 7 passes is enough to set all of the displays I've used to pure white or black
    bbepClear(&_state, BB_CLEAR_DARKEN, 7, NULL);
    bbepClear(&_state, BB_CLEAR_LIGHTEN, 7, NULL);
    bbepClear(&_state, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) bbepEinkPower(&_state, 0);
    return BBEP_SUCCESS;
} /* clearWhite() */

int FASTEPD::clearBlack(bool bKeepOn)
{
    if (bbepEinkPower(&_state, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    fillScreen(BBEP_BLACK);
    backupPlane(); // previous buffer set to the same color
    // 7 passes is enough to set all of the displays I've used to pure white or black
    bbepClear(&_state, BB_CLEAR_LIGHTEN, 7, NULL);
    bbepClear(&_state, BB_CLEAR_DARKEN, 7, NULL);
    bbepClear(&_state, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) bbepEinkPower(&_state, 0);
    return BBEP_SUCCESS;
} /* clearBlack() */

void FASTEPD::fillScreen(uint8_t u8Color)
{
    bbepFillScreen(&_state, u8Color);
} /* fillScreen() */

int FASTEPD::fullUpdate(int iClearMode, bool bKeepOn, BB_RECT *pRect)
{
    return bbepFullUpdate(&_state, iClearMode, bKeepOn, pRect);
} /* fullUpdate() */

int FASTEPD::partialUpdate(bool bKeepOn, int iStartLine, int iEndLine)
{
    return bbepPartialUpdate(&_state, bKeepOn, iStartLine, iEndLine);
} /* partialUpdate() */
int FASTEPD::smoothUpdate(bool bKeepOn, uint8_t u8Color)
{
    return bbepSmoothUpdate(&_state, bKeepOn, u8Color);
} /* smoothUpdate() */
