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
#ifdef __LINUX__
#include "linux_io.inl"
#include <arm_neon.h>
#include <pthread.h>
#else
#include "arduino_io.inl"
#endif // __LINUX__
#include "FastEPD.inl"
#include "bb_ep_gfx.inl"

//#pragma GCC optimize("O2")
// Display how much time each operation takes on the serial monitor
#define SHOW_TIME

#ifdef __LINUX__
// Expand each bit into a byte for SIMD processing
static const uint64_t u64_expand[] = {
0x0000000000000000, 0x0000000000000001, 0x0000000000000100, 0x0000000000000101,
0x0000000000010000, 0x0000000000010001, 0x0000000000010100, 0x0000000000010101,
0x0000000001000000, 0x0000000001000001, 0x0000000001000100, 0x0000000001000101,
0x0000000001010000, 0x0000000001010001, 0x0000000001010100, 0x0000000001010101,
0x0000000100000000, 0x0000000100000001, 0x0000000100000100, 0x0000000100000101,
0x0000000100010000, 0x0000000100010001, 0x0000000100010100, 0x0000000100010101,
0x0000000101000000, 0x0000000101000001, 0x0000000101000100, 0x0000000101000101,
0x0000000101010000, 0x0000000101010001, 0x0000000101010100, 0x0000000101010101,
0x0000010000000000, 0x0000010000000001, 0x0000010000000100, 0x0000010000000101,
0x0000010000010000, 0x0000010000010001, 0x0000010000010100, 0x0000010000010101,
0x0000010001000000, 0x0000010001000001, 0x0000010001000100, 0x0000010001000101,
0x0000010001010000, 0x0000010001010001, 0x0000010001010100, 0x0000010001010101,
0x0000010100000000, 0x0000010100000001, 0x0000010100000100, 0x0000010100000101,
0x0000010100010000, 0x0000010100010001, 0x0000010100010100, 0x0000010100010101,
0x0000010101000000, 0x0000010101000001, 0x0000010101000100, 0x0000010101000101,
0x0000010101010000, 0x0000010101010001, 0x0000010101010100, 0x0000010101010101,
0x0001000000000000, 0x0001000000000001, 0x0001000000000100, 0x0001000000000101,
0x0001000000010000, 0x0001000000010001, 0x0001000000010100, 0x0001000000010101,
0x0001000001000000, 0x0001000001000001, 0x0001000001000100, 0x0001000001000101,
0x0001000001010000, 0x0001000001010001, 0x0001000001010100, 0x0001000001010101,
0x0001000100000000, 0x0001000100000001, 0x0001000100000100, 0x0001000100000101,
0x0001000100010000, 0x0001000100010001, 0x0001000100010100, 0x0001000100010101,
0x0001000101000000, 0x0001000101000001, 0x0001000101000100, 0x0001000101000101,
0x0001000101010000, 0x0001000101010001, 0x0001000101010100, 0x0001000101010101,
0x0001010000000000, 0x0001010000000001, 0x0001010000000100, 0x0001010000000101,
0x0001010000010000, 0x0001010000010001, 0x0001010000010100, 0x0001010000010101,
0x0001010001000000, 0x0001010001000001, 0x0001010001000100, 0x0001010001000101,
0x0001010001010000, 0x0001010001010001, 0x0001010001010100, 0x0001010001010101,
0x0001010100000000, 0x0001010100000001, 0x0001010100000100, 0x0001010100000101,
0x0001010100010000, 0x0001010100010001, 0x0001010100010100, 0x0001010100010101,
0x0001010101000000, 0x0001010101000001, 0x0001010101000100, 0x0001010101000101,
0x0001010101010000, 0x0001010101010001, 0x0001010101010100, 0x0001010101010101,

0x0100000000000000, 0x0100000000000001, 0x0100000000000100, 0x0100000000000101,
0x0100000000010000, 0x0100000000010001, 0x0100000000010100, 0x0100000000010101,
0x0100000001000000, 0x0100000001000001, 0x0100000001000100, 0x0100000001000101,
0x0100000001010000, 0x0100000001010001, 0x0100000001010100, 0x0100000001010101,
0x0100000100000000, 0x0100000100000001, 0x0100000100000100, 0x0100000100000101,
0x0100000100010000, 0x0100000100010001, 0x0100000100010100, 0x0100000100010101,
0x0100000101000000, 0x0100000101000001, 0x0100000101000100, 0x0100000101000101,
0x0100000101010000, 0x0100000101010001, 0x0100000101010100, 0x0100000101010101,
0x0100010000000000, 0x0100010000000001, 0x0100010000000100, 0x0100010000000101,
0x0100010000010000, 0x0100010000010001, 0x0100010000010100, 0x0100010000010101,
0x0100010001000000, 0x0100010001000001, 0x0100010001000100, 0x0100010001000101,
0x0100010001010000, 0x0100010001010001, 0x0100010001010100, 0x0100010001010101,
0x0100010100000000, 0x0100010100000001, 0x0100010100000100, 0x0100010100000101,
0x0100010100010000, 0x0100010100010001, 0x0100010100010100, 0x0100010100010101,
0x0100010101000000, 0x0100010101000001, 0x0100010101000100, 0x0100010101000101,
0x0100010101010000, 0x0100010101010001, 0x0100010101010100, 0x0100010101010101,
0x0101000000000000, 0x0101000000000001, 0x0101000000000100, 0x0101000000000101,
0x0101000000010000, 0x0101000000010001, 0x0101000000010100, 0x0101000000010101,
0x0101000001000000, 0x0101000001000001, 0x0101000001000100, 0x0101000001000101,
0x0101000001010000, 0x0101000001010001, 0x0101000001010100, 0x0101000001010101,
0x0101000100000000, 0x0101000100000001, 0x0101000100000100, 0x0101000100000101,
0x0101000100010000, 0x0101000100010001, 0x0101000100010100, 0x0101000100010101,
0x0101000101000000, 0x0101000101000001, 0x0101000101000100, 0x0101000101000101,
0x0101000101010000, 0x0101000101010001, 0x0101000101010100, 0x0101000101010101,
0x0101010000000000, 0x0101010000000001, 0x0101010000000100, 0x0101010000000101,
0x0101010000010000, 0x0101010000010001, 0x0101010000010100, 0x0101010000010101,
0x0101010001000000, 0x0101010001000001, 0x0101010001000100, 0x0101010001000101,
0x0101010001010000, 0x0101010001010001, 0x0101010001010100, 0x0101010001010101,
0x0101010100000000, 0x0101010100000001, 0x0101010100000100, 0x0101010100000101,
0x0101010100010000, 0x0101010100010001, 0x0101010100010100, 0x0101010100010101,
0x0101010101000000, 0x0101010101000001, 0x0101010101000100, 0x0101010101000101,
0x0101010101010000, 0x0101010101010001, 0x0101010101010100, 0x0101010101010101,
};
static int PrepVideoRow(int iWidth, uint8_t *pSrc, uint8_t *pDest, uint8_t *pWave, uint8_t *pCounts)
{
int x, bChanges = 0;
//
// Theory of operation:
// The counts array keeps track of the current state of eink pixels.
// A count value of 0 indicates white and 7 indicates black.
// This prevents pixels from being "overpushed" in a specific direction.
// Each current pixel is compared to black and white and if the count
// of that pixel is not already in that state, then a push in that
// direction is generated. If an entire line contains no changing pixels
// then that info is returned from this function as a 0 to indicate
// that the row step logic should not wait at the end of the row for
// the electric fields to move any pixels. This can speed up overall
// the update process otherwise each pass would be fixed at about 12ms
//

#ifdef OLD_WAY
uint8_t uc, ucOld, ucOut, ucMask;

    uc = *pSrc++; // read first byte to start
    ucOld = *pDest;
    ucMask = 0x80;
    ucOut = 0;
    bChanges = 0;
    for (x=0; x<iWidth; x+=4) {
        for (int j=0; j<4; j++) { // 4 pixels per output byte
            ucOut <<= 2; // next pair of control bits
            if ((uc & ucMask) != (ucOld & ucMask)) { // color change
                // Reset the counts to do a full set of pushes
                pCounts[0] = (uc & ucMask) ? 5 : 0;
            }
            if (uc & ucMask) { // current pixel is white
                if (pCounts[0] > 0) { // changing to white
                    ucOut |= 2; // push white
                    pCounts[0]--; // decrement the count
                }
            } else { // current pixel is black
                if (pCounts[0] < 5) { // changing to black
                    ucOut |= 1; // push black
                    pCounts[0]++; // increment the count
                }
            }
            pCounts++;
            ucMask >>= 1;
       } // for j
       //bChanges |= ucOut; // non-zero row will be detected
       bChanges |= ucOut;
       *pWave++ = ucOut;
       if (ucMask == 0) { // next source byte
           *pDest++ = uc; // new becomes old
           uc = *pSrc++;
           ucOld = *pDest;
           ucMask = 0x80;
       }
    } // for x
#else // New way
uint8x16_t u64Ones = vdupq_n_u8(0x01);
//uint8x16_t u64Eights = vdupq_n_u8(0x08);
uint8x16_t u64Fives = vdupq_n_u8(0x05);
static const uint8_t u8PushWhite[] = {0x02,0x08,0x20,0x80,0x02,0x08,0x20,0x80,0x02,0x08,0x20,0x80,0x02,0x08,0x20,0x80};
static const uint8_t u8PushBlack[] = {0x01,0x04,0x10,0x40,0x01,0x04,0x10,0x40,0x01,0x04,0x10,0x40,0x01,0x04,0x10,0x40};
uint8x16_t u64White = vld1q_u8(u8PushWhite);
uint8x16_t u64Black = vld1q_u8(u8PushBlack);
uint8x16_t vchanges = vdupq_n_u8(0);
uint8x8_t vout64;

    for (x=0; x<iWidth; x+=16) { // work 16 pixels at a time
        uint8x16_t temp64, wideNew, wideOld, count64, cmp64, push64;
        push64 = vdupq_n_u8(0); // assume no pushing
        wideNew = vcombine_u8(vld1_u8((uint8_t *)&u64_expand[pSrc[0]]), vld1_u8((uint8_t *)&u64_expand[pSrc[1]])); // expanded source byte
        wideOld = vcombine_u8(vld1_u8((uint8_t *)&u64_expand[pDest[0]]), vld1_u8((uint8_t *)&u64_expand[pDest[1]])); // expanded comparison byte
        count64 = vld1q_u8(pCounts);
        cmp64 = vceqq_u8(wideNew, wideOld); // any color changes?
        count64 = vandq_u8(cmp64, count64); // keep counts for unchanging pixels
        temp64 = vmulq_u8(wideNew, u64Fives); //Eights); // white pixels -> 8
        cmp64 = vmvnq_u8(cmp64); // pixels which are different = 0xff
        cmp64 = vandq_u8(cmp64, temp64); // changing white pixels = 8
        count64 = vorrq_u8(count64, cmp64); // counts reset to 0 for black, 8 for white
        // test counts and adjust pixel pushing
        // if color == 1 (white) and count > 0, push
        cmp64 = vceqq_u8(wideNew, u64Ones);
        temp64 = vcgeq_u8(count64, u64Ones);
        cmp64 = vandq_u8(cmp64, temp64); // satisfy both conditions
        push64 = vandq_u8(cmp64, u64White); // a 2 in each slot to push white
        count64 = vsubq_u8(count64, vandq_u8(cmp64, u64Ones)); // decrement counts
        // if color == 0 (black) and count < 5, push
        cmp64 = vceqq_u8(wideNew, vdupq_n_u8(0));
        temp64 = vcltq_u8(count64, u64Fives);
        cmp64 = vandq_u8(cmp64, temp64); // satisfy both conditions
        temp64 = vandq_u8(cmp64, u64Black); // a 1 in each slot to push black
        push64 = vorrq_u8(push64, temp64); // combine with white pushes
        count64 = vaddq_u8(count64, vandq_u8(cmp64, u64Ones)); // increment counts
        vst1q_u8(pCounts, count64); // new counts are finished
        pCounts += 16;
        *pDest++ = *pSrc++; // old = new
        *pDest++ = *pSrc++;
        // combine 16 push flags into 4 bytes
        temp64 = vdupq_n_u8(0);
        push64 = vpaddq_u8(push64, temp64); // merge the push bits together
        push64 = vpaddq_u8(push64, temp64); // now we have 4 bytes
        vchanges = vorrq_u8(vchanges, push64);
        vout64 = vget_low_u8(push64);
        vout64 = vrev16_u8(vout64); // swap bytes due to our 8->64 bit lookup table
        *(uint32_t *)pWave = vget_lane_u32(vreinterpret_u32_u8(vout64), 0); // write 4 control bytes
        pWave += 4;
    } // for x
    bChanges = vgetq_lane_u32(vreinterpretq_u32_u8(vchanges), 0);
#endif
    return bChanges;
} /* PrepVideoRow() */

static void *video_thread(void *pArg)
{
FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pArg;
uint8_t *pCounts; // memory holding push counts for each pixel
int iWidth, iHeight, iPitch;
uint8_t *c, *p, *d, *pc;
int /*bActivity,*/ iRowStep;

    iWidth = pBBEP->width;
    iHeight = pBBEP->height;
    pCounts = (uint8_t *)malloc(iWidth * iHeight);
    memset(pCounts, 0, iWidth * iHeight);
    iPitch = (iWidth+7)/8;

// Continuously monitor the current buffer and keep the display up to
// date with any pixel changes
    while (pBBEP->bVideo) {
        // Loop through each row and update any changes
        // if a row contains no changes, push a faster unchanging row
        c = pBBEP->pCurrent;
        p = pBBEP->pPrevious;
        d = pBBEP->pTemp;
        pc = pCounts;
        bbepRowControl(pBBEP, ROW_START);
        for (int iRow = 0; iRow < iHeight; iRow++) {
            /*bActivity =*/ PrepVideoRow(iWidth, c, p, d, pc);
            if (iRow == 0) iRowStep = ROW_NOP;
//            else if (bActivity == 0) iRowStep = ROW_STEP_FAST;
            else iRowStep = ROW_STEP;
            bbepWriteRow(pBBEP, d, iPitch*2, iRowStep);
            c += iPitch; p += iPitch; d += iPitch*2; pc += iWidth;
        } // for each row
        bbepRowControl(pBBEP, ROW_END);
    } // while video is running
    free(pCounts);
    pthread_exit(NULL);
} /* video_thread() */
//
// Perform a single pass of video style update (pixel counters)
//
void FASTEPD::videoUpdate(void)
{
int iWidth, iHeight, iPitch;
uint8_t *c, *p, *d, *pc;
int bActivity, iRowStep;
static int iFrame = 0;

    iFrame++;

    if (_state.pCounts == NULL) {
        _state.pCounts = (uint8_t *)malloc(_state.width * _state.height);
        memset(_state.pCounts, 0, _state.width * _state.height);
    }
    einkPower(1); // make sure power is turned on
    iWidth = _state.width;
    iHeight =_state.height;
    iPitch = (iWidth+7)/8;
//    if ((iFrame & 127) == 127) {
        // give a little push to static colors once in a while
//        memset(_state.pCounts, 3, _state.width * _state.height);
//    }
    // Loop through each row and update any changes
    // if a row contains no changes, push a faster unchanging row
    c = _state.pCurrent;
    p = _state.pPrevious;
    d = _state.pTemp;
    pc = _state.pCounts;
    bbepRowControl(&_state, ROW_START);
    for (int iRow = 0; iRow < iHeight; iRow++) {
        bActivity = PrepVideoRow(iWidth, c, p, d, pc);
        if (iRow == 0) iRowStep = ROW_NOP;
        else if (bActivity == 0) iRowStep = ROW_STEP_FAST;
        else iRowStep = ROW_STEP;
        bbepWriteRow(&_state, d, iPitch*2, iRowStep);
        c += iPitch; p += iPitch; d += iPitch*2; pc += iWidth;
    } // for each row
    bbepRowControl(&_state, ROW_END);
} /* videoUpdate() */

void FASTEPD::startVideo(void)
{
pthread_t host;

    clearWhite(1); // start from a known state

    _state.bVideo = 1; // video is running
    pthread_create(&host, NULL, video_thread, (void *)&_state);
} /* startVideo() */

void FASTEPD::stopVideo(void)
{
    _state.bVideo = 0; // second thread will exit
} /* stopVideo() */
#endif // __LINUX__

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

// Implement missing print functions
#ifndef ARDUINO
void FASTEPD::print(int value, int format)
{
char c, ucTemp[32];
char *d = &ucTemp[31];

   if (value) {
   d[0] = 0;
   switch(format) {
      case DEC:
         while (value) {
             d--;
             *d = '0' + (value % 10);
             value /= 10;
         }
         break;
      case HEX:
         while (value) {
            d--;
            c = value & 0xf;
            if (c < 10)
                    *d = '0' + c;
            else
                    *d = 'A' + (c-10);
            value >>= 4;
         }
         break;
      case OCT:
         while (value) {
            d--;
            *d = '0' + (value & 7);
            value >>= 3;
         }
         break;
      case BIN:
         while (value) {
            d--;
            *d = '0' + (value & 1);
            value >>= 1;
         }
         break;
      default:
         break;
      }
   } else { // if zero value
     d--;
     *d = '0';
   }
   print((const char *)d);
} /* print() */

void FASTEPD::println(int value, int format)
{
char ucTemp[4];

        print(value, format);
        ucTemp[0] = '\n';
        ucTemp[1] = '\r';
        ucTemp[2] = 0;
        print((const char *)ucTemp);
} /* println() */

void FASTEPD::print(const std::string &str)
{
   print(str.c_str());
} /* print() */

void FASTEPD::println(const char *pString)
{
char ucTemp[4];

    print(pString);
    ucTemp[0] = '\n';
    ucTemp[1] = '\r';
    ucTemp[2] = 0;
    print((const char *)ucTemp);
} /* println() */
void FASTEPD::println(const std::string &str)
{
char ucTemp[4];

   print(str);
   ucTemp[0] = '\n';
   ucTemp[1] = '\r';
   ucTemp[2] = 0;
   print((const char *)ucTemp);
} /* println() */

void FASTEPD::print(const char *pString)
{
uint8_t *s = (uint8_t *)pString;

   while (*s != 0) {
      write(*s++);
   }
} /* print() */

#endif // !ARDUINO

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
    if (_state.mode == BB_MODE_1BPP) {
        bAntiAlias = false; // only works in 2 or 4-bit grayscale mode
    }
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
      h = (pBBF) ? pBBF->height : pBBFS->height;
      _state.iCursorX = 0;
      if (_state.anti_alias) { 
          _state.iCursorY += h/2;
      } else {
          _state.iCursorY += h;
      }
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
            if (_state.anti_alias) {
                _state.iCursorY += h/2;
            } else {
                _state.iCursorY += h;
            }
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

int FASTEPD::initIT8951(uint8_t u8MOSI, uint8_t u8MISO, uint8_t u8CLK, uint8_t u8CS, uint8_t u8Busy, uint8_t u8RST, uint8_t u8EN, uint8_t u8ITE_EN)
{
    return bbepInitIT8951(&_state, u8MOSI, u8MISO, u8CLK, u8CS, u8Busy, u8RST, u8EN, u8ITE_EN);
} /* initIT8951() */

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

int FASTEPD::fastUpdate(bool bKeepOn)
{
    return bbepFastUpdate(&_state, bKeepOn);
} /* fastUpdate() */

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
