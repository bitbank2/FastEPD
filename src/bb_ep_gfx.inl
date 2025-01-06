//
// bb_epdiy graphics library
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
//
// bb_ei_gfx.inl - graphics functions
//
// For constrained CPU environments, the NO_RAM macro will be defined
// to reduce codespace used by each of the functions
//
#ifndef __BB_EP_GFX__
#define __BB_EP_GFX__
#include "Group5.h"
#include "g5dec.inl"

static G5DECIMAGE g5dec;
// forward declarations
void InvertBytes(uint8_t *pData, uint8_t bLen);

const uint8_t ucFont[]PROGMEM = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x5f,0x5f,0x06,0x00,
    0x00,0x07,0x07,0x00,0x07,0x07,0x00,0x14,0x7f,0x7f,0x14,0x7f,0x7f,0x14,
    0x24,0x2e,0x2a,0x6b,0x6b,0x3a,0x12,0x46,0x66,0x30,0x18,0x0c,0x66,0x62,
    0x30,0x7a,0x4f,0x5d,0x37,0x7a,0x48,0x00,0x04,0x07,0x03,0x00,0x00,0x00,
    0x00,0x1c,0x3e,0x63,0x41,0x00,0x00,0x00,0x41,0x63,0x3e,0x1c,0x00,0x00,
    0x08,0x2a,0x3e,0x1c,0x3e,0x2a,0x08,0x00,0x08,0x08,0x3e,0x3e,0x08,0x08,
    0x00,0x00,0x80,0xe0,0x60,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,
    0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x60,0x30,0x18,0x0c,0x06,0x03,0x01,
    0x3e,0x7f,0x59,0x4d,0x47,0x7f,0x3e,0x40,0x42,0x7f,0x7f,0x40,0x40,0x00,
    0x62,0x73,0x59,0x49,0x6f,0x66,0x00,0x22,0x63,0x49,0x49,0x7f,0x36,0x00,
    0x18,0x1c,0x16,0x53,0x7f,0x7f,0x50,0x27,0x67,0x45,0x45,0x7d,0x39,0x00,
    0x3c,0x7e,0x4b,0x49,0x79,0x30,0x00,0x03,0x03,0x71,0x79,0x0f,0x07,0x00,
    0x36,0x7f,0x49,0x49,0x7f,0x36,0x00,0x06,0x4f,0x49,0x69,0x3f,0x1e,0x00,
    0x00,0x00,0x00,0x66,0x66,0x00,0x00,0x00,0x00,0x80,0xe6,0x66,0x00,0x00,
    0x08,0x1c,0x36,0x63,0x41,0x00,0x00,0x00,0x14,0x14,0x14,0x14,0x14,0x14,
    0x00,0x41,0x63,0x36,0x1c,0x08,0x00,0x00,0x02,0x03,0x59,0x5d,0x07,0x02,
    0x3e,0x7f,0x41,0x5d,0x5d,0x5f,0x0e,0x7c,0x7e,0x13,0x13,0x7e,0x7c,0x00,
    0x41,0x7f,0x7f,0x49,0x49,0x7f,0x36,0x1c,0x3e,0x63,0x41,0x41,0x63,0x22,
    0x41,0x7f,0x7f,0x41,0x63,0x3e,0x1c,0x41,0x7f,0x7f,0x49,0x5d,0x41,0x63,
    0x41,0x7f,0x7f,0x49,0x1d,0x01,0x03,0x1c,0x3e,0x63,0x41,0x51,0x33,0x72,
    0x7f,0x7f,0x08,0x08,0x7f,0x7f,0x00,0x00,0x41,0x7f,0x7f,0x41,0x00,0x00,
    0x30,0x70,0x40,0x41,0x7f,0x3f,0x01,0x41,0x7f,0x7f,0x08,0x1c,0x77,0x63,
    0x41,0x7f,0x7f,0x41,0x40,0x60,0x70,0x7f,0x7f,0x0e,0x1c,0x0e,0x7f,0x7f,
    0x7f,0x7f,0x06,0x0c,0x18,0x7f,0x7f,0x1c,0x3e,0x63,0x41,0x63,0x3e,0x1c,
    0x41,0x7f,0x7f,0x49,0x09,0x0f,0x06,0x1e,0x3f,0x21,0x31,0x61,0x7f,0x5e,
    0x41,0x7f,0x7f,0x09,0x19,0x7f,0x66,0x26,0x6f,0x4d,0x49,0x59,0x73,0x32,
    0x03,0x41,0x7f,0x7f,0x41,0x03,0x00,0x7f,0x7f,0x40,0x40,0x7f,0x7f,0x00,
    0x1f,0x3f,0x60,0x60,0x3f,0x1f,0x00,0x3f,0x7f,0x60,0x30,0x60,0x7f,0x3f,
    0x63,0x77,0x1c,0x08,0x1c,0x77,0x63,0x07,0x4f,0x78,0x78,0x4f,0x07,0x00,
    0x47,0x63,0x71,0x59,0x4d,0x67,0x73,0x00,0x7f,0x7f,0x41,0x41,0x00,0x00,
    0x01,0x03,0x06,0x0c,0x18,0x30,0x60,0x00,0x41,0x41,0x7f,0x7f,0x00,0x00,
    0x08,0x0c,0x06,0x03,0x06,0x0c,0x08,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x00,0x00,0x03,0x07,0x04,0x00,0x00,0x20,0x74,0x54,0x54,0x3c,0x78,0x40,
    0x41,0x7f,0x3f,0x48,0x48,0x78,0x30,0x38,0x7c,0x44,0x44,0x6c,0x28,0x00,
    0x30,0x78,0x48,0x49,0x3f,0x7f,0x40,0x38,0x7c,0x54,0x54,0x5c,0x18,0x00,
    0x48,0x7e,0x7f,0x49,0x03,0x06,0x00,0x98,0xbc,0xa4,0xa4,0xf8,0x7c,0x04,
    0x41,0x7f,0x7f,0x08,0x04,0x7c,0x78,0x00,0x44,0x7d,0x7d,0x40,0x00,0x00,
    0x60,0xe0,0x80,0x84,0xfd,0x7d,0x00,0x41,0x7f,0x7f,0x10,0x38,0x6c,0x44,
    0x00,0x41,0x7f,0x7f,0x40,0x00,0x00,0x7c,0x7c,0x18,0x78,0x1c,0x7c,0x78,
    0x7c,0x78,0x04,0x04,0x7c,0x78,0x00,0x38,0x7c,0x44,0x44,0x7c,0x38,0x00,
    0x84,0xfc,0xf8,0xa4,0x24,0x3c,0x18,0x18,0x3c,0x24,0xa4,0xf8,0xfc,0x84,
    0x44,0x7c,0x78,0x4c,0x04,0x0c,0x18,0x48,0x5c,0x54,0x74,0x64,0x24,0x00,
    0x04,0x04,0x3e,0x7f,0x44,0x24,0x00,0x3c,0x7c,0x40,0x40,0x3c,0x7c,0x40,
    0x1c,0x3c,0x60,0x60,0x3c,0x1c,0x00,0x3c,0x7c,0x60,0x30,0x60,0x7c,0x3c,
    0x44,0x6c,0x38,0x10,0x38,0x6c,0x44,0x9c,0xbc,0xa0,0xa0,0xfc,0x7c,0x00,
    0x4c,0x64,0x74,0x5c,0x4c,0x64,0x00,0x08,0x08,0x3e,0x77,0x41,0x41,0x00,
    0x00,0x00,0x00,0x77,0x77,0x00,0x00,0x41,0x41,0x77,0x3e,0x08,0x08,0x00,
    0x02,0x03,0x01,0x03,0x02,0x03,0x01,0x70,0x78,0x4c,0x46,0x4c,0x78,0x70};

// 5x7 font (in 6x8 cell)
const uint8_t ucSmallFont[] PROGMEM = {
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x06,0x5f,0x06,0x00,
    0x07,0x03,0x00,0x07,0x03,
    0x24,0x7e,0x24,0x7e,0x24,
    0x24,0x2b,0x6a,0x12,0x00,
    0x63,0x13,0x08,0x64,0x63,
    0x36,0x49,0x56,0x20,0x50,
    0x00,0x07,0x03,0x00,0x00,
    0x00,0x3e,0x41,0x00,0x00,
    0x00,0x41,0x3e,0x00,0x00,
    0x08,0x3e,0x1c,0x3e,0x08,
    0x08,0x08,0x3e,0x08,0x08,
    0x00,0xe0,0x60,0x00,0x00,
    0x08,0x08,0x08,0x08,0x08,
    0x00,0x60,0x60,0x00,0x00,
    0x20,0x10,0x08,0x04,0x02,
    0x3e,0x51,0x49,0x45,0x3e,
    0x00,0x42,0x7f,0x40,0x00,
    0x62,0x51,0x49,0x49,0x46,
    0x22,0x49,0x49,0x49,0x36,
    0x18,0x14,0x12,0x7f,0x10,
    0x2f,0x49,0x49,0x49,0x31,
    0x3c,0x4a,0x49,0x49,0x30,
    0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36,
    0x06,0x49,0x49,0x29,0x1e,
    0x00,0x6c,0x6c,0x00,0x00,
    0x00,0xec,0x6c,0x00,0x00,
    0x08,0x14,0x22,0x41,0x00,
    0x24,0x24,0x24,0x24,0x24,
    0x00,0x41,0x22,0x14,0x08,
    0x02,0x01,0x59,0x09,0x06,
    0x3e,0x41,0x5d,0x55,0x1e,
    0x7e,0x11,0x11,0x11,0x7e,
    0x7f,0x49,0x49,0x49,0x36,
    0x3e,0x41,0x41,0x41,0x22,
    0x7f,0x41,0x41,0x41,0x3e,
    0x7f,0x49,0x49,0x49,0x41,
    0x7f,0x09,0x09,0x09,0x01,
    0x3e,0x41,0x49,0x49,0x7a,
    0x7f,0x08,0x08,0x08,0x7f,
    0x00,0x41,0x7f,0x41,0x00,
    0x30,0x40,0x40,0x40,0x3f,
    0x7f,0x08,0x14,0x22,0x41,
    0x7f,0x40,0x40,0x40,0x40,
    0x7f,0x02,0x04,0x02,0x7f,
    0x7f,0x02,0x04,0x08,0x7f,
    0x3e,0x41,0x41,0x41,0x3e,
    0x7f,0x09,0x09,0x09,0x06,
    0x3e,0x41,0x51,0x21,0x5e,
    0x7f,0x09,0x09,0x19,0x66,
    0x26,0x49,0x49,0x49,0x32,
    0x01,0x01,0x7f,0x01,0x01,
    0x3f,0x40,0x40,0x40,0x3f,
    0x1f,0x20,0x40,0x20,0x1f,
    0x3f,0x40,0x3c,0x40,0x3f,
    0x63,0x14,0x08,0x14,0x63,
    0x07,0x08,0x70,0x08,0x07,
    0x71,0x49,0x45,0x43,0x00,
    0x00,0x7f,0x41,0x41,0x00,
    0x02,0x04,0x08,0x10,0x20,
    0x00,0x41,0x41,0x7f,0x00,
    0x04,0x02,0x01,0x02,0x04,
    0x80,0x80,0x80,0x80,0x80,
    0x00,0x03,0x07,0x00,0x00,
    0x20,0x54,0x54,0x54,0x78,
    0x7f,0x44,0x44,0x44,0x38,
    0x38,0x44,0x44,0x44,0x28,
    0x38,0x44,0x44,0x44,0x7f,
    0x38,0x54,0x54,0x54,0x08,
    0x08,0x7e,0x09,0x09,0x00,
    0x18,0xa4,0xa4,0xa4,0x7c,
    0x7f,0x04,0x04,0x78,0x00,
    0x00,0x00,0x7d,0x40,0x00,
    0x40,0x80,0x84,0x7d,0x00,
    0x7f,0x10,0x28,0x44,0x00,
    0x00,0x00,0x7f,0x40,0x00,
    0x7c,0x04,0x18,0x04,0x78,
    0x7c,0x04,0x04,0x78,0x00,
    0x38,0x44,0x44,0x44,0x38,
    0xfc,0x44,0x44,0x44,0x38,
    0x38,0x44,0x44,0x44,0xfc,
    0x44,0x78,0x44,0x04,0x08,
    0x08,0x54,0x54,0x54,0x20,
    0x04,0x3e,0x44,0x24,0x00,
    0x3c,0x40,0x20,0x7c,0x00,
    0x1c,0x20,0x40,0x20,0x1c,
    0x3c,0x60,0x30,0x60,0x3c,
    0x6c,0x10,0x10,0x6c,0x00,
    0x9c,0xa0,0x60,0x3c,0x00,
    0x64,0x54,0x54,0x4c,0x00,
    0x08,0x3e,0x41,0x41,0x00,
    0x00,0x00,0x77,0x00,0x00,
    0x00,0x41,0x41,0x3e,0x08,
    0x02,0x01,0x02,0x01,0x00,
    0x3c,0x26,0x23,0x26,0x3c};

void bbepFillScreen(FASTEPDSTATE *pState, uint8_t u8Color)
{
    int iPitch;
    if (pState->mode == BB_MODE_1BPP) {
        if (u8Color == BBEP_WHITE) u8Color = 0xff;
        iPitch = (pState->width + 7) / 8;
    } else {
        iPitch = (pState->width + 1) / 2;
        u8Color |= (u8Color << 4);
    }
    memset(pState->pCurrent, u8Color, iPitch * pState->height);
} /* bbepFillScreen() */
//
// Draw a sprite of any size in any position
// If it goes beyond the left/right or top/bottom edges
// it's trimmed to show the valid parts
// This function requires a back buffer to be defined
// The priority color (0 or 1) determines which color is painted
// when a 1 is encountered in the source image.
//
void bbepDrawSprite(FASTEPDSTATE *pBBEP, const uint8_t *pSprite, int cx, int cy, int iPitch, int x, int y, uint8_t iColor)
{
    int tx, ty, dx, dy, iStartX;
    uint8_t *s, pix, ucSrcMask;
    
    if (pBBEP == NULL) return;
    if (x+cx < 0 || y+cy < 0 || x >= pBBEP->native_width || y >= pBBEP->native_height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // out of bounds
    }
    dy = y; // destination y
    if (y < 0) // skip the invisible parts
    {
        cy += y;
        y = -y;
        pSprite += (y * iPitch);
        dy = 0;
    }
    if ((dy + cy) > pBBEP->native_height) {
        cy = pBBEP->native_height - y;
    }
    iStartX = 0;
    dx = x;
    if (x < 0)
    {
        cx += x;
        x = -x;
        iStartX = x;
        dx = 0;
    }
    if ((x + cx) > pBBEP->native_width)
        cx = pBBEP->native_width - x;
    for (ty=0; ty<cy; ty++)
    {
        s = (uint8_t *)&pSprite[(iStartX >> 3)];
        ucSrcMask = 0x80 >> (iStartX & 7);
        pix = *s++;
        for (tx=0; tx<cx; tx++) {
            if (pix & ucSrcMask) { // set pixel in source, set it in dest
                (*pBBEP->pfnSetPixelFast)(pBBEP, dx+tx, dy+ty, iColor);
            }
            ucSrcMask >>= 1;
            if (ucSrcMask == 0) { // read next byte
                ucSrcMask = 0x80;
                pix = *s++;
            }
        } // for tx
        pSprite += iPitch;
    } // for ty
} /* bbepDrawSprite() */

int bbepSetPixel2Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i = 0;
    int iPitch;
    uint8_t u8, u8Mask = 0;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    // only available for local buffer operations
    if (!pBBEP) return BBEP_ERROR_BAD_PARAMETER;
    
    iPitch = (pBBEP->native_width+7)>>3;
    
    if (x < 0 || x >= pBBEP->width || y < 0 || y >= pBBEP->height) { // off the screen
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    switch (pBBEP->rotation) {
        case 0:
            i = (x >> 3) + (y * iPitch);
            u8Mask = 0x80 >> (x & 7);
            break;
        case 90:
            i = (y >> 3) + ((pBBEP->width - 1 - x) * iPitch);
            u8Mask = 0x80 >> (y & 7);
            break;
        case 180:
            i = ((pBBEP->width - 1 - x) >> 3) + ((pBBEP->height - 1 - y) * iPitch);
            u8Mask = 1 << (x & 7);
            break;
        case 270:
            i = ((pBBEP->height - 1 - y) >> 3) + (x * iPitch);
            u8Mask = 1 << (y & 7);
            break;
    }
    u8 = pBBEP->pCurrent[i];
    if (ucColor == BBEP_WHITE) {
        u8 |= u8Mask;
    } else { // must be black
        u8 &= ~u8Mask;
    }
    pBBEP->pCurrent[i] = u8;
    return BBEP_SUCCESS;
} /* bbepSetPixel2Clr() */

void bbepSetPixelFast2Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = (pBBEP->width+7)>>3;
    
    i = (x >> 3) + (y * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (ucColor == BBEP_WHITE) {
        u8 |= (0x80 >> (x & 7));
    } else { // must be black
        u8 &= ~(0x80 >> (x & 7));
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast2Clr() */

void bbepSetPixelFast2Clr_180(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = (pBBEP->width+7)>>3;
    
    i = ((pBBEP->width-1-x) >> 3) + ((pBBEP->height-1-y) * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (ucColor == BBEP_WHITE) {
        u8 |= (1 << (x & 7));
    } else { // must be black
        u8 &= ~(1 << (x & 7));
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast2Clr_180() */

void bbepSetPixelFast2Clr_90(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = (pBBEP->native_width+7)>>3;
    
    i = (y >> 3) + ((pBBEP->width-1-x) * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (ucColor == BBEP_WHITE) {
        u8 |= (0x80 >> (y & 7));
    } else { // must be black
        u8 &= ~(0x80 >> (y & 7));
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast2Clr_90() */

void bbepSetPixelFast2Clr_270(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = (pBBEP->native_width+7)>>3;
    
    i = ((pBBEP->height-1-y) >> 3) + (x * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (ucColor == BBEP_WHITE) {
        u8 |= (1 << (y & 7));
    } else { // must be black
        u8 &= ~(1 << (y & 7));
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast2Clr_270() */

int bbepSetPixel16Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i = 0;
    int iPitch;
    uint8_t u8, u8Mask = 0xf0;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    // only available for local buffer operations
    if (!pBBEP) return BBEP_ERROR_BAD_PARAMETER;
    
    iPitch = pBBEP->native_width >> 1;
    
    if (x < 0 || x >= pBBEP->width || y < 0 || y >= pBBEP->height) { // off the screen
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    switch (pBBEP->rotation) {
        case 0:
            i = (x >> 1) + (y * iPitch);
            if (x & 1) {
                u8Mask >>= 4;
                ucColor <<= 4;
            }
            break;
        case 90:
            i = (y >> 1) + ((pBBEP->width - 1 - x) * iPitch);
            if (x & 1) {
                u8Mask >>= 4;
                ucColor <<= 4;
            }
            break;
        case 180:
            i = ((pBBEP->width - 1 - x) >> 1) + ((pBBEP->height - 1 - y) * iPitch);
            if (!(x & 1)) {
                u8Mask >>= 4;
                ucColor <<= 4;
            }
            break;
        case 270:
            i = ((pBBEP->height - 1 - y) >> 1) + (x * iPitch);
            if (!(y & 1)) {
                u8Mask >>= 4;
                ucColor <<= 4;
            }
            break;
    }
    u8 = pBBEP->pCurrent[i];
    u8 = (u8 & u8Mask) | ucColor;
    pBBEP->pCurrent[i] = u8;
    return BBEP_SUCCESS;
} /* bbepSetPixel16Clr() */

void bbepSetPixelFast16Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = pBBEP->native_width >> 1;
    i = (x >> 1) + (y * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (x & 1) {
        u8 &= 0xf0;
        u8 |= ucColor;
    } else {
        u8 &= 0x0f;
        u8 |= (ucColor << 4);
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast16Clr() */

void bbepSetPixelFast16Clr_90(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = pBBEP->native_width >> 1;
    i = (y >> 1) + ((pBBEP->width-1-x) * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (y & 1) {
        u8 &= 0xf0;
        u8 |= ucColor;
    } else {
        u8 &= 0x0f;
        u8 |= (ucColor << 4);
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast16Clr_90() */

void bbepSetPixelFast16Clr_180(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = pBBEP->native_width >> 1;
    i = ((pBBEP->width-1-x) >> 1) + ((pBBEP->height-1-y) * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (!(x & 1)) {
        u8 &= 0xf0;
        u8 |= ucColor;
    } else {
        u8 &= 0x0f;
        u8 |= (ucColor << 4);
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast16Clr_180() */

void bbepSetPixelFast16Clr_270(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    FASTEPDSTATE *pBBEP = (FASTEPDSTATE *)pb;
    
    iPitch = pBBEP->native_width >> 1;
    i = ((pBBEP->height-1-y) >> 1) + (x * iPitch);
    u8 = pBBEP->pCurrent[i];
    if (!(y & 1)) {
        u8 &= 0xf0;
        u8 |= ucColor;
    } else {
        u8 &= 0x0f;
        u8 |= (ucColor << 4);
    }
    pBBEP->pCurrent[i] = u8;
} /* bbepSetPixelFast16Clr_270() */

//
// Invert font data
//
void InvertBytes(uint8_t *pData, uint8_t bLen)
{
    uint8_t i;
    for (i=0; i<bLen; i++)
    {
        *pData = ~(*pData);
        pData++;
    }
} /* InvertBytes() */
//
// Load a 1-bpp Group5 compressed bitmap
// Pass the pointer to the beginning of the G5 file
// If the FG == BG color, and there is a back buffer, it will
// draw the 1's bits as the FG color and leave
// the background (0 pixels) unchanged - aka transparent.
//
int bbepLoadG5(FASTEPDSTATE *pBBEP, const uint8_t *pG5, int x, int y, int iFG, int iBG)
{
    uint16_t rc, tx, ty, cx, cy, size;
    BB_BITMAP *pbbb;
    
    if (pBBEP == NULL || pG5 == NULL) return BBEP_ERROR_BAD_PARAMETER;
    pbbb = (BB_BITMAP *)pG5;
    if (pgm_read_word(&pbbb->u16Marker) != BB_BITMAP_MARKER) return BBEP_ERROR_BAD_DATA;
    cx = pgm_read_word(&pbbb->width);
    cy = pgm_read_word(&pbbb->height);
    size = pgm_read_word(&pbbb->size);
    if (iFG == -1) iFG = BBEP_WHITE;
    if (iBG == -1) iBG = BBEP_BLACK;
    rc = g5_decode_init(&g5dec, cx, cy, (uint8_t *)&pbbb[1], size);
    if (rc != G5_SUCCESS) return BBEP_ERROR_BAD_DATA; // corrupt data?
    for (ty=y; ty<y+cy && ty < pBBEP->height; ty++) {
        uint8_t u8, *s, src_mask;
        g5_decode_line(&g5dec, u8Cache);
            src_mask = 0; // make it read a byte to start
            s = u8Cache;
            for (tx=x; tx<x+cx; tx++) {
                if (src_mask == 0) { // need to load the next byte
                    u8 = *s++;
                    src_mask = 0x80; // MSB on left
                }
                if (u8 & src_mask) {
                    if (iFG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, (uint8_t)iFG);
                } else {
                    if (iBG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, (uint8_t)iBG);
                }
                src_mask >>= 1;
            } // for tx
    } // for y
    return BBEP_SUCCESS;
} /* bbepLoadG5() */
//
// Load a 1-bpp Windows bitmap
// Pass the pointer to the beginning of the BMP file
// If the FG == BG color, it will
// draw the 1's bits as the FG color and leave
// the background (0 pixels) unchanged - aka transparent.
//
int bbepLoadBMP(FASTEPDSTATE *pBBEP, const uint8_t *pBMP, int dx, int dy, int iFG, int iBG)
{
    int16_t i16, cx, cy;
    int iOffBits; // offset to bitmap data
    int x, y, iPitch;
    uint8_t b=0, *s;
    uint8_t src_mask;
    uint8_t bFlipped = 0;
    
    // Don't use pgm_read_word because it can cause an unaligned
    // access on RP2040 for odd addresses
    i16 = pgm_read_byte(pBMP);
    i16 += (pgm_read_byte(&pBMP[1]) << 8);
    if (i16 != 0x4d42) { // must start with 'BM'
        pBBEP->last_error = BBEP_ERROR_BAD_DATA;
        return BBEP_ERROR_BAD_DATA; // not a BMP file
    }
    cx = pgm_read_byte(pBMP + 18);
    cx += (pgm_read_byte(pBMP+19)<<8);
    if (cx + dx > pBBEP->width) { // must fit on the display
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    cy = pgm_read_byte(pBMP + 22);
    cy += (pgm_read_byte(pBMP+23)<<8);
    if (cy < 0) cy = -cy;
    else bFlipped = 1;
    if (cy + dy > pBBEP->height) { // must fit on the display
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }    
    i16 = pgm_read_byte(pBMP + 28);
    i16 += (pgm_read_byte(pBMP+29)<<8);
    if (i16 != 1) { // must be 1 bit per pixel
        pBBEP->last_error = BBEP_ERROR_BAD_DATA;
        return BBEP_ERROR_BAD_DATA;
    }
    iOffBits = pgm_read_byte(pBMP + 10);
    iOffBits += (pgm_read_byte(pBMP+11));
    iPitch = (((cx+7)>>3) + 3) & 0xfffc; // must be DWORD aligned
    if (bFlipped)
    {
        iOffBits += ((cy-1) * iPitch); // start from bottom
        iPitch = -iPitch;
    }
    for (y=0; y<cy; y++) {
        s = (uint8_t *)&pBMP[iOffBits + (y*iPitch)];
            src_mask = 0; // make it read a byte to start
            for (x=0; x<cx; x++) {
                if (src_mask == 0) { // need to load the next byte
                    b = pgm_read_byte(s++);
                    src_mask = 0x80; // MSB on left
                }
                if (b & src_mask) {
                    if (iFG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, dx+x, dy+y, (uint8_t)iFG);
                } else {
                    if (iBG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, dx+x, dy+y, (uint8_t)iBG);
                }
                src_mask >>= 1;
            } // for x
    } // for y
    return BBEP_SUCCESS;
} /* bbepLoadBMP() */

//
// Set the current cursor position
// The column represents the pixel column (0-127)
// The row represents the text row (0-7)
//
void bbepSetCursor(FASTEPDSTATE *pBBEP, int x, int y)
{
    pBBEP->iCursorX = x;
    pBBEP->iCursorY = y;
} /* bbepSetCursor() */
//
// Turn text wrap on or off for the bbepWriteString() function
//
void bbepSetTextWrap(FASTEPDSTATE *pBBEP, int bWrap)
{
    pBBEP->wrap = bWrap;
} /* bbepSetTextWrap() */
//
// Draw a string of BB_FONT characters directly into the EPD framebuffer
//
int bbepWriteStringCustom(FASTEPDSTATE *pBBEP, BB_FONT *pFont, int x, int y, char *szMsg, int iColor)
{
    int rc, i, h, w, x_off, end_y, dx, dy, tx, ty, tw, iBG;
    signed int n;
    unsigned int c;
    uint8_t *s;
    BB_GLYPH *pGlyph;
    uint8_t *pBits, u8EndMask;
    uint8_t first, last;
    
    if (pBBEP == NULL) return BBEP_ERROR_BAD_PARAMETER;
    if (pFont == NULL) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER; // invalid param
    }
    iBG = pBBEP->iBG;
    if (iBG == -1) iBG = BBEP_TRANSPARENT; // -1 = don't care
    if (x == -1)
        x = pBBEP->iCursorX;
    if (y == -1)
        y = pBBEP->iCursorY;
    first = pgm_read_byte(&pFont->first);
    last = pgm_read_byte(&pFont->last);
    if (x == CENTER_X) { // center the string on the e-paper
        dx = i = 0;
        while (szMsg[i]) {
            c = szMsg[i++];
            if (c < first || c > last) // undefined character
                continue; // skip it
            c -= first; // first char of font defined
            pGlyph = &pFont->glyphs[c]; // glyph info for this character
            dx += pgm_read_byte(&pGlyph->xAdvance);
        }
        x = (pBBEP->width - dx)/2;
        if (x < 0) x = 0;
    }
    // Point to the start of the compressed data
    pBits = (uint8_t *)pFont;
    pBits += sizeof(BB_FONT);
    pBits += (last - first + 1) * sizeof(BB_GLYPH);
    i = 0;
    while (szMsg[i] && x < pBBEP->width && y < pBBEP->height) {
        c = szMsg[i++];
        if (c < first || c > last) // undefined character
            continue; // skip it
        c -= first; // first char of font defined
        pGlyph = &pFont->glyphs[c]; // glyph info for this character
        if (pgm_read_byte(&pGlyph->width) > 1) { // skip this if drawing a space
            x_off = pGlyph->xOffset;
            s = pBits + pgm_read_word(&pGlyph->bitmapOffset); // start of compressed bitmap data
            if (pgm_read_dword(&pFont->rotation) == 0 || pgm_read_dword(&pFont->rotation) == 180) {
                h = pgm_read_word(&pGlyph->height);
                w = pgm_read_byte(&pGlyph->width);
                dx = x + (int16_t)pgm_read_word(&pGlyph->xOffset); // offset from character UL to start drawing
                dy = y + (int16_t)pgm_read_word(&pGlyph->yOffset);
            } else { // rotated
                w = pgm_read_word(&pGlyph->height);
                h = pgm_read_byte(&pGlyph->width);
                n = (int16_t)pgm_read_word(&pGlyph->yOffset); // offset from character UL to start drawing
                dx = x;
                if (-n < w) dx -= (w+n); // since we draw from the baseline
                dy = y + (int16_t)pgm_read_word(&pGlyph->xOffset);
            }
            if ((dy + h) > pBBEP->height) { // trim it
                h = pBBEP->height - dy;
            }
            u8EndMask = 0xff;
            if (w & 7) { // width ends on a partial byte
                u8EndMask <<= (8-(w & 7));
            }
            end_y = dy + h;
            ty = (pgm_read_word(&pGlyph[1].bitmapOffset) - (intptr_t)(s - pBits)); // compressed size
            if (ty < 0 || ty > 4096) ty = 4096; // DEBUG
            rc = g5_decode_init(&g5dec, w, h, s, ty);
            if (rc != G5_SUCCESS) {
                pBBEP->last_error = BBEP_ERROR_BAD_DATA;
                 return BBEP_ERROR_BAD_DATA; // corrupt data?
            }
                tw = w;
                if (x+tw+x_off > pBBEP->width) tw = pBBEP->width - (x+x_off); // clip to right edge
                for (ty=dy; ty<end_y && ty < pBBEP->height; ty++) {
                    uint8_t u8, u8Count;
                    g5_decode_line(&g5dec, u8Cache);
                    s = u8Cache;
                    u8 = *s++;
                    u8Count = 8;
                    if (ty >= 0) { // don't draw off the screen
                        for (tx=x; tx<x+tw; tx++) {
                            if (u8 & 0x80) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, tx+x_off, ty, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, tx+x_off, ty, iBG);
                            }
                            u8 <<= 1;
                            u8Count--;
                            if (u8Count == 0) {
                                u8Count = 8;
                                u8 = *s++;
                            }
                        }
                    }
                }
        } // if not drawing a space
        if (pgm_read_dword(&pFont->rotation) == 0 || pgm_read_dword(&pFont->rotation) == 180) {
            x += pgm_read_byte(&pGlyph->xAdvance); // width of this character
        } else {
            y += pgm_read_byte(&pGlyph->xAdvance);
        }
    } // while drawing characters
    pBBEP->iCursorX = x;
    pBBEP->iCursorY = y;
    return BBEP_SUCCESS;
} /* EPDWriteStringCustom() */
//
// Rotate an 8x8 pixel block (8 bytes) by 90 degrees
// Used to draw the built-in font at 2 angles
//
void RotateCharBox(uint8_t *pSrc)
{
    int x, y;
    uint8_t ucDest[8], uc, ucSrcMask, ucDstMask;
    
    for (y=0; y<8; y++) {
        ucSrcMask = 1<<y;
        ucDstMask = 0x80;
        uc = 0;
        for (x=0; x<8; x++) {
            if (pSrc[x] & ucSrcMask) uc |= ucDstMask;
            ucDstMask >>= 1;
        }
        ucDest[y] = uc;
    }
    memcpy(pSrc, ucDest, 8); // rotate "in-place"
} /* RotateCharBox() */
//
// Double the size of a 1-bpp image and smooth the jaggies
//
void bbepStretchAndSmooth(uint8_t *pSrc, uint8_t *pDest, int w, int h, int iSmooth)
{
uint8_t c, uc1, uc2, *s, *d, ucMask;
int tx, ty, iPitch, iDestPitch;

    iPitch = (w+7)>>3;
    iDestPitch = ((w*2)+7)>>3;
// First, double the size of the source bitmap into the destination
    uc1 = uc2 = 0;
    memset(pDest, 0, iDestPitch * h*2);
    for (ty=0; ty<h; ty++) {
        s = &pSrc[ty * iPitch];
        d = &pDest[(ty*2) * iDestPitch];
        c = *s++;
        ucMask = 0xc0; // destination mask
        for (tx=0; tx<w; tx++) {
            if (c & 0x80) // a left-nibble bit is set
                uc1 |= ucMask;
            if (c & 0x08) // a right-nibble bit is set
                uc2 |= ucMask;
            ucMask >>= 2;
            c <<= 1;
            if (ucMask == 0) {
                c = *s++;
                d[0] = d[iDestPitch] = uc1;
                d[1] = d[iDestPitch+1] = uc2;
                d += 2;
                ucMask = 0xc0;
                tx += 4;
                uc1 = uc2 = 0;
            }
        } // for tx
    } // for ty
    if (iSmooth == BBEP_SMOOTH_NONE) return;
    // Now that the image is stretched, go through it and fill in corners
    // to smooth the blockiness
    for (ty = 0; ty < h; ty ++) {
        s = &pSrc[ty * iPitch];
        d = &pDest[ty*2 * iDestPitch];
        for (tx=0; tx<w-1; tx++) {
            uint8_t c00, c10, c01, c11; // 4 pixel group
            int dx = (tx*2)+1;
            c00 = s[tx>>3] & (0x80 >> (tx & 7));
            c10 = s[(tx+1)>>3] & (0x80 >> ((tx+1) & 7));
            c01 = s[iPitch + (tx>>3)] & (0x80 >> (tx & 7));
            c11 = s[iPitch + ((tx+1)>>3)] & (0x80 >> ((tx+1) & 7));
            if (iSmooth == BBEP_SMOOTH_HEAVY) {
                if (c00 && c11 && (!c10 || !c01)) {
                    // fill in the 'hole'
                    d[(dx>>3)+iDestPitch*2] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3) + iDestPitch] |= (0x80 >> ((dx+1) & 7));
                }
                if (c10 && c01 && (!c11 || !c00)) {
                    d[(dx>>3)+iDestPitch] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3)+iDestPitch*2] |= (0x80 >> ((dx+1) & 7));
                }
            } else { // BBEP_SMOOTH_LIGHT
                if ((c00 && !c10 && !c01 && c11) || (!c00 && c10 && c01 && !c11)) {
                    d[(dx>>3)+iDestPitch*2] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3) + iDestPitch] |= (0x80 >> ((dx+1) & 7));
                    d[(dx>>3)+iDestPitch] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3)+iDestPitch*2] |= (0x80 >> ((dx+1) & 7));
                }
            }
        } // for tx
    } // for ty
} /* bbepStretchAndSmooth() */
//
// Draw a string of normal (8x8), small (6x8) or large (16x32) characters
// At the given col+row
//
int bbepWriteString(FASTEPDSTATE *pBBEP, int x, int y, char *szMsg, int iSize, int iColor)
{
    int i, iFontOff, iLen;
    uint8_t c, *s;
    int iOldFG; // old fg color to make sure red works
    uint8_t u8Temp[40];
    int iBG;
    
    if (pBBEP == NULL) {
        return BBEP_ERROR_BAD_PARAMETER;
    }
    iBG = pBBEP->iBG;
    if (iBG == -1) iBG = BBEP_TRANSPARENT; // -1 = don't care
    
    if (x == -1 || y == -1) // use the cursor position
    {
        x = pBBEP->iCursorX; y = pBBEP->iCursorY;
    } else {
        pBBEP->iCursorX = x; pBBEP->iCursorY = y;
    }
    if (pBBEP->iCursorX >= pBBEP->width || pBBEP->iCursorY >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER; // can't draw off the display
    }
    
    iOldFG = pBBEP->iFG; // save old fg color
    if (iSize == FONT_8x8 || iSize == FONT_16x16) // 8x8 font (and stretched)
    {
        int iCount = (iSize == FONT_8x8) ? 8 : 16;
        i = 0;
        while (x < pBBEP->width && szMsg[i] != 0 && y < pBBEP->height)
        {
            c = (unsigned char)szMsg[i];
            iFontOff = (int)(c-32) * 7;
            // we can't directly use the pointer to FLASH memory, so copy to a local buffer
            u8Temp[0] = 0; // first column is blank
            memcpy_P(&u8Temp[1], &ucFont[iFontOff], 7); // only needed on AVR
            iLen = 8;
            if (x + iLen > pBBEP->width) { // clip right edge
                iLen = pBBEP->width - x;
            }
                uint8_t *s, u8Mask;
                if (iCount == 8) {
                    for (int ty=0; ty<8; ty++) {
                        u8Mask = 1<<ty;
                        for (int tx = 0; tx<iLen; tx++) {
                            if (u8Temp[tx] & u8Mask) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iBG);
                            }
                        }
                    }
                } else { // 16x16
                    bbepStretchAndSmooth(u8Temp, u8Cache, 8, 8, 1); // smooth too
                    for (int ty=0; ty<16; ty++) {
                        s = &u8Cache[2*ty];
                        u8Mask = 0x80;
                        for (int tx = 7; tx>=0; tx--) {
                            if (s[0] & u8Mask) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx+8, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx+8, iBG);
                            }
                            if (s[1] & u8Mask) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx+8, iBG);
                            }
                            u8Mask >>= 1;
                        }
                    }
                }
            x += iCount;
            if (x >= pBBEP->width-iCount-1 && pBBEP->wrap) { // word wrap enabled?
                x = 0; // start at the beginning of the next line
                y += iCount;
            }
            i++;
        } // while
        pBBEP->iFG = iOldFG; // restore color
        pBBEP->iCursorX = x;
        pBBEP->iCursorY = y;
        return BBEP_SUCCESS;
    } else if (iSize == FONT_12x16) { // 6x8 stretched to 12x16
        i = 0;
        while (pBBEP->iCursorX < pBBEP->width && pBBEP->iCursorY < pBBEP->height && szMsg[i] != 0) {
            // stretch the 'normal' font instead of using the big font
            int tx, ty;
            c = szMsg[i] - 32;
            unsigned char uc1, uc2, ucMask, *pDest;
            s = (unsigned char *)&ucSmallFont[(int)c*5];
            u8Temp[0] = 0; // first column is blank
            memcpy_P(&u8Temp[1], s, 6);
            // Stretch the font to double width + double height
            memset(&u8Temp[6], 0, 24); // write 24 new bytes
            for (tx=0; tx<6; tx++)
            {
                ucMask = 3;
                pDest = &u8Temp[6+tx*2];
                uc1 = uc2 = 0;
                c = u8Temp[tx];
                for (ty=0; ty<4; ty++)
                {
                    if (c & (1 << ty)) // a bit is set
                        uc1 |= ucMask;
                    if (c & (1 << (ty + 4)))
                        uc2 |= ucMask;
                    ucMask <<= 2;
                }
                pDest[0] = uc1;
                pDest[1] = uc1; // double width
                pDest[12] = uc2;
                pDest[13] = uc2;
            }
            // smooth the diagonal lines
            for (tx=0; tx<5; tx++)
            {
                uint8_t c0, c1, ucMask2;
                c0 = u8Temp[tx];
                c1 = u8Temp[tx+1];
                pDest = &u8Temp[6+tx*2];
                ucMask = 1;
                ucMask2 = 2;
                for (ty=0; ty<7; ty++)
                {
                    if (((c0 & ucMask) && !(c1 & ucMask) && !(c0 & ucMask2) && (c1 & ucMask2)) || (!(c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && !(c1 & ucMask2)))
                    {
                        if (ty < 3) // top half
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[1] &= ~(1 << ((ty * 2)+1));
                                pDest[2] &= ~(1 << ((ty * 2)+1));
                                pDest[1] &= ~(1 << ((ty+1) * 2));
                                pDest[2] &= ~(1 << ((ty+1) * 2));
                            } else {
                                pDest[1] |= (1 << ((ty * 2)+1));
                                pDest[2] |= (1 << ((ty * 2)+1));
                                pDest[1] |= (1 << ((ty+1) * 2));
                                pDest[2] |= (1 << ((ty+1) * 2));
                            }
                        }
                        else if (ty == 3) // on the border
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[1] &= ~0x80; pDest[2] &= ~0x80;
                                pDest[13] &= ~1; pDest[14] &= ~1;
                            } else {
                                pDest[1] |= 0x80; pDest[2] |= 0x80;
                                pDest[13] |= 1; pDest[14] |= 1;
                            }
                        }
                        else // bottom half
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[13] &= ~(1 << (2*(ty-4)+1));
                                pDest[14] &= ~(1 << (2*(ty-4)+1));
                                pDest[13] &= ~(1 << ((ty-3) * 2));
                                pDest[14] &= ~(1 << ((ty-3) * 2));
                            } else {
                                pDest[13] |= (1 << (2*(ty-4)+1));
                                pDest[14] |= (1 << (2*(ty-4)+1));
                                pDest[13] |= (1 << ((ty-3) * 2));
                                pDest[14] |= (1 << ((ty-3) * 2));
                            }
                        }
                    }
                    else if (!(c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && !(c1 & ucMask2))
                    {
                        if (ty < 4) // top half
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[1] &= ~(1 << ((ty * 2)+1));
                                pDest[2] &= ~(1 << ((ty+1) * 2));
                            } else {
                                pDest[1] |= (1 << ((ty * 2)+1));
                                pDest[2] |= (1 << ((ty+1) * 2));
                            }
                        }
                        else
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[13] &= ~(1 << (2*(ty-4)+1));
                                pDest[14] &= ~(1 << ((ty-3) * 2));
                            } else {
                                pDest[13] |= (1 << (2*(ty-4)+1));
                                pDest[14] |= (1 << ((ty-3) * 2));
                            }
                        }
                    }
                    ucMask <<= 1; ucMask2 <<= 1;
                }
            }
            iLen = 12;
            if (pBBEP->iCursorX + iLen > pBBEP->width) {// clip right edge
                iLen = pBBEP->width - pBBEP->iCursorX;
            }
            uint8_t u8Mask;
            for (int ty=0; ty<8; ty++) {
                u8Mask = 1<<ty;
                for (int tx = 0; tx<iLen; tx++) {
                    if (u8Temp[6+tx] & u8Mask) {
                        if (iColor != BBEP_TRANSPARENT) {
                            (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iColor);
                        }
                    } else if (iBG != BBEP_TRANSPARENT) {
                        (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iBG);
                    }
                    if (u8Temp[18+tx] & u8Mask) {
                        if (iColor != BBEP_TRANSPARENT) {
                            (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty+8, iColor);
                        }
                    } else if (iBG != BBEP_TRANSPARENT) {
                        (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty+8, iBG);
                    }
                }
            }
            x = pBBEP->iCursorX += iLen;
            if (pBBEP->iCursorX >= pBBEP->width-11 && pBBEP->wrap) // word wrap enabled?
            {
                x = pBBEP->iCursorX = 0; // start at the beginning of the next line
                y = pBBEP->iCursorY = y+16;
            }
            i++;
        } // while
        pBBEP->iFG = iOldFG; // restore color
        return BBEP_SUCCESS;
    } // 12x16
    else if (iSize == FONT_6x8)
    {
        i = 0;
        while (pBBEP->iCursorX < pBBEP->width && pBBEP->iCursorY < pBBEP->height && szMsg[i] != 0)
        {
            c = szMsg[i] - 32;
            // we can't directly use the pointer to FLASH memory, so copy to a local buffer
            u8Temp[0] = 0; // first column is blank
            memcpy_P(&u8Temp[1], &ucSmallFont[(int)c*5], 5);
            iLen = 6;
            if (pBBEP->iCursorX + iLen > pBBEP->width) {// clip right edge
                iLen = pBBEP->width - pBBEP->iCursorX;
            }
            uint8_t u8Mask;
            for (int ty=0; ty<8; ty++) {
                u8Mask = 1<<ty;
                for (int tx = 0; tx<iLen; tx++) {
                    if (u8Temp[tx] & u8Mask) {
                        if (iColor != BBEP_TRANSPARENT) {
                            (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, ty+y, iColor);
                        }
                    } else if (iBG != BBEP_TRANSPARENT) {
                        (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, ty+y, iBG);
                    }
                }
            }
            pBBEP->iCursorX += iLen;
            if (pBBEP->iCursorX >= pBBEP->width-5 && pBBEP->wrap) // word wrap enabled?
            {
                x = pBBEP->iCursorX = 0; // start at the beginning of the next line
                y = pBBEP->iCursorY = y+8;
            }
            i++;
        }
        pBBEP->iFG = iOldFG; // restore color
        return BBEP_SUCCESS;
    } // 6x8
    pBBEP->iFG = iOldFG; // restore color
    pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
    return BBEP_ERROR_BAD_PARAMETER; // invalid size
} /* bbepWriteString() */
//
// Get the bounding rectangle of text
// The position of the rectangle is based on the current cursor position and font
//
int bbepGetStringBox(FASTEPDSTATE *pBBEP, const char *szMsg, BBEPRECT *pRect)
{
    int cx = 0;
    unsigned int c, i = 0;
    BB_GLYPH *pBBG;
    BB_FONT *pFont;
    int miny, maxy = 0;
    
    if (!pBBEP || !szMsg || !pRect) return BBEP_ERROR_BAD_PARAMETER;
    
    pFont = (BB_FONT *)pBBEP->pFont;
    if (pBBEP->iFont == -1 && pFont) { // custom font
        miny = 1000; maxy = 0;
        while (szMsg[i]) {
            c = szMsg[i++];
            if (c < pFont->first || c > pFont->last) // undefined character
                continue; // skip it
            c -= pFont->first; // first char of font defined
            pBBG = &pFont->glyphs[c];
            cx += pBBG->xAdvance;
            if (pBBG->yOffset < miny) miny = pBBG->yOffset;
            if (pBBG->height+pBBG->yOffset > maxy) maxy = pBBG->height+pBBG->yOffset;
        }
    } else { // fixed fonts
        miny = 0;
        switch (pBBEP->iFont) {
            case FONT_6x8:
                cx = 6;
                maxy = 8;
                break;
            case FONT_8x8:
                cx = 8;
                maxy = 8;
                break;
            case FONT_12x16:
                cx = 12;
                maxy = 16;
                break;
            case FONT_16x16:
                cx = 16;
                maxy = 16;
                break;
        }
        cx *= strlen(szMsg);
    }
    pRect->x = pBBEP->iCursorX;
    pRect->y = pBBEP->iCursorY + miny;
    pRect->w = cx;
    pRect->h = maxy - miny;
    return BBEP_SUCCESS;
} /* bbepGetStringBox() */
//
// Draw a line from x1,y1 to x2,y2 in the given color
// This function supports both buffered and bufferless drawing
//
void bbepDrawLine(FASTEPDSTATE *pBBEP, int x1, int y1, int x2, int y2, uint8_t ucColor)
{
    int temp;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int error;
    int xinc, yinc;
    
    if (pBBEP == NULL) {
        return;
    }
    
    if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || x1 >= pBBEP->width || x2 >= pBBEP->width || y1 >= pBBEP->height || y2 >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return;
    }
    if(abs(dx) > abs(dy)) {
        // X major case
        if(x2 < x1) {
            dx = -dx;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
        
        dy = (y2 - y1);
        error = dx >> 1;
        yinc = 1;
        if (dy < 0)
        {
            dy = -dy;
            yinc = -1;
        }
        for(; x1 <= x2; x1++) {
            (*pBBEP->pfnSetPixelFast)(pBBEP, x1, y1, ucColor);
            error -= dy;
            if (error < 0) {
                error += dx;
                y1 += yinc;
            }
        } // for x1
    } else {
        // Y major case
        if(y1 > y2) {
            dy = -dy;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
        dx = (x2 - x1);
        error = dy >> 1;
        xinc = 1;
        if (dx < 0) {
            dx = -dx;
            xinc = -1;
        }
        for(; y1 <= y2; y1++) {
            (*pBBEP->pfnSetPixelFast)(pBBEP, x1, y1, ucColor);
            error -= dx;
            if (error < 0) {
                error += dy;
                x1 += xinc;
            }
        } // for y
    } // y major case
} /* bbepDrawLine() */

//
// For drawing ellipses, a circle is drawn and the x and y pixels are scaled by a 16-bit integer fraction
// This function draws a single pixel and scales its position based on the x/y fraction of the ellipse
//
static void DrawScaledPixel(FASTEPDSTATE *pBBEP, int iCX, int iCY, int x, int y, int32_t iXFrac, int32_t iYFrac, uint8_t ucColor)
{
    if (iXFrac != 0x10000) x = ((x * iXFrac) >> 16);
    if (iYFrac != 0x10000) y = ((y * iYFrac) >> 16);
    x += iCX; y += iCY;
    if (x < 0 || x >= pBBEP->width || y < 0 || y >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // off the screen
    }
    (*pBBEP->pfnSetPixelFast)(pBBEP, x, y, ucColor);
} /* DrawScaledPixel() */
//
// For drawing filled ellipses
//
static void DrawScaledLine(FASTEPDSTATE *pBBEP, int iCX, int iCY, int x, int y, int32_t iXFrac, int32_t iYFrac, uint8_t ucColor)
{
    int iLen, x2;
    
    if (iXFrac != 0x10000) x = ((x * iXFrac) >> 16);
    if (iYFrac != 0x10000) y = ((y * iYFrac) >> 16);
    iLen = x*2;
    x = iCX - x; y += iCY;
    x2 = x + iLen;
    if (y < 0 || y >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // completely off the screen
    }
    if (x < 0) x = 0;
    if (x2 >= pBBEP->width) x2 = pBBEP->width-1;
    bbepDrawLine(pBBEP, x, y, x2, y, ucColor);
} /* DrawScaledLine() */
//
// Draw the 8 pixels around the Bresenham circle
// (scaled to make an ellipse)
//
static void BresenhamCircle(FASTEPDSTATE *pBBEP, int iCX, int iCY, int x, int y, int32_t iXFrac, int32_t iYFrac, uint8_t ucColor, uint8_t u8Parts, uint8_t bFill)
{
    if (bFill) // draw a filled ellipse
    {
        // for a filled ellipse, draw 4 lines instead of 8 pixels
        DrawScaledLine(pBBEP, iCX, iCY, x, y, iXFrac, iYFrac, ucColor);
        DrawScaledLine(pBBEP, iCX, iCY, x, -y, iXFrac, iYFrac, ucColor);
        DrawScaledLine(pBBEP, iCX, iCY, y, x, iXFrac, iYFrac, ucColor);
        DrawScaledLine(pBBEP, iCX, iCY, y, -x, iXFrac, iYFrac, ucColor);
    }
    else // draw 8 pixels around the edges
    {
        if (u8Parts & 1) {
            DrawScaledPixel(pBBEP, iCX, iCY, -x, -y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, -y, -x, iXFrac, iYFrac, ucColor);
        }
        if (u8Parts & 2) {
            DrawScaledPixel(pBBEP, iCX, iCY, x, -y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, y, -x, iXFrac, iYFrac, ucColor);
        }
        if (u8Parts & 4) {
            DrawScaledPixel(pBBEP, iCX, iCY, x, y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, y, x, iXFrac, iYFrac, ucColor);
        }
        if (u8Parts & 8) {
            DrawScaledPixel(pBBEP, iCX, iCY, -x, y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, -y, x, iXFrac, iYFrac, ucColor);
        }
    }
} /* BresenhamCircle() */
//
// Draw an outline or filled ellipse
//
void bbepEllipse(FASTEPDSTATE *pBBEP, int iCenterX, int iCenterY, int32_t iRadiusX, int32_t iRadiusY, uint8_t u8Parts, uint8_t ucColor, uint8_t bFilled)
{
    int32_t iXFrac, iYFrac;
    int iRadius, iDelta, x, y;
    
    if (pBBEP == NULL) return;
    
    if (iRadiusX <= 0 || iRadiusY <= 0) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // invalid radii
    }
    if (iRadiusX > iRadiusY) {// use X as the primary radius
        iRadius = iRadiusX;
        iXFrac = 65536;
        iYFrac = (iRadiusY * 65536) / iRadiusX;
    } else {
        iRadius = iRadiusY;
        iXFrac = (iRadiusX * 65536) / iRadiusY;
        iYFrac = 65536;
    }
    iDelta = 3 - (2 * iRadius);
    x = 0; y = iRadius;
    while (x <= y) {
        BresenhamCircle(pBBEP, iCenterX, iCenterY, x, y, iXFrac, iYFrac, ucColor, u8Parts, bFilled);
        x++;
        if (iDelta < 0) {
            iDelta += (4*x) + 6;
        } else {
            iDelta += 4 * (x-y) + 10;
            y--;
        }
    }
} /* bbepEllipse() */
//
// Draw an outline or filled rectangle
//
void bbepRectangle(FASTEPDSTATE *pBBEP, int x1, int y1, int x2, int y2, uint8_t ucColor, uint8_t bFilled)
{
    int tmp;
    
    if (pBBEP == NULL) {
        return; // invalid - must have FASTEPDSTATE structure
    }
    
    if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 ||
        x1 >= pBBEP->width || y1 >= pBBEP->height || x2 >= pBBEP->width || y2 >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // invalid coordinates
    }
    // Make sure that X1/Y1 is above and to the left of X2/Y2
    // swap coordinates as needed to make this true
    if (x2 < x1)
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;
    }
    if (y2 < y1)
    {
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }
    if (bFilled)
    {
            int tx, ty;
            for (ty = y1; ty <= y2; ty++) {
                for (tx = x1; tx <= x2; tx++) {
                    (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, ucColor);
                }
            }
    }
    else // outline (only support on backbuffer for now
    {
            int tx, ty;
            for (ty = y1; ty <= y2; ty++) {
                (*pBBEP->pfnSetPixelFast)(pBBEP, x1, ty, ucColor);
                (*pBBEP->pfnSetPixelFast)(pBBEP, x2, ty, ucColor);
            }
            for (tx = x1; tx <= x2; tx++) {
                (*pBBEP->pfnSetPixelFast)(pBBEP, tx, y1, ucColor);
                (*pBBEP->pfnSetPixelFast)(pBBEP, tx, y2, ucColor);
            }
    } // outline
} /* bbepRectangle() */
//
// Draw a rectangle (optionally filled) with rounded corners. The radius of the rounded corners is specified
//
void bbepRoundRect(FASTEPDSTATE *pBBEP, int x, int y, int w, int h, int r, uint8_t iColor, int bFilled)
{
    if (bFilled) {
        bbepRectangle(pBBEP, x+r, y, x+w-1-r, y+h, iColor, 1);
        bbepRectangle(pBBEP, x, y+r, x+w-1, y+h-r, iColor, 1);
        // draw four corners
        bbepEllipse(pBBEP, x+w-r-1, y+r, r, r, 1, iColor, 1);
        bbepEllipse(pBBEP, x+r, y+r, r, r, 2, iColor, 1);
        bbepEllipse(pBBEP, x+w-r-1, y+h-r, r, r, 1, iColor, 1);
        bbepEllipse(pBBEP, x+r, y+h-r, r, r, 2, iColor, 1);
    } else {
        bbepDrawLine(pBBEP, x+r, y, x+w-r, y, iColor); // top
        bbepDrawLine(pBBEP, x+r, y+h-1, x+w-r, y+h-1, iColor); // bottom
        bbepDrawLine(pBBEP, x, y+r, x, y+h-r, iColor); // left
        bbepDrawLine(pBBEP, x+w-1, y+r, x+w-1, y+h-r, iColor); // right
        // four corners
        bbepEllipse(pBBEP, x+r, y+r, r, r, 1, iColor, 0);
        bbepEllipse(pBBEP, x+w-r-1, y+r, r, r, 2, iColor, 0);
        bbepEllipse(pBBEP, x+w-r-1, y+h-r-1, r, r, 4, iColor, 0);
        bbepEllipse(pBBEP, x+r, y+h-r-1, r, r, 8, iColor, 0);
    }
} /* bbepRoundRect() */
//
// Set the display rotation angle (0, 90, 180, 270)
//
int bbepSetRotation(FASTEPDSTATE *pState, int iAngle)
{
    iAngle %= 360;
    if (iAngle % 90 != 0) return BBEP_ERROR_BAD_PARAMETER;
    pState->rotation = iAngle;
    // set the correct fast pixel function
    switch (iAngle) {
        case 0:
            pState->width = pState->native_width;
            pState->height = pState->native_height;
            if (pState->mode == BB_MODE_1BPP) {
                pState->pfnSetPixelFast = bbepSetPixelFast2Clr;
            } else {
                pState->pfnSetPixelFast = bbepSetPixelFast16Clr;
            }
            break;
        case 90:
            pState->width = pState->native_height;
            pState->height = pState->native_width;
            if (pState->mode == BB_MODE_1BPP) {
                pState->pfnSetPixelFast = bbepSetPixelFast2Clr_90;
            } else {
                pState->pfnSetPixelFast = bbepSetPixelFast16Clr_90;
            }
            break;
        case 180:
            pState->width = pState->native_width;
            pState->height = pState->native_height;
            if (pState->mode == BB_MODE_1BPP) {
                pState->pfnSetPixelFast = bbepSetPixelFast2Clr_180;
            } else {
                pState->pfnSetPixelFast = bbepSetPixelFast16Clr_180;
            }
            break;
        case 270:
            pState->width = pState->native_height;
            pState->height = pState->native_width;
            if (pState->mode == BB_MODE_1BPP) {
                pState->pfnSetPixelFast = bbepSetPixelFast2Clr_270;
            } else {
                pState->pfnSetPixelFast = bbepSetPixelFast16Clr_270;
            }
            break;
    }
    return BBEP_SUCCESS;
} /* bbepSetRotation() */

//
// Set the graphics mode (1-bit or 4-bits per pixel)
//
int bbepSetMode(FASTEPDSTATE *pState, int iMode)
{
    if (iMode != BB_MODE_1BPP && iMode != BB_MODE_4BPP) return BBEP_ERROR_BAD_PARAMETER;
    pState->mode = iMode;
    return bbepSetRotation(pState, pState->rotation); // set up correct pixel functions
} /* setMode() */

#endif // __BB_EP_GFX__

