#include "bb_epdiy.h"
#include "bb_epdiy.inl"
#include "bb_ep_gfx.inl"
#include <Wire.h>

#if !(defined(CONFIG_ESP32_SPIRAM_SUPPORT) || defined(CONFIG_ESP32S3_SPIRAM_SUPPORT))
#error "Please enable PSRAM support"
#endif
#pragma GCC optimize("O2")

void BBEPDIY::rowControl(int iMode)
{
    bbepRowControl(&_state, iMode);
}
void BBEPDIY::writeRow(uint8_t *pData, int iLen)
{
    bbepWriteRow(&_state, pData, iLen);
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

void BBEPDIY::fillRect(int x, int y, int w, int h,
                   uint8_t color)
{
    bbepRectangle(&_state, x, y, x+w-1, y+h-1, color, 1);
}

void BBEPDIY::drawLine(int x1, int y1, int x2, int y2, int iColor)
{
    bbepDrawLine(&_state, x1, y1, x2, y2, iColor);
} /* drawLine() */ 

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

int BBEPDIY::initCustomPanel(BBPANELDEF *pPanel)
{
    _state.iPanelType = BB_PANEL_CUSTOM;
    memcpy(&_state.panelDef, pPanel, sizeof(BBPANELDEF));
    return bbepIOInit(&_state);
} /* setPanelType() */

int BBEPDIY::initPanel(int iPanel)
{
    int rc;
    if (iPanel > 0 && iPanel < BB_PANEL_COUNT) {
        _state.iPanelType = iPanel;
        _state.width = panelDefs[iPanel].width;
        _state.height = panelDefs[iPanel].height;
        memcpy(&_state.panelDef, &panelDefs[iPanel], sizeof(BBPANELDEF));
        rc = bbepIOInit(&_state);
        _state.mode = BB_MODE_1BPP; // start in 1-bit mode
        if (rc == BBEP_SUCCESS) {
            // allocate memory for the buffers
            _state.pCurrent = (uint8_t *)ps_malloc(_state.width * _state.height / 8); // current pixels
            _state.pPrevious = (uint8_t *)ps_malloc(_state.width * _state.height / 8); // comparison with previous buffer
            _state.pTemp = (uint8_t *)ps_malloc(_state.width * _state.height / 4); // LUT data
        }
        _state.pfnSetPixel = bbepSetPixel2Clr;
        _state.pfnSetPixelFast = bbepSetPixelFast2Clr;
        return rc;
    }
    return BBEP_ERROR_BAD_PARAMETER;
} /* initIO() */

int BBEPDIY::einkPower(int bOn)
{
    if (bOn == _state.pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        if (_state.panelDef.ioPWR & BB_IO_FLAG_TPS65186) {
            
        } else { // single MOSFET
            gpio_set_level((gpio_num_t)(_state.panelDef.ioOE & 0xff), 1);
            delayMicroseconds(100);
            gpio_set_level((gpio_num_t)(_state.panelDef.ioPWR & 0xff), 1);
            delayMicroseconds(100);
            gpio_set_level((gpio_num_t)(_state.panelDef.ioSPV & 0xff), 1);
            gpio_set_level((gpio_num_t)(_state.panelDef.ioSPH & 0xff), 1);
        }
        _state.pwr_on = 1;
    } else {
        gpio_set_level((gpio_num_t)(_state.panelDef.ioPWR & 0xff), 0);
        delayMicroseconds(10);
        gpio_set_level((gpio_num_t)(_state.panelDef.ioOE & 0xff), 0);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)(_state.panelDef.ioSPV & 0xff), 0);
        _state.pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* einkPower() */

//
// Clear the display with the given code for the given number of repetitions
// val: 0 = lighten, 1 = darken, 2 = discharge, 3 = skip
//
void BBEPDIY::clear(uint8_t val, uint8_t count)
{
    if (val == 0) val = 0xaa;
    else if (val == 1) val = 0x55;
    else if (val == 2) val = 0x00;
    else val = 0xff;
    memset(_state.dma_buf, val, _state.width / 4);

    for (int k = 0; k < count; ++k)
    {
        bbepRowControl(&_state, ROW_START);
        for (int i = 0; i < _state.height; ++i)
        {
            // Send the data using I2S DMA driver.
            bbepWriteRow(&_state, _state.dma_buf, _state.width / 4);
            delayMicroseconds(15);
            bbepRowControl(&_state, ROW_STEP);
        }
        delayMicroseconds(230);
    }
} /* clear() */

void BBEPDIY::fillScreen(uint8_t u8Color)
{
    int iPitch;
    if (_state.mode == BB_MODE_1BPP) {
        if (u8Color == BBEP_WHITE) u8Color = 0xff;
    }
    iPitch = (_state.width + 7) / 8;
    memset(_state.pCurrent, u8Color, iPitch * _state.height);
} /* fillScreen() */

int BBEPDIY::fullUpdate(bool bFast, bool bKeepOn)
{
    int passes;

    if (einkPower(1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
// Fast mode ~= 600ms, normal mode ~=1000ms
    passes = (bFast) ? 5:11;
    if (!bFast) { // skip initial black phase for fast mode
        clear(0, 1);
        clear(1, passes);
        clear(2, 1);
    }
    clear(0, passes);
    clear(2, 1);
    clear(1, passes);
    clear(2, 1);
    clear(0, passes);

    // Set the color in multiple passes starting from white
    // First create the 2-bit codes per pixel for the black pixels
    uint8_t *s, *d;
    for (int i = 0; i < _state.height; ++i)
    {
        s = &_state.pCurrent[i * (_state.width/8)];
        d = &_state.pTemp[i * (_state.width/4)];
        memcpy(&_state.pPrevious[i * (_state.width/8)], s, _state.width / 8); // previous = current
        for (int n = 0; n < (_state.width / 4); n += 4)
        {
            uint8_t dram1 = *s++;
            uint8_t dram2 = *s++;
            *(uint16_t *)&d[n+2] = LUTB_16[dram2];
            *(uint16_t *)&d[n] = LUTB_16[dram1];
        }
    }
    // Write 5 passes of the black data to the whole display
    for (int k = 0; k < 5; ++k)
    {
        rowControl(ROW_START);
        for (int i = 0; i < _state.height; ++i)
        {
            s = &_state.pTemp[i * (_state.width / 4)];
            // Send the data for the row
            memcpy(_state.dma_buf, s, _state.width/ 4);
            writeRow(_state.dma_buf, (_state.width / 4));
            delayMicroseconds(15);
            rowControl(ROW_STEP);
        }
        delayMicroseconds(230);
    }

    for (int k = 0; k < 1; ++k)
    {
        uint8_t *s, *d;
        rowControl(ROW_START);
        for (int i = 0; i < _state.height; ++i)
        {
            s = &_state.pCurrent[(_state.width/8) * i];
            d = &_state.pTemp[i * (_state.width/4)];
            for (int n = 0; n < (_state.width / 4); n += 4)
            {
                uint8_t dram1 = *s++;
                uint8_t dram2 = *s++;
                *(uint16_t *)&d[n+2] = LUT2_16[dram2];
                *(uint16_t *)&d[n] = LUT2_16[dram1];
            }
            // Send the data for the row
            memcpy(_state.dma_buf, d, _state.width/ 4);
            writeRow(d, (_state.width / 4));
            delayMicroseconds(15);
            rowControl(ROW_STEP);
        }
        delayMicroseconds(230);
    }

    for (int k = 0; k < 1; ++k)
    {
        rowControl(ROW_START);
        memset((void *)_state.dma_buf, 0, _state.width /4); // send 0's
        for (int i = 0; i < _state.height; ++i)
        {
            // Send the data for the row
            writeRow(_state.dma_buf, (_state.width / 4));
            delayMicroseconds(15);
            rowControl(ROW_STEP);
        }
        delayMicroseconds(230);
    }


    if (!bKeepOn) einkPower(0);
    return BBEP_SUCCESS;
} /* fullUpdate() */

int BBEPDIY::partialUpdate(bool bKeepOn, int iStartLine, int iEndLine)
{
    long l = millis();
    if (einkPower(1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    if (iStartLine < 0) iStartLine = 0;
    if (iEndLine >= _state.height) iEndLine = _state.height-1;
    if (iEndLine < iStartLine) return BBEP_ERROR_BAD_PARAMETER;

    uint32_t _send;
    uint8_t whiteMask, *pCur, *pPrev, *d, data = 0;
    uint8_t diffw, diffb;
    
    for (int i = iStartLine; i <= iEndLine; ++i) {
        d = &_state.pTemp[i * (_state.width/4)]; // LUT temp storage
        pCur = &_state.pCurrent[i * (_state.width / 8)];
        pPrev = &_state.pPrevious[i * (_state.width / 8)];
        for (int j = 0; j < _state.width / 16; ++j)
        {
            diffw = *pPrev & ~*pCur;
            diffb = ~*pPrev & *pCur;
            pPrev++; pCur++;
            *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

            diffw = *pPrev & ~*pCur;
            diffb = ~*pPrev & *pCur;
            pPrev++; pCur++;
            *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
            d += 4;
        }
    }
    for (int k = 0; k < 4; ++k) { // each pass is about 32ms
        uint8_t *dp = _state.pTemp;
        int iSkipped = 0;
        rowControl(ROW_START);
        for (int i = 0; i < _state.height; ++i) {
            if (i >= iStartLine && i <= iEndLine) {
                memcpy((void *)_state.dma_buf, dp, _state.width/4);
                // Send the data using I2S DMA driver.
                writeRow(_state.dma_buf, (_state.width / 4));
                delayMicroseconds(15);
                iSkipped = 0;
            } else {
                if (iSkipped >= 2) {
                    gpio_set_level((gpio_num_t)_state.panelDef.ioCKV, 1); // CKV_SET;
                    delayMicroseconds(15);
                } else {
                    // write 2 floating rows
                    if (iSkipped == 0) { // clean
                       memset((void *)_state.dma_buf, 0, _state.width/4);
                    }
                    writeRow(_state.dma_buf, (_state.width / 4));
                    delayMicroseconds(15);
                }
                iSkipped++;
            }
            rowControl(ROW_STEP);
            dp += (_state.width / 4);
        }
        //delayMicroseconds(230);
    }

    if (bKeepOn) {
        clear(2, 1);
    } else {
        einkPower(0);
    }
    int offset = iStartLine * (_state.width/8);
    memcpy(&_state.pPrevious[offset], &_state.pCurrent[offset], (_state.width/8) * (iEndLine - iStartLine+1));

    l = millis() - l;
    Serial.printf("partial update time: %dms\n", (int)l);
    return BBEP_SUCCESS;
} /* partialUpdate() */