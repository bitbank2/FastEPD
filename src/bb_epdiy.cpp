#include "bb_epdiy.h"
#include "bb_epdiy.inl"
#include "bb_ep_gfx.inl"
#include <Wire.h>

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
            _pCurrent = (uint8_t *)ps_malloc(_state.width * _state.height / 8); // current pixels
            _pPrevious = (uint8_t *)ps_malloc(_state.width * _state.height / 8); // comparison with previous buffer
        }
        _state.pfnSetPixel = bbepSetPixel2Clr;
        _state.pfnSetPixelFast = bbepSetPixelFast2Clr;
        return rc;
    }
    return BBEP_INVALID_PARAM;
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
    } else {
        gpio_set_level((gpio_num_t)(_state.panelDef.ioPWR & 0xff), 0);
        delayMicroseconds(10);
        gpio_set_level((gpio_num_t)(_state.panelDef.ioOE & 0xff), 0);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)(_state.panelDef.ioSPV & 0xff), 0);
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
            bbepRowControl(&_state, ROW_STEP);
        }
        delayMicroseconds(230);
    }
} /* clear() */

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

    if (!bKeepOn) einkPower(0);
    return BBEP_SUCCESS;
}