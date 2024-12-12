#include "bb_epdiy.h"
#include <Wire.h>

const BB_DISPLAY_T panelList[] = {
    {}, // invalid
    {960, 540, 8, 16},  // BB_ED047TC1 960x540, 8bits, 16MHz
    {1280, 720, 8, 16}, //  BB_ED052TC4 1280x720 5.2"
    {800, 600, 8, 20}, // BB_ED060SCT 800x600 6"
    {1024, 768, 8, 20}, // BB_ED060XC3 1024x768 6"
    {1200, 825, 8, 16}, // BB_ED097OC4 1200x825 9.7"
    {1200, 825, 8, 22}, // BB_ED097TC2 1200x825 9.7"
    {1600, 1200, 8, 20}, // BB_ED133UT2 1600x1200 13.3"
    {1872, 1404, 16, 11}, // BB_ED078KC1 1872x1404 7.8" 16-bit
};
int BBEPDIY::setPanelType(int iPanel)
{
    if (iPanel > 0 && iPanel < BB_PANEL_COUNT) {
        _state.iPanelType = iPanel;
        memcpy(&_state.display, &panelList[iPanel], sizeof(BB_DISPLAY_T));
        return BBEP_SUCCESS;
    }
    return BBEP_INVALID_PARAM;
} /* setPanelType() */
int BBEPDIY::initIO(BB_IO_CONFIG_T *pIO)
{
    if (pIO == NULL) return BBEP_INVALID_PARAM;
    memcpy(&_state.io_config, pIO, sizeof(BB_IO_CONFIG_T));
    Wire.begin(pIO->u8SDA, pIO->u8SCL);
    Wire.setClock(400000);
    return bbepIOInit(&_state);
} /* initIO() */

int BBEPDIY::powerOn(void)
{
    return bbepPowerOn(&_state);
} /* powerOn() */
void BBEPDIY::powerOff(void)
{
    bbepPowerOff(&_state);
} /* powerOff() */
