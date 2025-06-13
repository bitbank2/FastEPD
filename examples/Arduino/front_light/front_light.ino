//
// Front light control example
//
#include <FastEPD.h>
FASTEPD epaper;

void setup()
{
    epaper.initPanel(BB_PANEL_EPDIY_V7);
    epaper.setPanelSize(BBEP_DISPLAY_EC060TC1);
    epaper.initLights(11, 12); // cold on 11, warm on 12
}

void loop()
{
uint8_t i;

// switch between warm and cold light perpetually
  for (i=0; i<256; i++) {
    epaper.setBrightness(i, 255-i);
    delay(100);
  }
  for (i=255; i>=0; i--) {
    epaper.setBrightness(i, 255-i);
    delay(100);
  }
}