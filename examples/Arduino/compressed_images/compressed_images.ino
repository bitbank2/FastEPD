//
//
//
#include <FastEPD.h>
#include "smiley.h"
FASTEPD epaper;

void setup()
{
  int i, j;
  float f;
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");
  epaper.initPanel(BB_PANEL_LILYGO_T5V24);
 // epaper.initLights(40, -1);
//  for (i=0; i<4; i++) {
//      epaper.setBrightness(128);
//      delay(250);
//      epaper.setBrightness(0);
//      delay(250);
//  }

//  epaper.setPanelSize(1280, 720);
//  epaper.setMode(BB_MODE_4BPP);
  epaper.clearWhite(); // start with a white display (and buffer)
  // The smiley image is 100x100 pixels; draw it at various scales from 0.5 to 2.0
  i = 0;
  f = 0.5f; // start at 1/2 size (50x50)
  for (j = 0; j < 5; j++) {
    epaper.loadG5Image(smiley, i, i, BBEP_WHITE, BBEP_BLACK, f);
    i += (int)(100.0f * f);
    f += 0.5f;
  }
  epaper.partialUpdate(false);
}

void loop()
{

}
