//
// Anti-aliased font example
//
#include <FastEPD.h>
#include "Roboto_Black_40.h"
#include "Roboto_Black_80.h"
FASTEPD epaper;

void setup()
{
  epaper.initPanel(BB_PANEL_EPDIY_V7);
  epaper.setPanelSize(1024, 758);
  epaper.setMode(BB_MODE_4BPP);
  epaper.clearWhite();
  epaper.setFont(Roboto_Black_40);
  epaper.setCursor(0,60);
  epaper.print("Roboto Black 40pt");
  epaper.setFont(Roboto_Black_80, true);
  epaper.setCursor(0, 240); // really 0,120
  epaper.print("Roboto Black 80pt aa");
  epaper.fullUpdate(true, false);
}

void loop()
{

}