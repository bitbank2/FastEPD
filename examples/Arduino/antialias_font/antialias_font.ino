//
// Anti-aliased font example
//
#include <FastEPD.h>
#include "../../../Fonts/Roboto_Black_40.h"
#include "../../../Fonts/Roboto_Black_80.h"
FASTEPD epaper;

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("Starting...");
  epaper.initPanel(BB_PANEL_EPDIY_V7);
  epaper.setPanelSize(960, 540, BB_PANEL_FLAG_NONE, -1500);
  epaper.setMode(BB_MODE_4BPP);
  epaper.fillScreen(0xf); // white background
  epaper.fullUpdate();
  epaper.setFont(Roboto_Black_40);
  epaper.setCursor(0,70);
  epaper.setTextColor(0, 0xf); // black on white
  epaper.println("Roboto Black 40pt");
  epaper.setFont(Roboto_Black_80, true);
  epaper.setCursor(0, 150);
  epaper.println("Anti-aliased (80pt)");
  epaper.fillRect(0, epaper.height()/2, epaper.width(), epaper.height()/2, 0); // black rect
  epaper.setTextColor(0xf, 0); // white on black
  epaper.setCursor(0, 380);
  epaper.println("Inverted anti-aliased");
  epaper.fullUpdate(CLEAR_FAST, false);
}

void loop()
{

}