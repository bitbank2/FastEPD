#include <FastEPD.h>
FASTEPD epaper;
void setup() {
//  epaper.initPanel(BB_PANEL_T5EPAPERV1);
//  epaper.initPanel(BB_PANEL_M5PAPERS3);
//  epaper.initPanel(BB_PANEL_INKPLATE6PLUS);
//  epaper.initPanel(BB_PANEL_INKPLATE5V2);
  epaper.initPanel(BB_PANEL_EPDIY_V7);
  epaper.setPanelSize(1024, 758);
//  epaper.setPanelSize(1280, 720);
  epaper.setMode(BB_MODE_4BPP);
  epaper.fillScreen(0xf);
  for (int i=0; i<800; i+=50) {
    epaper.fillRect(i, 0, 50, 250, i/50);
  }
  epaper.drawRect(0, 0, 800, 250, 0); // draw black outline around
  epaper.setFont(FONT_12x16);
  epaper.setTextColor(BBEP_BLACK);
  for (int i=0; i<16; i++) {
    epaper.setCursor(i*50+12, 252);
    epaper.print(i, DEC);
  }
  epaper.fullUpdate();
}

void loop() {
  // put your main code here, to run repeatedly:

}
