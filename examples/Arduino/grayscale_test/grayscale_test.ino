#include <FastEPD.h>
FASTEPD epaper;

const uint8_t u8_103Grays[] = {
/* 0 */	  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,
/* 1 */		0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	2,	1,	1,	1,	0,	0,	0,	0,
/* 2 */		0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	1,	1,	2,	1,	0,	0,	0,	0,
/* 3 */		0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	0,	0,	0,	0,	0,
/* 4 */	  0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	0,	0,	0,	0,	0,	0,
/* 5 */		0,	0,	0,	0,	0,	1,	0,	0,	1,	2,	0,	1,	0,	0,	0,	0,	0,	0,
/* 6 */		0,	0,	0,	0,	0,	1,  2,	0,	1,	2,	0,	1,	0,	0,	0,	0,	0,	0,
/* 7 */ 	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	0,	0,	0,
/* 8 */ 	0,	0,	0,	0,	0,	0,	0,	1,	2,	2,	1,	2,	1,	0,	0,	0,	0,	0,
/* 9 */ 	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	2,	0,	0,	0,
/* 10 */	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	2,	0,	0,	0,
/* 11 */	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	2,	1,	2,	0,	0,
/* 12 */	0,	0,	0,	0,	0,	1,	1,	1,	2,	1,	2,	0,	0,	0,	0,	0,	0,	0,
/* 13 */	0,	0,	0,	0,	0,	1,	1,	2,	2,	2,	2,	1,	2,	0,	0,	0,	0,	0,
/* 14 */	1,	1,	1,	1,	1,	1,	2,	2,	1,	2,	2,	0,	0,	0,	0,	0,	0,	0,
/* 15 */	0,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2
};

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting...");
}

void loop() {
  int rc;
    epaper.initPanel(BB_PANEL_EPDIY_V7_16);
    // Set a custom gray matrix for this panel. This determines how the shades of gray
    // look in 4-bpp mode
    rc = epaper.setCustomMatrix(u8_103Grays, sizeof(u8_103Grays));
    if (rc != BBEP_SUCCESS) {
      Serial.printf("setCustomMatrix returned %d\n", rc);
    }
    rc = epaper.setPanelSize(1872, 1404, BB_PANEL_FLAG_MIRROR_X);
    if (rc != BBEP_SUCCESS) {
      Serial.printf("setPanelSize returned %d\n", rc);
    }
//  epaper.initPanel(BB_PANEL_T5EPAPERV1);
//  epaper.initPanel(BB_PANEL_M5PAPERS3);
//  epaper.initPanel(BB_PANEL_INKPLATE6PLUS);
//  epaper.initPanel(BB_PANEL_INKPLATE5V2);
//  epaper.initPanel(BB_PANEL_V7_RAW);
//  epaper.setPanelSize(1024, 758);
//  epaper.setPanelSize(1280, 720, BB_PANEL_FLAG_MIRROR_Y);
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
  while (1) {};
}
