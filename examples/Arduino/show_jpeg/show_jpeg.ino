//
// Example to show a JPEG image
// as 16-gray levels
//
#include <FastEPD.h>
#include <JPEGDEC.h>
#include "it_cartoon.h"
JPEGDEC jpg;
FASTEPD epaper;

int JPEGDraw(JPEGDRAW *pDraw)
{
  int x, y, iPitch = epaper.width()/2;
  uint8_t *s, *d, *pBuffer = epaper.currentBuffer();
  for (y=0; y<pDraw->iHeight; y++) {
    d = &pBuffer[((pDraw->y + y)*iPitch) + (pDraw->x/2)];
    s = (uint8_t *)pDraw->pPixels;
    s += (y * pDraw->iWidth);
    for (x=0; x<pDraw->iWidth; x+=2) {
        *d++ = (s[0] & 0xf0) | (s[1] >> 4);
        s += 2;
    } // for x
  } // for y
  return 1;
} /* JPEGDraw() */

void setup() {
  Serial.begin(115200);
  delay(3000); // wait for CDC-Serial to start
// For EPDiy v7 PCB, you need to specify the panel size explicitly
//    epaper.initPanel(BB_PANEL_EPDIY_V7_16);
//    epaper.setPanelSize(2760, 2070, 0);
//  epaper.initPanel(BB_PANEL_EPDIY_V7);
//  epaper.setPanelSize(1024, 758);
//  epaper.setPanelSize(1280, 720, BB_PANEL_FLAG_MIRROR_X);
  epaper.initPanel(BB_PANEL_M5PAPERS3);
//  epaper.initPanel(BB_PANEL_INKPLATE5V2);
  epaper.setMode(BB_MODE_4BPP);
  epaper.fillScreen(0xf);
  if (jpg.openFLASH((uint8_t *)it_cartoon, sizeof(it_cartoon), JPEGDraw)) {
      jpg.setPixelType(EIGHT_BIT_GRAYSCALE);
      jpg.decode(0, 0, 0);
      jpg.close();
      epaper.fullUpdate();
  }
} /* setup() */ 

void loop() {
  delay(50000);
 // esp_sleep_enable_timer_wakeup(5000000);
 // esp_deep_sleep_start();

} /* loop() */
