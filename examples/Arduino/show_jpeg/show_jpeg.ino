//
// Example to show a JPEG image
// as 16-gray levels
//
#include <bb_epdiy.h>
#include <JPEGDEC.h>
#include "it_cartoon.h"
JPEGDEC jpg;
BBEPDIY epaper;

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
  epaper.initPanel(BB_PANEL_M5PAPERS3);
  epaper.setMode(BB_MODE_4BPP);
  if (jpg.openFLASH((uint8_t *)it_cartoon, sizeof(it_cartoon), JPEGDraw)) {
      jpg.setPixelType(EIGHT_BIT_GRAYSCALE);
      jpg.decode(0, 0, 0);
      jpg.close();
      epaper.fullUpdate();
  }
} /* setup() */ 

void loop() {
} /* loop() */