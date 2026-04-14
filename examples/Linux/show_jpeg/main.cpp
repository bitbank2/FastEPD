//
// Example to show a JPEG image
// as 16-gray levels
//
#include <unistd.h>
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

int main(int argc, char *argv[]) {
// If a filename is passed on the command line, open it, otherwise use the
// pre-loaded cartoon image
uint8_t *pData;
int iDataSize, iCenterX, iCenterY;

  epaper.initPanel(BB_PANEL_RPI);
  epaper.setPanelSize(BBEP_DISPLAY_EC060TC1);
  epaper.setMode(BB_MODE_4BPP);
  epaper.fillScreen(0xf);
  if (argc == 2) { // load from file
      FILE *ihandle;
      ihandle = fopen(argv[1], "r+b");
      if (!ihandle) {
          printf("Error opening file: %s\n", argv[1]);
          return -1;
      }
      fseek(ihandle, 0, SEEK_END);
      iDataSize = (int)ftell(ihandle);
      fseek(ihandle, 0, SEEK_SET);
      pData = (uint8_t *)malloc(iDataSize);
      fread(pData, 1, iDataSize, ihandle);
      fclose(ihandle);
  } else { // use internal image
      pData = (uint8_t *)it_cartoon;
      iDataSize = sizeof(it_cartoon);
  }
  if (jpg.openFLASH(pData, iDataSize, JPEGDraw)) {
      if (jpg.getWidth() > epaper.width() || jpg.getHeight() > epaper.height()) {
          printf("Image larger than display; exiting...\n");
          return -1;
      }
      iCenterX = (epaper.width() - jpg.getWidth()) / 2;
      iCenterY = (epaper.height() - jpg.getHeight()) / 2;
      jpg.setPixelType(EIGHT_BIT_GRAYSCALE);
      jpg.decode(iCenterX, iCenterY, 0);
      jpg.close();
      epaper.fullUpdate(CLEAR_FAST);
  }
  usleep(3000000);
  epaper.clearWhite();
  epaper.einkPower(0);
  epaper.deInit();
  return 0;
} /* main() */ 

