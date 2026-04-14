#include <unistd.h>
#include <FastEPD.h>
#include <AnimatedGIF.h>
#include "Roboto_Black_50.h"
#include "1bitsmallcity.h"
#include "spiral_1bit.h"
AnimatedGIF gif;
FASTEPD epaper;
uint8_t *pFramebuffer;
int center_x, center_y;
// Variable that keeps count on how much screen has been partially updated
int n = 0;

//
// This doesn't have to be super efficient
//
void DrawPixel(int x, int y, uint8_t ucColor)
{
uint8_t ucMask;
int index;

  x += center_x;
  y += center_y;
  ucMask = 0x80 >> (x & 7);
  index = (x>>3) + (y * (epaper.width()/8));
  if (ucColor)
     pFramebuffer[index] |= ucMask; // black
  else
     pFramebuffer[index] &= ~ucMask;
}
//
// Callback function from the AnimatedGIF library
// It's passed each line as it's decoded
// Draw the line of pixels directly into the FastEPD framebuffer
//
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t *s;
    int x, y, iWidth;
    static uint8_t ucPalette[256]; // thresholded palette

    if (pDraw->y == 0) { // first line, convert palette to 0/1
      for (x = 0; x < 256; x++) {
        uint16_t usColor = pDraw->pPalette[x];
        int gray = (usColor & 0xf800) >> 8; // red
        gray += ((usColor & 0x7e0) >> 2); // plus green*2
        gray += ((usColor & 0x1f) << 3); // plus blue
        ucPalette[x] = (gray >> 9); // 0->511 = 0, 512->1023 = 1
      }
    }
    y = pDraw->iY + pDraw->y; // current line position within the GIF canvas
    iWidth = pDraw->iWidth;
    if (iWidth > epaper.width())
       iWidth = epaper.width();
    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) { // restore to background color
      for (x=0; x<iWidth; x++) {
        if (s[x] == pDraw->ucTransparent)
           s[x] = pDraw->ucBackground;
      }
      pDraw->ucHasTransparency = 0;
    }
    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) { // if transparency used
      uint8_t c, ucTransparent = pDraw->ucTransparent;
      int x;
      for(x=0; x < iWidth; x++) {
        c = *s++; // each source pixel is always 1 byte (even for 1-bit images)
        if (c != ucTransparent)
             DrawPixel(pDraw->iX + x, y, ucPalette[c]);
      }
    } else {
      s = pDraw->pPixels;
      // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
      for (x=0; x<pDraw->iWidth; x++)
        DrawPixel(pDraw->iX + x, y, ucPalette[*s++]);
    }
//    if (pDraw->y == pDraw->iHeight-1) // last line, render it to the display
    // Tell FastEPD to keep the power on and only update the lines which changed (start_y, end_y)
//   epaper.partialUpdate(true, center_y, center_y + gif.getCanvasHeight());
} /* GIFDraw() */

int main(int argc, char *argv[])
{
  BB_RECT rect; // rectangle for getting the text size
  int iFrame;
  uint8_t *pGIF;
  int iGIFSize;

printf("gif_player - optionally pass the name of a GIF to play\n");
    epaper.initPanel(BB_PANEL_RPI);
    epaper.setPanelSize(BBEP_DISPLAY_EC060TC1);
  gif.begin(LITTLE_ENDIAN_PIXELS);
  pFramebuffer = epaper.currentBuffer(); // we want to write directly into the framebuffer (faster)
  epaper.fillScreen(BBEP_WHITE);
  epaper.setFont(Roboto_Black_50); // A compressed BB_FONT
  epaper.setTextColor(BBEP_BLACK, BBEP_WHITE);
  epaper.getStringBox("FastEPD GIF Demo", &rect); // get the rectangle around the text
  epaper.setCursor((epaper.width() - rect.w)/2, 90); // center horizontally
  epaper.print("FastEPD GIF Demo");
  epaper.fullUpdate(CLEAR_SLOW, true); // start with a full update and leave the power ON
  //epaper.setPasses(1, 1);
  iFrame = 0;
  if (argc == 2) { // use passed a name
      FILE * ihandle = fopen(argv[1], "r+b");
      if (ihandle) { 
          fseek(ihandle, 0, SEEK_END);
          iGIFSize = (int)ftell(ihandle);
          fseek(ihandle, 0, SEEK_SET);
          pGIF = (uint8_t *)malloc(iGIFSize);
          fread(pGIF, 1, iGIFSize, ihandle);
          fclose(ihandle);
      } else {
          printf("Error opening file %s\n", argv[1]);
          return -1;
      }
  } else {
     pGIF = (uint8_t *)_1bitsmallcity;
     iGIFSize = sizeof(_1bitsmallcity);
  }
  if (gif.open(pGIF, iGIFSize, GIFDraw)) {
    center_x = (epaper.width() - gif.getCanvasWidth())/2;
    center_y = (epaper.height() - gif.getCanvasHeight())/2;
    printf("Successfully opened GIF; Canvas size = %d x %d\n", gif.getCanvasWidth(), gif.getCanvasHeight());
    while (iFrame < 400) {
    while (gif.playFrame(false, NULL)) {
      iFrame++;
      //epaper.videoUpdate();
      epaper.partialUpdate(true);
     } // play it as fast as possible
    gif.reset();
    }
  }
//  l = millis() - l;
//  printf("Played %d frames in %d ms\n", iFrame, (int)l);
  usleep(3000000); // wait a few seconds before erasing the display
  epaper.clearBlack(true);
  epaper.clearWhite(true);
  epaper.einkPower(false);
  epaper.deInit();
  return 0;
} /* main () */

