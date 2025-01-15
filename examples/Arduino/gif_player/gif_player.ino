#include <FastEPD.h>
#include <AnimatedGIF.h>
#include "Roboto_Black_50.h"
#include "1bitsmallcity.h"
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
    if (pDraw->y == pDraw->iHeight-1) // last line, render it to the display
    // Tell FastEPD to keep the power on and only update the lines which changed (start_y, end_y)
       epaper.partialUpdate(true, center_y, center_y + gif.getCanvasHeight());
} /* GIFDraw() */

void setup()
{
  BBEPRECT rect; // rectangle for getting the text size
//  Serial.begin(115200);
//  Serial.println("Starting...");
  epaper.initPanel(BB_PANEL_EPDIY_V7); // defaults to 1-bpp mode
  epaper.setPanelSize(1024, 758); // only set panel size if it's not part of the panel definition
  gif.begin(LITTLE_ENDIAN_PIXELS);
  pFramebuffer = epaper.currentBuffer(); // we want to write directly into the framebuffer (faster)
  epaper.fillScreen(BBEP_WHITE);
  epaper.setFont(Roboto_Black_50); // A compressed BB_FONT
  epaper.setTextColor(BBEP_BLACK);
  epaper.getStringBox("FastEPD GIF Demo", &rect); // get the rectangle around the text
  epaper.setCursor((epaper.width() - rect.w)/2, 90); // center horizontally
  epaper.print("FastEPD GIF Demo");
  epaper.fullUpdate(true, true); // start with a full update and leave the power ON
} /* setup () */

void loop()
{
  if (gif.open((uint8_t *)_1bitsmallcity, sizeof(_1bitsmallcity), GIFDraw)) {
    center_x = (epaper.width() - gif.getCanvasWidth())/2;
    center_y = (epaper.height() - gif.getCanvasHeight())/2;
//    Serial.printf("Successfully opened GIF; Canvas size = %d x %d\n", gif.getCanvasWidth(), gif.getCanvasHeight());
    while (gif.playFrame(false, NULL)) { } // play it as fast as possible
    gif.close();
  }
  delay(3000); // wait a few seconds before erasing the display
  epaper.fillScreen(BBEP_WHITE);
  epaper.fullUpdate(); // Do a full refresh, then turn off the eink power
  while (1) {}; // we're done, sit here forever
} /* loop() */
