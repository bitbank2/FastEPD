//
// Example for driving the Waveshare IT8951 "HAT" with FastEPD
//
#include <FastEPD.h>
#include "smiley.h"
FASTEPD epaper;

// BCM GPIO numbers for the RPI HAT
#define IT8951_CS 8
#define IT8951_SPI 0
#define IT8951_RST 17
#define IT8951_BUSY 24
#define IT8951_ITE_EN -1
#define IT8951_EN -1

int main(int argc, char *argv[])
{
  int i, j;
  float f;

// This configuration for this PCB contians info about the Eink connections and display type
// initIT8951(uint8_t u8MOSI, uint8_t u8MISO, uint8_t u8CLK, uint8_t u8CS, uint8_t u8Busy, uint8_t u8RST, uint8_t u8EN, uint8_t u8ITE_EN);

    i = epaper.initIT8951(IT8951_SPI, 0, 0, IT8951_CS, IT8951_BUSY, IT8951_RST, IT8951_EN, IT8951_ITE_EN);
    if (i != BBEP_SUCCESS) {
         printf("initIT8951 returned error: %d\n", i);
         return -1;
    }
    i = epaper.setPanelSize(BBEP_DISPLAY_ED078KC2);
    if (i != BBEP_SUCCESS) {
        printf("setPanelSize returned %d\n", i);
        return -1;
    }
    epaper.fillScreen(BBEP_WHITE);
  // The smiley image is 100x100 pixels; draw it at various scales from 0.5 to 2.0
  i = 0;
  f = 0.5f; // start at 1/2 size (50x50)
  for (j = 0; j < 12; j++) {
    epaper.loadG5Image(smiley, i, i, BBEP_WHITE, BBEP_BLACK, f);
    i += (int)(100.0f * f);
    f += 0.5f;
  }
  epaper.fullUpdate(CLEAR_SLOW, false); // the flag (false) tells it to turn off eink power after the update
  epaper.einkPower(0);
  epaper.deInit(); // save power by shutting down the TI power controller and I/O extender
  return 0;
} /* main() */
