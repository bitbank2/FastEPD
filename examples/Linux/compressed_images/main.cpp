//
// Example for displaying Group5 compressed images
//
#include <FastEPD.h>
#include "smiley.h"
FASTEPD epaper;

int main(int argc, char *argv[])
{
  int i, j;
  float f;

// This configuration for this PCB contians info about the Eink connections and display type
    epaper.initPanel(BB_PANEL_RPI);
    epaper.setPanelSize(BBEP_DISPLAY_EC060KD1);
    epaper.setPasses(4,4);
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
