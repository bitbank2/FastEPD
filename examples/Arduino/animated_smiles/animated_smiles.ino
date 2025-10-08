//
// Example for animating images
//
#include <FastEPD.h>
#include "smiley.h"
FASTEPD epaper;
// holds the position and direction of the objects
#define NUM_SMILEYS 8
int xArray[NUM_SMILEYS];
int yArray[NUM_SMILEYS];
int xDir[NUM_SMILEYS];
int yDir[NUM_SMILEYS];

void setup()
{
  int i, j;
  float f;

  epaper.initPanel(BB_PANEL_V7_RAW, 26666666);
  epaper.setPanelSize(BBEP_DISPLAY_ED052TC4);
  epaper.clearWhite(); // start with a white display (and buffer)
  epaper.setPasses(3); // fewer passes = faster updates
  // Create 8 random starting points
  for (i=0; i<NUM_SMILEYS; i++) {
    xArray[i] = random(4, epaper.width() - 104);
    yArray[i] = random(4, epaper.height() - 104);
    xDir[i] = random(0, 2) ? -4 : 4;
    yDir[i] = random(0, 2) ? -4 : 4;
  }
} /* setup() */

void loop()
{
int i, j;

  for (j=0; j<250; j++) { // 250 iterations
 //   epaper.fillScreen(BBEP_WHITE);
    for (i=0; i<NUM_SMILEYS; i++) { // draw current posiitions
        epaper.loadG5Image(smiley, xArray[i], yArray[i], BBEP_WHITE, BBEP_BLACK, 1.0f);
        // update the positions
        xArray[i] += xDir[i];
        if (xArray[i] < 4 || xArray[i] > epaper.width() - 104) xDir[i] = -xDir[i]; // bounce off X
        yArray[i] += yDir[i];
        if (yArray[i] < 4 || yArray[i] > epaper.height() - 104) yDir[i] = -yDir[i]; // bounce off Y
    } // for u;
    // Show on the display
    epaper.partialUpdate(true);
  } // for j
  delay(3000);
  epaper.clearWhite(false);
  epaper.deInit(); // save power by shutting down the TI power controller and I/O extender
  while (1) {
    delay(1);
  }
} /* loop() */
