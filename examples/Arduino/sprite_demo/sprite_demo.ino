//
// FastEPD Sprite Example
// Written by Larry Bank August 23, 2025
// Copyright (c) 2025 BitBank Software, Inc. All rights reserved
//
// Sprites in FastEPD refer to virtual drawing surfaces.
// in other words, instances of the FastEPD class without
// a physical Eink panel. Their purpose is to allow easy
// re-use of prepared graphics to composite a scene or to
// make use of the FastEPD font and graphics functions for
// an external program to utilize.
//
// 1-bpp sprites can be drawn on 4-bpp surfaces, but not the reverse
//
#include <FastEPD.h>
#include "Roboto_Regular_20.h"
#include "courierprime_14.h"
FASTEPD epaper;
FASTEPD sprite1, sprite2;

void setup()
{
  sprite1.initSprite(128, 64); // allocate the memory for 2 small drawing surfaces
  sprite2.initSprite(128, 64);
  epaper.initPanel(BB_PANEL_V7_RAW); // defaults to 1-bit mode
  epaper.setPanelSize(1280, 720);
  epaper.fillScreen(BBEP_WHITE);
  epaper.fullUpdate(); // clear the screen to white to begin
  sprite1.fillScreen(BBEP_WHITE);
  sprite1.setFont(courierprime_14);
  sprite1.setCursor(0, 24);
  sprite1.print("Sprite1");
  sprite2.fillScreen(BBEP_WHITE);
  sprite2.setFont(Roboto_Regular_20);
  sprite2.setCursor(0, 32);
  sprite2.print("Sprite2");
  for (int i=0; i<epaper.width(); i += 160) {
    epaper.drawSprite(&sprite1, i, 0);
    epaper.drawSprite(&sprite2, i, 256);
  }
  epaper.fullUpdate();
} /* setup() */

void loop()
{
} /* loop() */

