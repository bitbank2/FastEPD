//
// Anti-aliased font example
//
#include <cstdint>

#include "Lora_16.h"
#include "Lora_8.h"
#include "itc40.h"
#include "itc80.h"
#include "nico8.h"
#include "nico16.h"

#include "logo_small.h"
#include <FastEPD.h>
#include <cstdio>
#include <headless.h>

// clang antialias_font.cpp ../../../src/FastEPD.cpp ../../../src/Group5.cpp
// ../../../src/headless.cpp -I ../../../src -D HEADLESS -o antialias_font

FASTEPD epaper;
int width = 800;
int height = 480;

void dump(const char *filename) {
  uint8_t *buffer = epaper.currentBuffer();
  FILE *output = fopen(filename, "wb");
  char buff[64];
  snprintf(buff, sizeof(buff), "P5\n%d %d\n255\n", width, height);
  fwrite(buff, 1, strlen(buff), output);
  int stride = width / 2;
  for (int y = 0; y < height; ++y) {
    uint8_t *row = buffer + y * stride;
    for (int x = 0; x < width / 2; ++x) {
      uint8_t col1 = (row[x] >> 4) | row[x];
      uint8_t col2 = (row[x] & 0x0f) << 4 | row[x];
      fwrite(&col1, 1, 1, output);
      fwrite(&col2, 1, 1, output);
    }
  }
  fclose(output);
}

void setup() {
  printf("init\n");

  BBPANELDEF headlessdef = {width, height};
  BBPANELPROCS headlessprocs = {HeadlessEinkPower, HeadlessIOInit,
                                HeadlessRowControl};

  epaper.initCustomPanel(&headlessdef, &headlessprocs);
  printf("done init\n");

  epaper.setPanelSize(width, height);
  epaper.setMode(BB_MODE_4BPP);
  epaper.setFont(itc40);
  epaper.setTextColor(0);

  int steps = 16;
  int stepw = 15;
  int steph = 10;
  int posw = 0;
  int posh = 0;
  int rw = width;
  int rh = height;
  for (int i = 0; i < steps; ++i) {
    epaper.fillRect(posw, posh, rw, rh, i);
    posw += stepw;
    posh += steph;
    rw -= 2 * stepw;
    rh -= 2 * steph;
  }
  int logodim = 86;
  int xoffset = 20;
  int yoffset = 50;

    // small text
  const char *str2 = "Portez ce vieux whisky au juge blond qui fume";

    epaper.setFont(nico8, false);
  // epaper.getStringBox(str2, &bbox);
  // epaper.drawString(str2, (width - bbox.w) / 2, ((height - bbox.h) / 2));
epaper.drawString(str2, 100, 170);
  epaper.setFont(nico16, true);
  // epaper.getStringBox(str2, &bbox);
  // epaper.drawString(str2, (width - bbox.w) / 2, ((height - bbox.h) / 2));
epaper.drawString(str2, 200, 370);

  const char *str = "okay";
  epaper.setFont(itc40, false);
  BBEPRECT bbox;
  epaper.getStringBox(str, &bbox);
  int posx = 100;
  int posy = 100;
  epaper.drawString(str, posx, posy);

  posx += bbox.w + xoffset * 2;
  posy += bbox.h + yoffset * 2;
  epaper.setFont(itc80, true);
  epaper.drawString(str, posx * 2, posy * 2);

  posx += bbox.w + xoffset * 2;
  posy += bbox.h + yoffset * 2;
  epaper.drawString(str, posx * 2, posy * 2);


  epaper.setFont(Lora_8, false);
epaper.drawString(str2, 100, 400);
  epaper.setFont(nico16, true);
  epaper.drawString(str2, 200, 900);

  epaper.loadG5Image(logo_small, (width - logodim) / 2, (height + logodim) / 2,
                     BBEP_TRANSPARENT, 0);

  // epaser.fullUpdate(true, false);
  dump("rectangles.pgm");
  printf("done\n");
}

void loop() {}