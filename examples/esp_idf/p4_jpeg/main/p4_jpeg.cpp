#include <stdio.h>
#include <JPEGDEC.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../../../../src/FastEPD.h"
#include "it_cartoon.h"
FASTEPD epaper;
uint8_t *pBuffer;
JPEGDEC jpg;

int JPEGDraw(JPEGDRAW *pDraw)
{
  int x, y, iPitch = epaper.width()/2;
  uint8_t *s, *d;
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

extern "C" {
void app_main(void);
}

void app_main(void)
{
int rc;

    rc = epaper.initPanel(BB_PANEL_LILYGO_T5P4);
    if (rc == BBEP_SUCCESS) {
        printf("Panel init success!\n");
        pBuffer = epaper.currentBuffer();
        epaper.setMode(BB_MODE_4BPP);
        epaper.fillScreen(0xf);
        if (jpg.openRAM((uint8_t *)it_cartoon, sizeof(it_cartoon), JPEGDraw)) {
            jpg.setPixelType(EIGHT_BIT_GRAYSCALE);
            jpg.decode(0, 0, 0);
            epaper.fullUpdate();
        }
    }
    while (1) {
        vTaskDelay(1);
    }
} /* app_main() */
