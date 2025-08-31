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
FASTEPDSTATE epaper;
uint8_t *pBuffer;
JPEGIMAGE jpeg;

int JPEGDraw(JPEGDRAW *pDraw)
{
  int x, y, iPitch = epaper.width/2;
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

void app_main(void)
{
int rc;

    rc = bbepInitPanel(&epaper, BB_PANEL_LILYGO_T5PRO, 20000000);
    if (rc == BBEP_SUCCESS) {
        printf("Panel init success!\n");
        //bbepSetPanelSize(&epaper, 1024, 758, BB_PANEL_FLAG_NONE);
        pBuffer = epaper.pCurrent;
        epaper.mode = BB_MODE_4BPP;
        bbepFillScreen(&epaper, 0xf);
        if (JPEG_openRAM(&jpeg, (uint8_t *)it_cartoon, sizeof(it_cartoon), JPEGDraw)) {
            jpeg.ucPixelType = EIGHT_BIT_GRAYSCALE;
            JPEG_decode(&jpeg, 0, 0, 0);
            bbepFullUpdate(&epaper, CLEAR_SLOW, 0, NULL);
        }
    }
    while (1) {
        vTaskDelay(1);
    }
} /* app_main() */
