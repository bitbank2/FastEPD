#include <stdio.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../../../../src/FastEPD.h"
#include "../../../../Fonts/Roboto_Black_80.h"
#include "../../../../Fonts/Roboto_Black_40.h"

FASTEPDSTATE bbep;
 
void app_main(void)
{
int rc;
    rc = bbepInitPanel(&bbep, BB_PANEL_LILYGO_T5PRO, 20000000);
    if (rc == BBEP_SUCCESS) {
      //bbepSetPanelSize(&bbep, 1280, 720, BB_PANEL_FLAG_NONE);
      bbepFillScreen(&bbep, BBEP_WHITE);
      bbepFullUpdate(&bbep, CLEAR_SLOW, 0, NULL);
      bbepWriteStringCustom(&bbep, Roboto_Black_80, 0, 200, "Hello World!", BBEP_BLACK);
      bbepPartialUpdate(&bbep, 0, 0, 1000);
      bbepWriteStringCustom(&bbep, Roboto_Black_40, 0, 400, "Aló mundo de FastEPD!", BBEP_BLACK);
      bbepPartialUpdate(&bbep, 0, 0, 1000);
    }
}
