#include <stdio.h>
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "../../../../src/FastEPD.h"
FASTEPDSTATE bbep;
#include "Roboto_Black_100.h"


 
void app_main(void)
{
int rc;
    rc = bbepInitPanel(&bbep, BB_PANEL_EPDIY_V7);
    if (rc == BBEP_SUCCESS) {
      bbepSetPanelSize(&bbep, 1200, 825);
      bbepFillScreen(&bbep, BBEP_WHITE);
      bbepFullUpdate(&bbep, 0, 1, NULL);
      bbepWriteStringCustom(&bbep, (BB_FONT *)Roboto_Black_100, 0, 200, "Hello", BBEP_BLACK);
      bbepPartialUpdate(&bbep, 0, 0, 1000);
    }
}
