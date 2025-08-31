//
// Anti-aliased font demo
//
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

FASTEPDSTATE bbep, virt;
 
void app_main(void)
{
int rc;

// We have enough PSRAM to allocate full sized buffers
// for a full screen virtual device and the physical
// device (M5Stack PaperS3 in this case - 960x540)
// We'll draw on the virtual memory device, then copy
// it to the framebuffer of the physical device and
// do a 16-gray update

    bbepInitPanel(&virt, BB_PANEL_VIRTUAL, 0);
    // A virtual panel must be given a size
    bbepSetPanelSize(&virt, 960, 540, 0);

    // Predefined devices can have a display size
    // associated with them already
    rc = bbepInitPanel(&bbep, BB_PANEL_M5PAPERS3, 20000000);
    if (rc == BBEP_SUCCESS) {
      bbepSetMode(&virt, BB_MODE_4BPP); // source and destination
      bbepSetMode(&bbep, BB_MODE_4BPP); // need to be the same bit depth
      bbepFillScreen(&virt, 0xf); // fill with white

      // The y coordinate represents the character baseline
      // not the upper left corner
      bbepWriteStringCustom(&virt, Roboto_Black_40, 0, 200, "Normal, 1-bit font", BBEP_BLACK);
      virt.anti_alias = 1; // turn on antialiased mode
      // Use a font with a point size 2x bigger to get the same
      // size output in anti-alias mode. (80 vs 40pt)
      bbepWriteStringCustom(&virt, Roboto_Black_80, 0, 400, "Antialiased, 2-bit font", BBEP_BLACK);

      // Draw the virtual device framebuffer onto the 
      // physical device framebuffer at (0,0)
      bbepDrawSprite(&virt, &bbep, 0, 0, -1);

      // Draw the physical device framebuffer onto the Eink panel
      bbepFullUpdate(&bbep, CLEAR_SLOW, 0, NULL);
    }
}
