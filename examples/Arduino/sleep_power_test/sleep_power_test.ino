//
// FastEPD sleep power test
//
// A sketch written to help measure the power used by parallel Eink
// boards utilizing some form of ESP32
// The program goes through various steps of active use and different
// sleep levels to allow measurement of the current in every situation
//
#include <FastEPD.h>
FASTEPD epaper;

void lightSleep(uint64_t time_in_us)
{
  esp_sleep_enable_timer_wakeup(time_in_us);
  esp_light_sleep_start();
}
void deepSleep(uint64_t time_in_us)
{
  esp_sleep_enable_timer_wakeup(time_in_us);
  esp_deep_sleep_start();
}

void setup()
{
  epaper.initPanel(BB_PANEL_SENSORIA_C5);
  epaper.fillScreen(BBEP_WHITE);
  epaper.fullUpdate();
  epaper.deInit(); // shut down I/O extenders
  delay(3000); // 3 seconds of the ESP32 fully running
  lightSleep(3000000); // 3 seconds of the light sleep
  deepSleep(3000000); // 3 seconds of deep sleep
}

void loop()
{
}

