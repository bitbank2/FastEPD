//
// FastEPD clock example
//
// written by Larry Bank (bitbank@pobox.com)
// This program displays an attractive time, day and date display on your parallel
// eink project. The time is updated once a minute with a non-flickering update.
// To save power, the program uses the ESP32's deep sleep in between updates.
// This creates a new problem - PSRAM contents are lost during deep sleep, so how
// can we do a non-flickering update if we lost the previous image contents?
// The solution is to save the last epoch time displayed in the ESP32's RTC RAM.
// The 8 or 16K of RTC RAM can be retained in deep sleep for a few microamps of power.
// With this info, we draw the previous time/date into PSRAM (not displayed), then draw
// the new time/date and ask FastEPD to do a partial (differential) update.
//
// This program is free to use as you wish; no warranties are implied or granted
//
// N.B. For maximum battery life, remove the uSD card of your device (if present)
//
#include <NTPClient.h>           //https://github.com/taranais/NTPClient
#include <WiFi.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <HTTPClient.h>
HTTPClient http;
#include <FastEPD.h>
// Use these fonts for a lower resolution display
//#include "Roboto_Black_140.h"
//#include "Roboto_Black_40.h"
#include "Roboto_Black_80.h"
#include "Roboto_Black_200.h"
//
// Change the SSID and PASSWORD to your local WiFi settings (and set the time zone offset)
//
const char *ssid = "your_sside";
const char *password = "your_password";
// Define your timezone offset in seconds from GMT - e.g. GMT-5 = (-5 * 3600)
// This clock does not automatically adjust for daylight savings time, so you will need
// to manually adjust your timezone offset (e.g. US Eastern time defined below)
#define TIME_OFFSET -3600*5

FASTEPD bbep;
// Keep these variables in the ESP32's RTC RAM which is preserved during deep sleep
RTC_DATA_ATTR static int iCount = 0;        // number of wakeups from deep sleep
RTC_DATA_ATTR uint64_t oldEpoch = 0; // previous time for doing partial updates after deep sleep
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
// Change these to your favorite language; accented characters are okay too!
const char *szMonths[] = {"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
const char *szDays[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//
// Draw the current time
// Parameters:
//   bPartial - true = use partial (non-flickering update)
//   epoch - the epoch time/date to display
//   bUseAsOld - flag indicating to only draw the image internally and not display it
//
void  DisplayTime(bool bPartial, uint64_t epoch, bool bUseAsOld)
{
char szTemp[32];
BB_RECT rect;

  bbep.fillScreen(BBEP_WHITE);
  bbep.setFont(Roboto_Black_200);
  bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
  // Get the current time
//  localtime_r((time_t *)&epoch, &myTime);
  struct tm *myTime = gmtime ((time_t *)&epoch);
  sprintf(szTemp, "%02d:%02d", myTime->tm_hour, myTime->tm_min);
  bbep.getStringBox(szTemp, &rect);
  bbep.setCursor((bbep.width() - rect.w)/2, 350); // horizontal center
  bbep.print(szTemp);
  bbep.setFont(Roboto_Black_80);
  sprintf(szTemp, "%s", szDays[myTime->tm_wday]);
  bbep.getStringBox(szTemp, &rect);
  bbep.setCursor((bbep.width() - rect.w)/2, 500); // horizontal center
  bbep.print(szTemp); // print the day of the week
  sprintf(szTemp, "%s %d", szMonths[myTime->tm_mon], myTime->tm_mday);
  bbep.getStringBox(szTemp, &rect);
  bbep.setCursor((bbep.width() - rect.w)/2, 660); // horizontal center
  bbep.print(szTemp); // print the day of the week

  if (bUseAsOld) {
    bbep.backupPlane(); // copy current image to previous
  } else {
    int rc;
    if (bPartial) {
        rc = bbep.partialUpdate(false);
    } else {
        rc = bbep.fullUpdate(CLEAR_FAST, false);
    }
  }
} /* DisplayTime() */

void setup() {
bool res;
int rc, iTimeout;

Serial.begin(115200);
delay(3000);
Serial.println("Starting...");
// This demo was written for Martin Fasani's "Sensoria" S3 and C5 boards
// They feature a 5.2" 1280x720 8-bit parallel eink panel
  rc = bbep.initPanel(BB_PANEL_V7_RAW);
  if (rc != BBEP_SUCCESS) Serial.println("Init panel error");
  rc = bbep.setPanelSize(BBEP_DISPLAY_ED052TC4);
  if (rc != BBEP_SUCCESS) Serial.println("Init panel size error");
  bbep.setRotation(180);
  bbep.fillScreen(BBEP_WHITE);
  bbep.setPasses(6,6); // depends on the panel you're using - larger number = more contrast

  if (iCount == 0) { // First time starting from reset - show WiFi status
    bbep.setFont(FONT_12x16);
    bbep.setCursor(0, 16);
    bbep.setTextColor(BBEP_BLACK, BBEP_WHITE);
    bbep.printf("Connecting to WiFi AP: %s\n", ssid);
    bbep.fullUpdate(CLEAR_SLOW);
  }

  if ((iCount & 2047) == 0) { // sync the internal RTC to NTP time every 35 hours
      WiFi.begin(ssid, password);
      iTimeout = 0;
      while (WiFi.status() != WL_CONNECTED && iTimeout < 60) {
          delay(500); // allow up to 10 seconds to connect
          iTimeout++;
      }
      if (WiFi.status() == WL_CONNECTED) {
          if (iCount == 0) {
            bbep.println("\nConnected!");
            bbep.partialUpdate(false);
            bbep.fillScreen(BBEP_WHITE);
          }
      } else {
          if (iCount == 0) {
            bbep.println("\nConnection failed!");
            bbep.partialUpdate(false);
          }
          while (1) {}
      }
// Initialize a NTPClient to get time
      timeClient.begin();
      timeClient.setTimeOffset(TIME_OFFSET);
      uint64_t epochTime = 0;
      // loop until we get the NTP time; this can take many seconds on the ESP32-C5
      while (epochTime < 1577836800) {
        timeClient.update();
        epochTime = timeClient.getEpochTime();
        vTaskDelay(50); // wait until the time is actually set correctly
      }
      //Get a time structure
      struct tm *ptm = gmtime ((time_t *)&epochTime);
      timeval tv;
      tv.tv_sec = epochTime;
      settimeofday(&tv, nullptr); // set internal RTC with the correct time
      timeClient.end(); // don't need it any more
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
  } // need to sync the time

} /* setup() */

void deepSleep(uint64_t time_in_ms)
{
  esp_sleep_enable_timer_wakeup(time_in_ms * 1000L);
  esp_deep_sleep_start();
}

void loop() {
struct timeval tv;

  gettimeofday(&tv, NULL);
  if ((iCount % 10) == 0) {
    DisplayTime(false, tv.tv_sec, false); // do a full update every 10 times
  } else {
    DisplayTime(true, oldEpoch, true); // redraw old time buffer for partial update
    DisplayTime(true, tv.tv_sec, false); // draw new time and do partial update
  }
  oldEpoch = tv.tv_sec;
  iCount++;
  bbep.deInit(); // put eInk hardware in low power mode
  deepSleep(60000); // sleep for 1 minute (saves energy when running from a battery)
  //delay(60000); // use delay if you want to keep PSRAM and USB active
} /* loop() */
