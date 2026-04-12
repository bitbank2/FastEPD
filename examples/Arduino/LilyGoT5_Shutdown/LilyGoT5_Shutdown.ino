//
// Example to show how to turn off the power
// using the I2C battery control chip
// of the LilyGo T5 S3 E-Paper 4.7" Pro
//
#include <FastEPD.h>
#include <Wire.h>
FASTEPD epaper;

#define PWRMGMT_ADDR 0x6b
#define REG09 9
#define SDA_PIN 6
#define SCL_PIN 5
extern "C" {
void s3_prep_diff(uint8_t *pCurr, uint8_t *pPrev, uint8_t *pDest, int iWidth);
}

//
// Shut down the battery power of the LilyGo T5 S3 Pro
// This is the only way to turn off the power and will only
// work when the device is running from the battery. If the
// USB is connected, this will have no effect
//
void T5Pro_Shutdown(void)
{
uint8_t u8;

   Wire.end();
   Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C bus of the power management chip 
   Wire.beginTransmission(PWRMGMT_ADDR); 
   Wire.write(REG09); // read the old value of register 9 first
   Wire.endTransmission();
   Wire.requestFrom(PWRMGMT_ADDR, 1);
   u8 = Wire.read(); // read the current register contents
   Wire.beginTransmission(PWRMGMT_ADDR);
   u8 |= 0x20; // set the BATFET_DIS (disable the battery FET) bit
   Wire.write(REG09); // write back the new value
   Wire.write(u8);
   Wire.endTransmission(); // won't even get here :)
} /* T5Pro_Shutdown() */

void TestSIMD(void)
{
static uint8_t c[16], p[16], d[32];
   memset(c, 0, sizeof(c));
   memset(p, 0, sizeof(p));
   s3_prep_diff(c, p, d, 128);
   Serial.printf("black->black, out: 0x%02x\n", d[0]);
   memset(c, 0xff, sizeof(c));
   s3_prep_diff(c, p, d, 128);
   Serial.printf("black->white, out: 0x%02x\n", d[0]);
   memset(c, 0, sizeof(c));
   s3_prep_diff(c, p, d, 128);
   Serial.printf("white->black, out: 0x%02x\n", d[0]);
   memset(c, 0xff, sizeof(c));
   memset(p, 0xff, sizeof(c));
   s3_prep_diff(c, p, d, 128);
   Serial.printf("white->white, out: 0x%02x\n", d[0]);
   while (1) {
      vTaskDelay(1);
   }
} /* TestSIMD() */
void setup()
{
   Serial.begin(115200);
   delay(3000);
   Serial.println("Starting...");
 //  TestSIMD();
   epaper.initPanel(BB_PANEL_LILYGO_T5PRO);
   epaper.clearWhite();
   epaper.setFont(FONT_12x16);
   epaper.println("Power off in 5 seconds...");
   epaper.partialUpdate(false);
   for (int i=10; i>=0; i--) {
      epaper.printf("Counting...%d\n", i);
      epaper.partialUpdate(false);
      delay(500);
      if (i == 5) {
        T5Pro_Shutdown();
      }
   }
}

void loop()
{
}

