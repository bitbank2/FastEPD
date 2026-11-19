#include <FastEPD.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include <string>

FASTEPD epaper;

unsigned int WIDTH = 1872;
unsigned int HEIGHT = 1404;
int PANNEL = BB_PANEL_V7_103;

extern "C" {
void app_main();
}

static const char *TAG = "GRAYSCALE_TEST";

void drawTest(bool flipped) {
    epaper.fillScreen(0xf);
    int color = (flipped) ? 15 : 0;
    epaper.setFont(FONT_12x16);
    epaper.setTextColor(BBEP_BLACK);
    for (int i=0; i<WIDTH; i+=WIDTH/16) {
        std::string str = std::to_string(color);
        epaper.drawString(str.c_str(), i+(WIDTH/32), HEIGHT-50+5);
        epaper.fillRect(i, 0, WIDTH/16, HEIGHT-50, flipped ? color-- : color++);
    }
    epaper.fullUpdate();
}
void app_main(void)
{
    epaper.initPanel(PANNEL);
    epaper.setPanelSize(WIDTH, HEIGHT);
    epaper.setMode(BB_MODE_4BPP);

    ESP_LOGI(TAG, "Starting grayscale test");
    bool flipped = false;
    while (1) {
        ESP_LOGI(TAG, "Drawing test with orientation %d", flipped);
        drawTest(flipped);
        flipped = !flipped;
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}