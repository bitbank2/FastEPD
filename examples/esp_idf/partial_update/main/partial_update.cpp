#include <stdio.h>

#include <FastEPD.h>
#include "font80.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DISPLAY_WIDTH 1872
#define DISPLAY_HEIGHT 1404

int PANNEL = BB_PANEL_V7_103;

FASTEPD epaper;

extern "C" {
    void app_main(void);
}

void app_main(void)
{
    while(1) {
        epaper.initPanel(PANNEL);
        epaper.setPanelSize(DISPLAY_WIDTH, DISPLAY_HEIGHT);
        epaper.setMode(BB_MODE_1BPP);
        epaper.fillScreen(BBEP_WHITE);
        epaper.setFont(FONT_12x16);
        epaper.setTextColor(BBEP_BLACK);
        
        // Calculate progress bar position - centered with width 300px
        int progressBarWidth = 300;
        int progressBarHeight = 15;
        int progressBarX = (DISPLAY_WIDTH - progressBarWidth) / 2;
        int progressBarY = 550;
        

        // Demo 1: Small font with progress bar
        epaper.drawString("Partial Update Demo", 500, 500);
        epaper.fullUpdate();

        
        // Draw progress bar outline
        epaper.drawRect(progressBarX, progressBarY, progressBarWidth, progressBarHeight, BBEP_BLACK);

        for(int i=0; i < 100; i+=2) {
            char buffer[32];
            sprintf(buffer, "%d%%", i);  // Create formatted string with percentage
            
            // Clear previous text and progress fill
            epaper.fillRect(500, 520, 200, 40, BBEP_WHITE);
            epaper.fillRect(progressBarX + 1, progressBarY + 1, progressBarWidth - 2, progressBarHeight - 2, BBEP_WHITE);
            
            // Draw text
            epaper.drawString(buffer, 500, 520);
            
            // Fill progress bar according to percentage
            int fillWidth = (int)((progressBarWidth - 2) * i / 100.0);
            epaper.fillRect(progressBarX + 1, progressBarY + 1, fillWidth, progressBarHeight - 2, BBEP_BLACK);
            
            // Update only the areas that changed
            epaper.partialUpdate(true, 500, progressBarY + progressBarHeight);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        
        // Demo 2: Large font with progress bar
        epaper.setFont(font80);
        epaper.fillScreen(BBEP_WHITE);
        epaper.drawString("Partial Update Demo", 100, 500);
        epaper.fullUpdate();
        
        // Recalculate progress bar for second demo
        progressBarY = 700;
        progressBarHeight = 25;
        
        // Draw progress bar outline
        epaper.drawRect(progressBarX, progressBarY, progressBarWidth, progressBarHeight, BBEP_BLACK);
        
        const unsigned int font_height_pixel = 110;
        for(int i=0; i < 100; i+=2) {
            char buffer[32];
            sprintf(buffer, "%d%%", i);  // Create formatted string with percentage
            
            // Clear previous text and progress fill
            epaper.fillRect(100, 620-font_height_pixel, 400, font_height_pixel+2, BBEP_WHITE);
            epaper.fillRect(progressBarX + 1, progressBarY + 1, progressBarWidth - 2, progressBarHeight - 2, BBEP_WHITE);
            
            // Draw text
            epaper.drawString(buffer, 100, 620);
            
            // Fill progress bar according to percentage
            int fillWidth = (int)((progressBarWidth - 2) * i / 100.0);
            epaper.fillRect(progressBarX + 1, progressBarY + 1, fillWidth, progressBarHeight - 2, BBEP_BLACK);
            
            // Update only the areas that changed
            epaper.partialUpdate(true, 620-font_height_pixel, progressBarY + progressBarHeight);
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
