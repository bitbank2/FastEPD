//
// C core functions for bb_epdiy
// Written by Larry Bank (bitbank@pobox.com)
// Copyright (C) 2024 BitBank Software, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================
//
#include "FastEPD.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#if !(defined(CONFIG_ESP32_SPIRAM_SUPPORT) || defined(CONFIG_ESP32S3_SPIRAM_SUPPORT))
#error "Please enable PSRAM support"
#endif

#ifndef __BB_EP__
#define __BB_EP__

// 38 columns by 16 rows. From white (15) to each gray (0-black to 15-white) at 20C
const uint8_t u8GrayMatrix[] = {
/* 0 */	    0,  0,  0,  2,  2,  2,  2,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,
/* 1 */	    2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,  0,
/* 2 */		0,	0,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,  0,
/* 3 */		0,	0,	0,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	0,  0,
/* 4 */		0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,  0,
/* 5 */		0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	0,	0,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	0,  0,
/* 6 */		0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,	1,	1,  0,
/* 7 */		0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,  0,
/* 8 */		0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	0,  0,
/* 9 */		0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,  0,
/* 10 */	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,  0,
/* 11 */	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	0,  0,
/* 12 */	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	0,	0,  0,
/* 13 */	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	0,  0,
/* 14 */	0,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	2,  0,
/* 15 */	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,  0
};

const uint8_t u8SixInchMatrix[] = {
/* 0 */	    0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,
/* 1 */	    2,	2,	2,	2,	2,	2,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,  0,
/* 2 */		0,	0,	2,	2,	2,	2,	2,	2,	2,	0,	0,	0,	0,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,  0,
/* 3 */		0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,  0,
/* 4 */		0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	0,	0,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,  0,
/* 5 */		0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	2,	2,	2,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,	0,  0,
/* 6 */		0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	0,	0,	0,	1,	1,	0,	0,	0,	0,  0,
/* 7 */		0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	0,	0,	0,  0,
/* 8 */		0,	0,	0,	0,	0,	0,	0,	0,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,	1,	0,	0,	0,	0,  0,
/* 9 */		0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	1,	2,	1,	2,  0,
/* 10 */	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	2,	1,	2,	0,	0,	0,  0,
/* 11 */	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	0,	0,	0,	0,  0,
/* 12 */	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	1,	2,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,  0,
/* 13 */	0,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	2,  0,
/* 14 */	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	1,	1,	1,	2,	2,	2,	0,	0,	0,  0,
/* 15 */	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,  0
};

const uint8_t u8M5Matrix[] = {
/* 0 */	    0,  0,  0,  2,  2,  2,  2,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,
/* 1 */	    2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,  0,
/* 2 */		0,	0,	2,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,  0,
/* 3 */		0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,	0,  0,
/* 4 */		0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	0,	0,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,  0,
/* 5 */		0,	0,	0,	0,	0,	0,	0,	1,	1,	2,	2,	2,	2,	1,	1,	1,	1,	1,	0,	0,	0,	0,	0,	0,  0,
/* 6 */		0,	0,	0,	0,	0,	0,	1,	1,	1,	2,	2,	2,	2,	2,	2,	0,	0,	0,	1,	1,	1,	1,	0,	0,  0,
/* 7 */		0,	0,	0,	0,	1,	1,	0,	0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	1,	1,	1,	1,	0,	0,  0,
/* 8 */		0,	0,	0,	0,	0,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,	1,	1,	1,	0,	0,  0,
/* 9 */		0,	0,	1,	1,	1,	1,	1,	1,	1,	0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	2,	1,	1,	0,	0,  0,
/* 10 */	0,	0,	0,	0,	0,	0,	0,	1,	1,	0,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,	1,	1,  0,
/* 11 */	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1,	1,	2,	2,	2,	1,	0,	0,  0,
/* 12 */	0,	0,	0,	1,	1,	1,	1,	1,	1,	2,	2,	2,	1,	1,	2,	2,	2,	2,	2,	2,	2,	1,	0,	0,  0,
/* 13 */	0,	0,	0,	0,	0,	0,	0,	0,	0,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	0,  0,
/* 14 */	0,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	1,	2,  0,
/* 15 */	1,	1,	1,	1,	1,	1,	1,	1,	1,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	2,	0,  0
};
// Forward references
int bbepSetPixel2Clr(void *pb, int x, int y, unsigned char ucColor);
void bbepSetPixelFast2Clr(void *pb, int x, int y, unsigned char ucColor);

//
// Pre-defined panels for popular products and boards
//
// width, height, bus_speed, flags, data[8], bus_width, ioPWR, ioSPV, ioCKV, ioSPH, ioOE, ioLE,
// ioCL, ioPWR_Good, ioSDA, ioSCL, ioShiftSTR/Wakeup, ioShiftMask/vcom, ioDCDummy, graymatrix, sizeof(graymatrix), iLinePadding
const BBPANELDEF panelDefs[] = {
    {0}, // BB_PANEL_NONE
    {960, 540, 20000000, BB_PANEL_FLAG_NONE, {6,14,7,12,9,11,8,10}, 8, 46, 17, 18, 13, 45, 15,
      16, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, 47, u8M5Matrix, sizeof(u8M5Matrix), 0}, // BB_PANEL_M5PAPERS3


    {960, 540, 12000000, BB_PANEL_FLAG_SLOW_SPH, {8,1,2,3,4,5,6,7}, 8, 1, 4, 38, 40, 7, BB_NOT_USED,
      41, BB_NOT_USED, 13, 12, 0, 0x32, 47,u8GrayMatrix, sizeof(u8GrayMatrix), 32}, // BB_PANEL_T5EPAPERS3


    {0, 0, 10000000, BB_PANEL_FLAG_NONE, {5,6,7,15,16,17,18,8}, 8, 11, 45, 48, 41, 8, 42,
      4, 14, 39, 40, BB_NOT_USED, 0, 47, u8SixInchMatrix, sizeof(u8SixInchMatrix), 0}, // BB_PANEL_EPDIY_V7
    {1024, 758, 13333333, BB_PANEL_FLAG_SLOW_SPH, {4,5,18,19,23,25,26,27}, 8, 4, 2, 32, 33, 0, 2,
      0, 7, 21, 22, 3, 5, 15, u8GrayMatrix, sizeof(u8GrayMatrix), 0}, // BB_PANEL_INKPLATE6PLUS

    {1280, 720, 13333333, BB_PANEL_FLAG_SLOW_SPH | BB_PANEL_FLAG_MIRROR_X, {4,5,18,19,23,25,26,27}, 8, 4, 2, 32, 33, 0, 2,
      0, 7, 21, 22, 3, 5, 15, u8GrayMatrix, sizeof(u8GrayMatrix), 16}, // BB_PANEL_INKPLATE5V2


    {960, 540, 10000000, BB_PANEL_FLAG_SLOW_SPH, {33,32,4,19,2,27,21,22}, 8, 1, 4, 25, 26, 7, 15,
      5, BB_NOT_USED, 23, 18, 0, 0x32, 14, u8GrayMatrix, sizeof(u8GrayMatrix), 32}, // BB_PANEL_T5EPAPERV1
    {960, 540, 12000000, BB_PANEL_FLAG_NONE, {5,6,7,15,16,17,8,7}, 8, 11, 45, 48, 41, 8, 42,
      4, 14, 39, 40, BB_NOT_USED, 0, 13,u8GrayMatrix, sizeof(u8GrayMatrix), 0}, // BB_PANEL_T5EPAPERS3PRO
    {0, 0, 10000000, BB_PANEL_FLAG_NONE, {5,6,7,15,16,17,18,8,9,10,11,12,13,14,21,47}, 16, 11, 45, 48, 41, 8, 42,
      4, 14, 39, 40, BB_NOT_USED, 0, 1, u8GrayMatrix, sizeof(u8GrayMatrix), 0}, // BB_PANEL_EPDIY_V7_16
    {1200, 820, 13333333, BB_PANEL_FLAG_SLOW_SPH, {4,5,18,19,23,25,26,27}, 8, 4, 2, 32, 33, 0, 2,
      0, 7, 21, 22, 3, 5, 15, u8GrayMatrix, sizeof(u8GrayMatrix), 16}, // BB_PANEL_INKPLATE10
    {800, 600, 13333333, BB_PANEL_FLAG_SLOW_SPH, {4,5,18,19,23,25,26,27}, 8, 4, 2, 32, 33, 0, 2,
      0, 7, 21, 22, 3, 5, 15, u8GrayMatrix, sizeof(u8GrayMatrix), 16}, // BB_PANEL_INKPLATE6
};
//
// Forward references for panel callback functions
//
// M5Stack PaperS3
int PaperS3EinkPower(void *pBBEP, int bOn);
int PaperS3IOInit(void *pBBEP);
void PaperS3RowControl(void *pBBEP, int iMode);
// LilyGo T5 Epaper S3 V2.4
int LilyGoV24EinkPower(void *pBBEP, int bOn);
int LilyGoV24IOInit(void *pBBEP);
void LilyGoV24RowControl(void *pBBEP, int iMode);
// EPDiy V7
int EPDiyV7EinkPower(void *pBBEP, int bOn);
int EPDiyV7IOInit(void *pBBEP);
void EPDiyV7RowControl(void *pBBEP, int iMode);
// Inkplate6PLUS
int Inkplate6PlusEinkPower(void *pBBEP, int bOn);
int Inkplate6PlusIOInit(void *pBBEP);
void Inkplate6PlusRowControl(void *pBBEP, int iMode);
// Inkplate5V2
int Inkplate5V2EinkPower(void *pBBEP, int bOn);
int Inkplate5V2IOInit(void *pBBEP);
void Inkplate5V2RowControl(void *pBBEP, int iMode);
// List of predefined callback functions for the panels supported by bb_epdiy
// BB_EINK_POWER, BB_IO_INIT, BB_ROW_CONTROL
const BBPANELPROCS panelProcs[] = {
    {0}, // BB_PANEL_NONE
    {PaperS3EinkPower, PaperS3IOInit, PaperS3RowControl}, // BB_PANEL_M5PAPERS3
    {LilyGoV24EinkPower, LilyGoV24IOInit, LilyGoV24RowControl}, // BB_PANEL_T5EPAPERS3
    {EPDiyV7EinkPower, EPDiyV7IOInit, EPDiyV7RowControl}, // BB_PANEL_EPDIY_V7
    {Inkplate6PlusEinkPower, Inkplate6PlusIOInit, Inkplate6PlusRowControl}, // BB_PANEL_INKPLATE6PLUS
    {Inkplate5V2EinkPower, Inkplate5V2IOInit, Inkplate5V2RowControl}, // Inkplate5V2
    {LilyGoV24EinkPower, LilyGoV24IOInit, LilyGoV24RowControl}, // BB_PANEL_T5EPAPERV1
    {EPDiyV7EinkPower, EPDiyV7IOInit, EPDiyV7RowControl}, // BB_PANEL_T5EPAPERS3PRO
    {Inkplate6PlusEinkPower, Inkplate6PlusIOInit, Inkplate6PlusRowControl}, // BB_PANEL_INKPLATE10
    {Inkplate6PlusEinkPower, Inkplate6PlusIOInit, Inkplate6PlusRowControl}, // BB_PANEL_INKPLATE6 (old)
};

uint8_t ioRegs[24]; // MCP23017 copy of I/O register state so that we can just write new bits
static uint16_t LUTW_16[256];
static uint16_t LUTB_16[256];
static uint16_t LUT2_16[256];
// Lookup tables for grayscale mode
static uint32_t *GLUT, *GLUT2;
volatile bool transfer_is_done = true;
uint8_t u8Cache[512];
static gpio_num_t u8CKV, u8SPH;
static uint8_t bSlowSPH = 0;

static bool s3_notify_dma_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{           
    transfer_is_done = true;
    if (bSlowSPH) {
        gpio_set_level(u8SPH, 1); // CS deactivate
    }
    return false;
}           

// Maximum line width = 512 * 4 = 2048 pixels
#define MAX_TX_SIZE 512
esp_lcd_i80_bus_config_t s3_bus_config = { 
    .dc_gpio_num = 0,
    .wr_gpio_num = 0,
    .clk_src = /*LCD_CLK_SRC_DEFAULT,*/ LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    .bus_width = 0,
    .max_transfer_bytes = MAX_TX_SIZE, 
//    .psram_trans_align = 0, // 0 = use default values
//    .sram_trans_align = 0,
};
esp_lcd_panel_io_i80_config_t s3_io_config = {
        .cs_gpio_num = 0,
        .pclk_hz = 12000000,
        .trans_queue_depth = 4,
        .on_color_trans_done = s3_notify_dma_ready,
        .user_ctx = NULL, // debug
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .cs_active_high = 0,
            .reverse_color_bits = 0,
            .swap_color_bytes = 0, // Swap can be done in software (default) or DMA
            .pclk_active_neg = 0,
            .pclk_idle_low = 0,
        },
    };

esp_lcd_i80_bus_handle_t i80_bus = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
#define PWR_GOOD_OK            0xfa
int bbepReadPowerGood(void)
{
uint8_t uc;
    bbepI2CReadRegister(0x48, 0xf, &uc, 1); // register 0xF contains power good status
    return (uc == PWR_GOOD_OK);
} /* bbepReadPowerGood() */
#define MCP23017_IODIRA   0x00
#define MCP23017_GPPUA    0x0C
#define MCP23017_GPIOA    0x12

void bbepMCP23017Init(void)
{
    bbepI2CReadRegister(0x20, 0, &ioRegs[1], 22); // read all of the MCP23017 registers into RAM
    ioRegs[0] = 0; // starting register for writing
    ioRegs[1] = ioRegs[2] = 0xff;
    bbepI2CWrite(0x20, ioRegs, 23); // write 22 register back starting at 0
} /* bbepMCP23017Init */

void bbepMCPPinMode(uint8_t pin, int mode)
{
    uint8_t ucTemp[4];
    uint8_t port = pin / 8;
    pin &= 7;
    switch (mode) {
        case INPUT:
            ioRegs[MCP23017_IODIRA + port + 1] |= 1 << pin;   // Set it to input
            ioRegs[MCP23017_GPPUA + port + 1] &= ~(1 << pin); // Disable pullup
            break;
        case INPUT_PULLUP:
            ioRegs[MCP23017_IODIRA + port + 1] |= 1 << pin; // Set it to input
            ioRegs[MCP23017_GPPUA + port + 1] |= 1 << pin;  // Enable pullup on that pin
        break;
        case OUTPUT:
            ioRegs[MCP23017_IODIRA + port + 1] &= ~(1 << pin); // Set it to output
            ioRegs[MCP23017_GPPUA + port + 1] &= ~(1 << pin);  // Disable pullup on that pin
            break;
    }
    // Update the MCP registers on the device
    ucTemp[0] = MCP23017_IODIRA + port;
    ucTemp[1] = ioRegs[MCP23017_IODIRA + port + 1];
    bbepI2CWrite(0x20, ucTemp, 2);
    ucTemp[0] = MCP23017_GPPUA + port;
    ucTemp[1] = ioRegs[MCP23017_GPPUA + port + 1];
    bbepI2CWrite(0x20, ucTemp, 2);
} /* bbepMCPPinMode() */

void bbepMCPDigitalWrite(uint8_t pin, uint8_t value)
{
    uint8_t ucTemp[4];
    uint8_t port = pin / 8;
    pin &= 7;
    if (ioRegs[MCP23017_IODIRA + port + 1] & (1 << pin))
        return; // pin is set as an input, can't write to it
    if (value) {
        ioRegs[MCP23017_GPIOA + port + 1] |= (1 << pin);
    } else {
        ioRegs[MCP23017_GPIOA + port + 1] &= ~(1 << pin);
    }
    // Update the MCP register on the device
    ucTemp[0] = MCP23017_GPIOA + port;
    ucTemp[1] = ioRegs[MCP23017_GPIOA + port + 1];
    bbepI2CWrite(0x20, ucTemp, 2);
} /* bbepMCPDigitalWrite() */

uint8_t bbepMCPDigitalRead(uint8_t pin)
{
    uint8_t port = pin / 8;
    pin &= 7;

    bbepI2CReadRegister(0x20, MCP23017_GPIOA + port, &ioRegs[MCP23017_GPIOA + port + 1], 1);
    return (ioRegs[MCP23017_GPIOA + port + 1] & (1 << pin)) ? HIGH : LOW;
} /* bbepMCPDigitalRead() */
//
// Read port 0 or 1 data
//
uint8_t bbepPCA9535Read(uint8_t port)
{
uint8_t uc;
    bbepI2CReadRegister(0x20, port, &uc, 1);
    return uc;
} /* bbepPCA9535Read() */
//
// Read port 0 or 1 data
//
void bbepPCA9535Write(uint8_t port, uint8_t data)
{
uint8_t ucTemp[4];
    ucTemp[0] = 2+port; // output port
    ucTemp[1] = data;
    bbepI2CWrite(0x20,ucTemp, 2);
} /* bbepPCA9535Write() */
//
// Set the direction bits for the 16 I/O pins of the PCA9535
//
void bbepPCA9535SetConfig(uint8_t config)
{
uint8_t ucTemp[4];
    ucTemp[0] = 7; // configuration register for PORT1
    ucTemp[1] = config;
    bbepI2CWrite(0x20, ucTemp, 2);
} /* bbepPCA9535SetConfig() */

void bbepTPS65186Init(FASTEPDSTATE *pState)
{
    uint8_t ucTemp[8];
    ucTemp[0] = 0x9; // power up sequence register
    ucTemp[1] = 0x1b; // power up sequence
    ucTemp[2] = 0; // power up delay (3ms per rail)
    ucTemp[3] = 0x1b; // power down seq
    ucTemp[4] = 0; // power down delay (6ms per rail);
    bbepI2CWrite(0x48, ucTemp, 5);
}

void bbepPCAL6416Init(void)
{
    bbepI2CReadRegister(0x20, 0, ioRegs, 23); // read all of the registers into memory to start
}
#define PCAL6416A_INPORT0_ARRAY  0
#define PCAL6416A_OUTPORT0_ARRAY 2
#define PCAL6416A_CFGPORT0_ARRAY 6
#define PCAL6416A_PUPDEN_REG0_ARRAY    14
#define PCAL6416A_PUPDSEL_REG0_ARRAY   16
void bbepPCALPinMode(uint8_t pin, uint8_t mode)
{
    uint8_t ucTemp[4];
    uint8_t port = pin / 8;
    pin &= 7;

    switch (mode) {
    case INPUT:
        ioRegs[PCAL6416A_CFGPORT0_ARRAY + port] |= (1 << pin);
        break;
    case OUTPUT:
        // There is a one cacth! Pins are by default (POR) set as HIGH. So first change it to LOW and then set is as
        // output).
        ioRegs[PCAL6416A_CFGPORT0_ARRAY + port] &= ~(1 << pin);
        ioRegs[PCAL6416A_OUTPORT0_ARRAY + port] &= ~(1 << pin);
        break;
    case INPUT_PULLUP:
        ioRegs[PCAL6416A_CFGPORT0_ARRAY + port] |= (1 << pin);
        ioRegs[PCAL6416A_PUPDEN_REG0_ARRAY + port] |= (1 << pin);
        ioRegs[PCAL6416A_PUPDSEL_REG0_ARRAY + port] |= (1 << pin);
        break;
    case INPUT_PULLDOWN:
        ioRegs[PCAL6416A_CFGPORT0_ARRAY + port] |= (1 << pin);
        ioRegs[PCAL6416A_PUPDEN_REG0_ARRAY + port] |= (1 << pin);
        ioRegs[PCAL6416A_PUPDSEL_REG0_ARRAY + port] &= ~(1 << pin);
        break;
    }
    // Update device registers
    ucTemp[0] = PCAL6416A_CFGPORT0_ARRAY + port;
    ucTemp[1] = ioRegs[PCAL6416A_CFGPORT0_ARRAY + port];
    bbepI2CWrite(0x20, ucTemp, 2);
    ucTemp[0] = PCAL6416A_PUPDEN_REG0_ARRAY + port;
    ucTemp[1] = ioRegs[PCAL6416A_PUPDEN_REG0_ARRAY + port];
    bbepI2CWrite(0x20, ucTemp, 2);
    ucTemp[0] = PCAL6416A_PUPDSEL_REG0_ARRAY + port;
    ucTemp[1] = ioRegs[PCAL6416A_PUPDSEL_REG0_ARRAY + port];
    bbepI2CWrite(0x20, ucTemp, 2);
} /* bbepPCALPinMode() */

void bbepPCALDigitalWrite(uint8_t pin, uint8_t value)
{
    uint8_t ucTemp[4];
    uint8_t port = pin / 8;
    pin &= 7;

    if (value) {
        ioRegs[PCAL6416A_OUTPORT0_ARRAY + port] |= (1 << pin);
    } else {
        ioRegs[PCAL6416A_OUTPORT0_ARRAY + port] &= ~(1 << pin);
    }
    ucTemp[0] = PCAL6416A_OUTPORT0_ARRAY + port;
    ucTemp[1] = ioRegs[PCAL6416A_OUTPORT0_ARRAY + port];
    bbepI2CWrite(0x20, ucTemp, 2);
} /* bbepPCALDigitalWrite() */

uint8_t bbepPCALDigitalRead(uint8_t pin)
{
    uint8_t port = pin / 8;
    pin &= 7;
    bbepI2CReadRegister(0x20, PCAL6416A_INPORT0_ARRAY + port, &ioRegs[PCAL6416A_INPORT0_ARRAY + port], 1);
    return ((ioRegs[PCAL6416A_INPORT0_ARRAY + port] >> pin) & 1);
} /* bbepPCALDigitalRead() */
//
// Write 8 bits (_state.shift_data) to the shift register
//
void bbepSendShiftData(FASTEPDSTATE *pState)
{
    uint8_t uc = pState->shift_data;
    //Serial.printf("Sending shift data: 0x%02x\n", uc);
    // Clear STR (store) to allow updating the value
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 0);
    for (int i=0; i<8; i++) { // bits get pushed in reverse order (bit 7 first)
        gpio_set_level((gpio_num_t)pState->panelDef.ioSCL, 0);
        if (uc & 0x80)
            gpio_set_level((gpio_num_t)pState->panelDef.ioSDA, 1);
        else
            gpio_set_level((gpio_num_t)pState->panelDef.ioSDA, 0);
        uc <<= 1;
        gpio_set_level((gpio_num_t)pState->panelDef.ioSCL, 1);
    }
    // set STR to write the new data to the output to pins
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 1);
} /* bbepSendShiftData() */
//
// change the state of a bit in the shift register mask, and update the outputs
//
void bbepSetShiftBit(FASTEPDSTATE *pBBEP, uint8_t u8Bit, uint8_t u8State)
{
    if (u8State) { // set it
        pBBEP->shift_data |= (1 << u8Bit);
    } else {
        pBBEP->shift_data &= ~(1 << u8Bit);
    }
    bbepSendShiftData(pBBEP);
}
//
// Control the DC/DC power circuit of the M5Stack PaperS3
//
int PaperS3EinkPower(void *pBBEP, int bOn)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    if (bOn == pState->pwr_on) return BBEP_SUCCESS; // already on
    if (bOn) {
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 1);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)pState->panelDef.ioPWR, 1);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPV, 1);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1);
        pState->pwr_on = 1;
    } else { // power off
        gpio_set_level((gpio_num_t)pState->panelDef.ioPWR, 0);
        delay(1); // give a little time to power down
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 0);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPV, 0);
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* PaperS3EinkPower() */
int LilyGoV24EinkPower(void *pBBEP, int bOn)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    if (bOn == pState->pwr_on) return BBEP_SUCCESS; // already on
    if (bOn) {
//        bbepSetShiftBit(pState, 5, 1); // scan_direction = true
        bbepSetShiftBit(pState, 1, 0); // power_disable = false
        delayMicroseconds(100);
        bbepSetShiftBit(pState, 3, 1); // negative_power_enable = true
        delayMicroseconds(500);
        bbepSetShiftBit(pState, 2, 1); // positive_power_enable = true
        delayMicroseconds(100);
        bbepSetShiftBit(pState, 4, 1); // stv = true
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1);
//        bbepSetShiftBit(pState, 5, 1); // mode1 = true
        pState->pwr_on = 1;
    } else { // power off
        bbepSetShiftBit(pState, 2, 0); // positive_power_enable = false
        delayMicroseconds(10);
        bbepSetShiftBit(pState, 3, 0); // negative_power_enable = false
        delayMicroseconds(100);
        bbepSetShiftBit(pState, 1, 1); // power_disable = true
        bbepSetShiftBit(pState, 4, 0); // STV = false
//        gpio_set_level((gpio_num_t)pState->panelDef.ioSPV, 0); // stv = false            
       // bbepSetShiftBit(pState, 5, 0); // mode1 = false
        bbepSetShiftBit(pState, 7, 0); // output_enable = false
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
}
#define TPS_REG_ENABLE 0x01
#define TPS_REG_PG 0x0F
int EPDiyV7EinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t ucTemp[4];
uint8_t u8Value = 0; // I/O bits for the PCA9535

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        //  u8Value |= 4; // STV on DEBUG - not sure why it's not used
        u8Value |= 1; // OE on
        u8Value |= 0x20; // WAKEUP on
        bbepPCA9535Write(1, u8Value);
        u8Value |= 8; // PWRUP on
        bbepPCA9535Write(1, u8Value);
        u8Value |= 0x10; // VCOM CTRL on
        bbepPCA9535Write(1, u8Value);
        vTaskDelay(1); // allow time to power up
        while (!(bbepPCA9535Read(1) & 0x40 /*CFG_PIN_PWRGOOD*/)) { }
        ucTemp[0] = TPS_REG_ENABLE;
        ucTemp[1] = 0x3f; // enable output
        bbepI2CWrite(0x68, ucTemp, 2);
        // set VCOM to 1.6v (1600)
        ucTemp[0] = 3; // vcom voltage register 3+4 = L + H
        ucTemp[1] = (uint8_t)(160);
        ucTemp[2] = (uint8_t)(160 >> 8);
        bbepI2CWrite(0x68, ucTemp, 3);

        int iTimeout = 0;
        u8Value = 0;
        while (iTimeout < 400 && ((u8Value & 0xfa) != 0xfa)) {
            bbepI2CReadRegister(0x68, TPS_REG_PG, &u8Value, 1); // read power good
            iTimeout++;
            vTaskDelay(1);
        }
        if (iTimeout >= 400) {
            // Serial.println("The power_good signal never arrived!");
            return BBEP_IO_ERROR;
        }
        pState->pwr_on = 1;
    } else { // power off
        bbepPCA9535Write(1, 0x20); // only leave WAKEUP on
        vTaskDelay(1);
        bbepPCA9535Write(1, 0); // now turn everything off
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* EPDiyV7EinkPower() */

int Inkplate6PlusEinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t ucTemp[4];

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP_SET;
        delay(5);
        // Modify power up sequence  (VEE and VNEG are swapped)
        ucTemp[0] = 0x09;
        ucTemp[1] = 0xe1;
        bbepI2CWrite(0x48, ucTemp, 2);
        // Enable all rails
        ucTemp[0] = 0x01;
        ucTemp[1] = 0x3f;
        bbepI2CWrite(0x48, ucTemp, 2);
        bbepMCPDigitalWrite(pState->panelDef.ioPWR, 1); // PWRUP_SET;
        //pinsAsOutputs();
        gpio_set_level((gpio_num_t)pState->panelDef.ioLE, 0); // LE_CLEAR;
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 0); // OE_CLEAR;
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1); // SPH_SET;
        bbepMCPDigitalWrite(/*GMOD*/1, 1); // GMOD_SET
        bbepMCPDigitalWrite((uint8_t)pState->panelDef.ioSPV, 1); // SPV_SET;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV_CLEAR;
        bbepMCPDigitalWrite(pState->panelDef.ioShiftMask, 1); // VCOM_SET;
        unsigned long timer = millis();
        do {
            delay(1);
        } while (!bbepReadPowerGood() && (millis() - timer) < 250);
        if ((millis() - timer) >= 250) {
            bbepMCPDigitalWrite(pState->panelDef.ioShiftMask, 0); // VCOM_CLEAR;
            bbepMCPDigitalWrite(pState->panelDef.ioPWR, 0); // PWRUP_CLEAR;
            return BBEP_IO_ERROR;
        }
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 1); // OE_SET;
        pState->pwr_on = 1;
    } else { // power off
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 0); // OE_CLEAR;
        bbepMCPDigitalWrite(/*GMOD*/1, 0); // GMOD_CLEAR;
        bbepMCPDigitalWrite(pState->panelDef.ioLE, 0); //LE_CLEAR;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV_CLEAR
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 0); //SPH_CLEAR;
        bbepMCPDigitalWrite(pState->panelDef.ioSPV, 0); //SPV_CLEAR;
        bbepMCPDigitalWrite(pState->panelDef.ioShiftMask, 0); //VCOM_CLEAR;
        bbepMCPDigitalWrite(pState->panelDef.ioPWR, 0); // PWRUP_CLEAR;
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
}
int Inkplate5V2EinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t ucTemp[4];

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        bbepPCALDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP_SET;
        delay(5);
        // Modify power up sequence  (VEE and VNEG are swapped)
        ucTemp[0] = 0x09;
        ucTemp[1] = 0xe1;
        bbepI2CWrite(0x48, ucTemp, 2);
        // Enable all rails
        ucTemp[0] = 0x01;
        ucTemp[1] = 0x3f;
        bbepI2CWrite(0x48, ucTemp, 2);
        bbepPCALDigitalWrite(pState->panelDef.ioPWR, 1); // PWRUP_SET;
        //pinsAsOutputs();
        gpio_set_level((gpio_num_t)pState->panelDef.ioLE, 0); // LE_CLEAR;
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 0); // OE_CLEAR;
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1); // SPH_SET;
        bbepPCALDigitalWrite(/*GMOD*/1, 1); // GMOD_SET
        bbepPCALDigitalWrite((uint8_t)pState->panelDef.ioSPV, 1); // SPV_SET;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV_CLEAR;
        bbepPCALDigitalWrite(pState->panelDef.ioShiftMask, 1); // VCOM_SET;
        unsigned long timer = millis();
        do {
            delay(1);
        } while (!bbepReadPowerGood() && (millis() - timer) < 250);
        if ((millis() - timer) >= 250) {
            bbepPCALDigitalWrite(pState->panelDef.ioShiftMask, 0); // VCOM_CLEAR;
            bbepPCALDigitalWrite(pState->panelDef.ioPWR, 0); // PWRUP_CLEAR;
            return BBEP_IO_ERROR;
        }
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 1); // OE_SET;
        pState->pwr_on = 1;
    } else { // power off
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 0); // OE_CLEAR;
        bbepPCALDigitalWrite(/*GMOD*/1, 0); // GMOD_CLEAR;
        bbepPCALDigitalWrite(pState->panelDef.ioLE, 0); //LE_CLEAR;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV_CLEAR
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 0); //SPH_CLEAR;
        bbepPCALDigitalWrite(pState->panelDef.ioSPV, 0); //SPV_CLEAR;
        bbepPCALDigitalWrite(pState->panelDef.ioShiftMask, 0); //VCOM_CLEAR;
        bbepPCALDigitalWrite(pState->panelDef.ioPWR, 0); // PWRUP_CLEAR;
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
}
//
// Initialize the (non parallel data) lines of the M5Stack PaperS3
//
int PaperS3IOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    return BBEP_SUCCESS;
} /* PaperS3IOInit() */
//
// Initialize the IO for the LilyGo T5 4.7" V2.4 PCB
//
int LilyGoV24IOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    //Serial.println("Using shift register");
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    if (pState->panelDef.ioLE != BB_NOT_USED) {
        bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    }
    pState->shift_data = pState->panelDef.ioShiftMask;
    bbepPinMode(pState->panelDef.ioShiftSTR, OUTPUT);
    bbepPinMode(pState->panelDef.ioSDA, OUTPUT);
    bbepPinMode(pState->panelDef.ioSCL, OUTPUT);
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 0);
    bbepSendShiftData(pState); // send default control bits
    return BBEP_SUCCESS;
} /* LilyGoV24IOInit() */
//
// Initialize the IO for the EPDiy V7 PCB
//
int EPDiyV7IOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    if (pState->panelDef.ioPWR < 0x100) bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    if (pState->panelDef.ioSPV < 0x100) bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    if (pState->panelDef.ioCKV < 0x100) bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    if (pState->panelDef.ioSPH < 0x100) bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    if (pState->panelDef.ioOE < 0x100) bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    if (pState->panelDef.ioLE < 0x100) bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    if (pState->panelDef.ioCL < 0x100) bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL);
    bbepPCA9535SetConfig(0xc0); // set lower 6 bits as outputs and 6 (PWRGOOD) and 7 (INTR) as inputs
    return BBEP_SUCCESS;
} /* EPDiyV7IOInit() */
//
// Initialize the IO for the Inkplate6PLUS
//
int Inkplate6PlusIOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL);
    bbepMCP23017Init();
    bbepMCPPinMode(pState->panelDef.ioShiftMask, OUTPUT); // VCOM
    bbepMCPPinMode(pState->panelDef.ioPWR, OUTPUT); // PWRUP
    bbepMCPPinMode(pState->panelDef.ioShiftSTR, OUTPUT); // WAKEUP
    bbepMCPPinMode(/*GPIO0_ENABLE*/ 8, OUTPUT);
    bbepMCPDigitalWrite(/*GPIO0_ENABLE*/ 8, HIGH);
    bbepMCPPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepMCPPinMode(/*GMOD*/1, OUTPUT);
    bbepMCPPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP ON
    delay(1);
    bbepTPS65186Init(pState);
    delay(1);
    bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 0); // WAKEUP OFF
    return BBEP_SUCCESS;
} /* Inkplate6PlusIOInit() */

int Inkplate5V2IOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL);
    bbepPCAL6416Init();
    bbepPCALDigitalWrite(9, LOW);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepPCALPinMode(pState->panelDef.ioShiftMask, OUTPUT); // VCOM
    bbepPCALPinMode(pState->panelDef.ioPWR, OUTPUT); // PWRUP
    bbepPCALPinMode(pState->panelDef.ioShiftSTR, OUTPUT); // WAKEUP
    bbepPCALPinMode(/*GPIO0_ENABLE*/ 8, OUTPUT);
    bbepPCALDigitalWrite(/*GPIO0_ENABLE*/ 8, HIGH);
    bbepPCALPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPCALPinMode(/*GMOD*/1, OUTPUT);
    bbepPCALPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPCALDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP ON
    delay(1);
    bbepTPS65186Init(pState);
    delay(1);
    bbepPCALDigitalWrite(pState->panelDef.ioShiftSTR, 0); // WAKEUP OFF
    return BBEP_SUCCESS;
} /* Inkplate5V2IOInit() */
//
// Start or step the current row on the M5Stack PaperS3
//
void PaperS3RowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(ckv, 1); //CKV_SET
        delayMicroseconds(7);
        gpio_set_level(spv, 0); //SPV_CLEAR;
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(8);                    
        gpio_set_level(spv, 1); //SPV_SET;
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        gpio_set_level(le, 1); //LE_SET;
        gpio_set_level(le, 0); //LE_CLEAR;
        delayMicroseconds(0);
    }
} /* PaperS3RowControl() */

void LilyGoV24RowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        bbepSetShiftBit(pState, 6, 1); // EP_MODE = true
        gpio_set_level(ckv, 1); //CKV_SET
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR
        delayMicroseconds(10);
        bbepSetShiftBit(pState, 4, 0); // STV = false
       // gpio_set_level(spv, 0); //SPV_CLEAR;
        delayMicroseconds(1);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(100);                    
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(100);
        bbepSetShiftBit(pState, 4, 1); // STV = true
//        gpio_set_level(spv, 1); //SPV_SET;
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(100);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        bbepSetShiftBit(pState, 7, 1); // output_enable = true
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(10);
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(500);
        if (le == BB_NOT_USED) {
            bbepSetShiftBit(pState, 0, 1); // EP_LE = true
            bbepSetShiftBit(pState, 0, 0); // EP_LE = false
        } else {
            gpio_set_level(le, 1);
            gpio_set_level(le, 0);
        }
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
    }
}
void EPDiyV7RowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(ckv, 1); //CKV_SET
        delayMicroseconds(7);
        gpio_set_level(spv, 0); //SPV_CLEAR;
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(8);                    
        gpio_set_level(spv, 1); //SPV_SET;
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        gpio_set_level(le, 1); //LE_SET;
        gpio_set_level(le, 0); //LE_CLEAR;
        delayMicroseconds(0);
    }
}
void Inkplate6PlusRowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(ckv, 1); //CKV_SET
        delayMicroseconds(7);
        bbepMCPDigitalWrite(spv, 0);
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(8);                    
        bbepMCPDigitalWrite(spv, 1);
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 1);
        gpio_set_level(ckv, 1); //CKV_SET;
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        gpio_set_level(le, 1); //LE_SET;
        gpio_set_level(le, 0); //LE_CLEAR;
        delayMicroseconds(0);
    }
}
void Inkplate5V2RowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(ckv, 1); //CKV_SET
        delayMicroseconds(7);
        bbepPCALDigitalWrite(spv, 0);
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(8);                    
        bbepPCALDigitalWrite(spv, 1);
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 1);
        gpio_set_level(ckv, 1); //CKV_SET;
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        gpio_set_level(le, 1); //LE_SET;
        gpio_set_level(le, 0); //LE_CLEAR;
        delayMicroseconds(0);
    }
}
void bbepRowControl(FASTEPDSTATE *pState, int iType)
{
    (*(pState->pfnRowControl))(pState, iType);
    return;
} /* bbepRowControl() */

void bbepWriteRow(FASTEPDSTATE *pState, uint8_t *pData, int iLen)
{
    esp_err_t err;

    while (!transfer_is_done) {
        delayMicroseconds(1);
    }
    if (bSlowSPH) {
        gpio_set_level(u8SPH, 0); // CS active
        gpio_set_level(u8CKV, 1); // CKV set
    }
    transfer_is_done = false;
    gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); //CKV_SET;
    err = esp_lcd_panel_io_tx_color(io_handle, -1, pData, iLen + pState->panelDef.iLinePadding);
    if (err != ESP_OK) {
     //   Serial.printf("Error %d sending LCD data\n", (int)err);
    }
    while (!transfer_is_done) {
        delayMicroseconds(1);
    }
}

uint8_t TPS65185PowerGood(void)
{
uint8_t ucTemp[4];

    bbepI2CReadRegister(0x68, 0x0f, ucTemp, 1);
    return ucTemp[0];
}

int bbepIOInit(FASTEPDSTATE *pState)
{
    int rc = (*(pState->pfnIOInit))(pState);
    if (rc != BBEP_SUCCESS) return rc;
    // Initialize the ESP32 LCD API to drive parallel data at high speed
    // The code forces the use of a D/C pin, so we must assign it to an unused GPIO on each device
    s3_bus_config.dc_gpio_num = (gpio_num_t)pState->panelDef.ioDCDummy;
    s3_bus_config.wr_gpio_num = (gpio_num_t)pState->panelDef.ioCL;
    s3_bus_config.bus_width = pState->panelDef.bus_width;
    for (int i=0; i<pState->panelDef.bus_width; i++) {
        s3_bus_config.data_gpio_nums[i] = pState->panelDef.data[i];
    }   
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&s3_bus_config, &i80_bus));
    s3_io_config.pclk_hz = pState->panelDef.bus_speed;
    if (pState->panelDef.flags & BB_PANEL_FLAG_SLOW_SPH) {
        bSlowSPH = 1;
        u8SPH = (gpio_num_t)pState->panelDef.ioSPH;
        u8CKV = (gpio_num_t)pState->panelDef.ioCKV;
        s3_io_config.cs_gpio_num = -1; // disable hardware CS
    } else {
        s3_io_config.cs_gpio_num = (gpio_num_t)pState->panelDef.ioSPH;
    }
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &s3_io_config, &io_handle));
    transfer_is_done = true;
    //Serial.println("IO init done");
// Create the lookup tables for 1-bit mode. Allow for inverted and mirrored
    for (int i=0; i<256; i++) {
        uint16_t b, w, l2, u16W, u16B, u16L2;
        u16W = u16B = u16L2 = 0;
        for (int j=0; j<8; j++) {
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                if (!(i & (1<<(7-j)))) {
                    w = 2; b = 1; l2 = 1;
                } else {
                    w = 3; b = 3; l2 = 2;
                }
            } else {
                if (!(i & (0x80>>(7-j)))) {
                    w = 2; b = 1; l2 = 1;
                } else {
                    w = 3; b = 3; l2 = 2;
                }
            }
            u16W |= (w << (j * 2));
            u16B |= (b << (j * 2));
            u16L2 |= (l2 << (j * 2));
        } // for j
        LUTW_16[i] = __builtin_bswap16(u16W);
        LUTB_16[i] = __builtin_bswap16(u16B);
        LUT2_16[i] = __builtin_bswap16(u16L2);
    } // for i

    // Allocate memory for each line to transmit
      pState->dma_buf = (uint8_t *)heap_caps_malloc((pState->width / 4) + pState->panelDef.iLinePadding + 16, MALLOC_CAP_DMA);

    return BBEP_SUCCESS;
} /* bbepIOInit() */

int bbepSetPanelSize(FASTEPDSTATE *pState, int width, int height, int flags) {
    pState->width = pState->native_width = width;
    pState->height = pState->native_height = height;
    pState->iFlags = flags;
    pState->pCurrent = (uint8_t *)heap_caps_aligned_alloc(16, pState->width * pState->height / 2, MALLOC_CAP_SPIRAM); // current pixels
    pState->pPrevious = (uint8_t *)heap_caps_aligned_alloc(16, pState->width * pState->height / 2, MALLOC_CAP_SPIRAM); // comparison with previous buffer
    pState->pTemp = (uint8_t *)heap_caps_aligned_alloc(16, pState->width * pState->height / 4, MALLOC_CAP_SPIRAM); // LUT data
    return BBEP_SUCCESS;
} /* setPanelSize() */

//
// Initialize the panel based on the constant name
// Each name points to a configuration with info about the PCB and display
// e.g. BB_PANEL_M5PAPERs3
// The device is ready to use after returning from this function
//
int bbepInitPanel(FASTEPDSTATE *pState, int iPanel)
{
    int rc, iPasses;
    uint8_t *pMatrix;
    if (iPanel > 0 && iPanel < BB_PANEL_COUNT) {
        pState->iPanelType = iPanel;
        pState->width = pState->native_width = panelDefs[iPanel].width;
        pState->height = pState->native_height = panelDefs[iPanel].height;
        memcpy(&pState->panelDef, &panelDefs[iPanel], sizeof(BBPANELDEF));
        pState->iFlags = pState->panelDef.flags; // copy flags to main class structure
        // get the 3 callback functions
        pState->pfnEinkPower = panelProcs[iPanel].pfnEinkPower;
        pState->pfnIOInit = panelProcs[iPanel].pfnIOInit;
        pState->pfnRowControl = panelProcs[iPanel].pfnRowControl;
        rc = bbepIOInit(pState);
        pState->mode = BB_MODE_1BPP; // start in 1-bit mode
        pState->iFG = BBEP_BLACK;
        pState->iBG = BBEP_TRANSPARENT;
        if (rc == BBEP_SUCCESS) {
            // allocate memory for the buffers
            if (pState->width) { // if size is defined
                pState->pCurrent = (uint8_t *)heap_caps_aligned_alloc(16, pState->width * pState->height / 2, MALLOC_CAP_SPIRAM); // current pixels
                pState->pPrevious = (uint8_t *)heap_caps_aligned_alloc(16, pState->width * pState->height / 2, MALLOC_CAP_SPIRAM); // comparison with previous buffer
                pState->pTemp = (uint8_t *)heap_caps_aligned_alloc(16, pState->width * pState->height / 4, MALLOC_CAP_SPIRAM); // LUT data
            }
        }
        iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
        GLUT = (uint32_t *)malloc(256 * iPasses * sizeof(uint32_t));
        GLUT2 = (uint32_t *)malloc(256 * iPasses * sizeof(uint32_t));
        // Prepare grayscale lookup tables
        pMatrix = (uint8_t *)pState->panelDef.pGrayMatrix;
        for (int j = 0; j < iPasses; j++) {
            for (int i = 0; i < 256; i++) {
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    GLUT[j * 256 + i] = (pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j]);
                    GLUT2[j * 256 + i] = ((pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j])) << 4;
                } else {
                    GLUT[j * 256 + i] = (pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j]);
                    GLUT2[j * 256 + i] = ((pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j])) << 4;
                }
            }
        }
        pState->pfnSetPixel = bbepSetPixel2Clr;
        pState->pfnSetPixelFast = bbepSetPixelFast2Clr;
        return rc;
    }
    return BBEP_ERROR_BAD_PARAMETER;
} /* bbepInitPanel() */

int bbepEinkPower(FASTEPDSTATE *pState, int bOn)
{
    return (*(pState->pfnEinkPower))(pState, bOn);
} /* bbepEinkPower() */

//
// Clear the display with the given code for the given number of repetitions
// val: 0 = lighten, 1 = darken, 2 = discharge, 3 = skip
//
void bbepClear(FASTEPDSTATE *pState, uint8_t val, uint8_t count, BBEPRECT *pRect)
{
    uint8_t u8;
    int i, k, iStartCol, iEndCol, iStartRow, iEndRow; // clipping area
    if (val == 0) val = 0xaa;
    else if (val == 1) val = 0x55;
    else if (val == 2) val = 0x00;
    else val = 0xff;

    if (pRect) {
        iStartCol = pRect->x;
        iEndCol = iStartCol + pRect->w - 1;
        iStartRow = pRect->y;
        iEndRow = iStartRow + pRect->h - 1;
        if (iStartCol >= iEndCol || iStartRow >= iEndRow) return; // invalid area

        if (iStartCol < 0) iStartCol = 0;
        if (iStartRow < 0) iStartRow = 0;
        if (iEndCol >= pState->width) iEndCol = pState->width - 1;
        if (iEndRow >= pState->height) iEndRow = pState->height - 1;
        switch (pState->rotation) { // rotate to native panel direction
            case 0: // nothing to do
                break;
            case 90:
                i = iStartCol;
                iStartCol = iStartRow;
                iStartRow = pState->width - 1 - iEndCol;
                iEndCol = iEndRow;
                iEndRow = pState->width - 1 - i; // iStartCol
                break;
            case 270:
                i = iStartCol;
                iStartCol = pState->height - 1 - iEndRow;
                iEndRow = iEndCol;
                iEndCol = pState->height - 1 - iStartRow;
                iStartRow = i; // iStartCol
                break;
            case 180:
                iStartCol = pState->width - 1 - iStartCol;
                iEndCol = pState->width - 1 - iEndCol;
                iStartRow = pState->height - 1 - iStartRow;
                iEndRow = pState->height - 1 - iEndRow;
                break;
        }
    } else { // use the whole display
        iStartCol = iStartRow = 0;
        iEndCol = pState->native_width - 1;
        iEndRow = pState->native_height - 1;
    }
    // Prepare masked row
    memset(u8Cache, val, pState->native_width / 4);
    i = iStartCol/4;
    memset(u8Cache, 0xff, i); // whole bytes on left side
    if ((iStartCol & 3) != 0) { // partial byte
        u8 = 0xff << ((4-(iStartCol & 3))*2);
        u8 |= val;
        u8Cache[i] = u8;
    }
    i = (iEndCol + 3)/4;
    memset(&u8Cache[i], 0xff, (pState->native_width / 4) - i); // whole bytes on right side
    if ((iEndCol & 3) != 3) { // partial byte
        u8 = 0xff >> (((iEndCol & 3)+1)*2);
        u8 |= val;
        u8Cache[i-1] = u8;
    }
    for (k = 0; k < count; k++) {
        bbepRowControl(pState, ROW_START);
        for (i = 0; i < pState->native_height; i++)
        {
            // Send the data
            if (i < iStartRow || i > iEndRow) { // skip this row
                memset(pState->dma_buf, 0xff, pState->native_width / 4);
            } else { // mask the area we want to change
                memcpy(pState->dma_buf, u8Cache, pState->native_width / 4);
            }
            bbepWriteRow(pState, pState->dma_buf, pState->native_width / 4);
           // delayMicroseconds(15);
            bbepRowControl(pState, ROW_STEP);
        }
        delayMicroseconds(230);
    }
} /* bbepClear() */

int bbepFullUpdate(FASTEPDSTATE *pState, bool bFast, bool bKeepOn, BBEPRECT *pRect)
{
    int passes;
#ifdef SHOW_TIME
    long l = millis();
#endif
    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
// Fast mode ~= 600ms, normal mode ~=1000ms
    passes = (bFast) ? 6:11;
    if (!bFast) { // skip initial black phase for fast mode
        bbepClear(pState, 0, 1, pRect);
        bbepClear(pState, 1, passes, pRect);
        bbepClear(pState, 2, 1, pRect);
    }
    bbepClear(pState, 0, passes, pRect);
    bbepClear(pState, 2, 1, pRect);
    bbepClear(pState, 1, passes, pRect);
    bbepClear(pState, 2, 1, pRect);
    bbepClear(pState, 0, passes, pRect);

    if (pState->mode == BB_MODE_1BPP) {
        // Set the color in multiple passes starting from white
        // First create the 2-bit codes per pixel for the black pixels
        uint8_t *s, *d;
        int dy; // destination Y for flipped displays
        for (int i = 0; i < pState->native_height; i++) {
            s = &pState->pCurrent[i * (pState->native_width/8)];
            dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
            d = &pState->pTemp[dy * (pState->native_width/4)];
            memcpy(&pState->pPrevious[i * (pState->native_width/8)], s, pState->native_width / 8); // previous = current
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                s += (pState->native_width/8) - 1;
                for (int n = 0; n <(pState->native_width / 4); n += 4) {
                    uint8_t dram2 = *(s--);
                    uint8_t dram1 = *(s--);
                    *(uint16_t *)&d[n] = LUTB_16[dram2];
                    *(uint16_t *)&d[n+2] = LUTB_16[dram1];
                }
            } else {
                for (int n = 0; n < (pState->native_width / 4); n += 4) {
                    uint8_t dram1 = *s++;
                    uint8_t dram2 = *s++;
                    *(uint16_t *)&d[n+2] = LUTB_16[dram2];
                    *(uint16_t *)&d[n] = LUTB_16[dram1];
                }
            }
        }
        // Write 5 passes of the black data to the whole display
        for (int k = 0; k < 5; ++k) {
            bbepRowControl(pState, ROW_START);
            for (int i = 0; i < pState->native_height; i++) {
                s = &pState->pTemp[i * (pState->native_width / 4)];
                // Send the data for the row
                bbepWriteRow(pState, s, (pState->native_width / 4));
              //  delayMicroseconds(15);
                bbepRowControl(pState, ROW_STEP);
            }
            delayMicroseconds(230);
        }

        for (int k = 0; k < 1; ++k) {
            uint8_t *s, *d;
            bbepRowControl(pState, ROW_START);
            for (int i = 0; i < pState->native_height; i++) {
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                s = &pState->pCurrent[(pState->native_width/8) * dy];
                d = pState->dma_buf;
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                s += (pState->native_width/8) - 1;
                for (int n = 0; n < (pState->native_width / 4); n += 4) {
                    uint8_t dram2 = *s--;
                    uint8_t dram1 = *s--;
                    *(uint16_t *)&d[n] = LUT2_16[dram2];
                    *(uint16_t *)&d[n+2] = LUT2_16[dram1];
                }
            } else {
                for (int n = 0; n < (pState->native_width / 4); n += 4) {
                    uint8_t dram1 = *s++;
                    uint8_t dram2 = *s++;
                    *(uint16_t *)&d[n+2] = LUT2_16[dram2];
                    *(uint16_t *)&d[n] = LUT2_16[dram1];
                }
            }
                // Send the data for the row
                bbepWriteRow(pState, d, (pState->native_width / 4));
              //  delayMicroseconds(15);
                bbepRowControl(pState, ROW_STEP);
            }
            delayMicroseconds(230);
        }

        for (int k = 0; k < 1; ++k) {
            bbepRowControl(pState, ROW_START);
            memset((void *)pState->dma_buf, 0, pState->native_width /4); // send 0's
            for (int i = 0; i < pState->native_height; i++) {
                // Send the data for the row
                bbepWriteRow(pState, pState->dma_buf, (pState->native_width / 4));
            //    delayMicroseconds(15);
                bbepRowControl(pState, ROW_STEP);
            }
            delayMicroseconds(230);
        }
    } else { // must be 4BPP mode
        int iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
        for (int k = 0; k < iPasses; k++) { // number of passes to make 16 unique gray levels
            uint8_t *s, *d = pState->dma_buf;
            bbepRowControl(pState, ROW_START);
            for (int i = 0; i < pState->native_height; i++) {
                s = &pState->pCurrent[i *(pState->native_width / 2)];
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s += (pState->native_width / 2) - 8;
                    for (int j = 0; j < (pState->native_width / 4); j += 4) {
                        d[j + 0] = (GLUT2[k * 256 + s[7]] | GLUT[k * 256 + s[6]]);
                        d[j + 1] = (GLUT2[k * 256 + s[5]] | GLUT[k * 256 + s[4]]);
                        d[j + 2] = (GLUT2[k * 256 + s[3]] | GLUT[k * 256 + s[2]]);
                        d[j + 3] = (GLUT2[k * 256 + s[1]] | GLUT[k * 256 + s[0]]);
                        s -= 8;
                    } // for j
                } else {
                    for (int j = 0; j < (pState->native_width / 4); j += 4) {
                        d[j + 0] = (GLUT2[k * 256 + s[0]] | GLUT[k * 256 + s[1]]);
                        d[j + 1] = (GLUT2[k * 256 + s[2]] | GLUT[k * 256 + s[3]]);
                        d[j + 2] = (GLUT2[k * 256 + s[4]] | GLUT[k * 256 + s[5]]);
                        d[j + 3] = (GLUT2[k * 256 + s[6]] | GLUT[k * 256 + s[7]]);
                        s += 8;
                    } // for j
                }
                bbepWriteRow(pState, pState->dma_buf, (pState->native_width / 4));
               // delayMicroseconds(15);
                bbepRowControl(pState, ROW_STEP);
            } // for i
            delayMicroseconds(230);
        } // for k
    } // 4bpp

        // Set the drivers inside epaper panel into dischare state.
        bbepClear(pState, 2, 1, pRect);
        if (!bKeepOn) bbepEinkPower(pState, 0);
    
#ifdef SHOW_TIME
    l = millis() - l;
    //Serial.printf("fullUpdate time: %dms\n", (int)l);
#endif
    return BBEP_SUCCESS;
} /* bbepFullUdate() */

int bbepPartialUpdate(FASTEPDSTATE *pState, bool bKeepOn, int iStartLine, int iEndLine)
{
//   long l = millis();
    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    if (iStartLine < 0) iStartLine = 0;
    if (iEndLine >= pState->native_height) iEndLine = pState->native_height-1;
    if (iEndLine < iStartLine) return BBEP_ERROR_BAD_PARAMETER;

    uint8_t *pCur, *pPrev, *d;
    uint8_t diffw, diffb, cur, prev;
    
    for (int i = iStartLine; i <= iEndLine; i++) {
        d = &pState->pTemp[i * (pState->native_width/4)]; // LUT temp storage
        pCur = &pState->pCurrent[i * (pState->native_width / 8)];
        pPrev = &pState->pPrevious[i * (pState->native_width / 8)];
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
            pCur += (pState->native_width / 8) - 1;
            pPrev += (pState->native_width / 8) - 1;
            for (int j = 0; j < pState->native_width / 16; j++) {
                cur = *pCur--; prev = *pPrev--;
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

                cur = *pCur--; prev = *pPrev--;
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
                d += 4;
            }
        } else {
            for (int j = 0; j < pState->native_width / 16; j++) {
                cur = *pCur++; prev = *pPrev++;
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

                cur = *pCur++; prev = *pPrev++;
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
                d += 4;
            }
        }
    }
    for (int k = 0; k < 6; ++k) { // each pass is about 32ms
        uint8_t *dp = pState->pTemp;
        int iSkipped = 0;
        bbepRowControl(pState, ROW_START);
        for (int i = 0; i < pState->native_height; i++) {
            if (i >= iStartLine && i <= iEndLine) {
                // Send the data
                bbepWriteRow(pState, dp, (pState->native_width / 4));
                iSkipped = 0;
            } else {
                if (iSkipped >= 2) {
                    gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); // CKV_SET;
                    delayMicroseconds(35);
                } else {
                    // write 2 floating rows
                    if (iSkipped == 0) { // skip
                       memset((void *)pState->dma_buf, 0xff, pState->native_width/4);
                    }
                    bbepWriteRow(pState, pState->dma_buf, (pState->native_width / 4));
                }
                iSkipped++;
            }
            bbepRowControl(pState, ROW_STEP);
            dp += (pState->native_width / 4);
        }
        //delayMicroseconds(230);
    }

    if (bKeepOn) {
        bbepClear(pState, 2, 1, NULL);
    } else {
        bbepEinkPower(pState, 0);
    }
    int offset = iStartLine * (pState->native_width/8);
    memcpy(&pState->pPrevious[offset], &pState->pCurrent[offset], (pState->native_width/8) * (iEndLine - iStartLine+1));

//    l = millis() - l;
//    Serial.printf("partial update time: %dms\n", (int)l);
    return BBEP_SUCCESS;
} /* bbepPartialUpdate() */
//
// Copy the current pixels to the previous
// This facilitates doing partial updates after the power is lost
//
void bbepBackupPlane(FASTEPDSTATE *pState)
{
    int iSize = (pState->native_width/2) * pState->native_height;
    memcpy(pState->pPrevious, pState->pCurrent, iSize);
}
#endif // __BB_EP__
