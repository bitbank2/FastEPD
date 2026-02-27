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
#ifdef ESP_PLATFORM
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_log.h>
#if PSRAM != enabled && !defined(CONFIG_ESP32_SPIRAM_SUPPORT) && !defined(CONFIG_ESP32S3_SPIRAM_SUPPORT)
#error "Please enable PSRAM support"
#endif
#endif

#ifndef __BB_EP__
#define __BB_EP__
#pragma GCC optimize("O2")

// For measuring the performance of each stage of updates
//#define SHOW_TIME
// 8 columns by 16 rows. From white (15) to each gray (0-black to 15-white) at 20C
const uint8_t u8GrayMatrix[] = {
/* 0 */  2, 2, 1, 1, 1, 1, 1, 1, 
/* 1 */  2, 2, 2, 2, 1, 1, 2, 1,
/* 2 */  2, 2, 2, 1, 1, 1, 1, 2, 
/* 3 */  2, 2, 2, 1, 1, 1, 1, 2,
/* 4 */  2, 2, 2, 2, 1, 1, 1, 2,
/* 5 */  2, 2, 2, 2, 1, 2, 2, 1,
/* 6 */  2, 2, 1, 1, 1, 2, 1, 2,
/* 7 */  2, 2, 2, 1, 1, 2, 1, 2, 
/* 8 */  1, 1, 1, 1, 1, 1, 2, 2, 
/* 9 */  2, 1, 1, 1, 1, 1, 2, 2, 
/* 10 */  2, 2, 1, 1, 1, 1, 2, 2, 
/* 11 */  2, 2, 2, 1, 1, 1, 2, 2, 
/* 12 */  2, 1, 1, 2, 1, 1, 2, 2, 
/* 13 */  2, 2, 2, 2, 1, 1, 2, 2, 
/* 14 */  2, 2, 2, 2, 2, 1, 2, 2, 
/* 15 */  2, 2, 2, 2, 2, 2, 2, 2};
// For 5.2" 1280x720 display
const uint8_t u8FivePointTwoMatrix[] = {
/* 0 */  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
/* 1 */  0, 0, 0, 1, 1, 2, 1, 1, 1, 0,
/* 2 */  0, 0, 0, 1, 1, 1, 1, 2, 1, 0,
/* 3 */  0, 0, 0, 0, 0, 1, 2, 1, 1, 0,
/* 4 */  0, 1, 1, 1, 2, 2, 2, 1, 1, 0,
/* 5 */  0, 0, 0, 1, 1, 1, 1, 1, 2, 0,
/* 6 */  0, 0, 0, 0, 1, 1, 1, 1, 2, 0,
/* 7 */  0, 0, 0, 1, 1, 1, 2, 2, 1, 0,
/* 8 */  0, 1, 1, 1, 1, 1, 2, 1, 2, 0,
/* 9 */  0, 0, 0, 1, 1, 1, 2, 1, 2, 0,
/* 10 */  0, 0, 0, 0, 1, 1, 2, 1, 2, 0,
/* 11 */  0, 0, 1, 1, 1, 1, 1, 2, 2, 0,
/* 12 */  0, 0, 0, 0, 0, 1, 2, 1, 2, 0,
/* 13 */  0, 0, 1, 1, 1, 2, 1, 2, 2, 0,
/* 14 */  0, 0, 0, 0, 1, 1, 2, 2, 2, 0,
/* 15 */  2, 2, 2, 2, 2, 2, 2, 2, 2, 0
};
// For (Inkplate 5V2) 5.2" 1280x720 display
const uint8_t u8Ink5V2Matrix[] = {
/* 0 */  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
/* 1 */  0, 0, 0, 1, 1, 2, 1, 1, 1, 0,
/* 2 */  0, 0, 0, 1, 1, 1, 1, 2, 1, 0,
/* 3 */  0, 0, 0, 0, 0, 1, 2, 1, 1, 0,
/* 4 */  0, 1, 1, 1, 2, 2, 2, 1, 1, 0,
/* 5 */  0, 0, 0, 1, 1, 1, 1, 1, 2, 0,
/* 6 */  0, 0, 0, 0, 1, 1, 1, 1, 2, 0,
/* 7 */  0, 0, 0, 1, 1, 1, 2, 2, 1, 0,
/* 8 */  0, 1, 1, 1, 1, 1, 2, 1, 2, 0,
/* 9 */  0, 0, 0, 1, 1, 1, 2, 1, 2, 0,
/* 10 */  0, 0, 0, 0, 1, 1, 2, 1, 2, 0,
/* 11 */  0, 0, 1, 1, 1, 1, 1, 2, 2, 0,
/* 12 */  0, 0, 0, 0, 0, 1, 2, 1, 2, 0,
/* 13 */  0, 0, 1, 1, 1, 2, 1, 2, 2, 0,
/* 14 */  0, 0, 0, 0, 1, 1, 2, 2, 2, 0,
/* 15 */  2, 2, 2, 2, 2, 2, 2, 2, 2, 0
};
// For 10.3" 1872x1404 display
const uint8_t u8TenPointThreeMatrix[] = {
/* 0 */  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
/* 1 */  0, 0, 0, 0, 0, 2, 1, 1, 1, 0,
/* 2 */  1, 1, 1, 0, 1, 1, 1, 2, 1, 0,
/* 3 */  0, 0, 0, 0, 1, 1, 1, 2, 1, 0,
/* 4 */  0, 0, 0, 0, 0, 1, 1, 2, 1, 0,
/* 5 */  0, 0, 1, 1, 1, 1, 2, 2, 1, 0,
/* 6 */  0, 0, 0, 0, 1, 1, 2, 2, 1, 0,
/* 7 */  0, 0, 0, 1, 1, 1, 1, 1, 2, 0,
/* 8 */  1, 1, 1, 1, 1, 2, 1, 1, 2, 0,
/* 9 */  0, 0, 0, 1, 1, 2, 1, 1, 2, 0,
/* 10 */  0, 0, 1, 1, 1, 1, 2, 1, 2, 0,
/* 11 */  0, 0, 0, 0, 1, 1, 2, 1, 2, 0,
/* 12 */  0, 0, 0, 0, 0, 0, 2, 1, 2, 0,
/* 13 */  0, 0, 0, 2, 2, 2, 1, 2, 2, 0,
/* 14 */  0, 0, 0, 1, 2, 1, 2, 2, 2, 0,
/* 15 */  0, 0, 0, 0, 0, 0, 0, 0, 2, 0
};

// For 9.7" 1200x825 panels
const uint8_t u8NineInchMatrix[] = {
/* 0 */  0, 1, 1, 1, 1, 1, 1, 1, 1, 0,
/* 1 */  0, 0, 0, 0, 1, 2, 1, 1, 1, 0,
/* 2 */  0, 0, 0, 1, 1, 1, 1, 2, 1, 0,
/* 3 */  1, 1, 1, 1, 1, 2, 1, 2, 1, 0,
/* 4 */  0, 0, 0, 0, 0, 1, 1, 2, 1, 0,
/* 5 */  0, 0, 1, 1, 1, 1, 2, 2, 1, 0,
/* 6 */  0, 1, 1, 1, 1, 1, 1, 1, 2, 0,
/* 7 */  0, 0, 0, 0, 1, 1, 2, 2, 1, 0,
/* 8 */  0, 0, 0, 0, 0, 1, 2, 2, 1, 0,
/* 9 */  0, 0, 1, 1, 1, 1, 2, 1, 2, 0,
/* 10 */  0, 0, 0, 0, 1, 1, 2, 1, 2, 0,
/* 11 */  0, 0, 0, 0, 0, 1, 2, 1, 2, 0,
/* 12 */  0, 0, 0, 0, 0, 0, 2, 1, 2, 0,
/* 13 */  0, 1, 1, 1, 2, 2, 1, 2, 2, 0,
/* 14 */  0, 0, 0, 0, 1, 1, 2, 2, 2, 0,
/* 15 */  2, 2, 2, 2, 2, 2, 2, 2, 2, 0
};
// For 6.0" 1024x758 panels
const uint8_t u8SixInchMatrix[] = {
    /* 0 */  1, 1, 1, 1, 1, 1, 1, 1, 0,
    /* 1 */  0, 0, 0, 1, 2, 1, 1, 1, 0,
    /* 2 */  0, 0, 0, 1, 1, 1, 2, 1, 0,
    /* 3 */  0, 0, 0, 0, 1, 1, 2, 1, 0,
    /* 4 */  0, 0, 0, 1, 2, 1, 2, 1, 0,
    /* 5 */  0, 0, 0, 0, 2, 1, 2, 1, 0,
    /* 6 */  1, 1, 1, 1, 2, 1, 1, 2, 0,
    /* 7 */  0, 0, 1, 1, 2, 1, 1, 2, 0,
    /* 8 */  0, 0, 0, 1, 2, 1, 1, 2, 0,
    /* 9 */  0, 0, 1, 1, 1, 2, 1, 2, 0,
    /* 10 */ 0, 0, 0, 1, 1, 2, 1, 2, 0,
    /* 11 */ 0, 1, 1, 2, 1, 2, 1, 2, 0,
    /* 12 */ 0, 0, 0, 2, 1, 2, 1, 2, 0,
    /* 13 */ 1, 1, 1, 1, 2, 1, 2, 2, 0,
    /* 14 */ 1, 1, 1, 2, 2, 1, 2, 2, 0,
    /* 15 */ 0, 0, 0, 0, 2, 2, 2, 2, 0
    };

const uint8_t u8M5Matrix[] = {
    /* 0 */  1, 1, 1, 1, 1, 1, 1, 1,
    /* 1 */  2, 2, 1, 1, 2, 1, 1, 1,
    /* 2 */  2, 2, 1, 1, 1, 1, 2, 1,
    /* 3 */  2, 2, 1, 1, 2, 2, 1, 1,
    /* 4 */  2, 2, 2, 2, 1, 1, 2, 1,
    /* 5 */  2, 2, 1, 1, 1, 2, 2, 1,
    /* 6 */  2, 2, 1, 1, 2, 1, 1, 2,
    /* 7 */  2, 2, 2, 1, 2, 1, 1, 2,
    /* 8 */  2, 2, 2, 2, 2, 1, 2, 1,
    /* 9 */  1, 1, 1, 1, 1, 1, 2, 2,
    /* 10 */  2, 2, 1, 1, 1, 1, 2, 2,
    /* 11 */  1, 1, 1, 1, 2, 1, 2, 2,
    /* 12 */  2, 2, 1, 1, 2, 1, 2, 2,
    /* 13 */  2, 1, 1, 2, 2, 1, 2, 2,
    /* 14 */  2, 2, 1, 2, 2, 1, 2, 2,
    /* 15 */  2, 2, 2, 2, 2, 2, 2, 2,
    };
// Forward references
int bbepSetPixel2Clr(void *pb, int x, int y, unsigned char ucColor);
void bbepSetPixelFast2Clr(void *pb, int x, int y, unsigned char ucColor);
int bbepSetPanelSize(FASTEPDSTATE *pState, int width, int height, int flags, int iVCOM);
int bbepSetCustomMatrix(FASTEPDSTATE *pState, const uint8_t *pMatrix, size_t matrix_size);
//
// Pre-defined panels for popular products and boards
//
// width, height, bus_speed, flags, data[8], bus_width, ioPWR, ioSPV, ioCKV, ioSPH, ioOE, ioLE,
// ioCL, ioPWR_Good, ioSDA, ioSCL, ioShiftSTR/Wakeup, ioShiftMask/vcom, ioDCDummy, graymatrix, sizeof(graymatrix), iLinePadding
const BBPANELDEF panelDefs[] = {
    {0,0,0,0,{0},0,0,0,0,0,0,0,0,0,0,0,0,0,0,NULL,0,0,0}, // BB_PANEL_NONE
    {960, 540, 20000000, BB_PANEL_FLAG_NONE, {6,14,7,12,9,11,8,10}, 8, 46, 17, 18, 13, 45, 15,
      16, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, 47, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_M5PAPERS3

    {0, 0, 20000000, BB_PANEL_FLAG_NONE, {5,6,7,15,16,17,18,8}, 8, 11, 45, 48, 41, 8, 42,
      4, 14, 39, 40, BB_NOT_USED, 0, 0, u8SixInchMatrix, sizeof(u8SixInchMatrix), 16, -1600}, // BB_PANEL_EPDIY_V7

    {1024, 758, 13333333, BB_PANEL_FLAG_SLOW_SPH, {4,5,18,19,23,25,26,27}, 8, 4, 2, 32, 33, 0, 2,
      0, 7, 21, 22, 3, 5, 15, u8GrayMatrix, sizeof(u8GrayMatrix), 16, -1600}, // BB_PANEL_INKPLATE6PLUS

    {1280, 720, 13333333, BB_PANEL_FLAG_SLOW_SPH | BB_PANEL_FLAG_MIRROR_X, {4,5,18,19,23,25,26,27}, 8, 4, 2, 32, 33, 0, 2,
      0, 7, 21, 22, 3, 5, 0, u8Ink5V2Matrix, sizeof(u8Ink5V2Matrix), 16, -1600}, // BB_PANEL_INKPLATE5V2

    {0, 0, 20000000, BB_PANEL_FLAG_NONE, {9,10,11,12,13,14,21,47,5,6,7,15,16,17,18,8}, 16, 11, 45, 48, 41, 8, 42,
      4, 14, 39, 40, BB_NOT_USED, 0, 46, u8GrayMatrix, sizeof(u8GrayMatrix), 16, -1600}, // BB_PANEL_EPDIY_V7_16

    {0, 0, 26666666, BB_PANEL_FLAG_NONE, {5,6,7,15,16,17,18,8}, 8, 11, 45, 48, 41, 9, 42,
      4, 14, 39, 40, BB_NOT_USED, 0, 0, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_V7_RAW
    //                                             D8                 15 D0                  D7          STV,CKV,XSTL,OE,XLE
    {960, 540, 20000000, BB_PANEL_FLAG_SLOW_SPH, {11,12,13,14,21,47,45,38}, 8, BB_NOT_USED, BB_NOT_USED, 39, 9, 0, 0,
      10, 0, 2, 42, 1, 0, 46 /* LoRa CS */, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_LILYGO_T5PRO 
    {1440, 720, 40000000, BB_PANEL_FLAG_MIRROR_X, {27,28,29,30,31,32,33,34}, 8, BB_NOT_USED, 36, 13, 25, 0, 26,
      24, 0, 7, 8, 0, 0, 11 /* LED1_EN */, u8M5Matrix, sizeof(u8M5Matrix), 16, -1600}, // BB_PANEL_LILYGO_T5P4 
    {1872, 1404, 26666666, BB_PANEL_FLAG_MIRROR_X, {8,18,17,16,15,7,6,5,47,21,14,13,12,11,10,9}, 16, 11, 48, 45, 41, 8, 42,
      4, 14, 39, 40, BB_NOT_USED, 0, 46, u8GrayMatrix, sizeof(u8GrayMatrix), 16, -1100}, // BB_PANEL_TRMNL_X
{0, 0, 26666666, BB_PANEL_FLAG_NONE, {2,3,4,5,6,7,8,9}, 8, 26, 45, 51, 46, 47, 48,
      50, 27, 28, 29, 37, 0, 35, u8GrayMatrix, sizeof(u8GrayMatrix), 32, -1600}, // BB_PANEL_EPDINKY_P4
{0, 0, 26666666, BB_PANEL_FLAG_NONE, {10,11,12,13,14,15,16,17,2,3,4,5,6,7,8,9}, 16, 26, 45, 51, 46, 47, 48,
      50, 27, 28, 29, 37, 0, 35, u8GrayMatrix, sizeof(u8GrayMatrix), 16, -1600}, // BB_PANEL_EPDINKY_P4_16
};
//
// Forward references for panel callback functions
//
// LilyGo T5S3-Pro
int LilyGoEinkPower(void *pBBEP, int bOn);
int LilyGoIOInit(void *pBBEP);
void LilyGoRowControl(void *pBBEP, int iMode);
// M5Stack PaperS3
int PaperS3EinkPower(void *pBBEP, int bOn);
int PaperS3IOInit(void *pBBEP);
void PaperS3RowControl(void *pBBEP, int iMode);
// EPDiy V7
int EPDiyV7EinkPower(void *pBBEP, int bOn);
int EPDiyV7IOInit(void *pBBEP);
void EPDiyV7RowControl(void *pBBEP, int iMode);
void EPDiyV7IODeInit(void *pBBEP);
// EPDiy V7 RAW
int EPDiyV7RAWEinkPower(void *pBBEP, int bOn);
int EPDiyV7RAWIOInit(void *pBBEP);
// epdInky
int epdInkyEinkPower(void *pBBEP, int bOn);
int epdInkyIOInit(void *pBBEP);
// Inkplate6PLUS
int Inkplate6PlusEinkPower(void *pBBEP, int bOn);
int Inkplate6PlusIOInit(void *pBBEP);
void Inkplate6PlusRowControl(void *pBBEP, int iMode);
// Inkplate5V2
int Inkplate5V2EinkPower(void *pBBEP, int bOn);
int Inkplate5V2IOInit(void *pBBEP);
void Inkplate5V2RowControl(void *pBBEP, int iMode);
uint8_t EPDiyV7ExtIO(uint8_t iOp, uint8_t iPin, uint8_t iVal);
uint8_t Inkplate5V2ExtIO(uint8_t iOp, uint8_t iPin, uint8_t iVal);

// List of predefined callback functions for the panels supported by bb_epdiy
// BB_EINK_POWER, BB_IO_INIT, BB_ROW_CONTROL
const BBPANELPROCS panelProcs[] = {
    {NULL,NULL,NULL,NULL,NULL}, // BB_PANEL_NONE
    {PaperS3EinkPower, PaperS3IOInit, PaperS3RowControl, NULL, NULL}, // BB_PANEL_M5PAPERS3
    {EPDiyV7EinkPower, EPDiyV7IOInit, EPDiyV7RowControl, EPDiyV7IODeInit, EPDiyV7ExtIO}, // BB_PANEL_EPDIY_V7
    {Inkplate6PlusEinkPower, Inkplate6PlusIOInit, Inkplate6PlusRowControl, NULL, NULL}, // BB_PANEL_INKPLATE6PLUS
    {Inkplate5V2EinkPower, Inkplate5V2IOInit, Inkplate5V2RowControl, NULL, Inkplate5V2ExtIO}, // Inkplate5V2
    {EPDiyV7EinkPower, EPDiyV7IOInit, EPDiyV7RowControl, EPDiyV7IODeInit, EPDiyV7ExtIO}, // BB_PANEL_EPDIY_V7_16
    {EPDiyV7RAWEinkPower, EPDiyV7RAWIOInit, EPDiyV7RowControl, NULL, NULL}, // BB_PANEL_V7_RAW
    {LilyGoEinkPower, LilyGoIOInit, LilyGoRowControl, NULL, NULL},// BB_PANEL_LILYGO_T5PRO
    {EPDiyV7EinkPower, EPDiyV7IOInit, EPDiyV7RowControl, EPDiyV7IODeInit, EPDiyV7ExtIO}, // BB_PANEL_LILYGO_T5P4  
    {EPDiyV7EinkPower, EPDiyV7IOInit, EPDiyV7RowControl, EPDiyV7IODeInit, EPDiyV7ExtIO}, // BB_PANEL_TRMNL_X
    {epdInkyEinkPower, epdInkyIOInit, EPDiyV7RowControl, NULL, NULL}, // BB_PANEL_EPDINKY_P4
    {epdInkyEinkPower, epdInkyIOInit, EPDiyV7RowControl, NULL, NULL}, // BB_PANEL_EPDINKY_P4_16
};

uint8_t ioRegs[24]; // MCP23017 copy of I/O register state so that we can just write new bits
static uint16_t LUTW_16[256];
static uint16_t LUTB_16[256];
static uint16_t LUTBW_16[256];
// Lookup tables for grayscale mode
static uint8_t *pGrayLower = NULL, *pGrayUpper = NULL;
volatile bool dma_is_done = true;
static uint8_t u8Cache[1024]; // used also for masking a row of 2-bit codes, needs to handle up to 4096 pixels wide
static gpio_num_t u8CKV, u8SPH;
static uint8_t bSlowSPH = 0;

#ifdef ESP_PLATFORM
static bool s3_notify_dma_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{           
    if (bSlowSPH) {
        gpio_set_level(u8SPH, 1); // CS deactivate
    }
    dma_is_done = true;
    return false;
}           

// Maximum line width = 1024 * 4 = 4096 pixels
#define MAX_TX_SIZE 1024
static esp_lcd_i80_bus_config_t s3_bus_config = {
    .dc_gpio_num = 0,
    .wr_gpio_num = 0,
    .clk_src = /*LCD_CLK_SRC_DEFAULT,*/ LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    .bus_width = 0,
    .max_transfer_bytes = MAX_TX_SIZE, 
    .psram_trans_align = 0, // 0 = use default values
    .sram_trans_align = 0,
};
static esp_lcd_panel_io_i80_config_t s3_io_config = {
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

static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
#endif
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
// Set a specific pin's mode
//
void bbepPCA9535PinMode(uint8_t pin, uint8_t mode)
{
    uint8_t ucTemp[4];
    const uint8_t port = pin / 8;
    
    pin &= 7;
    if (mode == INPUT) {
        ioRegs[6 + port] |= (1 << pin);
    } else {
        ioRegs[6 + port] &= ~(1 << pin);
    }
    ucTemp[0] = 6 + port;
    ucTemp[1] = ioRegs[6 + port];
    bbepI2CWrite(0x20, ucTemp, 2); // update pin config
} /* bbepPCA9535PinMode() */

void bbepPCA9535DigitalWrite(uint8_t pin, uint8_t value)
{
    uint8_t ucTemp[4];
    const uint8_t port = pin / 8;
    
    pin &= 7;
    if (value) {
        ioRegs[2 + port] |= (1 << pin);
    } else {
        ioRegs[2 + port] &= ~(1 << pin);
    }
    ucTemp[0] = 2 + port;
    ucTemp[1] = ioRegs[2 + port];
    bbepI2CWrite(0x20, ucTemp, 2); // update pin state
} /* bbepPCA9535DigitalWrite() */

uint8_t bbepPCA9535DigitalRead(uint8_t pin)
{
    uint8_t uc, port = pin / 8;
    
    pin &= 7;
    bbepI2CReadRegister(0x20, port, &uc, 1);
    uc >>= (pin);
    return (uc & 1);
} /* bbepPCA9535DigitalRead() */

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
// Access to the IO extender
//
uint8_t EPDiyV7ExtIO(uint8_t iOp, uint8_t iPin, uint8_t iVal)
{
    uint8_t val = 0;
    if (iPin < 16) { // IO extenders only have 16 pins
        switch (iOp) {
            case BB_EXTIO_SET_MODE:
                bbepPCA9535PinMode(iPin, iVal);
                break;
            case BB_EXTIO_WRITE:
                bbepPCA9535DigitalWrite(iPin, iVal);
                break;
            case BB_EXTIO_READ:
                val = bbepPCA9535DigitalRead(iPin);
                break;
        }
    }
    return val;
} /* EPDiyV7ExtIO() */
//
// Access to the IO extender
//
uint8_t Inkplate5V2ExtIO(uint8_t iOp, uint8_t iPin, uint8_t iVal)
{
    uint8_t val = 0;
    if (iPin < 16) { // IO extenders only have 16 pins
        switch (iOp) {
            case BB_EXTIO_SET_MODE:
                bbepPCALPinMode(iPin, iVal);
                break;
            case BB_EXTIO_WRITE:
                bbepPCALDigitalWrite(iPin, iVal);
                break;
            case BB_EXTIO_READ:
                val = bbepPCALDigitalRead(iPin);
                break;
        }
    }
    return val;
} /* Inkplate5V2ExtIO() */

//
// Write 8 bits (_state.shift_data) to the shift register
//
void bbepSendShiftData(FASTEPDSTATE *pState)
{
    uint8_t uc = pState->shift_data;
//    Serial.printf("Sending shift data: 0x%02x\n", uc);
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
// LilyGo T5S3 Pro control functions
//
int LilyGoEinkPower(void *pBBEP, int bOn)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    if (bOn == pState->pwr_on) return BBEP_SUCCESS; // already on
    if (bOn) {
        pState->shift_data |= 0x80; // OE on
        bbepSendShiftData(pState);
        delayMicroseconds(100);
        pState->shift_data &= ~0x02; // !power_disable
        bbepSendShiftData(pState);
        delayMicroseconds(100);
        pState->shift_data |= 0x08; // neg_power_enable
        bbepSendShiftData(pState);
        delayMicroseconds(500);
        pState->shift_data |= 0x04; // pos_power_enable
        bbepSendShiftData(pState);
        delayMicroseconds(100);
        pState->shift_data |= 0x10; // stv = true
        bbepSendShiftData(pState);
        delayMicroseconds(100);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1);
        pState->pwr_on = 1;
    } else { // off
        pState->shift_data &= ~0x04; // pos_power_enable = false
        bbepSendShiftData(pState);
        delayMicroseconds(10);
        pState->shift_data &= ~0x08; // neg_power_enable = false
        bbepSendShiftData(pState);
        delayMicroseconds(100);
        pState->shift_data |= 0x02; // power_disable = true
        bbepSendShiftData(pState);
        delayMicroseconds(100);
        pState->shift_data &= ~0x80; // OE off
        bbepSendShiftData(pState);
        delayMicroseconds(100);
        pState->shift_data &= ~0x10; // stv = false
        bbepSendShiftData(pState);
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* LilyGoEinkPower() */

int LilyGoIOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP; 
    bbepPinMode(pState->panelDef.ioSDA, OUTPUT);
    bbepPinMode(pState->panelDef.ioSCL, OUTPUT);
    gpio_set_level((gpio_num_t)pState->panelDef.ioSCL, 0);
    bbepPinMode(pState->panelDef.ioShiftSTR, OUTPUT);
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 0);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    pState->shift_data = 0x2; // power disable
    pState->shift_data |= 0x20; // ep_scan_direction
    pState->shift_data |= 0x10; // ep_stv
    bbepSendShiftData(pState);
    return BBEP_SUCCESS;

} /* LilyGoIOInit() */

void LilyGoRowControl(void *pBBEP, int iMode)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    if (iMode == ROW_START) {
        pState->shift_data |= 0x40; // ep_mode = true
        bbepSendShiftData(pState);
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); // CKV on
        delayMicroseconds(7);
        pState->shift_data &= ~0x10; // spv = off
        bbepSendShiftData(pState);
        delayMicroseconds(10);
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); // CKV on
        delayMicroseconds(8);
        pState->shift_data |= 0x10; // spv = on
        bbepSendShiftData(pState); 
        delayMicroseconds(10);
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); // CKV on        
    } else if (iMode == ROW_STEP) {
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off
        pState->shift_data |= 0x01; // latch_enable = true
        bbepSendShiftData(pState);
        pState->shift_data &= ~0x01; // latch_enable = false
        bbepSendShiftData(pState);
    }
} /* LilyGoRowControl() */

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
#define TPS_REG_ENABLE 0x01
#define TPS_REG_UPSEQ0 0x9
#define TPS_REG_UPSEQ1 0xa
#define TPS_REG_PG     0x0F

int EPDiyV7EinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t ucTemp[4];
uint8_t u8Value = 0; // I/O bits for the PCA9535
int vcom;

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        //  u8Value |= 4; // STV on DEBUG - not sure why it's not used
        bbepPCA9535DigitalWrite(8, 1); // OE on
        bbepPCA9535DigitalWrite(9, 1); // GMOD on
        bbepPCA9535DigitalWrite(13, 1); // WAKEUP on
        bbepPCA9535DigitalWrite(11, 1); // PWRUP on
        bbepPCA9535DigitalWrite(12, 1); // VCOM CTRL on
        vTaskDelay(3); // allow time to power up
        while (!(bbepPCA9535DigitalRead(14))) { } // CFG_PIN_PWRGOOD

        ucTemp[0] = TPS_REG_UPSEQ0;
        ucTemp[1] = 0xe1;
        bbepI2CWrite(0x68, ucTemp, 2);

        ucTemp[0] = TPS_REG_UPSEQ1;
        ucTemp[1] = 0xaa;
        bbepI2CWrite(0x68, ucTemp, 2);

        ucTemp[0] = TPS_REG_ENABLE;
        ucTemp[1] = 0x3f; // enable output
        bbepI2CWrite(0x68, ucTemp, 2);
        // set VCOM (usually -1.6V = -1600mV = 160 value used in registers
        vcom = pState->iVCOM / -10; // convert to TPS format
        ucTemp[0] = 3; // vcom voltage register 3+4 = L + H
        ucTemp[1] = (uint8_t)vcom;
        ucTemp[2] = (uint8_t)(vcom >> 8);
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
        bbepPCA9535DigitalWrite(8, 0); // OE off
        bbepPCA9535DigitalWrite(9, 0); // GMOD off
        bbepPCA9535DigitalWrite(11, 0); // PWRUP off
        bbepPCA9535DigitalWrite(12, 0); // VCOM CTRL off
        vTaskDelay(1);
        bbepPCA9535DigitalWrite(13, 0); // WAKEUP off
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* EPDiyV7EinkPower() */

int EPDiyV7RAWEinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t ucTemp[4];
uint8_t u8Value = 0; // I/O bits for the PCA9535
int vcom;

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        //  u8Value |= 4; // STV on DEBUG - not sure why it's not used
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 1); // OE on
        gpio_set_level((gpio_num_t)10, 1); // EP_MODE/GMOD on
        gpio_set_level((gpio_num_t)14, 1); // WAKEUP on
        gpio_set_level((gpio_num_t)11, 1); // PWRUP on
        gpio_set_level((gpio_num_t)12, 1); // VCOM CTRL on
        vTaskDelay(1); // allow time to power up
      //  while (!gpio_get_level((gpio_num_t)47)) { }
        ucTemp[0] = TPS_REG_ENABLE;
        ucTemp[1] = 0x3f; // enable output
        bbepI2CWrite(0x68, ucTemp, 2);
        // set VCOM
        vcom = pState->iVCOM / -10; // convert to TPS format
        ucTemp[0] = 3; // vcom voltage register 3+4 = L + H
        ucTemp[1] = (uint8_t)(vcom);
        ucTemp[2] = (uint8_t)(vcom >> 8);
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
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 0); // OE off
        gpio_set_level((gpio_num_t)10, 0); // EP_MODE/GMOD off
        gpio_set_level((gpio_num_t)12, 0); // VCOM CTRL off
        gpio_set_level((gpio_num_t)11, 0); // PWRUP off
        gpio_set_level((gpio_num_t)14, 0); // WAKEUP off
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* EPDiyV7RAWEinkPower() */

int epdInkyEinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t ucTemp[4];
uint8_t u8Value = 0; // I/O bits for the PCA9535
int vcom;

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        //  u8Value |= 4; // STV on DEBUG - not sure why it's not used
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 1); // OE on
        gpio_set_level((gpio_num_t)52, 1); // EP_MODE/GMOD on
        gpio_set_level((gpio_num_t)37, 1); // WAKEUP on
        gpio_set_level((gpio_num_t)26, 1); // PWRUP on
        gpio_set_level((gpio_num_t)49, 1); // VCOM CTRL on
        vTaskDelay(3); // allow time to power up
//        ucTemp[0] = TPS_REG_UPSEQ0;
//        ucTemp[1] = 0xe1;
//        bbepI2CWrite(0x68, ucTemp, 2);

//        ucTemp[0] = TPS_REG_UPSEQ1;
//        ucTemp[1] = 0xaa;
//        bbepI2CWrite(0x68, ucTemp, 2);

        ucTemp[0] = TPS_REG_ENABLE;
        ucTemp[1] = 0x3f; // enable output
        bbepI2CWrite(0x68, ucTemp, 2);
        // set VCOM (usually -1.6v (-1600))
        vcom = pState->iVCOM / -10; // convert to TPS format
        ucTemp[0] = 3; // vcom voltage register 3+4 = L + H
        ucTemp[1] = (uint8_t)vcom;
        ucTemp[2] = (uint8_t)(vcom >> 8);
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
        gpio_set_level((gpio_num_t)pState->panelDef.ioOE, 0); // OE off
        gpio_set_level((gpio_num_t)52, 0); // EP_MODE/GMOD off
        gpio_set_level((gpio_num_t)49, 0); // VCOM CTRL off
        gpio_set_level((gpio_num_t)26, 0); // PWRUP off
        gpio_set_level((gpio_num_t)37, 0); // WAKEUP off
        pState->pwr_on = 0;
    }
    return BBEP_SUCCESS;
} /* epdInkyEinkPower() */

int Inkplate6PlusEinkPower(void *pBBEP, int bOn)
{
FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
uint8_t ucTemp[4];

    if (bOn == pState->pwr_on) return BBEP_SUCCESS;
    if (bOn) {
        bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP on;
        delay(5);
        ucTemp[0] = 0x09;
        ucTemp[1] = 0xe1;
        bbepI2CWrite(0x48, ucTemp, 2);
        // Enable all rails
        ucTemp[0] = 0x01;
        ucTemp[1] = 0x3f;
        bbepI2CWrite(0x48, ucTemp, 2);
        bbepMCPDigitalWrite(pState->panelDef.ioPWR, 1); // PWRUP on;
        //pinsAsOutputs();
        gpio_set_level((gpio_num_t)pState->panelDef.ioLE, 0); // LE off;
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 0); // OE off;
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1); // SPH on;
        bbepMCPDigitalWrite(1, 1); // GMOD on
        bbepMCPDigitalWrite((uint8_t)pState->panelDef.ioSPV, 1); // SPV on;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off;
        bbepMCPDigitalWrite(pState->panelDef.ioShiftMask, 1); // VCOM on;
        unsigned long timer = millis();
        do {
            delay(1);
        } while (!bbepReadPowerGood() && (millis() - timer) < 250);
        if ((millis() - timer) >= 250) {
            bbepMCPDigitalWrite(pState->panelDef.ioShiftMask, 0); // VCOM off;
            bbepMCPDigitalWrite(pState->panelDef.ioPWR, 0); // PWR off;
            return BBEP_IO_ERROR;
        }
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 1); // OE on;
        pState->pwr_on = 1;
    } else { // power off
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 0); // OE off;
        bbepMCPDigitalWrite(1, 0); // GMODE off;
        bbepMCPDigitalWrite(pState->panelDef.ioLE, 0); // LE off;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 0); //SPH off;
        bbepMCPDigitalWrite(pState->panelDef.ioSPV, 0); //SPV off;
        bbepMCPDigitalWrite(pState->panelDef.ioShiftMask, 0); // VCOM off;
        bbepMCPDigitalWrite(pState->panelDef.ioPWR, 0); // PWR off;
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
        bbepPCALDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP on;
        delay(5);
        ucTemp[0] = 0x09;
        ucTemp[1] = 0xe1;
        bbepI2CWrite(0x48, ucTemp, 2);
        // Enable all rails
        ucTemp[0] = 0x01;
        ucTemp[1] = 0x3f;
        bbepI2CWrite(0x48, ucTemp, 2);
        bbepPCALDigitalWrite(pState->panelDef.ioPWR, 1); // PWR on;
        //pinsAsOutputs();
        gpio_set_level((gpio_num_t)pState->panelDef.ioLE, 0); // LE off;
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 0); // OE off;
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 1); // SPH on;
        bbepPCALDigitalWrite(1, 1); // GMODE on
        bbepPCALDigitalWrite((uint8_t)pState->panelDef.ioSPV, 1); // SPV on;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off;
        bbepPCALDigitalWrite(pState->panelDef.ioShiftMask, 1); // VCOM on;
        unsigned long timer = millis();
        do {
            delay(1);
        } while (!bbepReadPowerGood() && (millis() - timer) < 250);
        if ((millis() - timer) >= 250) {
            bbepPCALDigitalWrite(pState->panelDef.ioShiftMask, 0); // VCOM off;
            bbepPCALDigitalWrite(pState->panelDef.ioPWR, 0); // PWR off;
            return BBEP_IO_ERROR;
        }
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 1); // OE on;
        pState->pwr_on = 1;
    } else { // power off
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 0); // OE off;
        bbepPCALDigitalWrite(1, 0); // GMODE off;
        bbepPCALDigitalWrite(pState->panelDef.ioLE, 0); // LE off;
        gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 0); // CKV off
        gpio_set_level((gpio_num_t)pState->panelDef.ioSPH, 0); // SPH off;
        bbepPCALDigitalWrite(pState->panelDef.ioSPV, 0); // SPV off;
        bbepPCALDigitalWrite(pState->panelDef.ioShiftMask, 0); // VCOM off;
        bbepPCALDigitalWrite(pState->panelDef.ioPWR, 0); // PWR off;
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
// Shut down the IO to save power (EPDiy V7 PCB)
//
void EPDiyV7IODeInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepPinMode(pState->panelDef.ioSPV, INPUT); // set I/O to high impedance
    bbepPinMode(pState->panelDef.ioCKV, INPUT);
    bbepPinMode(pState->panelDef.ioSPH, INPUT);
//    if (pState->panelDef.ioOE < 0x100) bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, INPUT);
    bbepPinMode(pState->panelDef.ioCL, INPUT);
    bbepPCA9535DigitalWrite(13, 1); // turn TPS65185 WAKEUP off
} /* EPDiyV7IODeInit() */
//
// Initialize the IO for the EPDiy V7 PCB
//
int EPDiyV7IOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
//    if (pState->panelDef.ioPWR < 0x100) bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
//    if (pState->panelDef.ioOE < 0x100) bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL, pState->bit_bang);
    memset(ioRegs, 0, sizeof(ioRegs)); // copy of IO expander registers
    for (int i=8; i<14; i++) { // set lower 6 bits as outputs
        bbepPCA9535PinMode(i, OUTPUT);
    }
    bbepPCA9535PinMode(14, INPUT); // TPS_PWR_GOOD
    bbepPCA9535PinMode(15, INPUT); // TPS_nINT
    return BBEP_SUCCESS;
} /* EPDiyV7IOInit() */
//
// Initialize the IO for the V7 RAW PCB
//
int EPDiyV7RAWIOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepPinMode(10, OUTPUT); // EP_MODE
    bbepPinMode(11, OUTPUT); // TPS_PWRUP
    bbepPinMode(12, OUTPUT); // TPS_VCOM_CTRL
    bbepPinMode(14, OUTPUT); // TPS_WAKEUP
    bbepPinMode(47, INPUT); // TPS_POWER_GOOD
    return BBEP_SUCCESS;
} /* EPDiyV7RAWIOInit() */

//
// Initialize the IO for the epdInky P4 PCB
//  
int epdInkyIOInit(void *pBBEP)
{       
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL, pState->bit_bang);
    bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepPinMode(52, OUTPUT); // EP_MODE
    bbepPinMode(26, OUTPUT); // TPS_PWRUP
    bbepPinMode(49, OUTPUT); // TPS_VCOM_CTRL
    bbepPinMode(37, OUTPUT); // TPS_WAKEUP
    bbepPinMode(27, INPUT); // TPS_POWER_GOOD
    return BBEP_SUCCESS;
} /* epdInkyIOInit() */

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
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL, pState->bit_bang);
    bbepMCP23017Init();
    bbepMCPPinMode(pState->panelDef.ioShiftMask, OUTPUT); // VCOM
    bbepMCPPinMode(pState->panelDef.ioPWR, OUTPUT); // PWRUP
    bbepMCPPinMode(pState->panelDef.ioShiftSTR, OUTPUT); // WAKEUP
    bbepMCPPinMode(8, OUTPUT);
    bbepMCPDigitalWrite(8, HIGH);
    bbepMCPPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepMCPPinMode(1, OUTPUT); // GMODE
    bbepMCPPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP on
    delay(1);
    bbepTPS65186Init(pState);
    delay(1);
    bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 0); // WAKEUP off
    return BBEP_SUCCESS;
} /* Inkplate6PlusIOInit() */

int Inkplate5V2IOInit(void *pBBEP)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL, pState->bit_bang);
    bbepPCAL6416Init();
    bbepPCALDigitalWrite(9, LOW);
    bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    bbepPCALPinMode(pState->panelDef.ioShiftMask, OUTPUT); // VCOM
    bbepPCALPinMode(pState->panelDef.ioPWR, OUTPUT); // PWRUP
    bbepPCALPinMode(pState->panelDef.ioShiftSTR, OUTPUT); // WAKEUP
    bbepPCALPinMode(8, OUTPUT);
    bbepPCALDigitalWrite(8, HIGH);
    bbepPCALPinMode(pState->panelDef.ioOE, OUTPUT);
    bbepPCALPinMode(1, OUTPUT); // GMODE
    bbepPCALPinMode(pState->panelDef.ioSPV, OUTPUT);
    bbepPCALDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP on
    delay(1);
    bbepTPS65186Init(pState);
    delay(1);
    bbepPCALDigitalWrite(pState->panelDef.ioShiftSTR, 0); // WAKEUP off
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
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(7);
        gpio_set_level(spv, 0); // SPV off
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(8);                    
        gpio_set_level(spv, 1); // SPV on
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); // CKV off
        gpio_set_level(le, 1); // LE toggle
        gpio_set_level(le, 0);
        delayMicroseconds(0);
    }
} /* PaperS3RowControl() */

void EPDiyV7RowControl(void *pBBEP, int iType)
{
    FASTEPDSTATE *pState = (FASTEPDSTATE *)pBBEP;
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(7);
        gpio_set_level(spv, 0); // SPV off
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(8);                    
        gpio_set_level(spv, 1); // SPV on
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); // CKV off
        gpio_set_level(le, 1); // LE toggle
        gpio_set_level(le, 0);
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
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(7);
        bbepMCPDigitalWrite(spv, 0);
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(8);                    
        bbepMCPDigitalWrite(spv, 1); // SPV on
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        bbepMCPDigitalWrite(pState->panelDef.ioOE, 1);
        gpio_set_level(ckv, 1); // CKV on
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); // CKV off
        gpio_set_level(le, 1); // LE toggle
        gpio_set_level(le, 0);
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
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(7);
        bbepPCALDigitalWrite(spv, 0);
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(8);                    
        bbepPCALDigitalWrite(spv, 1); // SPV on
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); // CKV on
        delayMicroseconds(18);
        gpio_set_level(ckv, 0); // CKV off
        delayMicroseconds(0);
        bbepPCALDigitalWrite(pState->panelDef.ioOE, 1); // OE on
        gpio_set_level(ckv, 1); // CKV on
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); // CKV off
        gpio_set_level(le, 1); // LE toggle
        gpio_set_level(le, 0);
        delayMicroseconds(0);
    }
}
void bbepRowControl(FASTEPDSTATE *pState, int iType)
{
    (*(pState->pfnRowControl))(pState, iType);
    return;
} /* bbepRowControl() */

// The data needs to come from a DMA buffer or the Espressif DMA driver
// will allocate (and leak) an internal buffer each time
void bbepWriteRow(FASTEPDSTATE *pState, uint8_t *pData, int iLen, int bRowStep)
{
   #ifdef ESP_PLATFORM
    esp_err_t err;

    while (!dma_is_done) {
        delayMicroseconds(1);
    }
    if (bRowStep) {
        bbepRowControl(pState, ROW_STEP);
    }
    if (bSlowSPH) {
        gpio_set_level(u8SPH, 0); // SPH/CS active
//        gpio_set_level(u8CKV, 1); // CKV on
    }
    dma_is_done = false;
    gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); // CKV on
    err = esp_lcd_panel_io_tx_color(io_handle, -1, pData, iLen + pState->panelDef.iLinePadding);
    if (err != ESP_OK) {
     //   Serial.printf("Error %d sending LCD data\n", (int)err);
    }
//    while (!dma_is_done) {
//        delayMicroseconds(1);
//        vTaskDelay(0);
//    }
#endif
} /* bbepWriteRow() */

uint8_t TPS65185PowerGood(void)
{
uint8_t ucTemp[4];

    bbepI2CReadRegister(0x68, 0x0f, ucTemp, 1);
    return ucTemp[0];
}
//
// Initialize the board-specific I/O components (IOInit callback)
// and then initialize the ESP32 LCD API to drive the parallel data bus
//
int bbepIOInit(FASTEPDSTATE *pState)
{
    #ifdef ESP_PLATFORM
    #ifndef ARDUINO
        esp_log_level_set("gpio", ESP_LOG_NONE);
    #endif
    int rc = (*(pState->pfnIOInit))(pState);
    if (rc != BBEP_SUCCESS) return rc;
    pState->iPartialPasses = 4; // N.B. The default number of passes for partial updates
    pState->iFullPasses = 5; // the default number of passes for smooth and full updates
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
    dma_is_done = true;
    //Serial.println("IO init done");
    #endif
    return BBEP_SUCCESS;
} /* bbepIOInit() */
//
// For displays with pre-defined configurations (size, speed, flags, gray matrix)
//
int bbepSetDefinedPanel(FASTEPDSTATE *pState, int iPanel)
{
    if (!pState) return BBEP_ERROR_BAD_PARAMETER;
    if (iPanel < 0 || iPanel >= BBEP_DISPLAY_COUNT) return BBEP_ERROR_BAD_PARAMETER;
    switch (iPanel) {
        case BBEP_DISPLAY_EC060TC1:
            bbepSetPanelSize(pState, 1024, 758, BB_PANEL_FLAG_NONE, -1600);
            bbepSetCustomMatrix(pState, u8SixInchMatrix, sizeof(u8SixInchMatrix));
            break;
        case BBEP_DISPLAY_EC060KD1:
            bbepSetPanelSize(pState, 1448, 1072, BB_PANEL_FLAG_NONE, -1600);
            bbepSetCustomMatrix(pState, u8SixInchMatrix, sizeof(u8SixInchMatrix));
            break;
        case BBEP_DISPLAY_ED0970TC1:
            bbepSetPanelSize(pState, 1280, 825, BB_PANEL_FLAG_NONE, -1600);
            bbepSetCustomMatrix(pState, u8NineInchMatrix, sizeof(u8NineInchMatrix));
            break;
        case BBEP_DISPLAY_ED103TC2:
            bbepSetPanelSize(pState, 1872, 1414, BB_PANEL_FLAG_MIRROR_X, -1600);
            bbepSetCustomMatrix(pState, u8TenPointThreeMatrix, sizeof(u8TenPointThreeMatrix));
            break;
        case BBEP_DISPLAY_ED052TC4:
            bbepSetPanelSize(pState, 1280, 720, BB_PANEL_FLAG_MIRROR_X, -1600);
            bbepSetCustomMatrix(pState, u8FivePointTwoMatrix, sizeof(u8FivePointTwoMatrix));
            break;
        case BBEP_DISPLAY_ED1150C1:
            bbepSetPanelSize(pState, 2760, 2070, BB_PANEL_FLAG_NONE, -1000);
            bbepSetCustomMatrix(pState, u8FivePointTwoMatrix, sizeof(u8FivePointTwoMatrix));
            break;
    } // switch on panel
    return BBEP_SUCCESS;
} /* bbepSetDefinedPanel() */
//
// For board definitions without an associated display (e.g. EPDiy V7 PCB)
// Set the display size and flags
//
int bbepSetPanelSize(FASTEPDSTATE *pState, int width, int height, int flags, int iVCOM) {
    int iPasses;
    uint8_t *pMatrix;

    if (pState->pCurrent) return BBEP_ERROR_BAD_PARAMETER; // panel size is already set

    pState->width = pState->native_width = width;
    pState->height = pState->native_height = height;
    pState->iFlags = flags;
    pState->iVCOM = iVCOM;
    pState->pCurrent = (uint8_t *)heap_caps_aligned_alloc(16, (pState->width * pState->height) / 2, MALLOC_CAP_SPIRAM); // current pixels
    if (!pState->pCurrent) return BBEP_ERROR_NO_MEMORY;
    if (pState->iPanelType == BB_PANEL_VIRTUAL) {
        return BBEP_SUCCESS; // for graphics only
    }
    pState->pPrevious = &pState->pCurrent[(width/4) * height]; // comparison with previous buffer (only 1-bpp mode)
    pState->pTemp = (uint8_t *)heap_caps_aligned_alloc(16, (pState->width * pState->height) / 4, MALLOC_CAP_SPIRAM); // LUT data
    if (!pState->pTemp) {
        free(pState->pCurrent);
        return BBEP_ERROR_NO_MEMORY;
    }
    // Allocate memory for each line to transmit
    pState->dma_buf = (uint8_t *)heap_caps_malloc((pState->width / 2) + pState->panelDef.iLinePadding + 16, MALLOC_CAP_DMA);
    iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
    pGrayLower = (uint8_t *)malloc(256 * iPasses);
    if (!pGrayLower) return BBEP_ERROR_NO_MEMORY;
    pGrayUpper = (uint8_t *)malloc(256 * iPasses);
    if (!pGrayUpper) {
        free(pGrayLower);
        return BBEP_ERROR_NO_MEMORY;
    }
    //Serial.printf("passes = %d\n", iPasses);
    // Prepare grayscale lookup tables
    pMatrix = (uint8_t *)pState->panelDef.pGrayMatrix;
    for (int j = 0; j < iPasses; j++) {
        for (int i = 0; i < 256; i++) {
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                pGrayLower[j * 256 + i] = (pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j])) << 4;
            } else {
                pGrayLower[j * 256 + i] = (pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j])) << 4;
            }
        } // for i
    } // for j
    // Create the lookup tables for 1-bit mode. Allow for inverted and mirrored
    for (int i=0; i<256; i++) {
        uint16_t b, w, bw, u16W, u16B, u16BW;
        u16W = u16B = u16BW = 0;
        for (int j=0; j<8; j++) {
            // a 1 means do nothing and 0 means move towards black or white (depending on the LUT)
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                if (!(i & (1<<(7-j)))) {
                    w = 2; b = 1;
                    bw = 1;
                } else {
                    w = 3; b = 3;
                    bw = 2;
                }
            } else {
                if (!(i & (0x80>>(7-j)))) {
                    w = 2; b = 1;
                    bw = 1;
                } else {
                    w = 3; b = 3;
                    bw = 2;
                }
            }
            u16W |= (w << (j * 2));
            u16B |= (b << (j * 2));
            u16BW |= (bw << (j * 2));
        } // for j
        LUTW_16[i] = __builtin_bswap16(u16W);
        LUTB_16[i] = __builtin_bswap16(u16B);
        LUTBW_16[i] = __builtin_bswap16(u16BW);
    } // for i
return BBEP_SUCCESS;
} /* setPanelSize() */

//
// Set the individual brightness of the 1 or 2 front lights
//
void bbepSetBrightness(FASTEPDSTATE *pState, uint8_t led1, uint8_t led2)
{
#ifdef ARDUINO
#if (ESP_IDF_VERSION_MAJOR > 4)
    ledcWrite(pState->u8LED1, led1); // PWM (0-255)
    if (pState->u8LED2 != 0xff) {
        ledcWrite(pState->u8LED2, led2);
    }
#else // old API
    ledcWrite(0, led1);
    if (pState->u8LED2 != 0xff) {
        ledcWrite(1, led2);
    }   
#endif
#else // disabled on esp-idf
    (void)pState; (void)led1; (void)led2;
#endif
} /* bbepSetBrightness() */

//
// Initialize the front light(s) if present
//
void bbepInitLights(FASTEPDSTATE *pState, uint8_t led1, uint8_t led2)
{
    pState->u8LED1 = led1;
    pState->u8LED2 = led2;
#ifdef ARDUINO // Arduino-only for now
#if (ESP_IDF_VERSION_MAJOR > 4)
    ledcAttach(led1, 5000, 8); // attach pin to channel 0
    ledcWrite(led1, 0); // set to off to start
    if (led2 != 0xff) {
        ledcAttach(led2, 5000, 8);
        ledcWrite(led2, 0); // set to off
    }
#else // old API
    ledcSetup(0, 5000, 8);
    ledcAttachPin(led1, 0);
    ledcWrite(0, 0);
    if (led2 != 0xff) {
        ledcSetup(1, 5000, 8);
        ledcAttachPin(led2, 1);
        ledcWrite(1, 0);
    }
#endif
#endif // ARDUINO
} /* bbepInitLights() */

//
// Initialize the panel based on the constant name
// Each name points to a configuration with info about the PCB and possibly a display
// e.g. BB_PANEL_M5PAPERs3 has both PCB and display info in a single configuration
//
int bbepInitPanel(FASTEPDSTATE *pState, int iPanel, uint32_t u32Speed)
{
    int rc;
    if (iPanel > 0 && iPanel < BB_PANEL_COUNT) {
        pState->iPanelType = iPanel;
        pState->mode = BB_MODE_1BPP; // start in 1-bit mode
        pState->iFG = BBEP_BLACK;
        pState->iBG = BBEP_TRANSPARENT;
        pState->iVCOM = -1600; // assume VCOM is -1.6V (typical)
        pState->pfnSetPixel = bbepSetPixel2Clr;
        pState->pfnSetPixelFast = bbepSetPixelFast2Clr;
        pState->pCurrent = NULL; // make sure the memory is allocated
        if (iPanel == BB_PANEL_VIRTUAL) {
            pState->pfnExtIO = NULL;
            pState->pfnEinkPower = NULL;
            pState->pfnRowControl = NULL;
            return BBEP_SUCCESS; 
        }
        pState->width = pState->native_width = panelDefs[iPanel].width;
        pState->height = pState->native_height = panelDefs[iPanel].height;
        memcpy(&pState->panelDef, &panelDefs[iPanel], sizeof(BBPANELDEF));
        if (u32Speed) pState->panelDef.bus_speed = u32Speed; // custom speed
        pState->iFlags = pState->panelDef.flags; // copy flags to main class structure
        // Get the 5 callback functions
        pState->pfnEinkPower = panelProcs[iPanel].pfnEinkPower;
        pState->pfnIOInit = panelProcs[iPanel].pfnIOInit;
        pState->pfnIODeInit = panelProcs[iPanel].pfnIODeInit;
        pState->pfnRowControl = panelProcs[iPanel].pfnRowControl;
        pState->pfnExtIO = panelProcs[iPanel].pfnExtIO;
        rc = bbepIOInit(pState);
        if (rc == BBEP_SUCCESS) {
            // allocate memory for the buffers if the paneldef contains the size
            if (pState->width) { // if size is defined
                // VCOM is defined for predefined products
                pState->iVCOM = pState->panelDef.iVCOM;
                rc = bbepSetPanelSize(pState, pState->width, pState->height, pState->iFlags, pState->iVCOM);
                if (rc != BBEP_SUCCESS) return rc; // no memory? stop
            }
        }
        return rc;
    }
    return BBEP_ERROR_BAD_PARAMETER;
} /* bbepInitPanel() */

//
// Allow the user to set up a custom grayscale matrix
// The number of passes is determined by dividing the table size by 16
//
int bbepSetCustomMatrix(FASTEPDSTATE *pState, const uint8_t *pMatrix, size_t matrix_size)
{
int iPasses;

    if (pState == NULL || pMatrix == NULL) return BBEP_ERROR_BAD_PARAMETER;
    if ((matrix_size & 15) != 0) return BBEP_ERROR_BAD_PARAMETER; // must be divisible by 16
    if (pGrayLower) free(pGrayLower);
    if (pGrayUpper) free(pGrayUpper);
    iPasses = (int)matrix_size / 16; // number of passes
    pState->panelDef.pGrayMatrix = (uint8_t *)pMatrix;
    pState->panelDef.iMatrixSize = matrix_size;
    pGrayLower = (uint8_t *)malloc(256 * iPasses);
    if (!pGrayLower) return BBEP_ERROR_NO_MEMORY;
    pGrayUpper = (uint8_t *)malloc(256 * iPasses);
        if (!pGrayUpper) {
            free(pGrayLower);
            return BBEP_ERROR_NO_MEMORY;
        }
    // Prepare grayscale lookup tables
    for (int j = 0; j < iPasses; j++) {
        for (int i = 0; i < 256; i++) {
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                pGrayLower[j * 256 + i] = (pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i & 0xf)*iPasses)+j] << 2) | (pMatrix[((i >> 4)*iPasses)+j])) << 4;
            } else {
                pGrayLower[j * 256 + i] = (pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j]);
                pGrayUpper[j * 256 + i] = ((pMatrix[((i >> 4)*iPasses)+j] << 2) | (pMatrix[((i & 0xf)*iPasses)+j])) << 4;
            }
        }
    }
    return BBEP_SUCCESS;
} /* () */
//
// Turn the DC/DC boost circuit on or off
//
int bbepEinkPower(FASTEPDSTATE *pState, int bOn)
{
    if (!pState->pfnEinkPower) return BBEP_ERROR_BAD_PARAMETER;

    return (*(pState->pfnEinkPower))(pState, bOn);
} /* bbepEinkPower() */
//
// Fix a rectangle's coordinates for the current rotation and mirroring flags
// returns FALSE (0) if okay, TRUE (1) if invalid
//
int bbepFixRect(FASTEPDSTATE *pState, BB_RECT *pRect, int *iStartCol, int *iEndCol, int *iStartRow, int *iEndRow)
{
    int i;
        *iStartCol = pRect->x;
        *iEndCol = *iStartCol + pRect->w - 1;
        *iStartRow = pRect->y;
        *iEndRow = *iStartRow + pRect->h - 1;
        if (*iStartCol >= *iEndCol || *iStartRow >= *iEndRow) return 1; // invalid area

        if (*iStartCol < 0) *iStartCol = 0;
        if (*iStartRow < 0) *iStartRow = 0;
        if (*iEndCol >= pState->width) *iEndCol = pState->width - 1;
        if (*iEndRow >= pState->height) *iEndRow = pState->height - 1;
        switch (pState->rotation) { // rotate to native panel direction
            case 0: // nothing to do
                break;
            case 90:
                i = *iStartCol;
                *iStartCol = *iStartRow;
                *iStartRow = pState->width - 1 - *iEndCol;
                *iEndCol = *iEndRow;
                *iEndRow = pState->width - 1 - i; // iStartCol
                break;
            case 270:
                i = *iStartCol;
                *iStartCol = pState->height - 1 - *iEndRow;
                *iEndRow = *iEndCol;
                *iEndCol = pState->height - 1 - *iStartRow;
                *iStartRow = i; // iStartCol
                break;
            case 180:
                i = *iStartCol;
                *iStartCol = pState->width - 1 - *iEndCol;
                *iEndCol = pState->width - 1 - i;
                i = *iStartRow;
                *iStartRow = pState->height - 1 - *iEndRow;
                *iEndRow = pState->height - 1 - i;
                break;
        }
    return 0;
} /* bbepFixRect() */

//
// Clear the display with the given code for the given number of repetitions
//
void bbepClear(FASTEPDSTATE *pState, uint8_t val, uint8_t count, BB_RECT *pRect)
{
    uint8_t u8;
    int i, k, dy, iStartCol, iEndCol, iStartRow, iEndRow; // clipping area
    if (val == BB_CLEAR_LIGHTEN) val = 0xaa;
    else if (val == BB_CLEAR_DARKEN) val = 0x55;
    else if (val == BB_CLEAR_NEUTRAL) val = 0x00;
    else val = 0xff; // skip

    if (pRect) {
        if (bbepFixRect(pState, pRect, &iStartCol, &iEndCol, &iStartRow, &iEndRow)) return;
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
            dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
            // Send the data
            if (dy < iStartRow || dy > iEndRow) { // skip this row
                memset(pState->dma_buf, 0xff, pState->native_width / 4);
            } else { // mask the area we want to change
                memcpy(pState->dma_buf, u8Cache, pState->native_width / 4);
            }
            bbepWriteRow(pState, pState->dma_buf, pState->native_width / 4, (i!=0));
            //bbepRowControl(pState, ROW_STEP);
        }
        delayMicroseconds(230);
    }
} /* bbepClear() */
//
// Perform a full update with a single color transition (user selected)
// This allows for a faster and less visually disturbing display that is also faster
// than a normal full update
//
int bbepSmoothUpdate(FASTEPDSTATE *pState, bool bKeepOn, uint8_t u8Color)
{
    int i, n, pass;
    
    if (pState->iPanelType == BB_PANEL_VIRTUAL) return BBEP_ERROR_BAD_PARAMETER;

    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    bbepClear(pState, (u8Color == BBEP_WHITE) ? BB_CLEAR_LIGHTEN : BB_CLEAR_DARKEN, 5, NULL);
    // The other update methods transition everything from white. In this case, we
    // need to allow the user to update from black also
    if (pState->mode == BB_MODE_1BPP) {
        // Set the color in multiple passes starting from white or black
        // First create the 2-bit codes per pixel for the changes
        uint8_t *s, *d;
        int dy; // destination Y for flipped displays
        uint8_t u8Invert = (u8Color == BBEP_WHITE) ? 0x00 : 0xff;
        uint16_t u16Invert = (u8Color == BBEP_WHITE) ? 0x00 : 0xffff;
        for (i = 0; i < pState->native_height; i++) {
            dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
            s = &pState->pCurrent[i * (pState->native_width/8)];
            d = &pState->pTemp[dy * (pState->native_width/4)];
            memcpy(&pState->pPrevious[i * (pState->native_width/8)], s, pState->native_width / 8); // previous = current
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                s += (pState->native_width/8) - 1;
                for (n = 0; n < (pState->native_width / 4); n += 4) {
                    uint8_t dram2 = *s--;
                    uint8_t dram1 = *s--;
                    *(uint16_t *)&d[n] = u16Invert ^ LUTB_16[dram2 ^ u8Invert];
                    *(uint16_t *)&d[n+2] = u16Invert ^ LUTB_16[dram1 ^ u8Invert];
                }
            } else {
                for (n = 0; n < (pState->native_width / 4); n += 4) {
                    uint8_t dram1 = *s++;
                    uint8_t dram2 = *s++;
                    *(uint16_t *)&d[n+2] = u16Invert ^ LUTB_16[dram2 ^ u8Invert];
                    *(uint16_t *)&d[n] = u16Invert ^ LUTB_16[dram1 ^ u8Invert];
                }
            }
        } // for i
        // Write N passes of the black data to the whole display
        for (pass = 0; pass < pState->iFullPasses; pass++) {
            bbepRowControl(pState, ROW_START);
            for (i = 0; i < pState->native_height; i++) {
                s = &pState->pTemp[i * (pState->native_width / 4)];
                // Send the data for the row
                memcpy(pState->dma_buf, s, pState->native_width/4);
                bbepWriteRow(pState, pState->dma_buf, (pState->native_width / 4), 0);
                bbepRowControl(pState, ROW_STEP);
            }
            delayMicroseconds(230);
        } // for pass
    } else { // must be 4BPP mode
        int dy, iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
        uint8_t u8Invert = (u8Color = BBEP_WHITE) ? 0x00 : 0xff;
        for (pass = 0; pass < iPasses; pass++) { // number of passes to make 16 unique gray levels
            uint8_t *s, *d = pState->dma_buf;
            uint8_t *pGrayU, *pGrayL;
            pGrayU = pGrayUpper + (pass * 256);
            pGrayL = pGrayLower + (pass * 256);
            bbepRowControl(pState, ROW_START);
            for (i = 0; i < pState->native_height; i++) {
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                s = &pState->pCurrent[dy * (pState->native_width / 2)];
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s += (pState->native_width / 2) - 8;
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        d[n + 0] = u8Invert ^ (pGrayU[(s[7] ^ u8Invert)] | pGrayL[(s[6] ^ u8Invert)]);
                        d[n + 1] = u8Invert ^ (pGrayU[(s[5] ^ u8Invert)] | pGrayL[(s[4] ^ u8Invert)]);
                        d[n + 2] = u8Invert ^ (pGrayU[(s[3] ^ u8Invert)] | pGrayL[(s[2] ^ u8Invert)]);
                        d[n + 3] = u8Invert ^ (pGrayU[(s[1] ^ u8Invert)] | pGrayL[(s[0] ^ u8Invert)]);
                        s -= 8;
                    } // for n
                } else {
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        d[n + 0] = u8Invert ^ (pGrayU[(s[0] ^ u8Invert)] | pGrayL[(s[1] ^ u8Invert)]);
                        d[n + 1] = u8Invert ^ (pGrayU[(s[2] ^ u8Invert)] | pGrayL[(s[3] ^ u8Invert)]);
                        d[n + 2] = u8Invert ^ (pGrayU[(s[4] ^ u8Invert)] | pGrayL[(s[5] ^ u8Invert)]);
                        d[n + 3] = u8Invert ^ (pGrayU[(s[6] ^ u8Invert)] | pGrayL[(s[7] ^ u8Invert)]);
                        s += 8;
                    } // for n
                    //  vTaskDelay(0);
                }
                bbepWriteRow(pState, pState->dma_buf, (pState->native_width / 4), 0);
                bbepRowControl(pState, ROW_STEP);
            } // for i
            delayMicroseconds(230);
        } // for pass
    } // 4bpp
    // Set the drivers inside epaper panel into discharge state.
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) bbepEinkPower(pState, 0);
    return BBEP_SUCCESS;
} /* bbepSmoothUpdate() */
//
// Perform a full (flashing) update given the current mode and pixels
// The time to perform the update can vary greatly depending on the pixel mode
// and selected options
//
int bbepFullUpdate(FASTEPDSTATE *pState, int iClearMode, bool bKeepOn, BB_RECT *pRect)
{
    int i, n, pass, iDMAOff = 0;
    int iStartCol, iStartRow, iEndCol, iEndRow;
    uint8_t u8;

#ifdef SHOW_TIME
    long l = millis();
#endif
    if (pState->iPanelType == BB_PANEL_VIRTUAL) return BBEP_ERROR_BAD_PARAMETER;

    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    switch (iClearMode) {
        case CLEAR_SLOW:
            bbepClear(pState, BB_CLEAR_DARKEN, 4, pRect);
            bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            bbepClear(pState, BB_CLEAR_DARKEN, 4, pRect);
            bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            break;
        case CLEAR_FAST:
            bbepClear(pState, BB_CLEAR_DARKEN, 2, pRect);
            bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            break;
        case CLEAR_WHITE:
            bbepClear(pState, BB_CLEAR_LIGHTEN, 8, pRect);
            break;
        case CLEAR_EXTRA_WHITE:
            bbepClear(pState, BB_CLEAR_LIGHTEN, 12, pRect);
            break;
        case CLEAR_BLACK: // probably a mistake
            bbepClear(pState, BB_CLEAR_DARKEN, 8, pRect);
            break;
        case CLEAR_NONE: // nothing to do
        default:
            break;
    }
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, pRect);
#if defined( SHOW_TIME ) && defined( ARDUINO )
    l = millis() - l;
    Serial.printf("clear time = %dms\n", (int)l);
    l = millis();
#endif // SHOW_TIME

    if (pRect) {
        if (bbepFixRect(pState, pRect, &iStartCol, &iEndCol, &iStartRow, &iEndRow)) return BBEP_ERROR_BAD_PARAMETER;
        // Prepare masked row
        memset(u8Cache, 0xff, pState->native_width / 4);
        i = iStartCol/4;
        memset(u8Cache, 0, i); // whole bytes on left side
        if ((iStartCol & 3) != 0) { // partial byte
            u8 = 0xff >> ((iStartCol & 3)*2);
            u8Cache[i] = u8;
        }
        i = (iEndCol + 3)/4;
        memset(&u8Cache[i], 0, (pState->native_width / 4) - i); // whole bytes on right side
        if ((iEndCol & 3) != 3) { // partial byte
            u8 = 0xff << ((3-(iEndCol & 3))*2);
            u8Cache[i-1] = u8;
        }
    } else { // use the whole display
        iStartCol = iStartRow = 0;
        iEndCol = pState->native_width - 1;
        iEndRow = pState->native_height - 1;
    }
    if (pState->mode == BB_MODE_2BPP) {
        // Do the update as a 'push-all' with both black and white pixels
        // getting pushed at each step. This will allow for this mode to
        // work from any starting point and without flickering
        uint8_t *s, *d;
        uint8_t *u8Gray2BW, *u8Gray2Gray;
        int dy; // destination Y for flipped displays
        // Create fast lookup tables to convert the pixels directly into pushes
        u8Gray2BW = u8Cache;
        u8Gray2Gray = &u8Cache[256];
        for (i=0; i<256; i++) {
            // black/white table
            uint8_t ucB, ucW, uc = i, ucBW = 0, ucGray = 0;
            if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                ucB = 0x40; ucW = 0x80;
            } else {
                ucB = 0x01; ucW = 0x02;
            }
            for (n=0; n<4; n++) { // for each pixel
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    ucBW >>= 2; ucGray >>= 2;
                } else {
                    ucBW <<= 2; ucGray <<= 2;
                }
                switch (uc & 0xc0) {
                    case 0:
                        ucBW |= ucB; // push black
                        ucGray |= ucB; // also push black
                        break;
                    case 0x40:
                        ucBW |= ucB; // push black
                        ucGray |= ucW; // push white
                        break;
                    case 0x80:
                        ucBW |= ucW; // push white
                        ucGray |= ucB; // push black
                        break;
                    case 0xc0:
                        ucBW |= ucW; // push white
                        ucGray |= ucW; // push white
                        break;
                } // switch
                uc <<= 2;
            } // for n
            u8Gray2BW[i] = ucBW;
            u8Gray2Gray[i] = ucGray;
        } // for i
        for (pass = 0; pass < 5; pass++) { // first 5 passes push to primary color
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                d = &pState->dma_buf[iDMAOff];
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s = &pState->pCurrent[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
                    // push white and black pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2BW[*s--]; // Turn each byte directly into black or white pushes
                    } // for n
                } else {
                    s = &pState->pCurrent[dy * (pState->native_width/4)];
                    // push white and black pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2BW[*s++]; // Turn each byte directly into black or white pushes
                    } // for n
                }
                // Send the data for the row
                bbepWriteRow(pState, &pState->dma_buf[iDMAOff], (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width/4);
            } // for i
            delayMicroseconds(230);
        } // for pass
        for (pass = 0; pass < 1; pass++) { // final 3 passes push to grays
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                d = &pState->dma_buf[iDMAOff];
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s = &pState->pCurrent[(dy * (pState->native_width/4)) + pState->native_width/4 - 1];
                    // push white and black pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2Gray[*s--]; // Turn each byte directly into black or white pushes
                    } // for n
                } else {
                    s = &pState->pCurrent[dy * (pState->native_width/4)];
                    // push light gray and dark gray pixels simultaneously
                    for (n = 0; n < (pState->native_width / 4); n++) {
                        *d++ = u8Gray2Gray[*s++]; // Turn each byte directly into light or dark pushes
                    } // for n
                }
                // Send the data for the row
                bbepWriteRow(pState, &pState->dma_buf[iDMAOff], (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width/4);
            } // for i
            delayMicroseconds(230);
        } // for pass
    } else if (pState->mode == BB_MODE_1BPP) {
        // Set the color in multiple passes starting from white
        // First create the 2-bit codes per pixel for the black pixels
        uint8_t *s, *d;
        int dy; // destination Y for flipped displays
        for (i = 0; i < pState->native_height; i++) {
            dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
            if (i >= iStartRow && i <= iEndRow) {
                s = &pState->pCurrent[i * (pState->native_width/8)];
                d = &pState->pTemp[dy * (pState->native_width/4)];
                memcpy(&pState->pPrevious[i * (pState->native_width/8)], s, pState->native_width / 8); // previous = current
                if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                    s += (pState->native_width/8) - 1;
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        uint8_t dram2 = *s--;
                        uint8_t dram1 = *s--;
                        *(uint16_t *)&d[n] = LUTB_16[dram2];
                        *(uint16_t *)&d[n+2] = LUTB_16[dram1];
                    }
                } else {
                    for (n = 0; n < (pState->native_width / 4); n += 4) {
                        uint8_t dram1 = *s++;
                        uint8_t dram2 = *s++;
                        *(uint16_t *)&d[n+2] = LUTB_16[dram2];
                        *(uint16_t *)&d[n] = LUTB_16[dram1];
                    }
                }
                if (iStartCol > 0 || iEndCol < pState->native_width-1) { // There is a region rectangle defined, clip the output to it
                    uint32_t *src, *dst;
                    src = (uint32_t *)u8Cache;
                    dst = (uint32_t *)d;
                    for (n=0; n<pState->native_width/16; n++) { // mask off non-changing pixels to 0s
                        dst[n] &= src[n];
                    }
                }
            } else { // row is not in update area
                d = &pState->pTemp[dy * (pState->native_width/4)];
                memset(d, 0, pState->native_width/4); // skip all these pixels
            }
            //vTaskDelay(0);
        } // for i
        // Write N passes of the black data to the whole display
        for (pass = 0; pass < pState->iFullPasses; pass++) {
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                d = &pState->dma_buf[iDMAOff];
                s = &pState->pTemp[i * (pState->native_width / 4)];
                // Send the data for the row
                memcpy(d, s, pState->native_width/4);
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width/4);
            }
            delayMicroseconds(230);
        } // for pass
    } else { // must be 4BPP mode
        int dy, iPasses = (pState->panelDef.iMatrixSize / 16); // number of passes
        for (pass = 0; pass < iPasses; pass++) { // number of passes to make 16 unique gray levels
            uint8_t *s, *d;
            uint8_t *pGrayU, *pGrayL;
            pGrayU = pGrayUpper + (pass * 256);
            pGrayL = pGrayLower + (pass * 256);
            bbepRowControl(pState, ROW_START);
            iDMAOff = 0;
            for (i = 0; i < pState->native_height; i++) {
                d = &pState->dma_buf[iDMAOff];
                dy = (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) ? pState->native_height - 1 - i : i;
                if (dy >= iStartRow && dy <= iEndRow) { // within the clip rectangle
                    s = &pState->pCurrent[dy * (pState->native_width / 2)];
                    if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
                        s += (pState->native_width / 2) - 8;
                        for (n = 0; n < (pState->native_width / 4); n += 4) {
                            d[n + 0] = (pGrayU[s[7]] | pGrayL[s[6]]);
                            d[n + 1] = (pGrayU[s[5]] | pGrayL[s[4]]);
                            d[n + 2] = (pGrayU[s[3]] | pGrayL[s[2]]);
                            d[n + 3] = (pGrayU[s[1]] | pGrayL[s[0]]);
                            s -= 8;
                        } // for n
                    } else {
                        for (n = 0; n < (pState->native_width / 4); n += 4) {
                            d[n + 0] = (pGrayU[s[0]] | pGrayL[s[1]]);
                            d[n + 1] = (pGrayU[s[2]] | pGrayL[s[3]]);
                            d[n + 2] = (pGrayU[s[4]] | pGrayL[s[5]]);
                            d[n + 3] = (pGrayU[s[6]] | pGrayL[s[7]]);
                            s += 8;
                        } // for n
                      //  vTaskDelay(0);
                    }
                    if (iStartCol > 0 || iEndCol < pState->native_width-1) { // There is a region rectangle defined, clip the output to it
                        uint32_t *src, *dst;
                        src = (uint32_t *)u8Cache;
                        dst = (uint32_t *)pState->dma_buf;
                        for (n=0; n<pState->native_width/16; n++) { // mask off non-changing pixels to 0s
                            dst[n] &= src[n];
                        }
                    }
                } else { // outside the clip rectangle
                    memset(d, 0, pState->native_width/4);
                }
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iDMAOff ^= (pState->native_width / 4); // toggle offset
                //bbepRowControl(pState, ROW_STEP);
            } // for i
            delayMicroseconds(230);
        } // for pass
    } // 4bpp
    // Set the drivers inside epaper panel into discharge state.
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, pRect);
    if (!bKeepOn) bbepEinkPower(pState, 0);
    
#ifdef SHOW_TIME
    l = millis() - l;
#ifdef ARDUINO
    Serial.printf("fullUpdate time: %dms\n", (int)l);
#else
    printf("fullUpdate time: %dms\n", (int)l);
#endif 
#endif // SHOW_TIME
    return BBEP_SUCCESS;
} /* bbepFullUpdate() */

int bbepPartialUpdate(FASTEPDSTATE *pState, bool bKeepOn, int iStartLine, int iEndLine)
{
    int i, n, pass, iDMAOff;
#ifdef SHOW_TIME
    long l = millis();
#endif
    if (pState->iPanelType == BB_PANEL_VIRTUAL) return BBEP_ERROR_BAD_PARAMETER;

// Only supported in 1-bit mode (for now)
    if (pState->mode != BB_MODE_1BPP) return BBEP_ERROR_BAD_PARAMETER;

    if (bbepEinkPower(pState, 1) != BBEP_SUCCESS) return BBEP_IO_ERROR;
    if (iStartLine < 0) iStartLine = 0;
    if (iEndLine >= pState->native_height) iEndLine = pState->native_height-1;
    if (iEndLine < iStartLine) return BBEP_ERROR_BAD_PARAMETER;

    uint8_t *pCur, *pPrev, *d;
    uint8_t diffw, diffb, cur, prev;
    
    for (i = iStartLine; i <= iEndLine; i++) {
        d = &pState->pTemp[i * (pState->native_width/4)]; // LUT temp storage
        pCur = &pState->pCurrent[i * (pState->native_width / 8)];
        pPrev = &pState->pPrevious[i * (pState->native_width / 8)];
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_X) {
            pCur += (pState->native_width / 8) - 1;
            pPrev += (pState->native_width / 8) - 1;
            for (n = 0; n < pState->native_width / 16; n++) {
                cur = *pCur--; prev = *pPrev;
                *pPrev-- = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

                cur = *pCur--; prev = *pPrev;
                *pPrev-- = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
                d += 4;
            }
        } else {
            for (n = 0; n < pState->native_width / 16; n++) {
                cur = *pCur++; prev = *pPrev;
                *pPrev++ = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[0] = LUTW_16[diffw] & LUTB_16[diffb];

                cur = *pCur++; prev = *pPrev;
                *pPrev++ = cur; // new->old
                diffw = prev & ~cur;
                diffb = ~prev & cur;
                *(uint16_t *)&d[2] = LUTW_16[diffw] & LUTB_16[diffb];
                d += 4;
            }
        }
    }
    if (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) {
        // adjust start/end line to be flipped
        int i;
        iStartLine = pState->native_height - 1 - iStartLine;
        iEndLine = pState->native_height - 1 - iEndLine;
        // now swap them
        i = iStartLine;
        iStartLine = iEndLine;
        iEndLine = i;
    }
    for (pass = 0; pass < pState->iPartialPasses; pass++) { // each pass is about 32ms
        uint8_t *d, *dp = pState->pTemp;
        int iDelta = pState->native_width / 4; // 2 bits per pixel
        int iSkipped = 0;
        if (pState->iFlags & BB_PANEL_FLAG_MIRROR_Y) {
            dp = &pState->pTemp[(pState->native_height-1) * iDelta]; // read the memory upside down
            iDelta = -iDelta;
        }
        bbepRowControl(pState, ROW_START);
        iDMAOff = 0;
        for (i = 0; i < pState->native_height; i++) {
            d = &pState->dma_buf[iDMAOff];
            if (i >= iStartLine && i <= iEndLine) {
                // Send the data
                memcpy(d, dp, pState->native_width/4);
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iSkipped = 0;
            } else {
               // write a neutral row
                if (iSkipped == 0) { // new skipped section
                    memset((void *)d, 0, pState->native_width/4);
                }
                bbepWriteRow(pState, d, (pState->native_width / 4), (i!=0));
                iSkipped++;
            }
            dp += iDelta;
            iDMAOff ^= (pState->native_width/4);
        }
    } // for each pass

// This clear to neutral step is necessary; do not remove
    bbepClear(pState, BB_CLEAR_NEUTRAL, 1, NULL);
    if (!bKeepOn) {
        bbepEinkPower(pState, 0);
    }

#ifdef SHOW_TIME
    l = millis() - l;
#ifdef ARDUINO
    Serial.printf("partialUpdate time: %dms\n", (int)l);
#else
    printf("partialUpdate time: %dms\n", (int)l);
#endif
#endif // SHOW_TIME
    return BBEP_SUCCESS;
} /* bbepPartialUpdate() */
//
// Copy the current pixels to the previous
// This facilitates doing partial updates after the power is lost
//
void bbepBackupPlane(FASTEPDSTATE *pState)
{
    int iSize = (pState->native_width/2) * pState->native_height;
    if (!pState->pPrevious || !pState->pCurrent) return;
    memcpy(pState->pPrevious, pState->pCurrent, iSize);
}
#endif // __BB_EP__
