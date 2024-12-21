//
// C core functions for bb_epdiy
// Written by Larry Bank
// Copyright (C) 2024 BitBank Software, Inc.
//
#include "bb_epdiy.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

static uint16_t LUTW_16[256];
static uint16_t LUTB_16[256];
static uint16_t LUT2_16[256];
volatile bool transfer_is_done = true;

static bool s3_notify_dma_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{           
//    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
//    lv_disp_flush_ready(disp_driver);
    transfer_is_done = true;
    return false;
}           

// Maximum line width = 512 * 4 = 2048 pixels
#define MAX_TX_SIZE 512
esp_lcd_i80_bus_config_t s3_bus_config = { 
    .dc_gpio_num = 0,
    .wr_gpio_num = 0,
    .clk_src = LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    .bus_width = 0,
    .max_transfer_bytes = MAX_TX_SIZE, 
//    .psram_trans_align = 0, // 0 = use default values
//    .sram_trans_align = 0,
};
esp_lcd_panel_io_i80_config_t s3_io_config = {
        .cs_gpio_num = 0,
        .pclk_hz = 12000000, // >12Mhz doesn't work on my setup
        .trans_queue_depth = 4,
        .on_color_trans_done = s3_notify_dma_ready,
        .user_ctx = nullptr, // debug
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

// width, height, bus_speed, flags, data[8], ioPWR, ioSPV, ioCKV, ioSPH, ioOE, ioLE,
// ioCL, ioPWR_Good, ioGMODE, ioSDA, ioSCL
BBPANELDEF panelDefs[] = {
    {0}, // BB_PANEL_NONE
    {960, 540, 12000000, BB_PANEL_FLAG_NONE, {6,14,7,12,9,11,8,10}, 46, 17, 18, 13, 45, 15,
      16, 0xffff, 0xffff, 0xffff, 0xffff}, // BB_PANEL_M5PAPERS3
      {0}, // BB_PANEL_CUSTOM
};

void bbepRowControl(BBEPDIYSTATE *pState, int iType)
{
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
} /* bbepRowControl() */

void bbepWriteRow(BBEPDIYSTATE *pState, uint8_t *pData, int iLen)
{
    while (!transfer_is_done) {
        delayMicroseconds(1);
    }
    transfer_is_done = false;
    gpio_set_level((gpio_num_t)pState->panelDef.ioCKV, 1); //CKV_SET;
    esp_lcd_panel_io_tx_color(io_handle, -1, pData, iLen);
}
int bbepIOInit(BBEPDIYSTATE *pState)
{
    if (pState->panelDef.ioPWR < 0x100) pinMode(pState->panelDef.ioPWR, OUTPUT);
    if (pState->panelDef.ioSPV < 0x100) pinMode(pState->panelDef.ioSPV, OUTPUT);
    if (pState->panelDef.ioCKV < 0x100) pinMode(pState->panelDef.ioCKV, OUTPUT);
    if (pState->panelDef.ioSPH < 0x100) pinMode(pState->panelDef.ioSPH, OUTPUT);
    if (pState->panelDef.ioOE < 0x100) pinMode(pState->panelDef.ioOE, OUTPUT);
    if (pState->panelDef.ioLE < 0x100) pinMode(pState->panelDef.ioLE, OUTPUT);
    if (pState->panelDef.ioCL < 0x100) pinMode(pState->panelDef.ioCL, OUTPUT);
    if (pState->panelDef.ioPWR_Good < 0x100) pinMode(pState->panelDef.ioPWR_Good, OUTPUT);
    if (pState->panelDef.ioGMODE < 0x100) pinMode(pState->panelDef.ioGMODE, OUTPUT);

    s3_bus_config.dc_gpio_num = (gpio_num_t)47; // DEBUG
    s3_bus_config.wr_gpio_num = (gpio_num_t)pState->panelDef.ioCL;
    s3_bus_config.bus_width = 8;
    for (int i=0; i<8; i++) {
        s3_bus_config.data_gpio_nums[i] = pState->panelDef.data[i];
    }   
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&s3_bus_config, &i80_bus));
    s3_io_config.pclk_hz = pState->panelDef.bus_speed;
    s3_io_config.cs_gpio_num = (gpio_num_t)pState->panelDef.ioSPH;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &s3_io_config, &io_handle));
    transfer_is_done = true;

// Create the lookup tables for 1-bit mode. Allow for inverted and mirrored
    for (int i=0; i<256; i++) {
        uint16_t b, w, l2, u16W, u16B, u16L2;
        u16W = u16B = u16L2 = 0;
        for (int j=0; j<8; j++) {
            if (!(i & (1<<j))) {
//            if (i & (1<<j)) {
                w = 2; b = 1; l2 = 1;
            } else {
                w = 3; b = 3; l2 = 2;
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
      pState->dma_buf = (uint8_t *)heap_caps_malloc((pState->width / 4) + 16, MALLOC_CAP_DMA);
    return BBEP_SUCCESS;
} /* bbepIOInit() */
