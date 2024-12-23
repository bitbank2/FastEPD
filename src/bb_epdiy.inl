//
// C core functions for bb_epdiy
// Written by Larry Bank
// Copyright (C) 2024 BitBank Software, Inc.
//
#include "bb_epdiy.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

// width, height, bus_speed, flags, data[8], ioPWR, ioSPV, ioCKV, ioSPH, ioOE, ioLE,
// ioCL, ioPWR_Good, ioSDA, ioSCL, ioShiftSTR, ioShiftMask
const BBPANELDEF panelDefs[] = {
    {0}, // BB_PANEL_NONE
    {960, 540, 20000000, BB_PANEL_FLAG_NONE, {6,14,7,12,9,11,8,10}, 46, 17, 18, 13, 45, 15,
      16, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED}, // BB_PANEL_M5PAPERS3
    {960, 540, 16000000, BB_PANEL_FLAG_NONE, {8,1,2,3,4,5,6,7}, 46, BB_IO_FLAG_SHIFTREG | 0x10, 18, 13, 45, 15,
      40, BB_NOT_USED, 13, 12, 0, 0x32}, // BB_PANEL_T5EPAPERS3
    {0, 0, 20000000, BB_PANEL_FLAG_TPS65185, {5,6,7,15,16,17,18,8}, BB_IO_FLAG_PCA9535 | 11, 45, 48, 41, BB_IO_FLAG_PCA9535 | 8, 42,
      4, BB_IO_FLAG_PCA9535 | 14, 39, 40, BB_NOT_USED, 0}, // BB_PANEL_EPDIY_V7
    {1024, 758, 16000000, BB_PANEL_FLAG_TPS65186, {4,5,18,19,23,25,26,27}, BB_IO_FLAG_MCP23017 | 4, BB_IO_FLAG_MCP23017 | 2, 32, 33, BB_IO_FLAG_MCP23017 | 0, 2,
      0, BB_IO_FLAG_MCP23017 | 7, 21, 22, BB_NOT_USED, BB_NOT_USED}, // BB_PANEL_INKPLATE6PLUS
    {0}, // BB_PANEL_CUSTOM
};

static uint16_t LUTW_16[256];
static uint16_t LUTB_16[256];
static uint16_t LUT2_16[256];
// Lookup tables for grayscale mode
static uint32_t *GLUT, *GLUT2;
volatile bool transfer_is_done = true;
uint8_t u8Cache[128];
// Last element (index = 8) in waveform array is not used!
static const uint8_t waveform3Bit[8][9] =                                                                                                    \
    {{0, 0, 1, 1, 1, 1, 1, 1, 0}, {0, 0, 1, 1, 1, 1, 0, 0, 0}, {0, 1, 1, 1, 1, 1, 2, 1, 0},                            \
     {0, 0, 1, 1, 1, 1, 1, 2, 0}, {1, 2, 1, 2, 1, 1, 1, 2, 0}, {0, 1, 1, 1, 2, 0, 1, 2, 0},                            \
     {1, 1, 1, 2, 2, 2, 1, 2, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0}};

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

//
// Write 8 bits (_state.shift_data) to the shift register
//
void bbepSendShiftData(BBEPDIYSTATE *pState)
{
    uint8_t uc = pState->shift_data;
    // Clear STR (store) to allow updating the value
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 0);
    for (int i=0; i<8; i++) { // bits get pushed in reverse order (bit 7 first)
        gpio_set_level((gpio_num_t)pState->panelDef.ioSCL, 0);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSDA, (uc & 0x80) ? 1 : 0);
        uc <<= 1;
        gpio_set_level((gpio_num_t)pState->panelDef.ioSCL, 1);
    }
    // set STR to write the new data to the output to pins
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 1);
} /* bbepSendShiftData() */

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

uint8_t TPS65185PowerGood(void)
{
uint8_t ucTemp[4];

    bbepI2CReadRegister(0x68, 0x0f, ucTemp, 1);
    return ucTemp[0];
}

void bbepTPS65186Init(void)
{
    uint8_t ucTemp[8];
    ucTemp[0] = 0x9; // power up sequence register
    ucTemp[1] = 0x1b; // power up sequence
    ucTemp[2] = 0; // power up delay (3ms per rail)
    ucTemp[3] = 0x1b; // power down seq
    ucTemp[4] = 0; // power down delay (6ms per rail);
    bbepI2CWrite(0x48, ucTemp, 5);
}

int bbepIOInit(BBEPDIYSTATE *pState)
{
    if (pState->panelDef.ioPWR < 0x100) bbepPinMode(pState->panelDef.ioPWR, OUTPUT);
    if (pState->panelDef.ioSPV < 0x100) bbepPinMode(pState->panelDef.ioSPV, OUTPUT);
    if (pState->panelDef.ioCKV < 0x100) bbepPinMode(pState->panelDef.ioCKV, OUTPUT);
    if (pState->panelDef.ioSPH < 0x100) bbepPinMode(pState->panelDef.ioSPH, OUTPUT);
    if (pState->panelDef.ioOE < 0x100) bbepPinMode(pState->panelDef.ioOE, OUTPUT);
    if (pState->panelDef.ioLE < 0x100) bbepPinMode(pState->panelDef.ioLE, OUTPUT);
    if (pState->panelDef.ioCL < 0x100) bbepPinMode(pState->panelDef.ioCL, OUTPUT);
    if (pState->panelDef.ioPWR_Good != BB_NOT_USED) bbepPinMode(pState->panelDef.ioPWR_Good, OUTPUT);
    if (pState->panelDef.ioShiftSTR != BB_NOT_USED) {
        bbepPinMode(pState->panelDef.ioShiftSTR, OUTPUT);
        bbepPinMode(pState->panelDef.ioSDA, OUTPUT);
        bbepPinMode(pState->panelDef.ioSCL, OUTPUT);
        gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 0);
        bbepSendShiftData(pState); // send default control bits
    }
    if (pState->panelDef.flags & BB_PANEL_FLAG_TPS65185) { // TI power controller in use
        bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL);
        bbepPCA9535SetConfig(0xc0); // set lower 6 bits as outputs and 6 (PWRGOOD) and 7 (INTR) as inputs
    }
    if (pState->panelDef.flags & BB_PANEL_FLAG_TPS65186) { // TI power controller in use
        bbepI2CInit((uint8_t)pState->panelDef.ioSDA, (uint8_t)pState->panelDef.ioSCL);
      //  bbepPCA9535SetConfig(0xc0); // set lower 6 bits as outputs and 6 (PWRGOOD) and 7 (INTR) as inputs
        bbepTPS65186Init();
    }
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
            if (!(i & (0x80>>(7-j)))) {
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
