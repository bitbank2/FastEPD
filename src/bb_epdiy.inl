//
// C core functions for bb_epdiy
// Written by Larry Bank
// Copyright (C) 2024 BitBank Software, Inc.
//
#include "bb_epdiy.h"
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
//#define SLOW_IO

// width, height, bus_speed, flags, data[8], ioPWR, ioSPV, ioCKV, ioSPH, ioOE, ioLE,
// ioCL, ioPWR_Good, ioSDA, ioSCL, ioShiftSTR/Wakeup, ioShiftMask/vcom, ioDCDummy
const BBPANELDEF panelDefs[] = {
    {0}, // BB_PANEL_NONE
    {960, 540, 20000000, BB_PANEL_FLAG_NONE, {6,14,7,12,9,11,8,10}, 46, 17, 18, 13, 45, 15,
      16, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, BB_NOT_USED, 47}, // BB_PANEL_M5PAPERS3
    {960, 540, 16000000, BB_PANEL_FLAG_SHIFTREG, {8,1,2,3,4,5,6,7}, BB_IO_FLAG_SHIFTREG | 5, BB_IO_FLAG_SHIFTREG | 4, 38, 40, BB_IO_FLAG_SHIFTREG | 7, BB_IO_FLAG_SHIFTREG | 0,
      41, BB_NOT_USED, 13, 12, 0, 0x32, 47}, // BB_PANEL_T5EPAPERS3
    {0, 0, 20000000, BB_PANEL_FLAG_TPS65185, {5,6,7,15,16,17,18,8}, BB_IO_FLAG_PCA9535 | 11, 45, 48, 41, BB_IO_FLAG_PCA9535 | 8, 42,
      4, BB_IO_FLAG_PCA9535 | 14, 39, 40, BB_NOT_USED, 0, 47}, // BB_PANEL_EPDIY_V7


    {1024, 758, 13333333, BB_PANEL_FLAG_TPS65186 | BB_PANEL_FLAG_SLOW_SPH, {4,5,18,19,23,25,26,27}, BB_IO_FLAG_MCP23017 | 4, BB_IO_FLAG_MCP23017 | 2, 32, 33, BB_IO_FLAG_MCP23017 | 0, 2,
      0, BB_IO_FLAG_MCP23017 | 7, 21, 22, BB_IO_FLAG_MCP23017 | 3, BB_IO_FLAG_MCP23017 | 5, 15}, // BB_PANEL_INKPLATE6PLUS

    {1280, 720, 16000000, BB_PANEL_FLAG_TPS65186 | BB_PANEL_FLAG_SLOW_SPH, {4,5,18,19,23,25,26,27}, BB_IO_FLAG_MCP23017 | 4, BB_IO_FLAG_MCP23017 | 2, 32, 33, BB_IO_FLAG_MCP23017 | 0, 2,
      0, BB_IO_FLAG_MCP23017 | 7, 21, 22, BB_IO_FLAG_MCP23017 | 3, BB_IO_FLAG_MCP23017 | 5, 15}, // BB_PANEL_INKPLATE5V2


    {960, 540, 12000000, BB_PANEL_FLAG_SHIFTREG, {33,32,4,19,2,27,21,22}, BB_IO_FLAG_SHIFTREG | 5, BB_IO_FLAG_SHIFTREG | 4, 25, 26, BB_IO_FLAG_SHIFTREG | 7, BB_IO_FLAG_SHIFTREG | 0,
      5, BB_NOT_USED, 23, 18, 0, 0x32, 15}, // BB_PANEL_T5EPAPERV1
    {0}, // BB_PANEL_CUSTOM
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
    uint8_t ucTemp[4];
    uint8_t port = pin / 8;
    pin &= 7;

    bbepI2CReadRegister(0x20, MCP23017_GPIOA + port, &ioRegs[MCP23017_GPIOA + port + 1], 1);
    return (ioRegs[MCP23017_GPIOA + port + 1] & (1 << pin)) ? HIGH : LOW;
} /* bbepMCPDigitalRead() */

//
// Write 8 bits (_state.shift_data) to the shift register
//
void bbepSendShiftData(BBEPDIYSTATE *pState)
{
    uint8_t uc = pState->shift_data;
    //Serial.printf("Sending shift data: 0x%02x\n", uc);
    // Clear STR (store) to allow updating the value
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 0);
    for (int i=0; i<8; i++) { // bits get pushed in reverse order (bit 7 first)
        gpio_set_level((gpio_num_t)pState->panelDef.ioSCL, 0);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSDA, (uc & 0x80) ? 1 : 0);
        gpio_set_level((gpio_num_t)pState->panelDef.ioSCL, 1);
        uc <<= 1;
    }
    // set STR to write the new data to the output to pins
    gpio_set_level((gpio_num_t)pState->panelDef.ioShiftSTR, 1);
} /* bbepSendShiftData() */
//
// change the state of a bit in the shift register mask, and update the outputs
//
void bbepSetShiftBit(BBEPDIYSTATE *pBBEP, uint8_t u8Bit, uint8_t u8State)
{
    if (u8State) { // set it
        pBBEP->shift_data |= (1 << u8Bit);
    } else {
        pBBEP->shift_data &= ~(1 << u8Bit);
    }
    bbepSendShiftData(pBBEP);
}

void bbepRowControl(BBEPDIYSTATE *pState, int iType)
{
    gpio_num_t ckv = (gpio_num_t)pState->panelDef.ioCKV;
    gpio_num_t spv = (gpio_num_t)pState->panelDef.ioSPV;
    gpio_num_t le = (gpio_num_t)pState->panelDef.ioLE;

    if (iType == ROW_START) {
        gpio_set_level(ckv, 1); //CKV_SET
        delayMicroseconds(7);
        if (pState->panelDef.ioSPV & BB_IO_FLAG_SHIFTREG) {
            bbepSetShiftBit(pState, spv, 0); // EP_STV = false
        } else if (pState->panelDef.ioSPV & BB_IO_FLAG_MCP23017) {
            bbepMCPDigitalWrite(spv, 0);
        } else {
            gpio_set_level(spv, 0); //SPV_CLEAR;
        }
        delayMicroseconds(10);
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        delayMicroseconds(0);
        gpio_set_level(ckv, 1); //CKV_SET;
        delayMicroseconds(8);                    
        if (pState->panelDef.ioSPV & BB_IO_FLAG_SHIFTREG) {
            bbepSetShiftBit(pState, spv, 1); // EP_SPV = true
        } else if (pState->panelDef.ioSPV & BB_IO_FLAG_MCP23017) {
            bbepMCPDigitalWrite(spv, 1);
        } else {
            gpio_set_level(spv, 1); //SPV_SET;
        }
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
        if (pState->panelDef.ioOE & BB_IO_FLAG_SHIFTREG) {
            bbepSetShiftBit(pState, 6, 1); // output_enable = true
        } else if (pState->panelDef.ioOE & BB_IO_FLAG_MCP23017) {
            bbepMCPDigitalWrite(pState->panelDef.ioOE, 1);
        }
        gpio_set_level(ckv, 1); //CKV_SET;
    } else if (iType == ROW_STEP) {
        gpio_set_level(ckv, 0); //CKV_CLEAR;
        if (pState->panelDef.ioLE & BB_IO_FLAG_SHIFTREG) { // LilyGo T5 S3
            bbepSetShiftBit(pState, le, 1); // EP_LE = true
            bbepSetShiftBit(pState, le, 0); // EP_LE = false
        } else {
            gpio_set_level(le, 1); //LE_SET;
            gpio_set_level(le, 0); //LE_CLEAR;
        }
        delayMicroseconds(0);
    }
} /* bbepRowControl() */

void bbepWriteRow(BBEPDIYSTATE *pState, uint8_t *pData, int iLen)
{
#ifdef SLOW_IO
    uint8_t uc, *s = pData;
    digitalWrite(pState->panelDef.ioSPH, 0); // CS = active
    for (int i=0; i<iLen; i++) {
        digitalWrite(pState->panelDef.ioCL, 0); // pixel clock low
        uc = *s++;
        for (int j=0; j<8; j++) {
            if (uc & (1<<j))
                digitalWrite(pState->panelDef.data[j], 1);
            else
                digitalWrite(pState->panelDef.data[j], 0);
        }
        digitalWrite(pState->panelDef.ioCL, 0); // pixel clock high
    }
#else
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
    err = esp_lcd_panel_io_tx_color(io_handle, -1, pData, iLen);
    if (err != ESP_OK) {
        Serial.printf("Error %d sending LCD data\n", (int)err);
    }
#endif // SLOW_IO
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

void bbepTPS65186Init(BBEPDIYSTATE *pState)
{
    uint8_t ucTemp[8];
    bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 1); // WAKEUP ON
    delay(1);
    ucTemp[0] = 0x9; // power up sequence register
    ucTemp[1] = 0x1b; // power up sequence
    ucTemp[2] = 0; // power up delay (3ms per rail)
    ucTemp[3] = 0x1b; // power down seq
    ucTemp[4] = 0; // power down delay (6ms per rail);
    bbepI2CWrite(0x48, ucTemp, 5);
    delay(1);
    bbepMCPDigitalWrite(pState->panelDef.ioShiftSTR, 0); // WAKEUP OFF
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
 //   if (pState->panelDef.ioPWR_Good != BB_NOT_USED) bbepPinMode(pState->panelDef.ioPWR_Good, OUTPUT);
    if (pState->panelDef.flags & BB_PANEL_FLAG_SHIFTREG) {
        Serial.println("Using shift register");
        pState->shift_data = pState->panelDef.ioShiftMask;
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
        if (pState->panelDef.ioPWR & BB_IO_FLAG_MCP23017) { // Inkplate series
            bbepMCP23017Init();
            bbepMCPPinMode(pState->panelDef.ioShiftMask, OUTPUT); // VCOM
            bbepMCPPinMode(pState->panelDef.ioPWR, OUTPUT); // PWRUP
            bbepMCPPinMode(pState->panelDef.ioShiftSTR, OUTPUT); // WAKEUP
            bbepMCPPinMode(/*GPIO0_ENABLE*/ 8, OUTPUT);
            bbepMCPDigitalWrite(/*GPIO0_ENABLE*/ 8, HIGH);
            bbepMCPPinMode(pState->panelDef.ioOE, OUTPUT);
            bbepMCPPinMode(/*GMOD*/1, OUTPUT);
            bbepMCPPinMode(pState->panelDef.ioSPV, OUTPUT);
        }
        bbepTPS65186Init(pState);
    }
    #ifdef SLOW_IO
    for (int i=0; i<8; i++) {
        pinMode((uint8_t)pState->panelDef.data[i], OUTPUT);
    }
    pinMode((uint8_t)pState->panelDef.ioCL, OUTPUT);
    pinMode((uint8_t)pState->panelDef.ioSPH, OUTPUT);
    #else
    s3_bus_config.dc_gpio_num = (gpio_num_t)pState->panelDef.ioDCDummy;
    s3_bus_config.wr_gpio_num = (gpio_num_t)pState->panelDef.ioCL;
    s3_bus_config.bus_width = 8;
    for (int i=0; i<8; i++) {
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
    #endif // SLOW_IO
    transfer_is_done = true;
    Serial.println("IO init done");
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
