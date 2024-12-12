#ifndef __DATABUS_H__
#define __DATABUS_H__
#include "bb_epdiy.h"

#include "driver/rtc_io.h"
#include "esp_system.h"
#include <driver/gpio.h>
#include <esp_attr.h>
#include <soc/gpio_periph.h>
#include <soc/gpio_struct.h>
#include <soc/io_mux_reg.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#if ESP_IDF_VERSION < (4, 0, 0) || ARDUINO_ARCH_ESP32
#include "rom/gpio.h"
#include "rom/lldesc.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/rtc.h"
#include "esp_private/periph_ctrl.h"
#else
#include "esp32/rom/gpio.h"
#include "esp32/rom/lldesc.h"
#endif

#ifdef ARDUINO_ESP32S3_DEV
#include <soc/lcd_periph.h>
#include <soc/rmt_periph.h>
#include <esp_check.h>
#include <esp_log.h>

#include <driver/rmt_tx.h>
#include <driver/rmt_types.h>
#include <driver/rmt_types_legacy.h>
#include <esp_private/periph_ctrl.h>
#include <hal/rmt_types.h>
#include <soc/clk_tree_defs.h>
/* LCD bus configuration parameters. */
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_private/gdma.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <hal/dma_types.h>
#include <hal/gdma_ll.h>
#include <hal/gpio_hal.h>
#include <hal/lcd_hal.h>
#include <hal/lcd_ll.h>
#include <hal/rmt_ll.h>
#include <rom/cache.h>
#include <soc/lcd_periph.h>
#include <soc/rmt_struct.h>

typedef struct {
    // GPIO numbers of the parallel bus pins.
    gpio_num_t data[16];

    // horizontal clock pin.
    gpio_num_t clock;
    // vertical clock pin
    gpio_num_t ckv;

    // horizontal "Start Pulse", enabling data input on the line shift register
    gpio_num_t start_pulse;
    // latch enable
    gpio_num_t leh;
    // vertical start pulse, resetting the vertical line shift register.
    gpio_num_t stv;
} lcd_bus_config_t;

/// Configuration structure for the LCD-based Epd driver.
typedef struct {
    // high time for CKV in 1/10us.
    size_t pixel_clock;    // = 12000000
    int ckv_high_time;     // = 70
    int line_front_porch;  // = 4
    int le_high_time;      // = 4
    int bus_width;         // = 16
    lcd_bus_config_t bus;
} LcdEpdConfig_t;

typedef bool (*line_cb_func_t)(void*, uint8_t*);
typedef void (*frame_done_func_t)(void*);

void epd_lcd_init(const LcdEpdConfig_t* config, int display_width, int display_height);
void epd_lcd_deinit();
void epd_lcd_frame_done_cb(frame_done_func_t, void* payload);
void epd_lcd_line_source_cb(line_cb_func_t, void* payload);
void epd_lcd_start_frame();
/**
 * Set the LCD pixel clock frequency in MHz.
 */
void epd_lcd_set_pixel_clock_MHz(int frequency);

#else // older ESP32

/// DMA descriptors for front and back line buffer.
/// We use two buffers, so one can be filled while the other
/// is transmitted.
typedef struct {
    volatile lldesc_t* dma_desc_a;
    volatile lldesc_t* dma_desc_b;

    /// Front and back line buffer.
    uint8_t* buf_a;
    uint8_t* buf_b;
} i2s_parallel_state_t;

/* I2S bus configuration parameters. */
typedef struct {
    // GPIO numbers of the parallel bus pins.
    gpio_num_t data_0;
    gpio_num_t data_1;
    gpio_num_t data_2;
    gpio_num_t data_3;
    gpio_num_t data_4;
    gpio_num_t data_5;
    gpio_num_t data_6;
    gpio_num_t data_7;

    // Data clock pin.
    gpio_num_t clock;

    // "Start Pulse", enabling data input on the slave device (active low)
    gpio_num_t start_pulse;
} i2s_bus_config;

/* Initialize the I2S data bus for communication with a 8bit parallel display interface. */
void i2s_bus_init(i2s_bus_config* cfg, uint32_t epd_row_width);
/* Attach I2S to gpio's */
void i2s_gpio_attach(i2s_bus_config* cfg);
/* Detach I2S from gpio's */
void i2s_gpio_detach(i2s_bus_config* cfg);
/* Get the currently writable line buffer. */
uint8_t* i2s_get_current_buffer();
/* Switches front and back line buffer. If the switched-to line buffer is currently in use, */
/* this function blocks until transmission is done. */
void i2s_switch_buffer();
/* Start transmission of the current back buffer. */
void i2s_start_line_output();
/* Returns true if there is an ongoing transmission. */
bool i2s_is_busy();
/* Give up allocated resources. */
void i2s_bus_deinit();
#endif // S3 vs older ESP32

#endif // __DATABUS_H__

