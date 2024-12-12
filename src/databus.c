#include "databus.h"

#ifdef ARDUINO_ESP32S3_DEV
#define TAG "bb_epdiy"

static inline int min(int x, int y) {
    return x < y ? x : y;
}
static inline int max(int x, int y) {
    return x > y ? x : y;
}

#define S3_LCD_PIN_NUM_BK_LIGHT -1
// #define S3_LCD_PIN_NUM_MODE           4

#define LINE_BATCH 1000
#define BOUNCE_BUF_LINES 4

#define RMT_CKV_CHAN RMT_CHANNEL_1

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
// The extern line is declared in esp-idf/components/driver/deprecated/rmt_legacy.c. It has access
// to RMTMEM through the rmt_private.h header which we can't access outside the sdk. Declare our own
// extern here to properly use the RMTMEM smybol defined in
// components/soc/[target]/ld/[target].peripherals.ld Also typedef the new rmt_mem_t struct to the
// old rmt_block_mem_t struct. Same data fields, different names
typedef rmt_mem_t rmt_block_mem_t;
extern rmt_block_mem_t RMTMEM;
#endif

// spinlock for protecting the critical section at frame start
static portMUX_TYPE frame_start_spinlock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    lcd_hal_context_t hal;
    intr_handle_t vsync_intr;
    intr_handle_t done_intr;

    frame_done_func_t frame_done_cb;
    line_cb_func_t line_source_cb;
    void* line_cb_payload;
    void* frame_cb_payload;

    int line_length_us;
    int line_cycles;
    int lcd_res_h;

    LcdEpdConfig_t config;

    uint8_t* bounce_buffer[2];
    // size of a single bounce buffer
    size_t bb_size;
    size_t batches;

    // Number of DMA descriptors that used to carry the frame buffer
    size_t num_dma_nodes;
    // DMA channel handle
    gdma_channel_handle_t dma_chan;
    // DMA descriptors pool
    dma_descriptor_t* dma_nodes;

    /// The number of bytes in a horizontal display register line.
    int line_bytes;

    // With 8 bit bus width, we need a dummy cycle before the actual data,
    // because the LCD peripheral behaves weirdly.
    // Also see:
    // https://blog.adafruit.com/2022/06/14/esp32uesday-hacking-the-esp32-s3-lcd-peripheral/
    int dummy_bytes;

    /// The number of lines of the display
    int display_lines;
} s3_lcd_t;

static s3_lcd_t lcd = { 0 };

void IRAM_ATTR epd_lcd_line_source_cb(line_cb_func_t line_source, void* payload) {
    lcd.line_source_cb = line_source;
    lcd.line_cb_payload = payload;
}

void IRAM_ATTR epd_lcd_frame_done_cb(frame_done_func_t cb, void* payload) {
    lcd.frame_done_cb = cb;
    lcd.frame_cb_payload = payload;
}

static IRAM_ATTR bool fill_bounce_buffer(uint8_t* buffer) {
    bool task_awoken = false;

    for (int i = 0; i < BOUNCE_BUF_LINES; i++) {
        if (lcd.line_source_cb != NULL) {
            // this is strange, with 16 bit need a dummy cycle. But still, the first byte in the
            // FIFO is correct. So we only need a true dummy byte in the FIFO in the 8 bit
            // configuration.
            int buffer_offset = i * (lcd.line_bytes + lcd.dummy_bytes) + (lcd.dummy_bytes % 2);
            task_awoken |= lcd.line_source_cb(lcd.line_cb_payload, &buffer[buffer_offset]);
        } else {
            memset(&buffer[i * lcd.line_bytes], 0x00, lcd.line_bytes);
        }
    }
    return task_awoken;
}

static void start_ckv_cycles(int cycles) {
    rmt_ll_tx_enable_loop_count(&RMT, RMT_CKV_CHAN, true);
    rmt_ll_tx_enable_loop_autostop(&RMT, RMT_CKV_CHAN, true);
    rmt_ll_tx_set_loop_count(&RMT, RMT_CKV_CHAN, cycles);
    rmt_ll_tx_reset_pointer(&RMT, RMT_CKV_CHAN);
    rmt_ll_tx_start(&RMT, RMT_CKV_CHAN);
}

/**
 * Build the RMT signal according to the timing set in the lcd object.
 */
static void ckv_rmt_build_signal() {
    int low_time = (lcd.line_length_us * 10 - lcd.config.ckv_high_time);
    volatile rmt_item32_t* rmt_mem_ptr = &(RMTMEM.chan[RMT_CKV_CHAN].data32[0]);
    rmt_mem_ptr->duration0 = lcd.config.ckv_high_time;
    rmt_mem_ptr->level0 = 1;
    rmt_mem_ptr->duration1 = low_time;
    rmt_mem_ptr->level1 = 0;
    rmt_mem_ptr[1].val = 0;
}

/**
 * Configure the RMT peripheral for use as the CKV clock.
 */
static void init_ckv_rmt() {
    periph_module_reset(rmt_periph_signals.groups[0].module);
    periph_module_enable(rmt_periph_signals.groups[0].module);

    rmt_ll_enable_periph_clock(&RMT, true);

    // Divide 80MHz APB Clock by 8 -> .1us resolution delay
    // idf >= 5.0 calculates the clock divider differently
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rmt_ll_set_group_clock_src(&RMT, RMT_CKV_CHAN, RMT_CLK_SRC_DEFAULT, 1, 0, 0);
#else
    rmt_ll_set_group_clock_src(
        &RMT, RMT_CKV_CHAN, (rmt_clock_source_t)RMT_BASECLK_DEFAULT, 0, 0, 0
    );
#endif
    rmt_ll_tx_set_channel_clock_div(&RMT, RMT_CKV_CHAN, 8);
    rmt_ll_tx_set_mem_blocks(&RMT, RMT_CKV_CHAN, 2);
    rmt_ll_enable_mem_access_nonfifo(&RMT, true);
    rmt_ll_tx_fix_idle_level(&RMT, RMT_CKV_CHAN, RMT_IDLE_LEVEL_LOW, true);
    rmt_ll_tx_enable_carrier_modulation(&RMT, RMT_CKV_CHAN, false);

    rmt_ll_tx_enable_loop(&RMT, RMT_CKV_CHAN, true);

    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[lcd.config.bus.ckv], PIN_FUNC_GPIO);
    gpio_set_direction(lcd.config.bus.ckv, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(
        lcd.config.bus.ckv, rmt_periph_signals.groups[0].channels[RMT_CKV_CHAN].tx_sig, false, 0
    );

    ckv_rmt_build_signal();
}

/**
 * Reset the CKV RMT configuration.
 */
static void deinit_ckv_rmt() {
    periph_module_reset(rmt_periph_signals.groups[0].module);
    periph_module_disable(rmt_periph_signals.groups[0].module);

    gpio_reset_pin(lcd.config.bus.ckv);
}

__attribute__((optimize("O3"))) IRAM_ATTR static void lcd_isr_vsync(void* args) {
    bool need_yield = false;

    uint32_t intr_status = lcd_ll_get_interrupt_status(lcd.hal.dev);
    lcd_ll_clear_interrupt_status(lcd.hal.dev, intr_status);

    if (intr_status & LCD_LL_EVENT_VSYNC_END) {
        int batches_needed = lcd.display_lines / LINE_BATCH;
        if (lcd.batches >= batches_needed) {
            lcd_ll_stop(lcd.hal.dev);
            if (lcd.frame_done_cb != NULL) {
                (*lcd.frame_done_cb)(lcd.frame_cb_payload);
            }
        } else {
            int ckv_cycles = 0;
            // last batch
            if (lcd.batches == batches_needed - 1) {
                lcd_ll_enable_auto_next_frame(lcd.hal.dev, false);
                lcd_ll_set_vertical_timing(lcd.hal.dev, 1, 0, lcd.display_lines % LINE_BATCH, 10);
                ckv_cycles = lcd.display_lines % LINE_BATCH + 10;
            } else {
                lcd_ll_set_vertical_timing(lcd.hal.dev, 1, 0, LINE_BATCH, 1);
                ckv_cycles = LINE_BATCH + 1;
            }
            // apparently, this is needed for the new timing to take effect.
            lcd_ll_start(lcd.hal.dev);

            // skip the LCD front porch line, which is not actual data
            esp_rom_delay_us(lcd.line_length_us);
            start_ckv_cycles(ckv_cycles);
        }

        lcd.batches += 1;
    }

    if (need_yield) {
        portYIELD_FROM_ISR();
    }
};

// ISR handling bounce buffer refill
static IRAM_ATTR bool lcd_rgb_panel_eof_handler(
    gdma_channel_handle_t dma_chan, gdma_event_data_t* event_data, void* user_data
) {
    dma_descriptor_t* desc = (dma_descriptor_t*)event_data->tx_eof_desc_addr;
    // Figure out which bounce buffer to write to.
    // Note: what we receive is the *last* descriptor of this bounce buffer.
    int bb = (desc == &lcd.dma_nodes[0]) ? 0 : 1;
    return fill_bounce_buffer(lcd.bounce_buffer[bb]);
}

static esp_err_t init_dma_trans_link() {
    lcd.dma_nodes[0].dw0.suc_eof = 1;
    lcd.dma_nodes[0].dw0.size = lcd.bb_size;
    lcd.dma_nodes[0].dw0.length = lcd.bb_size;
    lcd.dma_nodes[0].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
    lcd.dma_nodes[0].buffer = lcd.bounce_buffer[0];

    lcd.dma_nodes[1].dw0.suc_eof = 1;
    lcd.dma_nodes[1].dw0.size = lcd.bb_size;
    lcd.dma_nodes[1].dw0.length = lcd.bb_size;
    lcd.dma_nodes[1].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
    lcd.dma_nodes[1].buffer = lcd.bounce_buffer[1];

    // loop end back to start
    lcd.dma_nodes[0].next = &lcd.dma_nodes[1];
    lcd.dma_nodes[1].next = &lcd.dma_nodes[0];

    // alloc DMA channel and connect to LCD peripheral
    gdma_channel_alloc_config_t dma_chan_config = {
        .direction = GDMA_CHANNEL_DIRECTION_TX,
    };
    ESP_RETURN_ON_ERROR(
        gdma_new_channel(&dma_chan_config, &lcd.dma_chan), TAG, "alloc DMA channel failed"
    );
    gdma_trigger_t trigger = GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0);
    ESP_RETURN_ON_ERROR(gdma_connect(lcd.dma_chan, trigger), TAG, "dma connect error");
    gdma_transfer_ability_t ability = {
        .psram_trans_align = 64,
        .sram_trans_align = 4,
    };
    ESP_RETURN_ON_ERROR(gdma_set_transfer_ability(lcd.dma_chan, &ability), TAG, "dma setup error");

    gdma_tx_event_callbacks_t cbs = {
        .on_trans_eof = lcd_rgb_panel_eof_handler,
    };
    ESP_RETURN_ON_ERROR(
        gdma_register_tx_event_callbacks(lcd.dma_chan, &cbs, NULL), TAG, "dma setup error"
    );

    return ESP_OK;
}

void deinit_dma_trans_link() {
    gdma_reset(lcd.dma_chan);
    gdma_disconnect(lcd.dma_chan);
    gdma_del_channel(lcd.dma_chan);
}

/**
 * Configure LCD peripheral and auxiliary GPIOs
 */
static esp_err_t init_bus_gpio() {
    const int DATA_LINES[16] = {
        lcd.config.bus.data[14], lcd.config.bus.data[15], lcd.config.bus.data[12],
        lcd.config.bus.data[13], lcd.config.bus.data[10], lcd.config.bus.data[11],
        lcd.config.bus.data[8],  lcd.config.bus.data[9],  lcd.config.bus.data[6],
        lcd.config.bus.data[7],  lcd.config.bus.data[4],  lcd.config.bus.data[5],
        lcd.config.bus.data[2],  lcd.config.bus.data[3],  lcd.config.bus.data[0],
        lcd.config.bus.data[1],
    };

    // connect peripheral signals via GPIO matrix
    for (size_t i = (16 - lcd.config.bus_width); i < 16; i++) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[DATA_LINES[i]], PIN_FUNC_GPIO);
        gpio_set_direction(DATA_LINES[i], GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(
            DATA_LINES[i], lcd_periph_signals.panels[0].data_sigs[i], false, false
        );
    }
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[lcd.config.bus.leh], PIN_FUNC_GPIO);
    gpio_set_direction(lcd.config.bus.leh, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(
        lcd.config.bus.leh, lcd_periph_signals.panels[0].hsync_sig, false, false
    );

    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[lcd.config.bus.clock], PIN_FUNC_GPIO);
    gpio_set_direction(lcd.config.bus.clock, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(
        lcd.config.bus.clock, lcd_periph_signals.panels[0].pclk_sig, false, false
    );

    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[lcd.config.bus.start_pulse], PIN_FUNC_GPIO);
    gpio_set_direction(lcd.config.bus.start_pulse, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(
        lcd.config.bus.start_pulse, lcd_periph_signals.panels[0].de_sig, false, false
    );

    gpio_config_t vsync_gpio_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ull << lcd.config.bus.stv,
    };
    gpio_config(&vsync_gpio_conf);
    gpio_set_level(lcd.config.bus.stv, 1);
    return ESP_OK;
}

/**
 * Reset bus GPIO pin functions.
 */
static void deinit_bus_gpio() {
    for (size_t i = (16 - lcd.config.bus_width); i < 16; i++) {
        gpio_reset_pin(lcd.config.bus.data[i]);
    }

    gpio_reset_pin(lcd.config.bus.leh);
    gpio_reset_pin(lcd.config.bus.clock);
    gpio_reset_pin(lcd.config.bus.start_pulse);
    gpio_reset_pin(lcd.config.bus.stv);
}

/**
 * Check if the PSRAM cache is properly configured.
 */
static void check_cache_configuration() {
    if (CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE < 64) {
        ESP_LOGE(
            "epdiy",
            "cache line size is set to %d (< 64B)! This will degrade performance, please update "
            "this option in menuconfig.",
            CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE
        );
        ESP_LOGE(
            "epdiy",
            "If you are on arduino, you can't set this option yourself, you'll need to use a lower "
            "speed."
        );
        ESP_LOGE(
            "epdiy",
            "Reducing the pixel clock from %d MHz to %d MHz for now!",
            lcd.config.pixel_clock / 1000 / 1000,
            lcd.config.pixel_clock / 1000 / 1000 / 2
        );
        lcd.config.pixel_clock = lcd.config.pixel_clock / 2;

        // fixme: this would be nice, but doesn't work :(
        // uint32_t d_autoload = Cache_Suspend_DCache();
        /// Cache_Set_DCache_Mode(CACHE_SIZE_FULL, CACHE_4WAYS_ASSOC, CACHE_LINE_SIZE_32B);
        // Cache_Invalidate_DCache_All();
        // Cache_Resume_DCache(d_autoload);
    }
}

/**
 * Assign LCD configuration parameters from a given configuration, without allocating memory or
 * touching the LCD peripheral config.
 */
static void assign_lcd_parameters_from_config(
    const LcdEpdConfig_t* config, int display_width, int display_height
) {
    // copy over the configuraiton object
    memcpy(&lcd.config, config, sizeof(LcdEpdConfig_t));

    // Make sure the bounce buffers divide the display height evenly.
    lcd.display_lines = (((display_height + 7) / 8) * 8);

    lcd.line_bytes = display_width / 4;
    lcd.lcd_res_h = lcd.line_bytes / (lcd.config.bus_width / 8);

    // With 8 bit bus width, we need a dummy cycle before the actual data,
    // because the LCD peripheral behaves weirdly.
    // Also see:
    // https://blog.adafruit.com/2022/06/14/esp32uesday-hacking-the-esp32-s3-lcd-peripheral/
    lcd.dummy_bytes = lcd.config.bus_width / 8;

    // each bounce buffer holds a number of lines with data + dummy bytes each
    lcd.bb_size = BOUNCE_BUF_LINES * (lcd.line_bytes + lcd.dummy_bytes);

    check_cache_configuration();

    ESP_LOGI(TAG, "using resolution %dx%d", lcd.lcd_res_h, lcd.display_lines);
}

/**
 * Allocate buffers for LCD driver operation.
 */
static esp_err_t allocate_lcd_buffers() {
    uint32_t dma_flags = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;

    // allocate bounce buffers
    for (int i = 0; i < 2; i++) {
        lcd.bounce_buffer[i] = heap_caps_aligned_calloc(4, 1, lcd.bb_size, dma_flags);
        ESP_RETURN_ON_FALSE(lcd.bounce_buffer[i], ESP_ERR_NO_MEM, TAG, "install interrupt failed");
    }

    // So far, I haven't seen any displays with > 4096 pixels per line,
    // so we only need one DMA node for now.
    assert(lcd.bb_size < DMA_DESCRIPTOR_BUFFER_MAX_SIZE);
    lcd.dma_nodes = heap_caps_calloc(1, sizeof(dma_descriptor_t) * 2, dma_flags);
    ESP_RETURN_ON_FALSE(lcd.dma_nodes, ESP_ERR_NO_MEM, TAG, "no mem for dma nodes");
    return ESP_OK;
}

static void free_lcd_buffers() {
    for (int i = 0; i < 2; i++) {
        uint8_t* buf = lcd.bounce_buffer[i];
        if (buf != NULL) {
            heap_caps_free(buf);
            lcd.bounce_buffer[i] = NULL;
        }
    }

    if (lcd.dma_nodes != NULL) {
        heap_caps_free(lcd.dma_nodes);
        lcd.dma_nodes = NULL;
    }
}

/**
 * Initialize the LCD peripheral itself and install interrupts.
 */
static esp_err_t init_lcd_peripheral() {
    esp_err_t ret = ESP_OK;

    // enable APB to access LCD registers
    periph_module_enable(lcd_periph_signals.panels[0].module);
    periph_module_reset(lcd_periph_signals.panels[0].module);

    lcd_hal_init(&lcd.hal, 0);
    lcd_ll_enable_clock(lcd.hal.dev, true);
    lcd_ll_select_clk_src(lcd.hal.dev, LCD_CLK_SRC_PLL240M);
    ESP_RETURN_ON_ERROR(ret, TAG, "set source clock failed");

    // install interrupt service, (LCD peripheral shares the interrupt source with Camera by
    // different mask)
    int flags = ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED
                | ESP_INTR_FLAG_LOWMED;
    int source = lcd_periph_signals.panels[0].irq_id;
    uint32_t status = (uint32_t)lcd_ll_get_interrupt_status_reg(lcd.hal.dev);
    ret = esp_intr_alloc_intrstatus(
        source, flags, status, LCD_LL_EVENT_VSYNC_END, lcd_isr_vsync, NULL, &lcd.vsync_intr
    );
    ESP_RETURN_ON_ERROR(ret, TAG, "install interrupt failed");

    status = (uint32_t)lcd_ll_get_interrupt_status_reg(lcd.hal.dev);
    ret = esp_intr_alloc_intrstatus(
        source, flags, status, LCD_LL_EVENT_TRANS_DONE, lcd_isr_vsync, NULL, &lcd.done_intr
    );
    ESP_RETURN_ON_ERROR(ret, TAG, "install interrupt failed");

    lcd_ll_fifo_reset(lcd.hal.dev);
    lcd_ll_reset(lcd.hal.dev);

    // pixel clock phase and polarity
    lcd_ll_set_clock_idle_level(lcd.hal.dev, false);
    lcd_ll_set_pixel_clock_edge(lcd.hal.dev, false);

    // enable RGB mode and set data width
    lcd_ll_enable_rgb_mode(lcd.hal.dev, true);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    lcd_ll_set_dma_read_stride(lcd.hal.dev, lcd.config.bus_width);
    lcd_ll_set_data_wire_width(lcd.hal.dev, lcd.config.bus_width);
#else
    lcd_ll_set_data_width(lcd.hal.dev, lcd.config.bus_width);
#endif
    lcd_ll_set_phase_cycles(lcd.hal.dev, 0, (lcd.dummy_bytes > 0), 1);  // enable data phase only
    lcd_ll_enable_output_hsync_in_porch_region(lcd.hal.dev, false);     // enable data phase only

    // number of data cycles is controlled by DMA buffer size
    lcd_ll_enable_output_always_on(lcd.hal.dev, false);
    lcd_ll_set_idle_level(lcd.hal.dev, false, true, true);

    // configure blank region timing
    // RGB panel always has a front and back blank (porch region)
    lcd_ll_set_blank_cycles(lcd.hal.dev, 1, 1);

    // output hsync even in porch region?
    lcd_ll_enable_output_hsync_in_porch_region(lcd.hal.dev, false);
    // send next frame automatically in stream mode
    lcd_ll_enable_auto_next_frame(lcd.hal.dev, false);

    lcd_ll_enable_interrupt(lcd.hal.dev, LCD_LL_EVENT_VSYNC_END, true);
    lcd_ll_enable_interrupt(lcd.hal.dev, LCD_LL_EVENT_TRANS_DONE, true);

    // enable intr
    esp_intr_enable(lcd.vsync_intr);
    esp_intr_enable(lcd.done_intr);
    return ret;
}

static void deinit_lcd_peripheral() {
    // disable and free interrupts
    esp_intr_disable(lcd.vsync_intr);
    esp_intr_disable(lcd.done_intr);
    esp_intr_free(lcd.vsync_intr);
    esp_intr_free(lcd.done_intr);

    lcd_ll_fifo_reset(lcd.hal.dev);
    lcd_ll_reset(lcd.hal.dev);

    periph_module_reset(lcd_periph_signals.panels[0].module);
    periph_module_disable(lcd_periph_signals.panels[0].module);
}

/**
 * Configure the LCD driver for epdiy.
 */
void epd_lcd_init(const LcdEpdConfig_t* config, int display_width, int display_height) {
    esp_err_t ret = ESP_OK;
    assign_lcd_parameters_from_config(config, display_width, display_height);

    check_cache_configuration();

    ret = allocate_lcd_buffers();
    ESP_GOTO_ON_ERROR(ret, err, TAG, "lcd buffer allocation failed");

    ret = init_lcd_peripheral();
    ESP_GOTO_ON_ERROR(ret, err, TAG, "lcd peripheral init failed");

    ret = init_dma_trans_link();
    ESP_GOTO_ON_ERROR(ret, err, TAG, "install DMA failed");

    ret = init_bus_gpio();
    ESP_GOTO_ON_ERROR(ret, err, TAG, "configure GPIO failed");

    init_ckv_rmt();

    // setup driver state
    epd_lcd_set_pixel_clock_MHz(lcd.config.pixel_clock / 1000 / 1000);
    epd_lcd_line_source_cb(NULL, NULL);

    ESP_LOGI(TAG, "LCD init done.");
    return;
err:
    ESP_LOGE(TAG, "LCD initialization failed!");
    abort();
}

/**
 * Deinitializue the LCD driver, i.e., free resources and peripherals.
 */
void epd_lcd_deinit() {
    epd_lcd_line_source_cb(NULL, NULL);

    deinit_bus_gpio();
    deinit_lcd_peripheral();
    deinit_dma_trans_link();
    free_lcd_buffers();
    deinit_ckv_rmt();

    ESP_LOGI(TAG, "LCD deinitialized.");
}

void epd_lcd_set_pixel_clock_MHz(int frequency) {
    lcd.config.pixel_clock = frequency * 1000 * 1000;

    // set pclk
    int flags = 0;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
    hal_utils_clk_div_t clk_div = {};
    uint32_t freq
        = lcd_hal_cal_pclk_freq(&lcd.hal, 240000000, lcd.config.pixel_clock, flags, &clk_div);
    lcd_ll_set_group_clock_coeff(
        &LCD_CAM, (int)clk_div.integer, (int)clk_div.denominator, (int)clk_div.numerator
    );
#else
    uint32_t freq = lcd_hal_cal_pclk_freq(&lcd.hal, 240000000, lcd.config.pixel_clock, flags);
#endif

    ESP_LOGI(TAG, "pclk freq: %d Hz", freq);
    lcd.line_length_us = (lcd.lcd_res_h + lcd.config.le_high_time + lcd.config.line_front_porch - 1)
                             * 1000000 / lcd.config.pixel_clock
                         + 1;
    lcd.line_cycles = lcd.line_length_us * lcd.config.pixel_clock / 1000000;
    ESP_LOGI(TAG, "line width: %dus, %d cylces", lcd.line_length_us, lcd.line_cycles);

    ckv_rmt_build_signal();
}

void IRAM_ATTR epd_lcd_start_frame() {
    int initial_lines = min(LINE_BATCH, lcd.display_lines);

    // hsync: pulse with, back porch, active width, front porch
    int end_line
        = lcd.line_cycles - lcd.lcd_res_h - lcd.config.le_high_time - lcd.config.line_front_porch;
    lcd_ll_set_horizontal_timing(
        lcd.hal.dev,
        lcd.config.le_high_time - (lcd.dummy_bytes > 0),
        lcd.config.line_front_porch,
        // a dummy byte is neeed in 8 bit mode to work around LCD peculiarities
        lcd.lcd_res_h + (lcd.dummy_bytes > 0),
        end_line
    );
    lcd_ll_set_vertical_timing(lcd.hal.dev, 1, 1, initial_lines, 1);

    // generate the hsync at the very beginning of line
    lcd_ll_set_hsync_position(lcd.hal.dev, 1);

    // reset FIFO of DMA and LCD, incase there remains old frame data
    gdma_reset(lcd.dma_chan);
    lcd_ll_stop(lcd.hal.dev);
    lcd_ll_fifo_reset(lcd.hal.dev);
    lcd_ll_enable_auto_next_frame(lcd.hal.dev, true);

    lcd.batches = 0;
    fill_bounce_buffer(lcd.bounce_buffer[0]);
    fill_bounce_buffer(lcd.bounce_buffer[1]);

    // the start of DMA should be prior to the start of LCD engine
    gdma_start(lcd.dma_chan, (intptr_t)&lcd.dma_nodes[0]);

    // enter a critical section to ensure the frame start timing is correct
    taskENTER_CRITICAL(&frame_start_spinlock);

    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    gpio_set_level(lcd.config.bus.stv, 0);
    // esp_rom_delay_us(1);
    //  for picture clarity, it seems to be important to start CKV at a "good"
    //  time, seemingly start or towards end of line.
    start_ckv_cycles(initial_lines + 5);
    esp_rom_delay_us(lcd.line_length_us);
    gpio_set_level(lcd.config.bus.stv, 1);
    esp_rom_delay_us(lcd.line_length_us);
    esp_rom_delay_us(lcd.config.ckv_high_time / 10);

    // start LCD engine
    lcd_ll_start(lcd.hal.dev);

    taskEXIT_CRITICAL(&frame_start_spinlock);
}

#else // old ESP32
volatile uint8_t *_dmaLineBuffer;
volatile lldesc_t *_dmaI2SDesc; 
// Use only I2S1 (I2S0 is not compatible with 8 bit data).
volatile i2s_dev_t *myI2S;
void IRAM_ATTR I2SInit(i2s_dev_t *_i2sDev, uint8_t _clockDivider);

int bbepIOInit(BBEPDIYSTATE *pState) {
    myI2S = &I2S1;
    // Allocate memory for DMA descriptor and line buffer.
    _dmaLineBuffer = (uint8_t *)heap_caps_malloc((pState->display.iWidth / 4) + 16, MALLOC_CAP_DMA);
    _dmaI2SDesc = (lldesc_t *)heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
    if (_dmaLineBuffer == NULL || _dmaI2SDesc == NULL) {
        return BBEP_MEM_ERROR;
    }
    I2SInit(myI2S, 80 / pState->display.iBusSpeed);
    return BBEP_SUCCESS;
}
/// Indicates which line buffer is currently back / front.
static int current_buffer = 0;

/// The I2S state instance.
static i2s_parallel_state_t i2s_state;

static intr_handle_t gI2S_intr_handle = NULL;

/// Indicates the device has finished its transmission and is ready again.
static volatile bool output_done = true;
/// The start pulse pin extracted from the configuration for use in the "done"
/// interrupt.
static gpio_num_t start_pulse_pin;

/// Initializes a DMA descriptor.
static void fill_dma_desc(
    volatile lldesc_t* dmadesc, uint8_t* buf, uint32_t buf_size, uint32_t buf_length
) {
    assert(buf_length % 4 == 0);  // Must be word aligned
    dmadesc->size = buf_size;
    dmadesc->length = buf_length;
    dmadesc->buf = buf;
    dmadesc->eof = 1;
    dmadesc->sosf = 1;
    dmadesc->owner = 1;
    dmadesc->qe.stqe_next = 0;
    dmadesc->offset = 0;
}

/// Address of the currently front DMA descriptor,
/// which uses only the lower 20bits (according to TRM)
uint32_t dma_desc_addr() {
    return (uint32_t)(current_buffer ? i2s_state.dma_desc_a : i2s_state.dma_desc_b) & 0x000FFFFF;
}

// Helper function to help align values
static size_t align_up(size_t x, size_t a) {
    return (size_t)(x + ((size_t)a - 1)) & ~(size_t)(a - 1);
}

/// Set up a GPIO as output and route it to a signal.
static void gpio_setup_out(int gpio, int sig, bool invert) {
    if (gpio == -1)
        return;
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
    gpio_set_direction(gpio, GPIO_MODE_DEF_OUTPUT);
    gpio_matrix_out(gpio, sig, invert, false);
}

/// Resets "Start Pulse" signal when the current row output is done.
static void IRAM_ATTR i2s_int_hdl(void* arg) {
    i2s_dev_t* dev = &I2S1;
    if (dev->int_st.out_done) {
        // gpio_set_level(start_pulse_pin, 1);
        // gpio_set_level(GPIO_NUM_26, 0);
        output_done = true;
    }
    // Clear the interrupt. Otherwise, the whole device would hang.
    dev->int_clr.val = dev->int_raw.val;
}

uint8_t* IRAM_ATTR i2s_get_current_buffer() {
    return (uint8_t*)(current_buffer ? i2s_state.dma_desc_a->buf : i2s_state.dma_desc_b->buf);
}

bool IRAM_ATTR i2s_is_busy() {
    // DMA and FIFO must be done
    return !output_done || !I2S1.state.tx_idle;
}

void IRAM_ATTR i2s_switch_buffer() {
    // either device is done transmitting or the switch must be away from the
    // buffer currently used by the DMA engine.
    while (i2s_is_busy() && dma_desc_addr() != I2S1.out_link.addr) {
    };
    current_buffer = !current_buffer;
}

void IRAM_ATTR i2s_start_line_output() {
    output_done = false;

    i2s_dev_t* dev = &I2S1;
    dev->conf.tx_start = 0;
    dev->conf.tx_reset = 1;
    dev->conf.tx_fifo_reset = 1;
    dev->conf.rx_fifo_reset = 1;
    dev->conf.tx_reset = 0;
    dev->conf.tx_fifo_reset = 0;
    dev->conf.rx_fifo_reset = 0;
    dev->out_link.addr = dma_desc_addr();
    dev->out_link.start = 1;

    // sth is pulled up through peripheral interrupt
    // This is timing-critical!
    gpio_set_level(start_pulse_pin, 0);
    dev->conf.tx_start = 1;
}

void i2s_gpio_attach(i2s_bus_config* cfg) {
    gpio_num_t I2S_GPIO_BUS[] = { cfg->data_6, cfg->data_7, cfg->data_4, cfg->data_5,
                                  cfg->data_2, cfg->data_3, cfg->data_0, cfg->data_1 };

    gpio_set_direction(cfg->start_pulse, GPIO_MODE_OUTPUT);
    gpio_set_level(cfg->start_pulse, 1);
    // Use I2S1 with no signal offset (for some reason the offset seems to be
    // needed in 16-bit mode, but not in 8 bit mode.
    int signal_base = I2S1O_DATA_OUT0_IDX;

    // Setup and route GPIOS
    for (int x = 0; x < 8; x++) {
        gpio_setup_out(I2S_GPIO_BUS[x], signal_base + x, false);
    }

    // Free CKH after wakeup
    gpio_hold_dis(cfg->clock);
    // Invert word select signal
    gpio_setup_out(cfg->clock, I2S1O_WS_OUT_IDX, true);
}

void i2s_gpio_detach(i2s_bus_config* cfg) {
    gpio_set_direction(cfg->data_0, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_1, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_2, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_3, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_4, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_5, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_6, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->data_7, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->start_pulse, GPIO_MODE_INPUT);
    gpio_set_direction(cfg->clock, GPIO_MODE_INPUT);

    gpio_reset_pin(cfg->clock);
    if (cfg->clock != 5) {
        rtc_gpio_isolate(cfg->clock);
    }
}

void i2s_bus_init(i2s_bus_config* cfg, uint32_t epd_row_width) {
    i2s_gpio_attach(cfg);

    // store pin in global variable for use in interrupt.
    start_pulse_pin = cfg->start_pulse;

    periph_module_enable(PERIPH_I2S1_MODULE);

    i2s_dev_t* dev = &I2S1;

    // Initialize device
    dev->conf.tx_reset = 1;
    dev->conf.tx_reset = 0;

    // Reset DMA
    dev->lc_conf.in_rst = 1;
    dev->lc_conf.in_rst = 0;
    dev->lc_conf.out_rst = 1;
    dev->lc_conf.out_rst = 0;

    // Setup I2S config. See section 12 of Technical Reference Manual
    // Enable LCD mode
    dev->conf2.val = 0;
    dev->conf2.lcd_en = 1;

    // Enable FRAME1-Mode (See technical reference manual)
    dev->conf2.lcd_tx_wrx2_en = 1;
    dev->conf2.lcd_tx_sdx2_en = 0;

    // Set to 8 bit parallel output
    dev->sample_rate_conf.val = 0;
    dev->sample_rate_conf.tx_bits_mod = 8;

    // Half speed of bit clock in LCD mode.
    // (Smallest possible divider according to the spec).
    dev->sample_rate_conf.tx_bck_div_num = 2;

    // Initialize Audio Clock (APLL)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rtc_clk_apll_enable(true);
    rtc_clk_apll_coeff_set(0, 0, 0, 8);
#else
    rtc_clk_apll_enable(1, 0, 0, 8, 0);
#endif

    // Set Audio Clock Dividers
    dev->clkm_conf.val = 0;
    dev->clkm_conf.clka_en = 1;
    dev->clkm_conf.clkm_div_a = 1;
    dev->clkm_conf.clkm_div_b = 0;
    // 2 is the smallest possible divider according to the spec.
    dev->clkm_conf.clkm_div_num = 2;

    // Set up FIFO
    dev->fifo_conf.val = 0;
    dev->fifo_conf.tx_fifo_mod_force_en = 1;
    dev->fifo_conf.tx_fifo_mod = 1;
    dev->fifo_conf.tx_data_num = 32;
    dev->fifo_conf.dscr_en = 1;

    // Stop after transmission complete
    dev->conf1.val = 0;
    dev->conf1.tx_stop_en = 0;
    dev->conf1.tx_pcm_bypass = 1;

    // Configure TX channel
    dev->conf_chan.val = 0;
    dev->conf_chan.tx_chan_mod = 1;
    dev->conf.tx_right_first = 1;

    dev->timing.val = 0;

    // Allocate DMA descriptors
    const size_t buf_size = align_up(epd_row_width / 4, 4);  // Buf size must be word aligned
    i2s_state.buf_a = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    i2s_state.buf_b = heap_caps_malloc(buf_size, MALLOC_CAP_DMA);
    i2s_state.dma_desc_a = heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);
    i2s_state.dma_desc_b = heap_caps_malloc(sizeof(lldesc_t), MALLOC_CAP_DMA);

    // and fill them
    fill_dma_desc(i2s_state.dma_desc_a, i2s_state.buf_a, epd_row_width / 4, buf_size);
    fill_dma_desc(i2s_state.dma_desc_b, i2s_state.buf_b, epd_row_width / 4, buf_size);

    // enable "done" interrupt
    SET_PERI_REG_BITS(I2S_INT_ENA_REG(1), I2S_OUT_DONE_INT_ENA_V, 1, I2S_OUT_DONE_INT_ENA_S);
    // register interrupt
    esp_intr_alloc(ETS_I2S1_INTR_SOURCE, 0, i2s_int_hdl, 0, &gI2S_intr_handle);

    // Reset FIFO/DMA
    dev->lc_conf.in_rst = 1;
    dev->lc_conf.out_rst = 1;
    dev->lc_conf.ahbm_rst = 1;
    dev->lc_conf.ahbm_fifo_rst = 1;
    dev->lc_conf.in_rst = 0;
    dev->lc_conf.out_rst = 0;
    dev->lc_conf.ahbm_rst = 0;
    dev->lc_conf.ahbm_fifo_rst = 0;
    dev->conf.tx_reset = 1;
    dev->conf.tx_fifo_reset = 1;
    dev->conf.rx_fifo_reset = 1;
    dev->conf.tx_reset = 0;
    dev->conf.tx_fifo_reset = 0;
    dev->conf.rx_fifo_reset = 0;

    // Start dma on front buffer
    dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
    dev->out_link.addr = ((uint32_t)(i2s_state.dma_desc_a));
    dev->out_link.start = 1;

    dev->int_clr.val = dev->int_raw.val;

    dev->int_ena.val = 0;
    dev->int_ena.out_done = 1;

    dev->conf.tx_start = 0;
}

void i2s_bus_deinit() {
    esp_intr_disable(gI2S_intr_handle);
    esp_intr_free(gI2S_intr_handle);

    free(i2s_state.buf_a);
    free(i2s_state.buf_b);
    free((void*)i2s_state.dma_desc_a);
    free((void*)i2s_state.dma_desc_b);

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    rtc_clk_apll_coeff_set(0, 0, 0, 8);
    rtc_clk_apll_enable(true);
#else
    rtc_clk_apll_enable(0, 0, 0, 8, 0);
#endif

    periph_module_disable(PERIPH_I2S1_MODULE);
}

/**
 * @brief       Function Intializes I2S driver of the ESP32
 *
 * @param       i2s_dev_t *_i2sDev
 *              Pointer of the selected I2S driver
 *
 * @note        Function must be declared static to fit into Instruction RAM of the ESP32.
 */
void IRAM_ATTR I2SInit(i2s_dev_t *_i2sDev, uint8_t _clockDivider)
{
    // Enable I2S peripheral and reset it.
    periph_module_enable(PERIPH_I2S1_MODULE);
    periph_module_reset(PERIPH_I2S1_MODULE);

    // Reset the FIFO Buffer in I2S module.
    _i2sDev->conf.rx_fifo_reset = 1;
    _i2sDev->conf.rx_fifo_reset = 0;
    _i2sDev->conf.tx_fifo_reset = 1;
    _i2sDev->conf.tx_fifo_reset = 0;

    // Reset I2S DMA controller.
    _i2sDev->lc_conf.in_rst = 1;
    _i2sDev->lc_conf.in_rst = 0;
    _i2sDev->lc_conf.out_rst = 1;
    _i2sDev->lc_conf.out_rst = 0;

    // Reset I2S TX and RX module.
    _i2sDev->conf.rx_reset = 1;
    _i2sDev->conf.tx_reset = 1;
    _i2sDev->conf.rx_reset = 0;
    _i2sDev->conf.tx_reset = 0;

    // Set LCD mode on I2S, setup delays on SD and WR lines.
    _i2sDev->conf2.val = 0;
    _i2sDev->conf2.lcd_en = 1;
    _i2sDev->conf2.lcd_tx_wrx2_en = 1;
    _i2sDev->conf2.lcd_tx_sdx2_en = 0;

    _i2sDev->sample_rate_conf.val = 0;
    _i2sDev->sample_rate_conf.rx_bits_mod = 8;
    _i2sDev->sample_rate_conf.tx_bits_mod = 8;
    _i2sDev->sample_rate_conf.rx_bck_div_num = 2;
    _i2sDev->sample_rate_conf.tx_bck_div_num = 2;

    // Do not use APLL, divide by 5 by default, BCK should be ~16MHz.
    _i2sDev->clkm_conf.val = 0;
    _i2sDev->clkm_conf.clka_en = 0;
    _i2sDev->clkm_conf.clkm_div_b = 0;
    _i2sDev->clkm_conf.clkm_div_a = 1;
    _i2sDev->clkm_conf.clkm_div_num = _clockDivider;

    // FIFO buffer setup. Byte packing for FIFO: 0A0B_0B0C = 0, 0A0B_0C0D = 1, 0A00_0B00 = 3. Use dual mono single data
    _i2sDev->fifo_conf.val = 0;
    _i2sDev->fifo_conf.rx_fifo_mod_force_en = 1;
    _i2sDev->fifo_conf.tx_fifo_mod_force_en = 1;
    _i2sDev->fifo_conf.tx_fifo_mod =
        1; // byte packing 0A0B_0B0C = 0, 0A0B_0C0D = 1, 0A00_0B00 = 3. Use dual mono single data
    _i2sDev->fifo_conf.rx_data_num = 1;
    _i2sDev->fifo_conf.tx_data_num = 1;
    _i2sDev->fifo_conf.dscr_en = 1;

    // Send BCK only when needed (needs to be powered on in einkOn() function and disabled in einkOff()).
    _i2sDev->conf1.val = 0;
    _i2sDev->conf1.tx_stop_en = 0;
    _i2sDev->conf1.tx_pcm_bypass = 1;

    _i2sDev->conf_chan.val = 0;
    _i2sDev->conf_chan.tx_chan_mod = 1;
    _i2sDev->conf_chan.rx_chan_mod = 1;

    _i2sDev->conf.tx_right_first = 0; //!!invert_clk; // should be false / 0
    _i2sDev->conf.rx_right_first = 0; //!!invert_clk;

    _i2sDev->timing.val = 0;
} /* I2SInit() */

#endif // old ESP32
