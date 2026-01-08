// main/boards/es3c28p/es3c28p_board.cc

#include "wifi_board.h"
#include "../device_state.h"
#include "audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "board_config.h"
#include "config.h"
#include "settings.h"
#include "es3c28p_audio_codec.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9341.h>
#include <esp_lvgl_port.h>

#define TAG "ES3C28P_Board"

class SimpleBacklight : public Backlight {
public:
    SimpleBacklight(gpio_num_t pin) : pin_(pin) {
        gpio_config_t io_conf = {};
        io_conf.pin_bit_mask = (1ULL << pin_);
        io_conf.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io_conf);
    }
protected:
    virtual void SetBrightnessImpl(uint8_t brightness) override {
        gpio_set_level(pin_, brightness > 0 ? 1 : 0);
    }
private:
    gpio_num_t pin_;
};

class ES3C28PBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Button boot_button_;
    LcdDisplay* display_ = nullptr;
    ES3C28PAudioCodec* audio_codec_ = nullptr;

    void InitializeI2c() {
        gpio_reset_pin((gpio_num_t)16);
        gpio_reset_pin((gpio_num_t)15);
        vTaskDelay(pdMS_TO_TICKS(50));

        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = (gpio_num_t)16,
            .scl_io_num = (gpio_num_t)15,
            .clk_source = I2C_CLK_SRC_DEFAULT,
           // .i2c_clk_speed = 100000,
            .glitch_ignore_cnt = 7,
            .flags = { .enable_internal_pullup = 1 },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = (gpio_num_t)DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = (gpio_num_t)DISPLAY_MISO_PIN;
        buscfg.sclk_io_num = (gpio_num_t)DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = (gpio_num_t)DISPLAY_CS_PIN;
        io_config.dc_gpio_num = (gpio_num_t)DISPLAY_DC_PIN;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 10 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = (gpio_num_t)DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;

        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);

        display_ = new SpiLcdDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                     DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                     DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

public:
    ES3C28PBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        gpio_reset_pin((gpio_num_t)1);
        gpio_set_direction((gpio_num_t)1, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)1, 1);

        InitializeI2c();
        InitializeSpi();
        InitializeLcdDisplay();

        gpio_reset_pin((gpio_num_t)45);
        gpio_set_direction((gpio_num_t)45, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)45, 1);

        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle || app.GetDeviceState() == kDeviceStateListening) {
                ESP_LOGI(TAG, "Triggering manual listen via button");
                app.StartListening();
            }
        });

        ESP_LOGI(TAG, "ES3C28P Board initialized");
    }

    virtual AudioCodec* GetAudioCodec() override {
        if (audio_codec_ == nullptr) {
            audio_codec_ = new ES3C28PAudioCodec(4, 5, 7, 8, 6, 1);
            audio_codec_->Config(i2c_bus_);
        }
        return audio_codec_;
    }

    virtual Display* GetDisplay() override { return display_; }
    virtual Backlight* GetBacklight() override {
        static SimpleBacklight backlight((gpio_num_t)45);
        return &backlight;
    }
};

DECLARE_BOARD(ES3C28PBoard);