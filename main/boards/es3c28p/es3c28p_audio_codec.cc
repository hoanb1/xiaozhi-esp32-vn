/**
 * File: main/boards/es3c28p/es3c28p_audio_codec.cc
 */
#include "es3c28p_audio_codec.h"
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>

static const char* TAG = "ES3C28P_Codec";
#define I2C_TIMEOUT_MS 1000

ES3C28PAudioCodec::ES3C28PAudioCodec(int mclk, int bclk, int ws, int dout, int din, int pa_pin)
    : mclk_pin_(mclk), bclk_pin_(bclk), ws_pin_(ws), dout_pin_(dout), din_pin_(din), pa_pin_(pa_pin) {
    // Phải khởi tạo sample rate để tránh lỗi Divide By Zero trong AudioService/Resampler
    input_sample_rate_ = 16000;
    output_sample_rate_ = 16000;
    initialized_ = false;
    output_enabled_ = false;
}

ES3C28PAudioCodec::~ES3C28PAudioCodec() {
    Stop();
    if (i2c_device_) {
        i2c_master_bus_rm_device(i2c_device_);
    }
}

esp_err_t ES3C28PAudioCodec::WriteReg(uint8_t reg, uint8_t val) {
    if (!i2c_device_) return ESP_ERR_INVALID_STATE;
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(i2c_device_, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

void ES3C28PAudioCodec::Config(i2c_master_bus_handle_t i2c_bus) {
    if (i2c_bus == nullptr) return;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ES8311_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    if (i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_device_) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ES8311 to I2C bus");
        return;
    }

    WriteReg(0x00, 0x1F);
    vTaskDelay(pdMS_TO_TICKS(50));
    WriteReg(0x00, 0x00);
    WriteReg(0x01, 0x30);
    WriteReg(0x02, 0x10);
    WriteReg(0x03, 0x10);
    WriteReg(0x04, 0x10);
    WriteReg(0x05, 0x00);
    WriteReg(0x10, 0x0C);
    WriteReg(0x11, 0x0C);
    WriteReg(0x0D, 0x02);
    WriteReg(0x0E, 0x02);
    WriteReg(0x14, 0x1A);
    WriteReg(0x12, 0x00);

    SetOutputVolume(90);
    ESP_LOGI(TAG, "ES8311 initialized. PA Pin: %d (Active LOW)", pa_pin_);
}

void ES3C28PAudioCodec::InitializeI2S() {
    if (initialized_) return;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)mclk_pin_,
            .bclk = (gpio_num_t)bclk_pin_,
            .ws = (gpio_num_t)ws_pin_,
            .dout = (gpio_num_t)dout_pin_,
            .din = (gpio_num_t)din_pin_,
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, &rx_handle_));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle_));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle_));

    initialized_ = true;
}

int ES3C28PAudioCodec::Read(int16_t* dest, int samples) {
    if (!initialized_ || !rx_handle_) return 0;
    size_t bytes_read = 0;
    i2s_channel_read(rx_handle_, dest, samples * sizeof(int16_t), &bytes_read, portMAX_DELAY);
    return bytes_read / sizeof(int16_t);
}

int ES3C28PAudioCodec::Write(const int16_t* data, int samples) {
    if (!initialized_ || !tx_handle_ || !output_enabled_) return 0;
    size_t bytes_written = 0;
    i2s_channel_write(tx_handle_, data, samples * sizeof(int16_t), &bytes_written, pdMS_TO_TICKS(100));
    return bytes_written / sizeof(int16_t);
}

void ES3C28PAudioCodec::SetOutputVolume(int volume) {
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    if (!i2c_device_) return;
    uint8_t reg_val = (100 - volume) * 192 / 100;
    WriteReg(0x32, reg_val);
}

void ES3C28PAudioCodec::SetInputGain(float gain_db) {
    if (!i2c_device_) return;
    uint8_t reg_val = (uint8_t)((gain_db + 12.0f) * 4.0f / 3.0f + 0.5f);
    WriteReg(0x0E, (0x0F & reg_val) | 0x10);
}

void ES3C28PAudioCodec::EnableInput(bool enable) {
    if (!i2c_device_) return;
    WriteReg(0x13, enable ? 0x00 : 0x02);
}

void ES3C28PAudioCodec::EnableOutput(bool enable) {
    if (!i2c_device_) return;
    if (enable) {
        WriteReg(0x31, 0x00);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level((gpio_num_t)pa_pin_, 0); // Active LOW: 0 is ON
    } else {
        gpio_set_level((gpio_num_t)pa_pin_, 1); // 1 is OFF
        WriteReg(0x31, 0x04);
    }
    output_enabled_ = enable;
}

void ES3C28PAudioCodec::Start() {
    if (!initialized_) InitializeI2S();
    EnableInput(true);
    EnableOutput(true);
}

void ES3C28PAudioCodec::Stop() {
    EnableOutput(false);
    EnableInput(false);
}