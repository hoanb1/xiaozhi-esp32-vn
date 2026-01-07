/**
 * File: main/boards/es3c28p/es3c28p_audio_codec.cc
 */
#include "es3c28p_audio_codec.h"
#include <esp_log.h>
#include <esp_check.h>

static const char* TAG = "ES3C28PAudioCodec";

ES3C28PAudioCodec::ES3C28PAudioCodec(int mclk, int bclk, int ws, int dout, int din)
    : mclk_pin_(mclk), bclk_pin_(bclk), ws_pin_(ws), dout_pin_(dout), din_pin_(din) {
    // Set default sample rates to prevent divide by zero in resampler
    input_sample_rate_ = 16000;
    output_sample_rate_ = 16000;
}

ES3C28PAudioCodec::~ES3C28PAudioCodec() {
    if (tx_handle_) i2s_del_channel(tx_handle_);
    if (rx_handle_) i2s_del_channel(rx_handle_);
}

void ES3C28PAudioCodec::InitializeI2S() {
    if (initialized_) return;
    
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t std_cfg = {
        /* Fix narrowing conversion error by explicit casting to uint32_t */
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)output_sample_rate_),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)mclk_pin_,
            .bclk = (gpio_num_t)bclk_pin_,
            .ws = (gpio_num_t)ws_pin_,
            .dout = (gpio_num_t)dout_pin_,
            .din = (gpio_num_t)din_pin_
        },
    };
    
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, &rx_handle_));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));
    initialized_ = true;
}

/* Implements pure virtual Read */
int ES3C28PAudioCodec::Read(int16_t* dest, int samples) {
    if (!rx_handle_) return 0;
    size_t bytes_read = 0;
    if (i2s_channel_read(rx_handle_, dest, samples * sizeof(int16_t), &bytes_read, portMAX_DELAY) != ESP_OK) {
        return 0;
    }
    return (int)(bytes_read / sizeof(int16_t));
}

/* Implements pure virtual Write */
int ES3C28PAudioCodec::Write(const int16_t* data, int samples) {
    if (!tx_handle_) return 0;
    size_t bytes_written = 0;
    if (i2s_channel_write(tx_handle_, data, samples * sizeof(int16_t), &bytes_written, portMAX_DELAY) != ESP_OK) {
        return 0;
    }
    return (int)(bytes_written / sizeof(int16_t));
}

void ES3C28PAudioCodec::SetOutputVolume(int volume) {}
void ES3C28PAudioCodec::SetInputGain(float gain) {}
void ES3C28PAudioCodec::EnableInput(bool enable) {}
void ES3C28PAudioCodec::EnableOutput(bool enable) {}

void ES3C28PAudioCodec::Start() {
    InitializeI2S();
    if (tx_handle_) i2s_channel_enable(tx_handle_);
    if (rx_handle_) i2s_channel_enable(rx_handle_);
    ESP_LOGI(TAG, "Audio codec started at %d Hz", output_sample_rate_);
    }

extern "C" AudioCodec* CreateGenericAudioCodec(int mclk, int bclk, int ws, int dout, int din) {
    return new ES3C28PAudioCodec(mclk, bclk, ws, dout, din);
}