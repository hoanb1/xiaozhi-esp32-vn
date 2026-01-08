// main/boards/es3c28p/es3c28p_audio_codec.cc

#include "es3c28p_audio_codec.h"
#include <esp_log.h>
#include <esp_codec_dev_defaults.h>
#include <driver/gpio.h>

#define TAG "ES3C28P_Codec"

ES3C28PAudioCodec::ES3C28PAudioCodec(int mclk, int bclk, int ws, int dout, int din, int pa_pin)
    : mclk_pin_((int16_t)mclk), bclk_pin_((int16_t)bclk), ws_pin_((int16_t)ws),
      dout_pin_((int16_t)dout), din_pin_((int16_t)din), pa_pin_((int16_t)pa_pin) {
    input_sample_rate_ = 16000;
    output_sample_rate_ = 24000;
    InitializeI2S();
}

ES3C28PAudioCodec::~ES3C28PAudioCodec() {
    if (dev_) esp_codec_dev_delete(dev_);
}

void ES3C28PAudioCodec::InitializeI2S() {
    if (initialized_) return;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    // Tối ưu DMA để tiết kiệm RAM nội bộ cho UDP
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle_, &rx_handle_));

    uint32_t sample_rate_hz = (uint32_t)output_sample_rate_;
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz),
        // ES8311 cần Slot Stereo để đồng bộ Clock tốt hơn
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

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &std_cfg));

    i2s_channel_enable(tx_handle_);
    i2s_channel_enable(rx_handle_);
    initialized_ = true;
}

void ES3C28PAudioCodec::Config(i2c_master_bus_handle_t i2c_bus) {
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = rx_handle_,
        .tx_handle = tx_handle_,
    };
    const audio_codec_data_if_t* data_if = audio_codec_new_i2s_data(&i2s_cfg);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM_0,
        .addr = 0x18,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t* ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = audio_codec_new_gpio(),
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = pa_pin_,
        .pa_reverted = false,
    };
    const audio_codec_if_t* codec_if = es8311_codec_new(&es8311_cfg);

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec_if,
        .data_if = data_if,
    };
    dev_ = esp_codec_dev_new(&dev_cfg);

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 2,
        .sample_rate = (uint32_t)output_sample_rate_,
        .mclk_multiple = 256,
    };

    if (esp_codec_dev_open(dev_, &fs) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open codec, retrying...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_codec_dev_open(dev_, &fs);
    }

    esp_codec_dev_set_out_vol(dev_, 85);
    esp_codec_dev_set_in_gain(dev_, 42);
    ESP_LOGI(TAG, "Audio hardware forced configuration finished");
}

int ES3C28PAudioCodec::Write(const int16_t* data, int samples) {
    if (output_enabled_ && dev_) {
        // Kiểm tra xem dữ liệu có được đẩy ra loa không
        static int write_cnt = 0;
        if (write_cnt++ % 100 == 0) ESP_LOGD(TAG, "Speaker active");
        esp_codec_dev_write(dev_, (void*)data, samples * sizeof(int16_t));
    }
    return samples;
}

int ES3C28PAudioCodec::Read(int16_t* dest, int samples) {
    if (input_enabled_ && dev_) {
        esp_codec_dev_read(dev_, (void*)dest, samples * sizeof(int16_t));
    }
    return samples;
}

void ES3C28PAudioCodec::SetOutputVolume(int volume) {
    if (dev_) esp_codec_dev_set_out_vol(dev_, volume);
    AudioCodec::SetOutputVolume(volume);
}

void ES3C28PAudioCodec::SetInputGain(float gain_db) {
    if (dev_) esp_codec_dev_set_in_gain(dev_, (int)gain_db);
}

void ES3C28PAudioCodec::EnableOutput(bool enable) {
    output_enabled_ = enable;
    if (pa_pin_ != -1) {
        // GPIO 1: Board ES3C28P thường cần mức LOW (0) để bật PA
        gpio_set_level((gpio_num_t)pa_pin_, enable ? 0 : 1);
    }
    if (dev_) esp_codec_dev_set_out_mute(dev_, !enable);
}

void ES3C28PAudioCodec::EnableInput(bool enable) {
    input_enabled_ = enable;
    if (dev_) esp_codec_dev_set_in_mute(dev_, !enable);
}

void ES3C28PAudioCodec::Start() {
    EnableOutput(true);
    EnableInput(true);
}

void ES3C28PAudioCodec::Stop() {
    EnableOutput(false);
    EnableInput(false);
}