/**
 * File: main/boards/es3c28p/es3c28p_audio_codec.h
 */
#ifndef _ES3C28P_AUDIO_CODEC_H_
#define _ES3C28P_AUDIO_CODEC_H_

#include "audio_codec.h"
#include <driver/i2s_std.h>
#include <driver/i2c_master.h>
#include <esp_err.h>

#define ES8311_I2C_ADDR 0x18

class ES3C28PAudioCodec : public AudioCodec {
public:
    ES3C28PAudioCodec(int mclk, int bclk, int ws, int dout, int din, int pa_pin);
    virtual ~ES3C28PAudioCodec();

    virtual int Read(int16_t* dest, int samples) override;
    virtual int Write(const int16_t* data, int samples) override;
    virtual void SetOutputVolume(int volume) override;
    virtual void SetInputGain(float gain_db) override;
    virtual void EnableInput(bool enable) override;
    virtual void EnableOutput(bool enable) override;
    virtual void Start() override;
    virtual void Stop();

    void Config(i2c_master_bus_handle_t i2c_bus);

private:
    int mclk_pin_, bclk_pin_, ws_pin_, dout_pin_, din_pin_, pa_pin_;
    bool initialized_ = false;
    i2s_chan_handle_t tx_handle_ = nullptr;
    i2s_chan_handle_t rx_handle_ = nullptr;
    i2c_master_dev_handle_t i2c_device_ = nullptr;

    void InitializeI2S();
    esp_err_t WriteReg(uint8_t reg, uint8_t val);
};

#endif // _ES3C28P_AUDIO_CODEC_H_