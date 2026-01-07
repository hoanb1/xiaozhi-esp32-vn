/* File: main/boards/es3c28p/es3c28p_audio_codec.h */
#ifndef _ES3C28P_AUDIO_CODEC_H_
#define _ES3C28P_AUDIO_CODEC_H_

#include "audio_codec.h"
#include <driver/i2s_std.h>

class ES3C28PAudioCodec : public AudioCodec {
public:
    ES3C28PAudioCodec(int mclk, int bclk, int ws, int dout, int din);
    virtual ~ES3C28PAudioCodec();

    /* Core methods required by AudioCodec base class */
    virtual int Read(int16_t* dest, int samples) override;
    virtual int Write(const int16_t* data, int samples) override;

    virtual void SetOutputVolume(int volume) override;
    virtual void SetInputGain(float gain) override;
    virtual void EnableInput(bool enable) override;
    virtual void EnableOutput(bool enable) override;
    virtual void Start() override;

private:
    int mclk_pin_, bclk_pin_, ws_pin_, dout_pin_, din_pin_;
    bool initialized_ = false;
    i2s_chan_handle_t tx_handle_ = nullptr;
    i2s_chan_handle_t rx_handle_ = nullptr;

    void InitializeI2S();
};

#ifdef __cplusplus
extern "C" {
#endif
AudioCodec* CreateGenericAudioCodec(int mclk, int bclk, int ws, int dout, int din);

#ifdef __cplusplus
}
#endif

#endif // _ES3C28P_AUDIO_CODEC_H_
