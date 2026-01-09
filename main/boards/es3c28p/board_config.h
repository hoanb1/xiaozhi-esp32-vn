/* File: main/boards/es3c28p/board_config.h */
#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

/* Audio Settings */
#define AUDIO_INPUT_SAMPLE_RATE     16000
#define AUDIO_OUTPUT_SAMPLE_RATE    16000

/* I2S Audio Pins - Matched with Demo Log */
#define AUDIO_I2S_GPIO_MCLK         GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK         GPIO_NUM_5
#define AUDIO_I2S_GPIO_WS           GPIO_NUM_7
#define AUDIO_I2S_GPIO_DOUT         GPIO_NUM_8
#define AUDIO_I2S_GPIO_DIN          GPIO_NUM_6
#define AUDIO_CODEC_PA_PIN          GPIO_NUM_1

/* LCD Pins (ILI9341 via SPI) */
#define DISPLAY_WIDTH               320
#define DISPLAY_HEIGHT              240
#define DISPLAY_MOSI_PIN            GPIO_NUM_11
#define DISPLAY_MISO_PIN            GPIO_NUM_13
#define DISPLAY_CLK_PIN             GPIO_NUM_12
#define DISPLAY_CS_PIN              GPIO_NUM_10
#define DISPLAY_DC_PIN              GPIO_NUM_46
#define DISPLAY_RST_PIN             GPIO_NUM_NC
#define DISPLAY_BACKLIGHT_PIN       GPIO_NUM_45

/* FIXED MIRROR ISSUE: Adjusted to fix per-character mirroring */
#define DISPLAY_SWAP_XY             true
#define DISPLAY_MIRROR_X            true
#define DISPLAY_MIRROR_Y            true
#define DISPLAY_INVERT_COLOR        false
#define DISPLAY_RGB_ORDER           LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X            0
#define DISPLAY_OFFSET_Y            0

/* Touch Screen Pins (FT6336G via I2C) */
#define TOUCH_I2C_SDA               GPIO_NUM_16
#define TOUCH_I2C_SCL               GPIO_NUM_15
#define TOUCH_RST_PIN               GPIO_NUM_18
#define TOUCH_INT_PIN               GPIO_NUM_17

/* Peripheral Pins */
#define BUILTIN_LED_GPIO            GPIO_NUM_42
#define BOOT_BUTTON_GPIO            GPIO_NUM_0
#define BATTERY_ADC_GPIO            GPIO_NUM_9

/* SD Card (SDIO/QSPI) */
#define SD_CARD_CLK                 GPIO_NUM_38
#define SD_CARD_CMD                 GPIO_NUM_40
#define SD_CARD_D0                  GPIO_NUM_39
#define SD_CARD_D1                  GPIO_NUM_41
#define SD_CARD_D2                  GPIO_NUM_48
#define SD_CARD_D3                  GPIO_NUM_47

#endif /* _BOARD_CONFIG_H_ */