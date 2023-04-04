#pragma once
#if __has_include(<Arduino.h>) && __has_include(<Wire.h>)
#include <Arduino.h>
#include <Wire.h>
namespace arduino {
#else
#ifndef ESP_PLATFORM
#error "This library requires Arduino or an ESP32"
#endif
#include <driver/i2c.h>
#include <inttypes.h>
#include <stddef.h>
namespace esp_idf {
#endif


enum struct ac101_sample_rate {
    khz8000 = 0x0000,
    khz11052 = 0x1000,
    khz12000 = 0x2000,
    khz16000 = 0x3000,
    khz22050 = 0x4000,
    khz24000 = 0x5000,
    khz32000 = 0x6000,
    khz44100 = 0x7000,
    khz48000 = 0x8000,
    khz96000 = 0x9000,
    khz192000 = 0xa000,
};
enum struct ac101_bit_depth {
    bps8 = 0x00,
    bps16 = 0x01,
    bps20 = 0x02,
    bps24 = 0x03,
};
enum struct ac101_i2s_mode {
    master = 0,
    slave = 1,
};
enum struct ac101_format {
    i2s = 0x00,
    left = 0x01,
    right = 0x02,
    dsp = 0x03,
};

enum struct ac101_bclk_div {
    bclk_div_1 = 0x0,
    bclk_div_2 = 0x1,
    bclk_div_4 = 0x2,
    bclk_div_6 = 0x3,
    bclk_div_8 = 0x4,
    bclk_div_12 = 0x5,
    bclk_div_16 = 0x6,
    bclk_div_24 = 0x7,
    bclk_div_32 = 0x8,
    bclk_div_48 = 0x9,
    bclk_div_64 = 0xa,
    bclk_div_96 = 0xb,
    bclk_div_128 = 0xc,
    bclk_div_192 = 0xd,
};

enum struct ac101_lrclk_div {
    lrclk_div_16 = 0x0,
    lrclk_div_32 = 0x1,
    lrclk_div_64 = 0x2,
    lrclk_div_128 = 0x3,
    lrclk_div_256 = 0x4,
};
enum struct ac101_mode {
		adc,
		dac,
		adc_dac,
		line
};

typedef enum ac101_mixer_source {
	mic1	= 1,
	mic2	= 2,
	line_in	= 3,
}ac_output_mixer_source_t;

enum struct ac101_mixer_gain {
    dbneg45 = 0,
    dbneg30 = 1,
    dbneg15 = 2,
    db0   = 3,
    db15  = 4,
    db30  = 5,
    db45  = 6,
    db60  = 7,
};

class ac101 {
#ifdef ARDUINO
    TwoWire& m_i2c;
#else
    i2c_port_t m_i2c;
#endif
    bool m_initialized;
    ac101(const ac101& rhs) = delete;
    ac101& operator=(const ac101& rhs) = delete;
    void do_move(ac101& rhs);

   public:
#ifdef ARDUINO
    ac101(TwoWire& i2c = Wire);
#else
    ac101(i2c_port_t i2c = I2C_NUM_0);
#endif
    ac101(ac101&& rhs);
    ac101& operator=(ac101&& rhs);
    bool initialized() const;
    bool initialize();
    float speaker_volume() const;
    void speaker_volume(float value);
    float headphone_volume() const;
    void headphone_volume(float value);
    void sample_rate(ac101_sample_rate value);
    void i2s_mode(ac101_i2s_mode value);
    void clock(ac101_bclk_div bclk_div, bool bclk_inv, ac101_lrclk_div lrclk_div, bool lrclk_inv);
    void bit_depth(ac101_bit_depth value);
    void format(ac101_format value);
    void mixer_source(ac101_mixer_source source,ac101_mixer_gain gain);
    void start(ac101_mode mode);
    void stop();
};
}  // namespace esp_idf