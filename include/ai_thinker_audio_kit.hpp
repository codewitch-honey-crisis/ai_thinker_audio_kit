#pragma once
#ifndef ESP_PLATFORM
#error "This library requires an AI-Thinker ESP32 Audio Kit 2.2"
#endif

#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <inttypes.h>
#include <stddef.h>
#endif
constexpr static const uint8_t pin_led = 22;
constexpr static const uint8_t pin_i2c_sda = 33;
constexpr static const uint8_t pin_i2c_scl = 32;
constexpr static const uint8_t pin_pa_en = 21;
constexpr static const uint8_t pin_i2s_sclk = 27;
constexpr static const uint8_t pin_i2s_lrclk = 26;
constexpr static const uint8_t pin_i2s_din = 35;
constexpr static const uint8_t pin_i2s_dout = 25;
