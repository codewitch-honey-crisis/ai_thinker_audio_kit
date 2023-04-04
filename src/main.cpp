#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <stdio.h>
#include <string.h>
#include <driver/i2c.h>
#include <math.h>
#endif
#include <ai_thinker_audio_kit.hpp>
#include <ac101.hpp>
#include <driver/i2s.h>
#include <player.hpp>
#include <demo.hpp>
#ifdef ARDUINO
using namespace arduino;
#else
void loop();
using namespace esp_idf;
#endif

ac101 audio;
player sound;
uint16_t i2s_out_buf[128];

size_t demo_len = sizeof(demo_data);
size_t demo_pos = 0;
int read_demo(void* state) {
    if(demo_pos>=demo_len) {
        return -1;
    }
    return demo_data[demo_pos++];
}
void seek_demo(unsigned long long position, void* state) {
    demo_pos = position;
}
#ifdef ARDUINO
void setup() {
    Serial.begin(115200);
#else
extern "C" void app_main() {
#endif
#ifdef ARDUINO
    Wire.begin(pin_i2c_sda,pin_i2c_scl,100*1000);
#else
    i2c_config_t i2c_cfg;
    memset(&i2c_cfg,0,sizeof(i2c_cfg));
    i2c_cfg.master.clk_speed = 100*1000;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.scl_io_num=(gpio_num_t)pin_i2c_scl;
    i2c_cfg.scl_pullup_en = true;
    i2c_cfg.sda_io_num=(gpio_num_t)pin_i2c_sda;
    i2c_cfg.sda_pullup_en = true;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, i2c_cfg.mode, 0, 0, 0));
#endif
    if(!audio.initialize()) {
        printf("Audio initialization failure.");
    }
    gpio_set_direction((gpio_num_t)pin_pa_en,GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)pin_pa_en,1);
    audio.i2s_mode(ac101_i2s_mode::slave);
    audio.bit_depth(ac101_bit_depth::bps16);
    audio.sample_rate(ac101_sample_rate::khz44100);
    audio.start(ac101_mode::dac);
    i2s_config_t i2s_cfg;
    memset(&i2s_cfg,0,sizeof(i2s_config_t));
    i2s_cfg.bits_per_chan = I2S_BITS_PER_CHAN_16BIT;
    i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    i2s_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_cfg.dma_buf_count = 2;
    i2s_cfg.dma_buf_len = sizeof(i2s_out_buf)/sizeof(uint16_t);
    i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    i2s_cfg.sample_rate = 44100;
    i2s_cfg.use_apll= true;
    //i2s_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT;
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1,&i2s_cfg,8,nullptr));
    i2s_pin_config_t i2s_pin_cfg;
    memset(&i2s_pin_cfg,0,sizeof(i2s_pin_config_t));
    i2s_pin_cfg.ws_io_num = pin_i2s_lrclk;
    i2s_pin_cfg.bck_io_num = pin_i2s_sclk;
    i2s_pin_cfg.data_out_num = pin_i2s_dout;
    i2s_pin_cfg.data_in_num = I2S_PIN_NO_CHANGE;
    i2s_pin_cfg.mck_io_num = GPIO_NUM_0;
    ESP_ERROR_CHECK(i2s_set_pin(I2S_NUM_1,&i2s_pin_cfg));

    if(!sound.initialize()) {
        printf("Sound initialization failure.\n");    
    }
    sound.on_flush([](const void* buffer,size_t buffer_size,void* state){
        size_t written;
        i2s_write(I2S_NUM_1,buffer,buffer_size,&written,portMAX_DELAY);
    });
    sound.on_sound_disable([](void* state) {
        i2s_zero_dma_buffer(I2S_NUM_1);
    });
   voice_handle_t v1= sound.sin(120,.1);
    voice_handle_t v3=sound.wav(read_demo,nullptr,.5,true,seek_demo,nullptr);
   //voice_handle_t v2=sound.sqr(1000,.05);
   
    
#ifndef ARDUINO
    while(1) {
        loop();
        vTaskDelay(5);
    }
#endif
}
void loop() {
  
}
