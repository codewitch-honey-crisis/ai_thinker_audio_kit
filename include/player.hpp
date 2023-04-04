#ifndef ESP_PLATFORM
#error "This library requires an ESP32"
#endif
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <stddef.h>
#endif
typedef void* voice_handle_t;
typedef void (*on_sound_disable_callback)(void* state);
typedef void (*on_sound_enable_callback)(void* state);
typedef void (*on_flush_callback)(const void* buffer, size_t buffer_size, void* state);
typedef int (*on_read_stream_callback)(void* state);
typedef void (*on_seek_stream_callback)(unsigned long long pos, void* state);
class player final {
    void* m_task;
    void* m_sync;
    voice_handle_t m_first;
    void* m_buffer;
    size_t m_frame_count;
    unsigned int m_sample_rate;
    unsigned int m_channels;
    unsigned int m_bit_depth;
    unsigned int m_sample_max;
    bool m_sound_enabled;
    on_sound_disable_callback m_on_sound_disable_cb;
    void* m_on_sound_disable_state;
    on_sound_enable_callback m_on_sound_enable_cb;
    void* m_on_sound_enable_state;
    on_flush_callback m_on_flush_cb;
    void* m_on_flush_state;
    player(const player& rhs)=delete;
    player& operator=(const player& rhs)=delete;
    void do_move(player& rhs);
    static void task(void* state);
    bool realloc_buffer();
public:
    player(unsigned int sample_rate = 44100, unsigned short channels = 2, unsigned short bit_depth = 16, size_t frame_count = 512);
    player(player&& rhs);
    ~player();
    player& operator=(player&& rhs);
    bool initialized() const;
    bool initialize();
    void deinitialize();
    voice_handle_t sin(float frequency, float amplitude = .8);
    voice_handle_t sqr(float frequency, float amplitude = .8);
    voice_handle_t saw(float frequency, float amplitude = .8);
    voice_handle_t tri(float frequency, float amplitude = .8);
    voice_handle_t wav(on_read_stream_callback on_read_stream, void* on_read_stream_state, float amplitude = .8, bool loop = false,on_seek_stream_callback on_seek_stream = nullptr, void* on_seek_stream_state=nullptr);
    bool stop(voice_handle_t handle = nullptr);
    void on_sound_disable(on_sound_disable_callback cb, void* state=nullptr);
    void on_sound_enable(on_sound_enable_callback cb, void* state=nullptr);
    void on_flush(on_flush_callback cb, void* state=nullptr);
    size_t frame_count() const;
    bool frame_count(size_t value);
    unsigned int sample_rate() const;
    bool sample_rate(unsigned int value);
    unsigned short channels() const;
    bool channels(unsigned short value);
    unsigned short bit_depth() const;
    bool bit_depth(unsigned short value);
};