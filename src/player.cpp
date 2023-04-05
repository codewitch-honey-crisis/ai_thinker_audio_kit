#include <player.hpp>
#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
#include <inttypes.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#define PI (3.1415926535f)
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
constexpr static const float player_pi = PI;
constexpr static const float player_two_pi = player_pi*2.0f;

typedef struct voice_func_info {
    void* buffer;
    size_t frame_count;
    unsigned int channels;
    unsigned int bit_depth;
    unsigned int sample_max;
} voice_func_info_t;
typedef void (*voice_func_t)(const voice_func_info_t& info, void* state);
typedef struct voice {
    voice_func_t fn;
    void* fn_state;
    voice* next;
} voice_t;
typedef struct {
    float frequency;
    float amplitude;
    float phase;
    float phase_delta;
} waveform_info_t;
typedef struct wav_info {
    on_read_stream_callback on_read_stream;
    void* on_read_stream_state;
    on_seek_stream_callback on_seek_stream;
    void* on_seek_stream_state;
    float amplitude;
    bool loop;
    unsigned short channels;
    unsigned short bit_depth;
    unsigned long long start;
    unsigned long long length;
    unsigned long long pos;
} wav_info_t;
static void player_yield() {
    static uint32_t last_yield = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    if(ms>last_yield+500) {
        last_yield = ms;
        vTaskDelay(5);
    }
}
static bool player_read32(on_read_stream_callback on_read_stream, void* on_read_stream_state,uint32_t* out) {
    uint32_t res = 0;
    int v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    uint32_t t = (uint32_t)v;
    res|=(t);
    v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    t = (uint32_t)v;
    res|=(t<<8);
    v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    t = (uint32_t)v;
    res|=(t<<16);
    v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    t = (uint32_t)v;
    res|=(t<<24);
    *out = res;
    return true;
}
/*
static bool player_read24(on_read_stream_callback on_read_stream, void* on_read_stream_state,uint32_t* out) {
    uint32_t res = 0;
    int v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    uint32_t t = (uint32_t)v;
    res|=(t);
    v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    t = (uint32_t)v;
    res|=(t<<8);
    v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    t = (uint32_t)v;
    res|=(t<<16);
    *out = res;
    return true;
}
*/
static bool player_read16(on_read_stream_callback on_read_stream, void* on_read_stream_state,uint16_t* out) {
    uint16_t res = 0;
    int v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    uint16_t t = (uint16_t)v;
    res|=t;
    v = on_read_stream(on_read_stream_state);
    if(v==-1) {
        return false;
    }
    t = (uint16_t)v;
    res|=(t<<8);
    *out = res;
    return true;
}
static bool player_read16s(on_read_stream_callback on_read_stream, void* on_read_stream_state,int16_t* out) {
    uint16_t res = 0;
    if(player_read16(on_read_stream,on_read_stream_state,&res)) {
        *out = (int16_t)res;
    }
    return true;
}
static bool player_read_fourcc(on_read_stream_callback on_read_stream, void* on_read_stream_state, char* buf) {
    for(int i = 0;i<4;++i) {
        int v = on_read_stream(on_read_stream_state);
        if(v==-1) {
            return false;
        }
        *buf++=(char)v;
    }
    return true;
}
static void sin_voice(const voice_func_info_t& info, void*state) {
    waveform_info_t* wi = (waveform_info_t*)state;
    for(int i = 0;i<info.frame_count;++i) {
        float f = (sinf(wi->phase) + 1.0f) * 0.5f;
        wi->phase+=wi->phase_delta;
        if(wi->phase>=player_two_pi) {
            wi->phase-=player_two_pi;
        }
        float samp = (f*wi->amplitude)*info.sample_max;
        switch(info.bit_depth) {
            case 8: {
                uint8_t* p = ((uint8_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            case 16: {
                uint16_t* p = ((uint16_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            default:
            break;
        }
    }
}
static void sqr_voice(const voice_func_info_t& info, void*state) {
    waveform_info_t* wi = (waveform_info_t*)state; 
    for(int i = 0;i<info.frame_count;++i) {
        float f = ((wi->phase>player_pi)+1.0f)*.5f;
        wi->phase+=wi->phase_delta;
        if(wi->phase>=player_two_pi) {
            wi->phase-=player_two_pi;
        }
        float samp = (f*wi->amplitude)*info.sample_max;
        switch(info.bit_depth) {
            case 8: {
                uint8_t* p = ((uint8_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            case 16: {
                uint16_t* p = ((uint16_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            default:
            break;
        }
    }    
}

static void saw_voice(const voice_func_info_t& info, void*state) {
    waveform_info_t* wi = (waveform_info_t*)state;
    for(int i = 0;i<info.frame_count;++i) {
        float f = (wi->phase>=0.0f);
        if (wi->phase + wi->phase_delta <= -(player_pi) || wi->phase + wi->phase_delta >= (player_pi)) {
            wi->phase_delta=-wi->phase_delta;
        }
        wi->phase+=wi->phase_delta;
        float samp = (f*wi->amplitude)*info.sample_max;
        switch(info.bit_depth) {
            case 8: {
                uint8_t* p = ((uint8_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            case 16: {
                uint16_t* p = ((uint16_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            default:
            break;
        }
    }
}
static void tri_voice(const voice_func_info_t& info, void*state) {
    waveform_info_t* wi = (waveform_info_t*)state;
    for(int i = 0;i<info.frame_count;++i) {
        float f = ((wi->phase / (player_pi)) + 1.0f) * .5f;
        if (wi->phase + wi->phase_delta <= -(player_pi) || wi->phase + wi->phase_delta >= (player_pi)) {
            wi->phase_delta=-wi->phase_delta;
        }
        wi->phase+=wi->phase_delta;
        float samp = (f*wi->amplitude)*info.sample_max;
                switch(info.bit_depth) {
            case 8: {
                uint8_t* p = ((uint8_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            case 16: {
                uint16_t* p = ((uint16_t*)info.buffer)+(i*info.channels);
                uint32_t tmp = *p+roundf(samp);
                if(tmp>info.sample_max) {
                    tmp = info.sample_max;
                }
                for(int j = 0;j<info.channels;++j) {
                    *p++=tmp;
                }
            }
            break;
            default:
            break;
        }
    }
}
static void wav_voice_16_2_to_16_2(const voice_func_info_t& info, void*state) {
    wav_info_t* wi = (wav_info_t*)state;
    if(!wi->loop&&wi->pos>=wi->length) {
        return;
    }
    uint16_t* dst = (uint16_t*)info.buffer;
    for(int i = 0;i<info.frame_count;++i) {
        int16_t i16;
        
        if(wi->pos>=wi->length) {
            if(!wi->loop) {
                break;
            }
            wi->on_seek_stream(wi->start,wi->on_seek_stream_state);
            wi->pos = 0;
        }
        for(int j=0;j<info.channels;++j) {
            if(player_read16s(wi->on_read_stream,wi->on_read_stream_state,&i16)) {
                wi->pos+=2;
            } else {
                break;
            }
            *dst+=(uint16_t)((i16+32768U)*wi->amplitude);
            ++dst;
        }
    }
}
static void wav_voice_16_1_to_16_2(const voice_func_info_t& info, void*state) {
    wav_info_t* wi = (wav_info_t*)state;
    if(!wi->loop&&wi->pos>=wi->length) {
        return;
    }
    uint16_t* dst = (uint16_t*)info.buffer;
    for(int i = 0;i<info.frame_count;++i) {
        int16_t i16;
        if(wi->pos>=wi->length) {
            if(!wi->loop) {
                break;
            }
            wi->on_seek_stream(wi->start,wi->on_seek_stream_state);
            wi->pos = 0;
        }
        if(player_read16s(wi->on_read_stream,wi->on_read_stream_state,&i16)) {
            wi->pos+=2;
        } else {
            break;
        }
        uint16_t u16 = (uint16_t)((i16+32768U)*wi->amplitude);
        for(int j=0;j<info.channels;++j) {
            *dst+=u16;
            ++dst;
        }
    }
}
static voice_handle_t player_add_voice( voice_handle_t* in_out_first, voice_func_t fn, void* fn_state) {
    voice_t** pv = (voice_t**)in_out_first;
    voice_t* v = *pv;
    if(v!=nullptr) {
        while(v->next!=nullptr) {
            v=v->next;
        }
        v->next = (voice_t*)malloc(sizeof(voice_t));
        if(v->next==nullptr) {
            return nullptr;
        }
        v->next->next = nullptr;
        v->next->fn = fn;
        v->next->fn_state = fn_state;
        v=v->next;
    } else {
        *pv = (voice_t*)malloc(sizeof(voice_t));
        v=*pv;
        v->next = nullptr;
        v->fn = fn;
        v->fn_state = fn_state;
    }
   return v;
}
static bool player_remove_voice(voice_handle_t* in_out_first,voice_handle_t handle) {
    voice_t** pv = (voice_t**)in_out_first;
    voice_t* v = *pv;
    if(v==nullptr) {return false;}
    if(handle==v) {
        *pv = v->next;
        if(v->fn_state!=nullptr) {
            free(v->fn_state);
        }
        free(v);
    } else {
        while(v->next!=handle) {
            v=v->next;
            if(v->next==nullptr) {
                return false;
            }
        }
        void* to_free = v->next;
        if(to_free==nullptr) {
            return false;
        }
        void* to_free2 = v->next->fn_state;
        if(v->next->next!=nullptr) {
            v->next = v->next->next;
        } else {
            v->next = nullptr;
        }
        free(to_free);
        free(to_free2);
    }
    return true;
}
void player::task(void* state) {
    player* this_ = (player*)state;
    SemaphoreHandle_t sync = (SemaphoreHandle_t)this_->m_sync;
        
    while(1) {
        xSemaphoreTake(sync,portMAX_DELAY);
        const size_t buffer_size = this_->m_frame_count*this_->m_channels*(this_->m_bit_depth/8);
        voice* first = (voice*)this_->m_first;
        bool has_voices = false;
        voice_func_info_t vinf;
        vinf.buffer = this_->m_buffer;
        vinf.frame_count = this_->m_frame_count;
        vinf.channels = this_->m_channels;
        vinf.bit_depth = this_->m_bit_depth;
        vinf.sample_max = this_->m_sample_max;
        voice* v = first;
        memset(this_->m_buffer,0,buffer_size);
        while(v!=nullptr) {
            has_voices = true;
            v->fn(vinf, v->fn_state);
            v=v->next;
        }
        if(has_voices) {
            if(!this_->m_sound_enabled) {
                if(this_->m_on_sound_enable_cb!=nullptr) {
                    this_->m_on_sound_enable_cb(this_->m_on_sound_enable_state);
                }
                this_->m_sound_enabled = true;   
            }
        } else {
            if(this_->m_sound_enabled) {
                if(this_->m_on_sound_disable_cb!=nullptr) {
                    this_->m_on_sound_disable_cb(this_->m_on_sound_disable_state);
                }
                this_->m_sound_enabled = false;
            }
        }
        if(this_->m_sound_enabled && this_->m_on_flush_cb!=nullptr) {
            this_->m_on_flush_cb(this_->m_buffer, buffer_size, this_->m_on_flush_state);
        }
        xSemaphoreGive(sync);
        player_yield();
        //vTaskDelay(5);
    }
}
void player::do_move(player& rhs) {
    if(rhs.m_sync!=nullptr) {
        xSemaphoreTake((SemaphoreHandle_t)rhs.m_sync,portMAX_DELAY);
    }
    m_task = rhs.m_task;
    rhs.m_task = nullptr;
    m_sync = rhs.m_sync;
    rhs.m_sync = nullptr;
    m_first = rhs.m_first ;
    rhs.m_first = nullptr;
    m_buffer = rhs.m_buffer;
    rhs.m_buffer = nullptr;
    m_frame_count = rhs.m_frame_count;
    rhs.m_frame_count = 0;
    m_sample_rate = rhs.m_sample_rate;
    m_sample_max = rhs.m_sample_max;
    m_sound_enabled = rhs.m_sound_enabled;
    m_on_sound_disable_cb=rhs.m_on_sound_disable_cb;
    rhs.m_on_sound_enable_cb = nullptr;
    m_on_sound_disable_state = rhs.m_on_sound_disable_state;
    m_on_sound_enable_cb = rhs.m_on_sound_enable_cb;
    rhs.m_on_sound_enable_cb = nullptr;
    m_on_flush_cb = rhs.m_on_flush_cb;
    rhs.m_on_flush_cb = nullptr;
    m_on_flush_state = rhs.m_on_flush_state;
    if(m_sync) {
        xSemaphoreGive((SemaphoreHandle_t)rhs.m_sync);
    }
}
player::player(unsigned int sample_rate, unsigned short channels, unsigned short bit_depth, size_t frame_count) : m_task(nullptr),
                m_sync(nullptr),
                m_first(nullptr),
                m_buffer(nullptr),
                m_frame_count(frame_count),
                m_sample_rate(sample_rate),
                m_channels(channels),
                m_bit_depth(bit_depth),
                m_on_sound_disable_cb(nullptr),
                m_on_sound_disable_state(nullptr),
                m_on_sound_enable_cb(nullptr),
                m_on_sound_enable_state(nullptr),
                m_on_flush_cb(nullptr),
                m_on_flush_state(nullptr)
                {
}
player::~player() {
    deinitialize();
}
player::player(player&& rhs) {
    do_move(rhs);    
}
player& player::operator=(player&& rhs) {
    do_move(rhs);
    return *this;
}
bool player::initialized() const { return m_task!=nullptr;}
bool player::initialize() {
    if(m_task!=nullptr) {
        return true;
    }
    m_buffer=malloc(m_frame_count*m_channels*(m_bit_depth/8));
    if(m_buffer==nullptr) {
        return false;
    }
    m_sample_max = powf(2,m_bit_depth)-1;
    m_sync = xSemaphoreCreateMutex();
    if(m_sync==nullptr) {
        free(m_buffer);
        return false;
    }
    TaskHandle_t th=nullptr;
    int affinity = xTaskGetAffinity(xTaskGetCurrentTaskHandle());
    xTaskCreatePinnedToCore(task,"player_task",1024,this,configMAX_PRIORITIES-1,&th,1-affinity);
    if(th==nullptr) {
        free(m_buffer);
        m_buffer = nullptr;
        vSemaphoreDelete((SemaphoreHandle_t)m_sync);
        m_sync = nullptr;
    }
    m_sound_enabled = false;
    return true;
}
void player::deinitialize() {
    stop();
    vTaskDelay(pdMS_TO_TICKS(10)); // avoid a race
    vTaskDelete((TaskHandle_t)m_task);
    m_task = nullptr;
    vSemaphoreDelete((SemaphoreHandle_t)m_sync);
    m_sync = nullptr;
    free(m_buffer);
    m_buffer = nullptr;
}
static voice_handle_t player_waveform(unsigned int sample_rate,voice_handle_t* in_out_first, voice_func_t fn, float frequency, float amplitude = .8) {
    waveform_info_t* wi = (waveform_info_t*)malloc(sizeof(waveform_info_t));
    if(wi==nullptr) {
        return nullptr;
    }
    wi->frequency = frequency;
    wi->amplitude = amplitude;
    wi->phase = 0;
    wi->phase_delta = player_two_pi*wi->frequency/(float)sample_rate;
    return player_add_voice(in_out_first,fn,wi);
}
voice_handle_t player::sin(float frequency, float amplitude) {
    SemaphoreHandle_t sync = (SemaphoreHandle_t)m_sync;
    if(pdFALSE==xSemaphoreTake(sync,pdMS_TO_TICKS(2000))) {
        return nullptr;
    }
    voice_handle_t result = player_waveform(m_sample_rate,&m_first,sin_voice,frequency,amplitude);
    xSemaphoreGive(sync);
    return result;
}
voice_handle_t player::sqr(float frequency, float amplitude) {
    SemaphoreHandle_t sync = (SemaphoreHandle_t)m_sync;
    if(pdFALSE==xSemaphoreTake(sync,pdMS_TO_TICKS(2000))) {
        return nullptr;
    }
    voice_handle_t result = player_waveform(m_sample_rate,&m_first,sqr_voice,frequency,amplitude);
    xSemaphoreGive(sync);
    return result;
}
voice_handle_t player::saw(float frequency, float amplitude) {
    SemaphoreHandle_t sync = (SemaphoreHandle_t)m_sync;
    if(pdFALSE==xSemaphoreTake(sync,pdMS_TO_TICKS(2000))) {
        return nullptr;
    }
    voice_handle_t result = player_waveform(m_sample_rate,&m_first,saw_voice,frequency,amplitude);
    xSemaphoreGive(sync);
    return result;
}
voice_handle_t player::tri(float frequency, float amplitude) {
    SemaphoreHandle_t sync = (SemaphoreHandle_t)m_sync;
    if(pdFALSE==xSemaphoreTake(sync,pdMS_TO_TICKS(2000))) {
        return nullptr;
    }
    voice_handle_t result = player_waveform(m_sample_rate,&m_first,tri_voice,frequency,amplitude);
    xSemaphoreGive(sync);
    return result;
}

voice_handle_t player::wav(on_read_stream_callback on_read_stream, void* on_read_stream_state, float amplitude, bool loop, on_seek_stream_callback on_seek_stream, void* on_seek_stream_state) {
    if(on_read_stream==nullptr) {
        return nullptr;
    }
    if(loop && on_seek_stream==nullptr) {
        return nullptr;
    }
    unsigned int sample_rate=0;
    unsigned short channels=0;
    unsigned short bit_depth=0;
    unsigned long long start=0;
    unsigned long long length=0;
    uint32_t size;
    uint32_t remaining;
    uint32_t pos;
    //uint32_t fmt_len;
    int v = on_read_stream(on_read_stream_state);
    if(v!='R') { 
        return nullptr;
    }
    v = on_read_stream(on_read_stream_state);
    if(v!='I') { 
        return nullptr;
    }
    v = on_read_stream(on_read_stream_state);
    if(v!='F') { 
        return nullptr;
    }
    v = on_read_stream(on_read_stream_state);
    if(v!='F') { 
        return nullptr;
    }
    pos =4;
    uint32_t t32 = 0;
    if(!player_read32(on_read_stream,on_read_stream_state,&t32)) {
        return nullptr;
    }
    size = t32;
    pos+=4;
    remaining = size-8;
    v = on_read_stream(on_read_stream_state);
    if(v!='W') { 
        return nullptr;
    }
    v = on_read_stream(on_read_stream_state);
    if(v!='A') { 
        return nullptr;
    }
    v = on_read_stream(on_read_stream_state);
    if(v!='V') { 
        return nullptr;
    }
    v = on_read_stream(on_read_stream_state);
    if(v!='E') { 
        return nullptr;
    }
    pos+=4;
    remaining-=4;
    char buf[4];
    while(remaining) {
        if(!player_read_fourcc(on_read_stream,on_read_stream_state,buf)) {
            return nullptr;
        }
        pos+=4;
        remaining-=4;    
        if(!player_read32(on_read_stream,on_read_stream_state,&t32)) {
            return nullptr;
        }
        pos+=4;
        remaining-=4;
        if(0==memcmp("fmt ",buf,4)) {
            uint16_t t16;
            if(!player_read16(on_read_stream,on_read_stream_state,&t16)) {
                return nullptr;
            }
            if(t16!=1) { // PCM format
                return nullptr;
            }
            pos+=2;
            remaining-=2;
            if(!player_read16(on_read_stream,on_read_stream_state,&t16)) {
                return nullptr;
            }
            channels = t16;
            if(channels<1 || channels>2) {
                return nullptr;
            }
            pos+=2;
            remaining-=2;
            if(!player_read32(on_read_stream,on_read_stream_state,&t32)) {
                return nullptr;
            }
            sample_rate = t32;
            if(sample_rate!=this->sample_rate()) {
                return nullptr;
            }
            pos+=4;
            remaining-=4;
            if(!player_read32(on_read_stream,on_read_stream_state,&t32)) {
                return nullptr;
            }
            pos+=4;
            remaining-=4;
            if(!player_read16(on_read_stream,on_read_stream_state,&t16)) {
                return nullptr;
            }
            pos+=2;
            remaining-=2;
            if(!player_read16(on_read_stream,on_read_stream_state,&t16)) {
                return nullptr;
            }
            bit_depth = t16;
            pos+=2;
            remaining-=2;
            
        } else if(0==memcmp("data",buf,4)) {
            length = t32;
            start = pos;
            break;
        } else {
            // TODO: Seek instead
            while(t32--) {
                if(0>on_read_stream(on_read_stream_state)) {
                    return nullptr;
                }
                ++pos;
                --remaining;
            }
        }

    }
    wav_info_t* wi = (wav_info_t*)malloc(sizeof(wav_info_t));
    if(wi==nullptr) {
        return nullptr;
    }
    wi->on_read_stream = on_read_stream;
    wi->on_read_stream_state = on_read_stream_state;
    wi->on_seek_stream = on_seek_stream;
    wi->on_seek_stream_state = on_seek_stream_state;
    wi->amplitude = amplitude;
    wi->bit_depth = bit_depth;
    wi->channels = channels;
    wi->loop = loop;
    wi->on_read_stream = on_read_stream;
    wi->on_read_stream_state = on_read_stream_state;
    wi->start = start;
    wi->length = length;
    wi->pos = 0;

    SemaphoreHandle_t sync = (SemaphoreHandle_t)m_sync;
    if(pdFALSE==xSemaphoreTake(sync,pdMS_TO_TICKS(2000))) {
        free(wi);
        return nullptr;
    }
    if(wi->channels==2 && wi->bit_depth==16 && m_channels==2 && m_bit_depth==16) {
        voice_handle_t res = player_add_voice(&m_first,wav_voice_16_2_to_16_2,wi);
        xSemaphoreGive(sync);
        return res;
    } else if(wi->channels==1 && wi->bit_depth==16 && m_channels==2 && m_bit_depth==16) {
        voice_handle_t res = player_add_voice(&m_first,wav_voice_16_1_to_16_2,wi);
        xSemaphoreGive(sync);
        return res;
    }
    free(wi);
    xSemaphoreGive(sync);
    return nullptr;
    
}
bool player::stop(voice_handle_t handle) {
    if(m_first==nullptr) {
        return handle==nullptr;
    }
    SemaphoreHandle_t sync = (SemaphoreHandle_t)m_sync;
    if(pdFALSE==xSemaphoreTake(sync,pdMS_TO_TICKS(2000))) {
        return false;
    }
    if(handle==nullptr) {
        while(m_first!=nullptr) {
            player_remove_voice((voice_handle_t*)&m_first,(voice_handle_t)m_first);
        }
        xSemaphoreGive(sync);
        return m_first==nullptr;
    }
    bool result = player_remove_voice(&m_first,handle);
    xSemaphoreGive(sync);
    return result;
}
void player::on_sound_disable(on_sound_disable_callback cb, void* state) {
    m_on_sound_disable_cb = cb;
    m_on_sound_disable_state = state;
}
void player::on_sound_enable(on_sound_enable_callback cb, void* state) {
    m_on_sound_enable_cb = cb;
    m_on_sound_enable_state = state;
}
void player::on_flush(on_flush_callback cb, void* state) {
    m_on_flush_cb = cb;
    m_on_flush_state = state;
}
bool player::realloc_buffer() {
    size_t new_size = m_frame_count * m_channels * (m_bit_depth/8);
    if(new_size==0) {
        deinitialize();
        return true;
    }
    void* resized = realloc(m_buffer,new_size);
    if(resized==nullptr) {
        return false;
    }
    m_buffer = resized;
    return true;
}
size_t player::frame_count() const {
    return m_frame_count;
}
bool player::frame_count(size_t value) {
    if(value==0) {
        return false;
    }
    if(value!=m_frame_count) {
        size_t tmp = m_frame_count;
        m_frame_count = value;
        if(!realloc_buffer()) {
            m_frame_count = tmp;
            return false;
        }
    }
    return true;
}
unsigned int player::sample_rate() const {
    return m_sample_rate;
}
bool player::sample_rate(unsigned int value) {
    if(!value) {
        return false;
    }
    m_sample_rate = value;
    return true;
}
unsigned short player::channels() const {
    return m_channels;
}
bool player::channels(unsigned short value) {
    if(value==0) {
        return false;
    }
    if(value!=m_channels) {
        unsigned int tmp = m_channels;
        m_channels = value;
        if(!realloc_buffer()) {
            m_channels = tmp;
            return false;
        }
    }
    return true;
}
unsigned short player::bit_depth() const {
    return m_bit_depth;
}
bool player::bit_depth(unsigned short value) {
    if(value==0) {
        return false;
    }
    if(value!=m_bit_depth) {
        unsigned int tmp = m_bit_depth;
        m_bit_depth = value;
        if(!realloc_buffer()) {
            m_bit_depth = tmp;
            return false;
        }
    }
    return true;
}