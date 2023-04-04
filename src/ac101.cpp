#include <ac101.hpp>
#ifndef ARDUINO
#include <esp_log.h>
#endif

#define AC101_ADDR 0x1a /*!< Device address*/

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

#define CHIP_AUDIO_RS 0x00
#define PLL_CTRL1 0x01
#define PLL_CTRL2 0x02
#define SYSCLK_CTRL 0x03
#define MOD_CLK_ENA 0x04
#define MOD_RST_CTRL 0x05
#define I2S_SR_CTRL 0x06
#define I2S1LCK_CTRL 0x10
#define I2S1_SDOUT_CTRL 0x11
#define I2S1_SDIN_CTRL 0x12
#define I2S1_MXR_SRC 0x13
#define I2S1_VOL_CTRL1 0x14
#define I2S1_VOL_CTRL2 0x15
#define I2S1_VOL_CTRL3 0x16
#define I2S1_VOL_CTRL4 0x17
#define I2S1_MXR_GAIN 0x18
#define ADC_DIG_CTRL 0x40
#define ADC_VOL_CTRL 0x41
#define HMIC_CTRL1 0x44
#define HMIC_CTRL2 0x45
#define HMIC_STATUS 0x46
#define DAC_DIG_CTRL 0x48
#define DAC_VOL_CTRL 0x49
#define DAC_MXR_SRC 0x4c
#define DAC_MXR_GAIN 0x4d
#define ADC_APC_CTRL 0x50
#define ADC_SRC 0x51
#define ADC_SRCBST_CTRL 0x52
#define OMIXER_DACA_CTRL 0x53
#define OMIXER_SR 0x54
#define OMIXER_BST1_CTRL 0x55
#define HPOUT_CTRL 0x56
#define SPKOUT_CTRL 0x58
#define AC_DAC_DAPCTRL 0xa0
#define AC_DAC_DAPHHPFC 0xa1
#define AC_DAC_DAPLHPFC 0xa2
#define AC_DAC_DAPLHAVC 0xa3
#define AC_DAC_DAPLLAVC 0xa4
#define AC_DAC_DAPRHAVC 0xa5
#define AC_DAC_DAPRLAVC 0xa6
#define AC_DAC_DAPHGDEC 0xa7
#define AC_DAC_DAPLGDEC 0xa8
#define AC_DAC_DAPHGATC 0xa9
#define AC_DAC_DAPLGATC 0xaa
#define AC_DAC_DAPHETHD 0xab
#define AC_DAC_DAPLETHD 0xac
#define AC_DAC_DAPHGKPA 0xad
#define AC_DAC_DAPLGKPA 0xae
#define AC_DAC_DAPHGOPA 0xaf
#define AC_DAC_DAPLGOPA 0xb0
#define AC_DAC_DAPOPT 0xb1
#define DAC_DAP_ENA 0xb5
#ifdef ARDUINO
#define I2C_TYPE TwoWire&
using namespace arduino;
#else
#define I2C_TYPE i2c_port_t
using namespace esp_idf;
#endif
constexpr static const char* TAG = "ac101";
static bool write_reg(I2C_TYPE i2c, uint8_t reg, uint16_t val) {
#ifdef ARDUINO
    i2c.beginTransmission(AC101_ADDR);
    i2c.write(reg);
    i2c.write((uint8_t)((val >> 8) & 0xff));
    i2c.write((uint8_t)(val & 0xff));
    return 0 == i2c.endTransmission();
#else
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret = 0;
    uint8_t send_buff[4];
    send_buff[0] = (AC101_ADDR << 1);
    send_buff[1] = reg;
    send_buff[2] = (val >> 8) & 0xff;
    send_buff[3] = val & 0xff;
    ret |= i2c_master_start(cmd);
    ret |= i2c_master_write(cmd, send_buff, 4, ACK_CHECK_EN);
    ret |= i2c_master_stop(cmd);
    ret |= i2c_master_cmd_begin(i2c, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret != 0) {
        ESP_LOGE(TAG, "Error writing i2c");
        return false;
    }
    return true;
#endif
}
#ifndef ARDUINO
static esp_err_t read_data(I2C_TYPE i2c, uint8_t reg, uint8_t* data_rd, size_t size) {
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AC101_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AC101_ADDR << 1) | READ_BIT, ACK_CHECK_EN);  // check or not
    i2c_master_read(cmd, data_rd, size, (i2c_ack_type_t)ACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}
#endif
static uint16_t read_reg(I2C_TYPE i2c, uint8_t reg) {
    uint16_t result = 0;
#ifdef ARDUINO
    i2c.beginTransmission(AC101_ADDR);
    i2c.write(reg);
    i2c.endTransmission(false);

    if (2 == i2c.requestFrom((uint8_t)AC101_ADDR, (uint8_t)2, (uint8_t)1)) {
        result = (uint16_t)(i2c.read() << 8) + (uint16_t)(i2c.read());
    }
    
#else
    uint8_t data_rd[2];
    esp_err_t ret = read_data(i2c, reg, data_rd, 2);
    result = (data_rd[0] << 8) + data_rd[1];
    if (ret != 0) {
        ESP_LOGE(TAG, "Error reading i2c");
    }
#endif
    return result;
}
void ac101::do_move(ac101& rhs) {
    m_i2c = rhs.m_i2c;
    m_initialized = rhs.m_initialized;
}
ac101::ac101(
#ifdef ARDUINO
    TwoWire& i2c
#else
    i2c_port_t i2c
#endif
    ) : m_i2c(i2c), m_initialized(false) {
}
ac101::ac101(ac101&& rhs) : m_i2c(rhs.m_i2c) {
    do_move(rhs);
}
ac101& ac101::operator=(ac101&& rhs) {
    do_move(rhs);
    return *this;
}
bool ac101::initialized() const {
    return m_initialized;
}
bool ac101::initialize() {
    if (m_initialized) {
        return true;
    }
    if (!write_reg(m_i2c, CHIP_AUDIO_RS, 0x123)) {
        return false;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    if (!write_reg(m_i2c, SPKOUT_CTRL, 0xe880)) {
        return false;
    }

    // Enable the PLL from 256*44.1KHz MCLK source
    if (!write_reg(m_i2c, PLL_CTRL1, 0x014f)) {
        return false;
    }
    // res |= AC101_Write_Reg(PLL_CTRL2, 0x83c0);
    if (!write_reg(m_i2c, PLL_CTRL2, 0x8600)) {
        return false;
    }

    // Clocking system
    if (!write_reg(m_i2c, SYSCLK_CTRL, 0x8b08)) {
        return false;
    }
    if (!write_reg(m_i2c, MOD_CLK_ENA, 0x800c)) {
        return false;
    }
    if (!write_reg(m_i2c, MOD_RST_CTRL, 0x800c)) {
        return false;
    }
    if (!write_reg(m_i2c, I2S_SR_CTRL, 0x7000)) {  // sample rate
        return false;
    }
    // AIF config
    if (!write_reg(m_i2c, I2S1LCK_CTRL, 0x8850)) {  // BCLK/LRCK
        return false;
    }
    if (!write_reg(m_i2c, I2S1_SDOUT_CTRL, 0xc000)) {
        return false;
    }
    if (!write_reg(m_i2c, I2S1_SDIN_CTRL, 0xc000)) {
        return false;
    }
    if (!write_reg(m_i2c, I2S1_MXR_SRC, 0x2200)) {
        return false;
    }
    if (!write_reg(m_i2c, ADC_SRCBST_CTRL, 0xccc4)) {
        return false;
    }
    if (!write_reg(m_i2c, ADC_SRC, 0x2020)) {
        return false;
    }
    if (!write_reg(m_i2c, ADC_DIG_CTRL, 0x8000)) {
        return false;
    }
    if (!write_reg(m_i2c, ADC_APC_CTRL, 0xbbc3)) {
        return false;
    }

    // Path Configuration
    if (!write_reg(m_i2c, DAC_MXR_SRC, 0xcc00)) {
        return false;
    }
    if (!write_reg(m_i2c, DAC_DIG_CTRL, 0x8000)) {
        return false;
    }
    if (!write_reg(m_i2c, OMIXER_SR, 0x0081)) {
        return false;
    }
    if (!write_reg(m_i2c, OMIXER_DACA_CTRL, 0xf080)) {
        return false;
    }

    // Enable Speaker output
    if (!write_reg(m_i2c, 0x58, 0xeabd)) {
        return false;
    }
    m_initialized = true;
    return true;
}
float ac101::speaker_volume() const {
    int res;
    res = read_reg(m_i2c, SPKOUT_CTRL);
    res &= 0x1f;
    return (float)(res * 2) / 63.0f;
}
void ac101::speaker_volume(float value) {
    if (value != value || value < 0.0) {
        value = 0.0f;
    } else if (value > 1.0f) {
        value = 1.0f;
    }
    uint16_t res;
    uint16_t val = (uint16_t)(value * 63) / 2;
    res = read_reg(m_i2c, SPKOUT_CTRL);
    res &= (~0x1f);
    val &= 0x1f;
    res |= val;
    write_reg(m_i2c, SPKOUT_CTRL, res);
}

float ac101::headphone_volume() const {
    int res;
    res = read_reg(m_i2c, HPOUT_CTRL);
    uint8_t tmp = (res >> 4) & 0x3f;
    return ((float)tmp) / 63.0f;
}

void ac101::headphone_volume(float value) {
    if (value != value || value < 0.0) {
        value = 0.0f;
    } else if (value > 1.0f) {
        value = 1.0f;
    }
    uint16_t res, tmp;
    res = read_reg(m_i2c, HPOUT_CTRL);
    tmp = ~(0x3f << 4);
    res &= tmp;
    uint16_t val = value * 63;
    val &= 0x3f;
    res |= (val << 4);
    write_reg(m_i2c, HPOUT_CTRL, res);
}
void ac101::sample_rate(ac101_sample_rate value) {
    write_reg(m_i2c, I2S_SR_CTRL, (uint16_t)value);
}

void ac101::i2s_mode(ac101_i2s_mode value) {
    uint16_t val = read_reg(m_i2c, I2S1LCK_CTRL);
    val &= ~0x8000;
    val |= ((uint16_t)value) << 15;
    write_reg(m_i2c, I2S1LCK_CTRL, val);
}
void ac101::clock(ac101_bclk_div bclk_div, bool bclk_inv, ac101_lrclk_div lrclk_div, bool lrclk_inv) {
    uint16_t val = read_reg(m_i2c, I2S1LCK_CTRL);
    val &= ~0x7FC0;
    val |= ((uint16_t)bclk_div) << 9;
    val |= ((uint16_t)bclk_inv) << 14;
    val |= (uint16_t)(lrclk_div) << 6;
    val |= ((uint16_t)lrclk_inv) << 13;
    write_reg(m_i2c, I2S1LCK_CTRL, val);
}
void ac101::bit_depth(ac101_bit_depth value) {
    uint16_t val = read_reg(m_i2c, I2S1LCK_CTRL);
    val &= ~0x0030;
    val |= ((uint16_t)value) << 4;
    write_reg(m_i2c, I2S1LCK_CTRL, val);
}
void ac101::format(ac101_format value) {
    uint16_t val = read_reg(m_i2c, I2S1LCK_CTRL);
    val &= ~0x000C;
    val |= ((uint16_t)value) << 2;
    write_reg(m_i2c, I2S1LCK_CTRL, val);
}
void ac101::mixer_source(ac101_mixer_source source, ac101_mixer_gain gain) {
    uint16_t regval, temp, clrbit;
    esp_err_t ret;
    regval = read_reg(m_i2c, OMIXER_BST1_CTRL);
    switch (source) {
        case ac101_mixer_source::mic1:
            temp = (((int)gain) & 0x7) << 6;
            clrbit = ~(0x7 << 6);
            break;
        case ac101_mixer_source::mic2:
            temp = (((int)gain) & 0x7) << 3;
            clrbit = ~(0x7 << 3);
            break;
        case ac101_mixer_source::line_in:
            temp = (((int)gain) & 0x7);
            clrbit = ~0x7;
            break;
        default:
            return;
    }
    regval &= clrbit;
    regval |= temp;
    write_reg(m_i2c, OMIXER_BST1_CTRL, regval);
}
void ac101::start(ac101_mode mode) {
    if (mode == ac101_mode::line) {
        write_reg(m_i2c, 0x51, 0x0408);
        write_reg(m_i2c, 0x40, 0x8000);
        write_reg(m_i2c, 0x50, 0x3bc0);
    }
    if (mode == ac101_mode::adc || mode == ac101_mode::adc_dac || mode == ac101_mode::line) {
        // I2S1_SDOUT_CTRL
        // res |= AC101_Write_Reg(PLL_CTRL2, 0x8120);
        write_reg(m_i2c, 0x04, 0x800c);
        write_reg(m_i2c, 0x05, 0x800c);
        // res |= AC101_Write_Reg(0x06, 0x3000);
    }
    if (mode == ac101_mode::dac || mode == ac101_mode::adc_dac || mode == ac101_mode::line) {
        //* Enable Headphoe output   注意使用耳机时，最后开以下寄存器
        write_reg(m_i2c, OMIXER_DACA_CTRL, 0xff80);
        write_reg(m_i2c, HPOUT_CTRL, 0xc3c1);
        write_reg(m_i2c, HPOUT_CTRL, 0xcb00);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        write_reg(m_i2c, HPOUT_CTRL, 0xfbc0);

        //* Enable Speaker output
        write_reg(m_i2c, SPKOUT_CTRL, 0xeabd);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    headphone_volume(.5);
    speaker_volume(.5);
}
void ac101::stop() {
    write_reg(m_i2c, HPOUT_CTRL, 0x01);     // disable earphone
    write_reg(m_i2c, SPKOUT_CTRL, 0xe880);  // disable speaker
}