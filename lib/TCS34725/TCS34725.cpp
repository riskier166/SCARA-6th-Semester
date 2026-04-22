#include "TCS34725.h"

static const char *TAG = "TCS34725";

TCS34725::TCS34725() {}

esp_err_t TCS34725::begin(int sda_pin, int scl_pin, i2c_port_t port)
{
    _port = port;

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda_pin;
    conf.scl_io_num = scl_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;

    ESP_ERROR_CHECK(i2c_param_config(_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(_port, I2C_MODE_MASTER, 0, 0, 0));

    write8(TCS34725_ATIME, 0xF6);   // 24 ms
    write8(TCS34725_CONTROL, 0x01); // Ganancia 4x
    enable();

    ESP_LOGI(TAG, "TCS34725 inicializado");
    return ESP_OK;
}

void TCS34725::enable()
{
    write8(TCS34725_ENABLE, 0x01); 
    vTaskDelay(pdMS_TO_TICKS(3));
    write8(TCS34725_ENABLE, 0x03);
}

esp_err_t TCS34725::write8(uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = static_cast<uint8_t>(TCS34725_COMMAND_BIT | reg);
    data[1] = value;

    return i2c_master_write_to_device(
        _port, TCS34725_ADDR,
        data, sizeof(data),
        10 / portTICK_PERIOD_MS
    );
}

esp_err_t TCS34725::read16(uint8_t reg, uint16_t &value)
{
    uint8_t buffer[2];

    uint8_t cmd = TCS34725_COMMAND_BIT | reg;

    esp_err_t err = i2c_master_write_read_device(
        _port,
        TCS34725_ADDR,
        &cmd,
        1,
        buffer,
        2,
        10 / portTICK_PERIOD_MS
    );

    value = (buffer[1] << 8) | buffer[0];
    return err;
}

void TCS34725::readRGB(uint16_t &r, uint16_t &g, uint16_t &b)
{
    read16(TCS34725_RDATAL, r);
    read16(TCS34725_GDATAL, g);
    read16(TCS34725_BDATAL, b);
}

TCS34725::Color TCS34725::getColor()
{
    uint16_t r, g, b;
    readRGB(r, g, b);

    if (r > g && r > b)
        return COLOR_RED;
    else if (g > r && g > b)
        return COLOR_GREEN;
    else if (b > r && b > g)
        return COLOR_BLUE;

    return COLOR_UNKNOWN;
}
