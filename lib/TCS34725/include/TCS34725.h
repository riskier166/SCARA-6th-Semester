#ifndef _TCS34725_H_
#define _TCS34725_H_

#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>

#define TCS34725_ADDR          0x29
#define TCS34725_COMMAND_BIT   0x80

// Registers
#define TCS34725_ENABLE        0x00
#define TCS34725_ATIME         0x01
#define TCS34725_CONTROL       0x0F
#define TCS34725_CDATAL        0x14
#define TCS34725_RDATAL        0x16
#define TCS34725_GDATAL        0x18
#define TCS34725_BDATAL        0x1A

class TCS34725 {
public:

    // --- ENUM del color detectado ---
    enum Color {
        COLOR_RED,
        COLOR_GREEN,
        COLOR_BLUE,
        COLOR_UNKNOWN
    };

    TCS34725();
    esp_err_t begin(int sda_pin, int scl_pin, i2c_port_t port = I2C_NUM_0);
    void enable();
    void readRGB(uint16_t &r, uint16_t &g, uint16_t &b);

    // Nuevo método:
    Color getColor();

private:
    i2c_port_t _port;
    esp_err_t write8(uint8_t reg, uint8_t value);
    esp_err_t read16(uint8_t reg, uint16_t &value);
};

#endif
