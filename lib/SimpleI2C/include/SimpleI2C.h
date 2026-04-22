#ifndef SIMPLE_I2C_H
#define SIMPLE_I2C_H

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include <unordered_map>

#define I2C_TIMEOUT_MS 1000  // Definir un timeout de 1000ms

class SimpleI2C {
public:
    SimpleI2C();
    void setup_master(uint8_t sda_pin = 21, uint8_t scl_pin = 22, uint32_t freq = 400000, i2c_port_t i2c_num = I2C_NUM_0);
    void setup_device(uint8_t device_address);
    void scan_bus();
    void read(uint8_t device_address, uint8_t *data_out, size_t size_out);
    void write(uint8_t device_address, const uint8_t *data_in, size_t size_in);
    void master_read_write(uint8_t device_address, const uint8_t *data_in, size_t size_in, uint8_t *data_out, size_t size_out);

private:
    i2c_master_bus_handle_t _bus_handle;
    i2c_port_t _i2c_num;
    uint32_t _freq;
    std::unordered_map<uint8_t, i2c_master_dev_handle_t> _devices;
};

#endif // SIMPLE_I2C_H
