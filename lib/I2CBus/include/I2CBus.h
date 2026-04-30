#ifndef I2CBUS_H
#define I2CBUS_H

#pragma once

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

class I2CBus
{
public:
    I2CBus(i2c_port_num_t port,
           gpio_num_t sda,
           gpio_num_t scl,
           uint32_t frequency);

    ~I2CBus();

    esp_err_t init();

    esp_err_t addDevice(uint8_t device_address,
                        i2c_master_dev_handle_t *device_handle);

    esp_err_t write(i2c_master_dev_handle_t device_handle,
                    const uint8_t *data,
                    size_t length,
                    int timeout_ms = 100);

    esp_err_t read(i2c_master_dev_handle_t device_handle,
                   uint8_t *data,
                   size_t length,
                   int timeout_ms = 100);

    esp_err_t writeRegister(i2c_master_dev_handle_t device_handle,
                            uint8_t register_address,
                            uint8_t data,
                            int timeout_ms = 100);

    esp_err_t writeRegisters(i2c_master_dev_handle_t device_handle,
                             uint8_t register_address,
                             const uint8_t *data,
                             size_t length,
                             int timeout_ms = 100);

    esp_err_t readRegister(i2c_master_dev_handle_t device_handle,
                           uint8_t register_address,
                           uint8_t *data,
                           size_t length,
                           int timeout_ms = 100);

    bool deviceAvailable(uint8_t device_address,
                         int timeout_ms = 100);

private:
    i2c_port_num_t _port;
    gpio_num_t _sda;
    gpio_num_t _scl;
    uint32_t _frequency;

    i2c_master_bus_handle_t _bus_handle;
};

#endif