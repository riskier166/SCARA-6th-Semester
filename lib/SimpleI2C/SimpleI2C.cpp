#include "SimpleI2C.h"

SimpleI2C::SimpleI2C() : _bus_handle(nullptr), _i2c_num(I2C_NUM_0) {}

void SimpleI2C::setup_master(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq, i2c_port_t i2c_num) {
    if (_bus_handle != nullptr) {
        ESP_LOGW("I2C", "I2C bus already initialized");
        return;
    }
    _freq = freq;
    _i2c_num = i2c_num;

    i2c_master_bus_config_t conf = {};
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.scl_io_num = static_cast<gpio_num_t>(scl_pin);
    conf.sda_io_num = static_cast<gpio_num_t>(sda_pin);
    conf.i2c_port = i2c_num;
    conf.glitch_ignore_cnt = 7;
    conf.flags.enable_internal_pullup = true;
    //conf.flags.allow_pd = false;
    conf.intr_priority = 0;

    esp_err_t err = i2c_new_master_bus(&conf, &_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE("I2C", "Failed to initialize I2C master: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI("I2C", "Master bus initialized");
}

void SimpleI2C::setup_device(uint8_t device_address) {
    if (_bus_handle == nullptr) {
        ESP_LOGE("I2C", "Cannot configure device, I2C bus is not initialized.");
        return;
    }

    if (_devices.find(device_address) != _devices.end()) {
        ESP_LOGW("I2C", "Device 0x%02X already configured.", device_address);
        return;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = device_address & 0x7F;
    dev_cfg.scl_speed_hz = _freq;
    dev_cfg.scl_wait_us = 100;

    i2c_master_dev_handle_t dev_handle;
    esp_err_t err = i2c_master_bus_add_device(_bus_handle, &dev_cfg, &dev_handle);
    if (err == ESP_OK) {
        _devices[device_address] = dev_handle;
        ESP_LOGI("I2C", "Device 0x%02X configured successfully.", device_address);
    } else {
        ESP_LOGE("I2C", "Failed to configure device 0x%02X: %s", device_address, esp_err_to_name(err));
    }
}

void SimpleI2C::scan_bus() {
    if (_bus_handle == nullptr) {
        ESP_LOGE("I2C Scanner", "The I2C bus is not initialized. Call setup_master() first.");
        return;
    }

    ESP_LOGI("I2C Scanner", "Scanning for devices...");
    bool found = false;

    for (uint8_t address = 3; address < 127; address++) {
        i2c_master_dev_handle_t temp_handle;
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = address;
        dev_cfg.scl_speed_hz = _freq;
        dev_cfg.scl_wait_us = 100;
        
        esp_err_t err = i2c_master_bus_add_device(_bus_handle, &dev_cfg, &temp_handle);
        if (err == ESP_OK) {
            ESP_LOGI("I2C Scanner", "Device found at address: 0x%02X", address);
            i2c_master_bus_rm_device(temp_handle);
            found = true;
        }
    }

    if (!found) {
        ESP_LOGI("I2C Scanner", "No devices found on the I2C bus.");
    } else {
        ESP_LOGI("I2C Scanner", "Scan complete");
    }
}

void SimpleI2C::read(uint8_t device_address, uint8_t *data_out, size_t size_out) {
    auto it = _devices.find(device_address);
    if (it == _devices.end()) {
        ESP_LOGE("I2C", "Device 0x%02X not configured.", device_address);
        return;
    }
    esp_err_t ret = i2c_master_receive(it->second, data_out, size_out, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Read error %s", esp_err_to_name(ret));
    }
}

void SimpleI2C::write(uint8_t device_address, const uint8_t *data_in, size_t size_in) {
    auto it = _devices.find(device_address);
    if (it == _devices.end()) {
        ESP_LOGE("I2C", "Device 0x%02X not configured.", device_address);
        return;
    }
    esp_err_t ret = i2c_master_transmit(it->second, data_in, size_in, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Write error %s", esp_err_to_name(ret));
    }
}

void SimpleI2C::master_read_write(uint8_t device_address, const uint8_t *data_in, size_t size_in, uint8_t *data_out, size_t size_out) {
    auto it = _devices.find(device_address);
    if (it == _devices.end()) {
        ESP_LOGE("I2C", "Device 0x%02X not configured.", device_address);
        return;
    }
    esp_err_t ret = i2c_master_transmit_receive(it->second, data_in, size_in, data_out, size_out, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Read/Write error %s", esp_err_to_name(ret));
    }
}
