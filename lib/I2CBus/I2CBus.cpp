#include "I2CBus.h"

static const char *TAG = "I2CBus";

I2CBus::I2CBus(i2c_port_num_t port,
               gpio_num_t sda,
               gpio_num_t scl,
               uint32_t frequency)
    : _port(port),
      _sda(sda),
      _scl(scl),
      _frequency(frequency),
      _bus_handle(nullptr)
{
}

I2CBus::~I2CBus()
{
    if (_bus_handle != nullptr)
    {
        i2c_del_master_bus(_bus_handle);
        _bus_handle = nullptr;
    }
}

esp_err_t I2CBus::init()
{
    i2c_master_bus_config_t bus_config = {};

    bus_config.i2c_port = _port;
    bus_config.sda_io_num = _sda;
    bus_config.scl_io_num = _scl;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    esp_err_t err = i2c_new_master_bus(&bus_config, &_bus_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C bus initialized");
    return ESP_OK;
}

esp_err_t I2CBus::addDevice(uint8_t device_address,
                            i2c_master_dev_handle_t *device_handle)
{
    if (_bus_handle == nullptr)
    {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (device_handle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t device_config = {};

    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = device_address;
    device_config.scl_speed_hz = _frequency;

    esp_err_t err = i2c_master_bus_add_device(
        _bus_handle,
        &device_config,
        device_handle
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add device 0x%02X: %s",
                 device_address,
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Device 0x%02X added", device_address);
    return ESP_OK;
}

esp_err_t I2CBus::write(i2c_master_dev_handle_t device_handle,
                        const uint8_t *data,
                        size_t length,
                        int timeout_ms)
{
    if (device_handle == nullptr || data == nullptr || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit(
        device_handle,
        data,
        length,
        timeout_ms
    );
}

esp_err_t I2CBus::read(i2c_master_dev_handle_t device_handle,
                       uint8_t *data,
                       size_t length,
                       int timeout_ms)
{
    if (device_handle == nullptr || data == nullptr || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_receive(
        device_handle,
        data,
        length,
        timeout_ms
    );
}

esp_err_t I2CBus::writeRegister(i2c_master_dev_handle_t device_handle,
                                uint8_t register_address,
                                uint8_t data,
                                int timeout_ms)
{
    uint8_t buffer[2] = {
        register_address,
        data
    };

    return write(device_handle, buffer, sizeof(buffer), timeout_ms);
}

esp_err_t I2CBus::writeRegisters(i2c_master_dev_handle_t device_handle,
                                 uint8_t register_address,
                                 const uint8_t *data,
                                 size_t length,
                                 int timeout_ms)
{
    if (device_handle == nullptr || data == nullptr || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[length + 1];

    buffer[0] = register_address;

    for (size_t i = 0; i < length; i++)
    {
        buffer[i + 1] = data[i];
    }

    return write(device_handle, buffer, length + 1, timeout_ms);
}

esp_err_t I2CBus::readRegister(i2c_master_dev_handle_t device_handle,
                               uint8_t register_address,
                               uint8_t *data,
                               size_t length,
                               int timeout_ms)
{
    if (device_handle == nullptr || data == nullptr || length == 0)
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(
        device_handle,
        &register_address,
        1,
        data,
        length,
        timeout_ms
    );
}

bool I2CBus::deviceAvailable(uint8_t device_address,
                             int timeout_ms)
{
    if (_bus_handle == nullptr)
    {
        return false;
    }

    esp_err_t err = i2c_master_probe(
        _bus_handle,
        device_address,
        timeout_ms
    );

    return err == ESP_OK;
}