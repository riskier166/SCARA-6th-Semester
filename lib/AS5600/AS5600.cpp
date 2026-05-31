#include "AS5600.h"

static const char *TAG = "AS5600";

// AS5600 registers
static constexpr uint8_t RAW_ANGLE_HIGH_REG = 0x0C;
static constexpr uint8_t RAW_ANGLE_LOW_REG  = 0x0D;

AS5600::AS5600(I2CBus *i2c_bus)
    : _i2c_bus(i2c_bus),
      _device_handle(nullptr),
      _angle_deg(0.0f),
      _angle_rad(0.0f),
      _previous_angle_deg(0.0f),
      _angular_velocity_deg_s(0.0f),
      _angular_velocity_rad_s(0.0f),
      _previous_time_us(0)
{
}

esp_err_t AS5600::init()
{
    if (_i2c_bus == nullptr)
    {
        ESP_LOGE(TAG, "I2C bus is null");
        return ESP_ERR_INVALID_ARG;
    }

    if (!_i2c_bus->deviceAvailable(AS5600_ADDRESS))
    {
        ESP_LOGE(TAG, "AS5600 not detected at address 0x%02X", AS5600_ADDRESS);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t err = _i2c_bus->addDevice(AS5600_ADDRESS, &_device_handle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add AS5600 device");
        return err;
    }

    uint16_t raw_angle = 0;
    err = readRawAngle(&raw_angle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read initial angle");
        return err;
    }

    _angle_deg = (raw_angle * 360.0f) / 4096.0f;
    _angle_rad = _angle_deg * static_cast<float>(M_PI) / 180.0f;

    _previous_angle_deg = _angle_deg;
    _previous_time_us = esp_timer_get_time();

    ESP_LOGI(TAG, "AS5600 initialized successfully");

    return ESP_OK;
}

esp_err_t AS5600::readRawAngle(uint16_t *raw_angle)
{
    if (_device_handle == nullptr || raw_angle == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2] = {0};

    esp_err_t err = _i2c_bus->readRegister(
        _device_handle,
        RAW_ANGLE_HIGH_REG,
        data,
        2
    );

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read raw angle: %s", esp_err_to_name(err));
        return err;
    }

    *raw_angle = ((data[0] & 0x0F) << 8) | data[1];

    return ESP_OK;
}

esp_err_t AS5600::readAngleDegrees(float *angle_deg)
{
    if (angle_deg == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_angle = 0;

    esp_err_t err = readRawAngle(&raw_angle);

    if (err != ESP_OK)
    {
        return err;
    }

    *angle_deg = (raw_angle * 360.0f) / 4096.0f;

    return ESP_OK;
}

esp_err_t AS5600::readAngleRadians(float *angle_rad)
{
    if (angle_rad == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    float angle_deg = 0.0f;

    esp_err_t err = readAngleDegrees(&angle_deg);

    if (err != ESP_OK)
    {
        return err;
    }

    *angle_rad = angle_deg * static_cast<float>(M_PI) / 180.0f;

    return ESP_OK;
}

esp_err_t AS5600::update()
{
    float current_angle_deg = 0.0f;

    esp_err_t err = readAngleDegrees(&current_angle_deg);

    if (err != ESP_OK)
    {
        return err;
    }

    int64_t current_time_us = esp_timer_get_time();

    float dt = (current_time_us - _previous_time_us) / 1000000.0f;

    if (dt <= 0.0f)
    {
        return ESP_ERR_INVALID_STATE;
    }

    float delta_angle = calculateDeltaAngle(
        current_angle_deg,
        _previous_angle_deg
    );

    _angular_velocity_deg_s = delta_angle / dt;
    _angular_velocity_rad_s = _angular_velocity_deg_s *
                              static_cast<float>(M_PI) / 180.0f;

    _angle_deg = current_angle_deg;
    _angle_rad = _angle_deg * static_cast<float>(M_PI) / 180.0f;

    _previous_angle_deg = current_angle_deg;
    _previous_time_us = current_time_us;

    return ESP_OK;
}

float AS5600::getAngleDegrees()
{
    return _angle_deg;
}

float AS5600::getAngleRadians()
{
    return _angle_rad;
}

float AS5600::getAngularVelocityDegS()
{
    return _angular_velocity_deg_s;
}

float AS5600::getAngularVelocityRadS()
{
    return _angular_velocity_rad_s;
}

float AS5600::calculateDeltaAngle(float current_angle, float previous_angle)
{
    float delta = current_angle - previous_angle;

    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    return delta;
}