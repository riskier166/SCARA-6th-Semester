#ifndef AS5600_H
#define AS5600_H

#pragma once

#include "I2CBus.h"
#include "esp_timer.h"
#include <cmath>

#define AS5600_ADDRESS 0x36

class AS5600
{
public:
    AS5600(I2CBus *i2c_bus);

    esp_err_t init();

    esp_err_t readRawAngle(uint16_t *raw_angle);
    esp_err_t readAngleDegrees(float *angle_deg);
    esp_err_t readAngleRadians(float *angle_rad);

    esp_err_t update();

    float getAngleDegrees();              // 0° to 360°
    float getAngleRadians();

    float getContinuousAngleDegrees();    // Multi-turn angle
    float getContinuousAngleRadians();

    float getAngularVelocityDegS();
    float getAngularVelocityRadS();

    void resetContinuousAngle(float new_angle_deg = 0.0f);

private:
    I2CBus *_i2c_bus;
    i2c_master_dev_handle_t _device_handle;

    float _angle_deg;
    float _angle_rad;

    float _continuous_angle_deg;
    float _continuous_angle_rad;

    float _previous_angle_deg;

    float _angular_velocity_deg_s;
    float _angular_velocity_rad_s;

    int64_t _previous_time_us;

    float calculateDeltaAngle(float current_angle, float previous_angle);
};

#endif