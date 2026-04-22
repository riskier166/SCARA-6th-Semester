#ifndef __AS5600_H__
#define __AS5600_H__

#include <SimpleI2C.h>
#include <esp_timer.h>
#include <math.h>

enum MagnetStatus
{
    MD = 0,
    ML = 1,
    MH = 2,
    NO_Magnet = 3,
};


enum Address
{
    ZMCO = 0x00,
    ZPOS = 0x01,
    MPOS = 0x03,
    MANG = 0x05,
    CONF = 0x07,
    RAW_ANGLE = 0x0C,
    ANGLE = 0x0E,
    STATUS = 0x0B
};

class AS5600
{
public:
    AS5600();
    ~AS5600();
    void setup(SimpleI2C &i2c, uint8_t addr = 0x36);
    uint8_t readMagnet();
    uint8_t MagnetDetection();
    uint16_t readRawAngle();
    float getTotalAngle();
    float getSpeed();

    // get velocity()
    // getturns()

private:
    static constexpr uint8_t COMMAND_BIT = 0x80;

    void write8(uint8_t reg, uint8_t value);
    void read16(uint8_t reg, uint16_t &value);
    uint8_t read8(uint8_t reg);

    void correctAngle();
    void quadrantAngle();

    uint8_t magnet_status;
    MagnetStatus status;
    SimpleI2C *_I2C_ESP;
    uint8_t ADDRESS;

    uint64_t _current, _prev = 0;
    uint64_t _dt_us = 0;
    uint64_t _timeout_us = 70000;
    float _speed = 0;

#pragma region Angle reading and calculation variables
    uint16_t rawAngle;
    float degAngle;
    int resolution = 4096; // 12 bits
    float corrected_Angle;
    float start_Angle;
    float totalAngle;      // absolute displacement
    float current_Angle;
    float prev_Angle = 0;
    float delta_Angle;
#pragma endregion

#pragma region Quadrant detection variables
    int quadrant;      // 1,2,3,4
    int prev_Quadrant; // 1,2,3,4
    float number_of_turns;
#pragma endregion
};

#endif // __AS5600_H__