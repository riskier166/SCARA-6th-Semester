#include <AS5600.h>

AS5600::AS5600() : _I2C_ESP(nullptr), ADDRESS(0)
{
}

AS5600::~AS5600()
{
}

void AS5600::setup(SimpleI2C &i2c, uint8_t addr)
{
    _I2C_ESP = &i2c;
    ADDRESS = addr;

    _I2C_ESP->setup_device(ADDRESS);
}

uint8_t AS5600::readMagnet()
{
    magnet_status = read8(STATUS);
    return magnet_status;
}

uint8_t AS5600::MagnetDetection()
{

    magnet_status = readMagnet();
    vTaskDelay(pdMS_TO_TICKS(100)); // Delay to prevent excessive I2C reads, adjust as needed
    if (magnet_status & 0x20)
    {
        // printf("Magnet detected: %d\n", magnet_status);
        status = MD;
    }

    else if (magnet_status & 0x10)
    {
        printf("Magnet too weak - decrease distance or use stronger magnet: %d\n", magnet_status);
        status = ML;
    }

    else if (magnet_status & 0x08)
    {
        printf("Magnet too strong - increase distance or use weaker magnet: %d\n", magnet_status);
        status = MH;
    }

    else
    {
        printf("No magnet detected: %d\n", magnet_status);
        status = NO_Magnet;
    }

    switch (status)
    {
    case MD:
        printf("Magnet detected: %d\n", magnet_status);
        return MD;
    case ML:
        printf("Magnet too weak - decrease distance or use stronger magnet: %d\n", magnet_status);
        return ML;
    case MH:
        printf("Magnet too strong - increase distance or use weaker magnet: %d\n", magnet_status);
        return MH;
    case NO_Magnet:
        printf("No magnet detected: %d\n", magnet_status);
        return NO_Magnet;

    default:
        return NO_Magnet;
    }
}

void AS5600::correctAngle() //-15 --> 345
{
    corrected_Angle = (degAngle - start_Angle);
    if (corrected_Angle < 0)
    {
        corrected_Angle = corrected_Angle + 360;
    }
}

void AS5600::quadrantAngle()
{
    /*
    //Quadrants
    4 | 1
    -----
    3 | 2
    */
    // quadrant detection can be changed from every 90 to every 45 degrees
    if (degAngle >= 0 && degAngle < 90)
        quadrant = 1;
    else if (degAngle >= 90 && degAngle < 180)
        quadrant = 2;
    else if (degAngle >= 180 && degAngle < 270)
        quadrant = 3;
    else
        quadrant = 4;

    if (quadrant != prev_Quadrant)
    {
        if (quadrant == 1 && prev_Quadrant == 4)
            number_of_turns++;
        else if (quadrant == 4 && prev_Quadrant == 1)
            number_of_turns--;

        prev_Quadrant = quadrant;
    }
}

uint16_t AS5600::readRawAngle()
{
    read16(RAW_ANGLE, rawAngle);
    degAngle = (rawAngle * 360.0) / resolution;

    correctAngle();
    quadrantAngle();

    return rawAngle;
}

float AS5600::getTotalAngle()
{
    totalAngle = corrected_Angle + (number_of_turns * 360);
    current_Angle = totalAngle;
    return totalAngle;
}

float AS5600::getSpeed()
{
    _current = esp_timer_get_time();
    if (_prev == 0)
    {
        _prev = _current;
        prev_Angle = current_Angle;
        return 0.0f;
    }

    delta_Angle = current_Angle - prev_Angle;
    _dt_us = _current - _prev;

    if (fabs(delta_Angle) > 0.0001 && _dt_us > 0)
    {
        _speed = (delta_Angle * 1000000.0f) / _dt_us;
    }
    else
    {
        _speed = 0.0f;
    }

    prev_Angle = current_Angle;
    _prev = _current;

    return _speed;
}

void AS5600::write8(uint8_t reg, uint8_t value)
{
    uint8_t data[] = {reg, value};
    _I2C_ESP->write(ADDRESS, data, 2);
}

void AS5600::read16(uint8_t reg, uint16_t &value)
{
    uint8_t buffer[2];
    _I2C_ESP->master_read_write(ADDRESS, &reg, 1, buffer, 2);
    value = (buffer[0] << 8) | buffer[1];
}

uint8_t AS5600::read8(uint8_t reg)
{
    uint8_t value = 0;

    // 1. Escribes el registro que quieres leer
    _I2C_ESP->write(ADDRESS, &reg, 1);

    // 2. Lees el valor
    _I2C_ESP->read(ADDRESS, &value, 1);

    return value;
}
