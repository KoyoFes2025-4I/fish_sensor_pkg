#ifndef AS5600_HPP
#define AS5600_HPP

#include "fish_sensor_pkg/i2c_device.hpp"
#include <memory>
#include <string>

class AS5600 {
public:
    AS5600(const std::string& bus, uint8_t address);
    bool init();
    bool testConnection();
    double getAngleDeg();

private:
    std::unique_ptr<I2CDevice> i2c_dev_;
};

#endif // AS5600_HPP