#ifndef I2C_DEVICE_HPP
#define I2C_DEVICE_HPP

#include <string>
#include <cstdint>
#include <vector>

class I2CDevice {
public:
    I2CDevice(const std::string& bus, uint8_t address);
    ~I2CDevice();

    bool openDevice();
    void closeDevice();
    bool isOpened() const;

    bool writeByte(uint8_t reg, uint8_t data);
    bool writeBytes(uint8_t reg, const std::vector<uint8_t>& data);
    int16_t readByte(uint8_t reg);
    int16_t readWord(uint8_t reg);
    std::vector<uint8_t> readBytes(uint8_t reg, uint8_t length);

private:
    std::string bus_path_;
    uint8_t device_address_;
    int fd_;
};

#endif // I2C_DEVICE_HPP