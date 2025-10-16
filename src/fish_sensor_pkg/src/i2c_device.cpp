#include "fish_sensor_pkg/i2c_device.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <iostream>

I2CDevice::I2CDevice(const std::string& bus, uint8_t address)
    : bus_path_(bus), device_address_(address), fd_(-1) {}

I2CDevice::~I2CDevice() {
    closeDevice();
}

bool I2CDevice::openDevice() {
    fd_ = open(bus_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        // エラーログは呼び出し元で出す想定
        return false;
    }
    if (ioctl(fd_, I2C_SLAVE, device_address_) < 0) {
        // エラーログは呼び出し元で出す想定
        close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

void I2CDevice::closeDevice() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
}

bool I2CDevice::isOpened() const {
    return fd_ >= 0;
}

bool I2CDevice::writeByte(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    return write(fd_, buffer, 2) == 2;
}

bool I2CDevice::writeBytes(uint8_t reg, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> buffer;
    buffer.push_back(reg);
    buffer.insert(buffer.end(), data.begin(), data.end());
    return write(fd_, buffer.data(), buffer.size()) == static_cast<ssize_t>(buffer.size());
}

int16_t I2CDevice::readByte(uint8_t reg) {
    if (write(fd_, &reg, 1) != 1) return -1;
    uint8_t data;
    if (read(fd_, &data, 1) != 1) return -1;
    return data;
}

int16_t I2CDevice::readWord(uint8_t reg) {
    if (write(fd_, &reg, 1) != 1) return -1;
    uint8_t data[2];
    if (read(fd_, data, 2) != 2) return -1;
    return (int16_t)((data[0] << 8) | data[1]);
}

std::vector<uint8_t> I2CDevice::readBytes(uint8_t reg, uint8_t length) {
    if (write(fd_, &reg, 1) != 1) return {};
    std::vector<uint8_t> data(length);
    if (read(fd_, data.data(), length) != length) return {};
    return data;
}