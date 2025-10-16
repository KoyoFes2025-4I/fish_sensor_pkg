#include "fish_sensor_pkg/as5600.hpp"
#include <stdexcept>

// レジスタ定義
namespace AS5600_REG {
    constexpr uint8_t STATUS = 0x0B;
    constexpr uint8_t RAW_ANGLE_H = 0x0C;
}

AS5600::AS5600(const std::string& bus, uint8_t address) {
    i2c_dev_ = std::make_unique<I2CDevice>(bus, address);
}

bool AS5600::init() {
    if (!i2c_dev_->openDevice()) {
        return false;
    }
    return testConnection();
}

bool AS5600::testConnection() {
    if (!i2c_dev_->isOpened()) return false;
    // STATUSレジスタを読み、磁石が検出できているか確認
    int16_t status = i2c_dev_->readByte(AS5600_REG::STATUS);
    if (status < 0) return false;
    // MDビット(bit5)が1なら磁石検出
    return (status & 0b00100000);
}

double AS5600::getAngleDeg() {
    if (!i2c_dev_->isOpened()) {
        throw std::runtime_error("AS5600 I2C device not open.");
    }
    int16_t raw_angle = i2c_dev_->readWord(AS5600_REG::RAW_ANGLE_H);
    if (raw_angle < 0) {
        throw std::runtime_error("Failed to read AS5600 angle.");
    }
    // 12-bit分解能 (0-4095) を角度 (0-360) に変換
    return static_cast<double>(raw_angle) * 360.0 / 4096.0;
}