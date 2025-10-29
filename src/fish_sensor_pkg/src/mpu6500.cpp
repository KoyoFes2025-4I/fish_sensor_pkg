#include "fish_sensor_pkg/mpu6500.hpp"
#include <stdexcept>
#include <cmath>

// レジスタ定義
namespace MPU6500_REG {
    constexpr uint8_t PWR_MGMT_1   = 0x6B;
    constexpr uint8_t WHO_AM_I     = 0x75;
    constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    constexpr uint8_t GYRO_XOUT_H  = 0x43;
    constexpr uint8_t ACCEL_CONFIG = 0x1C;
    constexpr uint8_t GYRO_CONFIG  = 0x1B;
}

MPU6500::MPU6500(const std::string& bus, uint8_t address) {
    i2c_dev_ = std::make_unique<I2CDevice>(bus, address);
}

bool MPU6500::init() {
    if (!i2c_dev_->openDevice()) {
        return false;
    }

    if (!testConnection()) {
        return false;
    }

    // スリープモード解除
    if (!i2c_dev_->writeByte(MPU6500_REG::PWR_MGMT_1, 0x00)) return false;
    // 加速度計設定: ±2g
    if (!i2c_dev_->writeByte(MPU6500_REG::ACCEL_CONFIG, 0x00)) return false;
    // ジャイロ設定: ±2000dps
    if (!i2c_dev_->writeByte(MPU6500_REG::GYRO_CONFIG, 0x18)) return false;

    return true;
}

bool MPU6500::testConnection() {
    if (!i2c_dev_->isOpened()) return false;
    int16_t who_am_i = i2c_dev_->readByte(MPU6500_REG::WHO_AM_I);
    // MPU6500のIDは0x70
    return who_am_i == 0x70;
}

ImuData MPU6500::readSensorData() {
    if (!i2c_dev_->isOpened()) {
        throw std::runtime_error("MPU6500 I2C device not open.");
    }

    std::vector<uint8_t> data = i2c_dev_->readBytes(MPU6500_REG::ACCEL_XOUT_H, 14);
    if (data.size() != 14) {
        throw std::runtime_error("Failed to read MPU6500 data.");
    }
    
    int16_t ax_raw = (data[0] << 8) | data[1];
    int16_t ay_raw = (data[2] << 8) | data[3];
    int16_t az_raw = (data[4] << 8) | data[5];
    
    int16_t gx_raw = (data[8] << 8) | data[9];
    int16_t gy_raw = (data[10] << 8) | data[11];
    int16_t gz_raw = (data[12] << 8) | data[13];

    ImuData imu_data;
    imu_data.ax = (ax_raw / ACCEL_FS_SEL_2G) * G;
    imu_data.ay = (ay_raw / ACCEL_FS_SEL_2G) * G;
    imu_data.az = (az_raw / ACCEL_FS_SEL_2G) * G;

    imu_data.gx = (gx_raw / GYRO_FS_SEL_2000DPS) * (M_PI / 180.0);
    imu_data.gy = (gy_raw / GYRO_FS_SEL_2000DPS) * (M_PI / 180.0);
    imu_data.gz = (gz_raw / GYRO_FS_SEL_2000DPS) * (M_PI / 180.0);
    
    return imu_data;
}