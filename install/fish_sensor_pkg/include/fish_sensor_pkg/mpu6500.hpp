#ifndef MPU6500_HPP
#define MPU6500_HPP

#include "fish_sensor_pkg/i2c_device.hpp"
#include "rclcpp/rclcpp.hpp" // rclcppを追加
#include <memory>
#include <string>

struct ImuData {
    double ax, ay, az; // m/s^2
    double gx, gy, gz; // rad/s
};

class MPU6500 {
public:
    // コンストラクタにLoggerを追加
    MPU6500(const std::string& bus, uint8_t address, rclcpp::Logger logger);
    bool init();
    bool testConnection();
    ImuData readSensorData();

private:
    std::unique_ptr<I2CDevice> i2c_dev_;
    rclcpp::Logger logger_; // loggerメンバー変数を追加
    // スケールファクタ
    const double ACCEL_FS_SEL_2G = 16384.0;
    const double GYRO_FS_SEL_2000DPS = 16.4;
    const double G = 9.80665;
};

#endif // MPU6500_HPP