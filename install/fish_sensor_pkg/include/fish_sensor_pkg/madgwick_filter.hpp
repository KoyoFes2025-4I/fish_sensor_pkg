#ifndef MADGWICK_FILTER_HPP
#define MADGWICK_FILTER_HPP

#include <array>
#include "sensor_msgs/msg/imu.hpp"

class MadgwickFilter {
public:
    MadgwickFilter();
    void update(double gx, double gy, double gz, double ax, double ay, double az, double dt);
    std::array<double, 4> getQuaternion() const;
    geometry_msgs::msg::Vector3 getLinearAcceleration(double ax, double ay, double az) const;

private:
    // パラメータ
    double beta_ = 0.1; // 2 * Kp
    // 四元数
    double q0_ = 1.0, q1_ = 0.0, q2_ = 0.0, q3_ = 0.0;
};

#endif // MADGWICK_FILTER_HPP